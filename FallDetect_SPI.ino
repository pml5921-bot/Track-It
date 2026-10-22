#include <Adafruit_BNO08x.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>
#include <LittleFS.h>

// ================= USER CONFIGURATION =================
#define BUTTON_PIN        12
#define BUTTON_ACTIVE_LOW 1     // 1 = button wired from pin to GND (internal pull-up)
#define DEBOUNCE_MS       40
#define LED_PIN           15    // Feather S3 onboard LED; lit while recording

// Live ESP-NOW streaming to the receiver (optional, independent of logging)
#define ENABLE_ESPNOW 0
uint8_t receiverMAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

// Leave at 0 for standalone use: printing to USB with no computer attached can stall the loop
#define ENABLE_LOCAL_SERIAL 0

// Debug: print a #DIAG line whenever an IMU call blocks loop() or the sensor resets.
// Set to 0 for standalone use (same USB-stall reason as above).
#define ENABLE_DIAG_SERIAL 1
#define DIAG_SLOW_MS       50

// ================= FLASH LOGGING =================
#define LOG_FILE "/log.bin"

// One stored sample (binary, 23 bytes). Converted to CSV text only when dumped.
typedef struct __attribute__((packed)) {
  uint16_t session;
  uint32_t t_ms;           // ms since this recording started
  float activeDrop_cm;
  float peakDrop_cm;
  float worldAccZ;
  float rawAcc_g;
  uint8_t state;
} LogRecord;

#define LOG_BUF_RECORDS 1500   // RAM buffer (~34 KB, about 30 s of data)
#define FLUSH_RECORDS   200    // write to flash once this many are buffered (only while IDLE)

LogRecord logBuf[LOG_BUF_RECORDS];
uint16_t logCount = 0;
bool recording = false;
uint16_t sessionId = 0;
unsigned long sessionStartMs = 0;
bool fsReady = false;

// Button debounce state
bool lastRawPressed = false;
bool stablePressed = false;
unsigned long lastChangeMs = 0;

// ================= ESP-NOW =================
typedef struct __attribute__((packed)) {
  uint32_t seq;
  float activeDrop_cm;
  float peakDrop_cm;
  float worldAccZ;
  float rawAcc_g;
  uint8_t state;
} TelemetryPacket;

TelemetryPacket txPacket;
uint32_t txSeq = 0;
bool espNowReady = false;

// ================= HARDWARE PIN CONFIGURATION =================
// Adafruit ESP32-S3 Feather <-> Adafruit BNO085 breakout, SPI mode (Adafruit's wiring):
//   SCL -> SCK      SDA -> MISO      DI -> MOSI      (Feather's hardware SPI pins)
//   P0  -> 3V       P1  -> 3V        (both high = SPI mode)
#define BNO08X_CS     10    // D10          -> BNO085 CS
#define BNO08X_INT    5     // D5           -> BNO085 INT
#define BNO08X_RESET  16    // A2 (GPIO 16) -> BNO085 RST
#define SAMPLE_PERIOD 10    // 10ms = 100Hz

Adafruit_BNO08x imu(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// ================= IMU WATCHDOG =================
// Each report runs at 100Hz. If any one of them goes quiet for this long, the
// IMU has stopped reporting: reset it and re-enable the reports.
#define IMU_STALE_MS          250

unsigned long lastAccelMs = 0;
unsigned long lastLinAccelMs = 0;
unsigned long lastQuatMs = 0;
uint16_t imuRecoveries = 0;

// Snapshot of the most recent lockup, taken just before the watchdog resets things
uint8_t  lastStaleMask = 0;      // bit0 = accel, bit1 = lin accel, bit2 = rotation vector
int      lastIntLevel = -1;      // 0 = sensor had data waiting that was never read
bool     lastWasRecording = false;
uint32_t lastRecoveryUptimeMs = 0;
uint32_t emptyReads = 0;         // getSensorEvent() calls that returned no sensor data

// ================= THRESHOLDS & TUNING =================
const float G_CONST = 9.80665f;

const float FREE_FALL_THRESHOLD = 0.5f * G_CONST; // ~7.35 m/s^2
const unsigned long MIN_FREE_FALL_TIME_MS = 30;    // 30ms minimum free-fall duration
const float IMPACT_THRESHOLD = 1.8f * G_CONST;     // ~17.6 m/s^2
const unsigned long MAX_FALL_TO_IMPACT_MS = 800;   // Timeout window
const float MIN_DROP_HEIGHT_M = 0.15f;             // 15cm minimum drop threshold
const unsigned long REST_OBSERVE_TIME_MS = 1500;  // 1.5s stillness confirmation
const float REST_MOTION_THRESHOLD = 1.8f;          // Lin-accel threshold to detect recovery

// Dead Reckoning Filters
const float ACCEL_NOISE_DEADBAND = 0.12f;          // m/s^2 ignore minor sensor floor noise
const float VELOCITY_DAMPING = 0.985f;             // Leaky damping to arrest runaway drift

// ================= STATE MACHINE & KINEMATICS =================
enum FallState {
  STATE_IDLE,
  STATE_FREE_FALL,
  STATE_IMPACT_DETECTED,
  STATE_WAIT_FOR_STILLNESS
};

FallState currentState = STATE_IDLE;

unsigned long stateStartTime = 0;
unsigned long freeFallStartTime = 0;
unsigned long lastKinematicsTime = 0;
unsigned long lastPlotTime = 0;

// Dead Reckoning & Peak Tracking Variables
float worldAccZ = 0.0f;
float vertVelocity = 0.0f;
float activeDropDistance_cm = 0.0f;
float peakDropDistance_cm = 0.0f; // Holds the peak drop height over time
float peakImpactForce = 0.0f;

// Sensor measurements
float rawAccMag = G_CONST;
float linAccX = 0, linAccY = 0, linAccZ = 0;
float linAccMag = 0;
float quatI = 0.0f, quatJ = 0.0f, quatK = 0.0f, quatReal = 1.0f;

void updateDeadReckoning(float dt);
void processFallStateMachine();
void outputPlotData();
void sendTelemetry();
void logSample();

// ================= LED HELPERS =================
void blinkError() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

// ================= FLASH LOG FUNCTIONS =================
// If power was lost mid-write, the file may end with a partial record. Trim it
// so later appends stay aligned to record boundaries.
void repairLog() {
  File f = LittleFS.open(LOG_FILE, "r");
  if (!f) return;
  size_t sz = f.size();
  size_t good = (sz / sizeof(LogRecord)) * sizeof(LogRecord);
  if (sz == good) { f.close(); return; }

  File t = LittleFS.open("/tmp.bin", "w");
  if (!t) { f.close(); return; }
  uint8_t buf[230];  // 10 records
  size_t remaining = good;
  while (remaining > 0) {
    size_t n = remaining < sizeof(buf) ? remaining : sizeof(buf);
    if (f.read(buf, n) != n) break;
    t.write(buf, n);
    remaining -= n;
  }
  f.close();
  t.close();
  LittleFS.remove(LOG_FILE);
  LittleFS.rename("/tmp.bin", LOG_FILE);
}

uint16_t nextSessionId() {
  File f = LittleFS.open(LOG_FILE, "r");
  if (!f) return 1;
  size_t n = f.size() / sizeof(LogRecord);
  if (n == 0) { f.close(); return 1; }
  f.seek((n - 1) * sizeof(LogRecord));
  LogRecord r;
  f.read((uint8_t *)&r, sizeof(r));
  f.close();
  return r.session + 1;
}

bool flushLog() {
  if (logCount == 0) return true;
  size_t bytes = (size_t)logCount * sizeof(LogRecord);

  // Keep a safety margin of free space
  if ((LittleFS.totalBytes() - LittleFS.usedBytes()) < bytes + 8192) return false;

  File f = LittleFS.open(LOG_FILE, FILE_APPEND);
  if (!f) return false;
  size_t w = f.write((uint8_t *)logBuf, bytes);
  f.close();
  if (w != bytes) return false;

  logCount = 0;
  return true;
}

void startRecording() {
  if (!fsReady) { blinkError(); return; }
  sessionId = nextSessionId();
  sessionStartMs = millis();
  logCount = 0;
  recording = true;
  digitalWrite(LED_PIN, HIGH);
}

void stopRecording(bool errorStop) {
  recording = false;
  bool ok = flushLog();
  digitalWrite(LED_PIN, LOW);
  if (errorStop || !ok) blinkError();   // flash full or write failed
}

// Called at 50Hz while recording
void logSample() {
  if (logCount < LOG_BUF_RECORDS) {
    LogRecord &r = logBuf[logCount++];
    r.session       = sessionId;
    r.t_ms          = millis() - sessionStartMs;
    r.activeDrop_cm = activeDropDistance_cm;
    r.peakDrop_cm   = peakDropDistance_cm;
    r.worldAccZ     = worldAccZ;
    r.rawAcc_g      = rawAccMag / G_CONST;
    r.state         = (uint8_t)currentState;
  }

  // Flash writes can stall the loop for tens of ms, so only flush while IDLE
  // (not mid-fall). The RAM buffer is large enough to wait. Force a flush only
  // if the buffer is nearly full.
  bool idle = (currentState == STATE_IDLE);
  if ((logCount >= FLUSH_RECORDS && idle) || logCount >= LOG_BUF_RECORDS - 20) {
    if (!flushLog()) stopRecording(true);
  }
}

// ================= SERIAL COMMANDS (used by the PC script) =================
// 'b' = start/stop recording (same as the button), 'd' = dump log as CSV rows,
// 'e' = erase log, 's' = status
void dumpLog() {
  File f = LittleFS.open(LOG_FILE, "r");
  size_t n = f ? f.size() / sizeof(LogRecord) : 0;
  Serial.print("#DUMP_START,");
  Serial.println(n);

  LogRecord r;
  for (size_t i = 0; i < n; i++) {
    if (f.read((uint8_t *)&r, sizeof(r)) != sizeof(r)) break;
    Serial.print("D,");
    Serial.print(r.session);        Serial.print(",");
    Serial.print(r.t_ms);           Serial.print(",");
    Serial.print(r.activeDrop_cm, 3); Serial.print(",");
    Serial.print(r.peakDrop_cm, 3);   Serial.print(",");
    Serial.print(r.worldAccZ, 3);     Serial.print(",");
    Serial.print(r.rawAcc_g, 3);      Serial.print(",");
    Serial.println(r.state);
  }
  if (f) f.close();
  Serial.println("#DUMP_STOP");
}

void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;

    // 'b' = same as a button press: start recording, or stop if already recording
    if (c == 'b') {
      if (recording) {
        stopRecording(false);
        Serial.print("#REC_STOP,session=");
      } else {
        startRecording();
        Serial.print(recording ? "#REC_START,session=" : "#ERROR,could not start recording,session=");
      }
      Serial.println(sessionId);
      continue;
    }

    if (recording) {
      Serial.println("#ERROR,recording in progress");
      continue;
    }

    if (c == 'd') {
      dumpLog();
    } else if (c == 'e') {
      LittleFS.remove(LOG_FILE);
      Serial.println("#ERASED");
    } else if (c == 's') {
      File f = LittleFS.open(LOG_FILE, "r");
      size_t n = f ? f.size() / sizeof(LogRecord) : 0;
      if (f) f.close();
      Serial.print("#STATUS,records=");
      Serial.print(n);
      Serial.print(",free_bytes=");
      Serial.print(LittleFS.totalBytes() - LittleFS.usedBytes());
      // IMU health: how many times the watchdog had to reset the IMU since boot
      Serial.print(",imu_recoveries=");
      Serial.print(imuRecoveries);
      Serial.print(",int_pin=");
      Serial.print(digitalRead(BNO08X_INT));
      Serial.print(",empty_reads=");
      Serial.print(emptyReads);
      // Details of the most recent lockup (-1 = none yet)
      Serial.print(",last_stale_mask=");
      Serial.print(lastStaleMask);
      Serial.print(",last_int=");
      Serial.print(lastIntLevel);
      Serial.print(",last_while_recording=");
      Serial.print(lastWasRecording ? 1 : 0);
      Serial.print(",last_recovery_uptime_ms=");
      Serial.println(lastRecoveryUptimeMs);
    }
  }
}

// ================= BUTTON =================
// Returns true once per press
bool buttonPressedEvent() {
  bool raw = (digitalRead(BUTTON_PIN) == (BUTTON_ACTIVE_LOW ? LOW : HIGH));
  unsigned long now = millis();

  if (raw != lastRawPressed) {
    lastRawPressed = raw;
    lastChangeMs = now;
  }
  if ((now - lastChangeMs) >= DEBOUNCE_MS && raw != stablePressed) {
    stablePressed = raw;
    if (stablePressed) return true;
  }
  return false;
}

// ================= ESP-NOW SETUP =================
bool setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) return false;

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  return esp_now_add_peer(&peer) == ESP_OK;
}

// ================= HARDWARE RESET =================
// Explicit reset pulse so a watchdog recovery always starts from a clean sensor
void resetBNO085() {
  pinMode(BNO08X_RESET, OUTPUT);
  digitalWrite(BNO08X_RESET, LOW);
  delay(10);
  digitalWrite(BNO08X_RESET, HIGH);
  delay(10);
}

void enableImuReports() {
  // Adafruit's library takes the report interval in microseconds
  imu.enableReport(SH2_ACCELEROMETER,        SAMPLE_PERIOD * 1000UL);
  imu.enableReport(SH2_LINEAR_ACCELERATION,  SAMPLE_PERIOD * 1000UL);
  imu.enableReport(SH2_GAME_ROTATION_VECTOR, SAMPLE_PERIOD * 1000UL);
}

void markImuFresh() {
  unsigned long now = millis();
  lastAccelMs = lastLinAccelMs = lastQuatMs = now;
}

// Full sensor reset. Used at boot and by the watchdog.
bool initIMU() {
  resetBNO085();

  bool connected = imu.begin_SPI(BNO08X_CS, BNO08X_INT);
  if (connected) enableImuReports();

  // Restart the timeout window either way so a failed init is retried later
  // instead of on every loop pass
  markImuFresh();
  lastKinematicsTime = micros();
  return connected;
}

// Drop any in-progress fall: the sensor data across a reset gap is meaningless
void resetFallTracking() {
  currentState = STATE_IDLE;
  vertVelocity = 0.0f;
  activeDropDistance_cm = 0.0f;
  worldAccZ = 0.0f;
}

// Which reports have gone quiet (0 = all healthy)
uint8_t imuStaleMask() {
  unsigned long now = millis();
  uint8_t mask = 0;
  if (now - lastAccelMs    > IMU_STALE_MS) mask |= 0x01;
  if (now - lastLinAccelMs > IMU_STALE_MS) mask |= 0x02;
  if (now - lastQuatMs     > IMU_STALE_MS) mask |= 0x04;
  return mask;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUTTON_PIN, BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);

  fsReady = LittleFS.begin(true);   // true = format on first use
  if (fsReady) repairLog();
  else blinkError();

#if ENABLE_ESPNOW
  espNowReady = setupEspNow();
#endif

  pinMode(BNO08X_INT, INPUT_PULLUP);

  if (!initIMU()) {
    while (1) {
      Serial.println("#ERROR,BNO085 init failed");
      blinkError();   // visible even with no computer attached
      delay(1000);
    }
  }

  lastPlotTime = millis();
}

// ================= IMU READ =================
// Same pattern as the working bno085_spi_test sketch: one getSensorEvent() per
// pass. Over SPI the library waits for INT before reading, so with reports
// running at 100Hz this blocks at most a few ms. If the sensor dies it blocks
// for ~500 ms, which the watchdog below then catches.
void serviceIMU() {
  // false = no sensor data (e.g. a reply to an enable command, or a timeout)
  if (!imu.getSensorEvent(&sensorValue)) {
    emptyReads++;
    return;
  }

  unsigned long nowMs = millis();

  if (sensorValue.sensorId == SH2_ACCELEROMETER) {
    float ax = sensorValue.un.accelerometer.x;
    float ay = sensorValue.un.accelerometer.y;
    float az = sensorValue.un.accelerometer.z;
    rawAccMag = sqrt(ax * ax + ay * ay + az * az);
    lastAccelMs = nowMs;
  }
  else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
    linAccX = sensorValue.un.linearAcceleration.x;
    linAccY = sensorValue.un.linearAcceleration.y;
    linAccZ = sensorValue.un.linearAcceleration.z;
    linAccMag = sqrt(linAccX * linAccX + linAccY * linAccY + linAccZ * linAccZ);
    lastLinAccelMs = nowMs;
  }
  else if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    quatI    = sensorValue.un.gameRotationVector.i;
    quatJ    = sensorValue.un.gameRotationVector.j;
    quatK    = sensorValue.un.gameRotationVector.k;
    quatReal = sensorValue.un.gameRotationVector.real;
    lastQuatMs = nowMs;
  }

  // Run continuous kinematics & state supervisor
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastKinematicsTime) / 1000000.0f;
  lastKinematicsTime = nowMicros;

  if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;

  updateDeadReckoning(dt);
  processFallStateMachine();
}

// Reports any IMU call that blocked loop() for longer than DIAG_SLOW_MS.
// Over SPI the library waits up to 500 ms for INT before each transfer, so a
// slow call here means INT was not asserting when the library expected it.
void diagIfSlow(const char *what, unsigned long startMs) {
#if ENABLE_DIAG_SERIAL
  unsigned long took = millis() - startMs;
  if (took > DIAG_SLOW_MS) {
    Serial.print("#DIAG,slow_");
    Serial.print(what);
    Serial.print(",ms=");
    Serial.println(took);
  }
#endif
}

// ================= MAIN LOOP =================
void loop() {
  if (buttonPressedEvent()) {
    if (recording) stopRecording(false);
    else startRecording();
  }

  handleSerialCommands();

  // getSensorEvent() is what notices a sensor-side reset, so check after reading
  unsigned long t0 = millis();
  serviceIMU();
  diagIfSlow("read", t0);

  if (imu.wasReset()) {
#if ENABLE_DIAG_SERIAL
    Serial.println("#DIAG,sensor_reset");
#endif
    t0 = millis();
    enableImuReports();
    diagIfSlow("enable", t0);
    markImuFresh();
  }

  // Watchdog: a report has stopped arriving. Reset the sensor.
  // This blocks while the BNO085 reboots; the gap shows up as a jump in t_ms in the log.
  uint8_t staleMask = imuStaleMask();
  if (staleMask) {
    // Capture state before the reset changes it
    lastStaleMask = staleMask;
    lastIntLevel = digitalRead(BNO08X_INT);
    lastWasRecording = recording;
    lastRecoveryUptimeMs = millis();
    imuRecoveries++;
#if ENABLE_DIAG_SERIAL
    Serial.print("#DIAG,watchdog,mask=");
    Serial.print(staleMask);
    Serial.print(",int=");
    Serial.println(lastIntLevel);
#endif
    resetFallTracking();
    t0 = millis();
    initIMU();
    diagIfSlow("init", t0);
  }

  // 50Hz (every 20ms): local plot, ESP-NOW stream, flash log
  if (millis() - lastPlotTime >= 20) {
    lastPlotTime = millis();
#if ENABLE_LOCAL_SERIAL
    outputPlotData();
#endif
#if ENABLE_ESPNOW
    sendTelemetry();
#endif
    if (recording) logSample();
  }
}

// ================= ESP-NOW TRANSMIT =================
void sendTelemetry() {
  if (!espNowReady) return;

  txPacket.seq           = txSeq++;
  txPacket.activeDrop_cm = activeDropDistance_cm;
  txPacket.peakDrop_cm   = peakDropDistance_cm;
  txPacket.worldAccZ     = worldAccZ;
  txPacket.rawAcc_g      = rawAccMag / G_CONST;
  txPacket.state         = (uint8_t)currentState;

  esp_now_send(receiverMAC, (uint8_t *)&txPacket, sizeof(txPacket));
}

// ================= CONTINUOUS DEAD RECKONING =================
void updateDeadReckoning(float dt) {
  // 1. Rotate body-frame linear acceleration into World-Frame Z
  float r31 = 2.0f * (quatI * quatK - quatReal * quatJ);
  float r32 = 2.0f * (quatJ * quatK + quatReal * quatI);
  float r33 = 1.0f - 2.0f * (quatI * quatI + quatJ * quatJ);

  worldAccZ = (r31 * linAccX) + (r32 * linAccY) + (r33 * linAccZ);

  // 2. Physical Free-Fall Detection Override
  if (currentState == STATE_FREE_FALL && rawAccMag < FREE_FALL_THRESHOLD) {
    worldAccZ = -G_CONST; // Exact downward ballistic acceleration
  } else {
    // Noise floor deadband
    if (fabs(worldAccZ) < ACCEL_NOISE_DEADBAND) {
      worldAccZ = 0.0f;
    }
  }

  // 3. State-based Kinematic Dead Reckoning
  if (currentState == STATE_IDLE) {
    vertVelocity = 0.0f;
    activeDropDistance_cm = 0.0f;
  }
  else if (currentState == STATE_FREE_FALL) {
    // Integrate downward velocity and accumulated drop distance
    vertVelocity += worldAccZ * dt;
    vertVelocity *= VELOCITY_DAMPING;

    // Downward movement increases drop distance
    activeDropDistance_cm += (-vertVelocity * dt) * 100.0f;
    if (activeDropDistance_cm < 0.0f) activeDropDistance_cm = 0.0f;
  }
  else {
    // In IMPACT_DETECTED and WAIT_FOR_STILLNESS, freeze/dampen velocity to hold drop height
    vertVelocity *= 0.50f;
  }
}

// ================= STATE MACHINE =================
void processFallStateMachine() {
  unsigned long now = millis();

  switch (currentState) {
    case STATE_IDLE:
      if (rawAccMag < FREE_FALL_THRESHOLD) {
        currentState = STATE_FREE_FALL;
        freeFallStartTime = now;
        vertVelocity = 0.0f;
        activeDropDistance_cm = 0.0f;
        peakImpactForce = 0.0f;
      }
      break;

    case STATE_FREE_FALL: {
      unsigned long ffDuration = now - freeFallStartTime;

      if (rawAccMag > IMPACT_THRESHOLD) {
        if (ffDuration >= MIN_FREE_FALL_TIME_MS) {
          peakImpactForce = rawAccMag;
          currentState = STATE_IMPACT_DETECTED;
          stateStartTime = now;
        } else {
          currentState = STATE_IDLE;
        }
      }
      else if (ffDuration > MAX_FALL_TO_IMPACT_MS) {
        currentState = STATE_IDLE;
      }
      break;
    }

    case STATE_IMPACT_DETECTED:
      if (rawAccMag > peakImpactForce) {
        peakImpactForce = rawAccMag;
      }

      if (now - stateStartTime > 100) {
        currentState = STATE_WAIT_FOR_STILLNESS;
        stateStartTime = now;
      }
      break;

    case STATE_WAIT_FOR_STILLNESS:
      if ((now - stateStartTime) > 300) {
        if (linAccMag > REST_MOTION_THRESHOLD) {
          // If user moved or recovered, lock current fall distance as peak
          peakDropDistance_cm = activeDropDistance_cm;
          currentState = STATE_IDLE;
          break;
        }
      }

      if (now - stateStartTime >= REST_OBSERVE_TIME_MS) {
        // Fall cycle successfully completed: update peak height tracker
        peakDropDistance_cm = activeDropDistance_cm;
        currentState = STATE_IDLE;
      }
      break;
  }
}

// ================= SERIAL PLOTTER OUTPUT (LOCAL) =================
void outputPlotData() {
  Serial.print("ActiveDrop_cm:");
  Serial.print(activeDropDistance_cm, 2);
  Serial.print(",");
  Serial.print("PeakDrop_cm:");
  Serial.print(peakDropDistance_cm, 2); // Constant horizontal line updating after every fall
  Serial.print(",");
  Serial.print("WorldAccZ:");
  Serial.print(worldAccZ, 2);
  Serial.print(",");
  Serial.print("RawAcc_g:");
  Serial.print(rawAccMag / G_CONST, 2);
  Serial.print(",");
  Serial.print("State:");
  Serial.println((int)currentState * 10);
}

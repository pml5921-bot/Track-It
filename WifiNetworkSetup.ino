#include <WiFi.h>
void setup() {
  Serial.begin(115200);
  delay(1000); // Give the serial monitor a moment to initialize
  WiFi.mode(WIFI_AP);
  WiFi.softAP("TrackIt");
}
void loop() {
  delay(1000);
}

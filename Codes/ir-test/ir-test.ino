// ----- Configuration -----
const uint8_t sensorPins[5] = {A0, A1, A2, A3, A4};  // Analog pins

void setup() {
  Serial.begin(115200);
  Serial.println("Analog IR Sensor Test Starting...");
}

void loop() {
  Serial.print("Readings: ");
  for (uint8_t i = 0; i < 5; i++) {
    int val = analogRead(sensorPins[i]);
    Serial.print(val);
    if (i < 4) Serial.print(", ");
  }
  Serial.println();
  delay(200);
}

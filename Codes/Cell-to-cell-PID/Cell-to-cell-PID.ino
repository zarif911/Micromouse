// pins for your 4 pots
const uint8_t potPins[4] = { A0, A1, A2, A3 };
// array to hold the smoothed readings
int potValues[4];

// how many readings to average per channel
const uint8_t NUM_SAMPLES = 8;

void setup() {
  Serial.begin(115200);
}

void loop() {
  for (uint8_t i = 0; i < 4; i++) {
    // throw away first reading after channel switch
    analogRead(potPins[i]);
    // now accumulate NUM_SAMPLES readings
    long sum = 0;
    for (uint8_t s = 0; s < NUM_SAMPLES; s++) {
      sum += analogRead(potPins[i]);
      delay(2);  // small settling delay; tweak as needed
    }
    // store averaged value
    potValues[i] = sum / NUM_SAMPLES;
  }

  // print them in one line
  Serial.print("Pots: ");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(potValues[i]);
    if (i < 3) Serial.print(" | ");
  }
  Serial.println();

  // adjust or remove this if you want faster/slower loop rate
  delay(50);
}

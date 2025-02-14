#include <Arduino.h>
#include <Encoder.h>


#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3

Encoder encoder(ENCODER_A_PIN, ENCODER_B_PIN);

const int PULSES_PER_ROTATION = 4000;
const int FULL_ROTATION_DEGREES = 360;
const float PULSES_PER_DEGREE = PULSES_PER_ROTATION / FULL_ROTATION_DEGREES;

const int MAX_SAMPLES = 4000;
int index = 0;


int SAMPLE_RATE = 1000; // samples per second
float timePeriod = 1 / SAMPLE_RATE;

long int start;

float getPendulumnAngle() {
    long encoderPos = encoder.read();
    float angleDegrees = encoderPos / PULSES_PER_DEGREE;
    return angleDegrees;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setup.");

  while(abs(getPendulumnAngle()) < 90) {
    Serial.println(getPendulumnAngle());
  }

  Serial.println("Start.");
  start = millis();
}

void loop() {
  if (millis() - start > timePeriod) {
    Serial.println(getPendulumnAngle());
    index++;
    start = millis();
  }

  if (index > MAX_SAMPLES - 1) {
    Serial.println("Done.");
    delay(100000);
  }
}
#include <Arduino.h>
#include <Encoder.h>
#include <TMCStepper.h>         // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <AccelStepper.h>
#include <SoftwareSerial.h>     // Software serial for the UART to TMC2209 - https://www.arduino.cc/en/Reference/softwareSerial


#define EN_PIN           8      // Enable - PURPLE
#define DIR_PIN          7      // Direction - WHITE
#define STEP_PIN         6      // Step - ORANGE
#define SW_SCK           5      // Software Slave Clock (SCK) - BLUE
#define SW_TX            9      // SoftwareSerial transmit pin - BROWN
#define SW_RX            4      // SoftwareSerial receive pin - YELLOW
#define DRIVER_ADDRESS   0b00   // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f           // SilentStepStick series use 0.11 ...and so does my fysetc TMC2209 (?)

#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3

Encoder encoder(ENCODER_A_PIN, ENCODER_B_PIN);


SoftwareSerial SoftSerial(SW_RX, SW_TX, false);                          // Be sure to connect RX to TX and TX to RX between both devices

TMC2209Stepper TMCdriver(&SoftSerial, R_SENSE, DRIVER_ADDRESS);   // Create TMC driver
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

const int TRIGGER_ANGLE_DEGREES = 45;
const int PULSES_PER_ROTATION = 4000;
const int FULL_ROTATION_DEGREES = 360;
const float PULSES_PER_DEGREE = PULSES_PER_ROTATION / FULL_ROTATION_DEGREES;

const int MAX_SAMPLES = 5000;
int index = 0;

const int MICROSTEPS = 8;


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

  Serial.begin(115200);               // initialize hardware serial for debugging
  SoftSerial.begin(115200);           // initialize software serial for UART motor control
  SoftSerial.listen();
  //TMCdriver.beginSerial(115200);      // Initialize UART
  
  pinMode(EN_PIN, OUTPUT);           // Set pinmodes
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  TMCdriver.begin();                                                                                                                                                                                                                                                                                                                            // UART: Init SW UART (if selected) with default 115200 baudrate
  TMCdriver.toff(5);                 // Enables driver in software
  TMCdriver.rms_current(1000);        // Set motor RMS current
  TMCdriver.microsteps(MICROSTEPS);         // Set microsteps
  TMCdriver.en_spreadCycle(false);
  TMCdriver.pwm_autoscale(true);     // Needed for stealthChop
  
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true); 
  stepper.enableOutputs();

  stepper.setCurrentPosition(0);  // Zero position
  stepper.setMaxSpeed(5000000); // Steps per second
  stepper.setMinPulseWidth(5);

  while(abs(getPendulumnAngle()) < TRIGGER_ANGLE_DEGREES) {
    stepper.setCurrentPosition(0);
    stepper.runToPosition();
    Serial.println(getPendulumnAngle());
  }

  Serial.println("Start.");
  start = millis();
}

void loop() {
  stepper.setCurrentPosition(0);
  stepper.runToPosition();
  
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
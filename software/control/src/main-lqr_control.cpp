#include <Arduino.h>
#include <TMCStepper.h>         // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <AccelStepper.h>
#include <SoftwareSerial.h>     // Software serial for the UART to TMC2209 - https://www.arduino.cc/en/Reference/softwareSerial
#include <Stream.h>          // For serial debugging output - https://www.arduino.cc/reference/en/libraries/streaming/
#include <Encoder.h>


//#define DEBUG

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


typedef struct {
  double kThetaPendulum;
  double kThetaPendulumDot;
  double kThetaArm;
  double kThetaArmDot;
} lqrGains;

lqrGains gains {
  .kThetaPendulum = 6.3958e+03,
  .kThetaPendulumDot = 550.8507,
  .kThetaArm = 1000,
  .kThetaArmDot = 430.6257
};

float angleOffset = 180;

float controlInput = 0;
float inputSpeed = 0;

float prevPendulumAngleDegrees = angleOffset;
float pendulumSpeed;
float armAngleDegrees;
float armSpeed;
float prevArmAngleDegrees = 0;

float prevTime = 0;

const double SECONDS_PER_MICROSECOND = 1e-6;

const long MAX_SPEED = 20000; // degrees per second
const int MICROSTEPS = 8;
const float DEGREES_PER_STEPS = 1.8;

const int PULSES_PER_ROTATION = 4000;
const int FULL_ROTATION_DEGREES = 360;
const float PULSES_PER_DEGREE = PULSES_PER_ROTATION / FULL_ROTATION_DEGREES;
const float MICROSTEPS_PER_DEGREE = MICROSTEPS / DEGREES_PER_STEPS;

float prevInputSpeed = 0;

float startAngle = 5;

float integralTerm = 0;

float integralGain = 0.4;

float getPendulumnAngle() {
    long encoderPos = encoder.read();
    float angleDegrees = encoderPos / PULSES_PER_DEGREE + angleOffset;
    return angleDegrees;
}

float adjustForDirection(float angle, float controlInput) {
  if (angle < 0) {
    // Apply correction for counterclockwise direction (angle < 0)
    controlInput *= 1.6; // Example correction factor for counterclockwise
  } else {
    // Apply correction for clockwise direction (angle >= 0)
    controlInput *= 1.0; // Example correction factor for clockwise
  }
  return controlInput;
}

void setup() {

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


  prevTime = micros() * SECONDS_PER_MICROSECOND;


  while (prevPendulumAngleDegrees > startAngle) {
    prevTime = micros() * SECONDS_PER_MICROSECOND;
    prevPendulumAngleDegrees = getPendulumnAngle();
    prevArmAngleDegrees = stepper.currentPosition() / MICROSTEPS_PER_DEGREE;
    #ifdef DEBUG
      Serial.print("prevPendulumAngleDegrees: ");
      Serial.println(prevPendulumAngleDegrees);
    #endif
  }
}


void loop() {
  

  if (abs(stepper.currentPosition()) / MICROSTEPS_PER_DEGREE > 90) {
    stepper.stop();
    delay(10000);
  }
  

  double currTime = micros() * SECONDS_PER_MICROSECOND;
  double deltaT = currTime - prevTime;
  float pendulumAngleDegrees = getPendulumnAngle();
  pendulumSpeed = (pendulumAngleDegrees - prevPendulumAngleDegrees) / (deltaT);
  armAngleDegrees = stepper.currentPosition() / MICROSTEPS_PER_DEGREE;
  float armAngleDegreesError  = armAngleDegrees - 0;
  integralTerm += armAngleDegreesError;
  armSpeed = (armAngleDegrees - prevArmAngleDegrees) / (deltaT);
  controlInput = gains.kThetaPendulum * pendulumAngleDegrees 
                  + gains.kThetaPendulumDot * pendulumSpeed 
                  + gains.kThetaArm * armAngleDegrees 
                  + gains.kThetaArmDot * armSpeed
                  + integralGain * integralTerm;

  float inputAccel = controlInput;
  inputSpeed = prevInputSpeed + inputAccel * deltaT/18;

  #ifdef DEBUG
    Serial.print("controlInput: ");
    Serial.print(controlInput);
    Serial.print(", inputSpeed: ");
    Serial.print(inputSpeed);
    Serial.print(", inputAccel: ");
    Serial.print(inputAccel);
    Serial.print(", deltaT: ");
    Serial.print(deltaT, 8);
    Serial.print(", pendulumSpeed: ");
    Serial.print(pendulumSpeed);
    Serial.print(", armSpeed: ");
    Serial.print(armSpeed);
    Serial.print(", armAngleDegrees: ");
    Serial.println(armAngleDegrees);
  #endif

  
  stepper.setSpeed(inputSpeed * MICROSTEPS_PER_DEGREE);
  stepper.runSpeed();

  prevTime = currTime;
  prevPendulumAngleDegrees = pendulumAngleDegrees;
  prevArmAngleDegrees = armAngleDegrees;
  prevInputSpeed = inputSpeed;
}
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
  double kThetaArm;
  double kThetaArmDot;
  double kThetaPendulum;
  double kThetaPendulumDot;
} lqrGains;

lqrGains gains {
  .kThetaArm = -1000.00000002,
  .kThetaArmDot = -604.65043048,
  .kThetaPendulum = -10689.08311825,
  .kThetaPendulumDot = -958.04047177
};

float trackingGain = -1000.00000003;
float referenceArmAngleDegrees = 10;

#define TOTAL_REF_ANGLES 10
float refAngles[TOTAL_REF_ANGLES] = {0, -90, 0, 90, 0, 45, 100, 0, -80, 30};

int index = 0;
const float ANGLE_LIMIT = 360;
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

float balanceTriggerAngle = 20;
float balanceTriggerSpeed = 180;
float swingUpTriggerAngle = 45;
float K = 90;

float integralTerm = 0;

float integralGain = 0.4;

double prevRefAngleTime = 0;
const float REF_ANGLE_CHANGE_TIME_PERIOD = 3;

int sign (float val) {
  if (val > 0) {
    return 1;
  } else if (val < 0) {
    return -1;
  } else {
    return 0;
  }
}

float deg2rad(float deg) {
  return deg / 180 * PI;
}

float rad2deg(float rad) {
  return rad / PI * 180;
}

float getPendulumnAngle() {
    long encoderPos = encoder.read();
    float angleDegrees = encoderPos / PULSES_PER_DEGREE + angleOffset;
    return angleDegrees;
}


float swingUp (float pendulumAngleDegrees, float pendulumSpeed) {
  float cosPendulum = cos(pendulumAngleDegrees / 180 * PI);
  float energy = 0.5 * 0.0005989206600000001 * (pendulumSpeed / 180 * PI) * (pendulumSpeed / 180 * PI) + 0.095715 * 9.81 * 0.137/2 * (-1 + cosPendulum) * 1.93;
  float controlInput =  K * (energy - 0.1241357555) * sign(pendulumSpeed / 180 * PI * cosPendulum);

  return rad2deg(controlInput);
}

float balance(float pendulumAngleDegrees, float pendulumSpeed, float armAngleDegrees, float armSpeed) {

  double currRefAngleTime = micros() * SECONDS_PER_MICROSECOND;
  if (currRefAngleTime - prevRefAngleTime > REF_ANGLE_CHANGE_TIME_PERIOD) {
    referenceArmAngleDegrees = refAngles[index];
    index++;
    if (index > TOTAL_REF_ANGLES - 1) {
      index = 0;
    }
    prevRefAngleTime = currRefAngleTime;
  }

  controlInput = - gains.kThetaPendulum * pendulumAngleDegrees 
                  - gains.kThetaPendulumDot * pendulumSpeed 
                  - gains.kThetaArm * armAngleDegrees 
                  - gains.kThetaArmDot * armSpeed
                  //+ integralGain * integralTerm;
                  + trackingGain * referenceArmAngleDegrees;

  return rad2deg(controlInput) * PI / 7200;
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
}

bool balancing = false;
void loop() {
  double currTime = micros() * SECONDS_PER_MICROSECOND;
  double deltaT = currTime - prevTime;
  float pendulumAngleDegrees = getPendulumnAngle();
  pendulumSpeed = (pendulumAngleDegrees - prevPendulumAngleDegrees) / (deltaT);
  armAngleDegrees = stepper.currentPosition() / MICROSTEPS_PER_DEGREE;
  armSpeed = (armAngleDegrees - prevArmAngleDegrees) / (deltaT);

  if (balancing && abs(pendulumAngleDegrees) > swingUpTriggerAngle) {
    balancing = false;
    inputSpeed = 0;
    Serial.println("Swing up mode");
  } else if (!balancing && abs(pendulumAngleDegrees) < balanceTriggerAngle && abs(armSpeed) < balanceTriggerSpeed) {
    balancing = true;
    referenceArmAngleDegrees = armAngleDegrees;

    for (int i = 0; i < TOTAL_REF_ANGLES; i++) {
      refAngles[i] = armAngleDegrees + refAngles[i];
    }
    
    index = 0;
    prevRefAngleTime = micros() * SECONDS_PER_MICROSECOND;

    Serial.println("Balance mode");
  }

  if (abs(armAngleDegrees) > ANGLE_LIMIT) {
    stepper.stop();
    while(true) {
      continue;
    }
  }

  if (balancing) {
    controlInput = balance(pendulumAngleDegrees, pendulumSpeed, armAngleDegrees, armSpeed);
  } else {
    controlInput = swingUp(pendulumAngleDegrees, pendulumSpeed);
  }

  float inputAccel = controlInput;
  inputSpeed += inputAccel * deltaT;
  stepper.setSpeed(inputSpeed * MICROSTEPS_PER_DEGREE);
  stepper.runSpeed();


  prevTime = currTime;
  prevPendulumAngleDegrees = pendulumAngleDegrees;
  prevArmAngleDegrees = armAngleDegrees;
}
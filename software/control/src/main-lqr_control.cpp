#include <Arduino.h>
#include <TMCStepper.h>         // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <AccelStepper.h>
#include <SoftwareSerial.h>     // Software serial for the UART to TMC2209 - https://www.arduino.cc/en/Reference/softwareSerial
#include <Stream.h>          // For serial debugging output - https://www.arduino.cc/reference/en/libraries/streaming/
#include <Encoder.h>
#include "state_variable.hpp"
#include "control.hpp"
#include "reference.hpp"
#include "actuator.hpp"
#include "accel_stepper_adapter.hpp"
#include "encoder_adapter.hpp"
#include "actuator.hpp"
#include "sensor.hpp"
#include "state.hpp"
#include "transition.hpp"
#include "state_machine.hpp"
#include "rotary_inverted_pendulum_system.hpp"
#include "input_variable.hpp"

constexpr int EN_PIN = 8;           // Enable - PURPLE
constexpr int DIR_PIN = 7;          // Direction - WHITE
constexpr int STEP_PIN = 6;         // Step - ORANGE
constexpr int SW_SCK = 5;           // Software Slave Clock (SCK) - BLUE
constexpr int SW_TX = 9;            // SoftwareSerial transmit pin - BROWN
constexpr int SW_RX = 4;            // SoftwareSerial receive pin - YELLOW
constexpr uint8_t DRIVER_ADDRESS = 0b00; // TMC2209 Driver address according to MS1 and MS2
constexpr float R_SENSE = 0.11f;    // SilentStepStick series use 0.11 ...and so does my fysetc TMC2209 (?)
SoftwareSerial SoftSerial(SW_RX, SW_TX, false);                          // Be sure to connect RX to TX and TX to RX between both devices
TMC2209Stepper TMCdriver(&SoftSerial, R_SENSE, DRIVER_ADDRESS);   // Create TMC driver
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);


constexpr int ENCODER_A_PIN = 2;
constexpr int ENCODER_B_PIN = 3;
Encoder encoder(ENCODER_A_PIN, ENCODER_B_PIN);

constexpr int TRAJECTORY_LENGTH = 10;
constexpr int ANGLE_HOLD_TIME = 3;
double trajectory[TRAJECTORY_LENGTH] = {0, -90, 0, 90, 0, 45, 100, 0, -80, 30};
Reference reference(trajectory, TRAJECTORY_LENGTH, ANGLE_HOLD_TIME);

constexpr int MICROSTEPS = 8;
constexpr float DEGREES_PER_STEPS = 1.8;
constexpr int PULSES_PER_ROTATION = 4000;
constexpr int FULL_ROTATION_DEGREES = 360;
constexpr float PULSES_PER_DEGREE = PULSES_PER_ROTATION / FULL_ROTATION_DEGREES;
constexpr float MICROSTEPS_PER_DEGREE = MICROSTEPS / DEGREES_PER_STEPS;
float ANGLE_OFFSET = 180;
EncoderAdapter encoderAdapter(&encoder, ANGLE_OFFSET, PULSES_PER_DEGREE);
AccelStepperAdapter stepperAdapter(&stepper, MICROSTEPS_PER_DEGREE);

Sensor pendulumAngleSensor(&encoderAdapter);
Sensor armAngleSensor(&stepperAdapter);

double proportionalGain = 80; 
LyapunovControlMethod lyapunovControlMethod(proportionalGain);

constexpr double thetaArmGain = -1000.00000002;
constexpr double thetaArmDotGain = -604.65043048;
constexpr double thetaPendulumGain = -10689.08311825;
constexpr double thetaPendulumDotGain = -958.04047177;
constexpr double trackingGain = -1000.00000003;
LQRControlMethod lqrControlMethod(thetaArmGain, thetaArmDotGain, thetaPendulumGain, thetaPendulumDotGain, trackingGain);

StateVariable pendulumAngle(0, 0, 0);
StateVariable armAngle(0, 0, 0);
StateVariable pendulumAngleRateOfChange(0, 0, 0);
StateVariable armAngleRateOfChange(0, 0, 0);

constexpr int TOTAL_INPUTS = 10;
constexpr int HOLD_TIME = 3;
float referenceAngles[TOTAL_INPUTS] = {0, -90, 0, 90, 0, 45, 100, 0, -80, 30};
TimeVaryingInput referenceAngle(referenceAngles, TOTAL_INPUTS, 0, HOLD_TIME);

RotaryInvertedPendulumSystem rotaryInvertedPendulumSystem(&stepperAdapter,
                                                            &pendulumAngleSensor, &armAngleSensor,
                                                            &lyapunovControlMethod, &lqrControlMethod,
                                                            &pendulumAngle, &armAngle,
                                                            &pendulumAngleRateOfChange, &armAngleRateOfChange,
                                                            &referenceAngle);

State swingUpState(nullptr, nullptr, &RotaryInvertedPendulumSystem::runSwingUpControl);
State balanceState(nullptr, nullptr, &rotaryInvertedPendulumSystem::runBalanceControl);

constexpr float SWING_UP_TRIGGER_ANGLE = 45;
constexpr float BALANCE_TRIGGER_ANGLE = 20;
constexpr float ARM_ANGLE_LIMIT = 720;
StateMachine stateMachine(SWING_UP_TRIGGER_ANGLE, BALANCE_TRIGGER_ANGLE, ARM_ANGLE_LIMIT);

void setup() {

  Serial.begin(115200);               // initialize hardware serial for debugging
  SoftSerial.begin(115200);           // initialize software serial for UART motor control
  SoftSerial.listen();

  // TMCdriver.beginSerial(115200);      // Initialize UART
  // TMCdriver.begin();                                                                                                                                                                                                                                                                                                                            // UART: Init SW UART (if selected) with default 115200 baudrate
  // TMCdriver.toff(5);                 // Enables driver in software
  // TMCdriver.rms_current(1000);        // Set motor RMS current
  // TMCdriver.microsteps(MICROSTEPS);         // Set microsteps
  // TMCdriver.en_spreadCycle(false);
  // TMCdriver.pwm_autoscale(true);     // Needed for stealthChop
  
  pinMode(EN_PIN, OUTPUT);           // Set pinmodes
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true); 
  stepper.enableOutputs();

  stepper.setCurrentPosition(0);  // Zero position
  stepper.setMaxSpeed(5000000); // Steps per second
  stepper.setMinPulseWidth(5);

}

void loop() {
  systemStates.update(sensors.getArmAngle(), sensors.getPendulumAngle());
  reference.update();

  float pendulumAngle = systemStates.getPendulumAngle();
  float pendulumAngularVelocity = systemStates.getPendulumAngularVelocity();
  float armAngle = systemStates.getArmAngle();
  float armAngularVelocity = systemStates.getArmAngularVelocity();

  float referenceArmAngle = reference.getCurrentAngle();
  float controlInput = controller.getOutput(armAngle, armAngularVelocity, pendulumAngle, pendulumAngularVelocity, referenceArmAngle);

  stateMachine.update(pendulumAngle, pendulumAngularVelocity, armAngle, armAngularVelocity);

  StateMachineEvent lastTriggeredEvent = stateMachine.getLastTriggeredEvent();

  if (lastTriggeredEvent == TRIGGER_SWING_UP) {
    controller.setControlMode(SWING_UP);
  } else if (lastTriggeredEvent == TRIGGER_BALANCE) {
    reference.reset(armAngle);
    controller.setControlMode(BALANCE);
  } else if (lastTriggeredEvent == ANGLE_LIMIT_REACHED) {
    stepperActuator.stop();
    while(true) {
      continue;
    }
  }

  stepperActuator.actuate(controlInput);

}
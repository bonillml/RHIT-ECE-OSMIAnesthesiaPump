#include <Arduino.h>
#include <SPI.h>
#include <stdlib.h>
#include <AccelStepper.h>

/* RHIT-ECE-OSMIAnesthesiaPump_Sumer2025
*
*     Edited By: Matthew Bonilla
*     Last Edited: June 2025
*
*     This software is built upon the work done by
*     - Mitchell Boucher
*     - Tyler Koopman
*     - Jimmy (Jintian) Zhang
*
*/

/*  Define motor connections
*   
*   THESE USE THE UPDATED ESP32-S3 SCHEMATIC
*   Edited: June 5th, 2025
*/
#define MOTOR1_STEP   1
#define MOTOR1_DIR    2
#define MOTOR2_STEP   0
#define MOTOR2_DIR    45
#define MOTOR3_STEP   7 
#define MOTOR3_DIR    6
#define MOTOR4_STEP   18
#define MOTOR4_DIR    17

// Create AccelStepper instances
AccelStepper stepper1(AccelStepper::DRIVER, MOTOR1_STEP, MOTOR1_DIR);
AccelStepper stepper2(AccelStepper::DRIVER, MOTOR2_STEP, MOTOR2_DIR);
AccelStepper stepper3(AccelStepper::DRIVER, MOTOR3_STEP, MOTOR3_DIR);
AccelStepper stepper4(AccelStepper::DRIVER, MOTOR4_STEP, MOTOR4_DIR);

/*  Define rotary encoder input connections
*
*   THESE USE THE UPDATED ESP32-S3 SCHEMATIC
*   Edited: June 5th, 2025
*/
#define ENCODER_SW_PIN  37
#define ENCODER_DT_PIN  36
#define ENCODER_CLK_PIN 35

// Define useful fields, constants, and global variables.
#define STEPS_PER_ML    1480
#define MULTI_STEPPING  8    

bool newMotorStart = false;     // A flag used to determine if a new motor is to be started.
bool motorRunning = false;      // A flag used to determine if any motor is running
bool settingParameter = false;  //

int newMotorSpeed = 0;          // 
int newMotorNumber = 0;       // The channel number used when the rotary encoder is pressed.
int newMotorAmount = 0;       // The amount the stepper motor will output, in steps.

int UI_desiredSpeed = 0;
int UI_desiredAmount = 0;
int UI_desiredMotorNum = 0;     

int previousMillis = 0;
int interval = 500;

// UI Scroll Settings
int incUISpeed = 1000;
int incUIAmount = 5;
int incMotorNumber = 1;
int maxUISpeed = 7000;
int maxUIAmount = 100;
int maxMotorNumber = 4;


// Function Definitions

/**
 * Retrieves the address of a stepper motor given a number
 * from 1 to 4. Returns a NULL pointer otherwise.
 */
AccelStepper *getStepper(int);

/**
 * Retrieves the number of motor steps needed to inject the
 * desired milli-liter amount.
 */
int calculateML(float);

/**
 *  Retrieves the number of steps-per-second to produce the 
 * desired mL-per-hour amount.
 */
int calculateMotorSpeed(int);

/**
 * Initiates a stepper motor with a given speed and number of steps.
 */
void startNewMotor(int, int, int);

/**
 * Stops all stepper motors.
 */
void turnOffMotors();

// Function Implementations

AccelStepper *getStepper(int motorNum) {
  switch (motorNum) {
    case 1: return &stepper1;
    case 2: return &stepper2;
    case 3: return &stepper3;
    case 4: return &stepper4;
    default: return nullptr;
  };
}

int calculateML(float mLAmount) {
  return (mLAmount * MULTI_STEPPING * STEPS_PER_ML);
}

int calculateMotorSpeed(int mLPerHour) {
  /* Seconds-Per-Hour = 3600; the conversion is needed because
  *  because the method returns steps-per-second.
  */
  return ((mLPerHour * MULTI_STEPPING * STEPS_PER_ML) / 3600);
}


void startNewMotor(int motorNum, int motorSpeed, int numSteps) {
    
  // Retrieve the address of the selected motor.
  AccelStepper *curMotor = getStepper(motorNum);

  curMotor->setCurrentPosition(0);    
  curMotor->setMaxSpeed(motorSpeed);
  curMotor->setAcceleration(motorSpeed / 2);
  curMotor->moveTo(numSteps);

  motorRunning = true;
}

void turnOffMotors() {
  
  for (int i = 1; i <= 4; i++) {
    startNewMotor(i, 0, 0);
  }
  motorRunning = false;
}

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}
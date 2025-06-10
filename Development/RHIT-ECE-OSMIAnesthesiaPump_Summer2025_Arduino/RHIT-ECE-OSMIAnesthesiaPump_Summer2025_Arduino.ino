#include <A4988.h>
#include <BasicStepperDriver.h>
#include <DRV8825.h>
#include <DRV8834.h>
#include <DRV8880.h>
#include <MultiDriver.h>
#include <SyncDriver.h>

/*      Global Variables      */
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 4

// Define Stepper Motor Pins
#define MOTOR1_STEP   1
#define MOTOR1_DIR    2
#define MOTOR2_STEP   0
#define MOTOR2_DIR    45
#define MOTOR3_STEP   7 
#define MOTOR3_DIR    6
#define MOTOR4_STEP   18
#define MOTOR4_DIR    17

// Define Stepper Motor Objects
BasicStepperDriver stepper1(MOTOR_STEPS, MOTOR1_DIR, MOTOR1_STEP);
BasicStepperDriver stepper2(MOTOR_STEPS, MOTOR2_DIR, MOTOR2_STEP);
BasicStepperDriver stepper3(MOTOR_STEPS, MOTOR3_DIR, MOTOR3_STEP);
BasicStepperDriver stepper4(MOTOR_STEPS, MOTOR4_DIR, MOTOR4_STEP);
int activeDrivers[4] = {0, 0, 0, 0};

// Define LCD Pins
#define GFX_CS 11
#define GFX_DC 9
#define GFX_RST 10
#define GFX_BL 8
#define GFX_SCK 3
#define GFX_MOSI 46


void setup() {
  // Initializes all four Stepper Motors.
  stepper1.begin(RPM, MICROSTEPS);
  stepper2.begin(RPM, MICROSTEPS);
  stepper3.begin(RPM, MICROSTEPS);
  stepper4.begin(RPM, MICROSTEPS);

}

void loop() {
  
  // Test movement of two stepper motors
  stepper1.rotate(360 * 10);
  stepper2.rotate(-360 * 10);

  stepper1.move(-MOTOR_STEPS * MICROSTEPS * 10);
  stepper2.move(MOTOR_STEPS * MICROSTEPS * 10);

  delay(5000);

}

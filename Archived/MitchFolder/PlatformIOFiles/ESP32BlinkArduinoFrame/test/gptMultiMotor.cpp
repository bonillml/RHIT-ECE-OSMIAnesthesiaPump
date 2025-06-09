#include <AccelStepper.h>

// Define motor connections and create AccelStepper instances
#define MOTOR1_DIR 4
#define MOTOR1_STEP 5
// #define MOTOR2_DIR 4
// #define MOTOR2_STEP 5
// #define MOTOR3_DIR 6
// #define MOTOR3_STEP 7
// #define MOTOR4_DIR 8
// #define MOTOR4_STEP 9

AccelStepper stepper1(AccelStepper::DRIVER, MOTOR1_STEP, MOTOR1_DIR);
// AccelStepper stepper2(AccelStepper::DRIVER, MOTOR2_STEP, MOTOR2_DIR);
// AccelStepper stepper3(AccelStepper::DRIVER, MOTOR3_STEP, MOTOR3_DIR);
// AccelStepper stepper4(AccelStepper::DRIVER, MOTOR4_STEP, MOTOR4_DIR);

void setup() {
    // Set max speed and acceleration for each motor
    stepper1.setMaxSpeed(1000);
    stepper1.setAcceleration(500);

    // stepper2.setMaxSpeed(1200);
    // stepper2.setAcceleration(600);

    // stepper3.setMaxSpeed(800);
    // stepper3.setAcceleration(400);

    // stepper4.setMaxSpeed(1500);
    // stepper4.setAcceleration(700);

    // Set initial movement target
    stepper1.moveTo(10000);    //non-blocking
    // stepper2.moveTo(-1500);
    // stepper3.moveTo(1000);
    // stepper4.moveTo(-1800);
}

void loop() {
    // Continuously run all motors
    if (stepper1.distanceToGo() == 0) stepper1.moveTo(-stepper1.currentPosition());
    // if (stepper2.distanceToGo() == 0) stepper2.moveTo(-stepper2.currentPosition());
    // if (stepper3.distanceToGo() == 0) stepper3.moveTo(-stepper3.currentPosition());
    // if (stepper4.distanceToGo() == 0) stepper4.moveTo(-stepper4.currentPosition());

    // if(stepper1.distanceToGo() == 0) stepper1.setCurrentPosition(0);
    stepper1.run(); //non-blocking commands
    // stepper2.run();
    // stepper3.run();
    // stepper4.run();
}

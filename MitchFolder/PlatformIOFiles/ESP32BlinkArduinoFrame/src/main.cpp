#include <AccelStepper.h>

// Define motor connections
#define MOTOR1_DIR 39      //S3 numbers: 4
#define MOTOR1_STEP 34     // 5
#define MOTOR2_DIR 35      // 6
#define MOTOR2_STEP 32     // 7  
#define MOTOR3_DIR 1      // 1
#define MOTOR3_STEP 3     // 2
#define MOTOR4_DIR 21     // 42
#define MOTOR4_STEP 19    // 41

// Define Input connections
#define ENCODER_BUTTON 10

//Define LEDscreen connections

//Define occlusion detection connections


//Define constant values used
#define stepsPerML 1480
#define multiStepping 8   //standard is base 1/4, divide standard by this value to get used stepping (EX: 8 would represent a stepping of 1/32)


//create AccelStepper instances
AccelStepper stepper1(AccelStepper::DRIVER, MOTOR1_STEP, MOTOR1_DIR);
AccelStepper stepper2(AccelStepper::DRIVER, MOTOR2_STEP, MOTOR2_DIR);
AccelStepper stepper3(AccelStepper::DRIVER, MOTOR3_STEP, MOTOR3_DIR);
AccelStepper stepper4(AccelStepper::DRIVER, MOTOR4_STEP, MOTOR4_DIR);

//Global Variables
bool newMotorStart = false; //Flag to determine if a new motor is to be started (or old one changed)

int newMotorSpeed = 0;      //alternatively use this variable since only need to adjust max speed depending on motor
int newStepperNumber = 0;   //The number of the channel that will be used when the rotary button is pressed
int newStepperAmount = 0;   //the amount that the stepper will output (in steps, I can create a simple calculation to go from mL to this)

int UI_desiredSpeed = 0;
int UI_desiredAmount = 0;
int UI_desiredMotorNum = 0;   //kind of redundant, but for clarity of what needs to be specified by the UI

//Interupts
/*
  When the rotary encoder button is depressed the newMotorStart flag will raise
  The newMotorStart flag is used in the loop to set the speed and start rotations for the specified motor instance
*/
void handleButtonPress()
{
  newMotorStart = true;
}

void motorInit(){
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR1_STEP,OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR2_STEP,OUTPUT);
  pinMode(MOTOR3_DIR, OUTPUT);
  pinMode(MOTOR3_STEP,OUTPUT);
  pinMode(MOTOR4_DIR, OUTPUT);
  pinMode(MOTOR4_STEP,OUTPUT);
  // stepper.setMaxSpeed(4800.0);
  // stepper.setAcceleration(100.0);
  // stepper.setCurrentPosition(0);
  // Serial.println(stepper.currentPosition());
}

//Helper function - These funtions were written with the assistance of chatGPT
AccelStepper* getStepper(int stepperNumber) {
    switch (stepperNumber) {
        case 1: return &stepper1;
        case 2: return &stepper2;
        case 3: return &stepper3;
        case 4: return &stepper4;
        default: return nullptr; // Return null if invalid input
    }
}

/*
  returns the number of steps to give the desired mL amount
*/
int calculate_mL(int mLamount)
{
  return (mLamount * multiStepping * stepsPerML);
}

/*
  returns the number of steps/second to give the desired mL/hour amount
*/
int calculateMotorSpeed(int mLPerHour)
{
  int secondsPerHour = 3600;    //for code readability
  return (mLPerHour / secondsPerHour * stepsPerML * multiStepping);
}

//An idea for a method to set the stepper number if you don't want to edit it inside the code (you can make more via copy/paste if you like the idea)
void setNewStepperNumber(int settingNumber)
{
  newStepperNumber = settingNumber;
}


/*
  Inputs
  stepperNumber - The number of the channel that will be used for the current dosage <-- What you're inputting
  stepperSpeed - the speed in steps/second that will be used for 
*/
void startNewMotor(int stepperNumber, int stepperSpeed, int numberSteps)
{
  AccelStepper* currentStepper = getStepper(stepperNumber);
  currentStepper->setMaxSpeed(stepperSpeed);
  currentStepper->setAcceleration(stepperSpeed/2);
  currentStepper->moveTo(numberSteps);
}


void setup() {
    // Set max speed and acceleration for each motor
    stepper1.setMaxSpeed(1000*8);
    stepper1.setAcceleration(500*8);

    stepper2.setMaxSpeed(1200*8);
    stepper2.setAcceleration(600*8);

    stepper3.setMaxSpeed(800*8);
    stepper3.setAcceleration(400*8);

    stepper4.setMaxSpeed(1500*8);
    stepper4.setAcceleration(700*8);

    // Set initial movement target
    stepper1.moveTo(29600/2*8);    //non-blocking
    stepper2.moveTo(3000*8);
    stepper3.moveTo(1000*8);
    stepper4.moveTo(-1800*8);
}


void loop() {
    pinMode(ENCODER_BUTTON, INPUT_PULLUP);    //enabling the rotary encoder button
    attachInterrupt(digitalPinToInterrupt(ENCODER_BUTTON), handleButtonPress, FALLING); //attach interupt trigger to the button pressing down
    // Continuously run all motors
    // if (stepper1.distanceToGo() == 0) stepper1.moveTo(-stepper1.currentPosition());
    // if (stepper2.distanceToGo() == 0) stepper2.moveTo(-stepper2.currentPosition());
    if (stepper3.distanceToGo() == 0) stepper3.moveTo(-stepper3.currentPosition());
    if (stepper4.distanceToGo() == 0) stepper4.moveTo(-stepper4.currentPosition());

    if(stepper1.distanceToGo() == 0) {
      stepper1.setCurrentPosition(0);
      stepper1.moveTo(-20000*8);
    } 
    if(stepper2.distanceToGo() == 0) {
      stepper2.setCurrentPosition(0);
      stepper2.moveTo(-3000);
    } 
    stepper1.run(); //non-blocking commands
    stepper2.run();
    stepper3.run();
    stepper4.run();


    // Trigger flag checks
    if(newMotorStart)
    {
      newStepperNumber = UI_desiredMotorNum;
      newMotorSpeed = calculateMotorSpeed(UI_desiredSpeed);
      newStepperAmount = calculate_mL(UI_desiredAmount);
      startNewMotor(newStepperNumber, newMotorSpeed, newStepperAmount);
    }


}

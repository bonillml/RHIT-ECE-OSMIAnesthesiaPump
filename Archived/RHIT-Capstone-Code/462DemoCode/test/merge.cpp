#include <SPI.h>
#include <Arduino.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include <stdlib.h>
#include <AccelStepper.h>

#define dirPin 32
#define stepPin 33

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

// Define LEDscreen connections

// Define occlusion detection connections

// Define constant values used
#define stepsPerML 1480
#define multiStepping 8 // standard is base 1/4, divide standard by this value to get used stepping (EX: 8 would represent a stepping of 1/32)

// create AccelStepper instances
AccelStepper stepper1(AccelStepper::DRIVER, MOTOR1_STEP, MOTOR1_DIR);
AccelStepper stepper2(AccelStepper::DRIVER, MOTOR2_STEP, MOTOR2_DIR);
AccelStepper stepper3(AccelStepper::DRIVER, MOTOR3_STEP, MOTOR3_DIR);
AccelStepper stepper4(AccelStepper::DRIVER, MOTOR4_STEP, MOTOR4_DIR);

// Global Variables

//flags
bool newMotorStart = false; // Flag to determine if a new motor is to be started (or old one changed)
bool motorRunning = false;  // Flag to define if any motor is running (mostly created to stop all motors)
bool settingparam = false;

//motor
int newMotorSpeed = 0;    // alternatively use this variable since only need to adjust max speed depending on motor
int newStepperNumber = 0; // The number of the channel that will be used when the rotary button is pressed
int newStepperAmount = 0; // the amount that the stepper will output (in steps, I can create a simple calculation to go from mL to this)

//UI
int UI_desiredSpeed = 0;
int UI_desiredAmount = 0;
int UI_desiredMotorNum = 0; // kind of redundant, but for clarity of what needs to be specified by the UI


//UI declarations
TFT_eSPI tft = TFT_eSPI();

#define TFT_GREY 0x5AEB // Custom grey color

// Pin definitions for active-low buttons
#define ENCODER_SW_PIN 27  // this is sw on encoder
#define ENCODER_DT_PIN 26  // this is dt on encoder
#define ENCODER_CLK_PIN 25 // this is clk on encoder

enum
{
  deivcenum,
  devicespeed,
  devicevolume,
  confirm
};
int tab = deivcenum;
const unsigned long buttondebounceDelay = 50;
const unsigned long turndebounceDelay = 50;
unsigned long pressCount = 0;
unsigned long lastButtonPress = 0;
unsigned long lastEncoderTurn = 0;

int currentOption = 0; // Index of the highlighted option
bool stateChanged = false;
int position = 0;
int posVal = 0;

/*
  Return the stepper motor reference of the given input number
  The purpose is to cut down on if-else statements in accordance to each stepper
  Inputs:
  stepperNumber - the number of the stepper that should be returned
 */
AccelStepper *getStepper(int stepperNumber)
{
  switch (stepperNumber)
  {
  case 1:
    return &stepper1;
  case 2:
    return &stepper2;
  case 3:
    return &stepper3;
  case 4:
    return &stepper4;
  default:
    return nullptr; // Return null if invalid input
  }
}

//Unnecessary?
// void motorInit(){
//   pinMode(MOTOR1_DIR, OUTPUT);
//   pinMode(MOTOR1_STEP,OUTPUT);
//   pinMode(MOTOR2_DIR, OUTPUT);
//   pinMode(MOTOR2_STEP,OUTPUT);
//   pinMode(MOTOR3_DIR, OUTPUT);
//   pinMode(MOTOR3_STEP,OUTPUT);
//   pinMode(MOTOR4_DIR, OUTPUT);
//   pinMode(MOTOR4_STEP,OUTPUT);
// }

/*
  returns the number of steps to give the desired mL amount
*/
int calculate_mL(float mLamount)
{
  return (mLamount * multiStepping * stepsPerML);
}

/*
  returns the number of steps/second to give the desired mL/hour amount
*/
int calculateMotorSpeed(int mLPerHour)
{
  int secondsPerHour = 3600; // for code readability
  return ((mLPerHour * stepsPerML * multiStepping) / secondsPerHour);
}


/*
  Inputs
  stepperNumber - The number of the channel that will be used for the current dosage <-- What you're inputting
  stepperSpeed - the speed in steps/second that will be used for
*/
void startNewMotor(int stepperNumber, int stepperSpeed, int numberSteps)
{
  AccelStepper *currentStepper = getStepper(stepperNumber);
  currentStepper->setMaxSpeed(stepperSpeed);
  currentStepper->setAcceleration(stepperSpeed);
  currentStepper->moveTo(numberSteps);
  motorRunning = true;
}

void turnOffMotors()
{
  for (int i = 1; i <= 4; i++)
  {
    startNewMotor(i, 0, 0);
  }
  motorRunning = false;
}

void displayMenu()
{
  tft.fillScreen(TFT_GREY);
  tft.setTextColor(TFT_WHITE, TFT_GREY);
  tft.setTextFont(2);
  tft.setTextSize(3);

  if (tab == deivcenum)
  {
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
  }
  else
  {
    tft.setTextColor(TFT_WHITE, TFT_GREY);
  }
  tft.println("Motor Num #");
  tft.println(UI_desiredMotorNum);

  if (tab == devicespeed)
  {
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
  }
  else
  {
    tft.setTextColor(TFT_WHITE, TFT_GREY);
  }
  tft.println("Speed");
  tft.println(UI_desiredSpeed);

  if (tab == devicevolume)
  {
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
  }
  else
  {
    tft.setTextColor(TFT_WHITE, TFT_GREY);
  }
  tft.println("Volume");
  tft.print(UI_desiredAmount);
  tft.println("ml");
  if (tab == confirm)
  {
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
  }
  else
  {
    tft.setTextColor(TFT_WHITE, TFT_GREY);
  }
  tft.println("Confirm");
  tft.setCursor(0,0);
}

void buttonPressed(void)
{
  if ((millis() - lastButtonPress > buttondebounceDelay))
  {
    lastButtonPress = millis();
    if (pressCount % 2 == 0)
    {
      if (tab == confirm)
      {
        newMotorStart = true;
      }
      else
      {
        settingparam = !settingparam;
      }

      stateChanged = true;
    }
    pressCount++;
  }
}

void turn(void)
{
  if (millis() - lastEncoderTurn > turndebounceDelay)
  {
    if (digitalRead(ENCODER_CLK_PIN) != digitalRead(ENCODER_DT_PIN))
    {
      if (settingparam)
      {
        switch (tab)
        {
        case deivcenum:
          UI_desiredMotorNum = (UI_desiredMotorNum + 1) % 5;
          break;

        case devicespeed:
          UI_desiredSpeed = (UI_desiredSpeed + 1000) % 8000;
          break;

        case devicevolume:
          UI_desiredAmount = (UI_desiredAmount + 5) % 105;
          break;
        default:
          break;
        }
      }
      else
      {
        tab = (tab + 1) % 4;
      }
    }
    else
    {
      if (settingparam)
      {
        switch (tab)
        {
        case deivcenum:
          UI_desiredMotorNum = (UI_desiredMotorNum - 1);
          UI_desiredMotorNum <= 0 ? UI_desiredMotorNum = 1 : 0;
          break;

        case devicespeed:
          UI_desiredSpeed = (UI_desiredSpeed - 1000);
          UI_desiredSpeed < 0 ? UI_desiredSpeed = 0 : 0;
          break;

        case devicevolume:
          UI_desiredAmount = (UI_desiredAmount - 5);
          UI_desiredAmount < 0 ? UI_desiredAmount = 0 : 0;
          break;
        default:
          break;
        }
      }
      else
      {
        tab = (tab - 1) ;
        tab < 0 ? tab = 0 : 0;
      }
    }
    lastEncoderTurn = millis();
    stateChanged = true;
  }
}

void setup()
{
  // motorInit();
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("setup");
  tft.init();
  Serial.println("tftinit");

  // tft.setRotation(1);
  digitalWrite(4, LOW);   //TODO: What does the 4 represent?
  digitalWrite(4, HIGH);
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DT_PIN, INPUT);
  pinMode(ENCODER_CLK_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_SW_PIN), buttonPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), turn, CHANGE);

  displayMenu();
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (stateChanged)
  {
    displayMenu();
    stateChanged = false;
  }

  if (motorRunning)
  {
    stepper1.run(); // non-blocking commands
    stepper2.run();
    stepper3.run();
    stepper4.run();
  }

  // Trigger flag checks
  if (newMotorStart)
  {
    newStepperNumber = UI_desiredMotorNum;
    newMotorSpeed = calculateMotorSpeed(UI_desiredSpeed);
    newStepperAmount = calculate_mL(UI_desiredAmount);
    startNewMotor(newStepperNumber, newMotorSpeed, newStepperAmount);
    // Then set desired values back to 0
    UI_desiredMotorNum = 0;
    UI_desiredSpeed = 0;
    UI_desiredAmount = 0;
    newMotorStart = false;
  }
}

#include <SPI.h>
#include <Arduino.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include <stdlib.h>
#include <AccelStepper.h>

//TODO: set start position of motor + add to position instead of setting to 0 -> can be set to the sie of the pump 
// ^ This can be a future task <-- instead maybe set current position instead of creating a new one

// Define motor connections
#define MOTOR1_DIR  14     //S3 numbers: 4
#define MOTOR1_STEP 12     // 5
#define MOTOR2_DIR  32     // 6
#define MOTOR2_STEP 33     // 7  
#define MOTOR3_DIR  21     // 1     //UI
#define MOTOR3_STEP 3      // 2
#define MOTOR4_DIR  0      // 42
#define MOTOR4_STEP 15     // 41     //UI

// Define Input connections
#define ENCODER_SW_PIN 27  // this is sw on encoder
#define ENCODER_DT_PIN 26  // this is dt on encoder
#define ENCODER_CLK_PIN 25 // this is clk on encoder


//UI declarations
TFT_eSPI tft = TFT_eSPI();

#define TFT_GREY 0x5AEB // Custom grey color

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

//Define occlusion detection connections
// #define IR_LED_PIN  5      // Adjust to your IR LED pin
// #define IR_PHOTODIODE_PIN  34  // Adjust to your ADC pin
// #define ONBOARD_LED_PIN 2  // Usually GPIO 2, check your board


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
bool motorRunning = false;  //Flag to define if any motor is running (mostly created to stop all motors)
bool settingparam = false;

int newMotorSpeed = 0;      //alternatively use this variable since only need to adjust max speed depending on motor
int newStepperNumber = 0;   //The number of the channel that will be used when the rotary button is pressed
int newStepperAmount = 0;   //the amount that the stepper will output (in steps, I can create a simple calculation to go from mL to this)

int UI_desiredSpeed = 0;
int UI_desiredAmount = 0;
int UI_desiredMotorNum = 0;   //kind of redundant, but for clarity of what needs to be specified by the UI

int previousMillis = 0;
int interval = 500;

//UI scroll settings
int incUISpeed = 1000;
int incUIAmount = 5;
int incMotorNumber = 1;
int maxUISpeed = 7000; //+ incUISpeed;
int maxUIAmount = 100; //+ incUIAmount;
int maxMotorNumber = 4; //+ incMotorNumber;

//Interupts
/*
  When the rotary encoder button is depressed the newMotorStart flag will raise
  The newMotorStart flag is used in the loop to set the speed and start rotations for the specified motor instance
*/
// void buttonPressed()
// {
//   newMotorStart = true;
// }


//Helper function - These funtions were written with the assistance of chatGPT

/*
*/
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
int calculate_mL(float mLamount)
{
  return (mLamount * multiStepping * stepsPerML);
}

/*
  returns the number of steps/second to give the desired mL/hour amount
*/
int calculateMotorSpeed(int mLPerHour)
{
  int secondsPerHour = 3600;    //for code readability
  return ((mLPerHour * stepsPerML * multiStepping) / secondsPerHour);
}

//An idea for a method to set the stepper number if you don't want to edit it inside the code (you can make more via copy/paste if you like the idea)
void setNewStepperNumber(int settingNumber)
{
  newStepperNumber = settingNumber;
}

//Stepper Motor Commands
/*
  Inputs
  stepperNumber - The number of the channel that will be used for the current dosage <-- What you're inputting
  stepperSpeed - the speed in steps/second that will be used for 
*/
void startNewMotor(int stepperNumber, int stepperSpeed, int numberSteps)
{
  AccelStepper* currentStepper = getStepper(stepperNumber);
  currentStepper->setCurrentPosition(0);
  currentStepper->setMaxSpeed(stepperSpeed);
  currentStepper->setAcceleration(stepperSpeed/2);
  currentStepper->moveTo(numberSteps);
  Serial.print("Stepper Number:");
  Serial.println(stepperNumber);
  Serial.print("Stepper Speed:");
  Serial.println(stepperSpeed);
  Serial.print("Stepper amount:");
  Serial.println(numberSteps);
  motorRunning = true;
}

void turnOffMotors()
{
  for(int i = 1; i <= 4; i++) 
  {
    startNewMotor(i, 0, 0);
  }
  motorRunning = false;
}


//UI commands
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
    if (pressCount % 2 == 0)  //buttons bounce weird (2 "presses" per press)
    {
      Serial.println("Button Pressed");
      if (tab == confirm)
      {
        newMotorStart = true;
      }
      else
      {
        settingparam = !settingparam;   //Flip between editing text and not
      }
      stateChanged = true;
    }
    pressCount++;
  }
}

/*
  Trigger method for rotary encoder turning
*/
void turn(void)
{
  // Rotary turning debounce
  if (millis() - lastEncoderTurn > turndebounceDelay)
  {
    // Rotary is turned up
    if (digitalRead(ENCODER_CLK_PIN) != digitalRead(ENCODER_DT_PIN))
    {
      // Serial.println("Rotary Up");
      // Adjusting number options
      if (settingparam)
      {
        switch (tab)
        {
        case deivcenum:
          UI_desiredMotorNum = (UI_desiredMotorNum + incMotorNumber);
          UI_desiredMotorNum > maxMotorNumber ? UI_desiredMotorNum = maxMotorNumber : 0;  //don't scroll past max

          break;

        case devicespeed:
          UI_desiredSpeed = (UI_desiredSpeed + incUISpeed);
          UI_desiredSpeed > maxUISpeed ? UI_desiredSpeed = maxUISpeed : 0;                //don't scroll past max

          break;

        case devicevolume:
          UI_desiredAmount = (UI_desiredAmount + incUIAmount);
          UI_desiredAmount > maxUIAmount ? UI_desiredAmount = maxUIAmount : 0;            //don't scroll past max
          break;
        default:
          break;
        }
      }
      //Scrolling through options
      else
      {
        tab = (tab + 1);
        tab > 3 ? tab = 3 : 0;  //Can't scroll past max
      }
    }
    //Rotary Encoder turned down
    else
    {
      // Serial.println("Rotary Down");
      // Adjusting number options
      if (settingparam) //FIXME: WTF is this?
      {
        switch (tab)
        {
        case deivcenum:
          UI_desiredMotorNum = (UI_desiredMotorNum - 1);
          UI_desiredMotorNum <= 0 ? UI_desiredMotorNum = 1 : 0;   //don't scroll past 0
          break;

        case devicespeed:
          UI_desiredSpeed = (UI_desiredSpeed - 1000);
          UI_desiredSpeed < 0 ? UI_desiredSpeed = 0 : 0;         //don't scroll past 0
          break;

        case devicevolume:
          UI_desiredAmount = (UI_desiredAmount - 5);
          UI_desiredAmount < 0 ? UI_desiredAmount = 0 : 0;        //don't scroll past 0
          break;
        default:
          break;
        }
      }
      //Scrolling through menu
      else
      {
        tab = (tab - 1);
        tab < 0 ? tab = 0 : 0;  //Don't scroll past 0
      }
    }
    lastEncoderTurn = millis();
    stateChanged = true;
  }
}



//Main Routine
void setup() {

  Serial.begin(115200);

  //Rotary Input setup
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DT_PIN, INPUT);
  pinMode(ENCODER_CLK_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_SW_PIN), buttonPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), turn, CHANGE);

  // //IR setup
  // pinMode(IR_LED_PIN, OUTPUT);
  // pinMode(IR_PHOTODIODE_PIN, INPUT);
  // pinMode(ONBOARD_LED_PIN, OUTPUT);
  
  Serial.println("Test Print");

  //UI setup
  tft.init();
  displayMenu();  
}


void loop() {
  //motor check
  if(motorRunning)
  {
    stepper1.run(); //non-blocking commands
    stepper2.run();
    stepper3.run();
    stepper4.run();
  }
  // Trigger flag checks
  if(newMotorStart)
  {
    Serial.println("button");
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

  //IR detection
  //TODO: make a comparator that every cycle will check if it detected an air gap, then output + stop motors
  if (millis() - previousMillis >= interval)
    {
      // int irReading = analogRead(IR_PHOTODIODE_PIN); // Read the photodiode voltage
      // Serial.print("IR Sensor Reading: ");
      // Serial.println(irReading);
      previousMillis = millis();
    }

  //UI check
  if (stateChanged)
  {
    displayMenu();
    stateChanged = false;
  }
}




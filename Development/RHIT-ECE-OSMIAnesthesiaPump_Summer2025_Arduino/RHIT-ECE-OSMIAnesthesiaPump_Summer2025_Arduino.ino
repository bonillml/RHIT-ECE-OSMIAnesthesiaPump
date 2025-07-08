#include <ESP32Encoder.h>
#include <MultiStepperLite.h>
#include <SPI.h>
#include <TFT_eSPI.h>

/*      Global Variables      */
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 4

// Board time since previous Rotary Encoder turn
unsigned long prevTimeRE = 0;
const long debounceRE = 50;

// Board time since previous Rotary Encoder switch press
unsigned long prevTimeSW = 0;
const long debounceSW= 50;

volatile int prevCountRE;
volatile int curCountRE;

// Define Stepper Motor Pins (0-based indexing)
#define Motor1_STEP   1
#define Motor1_DIR    2
#define Motor2_STEP   35
#define Motor2_DIR    45
#define Motor3_STEP   7 
#define Motor3_DIR    6
#define Motor4_STEP   18
#define Motor4_DIR    17

// Define Stepper Motor Objects
MultiStepperLite steppers(4);
int activeMotors[4] = {0, 0, 0, 0};

// Define Rotary Encoder Pins
#define RE_CLK    38          // Encoder Pin A
#define RE_DT     37          // Encoder Pin B
#define RE_SW     36          // No-Push Button Switch

// Define Rotary Encoder Object
ESP32Encoder re;

// Define Rotary Encoder variables and direction enumeration
#define RE_SCROLL_COUNT 2
enum TURN_DIR {
  CW,         // Clockwise
  CCW,        // Counter-clockwise
  STOP
};
TURN_DIR dirRE = TURN_DIR::STOP; 

// Define LCD Object
TFT_eSPI tft = TFT_eSPI();

// Define menu array and metadata
/*
*     -=-IMPORTANT-=-:
*       *KEEP THE FIRST FOUR ELEMENTS (0-3) AS THE NAMES OF THE CHANNELS
*       *KEEP THE LAST SIX ELEMENTS (4-9) AS THE NAMES OF THE CHANNELS
*           - The names can be altered, by their placement in the 
*             array must be the same. 
*           - The element locations are used to print the names of 
*             the menu items, thus allowing customization.
*/
String menu_l[10] = {"Channel 1", "Channel 2", "Channel 3", "Channel 4", "Dosage", "Infusion Rate", "Syringe Start", "Syringe End", "Calibrate", "Main Menu"};

enum ACTIVE_MENU_WINDOW { // Tracks which menu screen to display 
  MAIN,       // Displays the main menu options
  CHANNEL1,   // Displays pump channel 1 menu options
  CHANNEL2,   // Displays pump channel 2 menu options
  CHANNEL3,   // Displays pump channel 3 menu options
  CHANNEL4,   // Displays pump channel 4 menu options

  CHANNEL1_ITEM1,      // Channel 1 Item 1
  CHANNEL1_ITEM2,      // Channel 1 Item 2
  CHANNEL1_ITEM3,      // Channel 1 Item 3
  CHANNEL1_ITEM4,      // Channel 1 Item 4
  CHANNEL1_ITEM5,      // Channel 1 Item 5

  CHANNEL2_ITEM1,      // Channel 2 Item 1
  CHANNEL2_ITEM2,      // Channel 2 Item 2
  CHANNEL2_ITEM3,      // Channel 2 Item 3
  CHANNEL2_ITEM4,      // Channel 2 Item 4
  CHANNEL2_ITEM5,      // Channel 2 Item 5

  CHANNEL3_ITEM1,      // Channel 3 Item 1
  CHANNEL3_ITEM2,      // Channel 3 Item 2
  CHANNEL3_ITEM3,      // Channel 3 Item 3
  CHANNEL3_ITEM4,      // Channel 3 Item 4
  CHANNEL3_ITEM5,      // Channel 3 Item 5

  CHANNEL4_ITEM1,      // Channel 4 Item 1
  CHANNEL4_ITEM2,      // Channel 4 Item 2
  CHANNEL4_ITEM3,      // Channel 4 Item 3
  CHANNEL4_ITEM4,      // Channel 4 Item 4
  CHANNEL4_ITEM5       // Channel 4 Item 5
};
ACTIVE_MENU_WINDOW activeWindow = ACTIVE_MENU_WINDOW::MAIN;

enum ACTIVE_MENU_ITEM {
  Channel_1,  // Main Menu Channel 1
  Channel_2,  // Main Menu Channel 2
  Channel_3,  // Main Menu Channel 3
  Channel_4,  // Main Menu Channel 4

  C1_I1,      // Channel 1 Item 1
  C1_I2,      // Channel 1 Item 2
  C1_I3,      // Channel 1 Item 3
  C1_I4,      // Channel 1 Item 4
  C1_I5,      // Channel 1 Item 5
  C1_MM,      // Channel 1 Main Menu

  C2_I1,      // Channel 2 Item 1
  C2_I2,      // Channel 2 Item 2
  C2_I3,      // Channel 2 Item 3
  C2_I4,      // Channel 2 Item 4
  C2_I5,      // Channel 2 Item 5
  C2_MM,      // Channel 2 Main Menu

  C3_I1,      // Channel 3 Item 1
  C3_I2,      // Channel 3 Item 2
  C3_I3,      // Channel 3 Item 3
  C3_I4,      // Channel 3 Item 4
  C3_I5,      // Channel 3 Item 5
  C3_MM,      // Channel 3 Main Menu

  C4_I1,      // Channel 4 Item 1
  C4_I2,      // Channel 4 Item 2
  C4_I3,      // Channel 4 Item 3
  C4_I4,      // Channel 4 Item 4
  C4_I5,      // Channel 4 Item 5
  C4_MM       // Channel 4 Main Menu
};
ACTIVE_MENU_ITEM activeItem = ACTIVE_MENU_ITEM::Channel_1;

// Flags for certain mechanics
short menuOn = 1;
short calibrating = 0;
short curSWCount = 0;  // Tells Rotary Encoder when to switch menu windows
short prevSWCount = 0;

// Structure that will store each channel's configuration

/*  Determines the status of a pump channel
*
*   Is used as a redundancy and for optimization.
*/
enum PUMP_STATUS {
  IDLE,
  CONFIG,
  CALIBRATE,
  RUNNING,
  PAUSED,
  COMPLETE
};

/*  Determines the resolution of inputs
*   
*   Is used for dosage and bound constraints
*/
enum RES_STATUS {
  ONES = 1,
  TENTHS = 10,
  HUNDREDTHS = 100,
  RES_DONE = 1000
};

typedef struct PumpChannel {
  unsigned short motorNumber = 0;           // The motor number of this channel
  PUMP_STATUS pstat = PUMP_STATUS::IDLE;    // The status of the pump channel.
  RES_STATUS rstat = RES_STATUS::ONES;      // The current resolution digit for inputs.
  double dosage = 0.0;                      // The amount of medicine in either mL or mg/kg
  double infusionRate = 0.0;                // The rate at which to pump the medcine in either mL/hr or mg/kg/hr
  double syringeStart = 0.0;                // The length marker on the track where the pump will start pushing the syringe.
  double syringeEnd = 0.0;                  // The length marker on the tracker at which the syringe can no longer be pushed.
  unsigned long stepCount = 0;              // The amount of steps this channel needs to take to complete its infusion.
  unsigned long stepDelay = 0;              // The amount of time needed between steps to ensure the correct infusion rate.

  double resolutionCodes[2] = {0.0, 0.0};       // The return code and error when exceeding set parameter bounds.
}Pump_Channel;

// Create instances of the pump channel structure for each channel.
Pump_Channel pumpChannel1, pumpChannel2, pumpChannel3, pumpChannel4;
Pump_Channel *channels[4] = {&pumpChannel1, &pumpChannel2, &pumpChannel3, &pumpChannel4};

void setup() {
  // Initializes all four Stepper Motors.
  init_Stepper_Motors();
  init_Rotary_Encoder();
  init_LCD_Menu();

  // Allow the correct Serial Baud Rate
  Serial.begin(115200);
  Serial.print("Booting Firmware on Core: " + String(xPortGetCoreID()) + "\n");
}

void loop() {

  // Check to see if the menu needs to update windows
  if (curSWCount != prevSWCount) {
    
    // Update the menu if the menu is active
    if (menuOn) 
      switch_Scroll_Menu();

    // Update switch counter
    prevSWCount = curSWCount;
  }

  // Check to see if Rotary Encoder performed actions
  steppers.do_tasks();
  if (!calibrating)
    re_Controller();
  else if (calibrating && steppers.is_finished(calibrating - 1)) {
    calibrating = 0;
  }

}

/* ---------- METHODS FOR INITIALIZING HARDWARE ---------- */

/**
*   Initializes all four stepper motors
*/
void init_Stepper_Motors(void) {
  pinMode(Motor1_DIR, OUTPUT);
  pinMode(Motor2_DIR, OUTPUT);
  pinMode(Motor3_DIR, OUTPUT);
  pinMode(Motor4_DIR, OUTPUT);
  digitalWrite(Motor1_DIR, LOW);
  digitalWrite(Motor2_DIR, LOW);
  digitalWrite(Motor3_DIR, LOW);
  digitalWrite(Motor4_DIR, LOW);

  steppers.init_stepper(0, Motor1_STEP);
  steppers.init_stepper(1, Motor2_STEP);
  steppers.init_stepper(2, Motor3_STEP);
  steppers.init_stepper(3, Motor4_STEP);

  pumpChannel1.motorNumber = 1;
  pumpChannel2.motorNumber = 2;
  pumpChannel3.motorNumber = 3;
  pumpChannel4.motorNumber = 4;

  pumpChannel1.pstat = PUMP_STATUS::IDLE;
  pumpChannel2.pstat = PUMP_STATUS::IDLE;
  pumpChannel3.pstat = PUMP_STATUS::IDLE;
  pumpChannel4.pstat = PUMP_STATUS::IDLE;
}

/**
*     Initializes the Rotary Encoder
*/
void init_Rotary_Encoder(void) {

  pinMode(RE_SW, INPUT);
  attachInterrupt(digitalPinToInterrupt(RE_SW), re_SWInterrupt, RISING);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  re.attachFullQuad(RE_CLK, RE_DT);
  re.setCount(0);
  prevCountRE = re.getCount();
}

/**
*     Initiallizes the LCD Screen and Menu
*/
void init_LCD_Menu(void) {
  tft.init();
  tft.setRotation(2);
  print_Scroll_Menu();
}

/* ---------- METHODS FOR ROTARY ENCODER ACTIONS ---------- *

/**
*      Controls the counting and direction detection of the Rotary Encoder
*/
void re_Controller(void) {

  // Obtain the current time
  unsigned long curTimeRE = millis();

  if (curTimeRE - prevTimeRE >= debounceRE) {
    prevTimeRE = curTimeRE;

    curCountRE = (int) (re.getCount() / 2);

    // Check to see if the Rotary Encoder has moved.
    // If it has, determine the direction of rotation.
    if (curCountRE != prevCountRE) {
      dirRE = (curCountRE - prevCountRE > 0) ? TURN_DIR::CCW : TURN_DIR::CW;

      // Determine if the Rotary Encoder has moved enough to warrant menu scrolling
      if (curCountRE % RE_SCROLL_COUNT == 0 && menuOn == 1) {

        // If menu is on a channel item window, perform that window's action
        if ((int)(activeWindow) - 5 >= 0) {
          perform_Menu_Action(dirRE);
        }

        // Boolean conditions to reduce menu scrolling/refreshing
        short onMainMenus = ((int)(activeWindow) <= 4);                 // Determines if on a main menu window.
        short onCalibrationPage = !(((int)(activeWindow) - 4) % 5) && !onMainMenus;     // Determines if on a calibration page
        short doneWithResSet = (((int)((*channels[0]).rstat) + (int)((*channels[1]).rstat) + (int)((*channels[2]).rstat) + (int)((*channels[3]).rstat)) >= 1000);

        // If the menu is on a non-updating channel item window, do not update the menu. 
        if (!onCalibrationPage && (onMainMenus || !doneWithResSet)) {
          update_Scroll_Menu(dirRE);
          print_Scroll_Menu();
        }
      }
    }
    else {
      dirRE = TURN_DIR::STOP;
    }
    prevCountRE = curCountRE;
  }
}

/**
*       Acts as the interrupt for the Rotary Encoder's push switch
*/
void re_SWInterrupt(void) {
    unsigned long curTimeSW = millis();

    if (curTimeSW - prevTimeSW >= debounceSW) {
      prevTimeSW = curTimeSW;

      // Update switch counter 
      prevSWCount = curSWCount;
      curSWCount ^= 1;
    }
}

/* ---------- METHODS FOR STEPPER MOTOR ACTIONS ---------- */

/**
*   Sets the direction of a Stepper Motor
*/
void set_Stepper_Motor_Direction(int motorNum, TURN_DIR dir) {

  // If Clockwise, then go LOW ("Away from motor"); otherwise, go HIGH ("Toward motor");
  int pinLevel = (dir == TURN_DIR::CW) ? LOW : HIGH;

  switch (motorNum) {
    case 0:
      digitalWrite(Motor1_DIR, pinLevel);
      break;
    case 1:
      digitalWrite(Motor2_DIR, pinLevel);
      break;
    case 2:
      digitalWrite(Motor3_DIR, pinLevel);
      break;
    case 3:
      digitalWrite(Motor4_DIR, pinLevel);
      break;
    default:
      break;
  }
}

/**
*   Activate a Stepper Motor.
*   Return a 0 if valid motor, -1 if invalid motor.
*/
int activate_Stepper_Motor(int motorNum, int numSteps, int stepTime) {

  if (motorNum > 3 || motorNum < 0) {
    return -1;
  }

  steppers.start_finite(motorNum, stepTime, numSteps);
  activeMotors[motorNum] = 1;
  return 0;
}

/**
*     Returns an integer to the active Stepper Motor.
*     Returns -1 if there are no active Stepper Motors.
*/
int get_Active_Motor(void) {

  for (int i = 0; i < std::size(activeMotors); i++) {
    if (activeMotors[i] == 1) {
      return i;
    }
  }
  return -1; 
}



/* ---------- METHODS FOR MENU DISPLAY, CONFIGURATION, & NAVIGATION  ---------- */

/**
*     Perform channel menu item window action.
*/
void perform_Menu_Action(TURN_DIR dir) {

  switch(activeWindow) {
    /* CHANNEL 1 MENU ITEM WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM1:
      
      set_Channel_Dosage(1);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM2:

      set_Channel_Infusion_Rate(1);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM3:
      
      set_Syringe_Start_Rate(1);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM4:
      
      set_Syringe_End_Rate(1);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM5:
      if (steppers.is_finished(1 - 1)) {
        calibrate_Stepper(1 - 1);
        calibrating = 1; 
      }
      break;

    /* CHANNEL 2 MENU ITEM WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM1:

      set_Channel_Dosage(2);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM2:

      set_Channel_Infusion_Rate(2);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM3:
      
      set_Syringe_Start_Rate(2);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM4:
      
      set_Syringe_End_Rate(2);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM5:
      if (steppers.is_finished(2 - 1)) {
        calibrate_Stepper(2 - 1);
        calibrating = 2;
      }
      break;

    /* CHANNEL 3 MENU ITEM WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM1:

      set_Channel_Dosage(3);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM2:

      set_Channel_Infusion_Rate(3);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM3:
      
      set_Syringe_Start_Rate(3);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM4:
      
      set_Syringe_End_Rate(3);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM5:
      if (steppers.is_finished(3 - 1)) {
        calibrate_Stepper(3 - 1);
        calibrating = 3;
      }
      break;

    /* CHANNEL 4 MENU ITEM WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM1:

      set_Channel_Dosage(4);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM2:

      set_Channel_Infusion_Rate(4);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM3:
      
      set_Syringe_Start_Rate(4);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM4:
      
      set_Syringe_End_Rate(4);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM5:
      if (steppers.is_finished(4 - 1)) {
        calibrate_Stepper(4 - 1);
        calibrating = 4;
      }
      break;
  }
}

/**
*     Updates the scroll menu's active item
*/
void update_Scroll_Menu(TURN_DIR dir) {

  int activeItemInt = activeItem;

  switch(activeWindow) {
    case ACTIVE_MENU_WINDOW::CHANNEL1:
      activeItemInt -= 4;   // Set Channel 1's start to 0
      if (dir == TURN_DIR::CW) {
        activeItemInt++;
        activeItemInt %= 6;
      }
      else if (dir == TURN_DIR::CCW) {
        activeItemInt--;
        activeItemInt = (activeItemInt < 0) ? activeItemInt + 6 : activeItemInt;
      }
      activeItem = (ACTIVE_MENU_ITEM) (activeItemInt + 4);
      break; // End update from Channel 1

    case ACTIVE_MENU_WINDOW::CHANNEL2:
      activeItemInt -= 10;   // Set Channel 2's start to 0
      if (dir == TURN_DIR::CW) {
        activeItemInt++;
        activeItemInt %= 6;
      }
      else if (dir == TURN_DIR::CCW) {
        activeItemInt--;
        activeItemInt = (activeItemInt < 0) ? activeItemInt + 6 : activeItemInt;
      }
      activeItem = (ACTIVE_MENU_ITEM) (activeItemInt + 10);
      break; // End update from Channel 2

    case ACTIVE_MENU_WINDOW::CHANNEL3:
      activeItemInt -= 16;   // Set Channel 3's start to 0
      if (dir == TURN_DIR::CW) {
        activeItemInt++;
        activeItemInt %= 6;
      }
      else if (dir == TURN_DIR::CCW) {
        activeItemInt--;
        activeItemInt = (activeItemInt < 0) ? activeItemInt + 6 : activeItemInt;
      }
      activeItem = (ACTIVE_MENU_ITEM) (activeItemInt + 16);
      break; // End update from Channel 3

    case ACTIVE_MENU_WINDOW::CHANNEL4:
      activeItemInt -= 22;   // Set Channel 4's start to 0
      if (dir == TURN_DIR::CW) {
        activeItemInt++;
        activeItemInt %= 6;
      }
      else if (dir == TURN_DIR::CCW) {
        activeItemInt--;
        activeItemInt = (activeItemInt < 0) ? activeItemInt + 6 : activeItemInt;
      }
      activeItem = (ACTIVE_MENU_ITEM) (activeItemInt + 22);
      break; // End update from Channel 4

    default:  // Also for ACTIVE_MENU_WINDOW::MAIN:
      if (dir == TURN_DIR::CW) {
        activeItemInt++;
        activeItemInt %= 4;
      }
      else if (dir == TURN_DIR::CCW) {
        activeItemInt--;
        activeItemInt = (activeItemInt < 0) ? activeItemInt + 4 : activeItemInt;
      }
      activeItem = (ACTIVE_MENU_ITEM) activeItemInt;
      break; // End update from Main Menu
  }
}

/**
*     Controls the scroll menu's active window
*/
void switch_Scroll_Menu(void) {

  switch(activeWindow) {
    /* CHANNEL MENU WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL1:

      switch(activeItem) {
        case ACTIVE_MENU_ITEM::C1_I1:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL1_ITEM1;
            //activeItem = 
          break;
        case ACTIVE_MENU_ITEM::C1_I2:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL1_ITEM2;
            //activeItem = 
          break;
        case ACTIVE_MENU_ITEM::C1_I3:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL1_ITEM3;
            //activeItem = 
          break;
        case ACTIVE_MENU_ITEM::C1_I4:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL1_ITEM4;
            //activeItem = 
          break;
        case ACTIVE_MENU_ITEM::C1_I5:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL1_ITEM5;
            //activeItem = 
          break;
        default: // Also for ACTIVE_MENU_ITEM::C1_MM:
            activeWindow = ACTIVE_MENU_WINDOW::MAIN;
            activeItem = ACTIVE_MENU_ITEM::Channel_1;
          break;
      }

      break; // End switch from Channel 1

    case ACTIVE_MENU_WINDOW::CHANNEL2:
      switch(activeItem) {
        case ACTIVE_MENU_ITEM::C2_I1:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL2_ITEM1;
            //activeItem = 
          break;
        case ACTIVE_MENU_ITEM::C2_I2:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL2_ITEM2;
            //activeItem = 
          break;
        case ACTIVE_MENU_ITEM::C2_I3:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL2_ITEM3;
            //activeItem = 
          break;
        case ACTIVE_MENU_ITEM::C2_I4:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL2_ITEM4;
            //activeItem = 
          break;
        case ACTIVE_MENU_ITEM::C2_I5:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL2_ITEM5;
            //activeItem = 
          break;
        default: // Also for ACTIVE_MENU_ITEM::C2_MM:
            activeWindow = ACTIVE_MENU_WINDOW::MAIN;
            activeItem = ACTIVE_MENU_ITEM::Channel_1;
          break;
      }
      break; // End switch from Channel 2

    case ACTIVE_MENU_WINDOW::CHANNEL3:
      switch(activeItem) {
        case ACTIVE_MENU_ITEM::C3_I1:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL3_ITEM1;
            //activeItem = 
          break;
        case ACTIVE_MENU_ITEM::C3_I2:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL3_ITEM2;
            //activeItem = 
          break;
        case ACTIVE_MENU_ITEM::C3_I3:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL3_ITEM3;
            //activeItem = 
          break;
        case ACTIVE_MENU_ITEM::C3_I4:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL3_ITEM4;
            //activeItem = 
          break;
        case ACTIVE_MENU_ITEM::C3_I5:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL3_ITEM5;
            //activeItem = 
          break;
        default: // Also for ACTIVE_MENU_ITEM::C3_MM:
            activeWindow = ACTIVE_MENU_WINDOW::MAIN;
            activeItem = ACTIVE_MENU_ITEM::Channel_1;
          break;
      }
      break; // End switch from Channel 3

    case ACTIVE_MENU_WINDOW::CHANNEL4:
      switch(activeItem) {
        case ACTIVE_MENU_ITEM::C4_I1:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL4_ITEM1;
            //activeItem = 
          break;
        case ACTIVE_MENU_ITEM::C4_I2:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL4_ITEM2;
            //activeItem = 
          break;
        case ACTIVE_MENU_ITEM::C4_I3:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL4_ITEM3;
            //activeItem = 
          break;
        case ACTIVE_MENU_ITEM::C4_I4:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL4_ITEM4;
            //activeItem = 
          break;
        case ACTIVE_MENU_ITEM::C4_I5:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL4_ITEM5;
            //activeItem = 
          break;
        default: // Also for ACTIVE_MENU_ITEM::C4_MM:
            activeWindow = ACTIVE_MENU_WINDOW::MAIN;
            activeItem = ACTIVE_MENU_ITEM::Channel_1;
          break;
      }
      break; // End switch from Channel 4
    
    /* CHANNEL 1 MENU ITEM WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM1:
      // Return to main channel page if done setting dosage.
      if (!((int)((*channels[0]).rstat) % 1000)) {
        (*channels[0]).rstat = RES_STATUS::ONES;
        activeWindow = ACTIVE_MENU_WINDOW::CHANNEL1;
        activeItem = ACTIVE_MENU_ITEM::C1_I1;
      }
      else {
        (*channels[0]).rstat = (RES_STATUS)((*channels[0]).rstat * 10);
      }
      break; 

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM2:
      // Return to main channel page if done setting infusion rate.
      if (!((int)((*channels[0]).rstat) % 1000)) {
        (*channels[0]).rstat = RES_STATUS::ONES;
        activeWindow = ACTIVE_MENU_WINDOW::CHANNEL1;
        activeItem = ACTIVE_MENU_ITEM::C1_I2;
      }
      else {
        (*channels[0]).rstat = (RES_STATUS)((*channels[0]).rstat * 10);
      }
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM3:
      // Return to main channel page if done setting syringe start.
      if (!((int)((*channels[0]).rstat) % 1000)) {
        (*channels[0]).rstat = RES_STATUS::ONES;
        activeWindow = ACTIVE_MENU_WINDOW::CHANNEL1;
        activeItem = ACTIVE_MENU_ITEM::C1_I3;
      }
      else {
        (*channels[0]).rstat = (RES_STATUS)((*channels[0]).rstat * 10);
      }
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM4:
      // Return to main channel page if done setting syringe end.
      if (!((int)((*channels[0]).rstat) % 1000)) {
        (*channels[0]).rstat = RES_STATUS::ONES;
        activeWindow = ACTIVE_MENU_WINDOW::CHANNEL1;
        activeItem = ACTIVE_MENU_ITEM::C1_I4;
      }
      else {
        (*channels[0]).rstat = (RES_STATUS)((*channels[0]).rstat * 10);
      }
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM5:
      activeWindow = ACTIVE_MENU_WINDOW::CHANNEL1;
      activeItem = ACTIVE_MENU_ITEM::C1_I5;
      break;

    /* CHANNEL 2 MENU ITEM WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM1:
      // Return to main channel page if done setting dosage.
      if (!((int)((*channels[1]).rstat) % 1000)) {
        (*channels[1]).rstat = RES_STATUS::ONES;
        activeWindow = ACTIVE_MENU_WINDOW::CHANNEL2;
        activeItem = ACTIVE_MENU_ITEM::C2_I1;
      }
      else {
        (*channels[1]).rstat = (RES_STATUS)((*channels[1]).rstat * 10);
      }
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM2:
      // Return to main channel page if done setting infusion rate.
      if (!((int)((*channels[1]).rstat) % 1000)) {
        (*channels[1]).rstat = RES_STATUS::ONES;
        activeWindow = ACTIVE_MENU_WINDOW::CHANNEL2;
        activeItem = ACTIVE_MENU_ITEM::C2_I2;
      }
      else {
        (*channels[1]).rstat = (RES_STATUS)((*channels[1]).rstat * 10);
      }
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM3:
      // Return to main channel page if done setting syringe start.
      if (!((int)((*channels[1]).rstat) % 1000)) {
        (*channels[1]).rstat = RES_STATUS::ONES;
        activeWindow = ACTIVE_MENU_WINDOW::CHANNEL2;
        activeItem = ACTIVE_MENU_ITEM::C2_I3;
      }
      else {
        (*channels[1]).rstat = (RES_STATUS)((*channels[1]).rstat * 10);
      }
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM4:
      // Return to main channel page if done setting syringe end.
      if (!((int)((*channels[1]).rstat) % 1000)) {
        (*channels[1]).rstat = RES_STATUS::ONES;
        activeWindow = ACTIVE_MENU_WINDOW::CHANNEL2;
        activeItem = ACTIVE_MENU_ITEM::C2_I4;
      }
      else {
        (*channels[1]).rstat = (RES_STATUS)((*channels[1]).rstat * 10);
      }
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM5:
      activeWindow = ACTIVE_MENU_WINDOW::CHANNEL2;
      activeItem = ACTIVE_MENU_ITEM::C2_I5;
      break;
    
    /* CHANNEL 3 MENU ITEM WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM1:
      // Return to main channel page if done setting dosage.
      if (!((int)((*channels[2]).rstat) % 1000)) {
        (*channels[2]).rstat = RES_STATUS::ONES;
        activeWindow = ACTIVE_MENU_WINDOW::CHANNEL3;
        activeItem = ACTIVE_MENU_ITEM::C3_I1;
      }
      else {
        (*channels[2]).rstat = (RES_STATUS)((*channels[2]).rstat * 10);
      }
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM2:
      // Return to main channel page if done setting infusion rate.
      if (!((int)((*channels[2]).rstat) % 1000)) {
        (*channels[2]).rstat = RES_STATUS::ONES;
        activeWindow = ACTIVE_MENU_WINDOW::CHANNEL3;
        activeItem = ACTIVE_MENU_ITEM::C3_I2;
      }
      else {
        (*channels[2]).rstat = (RES_STATUS)((*channels[2]).rstat * 10);
      }
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM3:
      // Return to main channel page if done setting syringe start.
      if (!((int)((*channels[2]).rstat) % 1000)) {
        (*channels[2]).rstat = RES_STATUS::ONES;
        activeWindow = ACTIVE_MENU_WINDOW::CHANNEL3;
        activeItem = ACTIVE_MENU_ITEM::C3_I3;
      }
      else {
        (*channels[2]).rstat = (RES_STATUS)((*channels[2]).rstat * 10);
      }
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM4:
      // Return to main channel page if done setting syringe end.
      if (!((int)((*channels[2]).rstat) % 1000)) {
        (*channels[2]).rstat = RES_STATUS::ONES;
        activeWindow = ACTIVE_MENU_WINDOW::CHANNEL3;
        activeItem = ACTIVE_MENU_ITEM::C3_I4;
      }
      else {
        (*channels[2]).rstat = (RES_STATUS)((*channels[2]).rstat * 10);
      }
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM5:
      activeWindow = ACTIVE_MENU_WINDOW::CHANNEL3;
      activeItem = ACTIVE_MENU_ITEM::C3_I5;
      break;

    /* CHANNEL 4 MENU ITEM WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM1:
      // Return to main channel page if done setting dosage.
      if (!((int)((*channels[3]).rstat) % 1000)) {
        (*channels[3]).rstat = RES_STATUS::ONES;
        activeWindow = ACTIVE_MENU_WINDOW::CHANNEL4;
        activeItem = ACTIVE_MENU_ITEM::C4_I1;
      }
      else {
        (*channels[3]).rstat = (RES_STATUS)((*channels[3]).rstat * 10);
      }
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM2:
      // Return to main channel page if done setting infusion rate.
      if (!((int)((*channels[3]).rstat) % 1000)) {
        (*channels[3]).rstat = RES_STATUS::ONES;
        activeWindow = ACTIVE_MENU_WINDOW::CHANNEL4;
        activeItem = ACTIVE_MENU_ITEM::C4_I2;
      }
      else {
        (*channels[3]).rstat = (RES_STATUS)((*channels[3]).rstat * 10);
      }
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM3:
      // Return to main channel page if done setting syringe start.
      if (!((int)((*channels[3]).rstat) % 1000)) {
        (*channels[3]).rstat = RES_STATUS::ONES;
        activeWindow = ACTIVE_MENU_WINDOW::CHANNEL4;
        activeItem = ACTIVE_MENU_ITEM::C4_I3;
      }
      else {
        (*channels[3]).rstat = (RES_STATUS)((*channels[3]).rstat * 10);
      }
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM4:
      // Return to main channel page if done setting syringe end.
      if (!((int)((*channels[3]).rstat) % 1000)) {
        (*channels[3]).rstat = RES_STATUS::ONES;
        activeWindow = ACTIVE_MENU_WINDOW::CHANNEL4;
        activeItem = ACTIVE_MENU_ITEM::C4_I4;
      }
      else {
        (*channels[3]).rstat = (RES_STATUS)((*channels[3]).rstat * 10);
      }
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM5:
      activeWindow = ACTIVE_MENU_WINDOW::CHANNEL4;
      activeItem = ACTIVE_MENU_ITEM::C4_I5;
      break;

    /* DEFAULT/MAIN MENU WINDOW*/
    default: // Also for ACTIVE_MENU_WINDOW::MAIN:

      switch(activeItem) {
        case ACTIVE_MENU_ITEM::Channel_2:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL2;
            activeItem = ACTIVE_MENU_ITEM::C2_I1;
          break;
        case ACTIVE_MENU_ITEM::Channel_3:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL3;
            activeItem = ACTIVE_MENU_ITEM::C3_I1;
          break;
        case ACTIVE_MENU_ITEM::Channel_4:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL4;
            activeItem = ACTIVE_MENU_ITEM::C4_I1;
          break;
        default: // Also for ACTIVE_MENU_WINDOW::CHANNEL1:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL1;
            activeItem = ACTIVE_MENU_ITEM::C1_I1;
          break;
      }

      break; // End switch from Main Menu
  }

  print_Scroll_Menu();
}

/**
*     Prints the scroll menu
*     (Currently, only for Serial Monitor)
*/
void print_Scroll_Menu(void) {
  
  // Fill screen with light grey
  tft.fillScreen(TFT_LIGHTGREY);

  // Set "cursor" at top left corner of display (0,0) and select font 2
  // (cursor will move to next line automatically during printing with 'tft.println'
  //  or stay on the line is there is room for the text with tft.print)
  tft.setCursor(0, 0, 2);

  // Set the font colour to be white with a black background
  tft.setTextColor(TFT_BLACK,TFT_LIGHTGREY);  
  // Set text size multiplier to 4
  tft.setTextSize(4);

  String linePrint;
  int activeItemLoopIndex;

  // Print respective menu page
  switch(activeWindow) { 
    /* CHANNEL MENU WINDOWS*/   
    case ACTIVE_MENU_WINDOW::CHANNEL1:

      // Find the location of the item in the menu matrix
      activeItemLoopIndex = (int)(activeItem) - 4 + 4;

      // Print menu header
      print_Menu_Header(1, 0, 0);

      // Print each current menu option
      for (int i = 4; i < std::size(menu_l); i++) {

        // Grab the item from the menu options
        linePrint = menu_l[i];

        // Check if the item is currently highlighted
        if (activeItemLoopIndex == i) {
          linePrint += " <";
          tft.setTextColor(TFT_BLACK, TFT_SKYBLUE);
        } else {
          tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
        }

        // Print the item 
        tft.println(linePrint);  
      }

      break; // End print from Channel 1

    case ACTIVE_MENU_WINDOW::CHANNEL2:

      // Find the location of the item in the menu matrix
      activeItemLoopIndex = (int)(activeItem) - 10 + 4;

      // Print menu header
      print_Menu_Header(2, 0, 0);

      // Print each current menu option
      for (int i = 4; i < std::size(menu_l); i++) {

        // Grab the item from the menu options
        linePrint = menu_l[i];

        // Check if the item is currently highlighted
        if (activeItemLoopIndex == i) {
          linePrint += " <";
          tft.setTextColor(TFT_BLACK, TFT_SKYBLUE);
        } else {
          tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
        }

        // Print the item 
        tft.println(linePrint);  
      }

      break; // End print from Channel 2

    case ACTIVE_MENU_WINDOW::CHANNEL3:

      // Find the location of the item in the menu matrix
      activeItemLoopIndex = (int)(activeItem) - 16 + 4;

      // Print menu header
      print_Menu_Header(3, 0, 0);

      // Print each current menu option
      for (int i = 4; i < std::size(menu_l); i++) {

        // Grab the item from the menu options
        linePrint = menu_l[i];

        // Check if the item is currently highlighted
        if (activeItemLoopIndex == i) {
          linePrint += " <";
          tft.setTextColor(TFT_BLACK, TFT_SKYBLUE);
        } else {
          tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
        }

        // Print the item 
        tft.println(linePrint);  
      }

      break; // End print from Channel 3

    case ACTIVE_MENU_WINDOW::CHANNEL4:

      // Find the location of the item in the menu matrix
      activeItemLoopIndex = (int)(activeItem) - 22 + 4;

      // Print menu header
      print_Menu_Header(4, 0, 0);

      // Print each current menu option
      for (int i = 4; i < std::size(menu_l); i++) {

        // Grab the item from the menu options
        linePrint = menu_l[i];

        // Check if the item is currently highlighted
        if (activeItemLoopIndex == i) {
          linePrint += " <";
          tft.setTextColor(TFT_BLACK, TFT_SKYBLUE);
        } else {
          tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
        }

        // Print the item 
        tft.println(linePrint);  
      }

      break; // End print from Channel 4
    
    /* CHANNEL 1 MENU ITEM WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM1:
      
      // Print menu header
      print_Menu_Header(1, 1, 1);

      // Print item information
      print_Channel_Dosage(1);
      
      break; 

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM2:

      // Print menu header
      print_Menu_Header(1, 2, 1);

      // Print item information
      print_Channel_Infusion_Rate(1);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM3:

      // Print menu header
      print_Menu_Header(1, 3, 1);

      //Print item information
      print_Syringe_Start_Rate(1);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM4:

      // Print menu header
      print_Menu_Header(1, 4, 1);
      
      // Print item information
      print_Syringe_End_Rate(1);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM5:

      // Print menu header
      print_Menu_Header(1, 5, 1);
     
      // Print item information
      print_Channel_Calibrate(1);

      break;

    /* CHANNEL 2 MENU ITEM WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM1:

      // Print menu header
      print_Menu_Header(2, 1, 1);

      // Print item information
      print_Channel_Dosage(2);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM2:

      // Print menu header
      print_Menu_Header(2, 2, 1);

      // Print item information
      print_Channel_Infusion_Rate(2);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM3:

      // Print menu header
      print_Menu_Header(2, 3, 1);

      //Print item information
      print_Syringe_Start_Rate(2);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM4:

      // Print menu header
      print_Menu_Header(2, 4, 1);

      // Print item information
      print_Syringe_End_Rate(2);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM5:

      // Print menu header
      print_Menu_Header(2, 5, 1);

      // Print item information
      print_Channel_Calibrate(2);

      break;
    
    /* CHANNEL 3 MENU ITEM WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM1:

      // Print menu header
      print_Menu_Header(3, 1, 1);

      // Print item information
      print_Channel_Dosage(3);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM2:

      // Print menu header
      print_Menu_Header(3, 2, 1);

      // Print item information
      print_Channel_Infusion_Rate(3);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM3:

      // Print menu header
      print_Menu_Header(3, 3, 1);

      //Print item information
      print_Syringe_Start_Rate(3);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM4:

      // Print menu header
      print_Menu_Header(3, 4, 1);

      // Print item information
      print_Syringe_End_Rate(3);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM5:

      // Print menu header
      print_Menu_Header(3, 5, 1);

      // Print item information
      print_Channel_Calibrate(3);

      break;

    /* CHANNEL 4 MENU ITEM WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM1:

      // Print menu header
      print_Menu_Header(4, 1, 1);

      // Print item information
      print_Channel_Dosage(4);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM2:

      // Print menu header
      print_Menu_Header(4, 2, 1);

      // Print item information
      print_Channel_Infusion_Rate(4);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM3:

      // Print menu header
      print_Menu_Header(4, 3, 1);

      //Print item information
      print_Syringe_Start_Rate(4);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM4:

      // Print menu header
      print_Menu_Header(4, 4, 1);

      // Print item information
      print_Syringe_End_Rate(4);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM5:

      // Print menu header
      print_Menu_Header(4, 5, 1);

      // Print item information
      print_Channel_Calibrate(4);

      break;
    /* DEFAULT/MAIN MENU WINDOW*/
    default:  // Also for ACTIVE_MENU_WINDOW::MAIN:

      // Find the location of the item in the menu matrix
      activeItemLoopIndex = (int)(activeItem);

      // Print menu header
      print_Menu_Header(0, 0, 0);

      // Print each current menu option 
      for (int i = 0; i < 4; i++) {
        
        // Grab the item from the menu options
        linePrint = menu_l[i];

        // Check if the item is currently highlighted
        if (activeItemLoopIndex == i) {
          linePrint += " <";
          tft.setTextColor(TFT_BLACK, TFT_SKYBLUE);
        } else {
          tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
        }

        // Print the item 
        tft.println(linePrint);
      }
      break; // End print from Main Menu
  }
}

/*
*     Prints the proper menu header
*/
void print_Menu_Header(int channelNum, int itemNum, short subheader) {

      tft.println(" " + menu_l[((channelNum == 0) ? 9 : channelNum - 1)]);  //Print the name of the channel
      if (subheader) {
        tft.setTextSize(3);
        tft.println("  " + menu_l[itemNum + 3]); // Print the channel's item
      }
      tft.setTextSize(4);
      tft.println("-------------");
      tft.setTextSize(3);
}

/* ---------- METHODS FOR PUMP CHANNEL PARAMETER RESOLUTION DISPLAY & CONFIGURATION---------- */

/**
*     Sets the proper parameter resolution setting.
*
*     Returns -1 if the resolution exceeded the lower bound.
*     Returns 1 if the resolution exceeded the upper bound.
*     Returns 0 if no bound has been exceeded.
*/
short set_Resolution_Setting(RES_STATUS channelRes, double *param, double lower, double upper) {

  // Determine which digit will be manipulated
  float mult = (1.0 / (int)(channelRes));

  // Update dosage depending on scroll
  if (dirRE == TURN_DIR::CW) {
    (*param) += mult;
    if ((upper >= 0) && (*param) > upper) {
      (*param) = upper;
      return 1;
    }
  } 
  else if (dirRE == TURN_DIR::CCW) {
    (*param) -= mult;
    if ((*param) < lower) {
      (*param) = lower;
      return -1;
    }
  }
  return 0;
}

/**
*     Prints the proper parameter resolution setting
*/
void print_Resolution_Setting(RES_STATUS channelRes, String name, double param, String units, String preSpace, double outOfBounds[2]) {

    RES_STATUS print_res = RES_STATUS::ONES;
    String resStr = String(param);

  if (channelRes % 1000) {
    tft.print(preSpace);
    for (int i = 0; resStr[i] != '\0'; i++) {
      if (resStr[i] == '.') {
        print_res = (RES_STATUS)((int)print_res * 10);
        tft.print(resStr[i]);
      }
      else if (print_res == channelRes) {
        tft.setTextColor(TFT_BLACK, TFT_SKYBLUE);
        tft.print(resStr[i]);
        tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
      }
      else {
        tft.print(resStr[i]);
      }

      if (resStr[i - 1] == '.') {
        print_res = (RES_STATUS)((int)print_res * 10);
      }
    }
    tft.print(" " + units + "\n");

    tft.setTextSize(2);
    switch(channelRes) {
      case RES_STATUS::ONES:
        tft.println("   Editing Ones Place");
        break;
      case RES_STATUS::TENTHS:
        tft.println("  Editing Tenths Place");
        break;
      default:
        tft.println(" Editing Hundreths Place");
        break;
    } 
    tft.setTextSize(4);
    tft.println("-------------");
    tft.setTextSize(2);
    tft.println("Scroll to Adjust");
    switch(channelRes) {
      case RES_STATUS::ONES:
        tft.println("Click to Tenths");
        break;
      case RES_STATUS::TENTHS:
        tft.println("Click to Hundreths");
        break;
      default:
        tft.println("Click to Finish");
        break;
    }
    
    // If there is an outOfBounds flag pointer, print accordingly
    if (outOfBounds[0]) {
      if (outOfBounds[0] > 0) {
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        tft.println("Cannot exceed upper\n bound of " + String(outOfBounds[1]));
        tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
      }
      else if (outOfBounds[0] < 0) {
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        tft.println("Cannot exceed lower\n bound of " + String(outOfBounds[1]));
        tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
      }
    }

    tft.setTextSize(3); 
  }
  else {
    tft.println("New " + name + " Set:");
    tft.println(preSpace + "" + String(param) + " " + units + "\n");
    tft.setTextSize(4);
    tft.println("-------------");
    tft.setTextSize(3);
    tft.println("Click to Exit");
  }
}

/**
*     Resets the resolution codes of a pump channel.
*
*     Used after error is printed.
*/
void reset_Resolution_Codes(Pump_Channel *channel) {

  // Reset the resolutions codes
  (*channel).resolutionCodes[0] = 0;
  (*channel).resolutionCodes[1] = 0;
}

/* ---------- METHODS FOR PUMP CHANNEL DOSAGE DISPLAY & CONFIGURATION---------- */

/**
*   Update the channel's dosage amount
*/
void set_Channel_Dosage(int channelNum) {
      
  // Retrieve correct pump
  Pump_Channel *channel = channels[channelNum - 1];

  (*channel).resolutionCodes[0] = set_Resolution_Setting((*channel).rstat, &(*channel).dosage, 0, -1);

  if ((*channel).resolutionCodes[0] > 0)        // Upper
    (*channel).resolutionCodes[1] = -1;
  else if ((*channel).resolutionCodes[0] < 0)   // Lower
    (*channel).resolutionCodes[1] = 0;
  else                                      // No exceed
    (*channel).resolutionCodes[1] = 0;
}

/**
*   Print the proper channel dosage 
*/
void print_Channel_Dosage(int channelNum) {

  // Retrieve pump structure
  Pump_Channel *channel = channels[channelNum - 1];

  print_Resolution_Setting((*channel).rstat, "Dosage", (*channel).dosage, "mg/kg", "  ", (*channel).resolutionCodes);
  reset_Resolution_Codes(channel);
}

/* ---------- METHODS FOR PUMP CHANNEL INFUSION RATE DISPLAY & CONFIGURATION ---------- */

/**
*     Update the channel's infusion rate.
*/
void set_Channel_Infusion_Rate(int channelNum) {

  // Retrieve correct pump
  Pump_Channel *channel = channels[channelNum - 1];

  (*channel).resolutionCodes[0] = set_Resolution_Setting((*channel).rstat, &(*channel).infusionRate, 0, -1);

  if ((*channel).resolutionCodes[0] > 0)        // Upper
    (*channel).resolutionCodes[1] = -1;
  else if ((*channel).resolutionCodes[0] < 0)   // Lower
    (*channel).resolutionCodes[1] = 0;
  else                                      // No exceed
    (*channel).resolutionCodes[1] = 0;
}

/**
*     Print the proper channel infusion rate
*/
void print_Channel_Infusion_Rate(int channelNum) {

  // Retrieve pump structure
  Pump_Channel *channel = channels[channelNum - 1];

  print_Resolution_Setting((*channel).rstat, "Rate", (*channel).infusionRate, "mg/kg/hr", " ", (*channel).resolutionCodes);
  reset_Resolution_Codes(channel);
}

/* ---------- METHODS FOR PUMP CHANNEL SYRINGE START DISPLAY & CONFIGURATION---------- */

/**
*     Update the channel's infusion rate.
*/
void set_Syringe_Start_Rate(int channelNum) {

  // Retrieve correct pump
  Pump_Channel *channel = channels[channelNum - 1];

  (*channel).resolutionCodes[0] = set_Resolution_Setting((*channel).rstat, &(*channel).syringeStart, (*channel).syringeEnd, -1);

  if ((*channel).resolutionCodes[0] > 0)        // Upper
    (*channel).resolutionCodes[1] = -1;
  else if ((*channel).resolutionCodes[0] < 0)   // Lower
    (*channel).resolutionCodes[1] = (*channel).syringeEnd;
  else                                      // No exceed
    (*channel).resolutionCodes[1] = 0;
}

/**
*     Print the proper channel infusion rate
*/
void print_Syringe_Start_Rate(int channelNum) {

  // Retrieve pump structure
  Pump_Channel *channel = channels[channelNum - 1];

  print_Resolution_Setting((*channel).rstat, "Start", (*channel).syringeStart, "cm", "  ", (*channel).resolutionCodes);
  reset_Resolution_Codes(channel);
}

/* ---------- METHODS FOR PUMP CHANNEL SYRINGE END & CONFIGURATION---------- */

/**
*     Update the channel's infusion rate.
*/
void set_Syringe_End_Rate(int channelNum) {

  // Retrieve correct pump
  Pump_Channel *channel = channels[channelNum - 1];

  (*channel).resolutionCodes[0] = set_Resolution_Setting((*channel).rstat, &(*channel).syringeEnd, 0, (*channel).syringeStart);

  if ((*channel).resolutionCodes[0] > 0)        // Upper
    (*channel).resolutionCodes[1] = (*channel).syringeEnd;
  else if ((*channel).resolutionCodes[0] < 0)   // Lower
    (*channel).resolutionCodes[1] = 0;
  else                                      // No exceed
    (*channel).resolutionCodes[1] = 0;
}

/**
*     Print the proper channel infusion rate
*/
void print_Syringe_End_Rate(int channelNum) {

  // Retrieve pump structure
  Pump_Channel *channel = channels[channelNum - 1];

  print_Resolution_Setting((*channel).rstat, "End", (*channel).syringeEnd, "cm", "   ", (*channel).resolutionCodes);
  reset_Resolution_Codes(channel);
}

/* ---------- METHODS FOR PUMP CHANNEL CALIBRATION DISPLAY & CONFIGURATION ---------- */

/**
*   Use the Rotary Encoder to manually move a selected Stepper Motor
*/ 
void calibrate_Stepper(int motorNum) {

  if (dirRE == TURN_DIR::CCW) {
    if (steppers.is_finished(motorNum)) {
      set_Stepper_Motor_Direction(motorNum, TURN_DIR::CCW);
      activate_Stepper_Motor(motorNum, MOTOR_STEPS * MICROSTEPS, 1000);
    }
    // else if (steppers.is_paused(motorNum)) {
    //   steppers.resume(motorNum);
    // }
  }
  else if (dirRE == TURN_DIR::CW) { 
    if (steppers.is_finished(motorNum)) {
      set_Stepper_Motor_Direction(motorNum, TURN_DIR::CW);
      activate_Stepper_Motor(motorNum, MOTOR_STEPS * MICROSTEPS, 1000);
    }
    // else if (steppers.is_paused(motorNum)) {
    //   steppers.resume(motorNum);
    // }
  }
}

/**
*    Print the calibration instructions.
*/
void print_Channel_Calibrate(int channelNum) {

  tft.println("CW -> Push");
  tft.println("CCW -> Pull");
  tft.setTextSize(4);
  tft.println("-------------");
  tft.setTextSize(3);
  tft.println("Scroll to Adjust");
  tft.println("");
  tft.println("Click to Exit");
}



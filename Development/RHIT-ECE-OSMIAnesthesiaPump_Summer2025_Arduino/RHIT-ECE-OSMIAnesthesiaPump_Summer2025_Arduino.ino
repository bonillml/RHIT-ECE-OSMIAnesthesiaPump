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



// Angle-Length (Revolutions per Centimeter)
#define ANGLE_LENGTH 13

// Step-Angle (Steps per Revolution)
#define STEP_ANGLE MOTOR_STEPS * MICROSTEPS

// Step-Length (Steps per Centimeter)
#define STEP_LENGTH ANGLE_LENGTH * STEP_ANGLE

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
String menu_l[11] = {"Channel 1", "Channel 2", "Channel 3", "Channel 4", "Dosage", "Infusion Rate", "Syringe Start", "Syringe End", "Calibrate", "Start", "Main Menu"};
short num_channel_options = std::size(menu_l) - 4;

enum ACTIVE_MENU_WINDOW { // Tracks which menu screen to display 
  MAIN,       // Displays the main menu options
  CHANNEL1,   // Displays pump channel 1 menu options
  CHANNEL2,   // Displays pump channel 2 menu options
  CHANNEL3,   // Displays pump channel 3 menu options
  CHANNEL4,   // Displays pump channel 4 menu options

  CHANNEL1_ITEM1 = 7,  // Channel 1 Item 1  (SET EQUAL TO num_channel_options)
  CHANNEL1_ITEM2,      // Channel 1 Item 2
  CHANNEL1_ITEM3,      // Channel 1 Item 3
  CHANNEL1_ITEM4,      // Channel 1 Item 4
  CHANNEL1_ITEM5,      // Channel 1 Item 5
  CHANNEL1_ITEM6,      // Channel 1 Item 6

  CHANNEL2_ITEM1,      // Channel 2 Item 1
  CHANNEL2_ITEM2,      // Channel 2 Item 2
  CHANNEL2_ITEM3,      // Channel 2 Item 3
  CHANNEL2_ITEM4,      // Channel 2 Item 4
  CHANNEL2_ITEM5,      // Channel 2 Item 5
  CHANNEL2_ITEM6,      // Channel 2 Item 6

  CHANNEL3_ITEM1,      // Channel 3 Item 1
  CHANNEL3_ITEM2,      // Channel 3 Item 2
  CHANNEL3_ITEM3,      // Channel 3 Item 3
  CHANNEL3_ITEM4,      // Channel 3 Item 4
  CHANNEL3_ITEM5,      // Channel 3 Item 5
  CHANNEL3_ITEM6,      // Channel 3 Item 6

  CHANNEL4_ITEM1,      // Channel 4 Item 1
  CHANNEL4_ITEM2,      // Channel 4 Item 2
  CHANNEL4_ITEM3,      // Channel 4 Item 3
  CHANNEL4_ITEM4,      // Channel 4 Item 4
  CHANNEL4_ITEM5,      // Channel 4 Item 5
  CHANNEL4_ITEM6       // Channel 4 Item 6
};
ACTIVE_MENU_WINDOW activeWindow = ACTIVE_MENU_WINDOW::MAIN;

enum ACTIVE_MENU_ITEM {
  Main,  // Extra piece for simpler enum calculations
  Channel_1,  // Main Menu Channel 1
  Channel_2,  // Main Menu Channel 2
  Channel_3,  // Main Menu Channel 3
  Channel_4,  // Main Menu Channel 4

  C1_I1 = 7,  // Channel 1 Item 1  (SET EQUAL TO num_channel_options)
  C1_I2,      // Channel 1 Item 2
  C1_I3,      // Channel 1 Item 3
  C1_I4,      // Channel 1 Item 4
  C1_I5,      // Channel 1 Item 5
  C1_I6,      // Channel 1 Item 6
  C1_MM,      // Channel 1 Main Menu

  C2_I1,      // Channel 2 Item 1
  C2_I2,      // Channel 2 Item 2
  C2_I3,      // Channel 2 Item 3
  C2_I4,      // Channel 2 Item 4
  C2_I5,      // Channel 2 Item 5
  C2_I6,      // Channel 2 ITem 6
  C2_MM,      // Channel 2 Main Menu

  C3_I1,      // Channel 3 Item 1
  C3_I2,      // Channel 3 Item 2
  C3_I3,      // Channel 3 Item 3
  C3_I4,      // Channel 3 Item 4
  C3_I5,      // Channel 3 Item 5
  C3_I6,      // Channel 3 Item 6
  C3_MM,      // Channel 3 Main Menu

  C4_I1,      // Channel 4 Item 1
  C4_I2,      // Channel 4 Item 2
  C4_I3,      // Channel 4 Item 3
  C4_I4,      // Channel 4 Item 4
  C4_I5,      // Channel 4 Item 5
  C4_I6,      // Channel 4 Item 6
  C4_MM       // Channel 4 Main Menu
};
ACTIVE_MENU_ITEM activeItem = ACTIVE_MENU_ITEM::Channel_1;

// Flags for certain mechanics
short menuOn = 1;
short startStop = 0; // 0 - Exit, 1 - Start/Stop
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
  unsigned short directionPin = 0;          // The direction pin number of this channel.
  PUMP_STATUS pstat = PUMP_STATUS::IDLE;    // The status of the pump channel.
  RES_STATUS rstat = RES_STATUS::ONES;      // The current resolution digit for inputs.
  double dosage = 9.2;                      // The amount of medicine in either mL or mg/kg
  double infusionRate = 36.80;              // The rate at which to pump the medcine in either mL/hr or mg/kg/hr
  double syringeStart = 10.0;               // The length marker on the track where the pump will start pushing the syringe.
  double syringeEnd = 14.5;                 // The length marker on the tracker at which the syringe can no longer be pushed.
  unsigned long stepCount = 0;              // The amount of steps this channel needs to take to complete its infusion.
  unsigned long stepDelay = 0;              // The amount of time needed between steps to ensure the correct infusion rate.

  double resolutionCodes[2] = {0.0, 0.0};   // The return code and error when exceeding set parameter bounds.
}Pump_Channel;

// Create instances of the pump channel structure for each channel.
Pump_Channel pumpChannel1, pumpChannel2, pumpChannel3, pumpChannel4;
Pump_Channel *channels[4] = {&pumpChannel1, &pumpChannel2, &pumpChannel3, &pumpChannel4};

// Setup an array of pointers to various setting functions. Allows quick and structured access to setting methods.
void set_Channel_Item_1(int channelNum) { set_Channel_Dosage(channelNum); }
void set_Channel_Item_2(int channelNum) { set_Channel_Infusion_Rate(channelNum); }
void set_Channel_Item_3(int channelNum) { set_Syringe_Start(channelNum); }
void set_Channel_Item_4(int channelNum) { set_Syringe_End(channelNum); }
void set_Channel_Item_5(int channelNum) { if (steppers.is_finished(channelNum - 1)) calibrate_Stepper(1 - 1); }
void set_Channel_Item_6(int channelNum) { set_Start_Stop(channelNum); }

void (*set_funcs[6])(int) = {set_Channel_Item_1, set_Channel_Item_2, set_Channel_Item_3, set_Channel_Item_4, set_Channel_Item_5, set_Channel_Item_6};

// Setup an array of pointers to various printing functions. Allows quick and structured access to printing methods.
void print_Channel_Item_1(int channelNum) { print_Channel_Dosage(channelNum); }
void print_Channel_Item_2(int channelNum) { print_Channel_Infusion_Rate(channelNum); }
void print_Channel_Item_3(int channelNum) { print_Syringe_Start(channelNum); }
void print_Channel_Item_4(int channelNum) { print_Syringe_End(channelNum); }
void print_Channel_Item_5(int channelNum) { print_Channel_Calibrate(channelNum); }
void print_Channel_Item_6(int channelNum) { print_Start_Stop(channelNum); }

void (*print_funcs[6])(int) = {print_Channel_Item_1, print_Channel_Item_2, print_Channel_Item_3, print_Channel_Item_4, print_Channel_Item_5, print_Channel_Item_6};

// Declare RTOS timer
TimerHandle_t motorTimer;

void motorTimerAction(TimerHandle_t xTimer) {
  steppers_do_tasks();
}

// Used for timing the UI updates
int time_test = 0;

void setup() {
  // Allow the correct Serial Baud Rate
  Serial.begin(115200);

  // Initializes all four Stepper Motors.
  init_Stepper_Motors();
  init_Rotary_Encoder();
  init_LCD_Menu();

  // Initialize the time test for UI updates
  time_test = 0;

  motorTimer = xTimerCreate("MotorTimer", pdMS_TO_TICKS(5), pdTRUE, 0, motorTimerAction);

  if (motorTimer) {
    Serial.println("RTOS Timer Initialized");
    xTimerStart(motorTimer, 0);
  }
  else
    Serial.println("RTOS Timer Failed to Initialize");

  Serial.print("Booting Firmware on Core: " + String(xPortGetCoreID()) + "\n");
  Serial.println("Un-optimized Software Booted");
}

void loop() {

  update_All_Channels();

  // Check to see if the menu needs to update windows
  if (curSWCount != prevSWCount) {
    
    // Update the menu if the menu is active
    if (menuOn) {
      if (time_test)
        time_test = millis();
      switch_Scroll_Menu();
      if (time_test)
        Serial.println("UI Switch Menu took " + String(millis() - time_test) + " ms.");
    }
    // Update switch counter
    prevSWCount = curSWCount;
  }

  //steppers_do_tasks();    // Attempt with Timer Interrupts
  // Check to see if Rotary Encoder performed actions
  re_Controller();

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

  // Set the motor number for each pump channel
  pumpChannel1.motorNumber = 1;
  pumpChannel2.motorNumber = 2;
  pumpChannel3.motorNumber = 3;
  pumpChannel4.motorNumber = 4;

  // Set the pin number for each pump channel
  pumpChannel1.directionPin = Motor1_DIR;
  pumpChannel2.directionPin = Motor2_DIR;
  pumpChannel3.directionPin = Motor3_DIR;
  pumpChannel4.directionPin = Motor4_DIR;

  // Set the default motor state for each pump channel
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

/* ---------- METHODS FOR PUMP CHANNEL INFORMATION ---------- */

// Updates a channel's status
void update_Channel_Status(int channelNum) {
 
  // Retrieve the channel 
  Pump_Channel *channel = channels[channelNum - 1];

  // Determine if the channel is calibrating or running
  // TODO: Test if removing libray-dependent condition works
  if ((steppers.is_running(channelNum - 1))) {
    
    if ((*channel).pstat == PUMP_STATUS::CALIBRATE)
      (*channel).pstat = PUMP_STATUS::CALIBRATE;
    else
      (*channel).pstat = PUMP_STATUS::RUNNING;

    return;
  }

  // Set status as paused
  // TODO: Test if removing libray-dependent condition works
  if (steppers.is_paused(channelNum - 1)) {
    (*channel).pstat = PUMP_STATUS::PAUSED;

    
    return;
  }

  // Don't do anything if channel is being configured.
  if ((*channel).pstat == PUMP_STATUS::CONFIG || (*channel).pstat == PUMP_STATUS::IDLE) {
    
    
    return;
  }
  // Determine if the channel is idle or complete
  // TODO: Test if removing libray-dependent condition works
  if (steppers.is_finished(channelNum - 1)) {

    if ((*channel).pstat == PUMP_STATUS::RUNNING) {
      (*channel).pstat = PUMP_STATUS::COMPLETE;
      Serial.println("Channel " + String((*channel).motorNumber + 1) + " Completed");
    }
    else
      (*channel).pstat = PUMP_STATUS::IDLE;
    
    
    return;
  }


}

// Updates every channel's status
void update_All_Channels(void) {

  for (int i = 1; i <= 4; i++)
    update_Channel_Status(i);
}

// Prints to the serial monitor the state of a given pump channel.
void print_Channel_Status(int channelNum) {

  // Retrieve the channel 
  Pump_Channel *channel = channels[channelNum - 1];

  switch ((*channel).pstat) {
    case PUMP_STATUS::IDLE:
      Serial.println("Channel " + String(channelNum) + ": IDLE");
      break;
    case PUMP_STATUS::CONFIG:
      Serial.println("Channel " + String(channelNum) + ": CONFIG");
      break;
    case PUMP_STATUS::CALIBRATE:
      Serial.println("Channel " + String(channelNum) + ": CALIBRATE");
      break;
    case PUMP_STATUS::RUNNING:
      Serial.println("Channel " + String(channelNum) + ": RUNNING");
      break;
    case PUMP_STATUS::PAUSED:
      Serial.println("Channel " + String(channelNum) + ": PAUSED");
      break;
    case PUMP_STATUS::COMPLETE:
      Serial.println("Channel " + String(channelNum) + ": COMPLETE");
      break;
    default:
      Serial.println("Channel " + String(channelNum) + ": UNKNOWN");
      break;
  }
}

// Prints to the serial monitor the state of all pump channels.
void print_All_Channels(void) {

  for (int i = 1; i <= 4; i++)
    print_Channel_Status(i);
  Serial.println();
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
        if ((int)(activeWindow) >= num_channel_options) {
          if (time_test)
            time_test = millis();
          perform_Menu_Action();
          if (time_test)
            Serial.println("UI Perform Action took " + String(millis() - time_test) + " ms.");
        }

        // Boolean conditions to reduce menu scrolling/refreshing
        short onMainMenus = ((int)(activeWindow) <= 4);                 // Determines if on a main menu window.
        short onCalibrationPage = !(((int)(activeWindow) - (int)(ACTIVE_MENU_WINDOW::CHANNEL1_ITEM5)) % (num_channel_options - 1)) && !onMainMenus;     // Determines if on a calibration page
        short doneWithResSet = (((int)((*channels[0]).rstat) + (int)((*channels[1]).rstat) + (int)((*channels[2]).rstat) + (int)((*channels[3]).rstat)) >= 1000);

        // If the menu is on a non-updating channel item window, do not update the menu. 
        if (!onCalibrationPage && (onMainMenus || !doneWithResSet)) {
          if (time_test)
            time_test = millis();
          update_Scroll_Menu();
          print_Scroll_Menu();
          if (time_test)
            Serial.println("UI Update and Print took " + String(millis() - time_test) + " ms.");
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
void set_Stepper_Motor_Direction(Pump_Channel *channel, TURN_DIR dir) {

  /* 
  * Clockwise          =    Motor Pushes   =  LOW
  * Counter Clockwise  =    Motor Pulls    =  HIGH
  */

  /*
  * Check which direction to set the motor to.
  *
  * ENSURES THAT THE MOTOR IS ALWAYS PUSHING SYRINGE WHEN ACTIVELY RUNNING
  */
  int pinLevel = (dir == TURN_DIR::CW || (*channel).pstat == PUMP_STATUS::RUNNING) ? LOW : HIGH;

  // Set the direction of the motor
  digitalWrite((*channel).directionPin, pinLevel);
}

/**
*   Activate a Stepper Motor.
*   Return a 0 if valid motor, -1 if invalid motor.
*/
int activate_Stepper_Motor(int motorNum, int numSteps, int stepTime) {

  if (motorNum > 3 || motorNum < 0) {
    return -1;
  }

  // TODO: Test if pausing timer here works
  
  steppers.start_finite(motorNum, stepTime, numSteps);

  // If the motor isn't calibrating, set it as running.
  if ((*channels[motorNum]).pstat != PUMP_STATUS::CALIBRATE)
    (*channels[motorNum]).pstat = PUMP_STATUS::RUNNING;

  return 0;
}

/**
*   Performs a step for each running motor and decrements their step count.
*/
void steppers_do_tasks(void) {

  steppers.do_tasks();

  uint32_t remaining_steps;

  for (int i = 0; i < 4; i++) {
    
    if ((remaining_steps = steppers.get_remaining_steps(i)) && ((*channels[i]).pstat == PUMP_STATUS::RUNNING || (*channels[i]).pstat == PUMP_STATUS::CALIBRATE))
      (*channels[i]).stepCount = remaining_steps;
  }
}

/**
*     Returns an integer to the active Stepper Motor.
*     Returns -1 if there are no active Stepper Motors.
*/
int get_Active_Motor(void) {

  for (int i = 0; i < std::size(channels); i++) {
    if ((*channels[i]).pstat == PUMP_STATUS::RUNNING) {
      return i;
    }
  }
  return -1; 
}

/* ---------- METHODS FOR MENU DISPLAY, CONFIGURATION, & NAVIGATION  ---------- */

/**
*     Perform channel menu item window action. ~ Nearly 125 lines of code
*     
*     (DEPRECATED)
*/
void perform_Menu_Action(void) {

  switch(activeWindow) {
    /* CHANNEL 1 MENU ITEM WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM1:
      
      set_Channel_Dosage(1);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM2:

      set_Channel_Infusion_Rate(1);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM3:
      
      set_Syringe_Start(1);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM4:
      
      set_Syringe_End(1);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM5:
      // TODO: Test if removing libray-dependent condition works
      if (steppers.is_finished(1 - 1)) {
        calibrate_Stepper(1 - 1);
      }
      break;
    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM6:
      set_Start_Stop(1);
      break;
    /* CHANNEL 2 MENU ITEM WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM1:

      set_Channel_Dosage(2);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM2:

      set_Channel_Infusion_Rate(2);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM3:
      
      set_Syringe_Start(2);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM4:
      
      set_Syringe_End(2);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM5:
      // TODO: Test if removing libray-dependent condition works
      if (steppers.is_finished(2 - 1)) {
        calibrate_Stepper(2 - 1);
      }
      break;
    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM6:
      set_Start_Stop(2);
      break;
    /* CHANNEL 3 MENU ITEM WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM1:

      set_Channel_Dosage(3);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM2:

      set_Channel_Infusion_Rate(3);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM3:
      
      set_Syringe_Start(3);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM4:
      
      set_Syringe_End(3);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM5:
      // TODO: Test if removing libray-dependent condition works
      if (steppers.is_finished(3 - 1)) {
        calibrate_Stepper(3 - 1);

      }
      break;
    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM6:
      set_Start_Stop(3);
      break;
    /* CHANNEL 4 MENU ITEM WINDOWS*/
    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM1:

      set_Channel_Dosage(4);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM2:

      set_Channel_Infusion_Rate(4);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM3:
      
      set_Syringe_Start(4);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM4:
      
      set_Syringe_End(4);
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM5:
      // TODO: Test if removing libray-dependent condition works
      if (steppers.is_finished(4 - 1)) {
        calibrate_Stepper(4 - 1);
      }
      break;
    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM6:
      set_Start_Stop(4);
      break;
  }
}

/**
*     Perform channel menu item window action. ~ Nearly 10 lines of code
*     
*     REQUIRES FOLLOWING ASSUMPTIONS;
*     - ACTIVE_MENU_WINDOW::<CHANNEL1> == ACTIVE_MENU_ITEM::<Channel_1> W.L.O.G.
*     - The MAIN MENU enum in both ACTIVE_MENU_WINDOW and ACTIVE_MENU_ITEM are 0
*     - All channel items are contiguous in their respective enums.
*     - Channel 1 Item 1 starts with value 7, Channel 2 Item 1 starts with value 14, and so forth.
*
*/
// void perform_Menu_Action_Optimized(void) {

//   // Determine the channel # and channel item #.
//   short channelNum = (short)((int)activeWindow / (double)num_channel_options);  // SHOULD BE BETWEEN 0 and 4 FOR THIS METHOD
//   short channelItemNum = (short)((int)activeWindow % num_channel_options);      // Zero Indexed: i.e., item 1 --> 0, item 2 --> 1, etc.

//   // Check if current window allows function to run
//   if ((int)activeWindow >= num_channel_options)
//     (*set_funcs[channelItemNum])(channelNum);       // Set Item Information
// }

/**
*     Updates the scroll menu's active item ~ Nearly 70 lines of code
*     
*     (DEPRECATED)
*/
void update_Scroll_Menu(void) {

  int activeItemInt = activeItem;

  switch(activeWindow) {
    case ACTIVE_MENU_WINDOW::CHANNEL1:
      activeItemInt -= (int)(ACTIVE_MENU_ITEM::C1_I1);   // Set Channel 1's start to 0
      if (dirRE == TURN_DIR::CW) {
        activeItemInt++;
        activeItemInt %= num_channel_options;
      }
      else if (dirRE == TURN_DIR::CCW) {
        activeItemInt--;
        activeItemInt = (activeItemInt < 0) ? activeItemInt + num_channel_options : activeItemInt;
      }
      activeItem = (ACTIVE_MENU_ITEM) (activeItemInt + (int)(ACTIVE_MENU_ITEM::C1_I1));
      break; // End update from Channel 1

    case ACTIVE_MENU_WINDOW::CHANNEL2:
      activeItemInt -= (int)(ACTIVE_MENU_ITEM::C2_I1);   // Set Channel 2's start to 0
      if (dirRE == TURN_DIR::CW) {
        activeItemInt++;
        activeItemInt %= num_channel_options;
      }
      else if (dirRE == TURN_DIR::CCW) {
        activeItemInt--;
        activeItemInt = (activeItemInt < 0) ? activeItemInt + num_channel_options : activeItemInt;
      }
      activeItem = (ACTIVE_MENU_ITEM) (activeItemInt + (int)(ACTIVE_MENU_ITEM::C2_I1));
      break; // End update from Channel 2

    case ACTIVE_MENU_WINDOW::CHANNEL3:
      activeItemInt -= (int)(ACTIVE_MENU_ITEM::C3_I1);   // Set Channel 3's start to 0
      if (dirRE == TURN_DIR::CW) {
        activeItemInt++;
        activeItemInt %= num_channel_options;
      }
      else if (dirRE == TURN_DIR::CCW) {
        activeItemInt--;
        activeItemInt = (activeItemInt < 0) ? activeItemInt + num_channel_options : activeItemInt;
      }
      activeItem = (ACTIVE_MENU_ITEM) (activeItemInt + (int)(ACTIVE_MENU_ITEM::C3_I1));
      break; // End update from Channel 3

    case ACTIVE_MENU_WINDOW::CHANNEL4:
      activeItemInt -= (int)(ACTIVE_MENU_ITEM::C4_I1);   // Set Channel 4's start to 0
      if (dirRE == TURN_DIR::CW) {
        activeItemInt++;
        activeItemInt %= num_channel_options;
      }
      else if (dirRE == TURN_DIR::CCW) {
        activeItemInt--;
        activeItemInt = (activeItemInt < 0) ? activeItemInt + num_channel_options : activeItemInt;
      }
      activeItem = (ACTIVE_MENU_ITEM) (activeItemInt + (int)(ACTIVE_MENU_ITEM::C4_I1));
      break; // End update from Channel 4

    default:  // Also for ACTIVE_MENU_WINDOW::MAIN:
      if (dirRE == TURN_DIR::CW) {
        activeItemInt++;
        activeItemInt %= 4;
      }
      else if (dirRE == TURN_DIR::CCW) {
        activeItemInt--;
        activeItemInt = (activeItemInt < 0) ? activeItemInt + 4 : activeItemInt;
      }
      activeItem = (ACTIVE_MENU_ITEM) activeItemInt;
      break; // End update from Main Menu
  }
}

/**
*     Updates the scroll menu's active item ~ Nearly 40 lines of code
*     
*     REQUIRES FOLLOWING ASSUMPTIONS;
*     - ACTIVE_MENU_WINDOW::<CHANNEL1> == ACTIVE_MENU_ITEM::<Channel_1> W.L.O.G.
*     - The MAIN MENU enum in both ACTIVE_MENU_WINDOW and ACTIVE_MENU_ITEM are 0
*     - All channel items are contiguous in their respective enums.
*     - Channel 1 Item 1 starts with value 7, Channel 2 Item 1 starts with value 14, and so forth.
*
*/
// void update_Scroll_Menu_Optimized(TURN_DIR dir) {

//   // Determine the channel # and channel item #.
//   short channelNum = (short)activeWindow;                                     // SHOULD BE BETWEEN 0 and 4 FOR THIS METHOD
//   short channelItemNum = (short)((int)activeItem % num_channel_options);      // Zero Indexed: i.e., item 1 --> 0, item 2 --> 1, etc.

//   // Define variable to track next highlighted option (Default values for main menu assignment)
//   int activeItemInt = activeItem;
//   short activeItemMod = 4;

//   // Determine the type of window
//   if (channelNum > 0 && channelNum < 5) {
//     // Channel menu assignment

//     // Reassign activeItemMod
//     activeItemMod = num_channel_options;

//     // Shift the channel item index to 0
//     activeItemInt -= (int)(num_channel_options * channelNum);
//   }
//   else if (!activeWindow) {
//     // Main menu assignment 
//     activeItemInt--;
//   }
//   else {
//     // Unexpected case
//     return;
//   }

//   if (dir == TURN_DIR::CW) {
//     activeItemInt++;
//     activeItemInt %= activeItemMod;
//   }
//   else if (dir == TURN_DIR::CCW) {
//     activeItemInt--;
//     activeItemInt = (activeItemInt < 0) ? activeItemInt + activeItemMod : activeItemInt;
//   }

//   // Cast activeItem back to enumeration
//   activeItem = (ACTIVE_MENU_ITEM) (activeItemInt + ((channelNum) ? (int)(num_channel_options * channelNum) : 1));
// }

/**
*     Controls the scroll menu's active window ~ Nearly 450 lines of code
*     
*     (DEPRECATED) 
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
        case ACTIVE_MENU_ITEM::C1_I6:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL1_ITEM6;
            calculate_Motor_Parameters(1);
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
        case ACTIVE_MENU_ITEM::C2_I6:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL2_ITEM6;
            calculate_Motor_Parameters(2); 
          break;
        default: // Also for ACTIVE_MENU_ITEM::C2_MM:
            activeWindow = ACTIVE_MENU_WINDOW::MAIN;
            activeItem = ACTIVE_MENU_ITEM::Channel_2;
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
        case ACTIVE_MENU_ITEM::C3_I6:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL3_ITEM6;
            calculate_Motor_Parameters(3);
          break;
        default: // Also for ACTIVE_MENU_ITEM::C3_MM:
            activeWindow = ACTIVE_MENU_WINDOW::MAIN;
            activeItem = ACTIVE_MENU_ITEM::Channel_3;
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
        case ACTIVE_MENU_ITEM::C4_I6:
            activeWindow = ACTIVE_MENU_WINDOW::CHANNEL4_ITEM6;
            calculate_Motor_Parameters(4); 
          break;
        default: // Also for ACTIVE_MENU_ITEM::C4_MM:
            activeWindow = ACTIVE_MENU_WINDOW::MAIN;
            activeItem = ACTIVE_MENU_ITEM::Channel_4;
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
    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM6:
      // Return to main page if clicked on exit
      if (!startStop) {
      }
      // TODO: Test if removing libray-dependent condition works
      else if (steppers.is_running(1 - 1)) {  // startStop = 1 - Stop
        pause_Start_Stop(1);
      }
      else if (steppers.is_paused(1 - 1)) {   // startStop = 1 - Start/Resume
        resume_Start_Stop(1);
      }
      else if (steppers.is_finished(1 - 1)) { // startStop = 1 - Start/Begin
        begin_Start_Stop(1);
      }
      activeWindow = ACTIVE_MENU_WINDOW::CHANNEL1;
      activeItem = ACTIVE_MENU_ITEM::C1_I6;
      startStop = 0;
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
    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM6:
      // Return to main page if clicked on exit
      if (!startStop) {
      }
      // TODO: Test if removing libray-dependent condition works
      else if (steppers.is_running(2 - 1)) {
        pause_Start_Stop(2);
      }
      else if (steppers.is_paused(2 - 1)) {
        resume_Start_Stop(2);
      }
      else if (steppers.is_finished(2 - 1)) {
        begin_Start_Stop(2);
      }
      activeWindow = ACTIVE_MENU_WINDOW::CHANNEL2;
      activeItem = ACTIVE_MENU_ITEM::C2_I6;
      startStop = 0;
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
    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM6:
      // Return to main page if clicked on exit
      if (!startStop) {
      }
      // TODO: Test if removing libray-dependent condition works
      else if (steppers.is_running(3 - 1)) {
        pause_Start_Stop(3);
      }
      else if (steppers.is_paused(3 - 1)) {
        resume_Start_Stop(3);
      }
      else if (steppers.is_finished(3 - 1)) {
        begin_Start_Stop(3);
      }
      activeWindow = ACTIVE_MENU_WINDOW::CHANNEL3;
      activeItem = ACTIVE_MENU_ITEM::C3_I6;
      startStop = 0;
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
    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM6:
      // Return to main page if clicked on exit
      if (!startStop) {
      }
      // TODO: Test if removing libray-dependent condition works
      else if (steppers.is_running(4 - 1)) {
        pause_Start_Stop(4);
      }
      else if (steppers.is_paused(4 - 1)) {
        resume_Start_Stop(4);
      }
      else if (steppers.is_finished(4 - 1)) {
        begin_Start_Stop(4);
      }
      activeWindow = ACTIVE_MENU_WINDOW::CHANNEL4;
      activeItem = ACTIVE_MENU_ITEM::C4_I6;
      startStop = 0;
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
*     Controls the scroll menu's active window ~ Nearly 60 lines of code
*     
*     REQUIRES FOLLOWING ASSUMPTIONS;
*     - ACTIVE_MENU_WINDOW::<CHANNEL1> == ACTIVE_MENU_ITEM::<Channel_1> W.L.O.G.
*     - The MAIN MENU enum in both ACTIVE_MENU_WINDOW and ACTIVE_MENU_ITEM are 0
*     - All channel items are contiguous in their respective enums.
*     - Channel 1 Item 1 starts with value 7, Channel 2 Item 1 starts with value 14, and so forth.
*
*/
// void switch_Scroll_Menu_Optimized(void) {

//   // Determine the channel # and channel item #.
//   short channelNum = (short)((int)activeItem / (double)num_channel_options);  // SHOULD BE BETWEEN 0 and 4 FOR THIS METHOD
//   short channelItemNum = (short)((int)activeItem % num_channel_options);      // Zero Indexed: i.e., item 1 --> 0, item 2 --> 1, etc.

//   if ((int)activeWindow < num_channel_options) {
//     // This is a main menu or channel window.

//     activeWindow = (ACTIVE_MENU_WINDOW)((channelItemNum == 6) ? 0 : (int)activeItem);
//     activeItem = (ACTIVE_MENU_ITEM)((int)activeItem * ((channelNum) ? 1 : num_channel_options));
//     activeItem = (channelItemNum == 6) ? (ACTIVE_MENU_ITEM)channelNum : activeItem;

//     // If the window is the start/stop page, calculate the motor parameters
//     if (channelNum && channelItemNum == 5)
//       calculate_Motor_Parameters(channelNum);
//   }
//   else {
//     // This is a channel item window

//     // If the window is the start/stop page, do custom operation
//     if (channelNum && channelItemNum == 5) {
//       // Return to main page if clicked on exit
//       if (!startStop) {
//       }
//       // TODO: Test if removing libray-dependent condition works
//       else if (steppers.is_running(channelNum - 1)) {  // startStop = 1 - Stop
//         pause_Start_Stop(channelNum);
//       }
//       else if (steppers.is_paused(channelNum - 1)) {   // startStop = 1 - Start/Resume
//         resume_Start_Stop(channelNum);
//       }
//       else if (steppers.is_finished(channelNum - 1)) { // startStop = 1 - Start/Begin
//         begin_Start_Stop(channelNum);
//       }
//       startStop = 0;
//     }

//     // Return to main channel page if done setting syringe end.
//     if (channelItemNum < 4 && !((int)((*channels[channelNum - 1]).rstat) % 1000)) {
//       (*channels[channelNum - 1]).rstat = RES_STATUS::ONES;
//     }
//     else if (channelItemNum < 4) {
//       (*channels[channelNum - 1]).rstat = (RES_STATUS)((*channels[channelNum - 1]).rstat * 10);

//       // Re-print the UI
//       print_Scroll_Menu_Optimized();
//       return;
//     }

//     activeWindow = (ACTIVE_MENU_WINDOW)channelNum;
//     activeItem = (ACTIVE_MENU_ITEM)((channelNum * num_channel_options) + channelItemNum);
//   }

//   // Re-print the UI
//   print_Scroll_Menu_Optimized();
// }

/**
*     Prints the scroll menu ~ Nearly 400 lines of code 
*     
*     (DEPRECATED) 
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

  // Declare variable to track which menu item is highlighted
  int activeItemLoopIndex;

  // Print respective menu page
  switch(activeWindow) { 
    /* CHANNEL MENU WINDOWS*/   
    case ACTIVE_MENU_WINDOW::CHANNEL1:

      // Find the location of the item in the menu matrix
      activeItemLoopIndex = (int)(activeItem) - (int)(ACTIVE_MENU_ITEM::C1_I1) + 4;

      // Print menu header
      print_Menu_Header(1, 0, 0);

      //Print menu window
      print_Menu_Window(channels[0], activeItemLoopIndex, 4, std::size(menu_l));

      break; // End print from Channel 1

    case ACTIVE_MENU_WINDOW::CHANNEL2:

      // Find the location of the item in the menu matrix
      activeItemLoopIndex = (int)(activeItem) - (int)(ACTIVE_MENU_ITEM::C2_I1) + 4;

      // Print menu header
      print_Menu_Header(2, 0, 0);

      //Print menu window
      print_Menu_Window(channels[1], activeItemLoopIndex, 4, std::size(menu_l));

      break; // End print from Channel 2

    case ACTIVE_MENU_WINDOW::CHANNEL3:

      // Find the location of the item in the menu matrix
      activeItemLoopIndex = (int)(activeItem) - (int)(ACTIVE_MENU_ITEM::C3_I1) + 4;

      // Print menu header
      print_Menu_Header(3, 0, 0);

      //Print menu window
      print_Menu_Window(channels[2], activeItemLoopIndex, 4, std::size(menu_l));

      break; // End print from Channel 3

    case ACTIVE_MENU_WINDOW::CHANNEL4:

      // Find the location of the item in the menu matrix
      activeItemLoopIndex = (int)(activeItem) - (int)(ACTIVE_MENU_ITEM::C4_I1) + 4;

      // Print menu header
      print_Menu_Header(4, 0, 0);

      //Print menu window
      print_Menu_Window(channels[3], activeItemLoopIndex, 4, std::size(menu_l));

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
      print_Syringe_Start(1);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM4:

      // Print menu header
      print_Menu_Header(1, 4, 1);
      
      // Print item information
      print_Syringe_End(1);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM5:

      // Print menu header
      print_Menu_Header(1, 5, 1);
     
      // Print item information
      print_Channel_Calibrate(1);

      break;
    case ACTIVE_MENU_WINDOW::CHANNEL1_ITEM6:

      // Print menu header
      print_Menu_Header(1, 6, 1);

      // Print item information
      print_Start_Stop(1);

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
      print_Syringe_Start(2);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM4:

      // Print menu header
      print_Menu_Header(2, 4, 1);

      // Print item information
      print_Syringe_End(2);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM5:

      // Print menu header
      print_Menu_Header(2, 5, 1);

      // Print item information
      print_Channel_Calibrate(2);

      break;
    case ACTIVE_MENU_WINDOW::CHANNEL2_ITEM6:

      // Print menu header
      print_Menu_Header(2, 6, 1);

      // Print item information
      print_Start_Stop(2);

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
      print_Syringe_Start(3);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM4:

      // Print menu header
      print_Menu_Header(3, 4, 1);

      // Print item information
      print_Syringe_End(3);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM5:

      // Print menu header
      print_Menu_Header(3, 5, 1);

      // Print item information
      print_Channel_Calibrate(3);

      break;
    case ACTIVE_MENU_WINDOW::CHANNEL3_ITEM6:

      // Print menu header
      print_Menu_Header(3, 6, 1);

      // Print item information
      print_Start_Stop(3);

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
      print_Syringe_Start(4);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM4:

      // Print menu header
      print_Menu_Header(4, 4, 1);

      // Print item information
      print_Syringe_End(4);

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM5:

      // Print menu header
      print_Menu_Header(4, 5, 1);

      // Print item information
      print_Channel_Calibrate(4);

      break;
    case ACTIVE_MENU_WINDOW::CHANNEL4_ITEM6:

      // Print menu header
      print_Menu_Header(4, 6, 1);

      // Print item information
      print_Start_Stop(4);

      break;
    
    /* DEFAULT/MAIN MENU WINDOW*/
    default:  // Also for ACTIVE_MENU_WINDOW::MAIN:

      // Find the location of the item in the menu matrix
      activeItemLoopIndex = (int)(activeItem);

      // Print menu header
      print_Menu_Header(0, 0, 0);

      //Print menu window
      print_Menu_Window(0, activeItemLoopIndex, 0, 4);

      break; // End print from Main Menu
  }
}

/**
*     Prints the scroll menu ~ Nearly 60 lines of code
*     
*     REQUIRES FOLLOWING ASSUMPTIONS;
*     - ACTIVE_MENU_WINDOW::<CHANNEL1> == ACTIVE_MENU_ITEM::<Channel_1> W.L.O.G.
*     - The MAIN MENU enum in both ACTIVE_MENU_WINDOW and ACTIVE_MENU_ITEM are 0
*     - All channel items are contiguous in their respective enums.
*     - Channel 1 Item 1 starts with value 7, Channel 2 Item 1 starts with value 14, and so forth.
*
*/
// void print_Scroll_Menu_Optimized(void) {

//   // Fill screen with light grey
//   tft.fillScreen(TFT_LIGHTGREY);

//   // Set "cursor" at top left corner of display (0,0) and select font 2
//   // (cursor will move to next line automatically during printing with 'tft.println'
//   //  or stay on the line is there is room for the text with tft.print)
//   tft.setCursor(0, 0, 2);

//   // Set the font colour to be white with a black background
//   tft.setTextColor(TFT_BLACK,TFT_LIGHTGREY);  
//   // Set text size multiplier to 4
//   tft.setTextSize(4);

//   // Determine the channel # and channel item #.
//   short channelNum = (short)((int)activeItem / (double)num_channel_options);  // SHOULD BE BETWEEN 0 and 4 FOR THIS METHOD
//   short channelItemNum = (short)((int)activeItem % num_channel_options);      // Zero Indexed: i.e., item 1 --> 0, item 2 --> 1, etc.

//   // Declare variable to track which menu_l element is highlighted
//   int activeItemLoopIndex;

//   // Determine the type of window 
//   if ((int)activeWindow < num_channel_options) {
//     // This is the main menu or a channel window.

//     // Print menu header
//     print_Menu_Header(channelNum, 0, 0);

//     // Assign activeItemLoopIndex
//     // (Based on main menu vs. channel menu)
//     if ((int)activeWindow) {
//       // Channel menu assignment

//       //Print menu window
//       print_Menu_Window(channels[channelNum - 1], (channelItemNum + 4), 4, std::size(menu_l));
//     }
//     else {
//       // Main menu assignment

//       //Print menu window
//       print_Menu_Window(0, ((int)activeItem - 1), 0, 4);
//     }
//   }
//   else {
//     // This is a channel item window.

//     // Print menu header
//     print_Menu_Header(channelNum, (channelItemNum + 1), 1);

//     // Print Item Information
//     (*print_funcs[channelItemNum])(channelNum);

//   }
// }

/*
*     Prints the proper menu header
*/
void print_Menu_Header(int channelNum, int itemNum, short subheader) {

  // Declare and initialize the subheader item
  String subheader_str = menu_l[itemNum + 3];

  // If the subheader is "Start", but the motor is actually running, change it to "Stop"
  if (channelNum && (subheader_str == "Start") && (*channels[channelNum - 1]).pstat == PUMP_STATUS::RUNNING)
    subheader_str = "Stop";
  
  // Print the rest of the header
  tft.println(" " + menu_l[((!channelNum) ? (std::size(menu_l) - 1) : channelNum - 1)]);  //Print the name of the channel
  if (subheader) {
    tft.setTextSize(3);
    tft.println("  " + subheader_str); // Print the channel's item
  }
  tft.setTextSize(4);
  tft.println("-------------");
  tft.setTextSize(3);
}

/*
*     Prints the proper channel menu window
*/
void print_Menu_Window(Pump_Channel *channel, int activeItemLoopIndex, int lStart, int lEnd) {
      
  String linePrint;

  // Print a smaller menu size for non-main menu windows.
  if (lStart)
    tft.setTextSize(2);

  // Print each current menu option 
  for (int i = lStart; i < lEnd; i++) {
    
    // Grab the item from the menu options
    linePrint = menu_l[i];
    if (channel && linePrint == "Start" && (*channel).pstat == PUMP_STATUS::RUNNING)
      linePrint = "Stop";

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

  tft.setTextSize(3);
}

/* ---------- METHODS FOR PUMP CHANNEL PARAMETER RESOLUTION DISPLAY & CONFIGURATION---------- */

/**
*     Sets the proper parameter resolution setting.
*
*     Returns -1 if the resolution exceeded the lower bound.
*     Returns 1 if the resolution exceeded the upper bound.
*     Returns 2 if attempting to write resolution while motor running.
*     Returns 0 if no bound has been exceeded.
*/
short set_Resolution_Setting(Pump_Channel *channel, double *param, double lower, double upper) {

  // Check if channel pump motor is running
  // TODO: Test if removing libray-dependent condition works
  if (steppers.is_running((*channel).motorNumber - 1))
    return 2;

  // Determine which digit will be manipulated
  float mult = (1.0 / (int)((*channel).rstat));

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

      switch((int)outOfBounds[0]) {

        case 2:   // Concurrent modification error
          tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
          tft.println("ERROR: Cannot change\nvalue while motor is running.");
          tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
          break;

        case 1:   // Out of upper bounds errors
          tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
          if (name != "Start")
            tft.println("ERROR: Cannot exceed\nupper bound of " + String(outOfBounds[1]) + units);
          else
            tft.println("ERROR: Cannot exceed\nSyringe End of " + String(outOfBounds[1]) + units);
          tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
          break;

        case -1:   // Out of lower bounds errors
          tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
          if (name != "End")
            tft.println("ERROR: Cannot exceed\nlowerbound of " + String(outOfBounds[1]) + units);
          else
            tft.println("ERROR: Cannot exceed\nSyringe Start of " + String(outOfBounds[1]) + units);
          tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
          break;
        default:    // Unknown error
          tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
          tft.println("ERROR: Unknown exception thrown.");
          tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
          break;
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
*     Sets the resolution codes of a pump channel
*/
void set_Resolution_Codes(Pump_Channel *channel, double upper, double lower) {

  if ((*channel).resolutionCodes[0] == 1)        // Upper
    (*channel).resolutionCodes[1] = upper;
  else if ((*channel).resolutionCodes[0] == -1)   // Lower
    (*channel).resolutionCodes[1] = lower;
  else if ((*channel).resolutionCodes[0] == 2)    // Attempt to update while running
    (*channel).resolutionCodes[1] = -1.0;
  else                                      // No exceed
    (*channel).resolutionCodes[1] = -1.0;
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

  // Define resolution bounds
  double upper = -1.0;
  double lower = 0.0;

  (*channel).resolutionCodes[0] = set_Resolution_Setting(channel, &(*channel).dosage, lower, upper);

  set_Resolution_Codes(channel, upper, lower);
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

  // Define resolution bounds
  double upper = -1.0;
  double lower = 0.01;

  (*channel).resolutionCodes[0] = set_Resolution_Setting(channel, &(*channel).infusionRate, lower, upper);

  set_Resolution_Codes(channel, upper, lower);
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
void set_Syringe_Start(int channelNum) {

  // Retrieve correct pump
  Pump_Channel *channel = channels[channelNum - 1];

  // Define resolution bounds
  double upper = (*channel).syringeEnd;
  double lower = 0.0;

  (*channel).resolutionCodes[0] = set_Resolution_Setting(channel, &(*channel).syringeStart, lower, upper);

  set_Resolution_Codes(channel, upper, lower);
}

/**
*     Print the proper channel infusion rate
*/
void print_Syringe_Start(int channelNum) {

  // Retrieve pump structure
  Pump_Channel *channel = channels[channelNum - 1];

  print_Resolution_Setting((*channel).rstat, "Start", (*channel).syringeStart, "cm", "  ", (*channel).resolutionCodes);
  reset_Resolution_Codes(channel);
}

/* ---------- METHODS FOR PUMP CHANNEL SYRINGE END & CONFIGURATION---------- */

/**
*     Update the channel's infusion rate.
*/
void set_Syringe_End(int channelNum) {

  // Retrieve correct pump
  Pump_Channel *channel = channels[channelNum - 1];

  // Define resolution bounds
  double upper = 15.25;
  double lower = (*channel).syringeStart;

  (*channel).resolutionCodes[0] = set_Resolution_Setting(channel, &(*channel).syringeEnd, lower, upper);

  set_Resolution_Codes(channel, upper, lower);
}

/**
*     Print the proper channel infusion rate
*/
void print_Syringe_End(int channelNum) {

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

  // Retrieve Pump Channel Structure
  Pump_Channel *channel = channels[motorNum];

  if (dirRE == TURN_DIR::CCW) {
    // TODO: Test if removing libray-dependent condition works
    if (steppers.is_finished(motorNum)) {
      (*channel).pstat = PUMP_STATUS::CALIBRATE;
      set_Stepper_Motor_Direction(channel, TURN_DIR::CCW);
      activate_Stepper_Motor(motorNum, MOTOR_STEPS * MICROSTEPS, 1000);
    }
  }
  else if (dirRE == TURN_DIR::CW) { 
    // TODO: Test if removing libray-dependent condition works
    if (steppers.is_finished(motorNum)) {
      (*channel).pstat = PUMP_STATUS::CALIBRATE;
      set_Stepper_Motor_Direction(channel, TURN_DIR::CW);
      activate_Stepper_Motor(motorNum, MOTOR_STEPS * MICROSTEPS, 1000);
    }
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
  tft.setTextSize(2);
  tft.println("Scroll to Adjust");
  tft.println("");
  tft.println("Click to Exit");
  tft.setTextSize(3);
}

/* ---------- METHODS FOR PUMP CHANNEL START/STOP ACTIONS ---------- */

/**
*     Begin a new pump channel motor mission
*/
void begin_Start_Stop(int channelNum) {

  // Retrieve pump channel
  Pump_Channel *channel = channels[channelNum - 1];

  // Set pump channel status to RUNNING
  (*channel).pstat = PUMP_STATUS::RUNNING;

  // Start the motor
  activate_Stepper_Motor(channelNum - 1, (*channel).stepCount, (*channel).stepDelay);
}

/**
*     Pause a current pump channel motor mission
*/
void pause_Start_Stop(int channelNum) {

  // Retrieve pump channel 
  Pump_Channel *channel = channels[channelNum - 1];

  // Set pump channel status to PAUSED
  (*channel).pstat = PUMP_STATUS::PAUSED;
  
  // TODO: Test if pausing timer works
  
  steppers.pause(channelNum - 1);
  
}

/**
*     Resume a paused pump channel motor mission
*/
void resume_Start_Stop(int channelNum) {
  
  // Retrieve pump channel 
  Pump_Channel *channel = channels[channelNum - 1];

  // Set pump channel status to PAUSED
  (*channel).pstat = PUMP_STATUS::RUNNING;

  // TODO: Test if pausing timer works
  
  steppers.resume(channelNum - 1);  
  
}

/**
*     Calculates the motor parameters of the channel.
*/
void calculate_Motor_Parameters(int channelNum) {

  Pump_Channel *channel = channels[channelNum - 1];

  // Determine the number of steps to achieve the pump distance
  (*channel).stepCount = ((*channel).syringeEnd - (*channel).syringeStart)*(STEP_LENGTH);

  // Determine the step delay 

  // Total Time of procedure (In Hours)
  double totalTime = ((*channel).dosage) / ((*channel).infusionRate);
  totalTime *= 3600;    // Total Time of procedure (In Seconds)
  totalTime *= 1000000; // Total Time of procedure (In Microseconds)
  
  // Step delay
  (*channel).stepDelay = (totalTime) / ((*channel).stepCount - 1);

  //Serial.println("Motor " + String(channelNum) + " | Dosage: " + String((*channel).dosage) + " | Rate: " + String((*channel).infusionRate));
  //Serial.println("Motor " + String(channelNum) + " | Steps: " + String((*channel).stepCount) + " | Delay [us]: " + String((*channel).stepDelay));
}

/**
*     Use the Rotary Encoder to select 
*/
void set_Start_Stop(int channelNum) {

  // If there is rotary encoder movement, update the startStop flag
  if (dirRE != TURN_DIR::STOP) {
    startStop ^= 1;
  }
}

/**
*     Print the Start/Stop information and instructions.
*/
void print_Start_Stop(int channelNum) {

  Pump_Channel *channel = channels[channelNum - 1];

  String cur_action = "Exit";  // The currently selected option 
  String re_action;   // The next option 
  String c_stat;      // The current status of the motor
  String temp_str;

  // TODO: Test if removing libray-dependent condition works
  if (steppers.is_running(channelNum - 1)) {

    c_stat = "Running";
    re_action = "Stop";
  }
  else {
    c_stat = "Stopped";
    re_action = "Start";
  }

  // If startStop = 0, then the main option is EXIT
  // Otherwise, startStop = 1, the the main option is either START or STOP.
  if (!startStop) {
    cur_action = "Exit";
  }
  else {
    temp_str = cur_action;
    cur_action = re_action;
    re_action = temp_str;
  }

  tft.println(" " + String((*channel).stepCount));
  tft.println("   Steps Left");
  tft.setTextSize(2);
  tft.println("   Motor is " + c_stat);
  tft.setTextSize(4);
  tft.println("-------------");
  tft.setTextSize(2);
  tft.print("Current Action: ");
  tft.setTextColor(TFT_BLACK, TFT_SKYBLUE);
  tft.println(cur_action);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.println("Scroll to " + re_action);
  tft.println("Click to Confirm");
  tft.setTextSize(3);
}
#include <ESP32Encoder.h>
#include <MultiStepperLite.h>
#include <SPI.h>
#include <TFT_eSPI.h>

/*      Global Variables      */
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 4

// Thread-Count (Theads per Centimeter)
#define THREAD_COUNT 13

// Linear-Angular-Conversion (Revolution per Thread)
#define LINEAR_ANGULAR_CONVERSION 1

// Angle-Length (Revolutions per Centimeter)
#define ANGLE_LENGTH THREAD_COUNT * LINEAR_ANGULAR_CONVERSION

// Step-Angle (Steps per Revolution)
#define STEP_ANGLE MOTOR_STEPS * MICROSTEPS

// Step-Length (Steps per Centimeter)
#define STEP_LENGTH STEP_ANGLE * ANGLE_LENGTH

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
  STOP        // Not moving
};
TURN_DIR dirRE = TURN_DIR::STOP; 

// Define LCD Object
TFT_eSPI tft = TFT_eSPI();

// Define menu array and metadata
/*
*     -=-IMPORTANT NOTES FOR UPDATING (TODO) -=-:
*       *KEEP THE FIRST FOUR ELEMENTS (0-3) AS THE NAMES OF THE CHANNELS
*       *KEEP THE LAST ELEMENTS (4-...) AS THE NAMES OF THE CHANNEL ITEMS
*           - The very last element MUST BE the Main Menu item.
*           - The names/Strings can be altered, but their placement in the array SHOULD remain the same. 
*              - If modifying the order of the remaining elements, ensure the 'print_funcs' and
*                'set_funcs' function pointer arrays correspond to the correct number of menu items
*                (i.e., num_channel_options - 1) and that the order of items in the 'menu_l' array 
*                matches the order in the two pointer arrays.
*           - The element locations are used to print the names of the menu items, thus allowing customization.
*       *DO NOT REMOVE THE 'TODO' FROM THIS SECTION, IT IDENTIFIES INSTRUCTIONS FOR OPEN-SOURCE UPDATING.
*/
String menu_l[11] = {"Channel 1", "Channel 2", "Channel 3", "Channel 4", "Dosage", "Infusion Rate", "Syringe Start", "Syringe End", "Calibrate", "Start", "Main Menu"};
#define OPTIONS_START_INDEX 4   // This can be thought of as the number of channels, or the index of the first menu item.
short num_channel_options = std::size(menu_l) - OPTIONS_START_INDEX;

// Define the enumerations to track the UI state.
/**
*     -=-IMPORTANT NOTES FOR UPDATING (TODO) -=-: (Notes apply to both 'ACTIVE_MENU_WINDOW' and 'ACTIVE_MENU_ITEM')
*       *THE NUMBERING IN 'ACTIVE_MENU_WINDOW' ENUMERATION MUST MATCH ITS COUNTERPART IN 'ACTIVE_MENU_ITEM'
*           - Do so even if there are menu items without menu windows or if there are menu windows without menu items 
*             (e.g., ACTIVE_MENU_WINDOW::CHANNEL1_MAIN isn't an actual window, but it is needed).
*       *THE VERY FIRST ENUMERATION MEMBER MUST BE A 'MAIN_MENU' WITH A VALUE OF 0.
*           - The values from 0 to 'num_channel_options - 1' are reserved for any main menu options.
*       *THE FIRST ITEM IN EACH CHANNEL's ENUMERATION FAMILY MUST BE '<channelNum> * num_channel_options'.
*           - The remaining enumeration members must be continuous with the starting item. No breaks are allowed.
*       *THE LAST ITEM IN EACH CHANNEL's ENUMERATION FAMILY MUST BE A 'MAIN_MENU' MENU ITEM.
*       *DO NOT REMOVE THE 'TODO' FROM THIS SECTION, IT IDENTIFIES INSTRUCTIONS FOR OPEN-SOURCE UPDATING.
*/
enum ACTIVE_MENU_WINDOW { // Tracks which menu screen to display 
  MAIN,       // Displays the main menu options
  CHANNEL1,   // Displays pump channel 1 menu options
  CHANNEL2,   // Displays pump channel 2 menu options
  CHANNEL3,   // Displays pump channel 3 menu options
  CHANNEL4,   // Displays pump channel 4 menu options

  // Channel 1 Enumeration Family
  CHANNEL1_ITEM1 = 7,  // Channel 1 Item 1  (SET EQUAL TO num_channel_options)
  CHANNEL1_ITEM2,      // Channel 1 Item 2
  CHANNEL1_ITEM3,      // Channel 1 Item 3
  CHANNEL1_ITEM4,      // Channel 1 Item 4
  CHANNEL1_ITEM5,      // Channel 1 Item 5
  CHANNEL1_ITEM6,      // Channel 1 Item 6
  CHANNEL1_MAIN,       // Channel 1 Main Menu

  // Channel 2 Enumeration Family
  CHANNEL2_ITEM1,      // Channel 2 Item 1
  CHANNEL2_ITEM2,      // Channel 2 Item 2
  CHANNEL2_ITEM3,      // Channel 2 Item 3
  CHANNEL2_ITEM4,      // Channel 2 Item 4
  CHANNEL2_ITEM5,      // Channel 2 Item 5
  CHANNEL2_ITEM6,      // Channel 2 Item 6
  ChANNEL2_MAIN,       // Channel 2 Main Menu

  // Channel 3 Enumeration Family
  CHANNEL3_ITEM1,      // Channel 3 Item 1
  CHANNEL3_ITEM2,      // Channel 3 Item 2
  CHANNEL3_ITEM3,      // Channel 3 Item 3
  CHANNEL3_ITEM4,      // Channel 3 Item 4
  CHANNEL3_ITEM5,      // Channel 3 Item 5
  CHANNEL3_ITEM6,      // Channel 3 Item 6
  CHANNEL3_MAIN,       // Channel 3 Main Menu

  // Channel 4 Enumeration Family
  CHANNEL4_ITEM1,      // Channel 4 Item 1
  CHANNEL4_ITEM2,      // Channel 4 Item 2
  CHANNEL4_ITEM3,      // Channel 4 Item 3
  CHANNEL4_ITEM4,      // Channel 4 Item 4
  CHANNEL4_ITEM5,      // Channel 4 Item 5
  CHANNEL4_ITEM6,      // Channel 4 Item 6
  CHANNEL4_MAIN        // Channel 4 Main Menu
};
ACTIVE_MENU_WINDOW activeWindow = ACTIVE_MENU_WINDOW::MAIN;

enum ACTIVE_MENU_ITEM {
  Main,  // Extra piece for simpler enum calculations
  Channel_1,  // Main Menu Channel 1
  Channel_2,  // Main Menu Channel 2
  Channel_3,  // Main Menu Channel 3
  Channel_4,  // Main Menu Channel 4

  // Channel 1 Enumeration Family
  C1_I1 = 7,  // Channel 1 Item 1  (SET EQUAL TO num_channel_options)
  C1_I2,      // Channel 1 Item 2
  C1_I3,      // Channel 1 Item 3
  C1_I4,      // Channel 1 Item 4
  C1_I5,      // Channel 1 Item 5
  C1_I6,      // Channel 1 Item 6
  C1_MM,      // Channel 1 Main Menu

  // Channel 2 Enumeration Family
  C2_I1,      // Channel 2 Item 1
  C2_I2,      // Channel 2 Item 2
  C2_I3,      // Channel 2 Item 3
  C2_I4,      // Channel 2 Item 4
  C2_I5,      // Channel 2 Item 5
  C2_I6,      // Channel 2 ITem 6
  C2_MM,      // Channel 2 Main Menu

  // Channel 3 Enumeration Family
  C3_I1,      // Channel 3 Item 1
  C3_I2,      // Channel 3 Item 2
  C3_I3,      // Channel 3 Item 3
  C3_I4,      // Channel 3 Item 4
  C3_I5,      // Channel 3 Item 5
  C3_I6,      // Channel 3 Item 6
  C3_MM,      // Channel 3 Main Menu

  // Channel 4 Enumeration Family
  C4_I1,      // Channel 4 Item 1
  C4_I2,      // Channel 4 Item 2
  C4_I3,      // Channel 4 Item 3
  C4_I4,      // Channel 4 Item 4
  C4_I5,      // Channel 4 Item 5
  C4_I6,      // Channel 4 Item 6
  C4_MM       // Channel 4 Main Menu
};
ACTIVE_MENU_ITEM activeItem = ACTIVE_MENU_ITEM::Channel_1;

// Define the various LCD pins (Found from User_Setup.h in the TFT_eSPI library folder)
#define TFT_MISO 13
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_CS   10  // Chip select control pin
#define TFT_DC   9  // Data Command control pin
#define TFT_RST  14  // Reset pin (could connect to RST pin)

int test = TOUCH_CS;

// Flags for certain mechanics
short menuOn = 1;
short startStop = 0;   // 0 - Exit, 1 - Start/Stop
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
  double dosage = 0.1;                      // The amount of medicine in either mL or mg/kg
  double infusionRate = 60;              // The rate at which to pump the medcine in either mL/hr or mg/kg/hr
  double syringeStart = 10.0;               // The length marker on the track where the pump will start pushing the syringe.
  double syringeEnd = 12.0;                 // The length marker on the tracker at which the syringe can no longer be pushed.
  unsigned long stepCount = 0;              // The amount of steps this channel needs to take to complete its infusion.
  unsigned long stepDelay = 0;              // The amount of time needed between steps to ensure the correct infusion rate.

  double resolutionCodes[2] = {0.0, 0.0};   // The return code and error when exceeding set parameter bounds.
}Pump_Channel;

// Create instances of the pump channel structure for each channel.
Pump_Channel pumpChannel1, pumpChannel2, pumpChannel3, pumpChannel4;
Pump_Channel *channels[4] = {&pumpChannel1, &pumpChannel2, &pumpChannel3, &pumpChannel4};

// Define function pointer arrays to allow quick menu item action and printing.
/*
*     -=-IMPORTANT NOTES FOR UPDATING (TODO) -=-:
*           *THE SIZE OF BOTH ARRAYS SHOULD BE 'num_channel_options - 1' (i.e., all items but the main menu).
*              - The order of functions in both function pointer arrays SHOULD reflect the order found in the 
*                latter half of the 'menu_l' array.
*       *DO NOT REMOVE THE 'TODO' FROM THIS SECTION, IT IDENTIFIES INSTRUCTIONS FOR OPEN-SOURCE UPDATING.
*/
// Setup an array of pointers to various setting functions. Allows quick and structured access to setting methods.
void set_Channel_Item_1(int channelNum) { set_Channel_Dosage(channelNum); }
void set_Channel_Item_2(int channelNum) { set_Channel_Infusion_Rate(channelNum); }
void set_Channel_Item_3(int channelNum) { set_Syringe_Start(channelNum); }
void set_Channel_Item_4(int channelNum) { set_Syringe_End(channelNum); }
void set_Channel_Item_5(int channelNum) { if (steppers.is_finished(channelNum - 1)) calibrate_Stepper(channelNum - 1); }
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

/* TESTING AUDITORY INFUSION END ALARM */
#define INF_END_ALM 8

void setup() {
  // Allow the correct Serial Baud Rate
  Serial.begin(115200);

  // Initializes subsystems.
  init_Stepper_Motors();
  init_Rotary_Encoder();
  init_LCD_Menu();
  init_Audio_Alarm();

  // Initialize the time test for UI updates
  time_test = 0;

  motorTimer = xTimerCreate("MotorTimer", pdMS_TO_TICKS(1), pdTRUE, 0, motorTimerAction);

  if (motorTimer) {
    Serial.println("RTOS Timer Initialized");
    xTimerStart(motorTimer, 0);
  }
  else
    Serial.println("RTOS Timer Failed to Initialize");

  Serial.print("Booting Firmware on Core: " + String(xPortGetCoreID()) + "\n");
  Serial.println("Optimized Software Booted");
}

void loop() {

  update_All_Channels();

  // Check to see if the menu needs to update windows
  if (curSWCount != prevSWCount) {
    
    // Update the menu if the menu is active
    if (menuOn) {
      if (time_test)
        time_test = millis();
      switch_Scroll_Menu_Optimized();
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
  print_Scroll_Menu_Optimized();
}

/**
*     Initializes the LCD Screen Touch functionality with provided calibration data.
*/
void init_Touch_Screen(void) {
  // Use this calibration code in setup():
  uint16_t calData[5] = { 355, 3461, 437, 3169, 7 };
  tft.setTouch(calData);
}

/**
*     Initializes the Auditory Alarm For Ending An Infusion
*/
void init_Audio_Alarm(void) {
  pinMode(INF_END_ALM, OUTPUT);
  //digitalWrite(INF_END_ALM, LOW);
}

/* ---------- METHODS FOR INFUSION ALARM ---------- */

void alarm_Start(void) {
  //digitalWrite(INF_END_ALM, HIGH);
  tone(INF_END_ALM, 440);
}

void alarm_End(void) {
  noTone(INF_END_ALM);
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

  // Don't do anything if channel is being configured, if it's idle, or if it finished a transfusion.
  if ((*channel).pstat == PUMP_STATUS::CONFIG || (*channel).pstat == PUMP_STATUS::IDLE || (*channel).pstat == PUMP_STATUS::COMPLETE) {
    return;
  }
  // Determine if the channel is idle or complete
  // TODO: Test if removing libray-dependent condition works
  if (steppers.is_finished(channelNum - 1)) {

    if ((*channel).pstat == PUMP_STATUS::RUNNING) {
      (*channel).pstat = PUMP_STATUS::COMPLETE;
      Serial.println("Channel " + String(channelNum) + " Completed");
      print_All_Channels();
      alarm_Start();
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
          perform_Menu_Action_Optimized();
          if (time_test)
            Serial.println("UI Perform Action took " + String(millis() - time_test) + " ms.");
        }

        // Boolean conditions to reduce menu scrolling/refreshing
        short onMainMenus = ((int)(activeWindow) < num_channel_options);                                                                            // Determines if on a main menu window.
        short onCalibrationPage = !(((int)(activeWindow) - (int)(ACTIVE_MENU_WINDOW::CHANNEL1_ITEM5)) % (num_channel_options)) && !onMainMenus;     // Determines if on a calibration page
        short doneWithResSet = (((int)((*channels[0]).rstat) + (int)((*channels[1]).rstat) + (int)((*channels[2]).rstat) + (int)((*channels[3]).rstat)) >= 1000);

        // If the menu is on a non-updating channel item window, do not update the menu. 
        if (!onCalibrationPage && (onMainMenus || !doneWithResSet)) {
          if (time_test)
            time_test = millis();
          update_Scroll_Menu_Optimized();
          print_Scroll_Menu_Optimized();
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
*     Perform channel menu item window action. ~ Nearly 10 lines of code
*     
*     REQUIRES FOLLOWING ASSUMPTIONS;
*     - ACTIVE_MENU_WINDOW::<CHANNEL1> == ACTIVE_MENU_ITEM::<Channel_1> W.L.O.G.
*     - The MAIN MENU enum in both ACTIVE_MENU_WINDOW and ACTIVE_MENU_ITEM are 0
*     - All channel items are contiguous in their respective enums.
*     - Channel 1 Item 1 starts with value 7, Channel 2 Item 1 starts with value 14, and so forth.
*
*/
void perform_Menu_Action_Optimized(void) {

  // Determine the channel # and channel item #.
  short channelNum = (short)((int)activeWindow / (double)num_channel_options);  // SHOULD BE BETWEEN 0 and 4 FOR THIS METHOD
  short channelItemNum = (short)((int)activeWindow % num_channel_options);      // Zero Indexed: i.e., item 1 --> 0, item 2 --> 1, etc.

  // Check if current window allows function to run
  if ((int)activeWindow >= num_channel_options)
    (*set_funcs[channelItemNum])(channelNum);       // Set Item Information
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
void update_Scroll_Menu_Optimized(void) {

  // Determine the channel # and channel item #.
  short channelNum = (short)activeWindow;                                     // SHOULD BE BETWEEN 0 and 4 FOR THIS METHOD
  short channelItemNum = (short)((int)activeItem % num_channel_options);      // Zero Indexed: i.e., item 1 --> 0, item 2 --> 1, etc.

  // Define variable to track next highlighted option (Default values for main menu assignment)
  int activeItemInt = activeItem;
  short activeItemMod = OPTIONS_START_INDEX;

  // Determine the type of window
  if (channelNum > 0 && channelNum < 5) {
    // Channel menu assignment

    // Reassign activeItemMod
    activeItemMod = num_channel_options;

    // Shift the channel item index to 0
    activeItemInt -= (int)(num_channel_options * channelNum);
  }
  else if (!activeWindow) {
    // Main menu assignment 
    activeItemInt--;
  }
  else {
    // Unexpected case
    return;
  }

  if (dirRE == TURN_DIR::CW) {
    activeItemInt++;
    activeItemInt %= activeItemMod;
  }
  else if (dirRE == TURN_DIR::CCW) {
    activeItemInt--;
    activeItemInt = (activeItemInt < 0) ? activeItemInt + activeItemMod : activeItemInt;
  }

  // Cast activeItem back to enumeration
  activeItem = (ACTIVE_MENU_ITEM) (activeItemInt + ((channelNum) ? (int)(num_channel_options * channelNum) : 1));
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
void switch_Scroll_Menu_Optimized(void) {

  // Determine the channel # and channel item #.
  short channelNum = (short)((int)activeItem / (double)num_channel_options);  // SHOULD BE BETWEEN 0 and 4 FOR THIS METHOD
  short channelItemNum = (short)((int)activeItem % num_channel_options);      // Zero Indexed: i.e., item 1 --> 0, item 2 --> 1, etc.

  if ((int)activeWindow < num_channel_options) {
    // This is a main menu or channel window.

    activeWindow = (ACTIVE_MENU_WINDOW)((channelItemNum == (num_channel_options - 1)) ? 0 : (int)activeItem); // Go to main menu if main menu item clicked
    activeItem = (ACTIVE_MENU_ITEM)((int)activeItem * ((channelNum) ? 1 : num_channel_options));              // Update the activeItem when item clicked
    activeItem = (channelItemNum == (num_channel_options - 1)) ? (ACTIVE_MENU_ITEM)channelNum : activeItem;   // Override changes if clicked on main menu item

    // If the window is the start/stop page, calculate the motor parameters
    // TODO: Will need to update the '5' if order or number of menu items changes
    if (channelNum && channelItemNum == 5)
      calculate_Motor_Parameters(channelNum);
  }
  else {
    // This is a channel item window

    // If the window is the start/stop page, do custom operation
    // TODO: Will need to update the '5' if order or number of menu items changes
    if (channelNum && channelItemNum == 5) {
      // Return to main page if clicked on exit
      if (!startStop) {
      }
      // TODO: Test if removing libray-dependent condition works
      else if (steppers.is_running(channelNum - 1)) {  // startStop = 1 - Stop
        pause_Start_Stop(channelNum);
      }
      else if (steppers.is_paused(channelNum - 1)) {   // startStop = 1 - Start/Resume
        resume_Start_Stop(channelNum);
      }
      else if (steppers.is_finished(channelNum - 1)) { // startStop = 1 - Start/Begin
        begin_Start_Stop(channelNum);
      }
      startStop = 0;
    }

    // Return to main channel page if done setting syringe end.
    // TODO: Will need to update the '5's if order or number of menu items changes
    if (channelItemNum < 5 && !((int)((*channels[channelNum - 1]).rstat) % 1000)) {
      (*channels[channelNum - 1]).rstat = RES_STATUS::ONES;
    }
    else if (channelItemNum < 5) {
      (*channels[channelNum - 1]).rstat = (RES_STATUS)((*channels[channelNum - 1]).rstat * 10);

      // Re-print the UI
      print_Scroll_Menu_Optimized();
      return;
    }

    activeWindow = (ACTIVE_MENU_WINDOW)channelNum;
    activeItem = (ACTIVE_MENU_ITEM)((channelNum * num_channel_options) + channelItemNum);
  }

  // Re-print the UI
  print_Scroll_Menu_Optimized();
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
void print_Scroll_Menu_Optimized(void) {

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

  // Determine the channel # and channel item #.
  short channelNum = (short)((int)activeItem / (double)num_channel_options);  // SHOULD BE BETWEEN 0 and 4 FOR THIS METHOD
  short channelItemNum = (short)((int)activeItem % num_channel_options);      // Zero Indexed: i.e., item 1 --> 0, item 2 --> 1, etc.

  // Declare variable to track which menu_l element is highlighted
  int activeItemLoopIndex;

  // Determine the type of window 
  if ((int)activeWindow < num_channel_options) {
    // This is the main menu or a channel window.

    // Print menu header
    print_Menu_Header(channelNum, 0, 0);

    // Assign activeItemLoopIndex
    // (Based on main menu vs. channel menu)
    if ((int)activeWindow) {
      // Channel menu assignment

      //Print menu window
      print_Menu_Window(channels[channelNum - 1], (channelItemNum + OPTIONS_START_INDEX), OPTIONS_START_INDEX, std::size(menu_l));
    }
    else {
      // Main menu assignment

      //Print menu window
      print_Menu_Window(0, ((int)activeItem - 1), 0, OPTIONS_START_INDEX);
    }
  }
  else {
    // This is a channel item window.

    // Print menu header
    print_Menu_Header(channelNum, (channelItemNum + 1), 1);

    // Print Item Information
    (*print_funcs[channelItemNum])(channelNum);

  }
}

/*
*     Prints the proper menu header
*/
void print_Menu_Header(int channelNum, int itemNum, short subheader) {

  // Declare and initialize the subheader item
  String subheader_str = menu_l[itemNum + (OPTIONS_START_INDEX - 1)];

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
    else if (channel && linePrint == "Start" && (*channel).pstat == PUMP_STATUS::COMPLETE)
      linePrint = "Reset";

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

  // Check if the motor is currently running
  if ((*channel).pstat == PUMP_STATUS::RUNNING)
    return;

  // Determine the amount of calibration movement 
  // Highest - 10 Revolution per Turn
  // Medium  - 1 Revolution per Turn
  // Lowest  - 0.1 Revolution per Turn
  float mult = (10.0 / (int)((*channel).rstat));

  if (dirRE == TURN_DIR::CCW) {
    // TODO: Test if removing libray-dependent condition works
    if (steppers.is_finished(motorNum)) {
      (*channel).pstat = PUMP_STATUS::CALIBRATE;
      set_Stepper_Motor_Direction(channel, TURN_DIR::CCW);
      activate_Stepper_Motor(motorNum, MOTOR_STEPS * MICROSTEPS * mult, 100);
    }
  }
  else if (dirRE == TURN_DIR::CW) { 
    // TODO: Test if removing libray-dependent condition works
    if (steppers.is_finished(motorNum)) {
      (*channel).pstat = PUMP_STATUS::CALIBRATE;
      set_Stepper_Motor_Direction(channel, TURN_DIR::CW);
      activate_Stepper_Motor(motorNum, MOTOR_STEPS * MICROSTEPS * mult, 100);
    }
  }
}

/**
*    Print the calibration instructions.
*/
void print_Channel_Calibrate(int channelNum) {

  tft.setTextSize(2);
  tft.println("Rev/Turn: " + String(10.0 / (*channels[channelNum - 1]).rstat));
  tft.println("CW -> Push");
  tft.println("CCW -> Pull");
  tft.setTextSize(4);
  tft.println("-------------");
  tft.setTextSize(2);
  tft.println("Scroll to Adjust");
  tft.println("");
  if (!((*channels[channelNum - 1]).rstat % 1000))
    tft.println("Click to Exit");
  else
    tft.println("Click to alter Rev/Turn");
  tft.setTextSize(3);
}

/* ---------- METHODS FOR PUMP CHANNEL START/STOP ACTIONS ---------- */

/**
*     Begin a new pump channel motor mission
*/
void begin_Start_Stop(int channelNum) {

  // Retrieve pump channel
  Pump_Channel *channel = channels[channelNum - 1];

  // If an infusion has just ended, update accordingly
  if ((*channel).pstat == PUMP_STATUS::COMPLETE) {
    alarm_End();
    (*channel).pstat = PUMP_STATUS::IDLE;
    return;
  }

  // Set pump channel status to RUNNING
  (*channel).pstat = PUMP_STATUS::RUNNING;

  // Start the motor - Ensures that the syringe is being pushed.
  set_Stepper_Motor_Direction(channel, TURN_DIR::CW);
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
  else if ((*channel).pstat == PUMP_STATUS::COMPLETE) {
    c_stat = "Complete";
    re_action = "Reset";
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
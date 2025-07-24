#include <ESP32Encoder.h>
#include <MultiStepperLite.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <TFT_eWidget.h>

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

// Board time since previous touch button check
unsigned long prevTouchTime = 0;
unsigned long debounceTouch = 250;    // Keep larger than 250ms (Avg Human Reaction time)

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
*              - If modifying the order of the remaining elements, ensure the 'setup_item_funcs' and
*                'update_item__funcs' function pointer arrays correspond to the correct number of menu items
*                (i.e., num_channel_options - 1) and that the order of items in the 'menu_l' array 
*                matches the order in the two pointer arrays.
*           - The element locations are used to print the names of the menu items, thus allowing customization.
*       *DO NOT REMOVE THE 'TODO' FROM THIS SECTION, IT IDENTIFIES INSTRUCTIONS FOR OPEN-SOURCE UPDATING.
*/
char  *menu_l[11] = {"Channel 1", "Channel 2", "Channel 3", "Channel 4", "Dosage", "Infusion Rate", "Syringe Start", "Syringe End", "Calibrate", "Start", "Main Menu"};
#define OPTIONS_START_INDEX 4   // This can be thought of as the number of channels, or the index of the first menu item.
short num_channel_options = std::size(menu_l) - OPTIONS_START_INDEX;

// Define the button widgets
// Buttons used within the main menu window;
ButtonWidget channel1Btn = ButtonWidget(&tft);
ButtonWidget channel2Btn = ButtonWidget(&tft);
ButtonWidget channel3Btn = ButtonWidget(&tft);
ButtonWidget channel4Btn = ButtonWidget(&tft);

// Buttons used within a channel menu window.
ButtonWidget dosageBtn = ButtonWidget(&tft);
ButtonWidget infusionRateBtn = ButtonWidget(&tft);
ButtonWidget syringeStartBtn = ButtonWidget(&tft);
ButtonWidget syringeEndBtn = ButtonWidget(&tft);
ButtonWidget calibrateBtn = ButtonWidget(&tft);
ButtonWidget startStopBtn = ButtonWidget(&tft);
ButtonWidget mainMenuBtn = ButtonWidget(&tft);

// Buttons used within a channel item's window.
ButtonWidget firstResBtn = ButtonWidget(&tft);
ButtonWidget secondResBtn = ButtonWidget(&tft);
ButtonWidget thirdResBtn = ButtonWidget(&tft);
ButtonWidget exitItemBtn = ButtonWidget(&tft);

// Create arrays of button instances to utilize in print loops.
ButtonWidget* main_b[] = {&channel1Btn, &channel2Btn, &channel3Btn, &channel4Btn};
ButtonWidget* channel_b[] = {&dosageBtn, &infusionRateBtn, &syringeStartBtn, &syringeEndBtn, &calibrateBtn, &startStopBtn, &mainMenuBtn};
ButtonWidget* item_b[] = {&firstResBtn, &secondResBtn, &thirdResBtn, &exitItemBtn};

// Calculate the size of each array.
uint8_t main_b_size = sizeof(main_b) / sizeof(main_b[0]);
uint8_t channel_b_size = sizeof(channel_b) / sizeof(channel_b[0]);
uint8_t item_b_size = sizeof(item_b) / sizeof(item_b[0]);

// Useful macro for calculating the height of buttons and spaces between buttons.
#define BUTTON_WIDTH(num_buttons, total_height) ((0.75 * total_height) / (num_buttons))
#define SPACE_WIDTH(num_buttons, total_height) ((0.25 * total_height) / (num_buttons + 1))

// Define the various LCD pins (Found from User_Setup.h in the TFT_eSPI library folder)
#define TFT_MISO 13
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_CS   10   // Chip select control pin
#define TFT_DC   9    // Data Command control pin
#define TFT_RST  14   // Reset pin (could connect to RST pin)

#define TOUCH_CS 3

// Flags for certain mechanics
short onChannelItem = 0;
short startStop = 0;   // 0 - Exit, 1 - Start/Stop
short curSWCount = 0;  // Tells Rotary Encoder when to switch menu windows
short prevSWCount = 0; 
int x_default = 0, y_default = 0;

/*  Determines which channel the current menu window belongs to.
*   
*   Is used to pass in parameters to button functions.
*/
enum SELECTED_CHANNEL_WINDOW {
  OTHER_MENU,
  CHANNEL_1,
  CHANNEL_2,
  CHANNEL_3,
  CHANNEL_4
};
// A variable to track which channel the current window belongs to.
short activeChannel = (short)(SELECTED_CHANNEL_WINDOW::OTHER_MENU);

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

// Structure that will store each channel's configuration
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
void setup_Main_Menu(void) {
  onChannelItem = 0;
  activeChannel = 0;
  print_Menu_Header(activeChannel, 0, 0, "  ");
  print_Menu_Window(activeChannel);
}
void setup_Channel_1(void) {
  onChannelItem = 0;
  activeChannel = 1;
  if (activeChannel) {
    print_Menu_Header(activeChannel, 0, 0, "   ");
    print_Menu_Window(activeChannel);
  }
}
void setup_Channel_2(void) {
  onChannelItem = 0;
  activeChannel = 2;
  if (activeChannel) {
    print_Menu_Header(activeChannel, 0, 0, "   ");
    print_Menu_Window(activeChannel);
  }
}
void setup_Channel_3(void) {
  onChannelItem = 0;
  activeChannel = 3;
  if (activeChannel) {
    print_Menu_Header(activeChannel, 0, 0, "   ");
    print_Menu_Window(activeChannel);
  }
}
void setup_Channel_4(void) {
  onChannelItem = 0;
  activeChannel = 4;
  if (activeChannel) {
    print_Menu_Header(activeChannel, 0, 0, "   ");
    print_Menu_Window(activeChannel);
  }
}

void (*setup_channel_funcs[])(void) = {setup_Channel_1, setup_Channel_2, setup_Channel_3, setup_Channel_4};

// Setup an array of pointers to various setting functions. Allows quick and structured access to setting methods.
void update_Channel_Item_1(void) { if (activeChannel) set_Channel_Dosage(activeChannel); }
void update_Channel_Item_2(void) { if (activeChannel) set_Channel_Infusion_Rate(activeChannel); }
void update_Channel_Item_3(void) { if (activeChannel) set_Syringe_Start(activeChannel); }
void update_Channel_Item_4(void) { if (activeChannel) set_Syringe_End(activeChannel); }
void update_Channel_Item_5(void) { if (activeChannel && steppers.is_finished(activeChannel - 1)) calibrate_Stepper(activeChannel - 1); }
void update_Channel_Item_6(void) { if (activeChannel) set_Start_Stop(activeChannel); }

void (*update_item_funcs[])(void) = {update_Channel_Item_1, update_Channel_Item_2, update_Channel_Item_3, update_Channel_Item_4, update_Channel_Item_5, update_Channel_Item_6};

// Setup an array of pointers to various printing functions. Allows quick and structured access to printing methods.
void setup_Channel_Item_1(void) { 
  if (activeChannel && !onChannelItem) {
    onChannelItem = 1;
    print_Menu_Header(activeChannel, 1, 1, "   ");
    print_Channel_Dosage(activeChannel);
  } 
}
void setup_Channel_Item_2(void) { 
  if (activeChannel && !onChannelItem) {
    onChannelItem = 1;
    print_Menu_Header(activeChannel, 2, 1, "   ");
    print_Channel_Infusion_Rate(activeChannel); 
  }
}
void setup_Channel_Item_3(void) { 
  if (activeChannel && !onChannelItem) {
    onChannelItem = 1;
    print_Menu_Header(activeChannel, 3, 1, "   ");
    print_Syringe_Start(activeChannel); 
  }
}
void setup_Channel_Item_4(void) { 
  if (activeChannel && !onChannelItem) {
    onChannelItem = 1;
    print_Menu_Header(activeChannel, 4, 1, "   ");
    print_Syringe_End(activeChannel); 
  }
}
void setup_Channel_Item_5(void) { 
  if (activeChannel && !onChannelItem) {
    onChannelItem = 1;
    print_Menu_Header(activeChannel, 5, 1, "   ");
    print_Channel_Calibrate(activeChannel); 
  }
}
void setup_Channel_Item_6(void) { 
  if (activeChannel && !onChannelItem) {
    onChannelItem = 1;
    print_Menu_Header(activeChannel, 6, 1, "   ");
    print_Start_Stop(activeChannel); 
  }
}
void setup_Channel_Item_MM(void) {
  if (!onChannelItem)
    setup_Main_Menu();
}

void (*setup_item_funcs[])(void) = {setup_Channel_Item_1, setup_Channel_Item_2, setup_Channel_Item_3, setup_Channel_Item_4, setup_Channel_Item_5, setup_Channel_Item_6, setup_Channel_Item_MM};

// Determines the actions to take given a touch occuring at coordinates x and y.
void check_Touch_Buttons(void) {

  uint16_t x = 9999, y = 9999; // To store the touch coordinates

  // Pressed will be set true if there is a valid touch on the screen
  bool pressed = tft.getTouch(&x, &y);

  if (activeChannel) {
    // Within a channel Menu
    for (int i = 0; i < channel_b_size; i++) {
      if (pressed) {
        if (!onChannelItem && (*channel_b[i]).contains(y, tft.height() - x - 105)) {
          (*channel_b[i]).press(true);
          (*channel_b[i]).pressAction();
          Serial.println("You pressed channel item button: " + String(i + 1));
          break;
        }
        else if (onChannelItem) {
          Serial.println("You clicked, but you're already on a channel item.");
          (*setup_channel_funcs[i])();
          break;
        }
      }
      else {
        (*channel_b[i]).press(false);
      }
    }
  }
  else {
    // Within the Main Menu
    for (int i = 0; i < main_b_size; i++) {
      if (pressed) {
        if ((*main_b[i]).contains(y, tft.height() - x - 70)) {
          (*main_b[i]).press(true);
          (*main_b[i]).pressAction();
          Serial.println("You pressed channel button: " + String(i + 1));
          break;
        }
      }
      else {
        (*main_b[i]).press(false);
      }
    }
  }
}

// Declare RTOS timer
TimerHandle_t motorTimer;

void motorTimerAction(TimerHandle_t xTimer) {
  steppers_do_tasks();
}


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

  // Set the default LCD cursor coordinate.
  x_default = tft.getCursorX();
  y_default = tft.getCursorY();

  // Initialize the Stepper Motor timer
  motorTimer = xTimerCreate("MotorTimer", pdMS_TO_TICKS(1), pdTRUE, 0, motorTimerAction);

  if (motorTimer) {
    Serial.println("RTOS Timer Initialized");
    xTimerStart(motorTimer, 0);
  }
  else
    Serial.println("RTOS Timer Failed to Initialize");

  Serial.print("Booting Firmware on Core: " + String(xPortGetCoreID()) + "\n");
  Serial.println("Optimized Software Booted");

  Serial.println("Dimensions (" + String(tft.width()) + ", " + String(tft.height()) + ")");
  setup_Main_Menu();
}

void loop() {

  update_All_Channels();

  // Scan keys every 50ms at most
  if (millis() - prevTouchTime >= debounceTouch) {

    prevTouchTime = millis();
    check_Touch_Buttons();
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
  //init_Touch_Screen();
  init_Touch_Buttons();
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
*     Initializes the actions and graphics for the touch buttons.
*/
void init_Touch_Buttons(void) {
  Serial.println("Width: " + String(tft.width()) + " | Height: " + String(tft.height()));

  uint16_t x = 70; // End of Menu Header
  uint16_t y = tft.width() / 2;   // For centralized buttons

  uint16_t buttonHeight = y;

  uint16_t buttonWidth = BUTTON_WIDTH(main_b_size, tft.height() - x);
  uint16_t spaceWidth = SPACE_WIDTH(main_b_size, tft.height() - x);
  x += (buttonWidth);

  // Set the setup function for each main or channel menu button.
  for (int i = 0; i < main_b_size; i++) {
    (*main_b[i]).initButton(y, x, buttonHeight, buttonWidth, TFT_BLACK, TFT_SKYBLUE, TFT_BLACK, menu_l[i], 1);
    (*main_b[i]).setPressAction(*(setup_channel_funcs + i));
    x += (buttonWidth + spaceWidth);
  } 

  // TODO: Complete this section of the channel setup.
  x = 70;

  buttonWidth = BUTTON_WIDTH(channel_b_size, tft.height() - x);
  spaceWidth = SPACE_WIDTH(channel_b_size, tft.height() - x);
  x += (buttonWidth);

  // Set the print function for each channel item button.
  for (int i = 0 ; i < channel_b_size; i++) {
    (*channel_b[i]).initButton(y, x, buttonHeight, buttonWidth, TFT_BLACK, TFT_SKYBLUE, TFT_BLACK, menu_l[i + OPTIONS_START_INDEX], 1);
    (*channel_b[i]).setPressAction(*(setup_item_funcs + i));
    x += (buttonWidth + spaceWidth);
  }
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

      // Because touch screen is implemented, there is no need
      // to update the menu from the Rotary Encoder

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

/*
*     Prints the proper menu header
*/
void print_Menu_Header(int channelNum, int itemNum, short subheader, String preSpace) {

  // Reset the screen to load a new window
  tft.fillScreen(TFT_WHITE);
  tft.setCursor(x_default, y_default);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);

  tft.setTextSize(1);
  tft.println();

  // Declare and initialize the subheader item
  String subheader_str = menu_l[itemNum + (OPTIONS_START_INDEX - 1)];

  // If the subheader is "Start", but the motor is actually running, change it to "Stop"
  if (channelNum && (subheader_str == "Start") && (*channels[channelNum - 1]).pstat == PUMP_STATUS::RUNNING)
    subheader_str = "Stop";
  
  // Print the rest of the header
  String menu_str(menu_l[((!channelNum) ? (std::size(menu_l) - 1) : channelNum - 1)]);
  tft.setTextSize(4);
  tft.println(preSpace + menu_str);  //Print the name of the channel
  if (subheader) {
    tft.setTextSize(3);
    tft.println("  " + subheader_str); // Print the channel's item
  }
  tft.setTextSize(4);
  tft.println("-------------");
  tft.setTextSize(3);
}

/**
*     Prints the proper menu window.
*/
void print_Menu_Window(int channelNum) {

  if (channelNum) {
    // Channel menu window.

    for (int i = 0; i < channel_b_size; i++) 
      (*channel_b[i]).drawSmoothButton(false, 3, TFT_BLACK);
  }
  else {
    // Main menu window.

    for (int i = 0; i < main_b_size; i++) 
      (*main_b[i]).drawSmoothButton(false, 3, TFT_BLACK);

  }
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
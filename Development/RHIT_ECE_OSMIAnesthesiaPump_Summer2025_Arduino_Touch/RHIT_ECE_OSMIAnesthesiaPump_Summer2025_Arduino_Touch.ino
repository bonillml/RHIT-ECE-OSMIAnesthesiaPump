#include <ESP32Encoder.h>
#include <MultiStepperLite.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <TFT_eWidget.h>
#include <FS.h>
#include <HX711.h>

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
unsigned long debounceTouch = 125;    // Keep larger than 250ms (Avg Human Reaction time)

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

#define CALIBRATION_FILE "/TouchCalData1"
#define REPEAT_CAL false
#define ROTATION 2

// Define the various LCD pins (Found from User_Setup.h in the TFT_eSPI library folder)
#define TFT_MISO 13
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_CS   10   // Chip select control pin
#define TFT_DC   9    // Data Command control pin
#define TFT_RST  14   // Reset pin (could connect to RST pin)

#define TOUCH_CS 3

// Flags for certain mechanics
short startStop = 0;   // 0 - Exit, 1 - Start/Stop
short curSWCount = 0;  // Tells Rotary Encoder when to switch menu windows
short prevSWCount = 0; 
int x_default = 0, y_default = 0;

// A variable to track which channel the current window belongs to.
short activeChannel = 0;

// A variable to track which channel item the current window belongs to.
short activeChannelItem = 0;

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
  HUNDREDTHS = 100
};

// Structure that will store each channel's configuration
typedef struct PumpChannel {
  unsigned short motorNumber = 0;           // The motor number of this channel
  unsigned short directionPin = 0;          // The direction pin number of this channel.
  PUMP_STATUS pstat = PUMP_STATUS::IDLE;    // The status of the pump channel.
  RES_STATUS rstat = RES_STATUS::ONES;      // The current resolution digit for inputs.
  double dosage = 10.0;                      // The amount of medicine in either mL or mg/kg
  double infusionRate = 5.0;              // The rate at which to pump the medcine in either mL/hr or mg/kg/hr
  double syringeStart = 10.0;               // The length marker on the track where the pump will start pushing the syringe.
  double syringeEnd = 15.0;                 // The length marker on the tracker at which the syringe can no longer be pushed.
  unsigned long stepCount = 0;              // The amount of steps this channel needs to take to complete its infusion.
  unsigned long stepDelay = 0;              // The amount of time needed between steps to ensure the correct infusion rate.

  double resolutionCodes[2] = {0.0, 0.0};   // The return code and error when exceeding set parameter bounds.
}Pump_Channel;

// Create instances of the pump channel structure for each channel.
Pump_Channel pumpChannel1, pumpChannel2, pumpChannel3, pumpChannel4;
Pump_Channel *channels[4] = {&pumpChannel1, &pumpChannel2, &pumpChannel3, &pumpChannel4};

/*
*     -=-IMPORTANT NOTES FOR UPDATING (TODO) -=-:
*           *THE SIZE OF BOTH ARRAYS SHOULD BE 'num_channel_options - 1' (i.e., all items but the main menu).
*              - The order of functions in both function pointer arrays SHOULD reflect the order found in the 
*                latter half of the 'menu_l' array.
*       *DO NOT REMOVE THE 'TODO' FROM THIS SECTION, IT IDENTIFIES INSTRUCTIONS FOR OPEN-SOURCE UPDATING.
*/

// Define the structure for a Channel Item
typedef struct ButtonItem {

  char *label;
  ButtonWidget btn = ButtonWidget(&tft);
  void (*setup_func)() = NULL;
  void (*update_func)() = NULL;

  // Constructors for buttons with only setup functions.
  ButtonItem(char *lbl, void (*setup)()) {
    label = lbl;
    setup_func = setup;
  }

  // Constructors for buttons with both setup and update functions.
  ButtonItem(char *lbl, void (*setup)(), void (*update)()) {
    label = lbl;
    setup_func = setup;
    update_func = update;
  }

}Button_Item;

void setup_Main_Menu(void) {
  activeChannelItem = 0;
  activeChannel = 0;
  print_Menu_Header(activeChannel, 0, 0, "  ");
  print_Menu_Window(activeChannel);
}
void setup_Channel_1(void) {
  activeChannelItem = 0;
  activeChannel = 1;
  if (activeChannel) {
    print_Menu_Header(activeChannel, 0, 0, "   ");
    print_Menu_Window(activeChannel);
  }
}
void setup_Channel_2(void) {
  activeChannelItem = 0;
  activeChannel = 2;
  if (activeChannel) {
    print_Menu_Header(activeChannel, 0, 0, "   ");
    print_Menu_Window(activeChannel);
  }
}
void setup_Channel_3(void) {
  activeChannelItem = 0;
  activeChannel = 3;
  if (activeChannel) {
    print_Menu_Header(activeChannel, 0, 0, "   ");
    print_Menu_Window(activeChannel);
  }
}
void setup_Channel_4(void) {
  activeChannelItem = 0;
  activeChannel = 4;
  if (activeChannel) {
    print_Menu_Header(activeChannel, 0, 0, "   ");
    print_Menu_Window(activeChannel);
  }
}

void update_Channel_Item_1(void) { 
  if (activeChannel) 
    set_Channel_Dosage(activeChannel); 
}
void update_Channel_Item_2(void) { 
  if (activeChannel) 
    set_Channel_Infusion_Rate(activeChannel); 
}
void update_Channel_Item_3(void) { 
  if (activeChannel) 
    set_Syringe_Start(activeChannel); 
}
void update_Channel_Item_4(void) { 
  if (activeChannel) 
    set_Syringe_End(activeChannel); 
}
void update_Channel_Item_5(void) { 
  if (activeChannel && steppers.is_finished(activeChannel - 1)) 
    calibrate_Stepper(activeChannel - 1); 
}
void update_Channel_Item_6(void) { 
  if (activeChannel) 
    set_Start_Stop(activeChannel); 
}

void setup_Channel_Item_1(void) { 
  if (activeChannel && (!activeChannelItem || activeChannelItem == 1)) {
    activeChannelItem = 1;
    print_Menu_Header(activeChannel, 1, 1, "   ");
    print_Channel_Dosage(activeChannel);
  } 
}
void setup_Channel_Item_2(void) { 
  if (activeChannel && (!activeChannelItem || activeChannelItem == 2)) {
    activeChannelItem = 2;
    print_Menu_Header(activeChannel, 2, 1, "   ");
    print_Channel_Infusion_Rate(activeChannel); 
  }
}
void setup_Channel_Item_3(void) { 
  if (activeChannel && (!activeChannelItem || activeChannelItem == 3)) {
    activeChannelItem = 3;
    print_Menu_Header(activeChannel, 3, 1, "   ");
    print_Syringe_Start(activeChannel); 
  }
}
void setup_Channel_Item_4(void) { 
  if (activeChannel && (!activeChannelItem || activeChannelItem == 4)) {
    activeChannelItem = 4;
    print_Menu_Header(activeChannel, 4, 1, "   ");
    print_Syringe_End(activeChannel); 
  }
}
void setup_Channel_Item_5(void) { 
  if (activeChannel && (!activeChannelItem || activeChannelItem == 5)) {
    activeChannelItem = 5;                              //TODO: Change to reflect actual channel item number.
    print_Menu_Header(activeChannel, 5, 1, "   ");  //TODO: Change second parameter ('4') to reflect actual channel item number.
    print_Channel_Calibrate(activeChannel); 
  }
}
void setup_Channel_Item_6(void) { 
  if (activeChannel && (!activeChannelItem || activeChannelItem == 6)) {
    activeChannelItem = 6;
    print_Menu_Header(activeChannel, 6, 1, "   ");
    print_Start_Stop(activeChannel, (*channels[activeChannel - 1]).resolutionCodes[0]); 
  }
}
void setup_Channel_Item_MM(void) {
  if (activeChannel && !activeChannelItem)
    setup_Main_Menu();
}

void cycle_Channel_Item_Resolution(void) {
  if (activeChannel && activeChannelItem)
    cycle_Channel_Resolution(activeChannel);
}
void retare_Channel_Item_Scale(void) {
  retare_Scale();
}
void clear_Channel_Item_Value(void) {
  if (activeChannel && activeChannelItem) 
    clear_Channel_Resolution(activeChannel, activeChannelItem);
}
void start_Stop_Channel_Item(void) {
  if (activeChannel && activeChannelItem)
    begin_Start_Stop(activeChannel);
    return;
}
void calculate_Channel_Item_Steps(void) {
  if (activeChannel && activeChannelItem) {
    calculate_Motor_Parameters(activeChannel);
    setup_Channel_Item_6();
    (*channels[activeChannel - 1]).resolutionCodes[0] = 0.0;
  }
}
void exit_Channel_Item(void) {
  if (activeChannel && activeChannelItem)
    exit_Item_Window(activeChannel);
}

// ButtonItem Stuctures used within the main menu window;
Button_Item channel1BI("Channel 1", setup_Channel_1); 
Button_Item channel2BI("Channel 2", setup_Channel_2); 
Button_Item channel3BI("Channel 3", setup_Channel_3); 
Button_Item channel4BI("Channel 4", setup_Channel_4); 

// ButtonItem Structures used within a channel menu window.
Button_Item dosageBI("Dosage", setup_Channel_Item_1, update_Channel_Item_1);
Button_Item infusionRateBI("Infusion Rate", setup_Channel_Item_2, update_Channel_Item_2);
Button_Item syringeStartBI("Syringe Start", setup_Channel_Item_3, update_Channel_Item_3);
Button_Item syringeEndBI("Syringe End", setup_Channel_Item_4, update_Channel_Item_4);
Button_Item calibrateBI("Calibrate", setup_Channel_Item_5, update_Channel_Item_5);
Button_Item startStopBI("Start", setup_Channel_Item_6, update_Channel_Item_6);
Button_Item mainMenuBI("Main Menu", setup_Channel_Item_MM);

// ButtonItem Structures used within a channel item's window.
Button_Item resolutionBI("Resolution", cycle_Channel_Item_Resolution);
Button_Item tareLoadCellBI("Re-Scale", retare_Channel_Item_Scale);
Button_Item clearItemBI("Clear", clear_Channel_Item_Value);
Button_Item startStopMotorBI("Start-Stop", start_Stop_Channel_Item);
Button_Item calculateStepsBI("Calculate", calculate_Channel_Item_Steps);
Button_Item exitItemBI("Exit Page", exit_Channel_Item);

// Create arrays of ButtonItem Structures
Button_Item *main_b[] = {&channel1BI, &channel2BI, &channel3BI, &channel4BI};
Button_Item *channel_b[] = {&dosageBI, &infusionRateBI, &syringeStartBI, &syringeEndBI, &calibrateBI, &startStopBI, &mainMenuBI};
Button_Item *item_b[][3] = {
                           {&resolutionBI, &clearItemBI, &exitItemBI},          // Dosage Page
                           {&resolutionBI, &clearItemBI, &exitItemBI},          // Infusion Rate Page
                           {&resolutionBI, &clearItemBI, &exitItemBI},          // Syringe Start Page
                           {&resolutionBI, &clearItemBI, &exitItemBI},          // Syringe End Page
                           {&resolutionBI, &tareLoadCellBI, &exitItemBI},       // Calibrate Page
                           {&startStopMotorBI, &calculateStepsBI, &exitItemBI}, // Start/Stop Page
                          };

// Calculate the size of each array.
uint8_t main_b_size = sizeof(main_b) / sizeof(main_b[0]);
uint8_t channel_b_size = sizeof(channel_b) / sizeof(channel_b[0]);
uint8_t item_b_size = sizeof(item_b) / sizeof(item_b[0]);
uint8_t item_bl_size = 3;

// Determines the actions to take given a touch occuring at coordinates x and y.
void check_Touch_Buttons(void) {

  uint16_t x = 9999, y = 9999; // To store the touch coordinates

  // Pressed will be set true if there is a valid touch on the screen
  bool pressed = tft.getTouch(&x, &y);

  Button_Item *BI = NULL;

  switch (ROTATION) {
    case 0:
      break;
    case 1:
      break;
    case 2:
      y = tft.height() - y;
      break;
    case 3:
      break;
    default:
      break;
  }

  if (activeChannel && !activeChannelItem) {
    // Within a channel Menu
    for (int i = 0; i < channel_b_size; i++) {

      BI = channel_b[i];

      if (pressed) {
        if ((*BI).btn.contains(x, y)) {
          (*BI).btn.press(true);
          (*BI).btn.pressAction();
          Serial.println("You pressed channel item button: " + String(i + 1));
          break;
        }
      }
      else {
        (*BI).btn.press(false);
      }
    }
  }
  else if (activeChannel && activeChannelItem) {
    // Within a channel item window.
    for (int i = 0; i < item_bl_size; i++) {

      BI = item_b[activeChannelItem - 1][i];

      if (pressed) {
        if ((*BI).btn.contains(x, y)) {
          (*BI).btn.press(true);
          (*BI).btn.pressAction();
          Serial.println("You pressed on a button in a channel item page.");
          break;
        }
      }
      else {
        (*BI).btn.press(false);
      }
    }
  }
  else {
    // Within the Main Menu
    for (int i = 0; i < main_b_size; i++) {

      BI = main_b[i];

      if (pressed) {
        if ((*BI).btn.contains(x, y)) {
          (*BI).btn.press(true);
          (*BI).btn.pressAction();
          Serial.println("You pressed channel button: " + String(i + 1));
          break;
        }
      }
      else {
        (*BI).btn.press(false);
      }
    }
  }
}

// Declare an RTOS timer for the stepper motor actions
TimerHandle_t motorTimer;

void motorTimerAction(TimerHandle_t xTimer) {
  steppers_do_tasks();
}

/* TESTING AUDITORY INFUSION END ALARM */
#define INF_END_ALM 8

/* TESTING LOAD CELL */
HX711 scale;
#define LOADCELL_SCALE    6039.245 //12078.49 //852.924
#define LOADCELL_AVG_TAKE 1
#define LOADCELL_DOUT_PIN 16
#define LOADCELL_SCK_PIN  4

TaskHandle_t loadCellTaskHandle = NULL;
void loadCellTask(void *args) {
  // Read and print the load cell state if ready.

  while (1) {
    if (scale.is_ready()) {
      Serial.println("Load Cell Weight: " + String(scale.get_units(LOADCELL_AVG_TAKE)));
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void setup() {
  // Allow the correct Serial Baud Rate
  Serial.begin(115200);

  // Initializes subsystems.
  init_Stepper_Motors();
  init_Rotary_Encoder();
  init_LCD_Menu();
  init_Audio_Alarm();
  init_Load_Cell();

  Serial.print("Booting Firmware on Core: " + String(xPortGetCoreID()) + "\n");
  Serial.println("Touch Software Booted");

  setup_Main_Menu();
}

void loop() {

  update_All_Channels();

  // Scan keys every 50ms at most
  if (millis() - prevTouchTime >= debounceTouch) {

    prevTouchTime = millis();
    check_Touch_Buttons();
  }
  
  // Check to see if Rotary Encoder performed actions
  re_Controller();
}

/* ---------- METHODS FOR INITIALIZING HARDWARE ---------- */

void touch_calibrate() {
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check file system exists
  if (!LittleFS.begin()) {
    Serial.println("Formating file system");
    LittleFS.format();
    LittleFS.begin();
  }

  // check if calibration file exists and size is correct
  if (LittleFS.exists(CALIBRATION_FILE)) {
    if (REPEAT_CAL)
    {
      // Delete if we want to re-calibrate
      LittleFS.remove(CALIBRATION_FILE);
    }
    else
    {
      fs::File f = LittleFS.open(CALIBRATION_FILE, "r");
      if (f) {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL) {
    // calibration data valid
    tft.setTouch(calData);
  } else {
    // data not valid so recalibrate
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.println("Touch corners as indicated");

    tft.setTextFont(1);
    tft.println();

    if (REPEAT_CAL) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");

    // store data
    fs::File f = LittleFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}

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

  // Initialize the Stepper Motor timer
  motorTimer = xTimerCreate("MotorTimer", pdMS_TO_TICKS(1), pdTRUE, 0, motorTimerAction);

  if (motorTimer) {
    Serial.println("RTOS Motor Timer Initialized");
    xTimerStart(motorTimer, 0);
  }
  else
    Serial.println("RTOS Motor Timer Failed to Initialize");
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
  tft.begin();
  tft.setRotation(ROTATION);
  tft.fillScreen(TFT_WHITE);
  init_Touch_Screen();
  init_Touch_Buttons();

  // Set the default LCD cursor coordinate.
  x_default = tft.getCursorX();
  y_default = tft.getCursorY();
}

/**
*     Initializes the LCD Screen Touch functionality with provided calibration data.
*/
void init_Touch_Screen(void) {
  // Use this calibration code in setup():
  touch_calibrate();
}

/**
*     Initializes the actions and graphics for the touch buttons.
*/
void init_Touch_Buttons(void) {

  Button_Item *BI = NULL;

  uint16_t x = tft.width() / 2;   // For centralizing buttons
  uint16_t y = 40;                // Determines button layout

  uint16_t buttonWidth = x;

  uint16_t buttonHeight = (uint16_t) get_Button_Height(main_b_size, tft.height() - y);
  uint16_t spaceHeight = (uint16_t) get_Space_Height(main_b_size, tft.height() - y);
  
  y += (spaceHeight + (buttonHeight / 2));

  // Set the setup function for each main or channel menu button.
  for (int i = 0; i < main_b_size; i++) {

    BI = main_b[i];

    (*BI).btn.initButton(x, y, buttonWidth, buttonHeight, TFT_BLACK, TFT_SKYBLUE, TFT_BLACK, (*BI).label, 1);
    (*BI).btn.setPressAction((*BI).setup_func);
    y += (spaceHeight + buttonHeight);
  } 

  // Return y-positioning to the beginning location.
  y = 40;

  buttonHeight = get_Button_Height(channel_b_size, tft.height() - y);
  spaceHeight = get_Space_Height(channel_b_size, tft.height() - y);
  y += (spaceHeight + (buttonHeight / 2));

  // Set the print function for each channel item button.
  for (int i = 0; i < channel_b_size; i++) {

    BI = channel_b[i];

    (*BI).btn.initButton(x, y, buttonWidth, buttonHeight, TFT_BLACK, TFT_SKYBLUE, TFT_BLACK, (*BI).label, 1);
    (*BI).btn.setPressAction((*BI).setup_func);
    y += (spaceHeight + buttonHeight);
  }

  // Set the print function for each channel item button.
  for (int i = 0; i < item_b_size; i++) {

    // Return y-positioning to the beginning location.
    y = 240;

    buttonHeight = get_Button_Height(item_bl_size, tft.height() - y);
    spaceHeight = get_Space_Height(item_bl_size, tft.height() - y);
    y += (spaceHeight + (buttonHeight / 2));

    for (int j = 0; j < item_bl_size; j++) {
      BI = item_b[i][j];

      (*BI).btn.initButton(x, y, buttonWidth, buttonHeight, TFT_BLACK, TFT_SKYBLUE, TFT_BLACK, (*BI).label, 1);
      (*BI).btn.setPressAction((*BI).setup_func);
      y += (spaceHeight + buttonHeight);
    }
  }
}

/**
*     Initializes the Auditory Alarm For Ending An Infusion
*/
void init_Audio_Alarm(void) {
  pinMode(INF_END_ALM, OUTPUT);
}

/**
*     Initializes the Load Cell scale and scaling factor.
*/
void init_Load_Cell(void) {
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(LOADCELL_SCALE);
  scale.tare();

  // Initialize Load Cell Task
  xTaskCreatePinnedToCore(
    loadCellTask,         // Function for task to run
    "LoadCellTask",       // Name of the task
    4096,                 // Stack size of the task
    NULL,                 // Parameters
    1,                    // Priority
    &loadCellTaskHandle,   // Handler
    1                     // Core for task to use
  );
}

/* ---------- METHODS FOR INFUSION ALARM ---------- */

void alarm_Start(void) {
  //digitalWrite(INF_END_ALM, HIGH);
  tone(INF_END_ALM, 440);
}

void alarm_End(void) {
  noTone(INF_END_ALM);
}

/* ---------- METHODS FOR LOAD CELL ---------- */

/**
*    Re-tares the load cell scale.
*/
void retare_Scale(void) {
  vTaskSuspend(loadCellTaskHandle); // Suspend the task
  scale.set_scale(LOADCELL_SCALE);
  scale.tare();
  vTaskResume(loadCellTaskHandle);  // Resume the task
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

      // Determine if the Rotary Encoder has moved enough to warrant an action.
      if (!(curCountRE % RE_SCROLL_COUNT)) {
        
        if (activeChannelItem) {
          Serial.println("Performing Action Item " + String(activeChannelItem));
          (*channel_b[activeChannelItem - 1]).update_func();
          if (activeChannelItem != 5)
            (*channel_b[activeChannelItem - 1]).setup_func();
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

/*
*     Prints the proper menu header
*/
void print_Menu_Header(int channelNum, int itemNum, short subheader, String preSpace) {

  Serial.println("Printing Menu Header ");

  // Reset the screen to load a new window
  tft.fillScreen(TFT_WHITE);
  tft.setCursor(x_default, y_default);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);

  tft.setTextSize(1);
  tft.println();

  // Declare and initialize the subheader item
  String subheader_str;
  if (itemNum)
    subheader_str = (*channel_b[itemNum - 1]).label;

  // If the subheader is "Start", but the motor is actually running, change it to "Stop"
  if (channelNum && (subheader_str == "Start") && (*channels[channelNum - 1]).pstat == PUMP_STATUS::RUNNING)
    subheader_str = "Stop";
  
  // Print the rest of the header
  String menu_str = (!channelNum) ? (*channel_b[channel_b_size - 1]).label : (*main_b[channelNum - 1]).label ;
  Serial.println("Menu Item: " + menu_str);
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
      (*channel_b[i]).btn.drawSmoothButton(false, 3, TFT_BLACK);
  }
  else {
    // Main menu window.

    for (int i = 0; i < main_b_size; i++) 
      (*main_b[i]).btn.drawSmoothButton(false, 3, TFT_BLACK);

  }
}

/**
*     Prints the respective channel item buttons.
*/
void print_Item_Buttons(int channelNum, int activeChannelItem) {

  Button_Item *BI;
  Pump_Channel *channel = channels[channelNum - 1];

  for (int i = 0; i < item_bl_size; i++) {

    BI = item_b[activeChannelItem - 1][i];

    // Update the Resolution Button's text depending on current resolution settings.
    if (!strcmp((*BI).label, "Resolution")) {
      String resName = "Res: " + String((*channel).rstat);
      (*BI).btn.drawSmoothButton(false, 3, TFT_BLACK, resName);
    }
    else
      (*BI).btn.drawSmoothButton(false, 3, TFT_BLACK);
  }
}

/**
*     Exits a channel item's page.
*/
void exit_Item_Window(int channelNum) {
  (*channels[channelNum - 1]).rstat = RES_STATUS::ONES;
  (*main_b[channelNum - 1]).setup_func();
}

/**
*     Calculates the height of a window's buttons.
*/
double get_Button_Height(short numButtons, double totalHeight) {
  return ((0.7 * totalHeight) / (numButtons));
}

/**
*     Calculates the height of a window's spaces.
*/
double get_Space_Height(short numButtons, double totalHeight) {
  return ((0.3 * totalHeight) / (numButtons + 1));
}


/* ---------- METHODS FOR PUMP CHANNEL PARAMETER RESOLUTION DISPLAY & CONFIGURATION---------- */

/**
*     Cycles a pump channel's resolution.
*/
void cycle_Channel_Resolution(short channelNum) {

  // Retrieve correct pump
  Pump_Channel *channel = channels[channelNum - 1];

  if ((*channel).rstat == RES_STATUS::HUNDREDTHS) {
    (*channel).rstat = RES_STATUS::ONES;
  }
  else 
    (*channel).rstat = (RES_STATUS)(10 * (int)((*channel).rstat));

  (*channel_b[activeChannelItem - 1]).setup_func();
}

/**
*     Clears a pump channel's resolution value.
*/
void clear_Channel_Resolution(short channelNum, short channelItemNum) {
    
    // Retreive the correc pump channel.
    Pump_Channel *channel = channels[channelNum - 1];

    switch (channelItemNum) {
      case 1:
        (*channel).dosage = 0.0;
        break;
      case 2:
        (*channel).infusionRate = 0.0;
        break;
      case 3:
        (*channel).syringeStart = 0.0;
        break;
      case 4:
        (*channel).syringeEnd = 0.0;
        break;
      default:
        break;
    }

  (*channel_b[channelItemNum - 1]).setup_func();
}

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

  tft.print(preSpace);
  for (int i = 0; resStr[i] != '\0'; i++) {
    if (resStr[i] == '.') {
      print_res = (RES_STATUS)((int)print_res * 10);
      tft.print(resStr[i]);
    }
    else if (print_res == channelRes) {
      tft.setTextColor(TFT_BLACK, TFT_SKYBLUE);
      tft.print(resStr[i]);
      tft.setTextColor(TFT_BLACK, TFT_WHITE);
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

  // If there is an outOfBounds flag pointer, print accordingly
  if (outOfBounds[0]) {

    switch((int)outOfBounds[0]) {

      case 2:   // Concurrent modification error
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.println("ERROR: Cannot change\nvalue while motor is running.");
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        break;

      case 1:   // Out of upper bounds errors
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        if (name != "Start")
          tft.println("ERROR: Cannot exceed\nupper bound of " + String(outOfBounds[1]) + units);
        else
          tft.println("ERROR: Cannot exceed\nSyringe End of " + String(outOfBounds[1]) + units);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        break;

      case -1:   // Out of lower bounds errors
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        if (name != "End")
          tft.println("ERROR: Cannot exceed\nlowerbound of " + String(outOfBounds[1]) + units);
        else
          tft.println("ERROR: Cannot exceed\nSyringe Start of " + String(outOfBounds[1]) + units);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        break;
      default:    // Unknown error
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.println("ERROR: Unknown exception thrown.");
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        break;
    }
  }

  print_Item_Buttons(activeChannel, activeChannelItem);

  tft.setTextSize(3); 
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
      activate_Stepper_Motor(motorNum, MOTOR_STEPS * MICROSTEPS * mult, 50);
    }
  }
  else if (dirRE == TURN_DIR::CW) { 
    // TODO: Test if removing libray-dependent condition works
    if (steppers.is_finished(motorNum)) {
      (*channel).pstat = PUMP_STATUS::CALIBRATE;
      set_Stepper_Motor_Direction(channel, TURN_DIR::CW);
      activate_Stepper_Motor(motorNum, MOTOR_STEPS * MICROSTEPS * mult, 50);
    }
  }
}

/**
*    Print the calibration instructions.
*/
void print_Channel_Calibrate(int channelNum) {

  tft.setTextSize(3);
  tft.println("Rev/Turn: " + String(10.0 / (*channels[channelNum - 1]).rstat) + "\n");
  tft.println("Turn Knob:");
  tft.println(" - CW to Push");
  tft.println(" - A/CCW to Pull");
  tft.setTextSize(4);
  tft.println("-------------");
  tft.setTextSize(2);
  tft.println("");
  tft.setTextSize(3);

  print_Item_Buttons(channelNum, activeChannelItem);
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

  if ((*channel).pstat == PUMP_STATUS::RUNNING) {
    (*channel).resolutionCodes[0] = 1.0;
    return;
  }
  // Determine the number of steps to achieve the pump distance
  (*channel).stepCount = ((*channel).syringeEnd - (*channel).syringeStart)*(STEP_LENGTH);

  // Determine the step delay 

  // Total Time of procedure (In Hours)
  double totalTime = ((*channel).dosage) / ((*channel).infusionRate);
  totalTime *= 3600;    // Total Time of procedure (In Seconds)
  totalTime *= 1000000; // Total Time of procedure (In Microseconds)
  
  // Step delay
  (*channel).stepDelay = (totalTime) / ((*channel).stepCount - 1);
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
void print_Start_Stop(int channelNum, double errorCode) {

  Pump_Channel *channel = channels[channelNum - 1];

  String cur_action = "Exit";  // The currently selected option 
  String re_action;   // The next option 
  String c_stat;      // The current status of the motor
  String temp_str;

  // Determine the motor's current status. 
  if (steppers.is_running(channelNum - 1)) 
    c_stat = "Running";
  else if ((*channel).pstat == PUMP_STATUS::COMPLETE)
    c_stat = "Complete";
  else
    c_stat = "Stopped";

  tft.println(" " + String((*channel).stepCount));
  tft.println("   Steps Left");
  tft.setTextSize(2);
  tft.println("   Motor is " + c_stat);
  tft.setTextSize(4);
  tft.println("-------------");
  tft.setTextSize(2);
  /*
   *    Displays error code information.
   *    
   *    0 - No error 
   *    1 - Attempted value re-calculation during running.
   */
  switch ((int)errorCode) {
    case 1:
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.println("ERROR: Cannot calculate\nsteps while motor is running.");
      tft.setTextColor(TFT_BLACK, TFT_WHITE);
      break;
    default:
      break;
  }

  tft.setTextSize(3);

  print_Item_Buttons(channelNum, activeChannelItem);
}
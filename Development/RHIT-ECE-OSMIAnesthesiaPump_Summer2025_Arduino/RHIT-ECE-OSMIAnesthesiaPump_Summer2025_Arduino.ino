#include <ESP32Encoder.h>
#include <MultiStepperLite.h>
#include <TFT_eSPI.h>
#include <User_Setup_Select.h>



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
#define MOTOR0_STEP   1
#define MOTOR0_DIR    2
#define MOTOR1_STEP   35
#define MOTOR1_DIR    45
#define MOTOR2_STEP   7 
#define MOTOR2_DIR    6
#define MOTOR3_STEP   18
#define MOTOR3_DIR    17

int test = TFT_CS;

// Define Stepper Motor Objects
MultiStepperLite steppers(4);
int activeMotors[4] = {0, 0, 0, 0};

// // Define LCD Pins
// #define GFX_CS    11
// #define GFX_DC    9
// #define GFX_RST   10
// #define GFX_BL    8
// #define GFX_SCK   3
// #define GFX_MOSI  46

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

// Define menu arrays and subarrays
String menu[4][4] = {{"Channel 1", "C1 Item 1", "C1 Item 2", "C1 Item 3"},
                     {"Channel 2", "C2 Item 1", "C2 Item 2", "C2 Item 3"}, 
                     {"Channel 3", "C3 Item 1", "C3 Item 2", "C3 Item 3"}, 
                     {"Channel 4", "C4 Item 1", "C4 Item 2", "C4 Item 3"}};
enum ACTIVE_MENU_WINDOW { // Tracks which menu screen to display 
  MAIN,       // Displays the main menu options
  CHANNEL1,   // Displays pump channel 1 menu options
  CHANNEL2,   // Displays pump channel 2 menu options
  CHANNEL3,   // Displays pump channel 3 menu options
  CHANNEL4    // Displays pump channel 4 menu options
};
ACTIVE_MENU_WINDOW activeWindow = ACTIVE_MENU_WINDOW::MAIN;

enum ACTIVE_MENU_ITEM {
  Channel_1,
  Channel_2,
  Channel_3,
  Channel_4
};
ACTIVE_MENU_ITEM activeItem = ACTIVE_MENU_ITEM::Channel_1;

// Structure that will store each channel's configuration
struct PumpChannel {
  unsigned short motorNumber;       // The motor number of this channel
  double travelTime;                // The amount of time required to push the syringe from start to finish.  

};

void setup() {
  // Initializes all four Stepper Motors.
  init_Stepper_Motors();
  init_Rotary_Encoder();

  // Allow the correct Serial Baud Rate
  Serial.begin(115200);
  Serial.print("Booting Firmware\n");
}

void loop() {

  re_Controller();
  //re_To_Manual_Stepper(0);

}

/**
*   Initializes all four stepper motors
*/
void init_Stepper_Motors(void) {
  pinMode(MOTOR0_DIR, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR3_DIR, OUTPUT);
  digitalWrite(MOTOR0_DIR, LOW);
  digitalWrite(MOTOR1_DIR, LOW);
  digitalWrite(MOTOR2_DIR, LOW);
  digitalWrite(MOTOR3_DIR, LOW);

  steppers.init_stepper(0, MOTOR0_STEP);
  steppers.init_stepper(1, MOTOR1_STEP);
  steppers.init_stepper(2, MOTOR2_STEP);
  steppers.init_stepper(3, MOTOR3_STEP);
}

/**
*   Activate a Stepper Motor.
*   Return a 0 if valid motor, -1 if invalid motor.
*/
int activate_Stepper_Motor(int motorNum, int numSteps) {

  if (motorNum > 3 || motorNum < 0) {
    return -1;
  }

  steppers.start_finite(motorNum, 4000, numSteps);
  activeMotors[motorNum] = 1;
  return 0;
}

/**
*   Sets the direction of a Stepper Motor
*/
void set_Stepper_Motor_Direction(int motorNum, TURN_DIR dir) {

  // If Clockwise, then go LOW ("Away from motor"); otherwise, go HIGH ("Toward motor");
  int pinLevel = (dir == TURN_DIR::CW) ? LOW : HIGH;

  switch (motorNum) {
    case 0:
      digitalWrite(MOTOR0_DIR, pinLevel);
      break;
    case 1:
      digitalWrite(MOTOR1_DIR, pinLevel);
      break;
    case 2:
      digitalWrite(MOTOR2_DIR, pinLevel);
      break;
    case 3:
      digitalWrite(MOTOR3_DIR, pinLevel);
      break;
    default:
      break;
  }
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
        if (curCountRE % RE_SCROLL_COUNT == 0) {
          Serial.print("Scrolling Through Menu\n");
          update_Scroll_Menu(dirRE);
          print_Scroll_Menu();
        }

        // Print results for debugging
        // Serial.print("Encoder count = " + String((int32_t) curCountRE) + " \n");
        // Serial.print("Going Direction: " + String((dirRE == TURN_DIR::CCW) ? "CCW" : "CW") + " \n");
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

      Serial.print("You pressed the Switch!\n Nice...\n");
    }
}

/**
*   Use the Rotary Encoder to manually move a selected Stepper Motor
*/ 
void re_To_Manual_Stepper(int motorNum) {

  if (dirRE == TURN_DIR::CCW) {
    set_Stepper_Motor_Direction(motorNum, TURN_DIR::CCW);
    if (steppers.is_finished(motorNum)) {
      activate_Stepper_Motor(motorNum, MOTOR_STEPS * MICROSTEPS);
    }
    // else if (steppers.is_paused(motorNum)) {
    //   steppers.resume(motorNum);
    // }
  }
  else if (dirRE == TURN_DIR::CW) { 
    set_Stepper_Motor_Direction(motorNum, TURN_DIR::CW);
    if (steppers.is_finished(motorNum)) {
      activate_Stepper_Motor(motorNum, MOTOR_STEPS * MICROSTEPS);
    }
    // else if (steppers.is_paused(motorNum)) {
    //   steppers.resume(motorNum);
    // }
  }
  steppers.do_tasks();
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

/**
*     Updates the scroll menu
*     (Currently, only for Serial Monitor)
*/
void  update_Scroll_Menu(TURN_DIR dir) {

  int activeItemInt = activeItem;

  switch(activeWindow) {
    case ACTIVE_MENU_WINDOW::CHANNEL1:

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2:

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3:

      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4:

      break;

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
      break;
  }
}

/**
*     Prints the scroll menu
*     (Currently, only for Serial Monitor)
*/
void print_Scroll_Menu(void) {
  String linePrint;
  int activeItemLoopIndex;

  Serial.print("\n\n\n\n\n\n\n\n");
  switch(activeWindow) {    
    case ACTIVE_MENU_WINDOW::CHANNEL1:

      Serial.print("--- Channel 1 ---\n");
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL2:

      Serial.print("--- Channel 2 ---\n");
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL3:

      Serial.print("--- Channel 3 ---\n");
      break;

    case ACTIVE_MENU_WINDOW::CHANNEL4:
      Serial.print("--- Channel 4 ---\n");
      break;
    
    default:  // Also for ACTIVE_MENU_WINDOW::MAIN:

      switch(activeItem) {
        case ACTIVE_MENU_ITEM::Channel_2:
            activeItemLoopIndex = 1;
          break;
        case ACTIVE_MENU_ITEM::Channel_3:
            activeItemLoopIndex = 2;
          break;
        case ACTIVE_MENU_ITEM::Channel_4:
            activeItemLoopIndex = 3;
          break;
        default: // Also for ACTIVE_MENU_WINDOW::CHANNEL1:
            activeItemLoopIndex = 0;
          break;
      }

      Serial.print("--- Main Menu ---\n");
      for (int i = 0; i < std::size(menu); i++) {
        linePrint = menu[i][0] + " " + ((activeItemLoopIndex == i) ? "<\n" : "\n");
        Serial.print(linePrint);
      }
      break;
  }
}
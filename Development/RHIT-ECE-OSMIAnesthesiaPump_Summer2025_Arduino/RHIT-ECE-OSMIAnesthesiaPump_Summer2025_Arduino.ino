#include <A4988.h>
#include <BasicStepperDriver.h>
#include <DRV8825.h>
#include <DRV8834.h>
#include <DRV8880.h>
#include <ESP32Encoder.h>
#include <MultiDriver.h>
#include <SyncDriver.h>

/*      Global Variables      */
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 4

// Board time since previous Rotary Encoder switch press
unsigned long prevTimeRE = 0;
const long debounceRE = 50;

volatile int prevCountRE;
volatile int curCountRE;

// Define Stepper Motor Pins
#define MOTOR1_STEP   1
#define MOTOR1_DIR    2
#define MOTOR2_STEP   35
#define MOTOR2_DIR    45
#define MOTOR3_STEP   7 
#define MOTOR3_DIR    6
#define MOTOR4_STEP   18
#define MOTOR4_DIR    17

// Define Stepper Motor Objects
BasicStepperDriver stepper1(MOTOR_STEPS, MOTOR1_DIR, MOTOR1_STEP);
BasicStepperDriver stepper2(MOTOR_STEPS, MOTOR2_DIR, MOTOR2_STEP);
BasicStepperDriver stepper3(MOTOR_STEPS, MOTOR3_DIR, MOTOR3_STEP);
BasicStepperDriver stepper4(MOTOR_STEPS, MOTOR4_DIR, MOTOR4_STEP);

unsigned int allMotors[4] = {(unsigned int) &stepper1, 
                             (unsigned int) &stepper2, 
                             (unsigned int) &stepper3, 
                             (unsigned int) &stepper4};
int activeMotors[4] = {0, 0, 0, 0};



// Define LCD Pins
#define GFX_CS    11
#define GFX_DC    9
#define GFX_RST   10
#define GFX_BL    8
#define GFX_SCK   3
#define GFX_MOSI  46

// Define Rotary Encoder Pins
#define RE_CLK    38          // Encoder Pin A
#define RE_DT     37          // Encoder Pin B
#define RE_SW     36          // No-Push Button Switch

// Define Rotary Encoder Object
ESP32Encoder re;

enum TURN_DIR {
  CW,         // Clockwise
  CCW,        // Counter-clockwise
  STOP
};
TURN_DIR dirRE = TURN_DIR::STOP; 



void setup() {
  // Initializes all four Stepper Motors.
  init_Stepper_Motors();
  init_Rotary_Encoder();

  // Allow the correct Serial Baud Rate
  Serial.begin(115200);
  Serial.print("Booting Firmware\n");

  Serial.print("Select a Stepper Motor to Activate: \n");
  while (Serial.available() == 0);  // Wait to receive an input
  int result = (Serial.readStringUntil('\n')).toInt();
  Serial.print("Activating Stepper Motor " + String(result) + ".\n");
  set_Active_Motor(result - 1);
}

void loop() {

  re_Interrupt();
  RE_To_Manual_Stepper();

}

/**
*   Initializes all four stepper motors
*/
void init_Stepper_Motors(void) {
  stepper1.begin(RPM, MICROSTEPS);
  stepper2.begin(RPM, MICROSTEPS);
  stepper3.begin(RPM, MICROSTEPS);
  stepper4.begin(RPM, MICROSTEPS);
}

/**
*     Initializes the Rotary Encoder
*/
void init_Rotary_Encoder(void) {

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  re.attachFullQuad(RE_CLK, RE_DT);
  re.setCount(0);
  prevCountRE = re.getCount();
}

/**
*      Acts as the interrupt for the Rotary Encoder
*/
void re_Interrupt(void) {
    // Obtain the current time
    unsigned long curTimeRE = millis();

    if (curTimeRE - prevTimeRE >= debounceRE) {
      prevTimeRE = curTimeRE;

      curCountRE = re.getCount();

      // Check to see if the Rotary Encoder has moved.
      // If it has, determine the direction of rotation.
      if (curCountRE != prevCountRE) {
        dirRE = (curCountRE - prevCountRE > 0) ? TURN_DIR::CCW : TURN_DIR::CW;
        Serial.print("Encoder count = " + String((int32_t) re.getCount()) + " \n");
        Serial.print("Going Direction: " + String((dirRE == TURN_DIR::CCW) ? "CCW" : "CW") + " \n");
      }
      else {
        dirRE = TURN_DIR::STOP;
      } 
      prevCountRE = re.getCount();
    }

}

/**
*   Use the Rotary Encoder to manually move the active Stepper Motor
*/ 
void RE_To_Manual_Stepper(void) {

  // Retrieve active stepper
  BasicStepperDriver *p = get_Active_Motor();
  if (dirRE == TURN_DIR::CCW) {
    (*p).move(MOTOR_STEPS * MICROSTEPS);
  }

  if (dirRE == TURN_DIR::CW) { 
    (*p).move(-MOTOR_STEPS * MICROSTEPS);
  }
}

/**
*   Sets one of four Stepper Motors as activated, or selected.
*   Will deactivate, or deselect, all other activated Stepper Motors.
*/
void set_Active_Motor(short num) {
  for (int i = 0; i < std::size(activeMotors); i++) {
    if (i == num) {
      activeMotors[i] = 1;
    }
    else {
      activeMotors[i] = 0;
    }
  }
}

/**
*     Returns a pointer to the active Stepper Motor
*/
BasicStepperDriver *get_Active_Motor(void) {

  // Create pointer to-be-returned
  BasicStepperDriver *p = nullptr;

  for (int i = 0; i < std::size(activeMotors); i++) {
    if (activeMotors[i] == 1) {
      return (p = (BasicStepperDriver *)allMotors[i]);
    }
  }

  return p; 
}

/**
*     Returns a pointer to a specified Stepper Motor
*/
BasicStepperDriver *get_Specific_Motor(short num) {
    
  // Create pointer to-be-returned
  BasicStepperDriver *p = nullptr;

  switch (num) {
    case 1: return (p = &stepper1);
    case 2: return (p = &stepper2);
    case 3: return (p = &stepper3);
    case 4: return (p = &stepper4);
    default: return p;
  }
}
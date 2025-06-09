/*
    Controlling multiple steppers with the AccelStepper and MultiStepper library

     by Dejan, https://howtomechatronics.com
*/
#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <AS5600.h>

// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(1, 2, 5); // (Typeof driver: with 2 pins, STEP, DIR)
AccelStepper stepper2(1, 3, 6);
AccelStepper stepper3(1, 4, 7);

MultiStepper steppersControl;  // Create instance of MultiStepper

long gotoposition[3] = {0, 0, 0}; // An array to store the target positions for each stepper motor

void setup() {
  Serial.begin(9600);
  stepper1.setMaxSpeed(1000); // Set maximum speed value for the stepper
  stepper2.setMaxSpeed(1000);
  stepper3.setMaxSpeed(1000);

  // Adding the 3 steppers to the steppersControl instance for multi stepper control
  steppersControl.addStepper(stepper1);
  steppersControl.addStepper(stepper2);
  steppersControl.addStepper(stepper3);
  // delay (3000);
}

void loop() {
  while (Serial.available() > 0)
  {
    //Create a place to hold the incoming message
    // static char message[12];
    // static unsigned int message_pos = 0;

    //Read the next available byte in the serial receive buffer
    char inByte = Serial.read();
    Serial.print(inByte);
    if(inByte == 'w')
    {
      gotoposition[0] = 3;
      steppersControl.moveTo(gotoposition); // Calculates the required speed for all motors
      steppersControl.runSpeedToPosition(); // Blocks until all steppers are in position
      // Serial.println(stepper)
      stepper1.setCurrentPosition(0);
    }
    if(inByte == 's')
    {
      gotoposition[0] = -3;
      steppersControl.moveTo(gotoposition); // Calculates the required speed for all motors
      steppersControl.runSpeedToPosition(); // Blocks until all steppers are in position
      // Serial.println(stepper)
      stepper1.setCurrentPosition(0);
    }
    if(inByte == 'q')
    {
      gotoposition[1] = 3;
      steppersControl.moveTo(gotoposition); // Calculates the required speed for all motors
      steppersControl.runSpeedToPosition(); // Blocks until all steppers are in position
      // Serial.println(stepper)
      stepper2.setCurrentPosition(0);
    }
    if(inByte == 'a')
    {
      gotoposition[1] = -3;
      steppersControl.moveTo(gotoposition); // Calculates the required speed for all motors
      steppersControl.runSpeedToPosition(); // Blocks until all steppers are in position
      // Serial.println(stepper)
      stepper2.setCurrentPosition(0);
    }

    // delay (250);
  }
}
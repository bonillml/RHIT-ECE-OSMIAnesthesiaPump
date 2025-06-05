#include <Arduino.h>
 
#define IR_LED_PIN  5      // Adjust to your IR LED pin
#define IR_PHOTODIODE_PIN  34  // Adjust to your ADC pin
#define ONBOARD_LED_PIN 2  // Usually GPIO 2, check your board
 
void setup() {
    Serial.begin(115200);
 
    pinMode(IR_LED_PIN, OUTPUT);
    pinMode(IR_PHOTODIODE_PIN, INPUT);
    pinMode(ONBOARD_LED_PIN, OUTPUT);
 
    // Blink onboard LED to indicate successful upload
    for (int i = 0; i < 3; i++) {
        digitalWrite(ONBOARD_LED_PIN, HIGH);
        delay(200);
        digitalWrite(ONBOARD_LED_PIN, LOW);
        delay(200);
    }
}
 
void loop() {
    digitalWrite(IR_LED_PIN, HIGH); // Turn ON IR LED
    delay(10); // Short delay to stabilize the IR light
 
    int irReading = analogRead(IR_PHOTODIODE_PIN); // Read the photodiode voltage
    Serial.print("IR Sensor Reading: ");
    Serial.println(irReading);
 
    digitalWrite(IR_LED_PIN, LOW); // Turn OFF IR LED
    // Blink onboard LED once every reading
    digitalWrite(ONBOARD_LED_PIN, HIGH);
    delay(50);
    digitalWrite(ONBOARD_LED_PIN, LOW);
    delay(100); // Adjust delay for your sampling rate
}
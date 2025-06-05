#include <SPI.h>
#include <Arduino.h>
#include <TFT_eSPI.h> // Hardware-specific library

TFT_eSPI tft = TFT_eSPI();

#define TFT_GREY 0x5AEB // Custom grey color

// Pin definitions for active-low buttons
#define UP_BUTTON_PIN 27
#define DOWN_BUTTON_PIN 26
#define SELECT_BUTTON_PIN 25

int currentOption = 0;      // Index of the highlighted option
int currentMenuLevel = 0;   // Current level in the menu
bool inSubMenu = false;     // Track if we're in a submenu

// Define menu options for each level
const char* mainMenu[] = {"Option 1", "Option 2", "Option 3", "Go to Submenu"};
const char* subMenu[] = {"Sub Option 1", "Sub Option 2", "Back to Main"};

int mainMenuSize = sizeof(mainMenu) / sizeof(mainMenu[0]);
int subMenuSize = sizeof(subMenu) / sizeof(subMenu[0]);

void moveUp() {
  currentOption = (currentOption - 1 + (inSubMenu ? subMenuSize : mainMenuSize)) % (inSubMenu ? subMenuSize : mainMenuSize);
}

void moveDown() {
  currentOption = (currentOption + 1) % (inSubMenu ? subMenuSize : mainMenuSize);
}

void selectOption() {
  if (!inSubMenu && currentOption == mainMenuSize - 1) {
    // Enter submenu
    inSubMenu = true;
    currentOption = 0;
  } else if (inSubMenu && currentOption == subMenuSize - 1) {
    // Exit submenu
    inSubMenu = false;
    currentOption = 0;
  } else {
    // Handle option selection (implement functionality here)
    Serial.print("Selected: ");
    Serial.println(inSubMenu ? subMenu[currentOption] : mainMenu[currentOption]);
  }
}

void displayMenu() {
  tft.fillScreen(TFT_GREY);
  tft.setTextColor(TFT_WHITE, TFT_GREY);
  tft.setTextFont(2);
  tft.setTextSize(5);

  const char** menu = inSubMenu ? subMenu : mainMenu;
  int menuSize = inSubMenu ? subMenuSize : mainMenuSize;

  for (int i = 0; i < menuSize; i++) {
    if (i == currentOption) {
      // Highlight the current option
      tft.setTextColor(TFT_BLACK, TFT_YELLOW);
    } else {
      tft.setTextColor(TFT_WHITE, TFT_GREY);
    }
    tft.setCursor(0, i * 70);
    tft.println(menu[i]);
  }
}
// Debounce delay time
const unsigned long debounceDelay = 50;
unsigned long lastUpButtonPress = 0;
unsigned long lastDownButtonPress = 0;
unsigned long lastSelectButtonPress = 0;

void setup() {
  Serial.begin(9600);

  // Initialize TFT
  tft.init();
  tft.setRotation(1);

  // Initialize buttons
  // detachInterrupt(UP_BUTTON_PIN);
  //   detachInterrupt(DOWN_BUTTON_PIN);

  // detachInterrupt(SELECT_BUTTON_PIN);

  pinMode(UP_BUTTON_PIN, INPUT_PULLUP);
  // digitalWrite(UP_BUTTON_PIN, LOW); 
  pinMode(DOWN_BUTTON_PIN, INPUT_PULLUP);
    // digitalWrite(DOWN_BUTTON_PIN, LOW); 

  pinMode(SELECT_BUTTON_PIN, INPUT_PULLUP);
      // digitalWrite(SELECT_BUTTON_PIN, LOW); 



  // Initial display of the menu
  displayMenu();
  
}

void loop() {
  // Check for button presses with debounce
 Serial.print(digitalRead(UP_BUTTON_PIN));
  Serial.print(digitalRead(DOWN_BUTTON_PIN));
   Serial.print(digitalRead(SELECT_BUTTON_PIN));
  if (digitalRead(UP_BUTTON_PIN) == LOW) {
    if (millis() - lastUpButtonPress > debounceDelay) {
      lastUpButtonPress = millis();
      moveUp();
      displayMenu();
    }
  } 
  
  if (digitalRead(DOWN_BUTTON_PIN) == LOW) {
    if (millis() - lastDownButtonPress > debounceDelay) {
      lastDownButtonPress = millis();
      moveDown();
      displayMenu();
    }
  } 
  
  if (digitalRead(SELECT_BUTTON_PIN) == LOW) {
    if (millis() - lastSelectButtonPress > debounceDelay) {
      lastSelectButtonPress = millis();
      selectOption();
      displayMenu();
    }
  }
}
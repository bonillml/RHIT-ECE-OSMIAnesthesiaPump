# Open-Source Medical Infusion Anesthesia Pump

# Table of Contents
1. Important Data Structures
    1. PumpChannel / Pump_Channel
        1. PUMP_STATUS
        2. RES_STATUS
        3. resolutionCodes
    2. ButtonItem / Button_Item
        1. ButtonWidget
        2. Setup Functions
        3. Update Functions




## Important Data Structures

### PumpChannel / Pump_Channel
The Pump_Channel struct stores important per-channel information. The stored information is a mixture user-inputted parameters, hardcoded global variables, and calculated parameters. The declaration of the struct can be found as so:

```C++
// Structure that will store each channel's configuration
typedef struct PumpChannel {
  unsigned short motorNumber = 0;           // (Hardcoded) The motor number of this channel              
  unsigned short directionPin = 0;          // (Hardcoded) The direction pin number of this channel.     
  PUMP_STATUS pstat = PUMP_STATUS::IDLE;    // (Calculated) The status of the pump channel.
  RES_STATUS rstat = RES_STATUS::ONES;      // (Calculated) The current resolution digit for inputs.
  double dosage = 10.0;                     // (User-Input) The amount of medicine in either mL or mg/kg
  double infusionRate = 5.0;                // (User-Input) The rate at which to pump the medcine in either mL/hr or mg/kg/hr
  double syringeStart = 10.0;               // (User-Input) The length marker on the track where the pump will start pushing the syringe.
  double syringeEnd = 15.0;                 // (User-Input) The length marker on the tracker at which the syringe can no longer be pushed.
  unsigned long stepCount = 0;              // (Calculated) The amount of steps this channel needs to take to complete its infusion.
  unsigned long stepDelay = 0;              // (Calculated) The amount of time needed between steps to ensure the correct infusion rate.

  double resolutionCodes[2] = {0.0, 0.0};   // (Calculated) The return code and error when exceeding set parameter bounds.
}Pump_Channel;

// Create instances of the pump channel structure for each channel.
Pump_Channel pumpChannel1, pumpChannel2, pumpChannel3, pumpChannel4;
Pump_Channel *channels[4] = {&pumpChannel1, &pumpChannel2, &pumpChannel3, &pumpChannel4};

```

The user-inputted parameters--with focus on infusion rate, syringe start, and syringe end--will likely be re-named or swapped for other parameters as the client or user sees fit. One prominant instance would be swapping infusion rate with patient weight. Alterations like these can be easily done via the following steps:
1. Update the corresponding element in the Pump_Channel struct.
2. Update the corresponding Button_Item data structures. (detailed in a **ButtonItem / Button_Item**)
    1. Alter the label in the respective Button_Item's declaration.
    2. Alter the user-inputted paramter's row in item_b[][] matrix.
    3. Alter the functionality of the Button_Item's setup and update functions. 
3. Update the corresponding page setup and page print functions near the bottom of the "RHIT_ECE_OSMIAnesthesiaPump_Summer2025_Arduino_Touch".
    1. These page setup and page print functions may relate to a Button_Item's setup and update function.

#### PUMP_STATUS
The PUMP_STATUS of a channel tracks the state or mode of a pump. Depending on the state, the PUMP_STATUS promotes or prevents various actions of the pump channel. These various actions are found within the page setup and page print functions mentioned in the Button_Items' setup and update functions. The declaration of the enumeration can be found as so:

```C++
/*  
*   Determines the status of a pump channel.
*   Is used as a redundancy and for optimization.
*/
enum PUMP_STATUS {
  IDLE,          // For when the pump isn't moving or mid-infusion.
  CONFIG,        // For when the pump is being configured. (DEPRECATED - May be redundant due to IDLE state.)
  CALIBRATE,     // For when the pump is being calibrated.
  RUNNING,       // For when the pump is not being calibrated but is running an infusion.
  PAUSED,        // For when the pump isn't moving mid-infusion.
  COMPLETE       // For when the pump has finished an infusion.
};
```

#### RES_STATUS
The RES_STATUS, short for resolution status, of a channel determines which digit the user can manipulate when entering a user-inputted parameter's value. For example, if the resolution status of a channel is **ONES**, then the user can only update the ones-place of a user-inputted parameter. Similarly, if the resolution status of a channel is **HUNDREDTHS**, then the user can only update the hundredths-place of a user-inputted parameter. 

However, when a channel is on the calibration page (associated with the **CALIBRATE** state), the resolution determines the number of revolutions per turn of the rotary encoder. For example, if the resolution status of a channel is **ONES**, then for every turn of the rotary encoder the stepper motor will rotate `(10 / **ONES**) = 10` revolutions, or ten revolutions. Similarly, if the resolution status of a channel is **TENTHS**, then for every turn of the rotary encoder the stepper motor will rotate `(10 / **TENTHS**) = 1` revolution, or one revolution. The declaration of the enumeration can be found as so:

```C++
/*  
*   Determines the resolution of inputs.
*   Is used for user-inputted parameters and bound constraints.
*/
enum RES_STATUS {
  ONES = 1,
  TENTHS = 10,
  HUNDREDTHS = 100
};
```

#### resolutionCodes
The resolution codes of a channel act as a place where page setup and page print functions can send return codes and receive error codes. Return codes are primarily implemented to pass information to error messages and error codes are primary implemented to print correct error codes. Dominant errors include user-inputted parameters attempting to update beyond set boundaries. Because the resolution codes fall in a two-element array (resolutionCodes[2]), the first element is dedicated to the error code and the second element is dedicated to the return code or other passed information.

### ButtonItem / Button_Item

#### ButtonWidget

#### Setup Functions

#### Update Functions






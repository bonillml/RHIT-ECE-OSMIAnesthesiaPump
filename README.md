# Open-Source Medical Infusion Anesthesia Pump

## Important Data Structures

### PumpChannel / Pump_Channel
The Pump_Channel struct stores important per-channel information. The stored information is a mixture user-inputted parameters, hardcoded global variables, and calculated parameters. The declaration of the struct can be found as so:

```C
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
  a. Alter the label in the respective Button_Item's declaration.
  b. Alter the user-inputted paramter's row in item_b[][] matrix.
  c. Alter the functionality of the Button_Item's setup and update functions. 
3. Update the corresponding page setup and page print functions near the bottom of the "RHIT_ECE_OSMIAnesthesiaPump_Summer2025_Arduino_<Variant>".
  a. These page setup and page print functions may relate to a Button_Item's setup and update function.

#### resolutionCodes
The resolution codes of a channel act as a place where page setup and page print functions can send return codes and receive error codes. Return codes are primarily implemented to pass information to error messages and error codes are primary implemented to print correct error codes. Dominant errors include user-inputted parameters attempting to update beyond set boundaries. Because the resolution codes fall in a two-element array (resolutionCodes[2]), the first element is dedicated to the error code and the second element is dedicated to the return code or other passed information.

#### PUMP_STATUS

#### RES_STATUS

### ButtonItem / Button_Item

#### Setup Functions

#### Update Functions

####






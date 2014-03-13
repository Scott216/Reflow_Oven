/*******************************************************************************
 * Title: Reflow Oven Controller
 * Company: Rocket Scream Electronics
 * Original Author: Lim Phang Moh
 * Website: www.rocketscream.com
 * 
 * Brief
 * =====
 * This is an example firmware for our Arduino compatible reflow oven controller. 
 * The reflow curve used in this firmware is meant for lead-free profile 
 * (it's even easier for leaded process!). You'll need to use the MAX31855 
 * library for Arduino if you are having a shield of v1.60 & above which can be 
 * downloaded from our GitHub repository. Please check our wiki 
 * http://www.rocketscream.com/wiki for more information on using this piece of code 
 * together with the reflow oven controller shield. 
 *
 * Temperature (Degree Celcius)                 Magic Happens Here!
 * 245-|                                               x  x  
 *     |                                            x        x
 *     |                                         x              x
 *     |                                      x                    x
 * 200-|                                   x                          x
 *     |                              x    |                          |   x   
 *     |                         x         |                          |       x
 *     |                    x              |                          |
 * 150-|               x                   |                          |
 *     |             x |                   |                          |
 *     |           x   |                   |                          | 
 *     |         x     |                   |                          | 
 *     |       x       |                   |                          | 
 *     |     x         |                   |                          |
 *     |   x           |                   |                          |
 * 30 -| x             |                   |                          |
 *     |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
 *     | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
 *  0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
 *                                 Time (Seconds)
 *
 * This firmware owed very much on the works of other talented individuals as
 * follows:
 * ==========================================
 * Brett Beauregard (www.brettbeauregard.com)
 * ==========================================
 * Author of Arduino PID library. On top of providing industry standard PID 
 * implementation, he gave a lot of help in making this reflow oven controller 
 * possible using his awesome library.
 *
 *
 * Disclaimer
 * ==========
 * Dealing with high voltage is a very dangerous act! Please make sure you know
 * what you are dealing with and have proper knowledge before hand. Your use of 
 * any information or materials on this reflow oven controller is entirely at 
 * your own risk, for which we shall not be liable. 
 *
 * Licences
 * ========
 * This reflow oven controller hardware and firmware are released under the 
 * Creative Commons Share Alike v3.0 license
 * http://creativecommons.org/licenses/by-sa/3.0/ 
 * You are free to take this piece of code, use it and modify it. 
 * All we ask is attribution including the supporting libraries used in this 
 * firmware. 
 *
 *
 * Source: http://github.com/CvW/Reflow-Oven-Controller/blob/master/reflowOvenController.pde
 * Which is based on http://github.com/rocketscream/Reflow-Oven-Controller
 *
 * Info on PID: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 * Build instructions: http://www.rocketscream.com/blog/portfolio-item/reflow-controller-shield
 *
 *
 * Rocket Scream Revisions
 * Revision  Description
 * ========  ===========
 * 1.20			Adds supports for v1.60 (and above) of Reflow Oven Controller 
 *           Shield:
 *					  - Uses MAX31855KASA+ chip and pin reassign (allowing A4 & A5 (I2C)
 *             to be used for user application).
 *					  - Uses analog based switch (allowing D2 & D3 to be used for user 
 *						  application).	
 *						Adds waiting state when temperature too hot to start reflow process.
 *						Corrected thermocouple disconnect error interpretation (MAX6675).
 * 1.10      Arduino IDE 1.0 compatible.
 * 1.00      Initial public release.
 *
 *
 * CvW Revisions
 * Revision  Description
 * ========  ===========
 * 1.20      Added option for lead solder profile, Enabled fan motor, added error message
 * 1.10      Arduino IDE 1.0 compatible.
 * 1.00      Initial public release.
 *
 * Scott216 Revisions
 * Revision  Description
 * ========  ===========
 * 1.21      Started with CvW version 1.2 and replaced MAX6675 library with Adafruit_MAX31855, changed some formatting
 * 1.22      Moved LCD display code to separate function, added NAN error checking on thermocouple, changed profiles to leaded and low temp, changed LED output so HIGH is on
 * 1.23      Took typedef out of enum, moved buttons to D0 & D1 because of I2C conflict, changed I2C library.  Removed debounce for button #2, it was causing problems where btn1 would make btn2 code run
 * 1.24      Updated profile temperatures, moved some code out of loop() into functions, removed 2 line LCD code.  Added seconds and rate to LCD display.  Changed some variables.
 * 1.25      Change pins for Mini-Pro, changed C/Sec to average over 5 seconds
 *******************************************************************************/

#include "Adafruit_MAX31855.h"  // http://github.com/adafruit/Adafruit-MAX31855-library
#include "I2C.h"                // http://github.com/rambo/I2C
#include "PID_v1.h"             // http://github.com/br3ttb/Arduino-PID-Library/
#include "Adafruit_GFX.h"       // https://github.com/adafruit/Adafruit-GFX-Library
#include "SSD1306_I2C_DSS.h"    // For OLED  http://github.com/Scott216/SSD1306_I2C_DSS
// #include "Adafruit_PCD8544"     // http://github.com/adafruit/Adafruit-PCD8544-Nokia-5110-LCD-library


// ***** TYPE DEFINITIONS *****
enum reflowState_t
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_ERROR
};

enum reflowStatus_t
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
};

enum debounceState_t
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} ;

enum printData_t { PRINT_HEADER, PRINT_DATA};
  
// ***** CONSTANTS *****
#define TEMPERATURE_ROOM             50
#define SENSOR_SAMPLING_TIME       1000
#define SOAK_TEMPERATURE_STEP         5
#define SOAK_MICRO_PERIOD          9000
#define DEBOUNCE_PERIOD_MIN          50


// Below temperature constants are for Pb-Free Operation
enum solderProfile_t
{
  LEADED_PROFILE,   // Henkel multicore 583489 (Digikey 82-143-ND), Melts 183°C
  LOWTEMP_PROFILE   // Chip Quik SMDLTLFP (Digikey SMDLTLFP-ND), Melts at 138°C
};

solderProfile_t solderType = LEADED_PROFILE; // used in arrays to choose which profile is being used, default to leaded

// Define profiles.  First element is leaded, 2nd is low temp
// Solder Info:  https://tds.us.henkel.com/NA/UT/HNAUTTDS.nsf/web/B5E62B506B289ABA882571870000DBBB/$File/MP200.pdf
// Leaded Solder: Henkel multicore 583489 (Digikey 82-143-ND)
// Data Sheet
// Desired Profile:
//   Preheat:     120 sec, 130-165°C
//   Soaking:     120 sec, 130-165°C
//   Reflow:    30-75 sec, 205-225°C
//   Cooling:     25+ sec, 100°C

// Low Temp Solder : Chip Quik SMDLTLFP Sn42Bi58 (42/58) (Digikey SMDLTLFP-ND)
// Data Sheet http://bit.ly/1cRVSGo
// Desired Profile:
//   Preheat:     120 sec, 90-120°C
//   Soaking:   60-90 sec, 90-130°C
//   Reflow:    30-60 sec, 158-165°C
//   Cooling:     15+ sec, 100°C
//                             Leaded  Low Temp
int TEMPERATURE_SOAK_MIN[] =   {130,       90};
int TEMPERATURE_SOAK_MAX[] =   {165,      130};
int TEMPERATURE_REFLOW_MAX[] = {220,      165};
int TEMPERATURE_COOL_MIN[] =   {100,      100};
// Controller will heat up to TEMPERATURE_SOAK_MIN, this is the preheat stage.  When it reaches this temperature, the soak stage starts.
// Soak stage increases setpoint temp 5°C (SOAK_TEMPERATURE_STEP) every 9 seconds (SOAK_MICRO_PERIOD).  Then goes into reflow stage
// Controller will then heat until TEMPERATURE_REFLOW_MAX is reached, then cooling stage starts.
// Cooling continues until temp is TEMPERATURE_COOL_MIN


// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT   300
#define PID_KI_PREHEAT  0.05
#define PID_KD_PREHEAT   400
// ***** SOAKING STAGE *****
#define PID_KP_SOAK      300
#define PID_KI_SOAK     0.05
#define PID_KD_SOAK      250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW    300
#define PID_KI_REFLOW   0.05
#define PID_KD_REFLOW    350
#define PID_SAMPLE_TIME 1000

// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {"Ready", "Pre-heat", "Soak", "Reflow", "Cool", "Complete", "Error" };

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {140,146,146,140,128,128,128,128};

// Pro Mini Pin Assignemts
// Thermocouple Breakout
const int thermocoupleSO =  A2;
const int thermocoupleCS =  A1;
const int thermocoupleCLK = A0;

// I/O
const int ledRed =            4;  // Blinks when oven is on
const int ledGreen =          8;  // Indicates reflow has completed
const int cycleStartStopBtn = 2;  // Start-stop button, on interrupt pin (in case you want to use them
const int changeProfileBtn  = 3;  // Change profile button, , on interrupt pin (in case you want to use them
const int fan =               5;  // PWM pin
const int heater =            6;  // SSR Relay
const int buzzer =            7;

// LCD Display
const int dispClk =            9;
const int dispDin =           10;
const int dispDC =            11;
const int dispCS =            12;
const int dispRst =           13;
 
// Pins A4 & A5 are used by I2C
// Spare D0, D1, A3, A6, A7


/*  Leonardo Pins
// ***** PIN ASSIGNMENT *****
const int thermocoupleSO =  A5;
const int thermocoupleCS =  A4;
const int thermocoupleCLK = A3;

// Pins D2 & D3 are used by I2C

const int ledRed =           13;  // Blinks when oven is on
const int ledGreen =         A0;  // Indicates reflow has completed
const int cycleStartStopBtn = 0;    // Start-stop button
const int changeProfileBtn  = 1;    // change profile button
const int fan =               4;
const int heater =            5;
const int buzzer =            6;
const int oledReset =         7;
*/

// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;

const int windowSize = 2000;     // Set PID window size
uint32_t windowStartTime;
uint32_t nextCheck;
uint32_t nextRead;
uint32_t timerSoak;
uint32_t buzzerPeriod;

reflowState_t   reflowState;           // Reflow oven controller state machine state variable
reflowStatus_t  reflowStatus;          // Reflow oven controller status
debounceState_t debounceState;         // Button debounce state machine state variable
uint32_t        lastDebounceTime;      // Button debounce timer
bool            cycleStartStopStatus;  // Button press status
int             timerSeconds;          // Seconds timer

// Specify PID control interface
PID reflowOvenPID(&input, &output, &setpoint, PID_KP_PREHEAT, PID_KI_PREHEAT, PID_KD_PREHEAT, DIRECT);

Adafruit_SSD1306 display(dispRst);
// Adafruit_PCD8544 display = Adafruit_PCD8544(dispClk, dispDin, dispDC, dispCS, dispRst);


// Specify thermocouple interface
Adafruit_MAX31855 thermocouple(thermocoupleCLK, thermocoupleCS, thermocoupleSO);


// Function Prototypes
void getTemperature();
void checkButtons();
void lcdDisplay(int line, const char *lcdText);
void printData(printData_t whatToPrint);


//==============================================================================================================================
//==============================================================================================================================
void setup()
{

  // Configure I/O pins
  pinMode(heater,   OUTPUT);
  pinMode(fan,      OUTPUT);
  pinMode(buzzer,   OUTPUT);
  pinMode(ledRed,   OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(cycleStartStopBtn, INPUT_PULLUP);
  pinMode(changeProfileBtn,  INPUT_PULLUP);
  
  Serial.begin(9600);

  // Start-up splash
  digitalWrite(ledRed,   HIGH);
  digitalWrite(ledGreen, HIGH);
  digitalWrite(buzzer, HIGH);
  lcdDisplay(-1, "");  // initialze display
  lcdDisplay(0, "Reflow");
  lcdDisplay(1, "Oven 1.25");
  digitalWrite(buzzer, LOW);
  delay(1500);
  digitalWrite(ledRed,   LOW);
  digitalWrite(ledGreen, LOW);
  
  nextCheck = millis();  // Initialize time keeping variable
  nextRead =  millis();  // Initialize thermocouple reading varible

  Serial.println("Finished setup");
  
}  // setup()


//==============================================================================================================================
//==============================================================================================================================
void loop()
{
  
  // Initialize static variables
  static reflowState_t prevReflowState = REFLOW_STATE_IDLE;  // Use to see when reflow state changes
  static float         prevInput = input;                    // Previous temperature used for temp rise / second calc
  static int           stageTimeSeconds = 0;                 // Seconds in current stage
  static float riseRate;
  static int iRiseRate;
  static int fRiseRate;
  
  getTemperature();
  
  if (millis() > nextCheck)
  {
    // Check input in the one second
    nextCheck = millis() + 1000;
    
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Toggle red LED as system heart beat
      digitalWrite(ledRed, !(digitalRead(ledRed)));
      // Increase seconds timer for reflow curve analysis
      timerSeconds++;
      stageTimeSeconds++;  // number of seconds in current stage
      
      printData(PRINT_DATA);
    }
    else
    { digitalWrite(ledRed, LOW); }
    
    // Display current system state
    if (input > TEMPERATURE_ROOM && reflowState == REFLOW_STATE_IDLE)
    { lcdDisplay(0, "Not Ready"); }   // Oven needs to cool down before reflow cycle can begin
    else
    { lcdDisplay(0, lcdMessagesReflowStatus[reflowState]); }
    
    
    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR)
    {  lcdDisplay(1, "TC Error!"); } // // No thermocouple wire connected
    else
    {
      // Print current temperature
      char buf[20];
      sprintf(buf, "%d C", (int) input);
      lcdDisplay(1, buf);
      
      // When stage changes, reset seconds for current stage
      if ( prevReflowState != reflowState )
      {
        stageTimeSeconds = 0;
        prevReflowState = reflowState;
      }
      
      // When oven is running, also display time and temperature rise rate (averaged over 5 seconds)
      
      if ( reflowStatus == REFLOW_STATUS_ON )
      {
        // Update temperature rate every 5 seconds
        if ( stageTimeSeconds % 5 == 0)
        {
          riseRate = (input - prevInput) / 5.0;
          iRiseRate = (int) riseRate;
          fRiseRate = (riseRate - iRiseRate ) * 100;
          prevInput = input;
        }
        sprintf(buf, "%d.%d C/Sec", iRiseRate, fRiseRate );
        lcdDisplay(2, buf);

        sprintf(buf, "%d sec", stageTimeSeconds );
        lcdDisplay(3, buf);
      }
    }
  }  // end nextCheck
  
  
  // Reflow oven controller state machine
  switch (reflowState)
  {
    case REFLOW_STATE_IDLE:
      if ( input > TEMPERATURE_ROOM )
      { digitalWrite(fan, HIGH); } // Turn fan on if temp is > room temperature
      else
      { digitalWrite(fan, LOW); }
      
      // If button is pressed to start reflow process
      if ( cycleStartStopStatus == true )
      {
        // Ensure current temperature is comparable to room temperature
        if (input <= TEMPERATURE_ROOM)
        {
          printData(PRINT_HEADER);
          // Intialize seconds timer for serial debug information
          timerSeconds = 0;
          // Initialize PID control window starting time
          windowStartTime = millis();
          // Ramp up to minimum soaking temperature
          setpoint = TEMPERATURE_SOAK_MIN[solderType];
          // Tell the PID to range between 0 and the full window size
          reflowOvenPID.SetOutputLimits(0, windowSize);
          reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
          // Turn the PID on
          reflowOvenPID.SetMode(AUTOMATIC);
          // Proceed to preheat stage
          reflowState = REFLOW_STATE_PREHEAT;
        }
        else // temperature inside too high, do not risk thermal shock!
        {
          digitalWrite(buzzer, HIGH);
          lcdDisplay(0, "Temp Too High");
          lcdDisplay(1, "Let Cool");
          digitalWrite(buzzer, LOW);
          delay(2000);
        }
      }
      break;
      
    case REFLOW_STATE_PREHEAT:
      reflowStatus = REFLOW_STATUS_ON;
      digitalWrite(fan, HIGH);
      // If minimum soak temperature is achieve
      if (input >= TEMPERATURE_SOAK_MIN[solderType])
      {
        // Chop soaking period into smaller sub-period
        timerSoak = millis() + SOAK_MICRO_PERIOD;
        // Set less agressive PID parameters for soaking ramp
        reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_SOAK_MIN[solderType] + SOAK_TEMPERATURE_STEP;
        // Proceed to soaking state
        reflowState = REFLOW_STATE_SOAK;
      }
      break;
      
    case REFLOW_STATE_SOAK:
      digitalWrite(fan, HIGH);
      // If micro soak temperature is achieved
      if (millis() > timerSoak)
      {
        timerSoak = millis() + SOAK_MICRO_PERIOD;  // Increase setpoint temp in small steps
        // Increment micro setpoint
        setpoint += SOAK_TEMPERATURE_STEP;
        if (setpoint > TEMPERATURE_SOAK_MAX[solderType])
        {
          // Set agressive PID parameters for reflow ramp
          reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
          // Ramp up to first section of soaking temperature
          setpoint = TEMPERATURE_REFLOW_MAX[solderType];
          // Proceed to reflowing state
          reflowState = REFLOW_STATE_REFLOW;
        }
      }
      break;
      
    case REFLOW_STATE_REFLOW:
      digitalWrite(fan, HIGH);
      // Avoid hovering at peak temperature for too long
      // Crude method that works like a charm and safe for the components
      if (input >= (TEMPERATURE_REFLOW_MAX[solderType] - 5))
      {
        // Set PID parameters for cooling ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);  // same params as reflow
        // Ramp down to minimum cooling temperature
        setpoint = TEMPERATURE_COOL_MIN[solderType];
        // Proceed to cooling state
        reflowState = REFLOW_STATE_COOL;
      }
      break;
      
    case REFLOW_STATE_COOL:
      // If minimum cool temperature is achieve
      if (input <= TEMPERATURE_COOL_MIN[solderType])
      {
        digitalWrite(fan, HIGH);
        // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 1000;
        // Turn on buzzer and green LED to indicate completion
        digitalWrite(ledGreen, HIGH);
        digitalWrite(buzzer,   HIGH);
        // Turn off reflow process
        reflowStatus = REFLOW_STATUS_OFF;
        // Proceed to reflow Completion state
        reflowState = REFLOW_STATE_COMPLETE;
      }
      break;
      
    case REFLOW_STATE_COMPLETE:
      if (millis() > buzzerPeriod)
      {
        digitalWrite(buzzer,   LOW);
        digitalWrite(ledGreen, LOW);
        reflowState = REFLOW_STATE_IDLE; // Reflow process ended
      }
      break;
      
    case REFLOW_STATE_ERROR:
      // thermocouple error
      if (isnan(input))
      { reflowState = REFLOW_STATE_ERROR; }   // Wait until thermocouple wire is connected
      else
      { reflowState = REFLOW_STATE_IDLE; }    // Clear to perform reflow process
      break;
  }  // end switch (reflowState)
  
  
  checkButtons();
  
  
  // PID computation and heater control
  unsigned long now;
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    now = millis();
    reflowOvenPID.Compute();
    
    if((now - windowStartTime) > windowSize)
    {   windowStartTime += windowSize;  }  // Time to shift the Relay Window
    if( output > (now - windowStartTime) )
    { digitalWrite(heater, HIGH); }
    else 
    { digitalWrite(heater, LOW); }
  }
  // Reflow oven process is off, ensure oven is off
  else 
  { digitalWrite(heater, LOW); }
  
} // end loop()


//==============================================================================================================================
// get temperature inside reflow oven
//==============================================================================================================================
void getTemperature()
{
  // Time to read thermocouple?
  if (millis() > nextRead)
  {
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    // Read current temperature, loop until you get a valid reading
    uint32_t waitForValidTempTimer = millis() + 30000;
    do
    {
      input = thermocouple.readCelsius();
      if ( isnan(input) )
      { delay(150); Serial.println("Got nan"); }
    }
    while ( isnan(input) && (millis() < waitForValidTempTimer) );
    
    // If thermocouple is not connected
    if (isnan(input) )
    {
      // Illegal operation without thermocouple
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;
      Serial.println("Thermocouple Error");
    }
  }

}

//==============================================================================================================================
// Check the two pushbuttons: Oven Start/Stop, Change Profile
//==============================================================================================================================
void checkButtons()
{
  // If button is pressed
  if ( cycleStartStopStatus == true )
  {
    // If currently reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Button press is for cancelling
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;
      // Reinitialize state machine
      reflowState = REFLOW_STATE_IDLE;
    }
  }
  
  if ( digitalRead(changeProfileBtn) == LOW )
  {
    // Switch profile
    if (solderType == LOWTEMP_PROFILE)
    {
      solderType = LEADED_PROFILE;
      lcdDisplay(0, "Changing to");
      lcdDisplay(1, "Leaded Profile");
      delay(2000);
    }
    else
    {
      solderType = LOWTEMP_PROFILE;
      lcdDisplay(0, "Changing to");
      lcdDisplay(1, "Low Temp Profile");
      delay(2000);
    }
  }
  
  // Simple button debounce state machine, for cycleStartStop button only
  // Change profile doesn't need debounce because there is delay in displaying profile change info
  switch (debounceState)
  {
    case DEBOUNCE_STATE_IDLE:
      // No valid button press
      cycleStartStopStatus = false;
      // If button is pressed
      if ( digitalRead(cycleStartStopBtn) == LOW )
      {
        // Intialize debounce counter
        lastDebounceTime = millis();
        // Proceed to check validity of button press
        debounceState = DEBOUNCE_STATE_CHECK;
      }
      break;
      
    case DEBOUNCE_STATE_CHECK:
      // If button is still pressed
      if ( digitalRead(cycleStartStopBtn) == LOW )
      {
        // If minimum debounce period is completed
        if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN)
        {  debounceState = DEBOUNCE_STATE_RELEASE; } // Proceed to wait for button release
      }
      else // False trigger
      { debounceState = DEBOUNCE_STATE_IDLE; } // Reinitialize button debounce state machine
      break;
      
    case DEBOUNCE_STATE_RELEASE:
      if (digitalRead(cycleStartStopBtn) == HIGH)
      {
        // Valid button press
        cycleStartStopStatus = true;
        debounceState = DEBOUNCE_STATE_IDLE;  // Reinitialize button debounce state machine
      }
      break;
  }  // end switch(debounceState)
  
}  // end checkButtons()



//==============================================================================================================================
// LCD display code is in it's own function so program can more easily switch between displays
//==============================================================================================================================
void lcdDisplay(int line, const char *lcdText)
{

  if ( line > 2 )  // srg - remove this after you get new display
  {return;}
  
  
  if (line == -1)
  {
    // Initialize I2C communication
    I2c.begin();
    I2c.timeOut(30000);  // set I2C timeout to 30 seconds
    I2c.pullup(0);       // disable internal pullup resistors on I2C pins
    
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
    display.setTextSize(1);
    display.setTextColor(WHITE);
    return;
  }

  if (line == 0)
  { display.clearDisplay(); }
  
  display.setCursor(1, line * 11);
  display.print(lcdText);
  display.display(); // show splashscreen


} // end lcdDisplay()


//==============================================================================================================================
//  Print data to serial printer
//==============================================================================================================================
void printData(printData_t whatToPrint)
{
  
  if (whatToPrint == PRINT_HEADER )
  {
    Serial.println("Time\tSetpoint\tTemp      \tStage\theater\tfan\tSolder");
  }
  else
  {
    // Send temperature and time stamp to serial monitor, this can be used to graph profile
    Serial.print(timerSeconds);
    Serial.print("\t");
    Serial.print(setpoint);
    Serial.print("   \t");
    Serial.print(input);
    Serial.print("   \t");
    Serial.print(reflowState);
    Serial.print("\t");
    Serial.print(digitalRead(heater));
    Serial.print("\t");
    Serial.print(digitalRead(fan));
    Serial.print("\t");
    Serial.print(solderType);
    Serial.println("");
    
  }

}  // end printData()


// Uses Arduino ProMini 3.3 volts

// to do
// had hall effect sensor to detect fan current and shut down if too high - maybe
// put a switch on the fan - maybel
// PCB Circuit swaps D2 and D4
// Put limits on setpoints so they cont go to high if there is a problem

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
 * Kester's standard leaded profile http://www.kester.com/kester-content/uploads/2013/06/Standard_Profile.pdf
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
 * 1.25      Change pins for Pro-Mini, changed C/Sec to average over 5 seconds
 * 1.26      Changed display to Nokia 5110, tested for negative rate change so it will display properly
 * 1.27      Changed I/O pins, added pin for LED backlight
 * 1.28      When calling for heat, top and bottom heaters will altenrate coming on.
 * 1.29      Reversed LED output (Low = on), made display text 3 lines.  Changed fan from digital to PWM
 * 1.30      Buzzer sounds and each stage in reflow.  Adjusted heat on to better balance top and bottom heaters, display profile when idle
 * 1.31      Added mircostepping temp target to pre-heat stage, show setpoint temp in display
 * 1.32      Fan was not turning off after cool period was over, should be fixed now 
 * 1.33      Changed contrast, moved inputs to match new PCB
 * 1.34      Adjusted contrast
 * 1.35      Adjusted contrast again, added more beeps when cooling stage starts 
 * 1.36      Changed constast from 40 to 50.  Added FAN_OFF constant
 * 1.37      Added max setpoint to prevent run-away temp, renamed global variables
 
 *******************************************************************************/
#define VERSION  "ver 1.37"

#include "Adafruit_MAX31855.h"  // http://github.com/adafruit/Adafruit-MAX31855-library
#include "PID_v1.h"             // http://github.com/br3ttb/Arduino-PID-Library/
#include "Adafruit_GFX.h"       // https://github.com/adafruit/Adafruit-GFX-Library
#include "Adafruit_PCD8544.h"   // http://github.com/adafruit/Adafruit-PCD8544-Nokia-5110-LCD-library


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
#define TEMPERATURE_ROOM             50   // Temp must be below this to start a new reflow
#define SENSOR_SAMPLING_TIME       1000
#define PREHT_TEMPERATURE_STEP        5   // increment pre-heat temp in 5 degree steps
#define SOAK_TEMPERATURE_STEP         5   // increment soak temp in 5 degree steps
#define PREHT_MICRO_PERIOD         4250   // set new pre-heat target temp every 4-1/4 seconds
#define SOAK_MICRO_PERIOD          9000   // set new soak target temp evey 9 seconds
#define DEBOUNCE_PERIOD_MIN          50
#define MAX_ALLOWED_TEMP            250  // Max temp oven is allowed to go, Centegrade

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

// Low Temp Solder: Chip Quik SMDLTLFP Sn42Bi58 (42/58) (Digikey SMDLTLFP-ND)
// Low Temp Profile:
//   Preheat:     120 sec, 90-120°C
//   Soaking:   60-90 sec, 90-130°C
//   Reflow:    30-60 sec, 158-165°C
//   Cooling:     15+ sec, 100°C
//
// Leaded solder: Multicore 583489, Digikey 82-143-ND,
// Leaded Profile
//   Preheat:    60-90 sec, 130-165°C
//   Soaking:   60-120 sec, 130-165°C
//   Reflow:    90-120 sec, 205-225°C
//   Cooling:      15+ sec, 100°C
// Data Sheet  http://bit.ly/1cRVSGo
//                             Leaded   Low Temp
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

// Pro Mini Pin Assignemts
// Thermocouple Breakout
const int thermocoupleSO =  A3;
const int thermocoupleCS =  A2;
const int thermocoupleCLK = A1;

// I/O
const int spareBtn =          1;  // Spare pushbutton
const int changeProfileBtn  = 4;  // Change profile button, , on interrupt pin (in case you want to use them
const int cycleStartStopBtn = 3;  // Start-stop button, on interrupt pin (in case you want to use them
const int grnLED =            2;  // Blinks when oven is on
const int buzzer =            5;  // PWM pin - useful if you want to use a piezo buzzer
const int fan =               6;  // PWM pin
const int heater_top =        7;
const int heater_bottom =     8;

// LCD Display
const int dispLED =            9; // pull low to turn backlight off, PWM pin
const int dispClk =           A0;
const int dispDin =           13;
const int dispDC =            12;
const int dispCS =            11;
const int dispRst =           10;

// Spare D0, A4, A5, A6, A7

const int ledOn = LOW;   // green LEDs on original circuit board are on when LOW (uses PNP transistor)
const int ledOff = HIGH;
const byte FAN_ON = 255;
const byte FAN_OFF =  0; 

// ***** PID CONTROL VARIABLES *****
double g_pid_setpoint;
double g_pid_input;
double g_pid_output;

const int windowSize = 2000;     // Set PID window size
uint32_t windowStartTime;
uint32_t nextCheck;
uint32_t nextRead;
uint32_t timerPreHeat;           // Timer used for micro-stepping setpoint in Pre-heat stage
uint32_t timerSoak;              // Timer used for micro-stepping setpoint in Soak stage
uint32_t buzzerPeriod;           // Time for buzzer


reflowState_t   reflowState;           // Reflow oven controller state machine state variable
reflowStatus_t  reflowStatus;          // Reflow oven controller status
debounceState_t debounceState;         // Button debounce state machine state variable
uint32_t        lastDebounceTime;      // Button debounce timer
bool            cycleStartStopStatus;  // Button press status
int             timerSeconds;          // Seconds timer

// Specify PID control interface
PID reflowOvenPID(&g_pid_input, &g_pid_output, &g_pid_setpoint, PID_KP_PREHEAT, PID_KI_PREHEAT, PID_KD_PREHEAT, DIRECT);

// Adafruit_SSD1306 display(dispRst);
Adafruit_PCD8544 display = Adafruit_PCD8544(dispClk, dispDin, dispDC, dispCS, dispRst);


// Specify thermocouple interface
Adafruit_MAX31855 thermocouple(thermocoupleCLK, thermocoupleCS, thermocoupleSO);


// Function Prototypes
void getTemperature();
void checkButtons();
void lcdDisplay(int line, const char *lcdText);
void setHeatOn();
void setHeatOff();
void printData(printData_t whatToPrint);


//==============================================================================================================================
//==============================================================================================================================
void setup()
{
  // Configure I/O pins
  pinMode(dispLED,                 OUTPUT);  // pwm output
  pinMode(heater_top,              OUTPUT);
  pinMode(heater_bottom,           OUTPUT);
  pinMode(fan,                     OUTPUT);  // pwm output
  pinMode(buzzer,                  OUTPUT);
  pinMode(grnLED,                  OUTPUT);
  pinMode(cycleStartStopBtn, INPUT_PULLUP);
  pinMode(changeProfileBtn,  INPUT_PULLUP);
  pinMode(spareBtn,          INPUT_PULLUP);
  
  analogWrite(dispLED, 127);  // turn LCD backlignt on at 50%
  
  Serial.begin(9600);

  // Start-up
  lcdDisplay(-1, "");  // initialze display
  lcdDisplay(0, "Reflow Oven");
  lcdDisplay(1, VERSION);
  digitalWrite(buzzer, HIGH);
  delay(500);
  digitalWrite(buzzer, LOW);
  delay(1500);
  
  nextCheck = millis();  // Initialize time keeping variable
  nextRead =  millis();  // Initialize thermocouple reading varible

  Serial.println("Finished setup");
  
}  // end setup()


//==============================================================================================================================
//==============================================================================================================================
void loop()
{
  // Initialize static variables
  static reflowState_t prevReflowState = REFLOW_STATE_IDLE;  // Use to see when reflow state changes
  static float         prevInput = g_pid_input;                    // Previous temperature used for temp rise / second calc
  static int           stageTimeSeconds = 0;                 // Seconds in current stage
  
  getTemperature();
  
  if (millis() > nextCheck)
  {
    // Check input in the one second
    nextCheck = millis() + 1000;
    
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Toggle LED as system heart beat
      digitalWrite(grnLED, !(digitalRead(grnLED)));
      // Increase seconds timer for reflow curve analysis
      timerSeconds++;
      stageTimeSeconds++;  // number of seconds in current stage
      
      printData(PRINT_DATA);
    }
    else
    { digitalWrite(grnLED, ledOff); }
    
    // Display current system state
    if (g_pid_input > TEMPERATURE_ROOM && reflowState == REFLOW_STATE_IDLE)
    { lcdDisplay(0, "Not Ready"); }   // Oven needs to cool down before reflow cycle can begin
    else
    { lcdDisplay(0, lcdMessagesReflowStatus[reflowState]); }
    
    
    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR)
    {  lcdDisplay(1, "TC Error!"); } // No thermocouple wire connected
    else
    {
      char buf[20];
      
      // If idle, display temp and currently selected profile
      if (reflowState == REFLOW_STATE_IDLE) 
      {
        // Display current temperature
        sprintf(buf, "%dC", (int) g_pid_input);
        lcdDisplay(1, buf);
        // display profile
        if (solderType == LOWTEMP_PROFILE)
        { lcdDisplay(2, "Low Temp"); }
        else
        { lcdDisplay(2, "Leaded"); }
        lcdDisplay(3, "Profile");
      }
      else
      { 
        // not idle, display current temp and setpoint temp
        sprintf(buf, "%dC, SP %dC", (int) g_pid_input, (int) g_pid_setpoint );
        lcdDisplay(1, buf);
      }
      // When stage changes, reset seconds for current stage
      if ( prevReflowState != reflowState )
      {
        stageTimeSeconds = 0;
        prevReflowState = reflowState;
      }
      
      // When oven is running, also display time and temperature rise rate (averaged over 5 seconds)
      static float riseRate;
      static int iRiseRate;
      static int fRiseRate;
      static bool isPositiveRate;
      if ( reflowStatus == REFLOW_STATUS_ON )
      {
        // Update temperature rate every 5 seconds,
        // Need check if rate is negative or positive and put a negative in sprintf because if the rate is -0.5, you can't have -0 as the integer portion
        if ( stageTimeSeconds % 5 == 0)
        {
          if ( g_pid_input >= prevInput )
          {
            riseRate = (g_pid_input - prevInput) / 5.0;
            isPositiveRate = true;
          }
          else
          {
            riseRate = (prevInput - g_pid_input) / 5.0;
            isPositiveRate = false;
          }
          iRiseRate = (int) riseRate;
          fRiseRate = (riseRate - iRiseRate ) * 100;
          prevInput = g_pid_input;
        }
        if ( isPositiveRate )
        { sprintf(buf, "%d.%d C/Sec", iRiseRate, fRiseRate ); }
        else
        { sprintf(buf, "-%d.%d C/Sec", iRiseRate, fRiseRate ); }
        
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
      analogWrite(fan, FAN_OFF);
      
      // If button is pressed to start reflow process
      if ( cycleStartStopStatus == true )
      {
        // Ensure current temperature is comparable to room temperature
        if (g_pid_input <= TEMPERATURE_ROOM)
        {
          digitalWrite(buzzer, HIGH);
          delay(100);
          digitalWrite(buzzer, LOW);
          printData(PRINT_HEADER);
          // Intialize seconds timer for serial debug information
          timerSeconds = 0;
          // Initialize PID control window starting time
          windowStartTime = millis();
          // Ramp up to first section of pre-heat temperature
          g_pid_setpoint = g_pid_input + PREHT_TEMPERATURE_STEP;
          
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
      if (millis() > buzzerPeriod)
      { digitalWrite(buzzer, LOW); }
      analogWrite(fan, FAN_ON);
      

      // If micro pre-heat temperature is achieved
      if (millis() > timerPreHeat)
      {
        timerPreHeat = millis() + PREHT_MICRO_PERIOD;  // Increase setpoint temp in small steps
        // Increment micro setpoint
        g_pid_setpoint += PREHT_TEMPERATURE_STEP;
        if ( g_pid_setpoint > MAX_ALLOWED_TEMP )
        { g_pid_setpoint = MAX_ALLOWED_TEMP; } 
        
        // If minimum soak temperature is achieved
        if (g_pid_input >= TEMPERATURE_SOAK_MIN[solderType])
        {
          // Chop soaking period into smaller sub-period
          timerSoak = millis() + SOAK_MICRO_PERIOD; // First target temp for soak cycle with is about to start
          // Set less agressive PID parameters for soaking ramp
          reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
          // Ramp up to first section of soaking temperature
          g_pid_setpoint = TEMPERATURE_SOAK_MIN[solderType] + SOAK_TEMPERATURE_STEP;
          // Proceed to soaking state
          reflowState = REFLOW_STATE_SOAK;
          digitalWrite(buzzer,  HIGH);
          buzzerPeriod = millis() + 100;
        }
      }
      break;
      
    case REFLOW_STATE_SOAK:
      if (millis() > buzzerPeriod)
      { digitalWrite(buzzer, LOW); }
      analogWrite(fan, FAN_ON);
      // If micro soak temperature is achieved
      if (millis() > timerSoak)
      {
        timerSoak = millis() + SOAK_MICRO_PERIOD;  // Increase setpoint temp in small steps
        // Increment micro setpoint
        g_pid_setpoint += SOAK_TEMPERATURE_STEP;
        if ( g_pid_setpoint > MAX_ALLOWED_TEMP )
        { g_pid_setpoint = MAX_ALLOWED_TEMP; } 
        
        if (g_pid_setpoint > TEMPERATURE_SOAK_MAX[solderType])
        {
          // Set agressive PID parameters for reflow ramp
          reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
          // Ramp up to first section of soaking temperature
          g_pid_setpoint = TEMPERATURE_REFLOW_MAX[solderType];
          // Proceed to reflowing state
          reflowState = REFLOW_STATE_REFLOW;
          digitalWrite(buzzer, HIGH);
          buzzerPeriod = millis() + 100;
        }
      }
      break;
      
    case REFLOW_STATE_REFLOW:
      if (millis() > buzzerPeriod)
      { digitalWrite(buzzer, LOW); }
      analogWrite(fan, FAN_ON);
      // Avoid hovering at peak temperature for too long
      // Crude method that works like a charm and safe for the components
      if (g_pid_input >= (TEMPERATURE_REFLOW_MAX[solderType] - 5))
      {
        // Set PID parameters for cooling ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);  // same params as reflow
        // Ramp down to minimum cooling temperature
        g_pid_setpoint = TEMPERATURE_COOL_MIN[solderType];
        // Proceed to cooling state
        reflowState = REFLOW_STATE_COOL;
        digitalWrite(buzzer, HIGH);
        delay(100);
        digitalWrite(buzzer, LOW);
        delay(100);
        digitalWrite(buzzer, HIGH);
        delay(100);
        digitalWrite(buzzer, LOW);
        delay(100);
        digitalWrite(buzzer, HIGH);
        buzzerPeriod = millis() + 100;
      }
      break;
      
    case REFLOW_STATE_COOL:
      if (millis() > buzzerPeriod)
      { digitalWrite(buzzer, LOW); }
      // If minimum cool temperature is achieved
      if (g_pid_input <= TEMPERATURE_COOL_MIN[solderType])
      {
        analogWrite(fan, FAN_OFF);
        // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 1000;
        // Turn on buzzer
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
        digitalWrite(buzzer, LOW);
        reflowState = REFLOW_STATE_IDLE; // Reflow process ended
      }
      break;
      
    case REFLOW_STATE_ERROR:
      // thermocouple error
      if (isnan(g_pid_input))
      { reflowState = REFLOW_STATE_ERROR; }   // Wait until thermocouple wire is connected
      else
      { reflowState = REFLOW_STATE_IDLE; }    // Clear to perform reflow process
      analogWrite(fan, FAN_OFF);
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
    if( g_pid_output > (now - windowStartTime) )
    { setHeatOn(); }
    else 
    { setHeatOff(); }
  }
  // Reflow oven process is off, ensure oven is off
  else 
  { setHeatOff(); }
  
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
      g_pid_input = thermocouple.readCelsius();
      if ( isnan(g_pid_input) )
      { 
        delay(150); 
        Serial.println("Got nan"); 
      }
    }
    while ( isnan(g_pid_input) && (millis() < waitForValidTempTimer) );
    
    // If thermocouple is not connected
    if (isnan(g_pid_input) )
    {
      // Illegal operation without thermocouple
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;
      Serial.println("Thermocouple Error");
    }
  }

}  // end getTemperature()

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
      lcdDisplay(1, "Leaded");
      lcdDisplay(2, "Profile");
      delay(2000);
    }
    else
    {
      solderType = LOWTEMP_PROFILE;
      lcdDisplay(0, "Changing to");
      lcdDisplay(1, "Low Temp");
      lcdDisplay(2, "Profile");
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

  // Initialize display
  if (line == -1)
  {
    display.begin();
    display.setTextSize(1);
    display.setTextColor(BLACK);
    display.setContrast(50);
    return;
  }
  
  // Clear the display 
  if (line == 0)
  { display.clearDisplay(); }
  
  display.setCursor(0, line * 11);
  display.print(lcdText);
  display.display();

} // end lcdDisplay()


//==============================================================================================================================
// When heat is called for, alternate between top and bottom heat every 1/2 second
//==============================================================================================================================
void setHeatOn()
{
  static uint32_t flipFlopTimer = millis(); // initialize timer

  if (long(millis() - flipFlopTimer) > 0)
  {
     flipFlopTimer = millis() + 500;
    if (digitalRead(heater_top) == LOW)
    {
      digitalWrite(heater_top, HIGH);
      digitalWrite(heater_bottom, LOW);
    }
    else
    {
      digitalWrite(heater_top, LOW);
      digitalWrite(heater_bottom, HIGH);
    }
  }
  
}  // end setHeatOnIO()


//==============================================================================================================================
//  Turn heater off
//==============================================================================================================================
void setHeatOff()
{
  digitalWrite(heater_top, LOW);
  digitalWrite(heater_bottom, LOW);
  
} // end setHeatOff()

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
    Serial.print(g_pid_setpoint);
    Serial.print("   \t");
    Serial.print(g_pid_input);
    Serial.print("   \t");
    Serial.print(reflowState);
    Serial.print("\t");
    Serial.print(digitalRead(heater_top));
    Serial.print("\t");
    Serial.print(analogRead(fan));
    Serial.print("\t");
    Serial.print(solderType);
    Serial.println("");
  }

}  // end printData()


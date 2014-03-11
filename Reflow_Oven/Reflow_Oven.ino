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
 *                                                                Time (Seconds)
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
 * ==========================================
 * Limor Fried of Adafruit (www.adafruit.com)
 * ==========================================
 * Author of Arduino MAX6675 library. Adafruit has been the source of tonnes of
 * tutorials, examples, and libraries for everyone to learn.
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
 * 1.23      Took typedef out of enum, moved buttons to D0 & D1 because of I2C conflict, changed I2C library.  Removed debounce for button #2, it was causing wrong code to run
 
 
 *******************************************************************************/

// #include <LiquidCrystal.h>      // http://arduino.cc/en/Reference/LiquidCrystal
#include "Adafruit_MAX31855.h"  // http://github.com/adafruit/Adafruit-MAX31855-library
#include "PID_v1.h"             // http://github.com/br3ttb/Arduino-PID-Library/
#include <Adafruit_GFX.h>
#include <I2C.h>             // http://github.com/rambo/I2C
#include <SSD1306_I2C_DSS.h> // For OLED  https://github.com/Scott216/SSD1306_I2C_DSS

// Choose output screen
#define SCREENTYPE_OLED32X128
// #define SCREENTYPE_TWOLINE
// #define SCREENTYPE_SERIAL
// #define SCREENTYPE_NODISPLAY

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

// ***** CONSTANTS *****
#define TEMPERATURE_ROOM             50
#define SENSOR_SAMPLING_TIME       1000
#define SOAK_TEMPERATURE_STEP         5
#define SOAK_MICRO_PERIOD          9000
#define DEBOUNCE_PERIOD_MIN          50


// Below temperature constants are for Pb-Free Operation
enum solderProfile_t
{
  LEADED_PROFILE,   // Henkel multicore 583489 (Digikey 82-143-ND), Melts 183 C
  LOWTEMP_PROFILE   // Chip Quik SMDLTLFP (Digikey SMDLTLFP-ND), Melts at 138 C
};

solderProfile_t ProfileToUse = LEADED_PROFILE; // used in arrays to choose which profile is being used
// Define profiles.  First element is leaded, 2nd is low temp
int TEMPERATURE_SOAK_MIN[] =   {100,  90};
int TEMPERATURE_SOAK_MAX[] =   {150, 120};
int TEMPERATURE_REFLOW_MAX[] = {215, 175};
int TEMPERATURE_COOL_MIN[] =   {100, 100};



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
const char* lcdMessagesReflowStatus[] =
{
  "Ready",
  "Pre-heat",
  "Soak",
  "Reflow",
  "Cool",
  "Complete",
  "Error"
};

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {140,146,146,140,128,128,128,128};

// ***** PIN ASSIGNMENT *****
//Don't change so use constants to save memory
const int thermocoupleSO =  A5;
const int thermocoupleCS =  A4;
const int thermocoupleCLK = A3;

#ifdef SCREENTYPE_TWOLINE
  const int lcdRs =  7;
  const int lcdE =   8;
  const int lcdD4 =  9;
  const int lcdD5 = 10;
  const int lcdD6 = 11;
  const int lcdD7 = 12;
#endif


// Pins 2 & 3 are used by I2C

const int ledRed =   13;  // Blinks when oven is on
const int ledGreen = A0;  // Indicates reflow has completed
const int cycleStartStopBtn = 0;    // Start-stop button
const int changeProfileBtn  = 1;    // change profile button
const int fan =     4;
const int heater =     5;
const int buzzer =  6;

// ***** Process Control Variables *****
byte LeadState = 0;

// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize = 2000;   // Set PID window size
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long timerSoak;
unsigned long buzzerPeriod;

reflowState_t   reflowState;           // Reflow oven controller state machine state variable
reflowStatus_t  reflowStatus;          // Reflow oven controller status
debounceState_t debounceState;         // Button debounce state machine state variable
long            lastDebounceTime;      // Button debounce timer
bool            cycleStartStopStatus;  // Button press status
int             timerSeconds;          // Seconds timer

// Specify PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

#ifdef SCREENTYPE_TWOLINE
  // Specify LCD interface
  LiquidCrystal lcd(lcdRs, lcdE, lcdD4, lcdD5, lcdD6, lcdD7);
#endif

#ifdef SCREENTYPE_OLED32X128
 const int OLED_RESET = 7;
 Adafruit_SSD1306 display(OLED_RESET);
#endif

// Specify thermocouple interface
Adafruit_MAX31855 thermocouple(thermocoupleCLK, thermocoupleCS, thermocoupleSO);

// Function Prototypes
void lcdDisplay(int line, const char *lcdText);


void setup()
{
  // Make sure everything is off
  digitalWrite(heater, LOW);
  pinMode(heater, OUTPUT);
  digitalWrite(fan, LOW);
  pinMode(fan, OUTPUT);
  digitalWrite(buzzer, LOW);
  pinMode(buzzer, OUTPUT);
  
  // Setup LED and pushbutton pins
  digitalWrite(ledRed, HIGH);
  digitalWrite(ledGreen, HIGH);
  pinMode(ledRed,   OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(cycleStartStopBtn, INPUT_PULLUP);
  pinMode(changeProfileBtn,  INPUT_PULLUP);
  
  Serial.begin(9600);

  // Start-up splash
  digitalWrite(buzzer, HIGH);
  lcdDisplay(-1, "");
  lcdDisplay(0, "Reflow");
  lcdDisplay(1, "Oven 1.23");
  digitalWrite(buzzer, LOW);
  delay(1500);
 
  // Turn off LEDs
  digitalWrite(ledRed,   LOW);
  digitalWrite(ledGreen, LOW);
  
  while (!Serial && millis() < 8000) {}// wait up to 8 seconds for serial monitor t open

  nextCheck = millis();  // Initialize time keeping variable
  nextRead = millis();   // Initialize thermocouple reading varible

  Serial.println("Finished setup");
  
}  // setup()


void loop()
{
  // Current time
  unsigned long now;
  
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
  }  // end nextRead
  
  if (millis() > nextCheck)
  {
    // Check input in the next seconds
    nextCheck = millis() + 1000;
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Toggle red LED as system heart beat
      digitalWrite(ledRed, !(digitalRead(ledRed)));
      // Increase seconds timer for reflow curve analysis
      timerSeconds++;
      
      // Send temperature and time stamp to serial monitor
      Serial.print(timerSeconds);
      Serial.print("\t");
      Serial.print(setpoint);
      Serial.print("\t\t");
      Serial.print(input);
      Serial.print("\t\t");
      Serial.print(reflowState);
      Serial.print("\t");
      Serial.print(digitalRead(heater));
      Serial.print("\t");
      Serial.print(digitalRead(fan));
      Serial.println("");
    }
    else
    {
      // Turn off red LED
      digitalWrite(ledRed, LOW);
    }
    
    // Display current system state
    if (input > TEMPERATURE_ROOM && reflowState == REFLOW_STATE_IDLE)
    { lcdDisplay(0, "Not Ready"); }   // Oven needs to cool down before reflow can begin
    else
    { lcdDisplay(0, lcdMessagesReflowStatus[reflowState]); }
    
    
    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR)
    {  lcdDisplay(1, "No TC!"); } // // No thermocouple wire connected
    else
    {
      // Print current temperature
      char buf[8];
      sprintf(buf, "%d C", (int) input);
      lcdDisplay(1, buf);
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
          // Send header for CSV file
          Serial.println("Time\tSetpoint\tTemp\t\tStage\theater\tfan");
          // Intialize seconds timer for serial debug information
          timerSeconds = 0;
          // Initialize PID control window starting time
          windowStartTime = millis();
          // Ramp up to minimum soaking temperature
          setpoint = TEMPERATURE_SOAK_MIN[ProfileToUse];
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
          lcdDisplay(0, "OvenTemp");
          lcdDisplay(1, "Too High!");
          
          digitalWrite(buzzer, LOW);
          delay(2000);
          digitalWrite(buzzer, HIGH);
          lcdDisplay(0, "Aborting");
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
      if (input >= TEMPERATURE_SOAK_MIN[ProfileToUse])
      {
        // Chop soaking period into smaller sub-period
        timerSoak = millis() + SOAK_MICRO_PERIOD;
        // Set less agressive PID parameters for soaking ramp
        reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_SOAK_MIN[ProfileToUse] + SOAK_TEMPERATURE_STEP;
        // Proceed to soaking state
        reflowState = REFLOW_STATE_SOAK;
      }
      break;
      
    case REFLOW_STATE_SOAK:
      //srg - should there be statement to turn fan on here?
      // If micro soak temperature is achieved
      if (millis() > timerSoak)
      {
        timerSoak = millis() + SOAK_MICRO_PERIOD;
        // Increment micro setpoint
        setpoint += SOAK_TEMPERATURE_STEP;
        if (setpoint > TEMPERATURE_SOAK_MAX[ProfileToUse])
        {
          // Set agressive PID parameters for reflow ramp
          reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
          // Ramp up to first section of soaking temperature
          setpoint = TEMPERATURE_REFLOW_MAX[ProfileToUse];
          // Proceed to reflowing state
          reflowState = REFLOW_STATE_REFLOW;
        }
      }
      break;
      
    case REFLOW_STATE_REFLOW:
      digitalWrite(fan, HIGH);
      // Avoid hovering at peak temperature for too long
      // Crude method that works like a charm and safe for the components
      if (input >= (TEMPERATURE_REFLOW_MAX[ProfileToUse] - 5))
      {
        // Set PID parameters for cooling ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp down to minimum cooling temperature
        setpoint = TEMPERATURE_COOL_MIN[ProfileToUse];
        // Proceed to cooling state
        reflowState = REFLOW_STATE_COOL;
      }
      break;
      
    case REFLOW_STATE_COOL:
      // If minimum cool temperature is achieve
      if (input <= TEMPERATURE_COOL_MIN[ProfileToUse])
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
    if (LeadState == LOWTEMP_PROFILE)
    {
      LeadState = LEADED_PROFILE;
      ProfileToUse = LEADED_PROFILE;
      lcdDisplay(0, "Changing");
      lcdDisplay(1, "to using");
      delay(2000);
      lcdDisplay(0, "Leaded");
      lcdDisplay(1, "Profile");
      delay(2000);
    }
    else
    {
      LeadState = LOWTEMP_PROFILE;
      ProfileToUse = LOWTEMP_PROFILE;
      lcdDisplay(0, "Changing");
      lcdDisplay(1, "to using");
      delay(2000);
      lcdDisplay(0, "Low Temp");
      lcdDisplay(1, "Profile");
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
  
  
  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    now = millis();
    reflowOvenPID.Compute();
    
    if((now - windowStartTime) > windowSize)
    {   windowStartTime += windowSize;  }  // Time to shift the Relay Window
    if(output > (now - windowStartTime)) 
    { digitalWrite(heater, HIGH); }
    else 
    { digitalWrite(heater, LOW); }
  }
  // Reflow oven process is off, ensure oven is off
  else 
  { digitalWrite(heater, LOW); }
  
} // end loop()


// LCD display code is in it's own function so program can more easily switch between displays
void lcdDisplay(int line, const char *lcdText)
{

#ifdef SCREENTYPE_TWOLINE
  // if line is -1 then initialize the LCD display
  if (line == -1)
  {
    lcd.begin(8, 2);
    return;
  }

  if (line == 0)
  { lcd.clear(); }
  
  lcd.setCursor(0, line);
  lcd.print(lcdText);
#endif

#ifdef SCREENTYPE_OLED32X128
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


#endif
  
#ifdef SCREENTYPE_SERIAL
  Serial.println(lcdText);
#endif
  
#ifdef SCREENTYPE_NODISPLAY
  return;
#endif

} // end lcdDisplay()





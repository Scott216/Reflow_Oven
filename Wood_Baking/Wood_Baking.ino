/*
Uses Arduino ProMini 3.3 volts
Use reflow oven to cure wood stabilizer in pen blanks
Bake at 93C for 40 minutes



Change Log
08/14/15  ver 1.00  Initial Version.  Tried to incorporate this into the reflow code, but I couldn't get it to work
08/15/15  ver 1.01  Made change so finished buzzer doesn't keep going off

*/


#define VERSION  "ver 1.00"

#include "Adafruit_MAX31855.h"  // http://github.com/adafruit/Adafruit-MAX31855-library
#include "PID_v1.h"             // http://github.com/br3ttb/Arduino-PID-Library/
#include "Adafruit_GFX.h"       // https://github.com/adafruit/Adafruit-GFX-Library
#include "Adafruit_PCD8544.h"   // http://github.com/adafruit/Adafruit-PCD8544-Nokia-5110-LCD-library



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

const byte BUTTON_PRESSED = LOW;  
const uint16_t DEBOUNCE_TIME = 300;  // button debounce time in mS


const int ledOn =   LOW;   // green LEDs on original circuit board are on when LOW (uses PNP transistor)
const int ledOff = HIGH;
const byte FAN_ON = 255;
const byte FAN_OFF =  0;
const byte OVEN_ON =  1;
const byte OVEN_OFF = 0;


// ***** PID CONTROL VARIABLES *****
double g_setpoint = 93;
double g_input;
double g_output;

// ***** PID PARAMETERS *****
#define PID_KP                 300
#define PID_KI                   0.05
#define PID_KD                 350
#define PID_SAMPLE_TIME       1000
#define SENSOR_SAMPLING_TIME  1000


// Specify PID control interface
PID OvenPID(&g_input, &g_output, &g_setpoint, PID_KP, PID_KI, PID_KD, DIRECT);

// Adafruit_SSD1306 display(dispRst);
Adafruit_PCD8544 display = Adafruit_PCD8544(dispClk, dispDin, dispDC, dispCS, dispRst);

// Specify thermocouple interface
Adafruit_MAX31855 thermocouple(thermocoupleCLK, thermocoupleCS, thermocoupleSO);

// Function Prototypes
void getTemperature();
void setHeatOn();
void setHeatOff();


//==============================================================================================================================
//==============================================================================================================================
void setup()
{
  Serial.begin(9600);

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
  analogWrite(fan, FAN_OFF);

  BuzzerChirp();
  BuzzerChirp();

  display.begin();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setContrast(50);
  display.clearDisplay(); 
  display.setCursor(0, 11);
  display.print("Bake Wood");
  display.setCursor(0, 22);
  display.print(VERSION);
  display.display();
  delay(1500);   // time to read display before it changes 
  
  Serial.println("Bake Wood");
  Serial.println(VERSION);

}  // end setup()


//==============================================================================================================================
//==============================================================================================================================
void loop() 
{
  static byte ovenStatus =   OVEN_OFF; 
  const int windowSize =         2000;     // PID window size
  static uint32_t windowStartTime = 0;
  static uint32_t ovenStartTime = 0;
  
  getTemperature();
  
  // check for start button press
  if (digitalRead(cycleStartStopBtn) == BUTTON_PRESSED  && ovenStatus == OVEN_OFF )
  {
    ovenStatus = OVEN_ON;
    windowStartTime = millis();  // Initialize PID control window starting time
    // Tell the PID to range between 0 and the full window size
    OvenPID.SetOutputLimits(0, windowSize);
    OvenPID.SetSampleTime(PID_SAMPLE_TIME);
    OvenPID.SetMode(AUTOMATIC);  // Turn the PID on
    ovenStartTime = millis(); 
    BuzzerChirp();
    delay(DEBOUNCE_TIME);
  }
  
  // check for button press to stop oven
  if (digitalRead(cycleStartStopBtn) == BUTTON_PRESSED  && ovenStatus == OVEN_ON )
  {
    ovenStatus = OVEN_OFF;
    BuzzerChirp();
    delay(DEBOUNCE_TIME);
  }

  // Reached end of baking cycle
  if ((long)(millis() - ovenStartTime) > 2400000UL && (ovenStatus == OVEN_ON))
  {
    ovenStatus = OVEN_OFF;
    display.clearDisplay(); 
    display.setCursor(0, 11);
    display.print("Finished!");
    display.display();
    BuzzerChirp();
    delay(200);
    BuzzerChirp();
    delay(200);
    BuzzerChirp();
    delay(6000);    
  }
  

  char buf[20];
  if (ovenStatus == OVEN_OFF)
  {
    sprintf(buf, "%dC", (int) g_input);  // current temperature
    
    display.clearDisplay(); 
    display.setCursor(0, 11);
    display.print("Ready");
    display.setCursor(0, 22);
    display.print(buf);
    display.display();
  }

  if (ovenStatus == OVEN_ON)
  {
    sprintf(buf, "%dC  %d Min", (int) g_input, (int) ((millis() - ovenStartTime)/60000));  // current temperature
    
    display.clearDisplay(); 
    display.setCursor(0, 11);
    display.print("Oven On");
    display.setCursor(0, 22);
    display.print(buf);
    display.display();
  }

  // PID computation and heater control
  unsigned long now;
  if (ovenStatus == OVEN_ON)
  {
    now = millis();
    OvenPID.Compute();
    
    if((now - windowStartTime) > windowSize)
    {   windowStartTime += windowSize;  }  // Time to shift the Relay Window
    if( g_output > (now - windowStartTime) )
    { setHeatOn(); }
    else 
    { setHeatOff(); }
  }
  // Reflow oven process is off, ensure oven is off
  else 
  { setHeatOff(); }
  
  
  // Shutdown if temp is too high
  if ( g_input > 120 )
  { 
    sprintf(buf, "%dC", (int) g_input);  // current temperature
    ovenStatus = OVEN_OFF;
    setHeatOff();
    display.clearDisplay(); 
    display.setCursor(0, 11);
    display.print("Over Temp!");
    display.setCursor(0, 22);
    display.print(buf);
    display.display();
    BuzzerChirp();
    delay(2000);
  }  
  
}  // end loop()


//==============================================================================================================================
// get temperature inside toaster
//==============================================================================================================================
void getTemperature()
{
  static uint32_t nextRead = 0;
  
  // Time to read thermocouple?
  if (millis() > nextRead)
  {
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    // Read current temperature, loop until you get a valid reading
    uint32_t waitForValidTempTimer = millis() + 30000;
    do
    {
      g_input = thermocouple.readCelsius();
      if ( isnan(g_input) )
      { 
        delay(150); 
        Serial.println("Got nan"); 
      }
    }
    while ( isnan(g_input) && (millis() < waitForValidTempTimer) );
    
    // If thermocouple is not connected
    if (isnan(g_input) )
    {
      // Illegal operation without thermocouple
      display.clearDisplay(); 
      display.setCursor(0, 11);
      display.print("Thermocouple");
      display.setCursor(0, 22);
      display.print("Error");
      display.display();
      Serial.println("Thermocouple Error");
    }
  }

}  // end getTemperature()


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
      digitalWrite(grnLED, HIGH);
    }
    else
    {
      digitalWrite(heater_top, LOW);
      digitalWrite(heater_bottom, HIGH);
      digitalWrite(grnLED, LOW);
    }
  }
  Serial.println("HEat On");
}  // end setHeatOnIO()


//==============================================================================================================================
//  Turn heater off
//==============================================================================================================================
void setHeatOff()
{
  digitalWrite(heater_top,    LOW);
  digitalWrite(heater_bottom, LOW);
  digitalWrite(grnLED, HIGH);

  
} // end setHeatOff()


//==============================================================================================================================
//==============================================================================================================================
void BuzzerChirp()
{
  digitalWrite(buzzer, HIGH);
  delay(500);
  digitalWrite(buzzer, LOW);
}  // end BuzzerChirp()


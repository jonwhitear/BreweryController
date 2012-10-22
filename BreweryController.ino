/* 
*  Jon's Automorated Brewery Controller
*  Copyright (C) 2012  Jon Whitear
* 
*  compiled on Arduino V1.0.1
* 
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*  
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*  
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*/

// Included libraries

//#include <SPI.h>
#include <OneWire.h>
#include <Wire.h> 
//#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include <Time.h>
#include <PID_v1.h>

//#define DEBUG_SERIAL

// https://arduino-info.wikispaces.com/LCD-Blue-I2C
// set the LCD address to 0x27 for a 20 chars 4 line display

//LiquidCrystal_I2C lcd(0x27,20,4);

// degree c sybmol 
const byte degc[8] = {B01000, B10100, B01000, B00011, B00100, B00100, B00011, B00000};
const byte UNLOCK_ICON[] PROGMEM = {B00110, B01001, B01001, B01000, B01111, B01111, B01111, B00000};
const byte PROG_ICON[] PROGMEM = {B00001, B11101, B10101, B11101, B10001, B10001, B00001, B11111};
const byte BELL[] PROGMEM = {B00100, B01110, B01110, B01110, B11111, B00000, B00100, B00000};
        
time_t prevDisplay = 0; // when the digital clock was displayed
    
const int PUMP = 2;
const int HLT_HEAT = 5;
const int MLT_HEAT = 6;
const int BUZZER = 7;
const int ONE_WIRE_BUS = 8;
const int HEARTBEAT = 9;

// Anlog pins as digital inputs

const int BUTTON_UP = 14;
const int BUTTON_DOWN = 15;
const int BUTTON_NEXT = 16;
const int BUTTON_BACK = 17;

// Define one wire bus piun and DS18x20 precision

const int TEMPERATURE_PRECISION = 9;

// Defines for clock module

#define TIME_MSG_LEN 11    // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

// Variables used by process

double MLT_TEMP_CURRENT , HLT_TEMP_CURRENT , BK_TEMP_CURRENT;	// measured temp may have decimal places. They are stored as 32 bits (4 bytes) of information.
double MLT_TEMP_SETPOINT, HLT_TEMP_SETPOINT, BK_TEMP_SETPOINT;	//

double HLT_PID_OUTPUT, HLT_PID_OUTPUT_LAST;
double MLT_PID_OUTPUT, MLT_PID_OUTPUT_LAST;

//Specify the links and initial tuning parameters

PID HLT_PID(&HLT_TEMP_CURRENT, &HLT_PID_OUTPUT, &HLT_TEMP_SETPOINT,100,20,5, DIRECT);
PID MLT_PID(&MLT_TEMP_CURRENT, &MLT_PID_OUTPUT, &MLT_TEMP_SETPOINT,100,20,5, DIRECT);

int PID_WINDOW_SIZE = 5000;
unsigned long PID_WINDOW_START_TIME;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress HLT_Probe = { 0x10, 0x0A, 0xFF, 0x04, 0x01, 0x08, 0x00, 0x2E };
DeviceAddress MLT_Probe = { 0x10, 0xCE, 0xFA, 0x04, 0x01, 0x08, 0x00, 0xBF };
DeviceAddress BK_Probe  = { 0x10, 0x9E, 0x2E, 0x05, 0x01, 0x08, 0x00, 0x5f };

boolean TIMER = false;             // are we on a timer?
boolean HEARTBEAT_TOGGLE = true;   // value used to set heartbeat pin
boolean MLT_HEAT_ENABLED = false;
boolean HLT_HEAT_ENABLED = false;
boolean PUMP_ENABLED = false;

/****************************************************************************************
*
* Setup
*
*****************************************************************************************/

void setup()
{  
  // Initial setup for the hardware
  InitPins();                       // Initialise the input/output pins
  digitalWrite(HEARTBEAT, HEARTBEAT_TOGGLE);  // swtich the heartbeat LED on during setup
  
  InitSerial();                     // Initialise the serial line
//  InitLCD();                        // Initialise the LCD module
  InitSensors();                    // Initialise temp sensors (one wire)
  InitClock();				// Initialise Arduino clock i.e. set system time
  Buzz(4);                          // Beep the buzzer four times

  // Initial setup for this brew (e.g. config UI to set vars)

  // Set HLT PID set temp
  HLT_TEMP_SETPOINT = 30.0;
  MLT_TEMP_SETPOINT = 30.0;
  HLT_HEAT_ENABLED = true;
  MLT_HEAT_ENABLED = true;
  
  HLT_PID.SetOutputLimits(0, PID_WINDOW_SIZE);
  MLT_PID.SetOutputLimits(0, PID_WINDOW_SIZE);
  
    //turn the PID on
  HLT_PID.SetMode(AUTOMATIC);
  MLT_PID.SetMode(AUTOMATIC);
  
  // Start now? If no set timer
  TIMER = false;
}

/****************************************************************************************
*
* Main loop
*
*****************************************************************************************/

void loop()
{
  // Update the clock and toggle heartbeat when time changes
  if( now() != prevDisplay) //update the display only if the time has changed
  {
    prevDisplay = now();
    //ClockDigitalDisplay();
    
    char HLTtemp[10];
    dtostrf(HLT_TEMP_CURRENT,1,2,HLTtemp);

    char MLTtemp[10];
    dtostrf(MLT_TEMP_CURRENT,1,2,MLTtemp);   

    String DEBUG_MESSAGE = "Temps - HLT: " + String(HLTtemp) + ", MLT: " + String(MLTtemp);
    DebugMessage(DEBUG_MESSAGE);
    
    HEARTBEAT_TOGGLE = !HEARTBEAT_TOGGLE;
    digitalWrite(HEARTBEAT, HEARTBEAT_TOGGLE);
  } 

  // Check time. Start now?
  // Check temps
  ReadTemps();
  
  // Control heat
  ControlHeat();

  // Check for input  
  if (ButtonPress(BUTTON_UP)) {  // Pump Control
    PUMP_ENABLED = !PUMP_ENABLED;
    digitalWrite(PUMP, PUMP_ENABLED);
    Serial.println("Pump control toggled");
    }

  if (ButtonPress(BUTTON_DOWN)) {  // HLT Control
    HLT_HEAT_ENABLED = !HLT_HEAT_ENABLED;
    digitalWrite(HLT_HEAT, HLT_HEAT_ENABLED);
    Serial.println("HLT heat control toggled");
    }

  if (ButtonPress(BUTTON_NEXT)) {  // MLT Control
    MLT_HEAT_ENABLED = !MLT_HEAT_ENABLED;
    digitalWrite(MLT_HEAT, MLT_HEAT_ENABLED);
    Serial.println("MLT heat control toggled");
    }
}

/****************************************************************************************
*
* Function / procedure definitions
*
*****************************************************************************************/

void InitSerial(void)
{
  Serial.begin(9600);                // Open the serial line
  
  Serial.println("Jon's Automated Brewery Controller.");
  Serial.println("Version 14.10.12.");
  Serial.println("Copyright (C) 2012  Jon Whitear");
}
  
/*void InitLCD(void)
{
  lcd.init();                     // Start up the LCD library
  lcd.createChar(0,degc);         // write custom symbol to LCD
  
  // Show the welcome screen

  lcd.backlight();                  //Backlight ON if under program control
  lcd.setCursor(2,0);               //Start at character 2 on line 0
  lcd.print("Brewery Controller");
  lcd.setCursor(5,1);
  lcd.print("Version 14.10.12");
  lcd.setCursor(2,3);
  lcd.print("www.whitear.org");

}
  */
  
void InitPins(void)
{
  // Set pin modes - analog pins as digital inputs
  
  pinMode (BUTTON_UP,INPUT);
  pinMode (BUTTON_DOWN,INPUT);
  pinMode (BUTTON_NEXT,INPUT);
  pinMode (BUTTON_BACK,INPUT);

  // Set pin modes - digital outputs  

  pinMode (HLT_HEAT,OUTPUT);
  pinMode (MLT_HEAT,OUTPUT);
  pinMode (PUMP,OUTPUT);
  pinMode (BUZZER,OUTPUT);
  pinMode (HEARTBEAT,OUTPUT);  
  
  // Flash the LEDs for half a second so we know it's running
  
  digitalWrite(MLT_HEAT, HIGH);    // MLT Element on
  digitalWrite(HLT_HEAT, HIGH);    // HLT Element on
  digitalWrite(PUMP, HIGH);        // Pump on
  digitalWrite(BUZZER, HIGH);      // Buzzer on
  digitalWrite(HEARTBEAT, HIGH);
  delay(500);
  
  // Set initial pin state

  digitalWrite(MLT_HEAT, LOW);    // MLT Element off
  digitalWrite(HLT_HEAT, LOW);    // HLT Element off
  digitalWrite(PUMP, LOW);        // Pump off
  digitalWrite(BUZZER, LOW);      // Buzzer off  
  digitalWrite(HEARTBEAT, LOW);   // Heartbeat LED off
}

void InitSensors(void)
{
  Serial.println("Dallas Temperature IC Control Library");
  // Code from the 'Multiple' example
    
  // Start up the library
  sensors.begin();

  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // set the resolution
  Serial.print("Setting resolution to ");
  Serial.print(TEMPERATURE_PRECISION);
  Serial.println(" bits.");
  
  sensors.setResolution(HLT_Probe, TEMPERATURE_PRECISION);
  sensors.setResolution(MLT_Probe, TEMPERATURE_PRECISION);  
  sensors.setResolution(BK_Probe,  TEMPERATURE_PRECISION);

  // Read the probes
  Serial.print("Requesting current temperatures...");
  ReadTemps();
  Serial.println("DONE");

  // show info about each probe
  Serial.print("HLT Probe:    ");
  DSPrintAddress(HLT_Probe);
  DSPrintResolution(HLT_Probe);
  DSPrintTemperature(HLT_Probe);
  Serial.println();
  
  Serial.print("MLT Probe:    ");
  DSPrintAddress(MLT_Probe);
  DSPrintResolution(MLT_Probe);
  DSPrintTemperature(MLT_Probe);
  Serial.println();

  Serial.print("Kettle Probe: ");
  DSPrintAddress(BK_Probe);
  DSPrintResolution(BK_Probe);
  DSPrintTemperature(BK_Probe);
  Serial.println();
}

void InitClock(void)
{
  // obtain/set the current time from host via serial line

  Serial.println("Initialising clock using host clock via serial.");
  setSyncProvider( requestSync);                                          //set function to call when sync required
  Serial.println("Waiting for sync message...");
  while (timeStatus() == timeNotSet)
  {
    if(Serial.available()) ClockProcessSyncMessage();
  }
  Serial.print("System time set to: ");
  ClockDigitalDisplay();
  Serial.println();  
  
  prevDisplay = now();
}

/****************************************************************************************
*
* Supporting function / procuedure definitions
*
*****************************************************************************************/


// Buzzer

void Buzz(int beeps)
{
  for (int i=0; i < beeps; i++)
  {
    digitalWrite (BUZZER,HIGH);
    delay (500);
    digitalWrite(BUZZER,LOW);
    delay(100);
  }
}

void DebugMessage(String message)
{
  Serial.print("[");
  Serial.print(hour());
  ClockPrintDigits(minute());
  ClockPrintDigits(second());
  Serial.print("] ");
  Serial.println(message);
}
// **************** Temperature Functions ***************************

void ReadTemps() 
{
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  sensors.requestTemperatures();
  HLT_TEMP_CURRENT = sensors.getTempC(HLT_Probe);
  MLT_TEMP_CURRENT = sensors.getTempC(MLT_Probe);
  BK_TEMP_CURRENT  = sensors.getTempC(BK_Probe);
}

// Functions from the dallas temp "multiple" example
// function to print a device address

void DSPrintAddress(DeviceAddress deviceAddress)
{
  Serial.print("Device Address: ");
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print("0x");
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) {
      Serial.print(",");
     }
     else {
      Serial.print(".");
     } 
    Serial.print(" ");
  }
}

// function to print the temperature for a device
void DSPrintTemperature(DeviceAddress deviceAddress)
{
  Serial.print("Temp: ");
  Serial.print(sensors.getTempC(deviceAddress));
  Serial.print(" degrees C. ");
}

// function to print a device's resolution
void DSPrintResolution(DeviceAddress deviceAddress)
{
  Serial.print("Resolution: ");
  Serial.print(sensors.getResolution(deviceAddress));
  Serial.print(" bits. ");
}

// **************** Clock Functions ***************************
// http://www.arduino.cc/playground/Code/Time

void ClockProcessSyncMessage()
{
  // if time sync available from serial port, update time and return true
  while(Serial.available() >=  TIME_MSG_LEN)
  {  // time message consists of header & 10 ASCII digits
    char c = Serial.read() ; 
    Serial.print(c);  
    if( c == TIME_HEADER ) {       
      time_t pctime = 0;
      for(int i=0; i < TIME_MSG_LEN -1; i++){   
        c = Serial.read();          
        if( c >= '0' && c <= '9'){   
          pctime = (10 * pctime) + (c - '0') ; // convert digits to a number    
        }
      }   
     setTime(pctime);   // Sync Arduino clock to the time received on the serial port
     }  
   }
}

void ClockDigitalDisplay()
{
  // digital clock display of the time
  Serial.print(hour());
  ClockPrintDigits(minute());
  ClockPrintDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print("/");
  Serial.print(month());
  Serial.print("/");
  Serial.print(year()); 
  Serial.println(); 
}

void ClockPrintDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

time_t requestSync()
{
  Serial.print(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}

// **************** PID Control Functions ***************************


void ControlHeat() {
  // Control heat output based on PID calculations
  // MLT takes preference over HLT

  unsigned long NOW = millis();

  // If the MLT is enabled, calculate the PID output
  
  if (MLT_HEAT_ENABLED) {
   if  ((MLT_TEMP_SETPOINT - MLT_TEMP_CURRENT) > 30.0) {           // MLT's heating but we're more than 5deg below setpoint
     MLT_PID_OUTPUT = PID_WINDOW_SIZE;								// = window size
     }
   else {	                // MLT's heating but we're LESS than 5deg below setpoint  
    MLT_PID.Compute();
    }
   if (MLT_PID_OUTPUT != MLT_PID_OUTPUT_LAST) {
    char MLTtemp[10];
    dtostrf(MLT_PID_OUTPUT,1,2,MLTtemp);
    DebugMessage(String("MLT PID output changed to " + String(MLTtemp) + "ms."));
    MLT_PID_OUTPUT_LAST = MLT_PID_OUTPUT;
    } 
  }

  // If the HLT is enabled, calculate the PID output
  
  if (HLT_HEAT_ENABLED) {
   if  ((HLT_TEMP_SETPOINT - HLT_TEMP_CURRENT) > 30.0) {           // HLT's heating but we're more than 5deg below setpoint
     HLT_PID_OUTPUT = PID_WINDOW_SIZE;								// = window size
     }
   else {	                // HLT's heating but we're LESS than 5deg below setpoint  
    HLT_PID.Compute();
    }
   if (HLT_PID_OUTPUT != HLT_PID_OUTPUT_LAST) {
    char HLTtemp[10];
    dtostrf(HLT_PID_OUTPUT,1,2,HLTtemp);
    DebugMessage(String("HLT PID output changed to " + String(HLTtemp) + "ms."));
    HLT_PID_OUTPUT_LAST = HLT_PID_OUTPUT;
    } 
  } 

  if ((NOW - PID_WINDOW_START_TIME) > PID_WINDOW_SIZE) {  // If we've past the end of a window, roll it over to start again
    PID_WINDOW_START_TIME += PID_WINDOW_SIZE;			// Put a debug in here to log window increment
    }

  if (HLT_HEAT_ENABLED && MLT_HEAT_ENABLED) {			// Both enabled: HLT is ON, AND MLT is ON
    if (MLT_PID_OUTPUT > (NOW - PID_WINDOW_START_TIME)) { // Output within window
      digitalWrite (HLT_HEAT,LOW);					// switch HLT off...
      digitalWrite (MLT_HEAT,HIGH);					// then switch MLT on.
      }
    else if ((MLT_PID_OUTPUT + HLT_PID_OUTPUT) > (NOW - PID_WINDOW_START_TIME)) { // Output within window
      digitalWrite (MLT_HEAT,LOW);					// switch MLT off...
      digitalWrite (HLT_HEAT,HIGH);					// then switch HLT on.
      }
    else {
      digitalWrite (HLT_HEAT,LOW);					// switch HLT OFF.
      digitalWrite (MLT_HEAT,LOW);					// switch MLT OFF.
    }
  }
  
  else if (MLT_HEAT_ENABLED && ! HLT_HEAT_ENABLED) {			// MLT is ON, but HLT is OFF
    digitalWrite (HLT_HEAT,LOW);				// switch HLT off...
    if (MLT_PID_OUTPUT > (NOW - PID_WINDOW_START_TIME)) {       // Output within window
      digitalWrite (MLT_HEAT,HIGH);					// then switch MLT on.
      }
    else {
      digitalWrite (MLT_HEAT,LOW);					// switch MLT OFF.
    }
  }
  
  else if (HLT_HEAT_ENABLED && ! MLT_HEAT_ENABLED) {			// HLT is ON, but MLT is OFF
    digitalWrite (MLT_HEAT,LOW);					// switch MLT off...
    if (HLT_PID_OUTPUT > (NOW - PID_WINDOW_START_TIME)) {               // Output within window
      digitalWrite (HLT_HEAT,HIGH);					// then switch HLT on.
      }
    else {
      digitalWrite (HLT_HEAT,LOW);					// switch HLT OFF.
    }
  }
  
  else {                                                                 // Neither HLT nor MLT enabled
      digitalWrite (HLT_HEAT,LOW);					// switch HLT OFF.
      digitalWrite (MLT_HEAT,LOW);					// switch MLT OFF.
  }
}

// **************** User Input (i.e. button) Functions ***************************

int ButtonPress(int BUTTON)
{
  if (digitalRead(BUTTON)==1) {
    delay(10); //debounce
    if (digitalRead(BUTTON)==1) {
      delay(100);
      return 1;
    } 
  }
  else {
    return 0;
  }
}  

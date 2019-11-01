/*
  Control an espresso machine boiler using a PID controller

  Code samples used from the following:
  PID Library: https://github.com/br3ttb/Arduino-PID-Library
  PID Lab: https://www.pdx.edu/nanogroup/sites/www.pdx.edu.nanogroup/files/2013_Arduino%20PID%20Lab_0.pdf
  Smoothing: https://www.arduino.cc/en/Tutorial/Smoothing
  Arduino <> Wemos D1 Mini pins: https://github.com/esp8266/Arduino/blob/master/variants/d1_mini/pins_arduino.h
  ESP8266 file uploading https://tttapa.github.io/ESP8266/Chap12%20-%20Uploading%20to%20Server.html

  Hardware:
  Wemos D1 Mini ( https://wiki.wemos.cc/products:d1:d1_mini )
  128 x 64 OLED Display using Adafruit_SSD1306 library
  AD8495 Thermocouple Amplifier ( https://www.adafruit.com/product/1778 )
  ADS1115 16-Bit ADC using library from https://github.com/baruch/ADS1115
  Solid State Relay ( Crydom TD1225 )
*/

#define ADA_TC // Uncomment this if using the Adafruit TC board or similar boards with 1.25v reference

#ifdef ADA_TC
const int ADSGAIN = 2;
const double Vref = 1.2362;
#else
const int ADSGAIN = 16;
const int Vref = 0;
#endif

// set to true for testing the code on the breadboard setup. This will set the hostname to xespresso
const bool breadboard = false;

#include <PID_v1.h>

// Needed for the OLED display
#include <Wire.h>

//#include <Adafruit_SSD1306.h>
//#include <Fonts/FreeSerifBold18pt7b.h
//#include <Fonts/FreeSans9pt7b.h>

// Needed for ESP8266
//#include <ESP8266WiFi.h>
//#include <ESP8266WebServer.h>
#include <FS.h>   //Include File System Headers
//const char* htmlfile = "/index.html";

// Needed for pushing new sketches over WiFi
//#include <ESP8266mDNS.h>
//#include <WiFiUdp.h>
//#include <ArduinoOTA.h>

// Use WiFiManager for setting SSID and Password
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
//#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

// Needed for ADS1115 ADC
#include <ADS1115.h>

// *****************************************
// * Config options that you can customize *
// *****************************************

//#define ThermocouplePin 0 // ** Not needed with ADS1115 board
#define RelayPin 15 // Ardunio D4 = Wemos D1 Mini Pin D2

// After powering on, how many minutes until we force the boiler to power down
// Turning the machine off and on again will reset the timer
const int maxRunTime = 180;

// Turn the display off after 200 minutes

// Default to being ON
bool operMode = true;

// Define the PID setpoint
double Setpoint = 105;

// Define the PID tuning Parameters
//double Kp = 3.5; working ok on 2018-09-14
double Kp = 4.5;
double Ki = 0.125;
double Kd = 0.2;

// PWM Window in milliseconds
const int WindowSize = 5000;

// ***********************************************************
// ***********************************************************
// * There should be no need to tweak many things below here *
// ***********************************************************
// ***********************************************************

// PID variables
// Using P_ON_M mode ( http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/ )
double Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);
double PWMOutput;
uint32_t windowStartTime;

// Define the info needed for the temperature averaging
const int numReadings = 8;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

// Thermocouple variables
float Vout; // The voltage coming from the out pin on the TC amp
float Vtc;
// const float Vbits = brdVolts / 1023; // ** Not needed with ADS1115 board

// Corrected temperature readings for a K-type thermocouple
// https://srdata.nist.gov/its90/type_k/kcoefficients_inverse.html
// Coefficient values for 0C - 500C / 0mV - 20.644mV
const double c0 = 0.000000E+00;
const double c1 = 2.508355E+01;
const double c2 = 7.860106E-02;
const double c3 = -2.503131E-01;
const double c4 = 8.315270E-02;
const double c5 = -1.228034E-02;
const double c6 = 9.804036E-04;
const double c7 = -4.413030E-05;
const double c8 = 1.057734E-06;
const double c9 = -1.052755E-08;


// All timers reference the value of now
uint32_t now = 0; //This variable is used to keep track of time

// OLED display timer
const int OLEDinterval = 250;           // interval at which to write new data to the OLED
uint32_t previousOLEDMillis = now;            // will store last time OLED was updated

// Serial output timer
const int serialPing = 500; //This determines how often we ping our loop
uint32_t lastMessage = now; //This keeps track of when our loop last spoke to serial

int runTimeMins;
long runTimeSecs;
uint32_t runTimeStart = now;

// Temp read interval
const int TempInterval = 5;
uint32_t currentTempMillis;
uint32_t previousTempMillis = now;

#define ESP_SDA 21 // Arduino 14 = ESP8266 Pin 5
#define ESP_SCL 22 // Arduino 12 = ESP8266 Pin 6

// Server tasks interval
const int serverInterval = 50;
uint32_t currentServerMillis;
uint32_t previousServerMillis = now;

// ADS1115 ADC
int adcval;
ADS1115 adc;
float ADS_PGA;

uint32_t prevLoopMillis;
uint32_t numLoops = 0;
uint32_t currLoops = 0;


void keepTime(void)
{
  //Keep track of time
  now = millis();
  runTimeSecs = (now - runTimeStart) / 1000;
  runTimeMins = (now - runTimeStart) / 60000;
}


int readADC()
{
  static int read_triggered = 0;
  if (!read_triggered) {
    if (adc.trigger_sample() == 0)
      read_triggered = 1;
  } else {
    if (!adc.is_sample_in_progress()) {
      adcval = adc.read_sample();
      read_triggered = 0;
    }
  }
  return adcval;
}


void readTemps(void)
{
  currentTempMillis = now;
  if (currentTempMillis - previousTempMillis > TempInterval) {

    // subtract the last reading:
    total = total - readings[readIndex];
    // Read the temps from the thermocouple
    readings[readIndex] = readADC();
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;
    // if we're at the end of the array...
    if (readIndex >= numReadings)
    {
      // ...wrap around to the beginning:
      readIndex = 0;
    }

    // calculate the average:
    average = total / numReadings;

    // This should match the output voltage on the Out pin of the AD8945
    Vout = average * ADS_PGA / 1000;

    // Based on Analog Devices AN-1087
    // Convert the AD8495 output back to millivolts so we can perform the NIST calc
    Vtc = ((Vout * 1000) - (Vref * 1000) - 1.25) / 122.4;

    // Use the NIST corrected temperature readings for a K-type thermocouple in the 0-500°C range
    // https://srdata.nist.gov/its90/type_k/kcoefficients_inverse.html
    Input = c0
            + c1 * Vtc
            + c2 * pow(Vtc, 2)
            + c3 * pow(Vtc, 3)
            + c4 * pow(Vtc, 4)
            + c5 * pow(Vtc, 5)
            + c6 * pow(Vtc, 6)
            + c7 * pow(Vtc, 7)
            + c8 * pow(Vtc, 8)
            + c9 * pow(Vtc, 9);

    previousTempMillis = currentTempMillis;
  }
}


void relayControl(void)
{
  // Calculate the number of running minutes

  // If more than maxRunTime minutes has elapsed, turn the boiler off
  // and dont perform any other PID functions
  if ( (runTimeMins >= maxRunTime) || (operMode == false) )
  {
    digitalWrite(RelayPin, LOW);
    myPID.SetMode(MANUAL);
    Output = 0;
    operMode = false;
  } else {
    // Compute the PID values
    myPID.SetMode(AUTOMATIC);
    myPID.Compute();
  }

  PWMOutput = Output * (WindowSize / 100.00);
  // Starts a new PWM cycle every WindowSize milliseconds
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }

  // Calculate the number of milliseconds that have passed in the current PWM cycle.
  // If that is less than the Output value, the relay is turned ON
  // If that is greater than (or equal to) the Output value, the relay is turned OFF.
  if (PWMOutput > (now - windowStartTime))
  {
    digitalWrite(RelayPin, HIGH);  // Wemos BUILTIN_LED LOW = ON
  } else {
    digitalWrite(RelayPin, LOW); // Wemos BUILTIN_LED HIGH = OFF
  }
}


// Track how many loops per second are executed.
void trackloop() {
  if ( now - prevLoopMillis >= 1000) {
    currLoops = numLoops;
    numLoops = 0;
    prevLoopMillis = now;
  }
  numLoops++;
}



void displaySerial(void)
{
  // Output some data to serial to see what's happening
  if (now - lastMessage > serialPing)
  {
    if ( operMode == true )
    {
      //Serial.print("Time: ");
      //Serial.println(runTimeSecs);
      Serial.println(Input);
    } else {
      Serial.print("Input: ");
      Serial.print(Input, 1);
      Serial.print(", ");
      Serial.print("Mode: off");
      Serial.print("\n");
    }
    lastMessage = now; //update the time stamp.
  }
}



void configADC(void)
{
  // Setup ADS1115
  adc.begin();
  adc.set_data_rate(ADS1115_DATA_RATE_860_SPS);
  adc.set_mode(ADS1115_MODE_SINGLE_SHOT);
  adc.set_mux(ADS1115_MUX_GND_AIN0);

  //  TWO_THIRDS // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
  //  ONE        // 1x gain   +/- 4.096V  1 bit = 0.125mV
  //  TWO        // 2x gain   +/- 2.048V  1 bit = 0.0625mV
  //  FOUR       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
  //  EIGHT      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
  //  SIXTEEN    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV

  if ( ADSGAIN == 23 ) {
    adc.set_pga(ADS1115_PGA_TWO_THIRDS);
    ADS_PGA = 0.1875;
  }
  else if ( ADSGAIN == 1 ) {
    adc.set_pga(ADS1115_PGA_ONE);
    ADS_PGA = 0.125;
  }
  else if ( ADSGAIN == 2 ) {
    adc.set_pga(ADS1115_PGA_TWO);
    ADS_PGA = 0.0625;
  }
  else if ( ADSGAIN == 4 ) {
    adc.set_pga(ADS1115_PGA_FOUR);
    ADS_PGA = 0.03125;
  }
  else if ( ADSGAIN == 8 ) {
    adc.set_pga(ADS1115_PGA_EIGHT);
    ADS_PGA = 0.015625;
  }
  else if ( ADSGAIN == 16 ) {
    adc.set_pga(ADS1115_PGA_SIXTEEN);
    ADS_PGA = 0.0078125;
  }
}


void setup()
{
  Serial.begin(115200); //Start a serial session
  lastMessage = now; // timestamp

  // Set the Relay to output mode and ensure the relay is off
  pinMode(RelayPin, OUTPUT);
  digitalWrite(RelayPin, LOW);

  // PID settings
  windowStartTime = now;

  myPID.SetOutputLimits(0, 100);
  myPID.SetSampleTime(100);

  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
  {
    readings[thisReading] = 0;
  }

  // Enable I2C communication
  Wire.setClock(400000L); // ESP8266 Only
  Wire.begin(ESP_SDA, ESP_SCL);


  configADC();



} // end of setup()


void loop()
{
  keepTime();
  readTemps();
  relayControl();
  displaySerial();
} // End of loop()

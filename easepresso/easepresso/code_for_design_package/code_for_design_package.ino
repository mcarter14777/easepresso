/*************************************/
/*        Included Libraries         */
/*************************************/
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSerifBold18pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Wire.h>
#include <ADS1115.h> //https://github.com/baruch/ADS1115

/*************************************/
/*              Defines              */
/*************************************/
#define relayPin 15 // Ardunio D4 = Wemos D1 Mini Pin D2
#define zcdPin 33
#define solenoidPin 26
#define pumpPin 32
#define ESP_SDA 21
#define ESP_SCL 22 
#define OLED_RESET 16
#define OLED_I2C 0x3C
#define halfPeriodUsec 8333
#define zcdDelayUsec 36

/*************************************/
/*   Global Variables & Constants    */
/*************************************/
// ADC vairables
const int ADSGAIN = 2;
const double Vref = 1.2362;
int adcval;
float ADS_PGA;

// Temperature Averaging
const int numMeasurements = 8;
int measurements[numMeasurements];      // the readings from the analog input
int measurementIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

// Thermocouple variables
float Vout;
float Vtc;

uint32_t now = 0; //This variable is used to keep track of time

// Temp read interval
const int TempInterval = 5;
uint32_t currentTempMillis;
uint32_t previousTempMillis = now;

// PID variables
double Kp = 4.5;
double Ki = 0.125;
double Kd = 0.2;
double tempSetpoint = 105;
double tempInput, tempOutput;
double boilerPWMOutput;
uint32_t windowStartTime;
const int WindowSize = 5000;

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

// Pump Control variables
const int pumpOnDelay[101] = {0, 531, 753, 924, 1068, 1196, 1313, 1421, 1521, 1616, //pump on delay for levels 0 to 1000
                        1707, 1793, 1877, 1957, 2035, 2110, 2183, 2255, 2324,
                        2393, 2460, 2525, 2590, 2654, 2716, 2778, 2839, 2899,
                        2958, 3017, 3075, 3133, 3190, 3246, 3303, 3358, 3414,
                        3469, 3524, 3578, 3633, 3687, 3740, 3794, 3848, 3901,
                        3954, 4007, 4061, 4114, 4167, 4220, 4273, 4326, 4379, 
                        4432, 4486, 4539, 4593, 4647, 4701, 4755, 4810, 4864,
                        4919, 4975, 5031, 5087, 5144, 5201, 5258, 5316, 5375,
                        5435, 5495, 5556, 5617, 5680, 5743, 5808, 5874, 5941,
                        6009, 6079, 6150, 6223, 6299, 6376, 6457, 6540, 6626,
                        6717, 6812, 6913, 7020, 7137, 7265, 7410, 7581, 7802,
                        8333};
unsigned int pumpOffDelay;
volatile unsigned int pumpLevel = 0;
unsigned int pulsecount;
unsigned long unow = 0;
volatile unsigned long lastZeroCrossRise = 0;
volatile unsigned long lastZeroCrossFall = 0;


// OLED display timer
const int OLEDinterval = 250;          
uint32_t previousOLEDMillis = now;           

/*************************************/
/*       Object Instantiation        */
/*************************************/
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
ADS1115 adc;
PID boilerPID(&tempInput, &tempOutput, &tempSetpoint, Kp, Ki, Kd, P_ON_M, DIRECT);

/*************************************/
/*             Functions             */
/*************************************/
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
    total = total - measurements[measurementIndex];
    measurements[measurementIndex] = readADC();
    total = total + measurements[measurementIndex];
    measurementIndex = measurementIndex + 1;
    if (measurementIndex >= numMeasurements)
    {
      measurementIndex = 0;
    }

    average = total / numMeasurements;
    Vout = average * ADS_PGA / 1000;

    // Based on Analog Devices AN-1087
    // Convert the AD8495 output back to millivolts so we can perform the NIST calc
    Vtc = ((Vout * 1000) - (Vref * 1000) - 1.25) / 122.4;

    // Use the NIST corrected temperature readings for a K-type thermocouple in the 0-500°C range
    // https://srdata.nist.gov/its90/type_k/kcoefficients_inverse.html
    tempInput = c0
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

  boilerPID.SetMode(AUTOMATIC);
  boilerPID.Compute();
  

  boilerPWMOutput = tempOutput * (WindowSize / 100.00);
  // Starts a new PWM cycle every WindowSize milliseconds
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }

  // Calculate the number of milliseconds that have passed in the current PWM cycle.
  // If that is less than the Output value, the relay is turned ON
  // If that is greater than (or equal to) the Output value, the relay is turned OFF.
  if (boilerPWMOutput > (now - windowStartTime))
  {
    digitalWrite(relayPin, HIGH);  // Wemos BUILTIN_LED LOW = ON
  } else {
    digitalWrite(relayPin, LOW); // Wemos BUILTIN_LED HIGH = OFF
  }
}


// Track how many loops per second are executed.


void displayOLED(void)
{
  uint32_t currentOLEDMillis = now;

  if (currentOLEDMillis - previousOLEDMillis > OLEDinterval) {
    // save the last time you wrote to the OLED display
    previousOLEDMillis = currentOLEDMillis;

    // have to wipe the buffer before writing anything new
    display.clearDisplay();

    // TOP HALF = Temp + Input Temp
    display.setFont(&FreeSans9pt7b);
    display.setCursor(0, 22);
    display.print("Temp");

    display.setFont(&FreeSerifBold18pt7b);
    display.setCursor(48, 26);
    if ( tempInput >= 100 )
      display.print(pulsecount, 1);
    else
      display.print(pulsecount);

    // BOTTOM HALF = Output + Output Percent
    display.setFont(&FreeSans9pt7b);
    display.setCursor(0, 56);
    display.print("Output");

    display.setFont(&FreeSerifBold18pt7b);
    display.setCursor(60, 60);
    // Dont add a decimal place for 100 or 0
    if ( (tempOutput >= 100.0) || (tempOutput == 0.0) )
    {
      display.print(tempOutput, 0);
    }
    else if ( tempOutput < 10 )
    {
      display.print(tempOutput, 2);
    }
    else
    {
      display.print(tempOutput, 1);
    }
    

    // Do the needful!
    display.display();
  }
}

//function that handles phase control timing
void phaseControl()
{
  unow = micros(); //micros returs current number of microseconds since start of microcontroller
  if(unow - lastZeroCrossRise >= pumpOffDelay) digitalWrite(32, LOW);
  if(unow - lastZeroCrossFall >= halfPeriodUsec - pumpOnDelay[pumpLevel] - zcdDelayUsec) digitalWrite(32, HIGH);
}

//function called when zerocross is rising
void IRAM_ATTR zeroCrossRising() //IRAM_ATTR stores this function in ram to be accessed quickly
{
  
  lastZeroCrossRise = micros(); //micros returs current number of microseconds since start of microcontroller
}

//function called when zerocross is falling
void IRAM_ATTR zeroCrossFalling() //IRAM_ATTR stores this function in ram to be accessed quickly
{
  pulsecount = pulsecount + 1;
  lastZeroCrossRise = micros(); //micros returs current number of microseconds since start of microcontroller
}
//zero cross interupt service routine.
void IRAM_ATTR isr() //IRAM_ATTR stores this function in ram to be accessed quickly
{
  if (digitalRead(33) == HIGH) zeroCrossRising(); 
  else zeroCrossFalling();
}
unsigned long previousMillis1;
unsigned long previousMillis2;
void setup()
{

  Serial.begin(115200); //Start a serial session
  pinMode(zcdPin, INPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(solenoidPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  digitalWrite(solenoidPin, HIGH);
  digitalWrite(pumpPin, LOW);
  attachInterrupt(digitalPinToInterrupt(33), isr, CHANGE);
  // PID settings
  windowStartTime = now;
  boilerPID.SetOutputLimits(0, 100);
  boilerPID.SetSampleTime(100);
  // initialize all the readings to 0:
  for (int thisMeasurement = 0; thisMeasurement < numMeasurements; thisMeasurement++)
  {
    measurements[thisMeasurement] = 0;
  }
  // Enable I2C communication
  Wire.setClock(400000L); // ESP8266 Only
  Wire.begin(ESP_SDA, ESP_SCL);
  // Setup Display
  display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.display();
  
  // Setup ADS1115
  // From example usage at https://github.com/baruch/ADS1115
  //Gain Settings:
  //  TWO_THIRDS // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
  //  ONE        // 1x gain   +/- 4.096V  1 bit = 0.125mV
  //  TWO        // 2x gain   +/- 2.048V  1 bit = 0.0625mV
  //  FOUR       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
  //  EIGHT      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
  //  SIXTEEN    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV
  adc.begin();
  adc.set_data_rate(ADS1115_DATA_RATE_860_SPS);
  adc.set_mode(ADS1115_MODE_SINGLE_SHOT);
  adc.set_mux(ADS1115_MUX_GND_AIN0);
  adc.set_pga(ADS1115_PGA_TWO);
  ADS_PGA = 0.0625;
} // end of setup()


void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis1 >= 1000) {
  // save the last time you blinked the LED
  previousMillis1 = currentMillis;
  pumpLevel = 75;

  // set the LED with the ledState of the variable:
  
  }
  if (currentMillis - previousMillis2 >= 2000) {
  // save the last time you blinked the LED
  previousMillis2 = currentMillis;
  pumpLevel = 50;

  // set the LED with the ledState of the variable:
  
  }  
  readTemps();
  phaseControl();
  relayControl();
  displayOLED();
} // End of loop()

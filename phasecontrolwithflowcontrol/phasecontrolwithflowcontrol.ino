#include <ADS1115.h>
#include <PID_v1.h>


#define Nextion Serial2

#define relayPin 15 
#define zcdPin 33
#define flowPin 34
#define pumpPin 32
#define solenoidPin 26 


const int halfCycleUsec = 8333; //half of 60 hz period
const int zcdDelay = 36;  //zero cross detection delay. due to RC rise/fall time
const uint16_t pumpOnDelay[101] =   // delay for power levels mapped to array index 0-100
    {0, 531, 753, 924, 1068, 1196, 1313, 1421, 1521, 1616, 
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
const int pumpOffDelay = 4167; //quarter of 60 hz period
unsigned int  level =  0; // variable for pump level

volatile unsigned long zcdLastFall = 0; //variable to record last occurance of zerocross rising edge 
volatile unsigned long zcdLastRise = 0; //variable to record last occurance of zerocross falling edge
volatile unsigned long currentMicros = 0; //variable to record current time in microsec
volatile unsigned long zcdLastMicros = 0;
unsigned long lastPressureChange = 0;
volatile bool zcdNewFall = false; 
volatile bool zcdNewRise = false;

const float millilitersPerPulse = 0.51706;
volatile unsigned long lastFlowPulseMicros = 0;
volatile unsigned long lastFlowRateMillis = 0;
unsigned long flowPulseCount;
unsigned long lastFlowPulseCount;
long flowPulseInterval, flowPulseMicros;
float rateOfFlow, flowPulseSecs;
volatile bool firstFlowPulse = true;
double flowSetpoint, flowInput, flowOutput;
// flowKp = 6.6, flowKi = 6.25 , flowKd = 0.05;
double flowKp = 6.6, flowKi = 6.25 , flowKd = 0.05;

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

float Vout;
float Vtc;
const int numTempMeasurements = 8;
int tempMeasurements[numTempMeasurements];      // the readings from the analog input
int tempMeasurementIndex = 0;              // the index of the current reading
int tempTotal = 0;                  // the running total
int tempAverage = 0; // the average
const int TempInterval = 5;
uint32_t currentTempMillis;
uint32_t previousTempMillis = 0;
double tempSetpoint, tempInput, tempOutput;
double tempKp = 4.5, tempKi =0.125, tempKd = 0.2;
double boilerPWMOutput;
uint32_t windowStartTime;
const int WindowSize = 2000;

const int numPressureMeasurements = 8;
int pressureMeasurements[numTempMeasurements];      // the readings from the analog input
int pressureMeasurementIndex = 0;              // the index of the current reading
int pressureTotal = 0;                  // the running total
int pressureAverage = 0;                // the average
const int pressureInterval = 5;
uint32_t currentPressureMillis;
uint32_t previousPressureMillis = 0;

const double Vref = 1.2362;
int adcval;

PID flowPID(&flowInput, &flowOutput, &flowSetpoint, flowKp, flowKi, flowKd, P_ON_M, DIRECT);
PID tempPID(&tempInput, &tempOutput, &tempSetpoint, tempKp, tempKi, tempKd, P_ON_M, DIRECT);
ADS1115 adc;

void Nextion_serial_listen() 
{
    if(Nextion.available() > 2){                // Read if more then 2 bytes come (we always send more than 2 <#> <len> <cmd> <id>
        char start_char = Nextion.read();      // Create a local variable (start_char) read and store the first byte on it  
        if(start_char == '#'){                // And when we find the character #
          uint8_t len = Nextion.read();      // Create local variable (len) / read and store the value of the second byte
                                            // <len> is the lenght (number of bytes following) 
          unsigned long tmr_1 = millis();
          boolean cmd_found = true;
            
          while(Nextion.available() < len){ // Waiting for all the bytes that we declare with <len> to arrive              
            if((millis() - tmr_1) > 100){    // Waiting... But not forever...... 
              cmd_found = false;              // tmr_1 a timer to avoid the stack in the while loop if there is not any bytes on Serial
              break;                            
            }                                     
            delay(1);                            // Delay for nothing delete it if you want
          }                                   
                                               
            if(cmd_found == true){            // So..., A command is found (bytes in Serial buffer egual more than len)
              uint8_t cmd = Nextion.read();  // Create local variable (cmd). Read and store the next byte. This is the command group
                                             
              switch (cmd){
                                    
                case 'P': /*or <case 0x50:>  IF 'P' matches, we have the command group "Page". 
                           *The next byte is the page <Id> according to our protocol.
                           *It reads the next byte as a type of <uint8_t variable> and direct send it to:
                           *first_refresh_page() function as parameter. 
                           */
//                  first_refresh_page((uint8_t)Nextion.read());  
                break;
                
                case 'N': 
                /* < case 0x4e: > (0x4e = ‘N’) IF there is a matching with 'N' then we have the command group "Line" according to the protocol
                 * We are waiting 2 more bytes from the <nextion_var> and the <cnt>... (<#> <len> <cmd> <nextion_var> <cnt>)
                 * <nextion_var> is the Number of the variable on Nextion that we want to write
                 * <cnt> is the number of the text array Line that we want to store into Nextion's variable
                 * From Nextion we have sent < printh 23 04 4C 04 xx > where 04, in this example, the <nextion_var> and xx the <cnt>
                 * Same as the above, we are going to read the next 2 bytes as <uint8_t> and direct send them to
                 * < sending_text () > fuction as parameters to the local variables of the function 
                 * < sending_text(byte nextion_var, byte cnt) > that will be created on start up of the Function
                 */

                    recieve_number((uint8_t)Nextion.read(),(uint8_t)Nextion.read()); 
                    
                break;
                case 'S': 

                    recieve_state((uint8_t)Nextion.read(),(uint8_t)Nextion.read()); 
                    
                break;
                case 'Event': 
                /* < case 0x4e: > (0x4e = ‘N’) IF there is a matching with 'N' then we have the command group "Line" according to the protocol
                 * We are waiting 2 more bytes from the <nextion_var> and the <cnt>... (<#> <len> <cmd> <nextion_var> <cnt>)
                 * <nextion_var> is the Number of the variable on Nextion that we want to write
                 * <cnt> is the number of the text array Line that we want to store into Nextion's variable
                 * From Nextion we have sent < printh 23 04 4C 04 xx > where 04, in this example, the <nextion_var> and xx the <cnt>
                 * Same as the above, we are going to read the next 2 bytes as <uint8_t> and direct send them to
                 * < sending_text () > fuction as parameters to the local variables of the function 
                 * < sending_text(byte nextion_var, byte cnt) > that will be created on start up of the Function
                 */

                    recieve_number((uint8_t)Nextion.read(),(uint8_t)Nextion.read()); 
                    
                break;                                
              }
            }
        }  
    }    
}

void recieve_number(byte nextion_attr, byte val)
{
  if(nextion_attr == 0x01) tempSetpoint = val;
  if(nextion_attr == 
}

void receiveBrewStart()
{
  brewing = true;
  brewStartTime = millis();
  digitalWrite(solenoidPin, HIGH);
}

void receivePreinfusionSetting(int val)
{
  val = preinfusionSetting();
}

void recievePreinfusionTime(int val)
{
  val = preinfusionTime;
}

void recieveVolumeTarget(int val)
{
  val = volumeTarget;
}

void receiveExtractionTimeTarget(int val)
{
  val = extractionTimeTarget;
}

void brewControl()
{
  if(brewing == true)
  {
    if(preinfusionSetting == true && (millis() - brewStartTime <= preinfusionTime))
    {
      preinfusion();
    }
    elseif((preinfusionSetting == false || (millis() - brewStartTime >=  preinfusionTime) )&& millis() - extractionStartTime < extractionTimeTarget) )
    {
      extraction();
    }
    else
    {
      endBrew();
    }
  }
}

void preinfusion()
{
  level = 30;  
}

void extraction()
{
  if(extracting == false)
  {
    extractionStartTime = millis();
    extracting == true;
    extractionStartPulses = flowPulseCount;
  }
  millilitersBrewed = flowPulseCount - extractionStartPulses;
  flowSetpoint = (volumeTarget - millilitersBrewed) / (extractionTimeTarget - (millis() - extractionStartTime));
}

void endBrew() 
{
  brewstart = false;
  level = 0;
  extracting = false;
}

void IRAM_ATTR zcdFall()
{
 
  zcdLastFall = micros(); //sets equal to current microseconds
  zcdNewFall = true;
 
}

void IRAM_ATTR zcdRise()
{

  zcdLastRise = micros(); //sets equal to current microseconds
  zcdNewRise = true;

}

void IRAM_ATTR zcdISR()
{
  if( micros()- zcdLastMicros >= 50)
  {

    if( digitalRead(33) == HIGH) zcdRise();
    else zcdFall();    
    zcdLastMicros = micros();

  }
}

void phasecontrol()
{

  //currentMicros = micros();  //records current time in microseconds
  if(zcdNewFall)
  {
    if(micros() - zcdLastFall >= 4167) 
    {
      
      digitalWrite(32, LOW);  //turns pump off
      zcdNewFall = false;

    } 
  }

  //currentMicros = micros();
  if(zcdNewRise)
  {
    if(micros() - zcdLastRise >= 8333 - pumpOnDelay[level])
    {
      digitalWrite(32, HIGH);//turns pump on
      zcdNewRise = false; 

    }  
  }
}

void IRAM_ATTR flowISR()
{
  if( micros() - lastFlowPulseMicros >= 100)
  {
    if(digitalRead(flowPin) == LOW) 
    {
      flowPulseCount++;
      flowPulseInterval = micros() - lastFlowPulseMicros;
      lastFlowPulseMicros = micros();
    }
  }
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


float readPressure()
{
  
  adc.set_pga(ADS1115_PGA_ONE);
  adc.set_mux(ADS1115_MUX_GND_AIN1);
  currentPressureMillis = millis();
  if (currentPressureMillis - previousPressureMillis > pressureInterval) 
  {

    // subtract the last reading:
    pressureTotal = pressureTotal - pressureMeasurements[pressureMeasurementIndex];
    pressureMeasurements[pressureMeasurementIndex] = readADC();
    pressureTotal = pressureTotal + pressureMeasurements[pressureMeasurementIndex];
    pressureMeasurementIndex = pressureMeasurementIndex + 1;
    if (pressureMeasurementIndex >= numPressureMeasurements)
    {
      pressureMeasurementIndex = 0;
    }

  }
}

float readFlowRate() // returns milliliters per second flow rate
{
  if(micros() - lastFlowPulseMicros > 2000000)
  {
    return 0;
  }
  if(flowPulseInterval)
  {
    return ( millilitersPerPulse / ((float)flowPulseInterval/1000000));
  }
  else return 0;
}

void flowControl()
{
  rateOfFlow= readFlowRate();
  flowInput = rateOfFlow;
  flowPID.Compute();
  level = (int)flowOutput;
}

void readTemp()
{
  adc.set_pga(ADS1115_PGA_TWO);
  adc.set_mux(ADS1115_MUX_GND_AIN0);
  currentTempMillis = millis();
  if (currentTempMillis - previousTempMillis > TempInterval) {

    // subtract the last reading:
    tempTotal = tempTotal - tempMeasurements[tempMeasurementIndex];
    tempMeasurements[tempMeasurementIndex] = readADC();
    tempTotal = tempTotal + tempMeasurements[tempMeasurementIndex];
    tempMeasurementIndex = tempMeasurementIndex + 1;
    if (tempMeasurementIndex >= numTempMeasurements)
    {
      tempMeasurementIndex = 0;
    }

    tempAverage = tempTotal / numTempMeasurements;
    Vout = tempAverage * 0.0625 / 1000;

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

void boilerControl(void)
{

  tempPID.SetMode(AUTOMATIC);
  tempPID.Compute();
  

  boilerPWMOutput = tempOutput * (WindowSize / 100.00);
  // Starts a new PWM cycle every WindowSize milliseconds
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }

  // Calculate the number of milliseconds that have passed in the current PWM cycle.
  // If that is less than the Output value, the relay is turned ON
  // If that is greater than (or equal to) the Output value, the relay is turned OFF.
  if (boilerPWMOutput > (millis() - windowStartTime))
  {
    digitalWrite(relayPin, HIGH);
  } else {
    digitalWrite(relayPin, LOW);
  }
}


void setupADC()
{
  adc.begin();
  adc.set_data_rate(ADS1115_DATA_RATE_860_SPS);
  adc.set_mode(ADS1115_MODE_SINGLE_SHOT);
  //adc.set_mux(ADS1115_MUX_GND_AIN0); to be set when reading value
  adc.set_pga(ADS1115_PGA_TWO);
}
char temp_data[50];
unsigned long lastNextionPrint=0;
void coreTask( void * pvParameters )
{
 
    while(true)
    {

      if(millis()- lastNextionPrint >= 1000)
      {
        Serial.println(tempInput);
        lastNextionPrint = millis();
      } 
       // Serial.print(",");
       //Serial.println(flowInput);
      sprintf(temp_data, "t0.txt=\"%d\"", (int)tempInput);
      Nextion.print(temp_data);
      Nextion.print("\xFF\xFF\xFF");


      //delay(1000);
      Nextion_serial_listen();
      //  Serial.print("level");
       // Serial.println(level);
    }
 
}
 
void setup()
{
  Nextion.begin(115200, SERIAL_8N1, 19, 23);
  Serial.begin(115200);
  delay(1000);
  pinMode(zcdPin, INPUT_PULLUP); //zcd  (zero cross detector) pin
  pinMode(flowPin, INPUT_PULLUP); //flow sensor pin
  pinMode(pumpPin, OUTPUT); //pump pin 
  pinMode(15, OUTPUT); //boiler pin
  pinMode(solenoidPin, OUTPUT); //Solenoid pin 
  digitalWrite(solenoidPin, HIGH); //turn on to allow water to flow out of goup head
  digitalWrite(relayPin,LOW);
  attachInterrupt(digitalPinToInterrupt(flowPin), flowISR, FALLING); //attaches interrupt to flow pin and calls flowISR() on rising or falling edge
  attachInterrupt(digitalPinToInterrupt(zcdPin), zcdISR, CHANGE); //attaches interrupt to zcd pin and calls zcdISR() on rising or falling edge
  setupADC();
  tempPID.SetOutputLimits(0, 100);
  flowPID.SetOutputLimits(30,100);
  flowPID.SetMode(AUTOMATIC);
  flowSetpoint = 7;
  tempSetpoint = 105;
  xTaskCreatePinnedToCore(
                    coreTask,   /* Function to implement the task */
                    "coreTask", /* Name of the task */
                    10000,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    0,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    0);  /* Core where the task should run */
 
  Serial.println("Task created...");
 
  
}

void loop()
{
  //currentMicros = micros();
  //level = 100;
  readTemp();
  boilerControl();
  readFlowRate();
  //readPressure();
  brewControl();
  phasecontrol();
  //delayMicroseconds(100);
}

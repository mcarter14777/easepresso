#define zcdPin 33
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
unsigned long lastPressureChange = 0;
volatile bool zcdNewFall = false; 
volatile bool zcdNewRise = false;

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
  delayMicroseconds(50);
  if( digitalRead(33) == HIGH) zcdRise();
  else zcdFall();

}

void phasecontrol()
{
  //delayMicroseconds(100);
  //currentMicros = micros();  //records current time in microseconds
  if((currentMicros - zcdLastFall >= 4167) && zcdNewFall) 
  {
    digitalWrite(32, LOW);  //turns pump off
    zcdNewFall = false;
  } 
  //currentMicros = micros();
  if((currentMicros - zcdLastRise >= 8333 - pumpOnDelay[level]) && zcdNewRise)
  {
    digitalWrite(32, HIGH);//turns pump on
    zcdNewRise = false; 
  }  
}


void pressuredemo()
{
 if(currentMiicros - lastPressureChange >= 1000000)
 {
  if( level == 100) level = 0;
  else level+=10;
 
  lastPressureChange = micros();
 }
}

void setup()
{
  pinMode(zcdPin, INPUT_PULLUP);  //zcd  (zero cross detector) pin
  pinMode(pumpPin, OUTPUT); //pump pin 
  pinMode(15, OUTPUT); //boiler pin
  pinMode(solenoidPin, OUTPUT); //Solenoid pin 
  digitalWrite(solenoidPin, HIGH); //turn on to allow water to flow out of goup head
  
  attachInterrupt(digitalPinToInterrupt(33), zcdISR, CHANGE); //attaches interrupt to zcd pin and calls zcdISR() on rising or falling edge
}

void loop()
{
  currentMicros = micros();
  pressuredemo();
  phasecontrol();
  delayMicroseconds(100);
}

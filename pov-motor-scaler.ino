/*
https://github.com/FastLED/FastLED/wiki/SPI-Hardware-or-Bit-banging
Teensy 3/3.1:
Hardware SPI - data 11, clock 13
Hardware SPI - data 7, clock 14
*/

//Define Variables we'll be connecting to
double gap;

//Define the aggressive and conservative Tuning Parameters
  
int power = 0;
const int maxPower = 130;
const int motorInit = 101;

float maxVoltage = 2.4;
float voltage = 0.0;

signed int ascending = 1;

const int avgFactor = 16;

boolean cango = false;

const unsigned int DRIVEPIN = 20;
const unsigned int HALLPIN = 21; 
const unsigned int BUTTONPORT = 5;
const unsigned int TARGETRPM = 500;
const unsigned int ACCELFACTOR = 1;

const float RCDIFF = 0.128;

const float SYSTEMVOLTAGE = 3.3;// TEENSY 3.3v ARDUINO 5v

float avg = 0.1;
volatile int rpmcount = 0;
volatile float rpm = 0.1;

const int smoothCyclesLimit = 3;
int smoothCyclesIterator = 0;

volatile int current = 0;
volatile int last = 0;

float now = 0.0;
int innerNow = 0;
int lastmillis = 0;


// http://www.massmind.org/Techref/io/sensor/interface.htm
void average(int sample) // sample is in ms
{ 
  //avg -= (avg>>4)
  avg -= (avg / avgFactor);   // output result is 1/16th of accumulator   // subtract l/16th of the accumulator
  avg += sample;     // add in the new sample
}

void setupHallSensor()
{
  digitalWrite(HALLPIN, HIGH);
  delay(1000);
  Serial.println("Hall Setup.");  
  delay(3000);
  attachInterrupt(HALLPIN, interruptHandler, FALLING);
  Serial.println("Go!");
  rpmcount = 0.0;
  rpm = 0.0;
}

void interruptHandler()
{ 
    rpmcount++;
    current = now - last;
    average(current);
    last = millis();
    //Serial.println("TICK!!!");
}


void printPowerStatus()
{
    //Serial.print(" Power: ");    
    //Serial.print(power);

//    Serial.print(" Voltage: ");    
//    Serial.print(voltage);

//    Serial.print(" Max Voltage: ");    
//    Serial.print(maxVoltage);
}

void printRpmStatus()
{
    //Serial.print(" gap: " );
    //Serial.print(gap);
    
    //Serial.print(" AVG RPM: ");    
    Serial.println(valueToRpm(avg) * 16);
    
    //Serial.print(" Hz: ");    
    //Serial.print(rpmcount);

    //Serial.print(" RPM: ");
    //Serial.println(rpm);
    
}

void setup() {
  Serial.begin(9600);
  pinMode(DRIVEPIN, OUTPUT);
  analogWrite(DRIVEPIN, 0);
  pinMode(HALLPIN, INPUT);
  setupHallSensor();
  power = motorInit;
  Serial.println(power);
  analogWrite(DRIVEPIN, power);  
}

void loop() {
  now = millis();
//  if(cango 
//  && (now - lastmillis) > 100) {
//    if(gap < (TARGETRPM / 20)) {
//      decelerate();
//    }
//    if(gap > (TARGETRPM / 20)) {
//      accelerate();
//    }    
//  }
  if (now - lastmillis > 1000) { 
    cango = true;
    lastmillis = now;      
    rpm = valueToRpm(current);
    current = 0;
    voltage = powerToVolts(power);
    //Serial.println(rpm);
    printPowerStatus();
    printRpmStatus();
    rpmcount = 0;
          
    gap = TARGETRPM - rpm; //distance away from TARGETRPM    

    if(smoothCyclesLimit == smoothCyclesIterator) {
      if(gap < (TARGETRPM / 20)) {
        decelerate();
      }
      if(gap > (TARGETRPM / 20)) {
        accelerate();
      }        
      smoothCyclesIterator = 0;
    } else {
      smoothCyclesIterator++;
    }
    
  }
}

float valueToRpm(float value)
{
  if(value == 0) {
    return 0;
  }
  return (60.0 / value) * 1000.0;// (60/B2)*1000
}

void accelerate()
{
  ascending = 1;
  signalMotor();
}

void decelerate()
{
  ascending = -1;
  signalMotor();  
}

void signalMotor()
{
  power += (ascending * ACCELFACTOR);
  checkRange();
  analogWrite(DRIVEPIN, (int) power);
}

void checkRange()
{    
    power = constrain(power, motorInit, maxPower);
    voltage = constrain(voltage, 0, maxVoltage);
}

float powerToVolts(float p)
{
  return (p/255) * SYSTEMVOLTAGE + RCDIFF;// p >> 8
}

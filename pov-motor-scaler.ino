/*
https://github.com/FastLED/FastLED/wiki/SPI-Hardware-or-Bit-banging Teensy 3/3.1:
Hardware SPI - data 11, clock 13
Hardware SPI - data 7, clock 14
*/

#define TEENSY_SYS 3.3
#define ARDUINO_SYS 5
//Define Variables we'll be connecting to
double gap;

elapsedMicros timeSinceMagnet;

elapsedMicros displayTimer;

volatile uint32_t periodDuration = 0;


//Define the aggressive and conservative Tuning Parameters
  
int power = 0;
const int maxPower = 140;
const int motorInit = 101;

float maxVoltage = 2.8;
float voltage = 0.0;

signed int ascending = 1;

boolean cango = false;

const unsigned int DRIVEPIN = 20;
const unsigned int HALLPIN = 21; 
const unsigned int BUTTONPORT = 5;
const unsigned int TARGETRPM = 700;
const unsigned int ACCELFACTOR = 1;

const float RCDIFF = 0.128;

const float SYSTEMVOLTAGE = TEENSY_SYS;// TEENSY 3.3v ARDUINO 5v


int tmp = 0;
const int avgFactor = 16;
float avg = 0.1;
volatile float rpm = 0.0;

const int smoothCyclesLimit = 3;
int smoothCyclesIterator = 0;

// http://www.massmind.org/Techref/io/sensor/interface.htm
void average(float sample) // sample is in ms
{ 
  //avg -= (avg>>4) /// can only shift int, so use avgfactor
  avg -= (avg / avgFactor);   // output result is 1/16th of accumulator   // subtract l/16th of the accumulator
  avg += sample;     // add in the new sample
}

void setupHallSensor()
{
  pinMode(HALLPIN, INPUT);
  digitalWrite(HALLPIN, HIGH);
  delay(2000);
  Serial.println("Hall Setup.");  
  delay(2000);
  attachInterrupt(HALLPIN, interruptHandler, RISING);
  Serial.println("Go!");
}

void interruptHandler()
{   
    periodDuration = timeSinceMagnet;
    timeSinceMagnet = 0;
    rpm = rpmFromMicros(periodDuration);    
    
}


void printPowerStatus()
{
//    Serial.print(" Power: ");    
//    Serial.print(power);

//    Serial.print(" Voltage: ");    
//    Serial.print(voltage);

//    Serial.print(" Max Voltage: ");    
//    Serial.print(maxVoltage);
}

void printRpmStatus()
{
    
    //Serial.print(" AVG RPM: "); 
    //Serial.print(avg);   

    Serial.print(" GAP: "); 
    Serial.print(gap);   

    //Serial.print(" HZ: ");
    //Serial.print(rpm/60);
    
    Serial.print(" RPM: ");
    Serial.print(rpm);

    Serial.print(" HZ: ");
    Serial.println(1000000.0 / periodDuration);
    
}

void setup() {
  Serial.begin(9600);
  pinMode(DRIVEPIN, OUTPUT);
  analogWrite(DRIVEPIN, 0);
  setupHallSensor();
  power = motorInit;
  Serial.println(power);
  analogWrite(DRIVEPIN, power);  
}


void loop() {
  
//  if(cango 
//  && (now - lastmillis) > 100) {
//    if(gap < (TARGETRPM / 20)) {
//      decelerate();
//    }
//    if(gap > (TARGETRPM / 20)) {
//      accelerate();
//    }    
//  }
  if (displayTimer >= 1000000) { // 1 sec intervals
    displayTimer = 0;
    average(rpm); 
    cango = true;    
    voltage = powerToVolts(power);
    printPowerStatus();
    printRpmStatus();          
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

void accelerate() {
  ascending = 1;
  signalMotor();
}

void decelerate() {
  ascending = -1;
  signalMotor();  
}

void signalMotor() {
  power += (ascending * ACCELFACTOR);
  checkRange();
  analogWrite(DRIVEPIN, (int) power);
}

void checkRange() {    
    power = constrain(power, motorInit, maxPower);
    voltage = constrain(voltage, 0, maxVoltage);
}

float powerToVolts(float p) {
  return (p/255) * SYSTEMVOLTAGE + RCDIFF;// p >> 8
}

float rpmFromMicros(uint32_t in) {
  return (1000000.0 / in) * 60;
}

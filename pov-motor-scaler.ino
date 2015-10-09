/*
https://github.com/FastLED/FastLED/wiki/SPI-Hardware-or-Bit-banging
Teensy 3/3.1:
Hardware SPI - data 11, clock 13
Hardware SPI - data 7, clock 14
*/

int power = 115;
int maxPower = 120;
int motorInit = 115;

float maxVoltage = 2.2;
float voltage = 0.0;

signed int ascending = 1;

const unsigned int DRIVEPIN = 20;
const unsigned int HALLPIN = 21; 
const unsigned int BUTTONPORT = 5;
const unsigned int TARGETRPM = 120;
const unsigned int ACCELFACTOR = 1;

const float SYSTEMVOLTAGE = 3.3;

volatile int avg = 1;
volatile int rpmcount = 0;
volatile float rpm = 0.0;

volatile float current = 0.0;
volatile float last = 0.0;

float now = 0.0;
float innerNow = 0.0;
float lastmillis = 0.0;


// http://www.massmind.org/Techref/io/sensor/interface.htm
void average(int sample) // sample is in ms
{
  avg -= (avg>>4);   // output result is 1/16th of accumulator   // subtract l/16th of the accumulator
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
}


void printPowerStatus()
{
    Serial.print(" Power: ");    
    Serial.print(power);

    Serial.print(" Voltage: ");    
    Serial.print(voltage);

    Serial.print(" Max Voltage: ");    
    Serial.println(maxVoltage);
}

void printRpmStatus()
{
    Serial.print(" AVG RPM: ");    
    Serial.print(avg);
    
    Serial.print(" Hz: ");    
    Serial.print(rpmcount);

    Serial.print(" RPM: ");
    Serial.println(rpm);
    
}

void setup() {
  Serial.begin(9600);
  pinMode(DRIVEPIN, OUTPUT);
  analogWrite(DRIVEPIN, 0);
  pinMode(HALLPIN, INPUT);
  setupHallSensor();
  analogWrite(DRIVEPIN, motorInit);
}

void loop() {
    now = millis();
    
    if (now - lastmillis > 1000) { 
      lastmillis = now;
      rpm = (60 / avg) * 1000;// (60/B2)*1000
      printRpmStatus();
      //printPowerStatus();
      rpmcount = 0;
    }
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
  return (p/255) * SYSTEMVOLTAGE;// p >> 8
}




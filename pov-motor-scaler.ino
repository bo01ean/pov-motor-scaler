//#if defined (__arm__) && defined (__SAM3X8E__) // Arduino Due compatible*/
#include "teensyTimers.h"
//#endif

#define TEENSY_SYS 3.3
#define ARDUINO_SYS 5

volatile float gap = 0.0; 

float power = 0.0;
const int maxPower = 2600;
int motorInit;

elapsedMicros timeSinceMagnet;
elapsedMicros displayTimer;

volatile uint32_t periodDuration = 1;
//Define the aggressive and conservative Tuning Parameters
  
float maxVoltage = 2.8;
float voltage = 0.0;

signed int ascending = 1;

boolean cango = false;

const unsigned int DRIVEPIN = 0; // A6
const unsigned int HALLPIN = 1; // 21 
const float TARGETRPM = 100.0;
const unsigned int ACCELFACTOR = 1 << 3;

const float RCDIFF = 0.128;

const float SYSTEMVOLTAGE = TEENSY_SYS;// TEENSY 3.3v ARDUINO 5v

const int ANALOG_PRECISION_BITS = 12;// 4096

int tmp = 0;
const int avgFactor = 16;
float avg = 0.1;

#define filterSamples   16              // filterSamples should  be an odd number, no smaller than 3
int rpmSmooth[filterSamples];   // array for holding raw sensor values for sensor1 
int smoothRpm;
volatile float rpm = 0.0;

const int smoothCyclesLimit = 1;
int smoothCyclesIterator = 1;

// http://www.massmind.org/Techref/io/sensor/interface.htm
void average(float sample) { // sample is in ms
  //avg -= (avg>>4) /// can only shift int, so use avgfactor
  avg -= (avg / avgFactor);   // output result is 1/16th of accumulator   // subtract l/16th of the accumulator
  avg += sample;     // add in the new sample
}

void setupHallSensor() {
  pinMode(HALLPIN, INPUT);
  digitalWrite(HALLPIN, HIGH);
  delay(2000);
  //Serial.println("Hall Setup.");  
  delay(2000);
  attachInterrupt(HALLPIN, interruptHandler, RISING);
  //Serial.println("Go!");
}

void interruptHandler() {   
    periodDuration = timeSinceMagnet;
    timeSinceMagnet = 0;
    rpm = rpmFromMicros(periodDuration);
    smoothRpm = digitalSmooth(rpm, rpmSmooth);
}

void printStatus() {
/*    
    Serial.print(" TARGETRPM: "); 
    Serial.print(TARGETRPM);   

    Serial.print(" GAP: "); 
    Serial.print(gap);   
    
    Serial.print(" GGAP: "); 
    Serial.print(ggap);

    
    Serial.print(" RPM: ");
    Serial.print(rpm);

    Serial.print(" (smoothed): ");
    Serial.print(smoothRpm);

    Serial.print(" HZ: ");
    Serial.print(1000000.0 / periodDuration);

    Serial.print(" Ascending: ");
    Serial.print(ascending);

    Serial.print(" Power: ");
    Serial.print(power);

    Serial.print(" Voltage: ");    
    Serial.print(voltage);

    Serial.print(" Output: ");    
    Serial.println(Output);
*/
}

void setup() {
  //Serial.begin(9600);

  motorInit = map(101, 0, (1 << 8) - 1, 0, (1 << ANALOG_PRECISION_BITS) - 1);

  //analogWriteResolution(ANALOG_PRECISION_BITS);
  
  pinMode(DRIVEPIN, OUTPUT);
  analogWrite(DRIVEPIN, 0);
  setupHallSensor();
  power = motorInit;
  //Serial.println(power);
  analogWrite(DRIVEPIN, power);  

}


void loop() {

  if (displayTimer >= 500000) { // 1 sec intervals
    displayTimer = 0;
    average(rpm); 
    cango = true;    
    
    gap = TARGETRPM - rpm; //distance away from TARGETRPM  
  
    voltage = powerToVolts(power);
    printStatus();          
  
    if (smoothCyclesLimit == smoothCyclesIterator) {
      if (gap < TARGETRPM) {
        accelerate();
      }
      if (gap > TARGETRPM) {
        decelerate();
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
  return (p/4096) * SYSTEMVOLTAGE;// + RCDIFF;// p >> 8
}

float rpmFromMicros(uint32_t in) {
  return (1000000.0 / in) * 60;
}

int digitalSmooth(int rawIn, int *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int j, k, temp, top, bottom;
  long total;
  static int i;
  static int sorted[filterSamples];
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  // Serial.print("raw = ");

  for (j=0; j<filterSamples; j++){     // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmoothArray[j];
  }

  done = 0;                // flag to know when we're done sorting              
  while(done != 1) {        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++){
      if (sorted[j] > sorted[j + 1]) {     // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j+1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++) {
    total += sorted[j];  // total remaining indices
    k++; 
    // Serial.print(sorted[j]); 
    // Serial.print("   "); 
  }

//  Serial.println();
//  Serial.print("average = ");
//  Serial.println(total/k);
  return total / k;    // divide by number of samples
}




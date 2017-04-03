//#if defined (__arm__) && defined (__SAM3X8E__) // Arduino Due compatible*/
// These are MAGICAL teensy timer shortcuts.
#include "teensyTimers.h"
//#endif

// DEFINES are compiler macros, which cannot be changed. When the compiler encounters these in code, it replaces with the macro value
#define TEENSY_SYS 3.3 // Teensy is 3.3 volt
#define ARDUINO_SYS 5 // SOME arduinos are 5v (DUE / Trinket)



// How much power are we sending to the motor pin
float power = 0.0;
// What is the upper limit for that pin
const int maxPower = 2600;

// This is where the motor loop starts at ( defined below )
int motorInit = 0;

// These are more limits, this time in voltage.
float maxVoltage = 2.8;
// This stores our approx. voltage and is set from powerToVolts
float voltage = 0.0;

// These are magical timers from teensyTimers
// This one is the hall effect timer
elapsedMicros timeSinceMagnet;
// This one is to display stats
elapsedMicros displayTimer;

// This is the copy of the time since the magnet
volatile uint32_t periodDuration = 1;


// Which way do we apply power?
signed int ascending = 1;

boolean cango = false;

const unsigned int DRIVEPIN = 0; // A6
const unsigned int HALLPIN = 1; // 21

volatile float gap = 0.0; // The LOSS value target RPM vs. ACTUAL RPM

const float TARGETRPM = 100.0;
// This is how many step we add or subtract when accelerating / decellerating
const unsigned int ACCELFACTOR = 1 << 3; // 2 to the power of three is 8

// This is a Resistor APPROX. for our LPF. The LPF smooths PWM for the controller
const float RCDIFF = 0.128;

// We map our MCU Voltage.
const float SYSTEMVOLTAGE = TEENSY_SYS;// TEENSY 3.3v ARDUINO 5v

// This is how many steps our pins will have.
const int ANALOG_PRECISION_BITS = 12;// 4096

// This is how we figure moving averages using an accumulator / divider
const int avgFactor = 1 << 4; // 2 to the power of 4 is 16
float avg = 0.1;

// This is another smoothing algorithm
#define filterSamples avgFactor - 1 // filterSamples should  be an odd number, no smaller than 3
// This is the list of values we use for statistical smoothing
int rpmSmooth[filterSamples];   // array for holding raw sensor values for sensor1
// This is where we store the smoothed RPM
int smoothRpm;
// This is the 'rough' RPM from the sensor
volatile float rpm = 0.0;

// How often we check for "loss" of power
const int smoothCyclesLimit = 1;
// How often we iterate
int smoothCyclesIterator = 1;


// Simple (FAST) bitwise accumulator / divider
// http://www.massmind.org/Techref/io/sensor/interface.htm
void average(float sample) { // sample is in ms
  //avg -= (avg>>4) /// can only shift int, so use avgfactor
  avg -= (avg / avgFactor);   // output result is 1/16th of accumulator   // subtract l/16th of the accumulator
  avg += sample;     // add in the new sample
}


/*
SETUP
We have to map the hall effect pin and pull it HIGH.
*/
void setupHallSensor() {
  pinMode(HALLPIN, INPUT);
  digitalWrite(HALLPIN, LOW);
  delay(2000);
  //Serial.println("Hall Setup.");
  delay(2000);
  attachInterrupt(HALLPIN, interruptHandler, RISING);
  //Serial.println("Go!");
}


/*
Interrupt: processors have what is called an interrupt: A high priority function
That is called when something happens. Here, when the hall sensor is activated,
We interrupt the MCU and calculate the RPM based on the last time it was interrupted.
*/
void interruptHandler() {
    // we copy since it is volatile
    periodDuration = timeSinceMagnet;
    // We clear in case it is 'mid-state'
    timeSinceMagnet = 0;
    // Get the actual RPM
    rpm = rpmFromMicros(periodDuration);
    // make a smoothed version in case noise ( there will be noise )
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


/*

This function setups up the MCU with communications and pi assignments

*/
void setup() {

  //Serial.begin(9600);
  // This value is specific to the kelley controller
  motorInit = map(101, 0, (1 << 8) - 1, 0, (1 << ANALOG_PRECISION_BITS) - 1);
  //analogWriteResolution(ANALOG_PRECISION_BITS);

  pinMode(DRIVEPIN, OUTPUT);
  analogWrite(DRIVEPIN, 0); // We set to 0 first in case it is 'floating'
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
  return (p / 1 << ANALOG_PRECISION_BITS) * SYSTEMVOLTAGE;
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

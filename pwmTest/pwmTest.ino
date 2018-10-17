#include "TimerOne.h" //Timer 1 library

//Pin Assignments
int TACH_PIN[] = {2,3,21}; //digital pins that can be converted to interrupt pins: 2,3,21,20,19,18 for INT0-5 respectively
int PWM_PIN[] = {13,14,15};

//Constants
const double TICKS_PER_REV = 2; 
const int NUMBER_LEVELS = 3;
const int BAUD_RATE = 9600;
const long TIMER_LENGTH = 1000000; //length of timer in microseconds
const double TIMER_LENGTH_SECONDS = (double) (TIMER_LENGTH/1000000);

//Variables
int targetSpeed[] = {0,0,0}; //target speeds for each level of fans. To be set by computer (pi) control (over serial?)  
int setValue[] = {0,0,0};

volatile double ticks[] = {0,0,0}; //counter for tachometer
double gain = 1; //proportional control gain for fan control

volatile byte sweep = 0;

//three separate interrupt service routines for each level of fan
void rpm0(){ 
  ticks[0]++;
}

void controlMotors() //interrupt driven. Every 1 second, modify pwm to each pair of fans
{ 
  noInterrupts(); //turn off interrupts briefly during these calculations
  for(int i = 0; i<NUMBER_LEVELS; i++)
  {
    setValue[i] = (int) (ticks[i]/(TIMER_LENGTH_SECONDS*TICKS_PER_REV)-targetSpeed[i])*gain; 
    //check to make sure setValue is within valid range. Correct if not.
    if(setValue[i] >255){
    setValue[i] = 255;
    }
    else if(setValue[i] <0){
      setValue[i] = 0;
    }    
    ticks[i] = 0; //reset value    
  }
  interrupts(); //re-enable interrupts
}

void setup() {
  pinMode(TACH_PIN[0], INPUT);
  pinMode(TACH_PIN[1], INPUT);
  pinMode(TACH_PIN[2], INPUT);
  pinMode(PWM_PIN[0], OUTPUT); 
  pinMode(PWM_PIN[1], OUTPUT); 
  pinMode(PWM_PIN[2], OUTPUT); 
  Serial.begin(BAUD_RATE); 
}

void loop() {
  //moved writing writing motor speed into main loop to slim down interrupts and reduce system lag
  analogWrite(PWM_PIN[0], 20);

  delay(1000);
  sweep = sweep + 64;
  
  Serial.println(sweep); //print to debug or communicate... 
}

//TODO: modify controlMotors function to set minimum threshold pwm value to correspond with minimum voltage to operate fan
//TODO: determine whether capacitive filtering is enough to clean up tachometer signal (or if it's even needed when operating at a higher voltage)
//      notes: for cap, rule of thumb is C=I*t/dV
//      might want decoupling cap for power supply to prevent relatively low frequency PWM from causing noise in the system
//TODO: recieve motor speed command from Pi
//TODO: increase controller bandwidth if desired... could aim for ~10*slowest frequency of the fans (i.e. 20* slowest tach frequency). Determine max and min bandwidths
//TODO: see if actually need to check velocity on all 6 motors or just on 3 and assume it's representative of the other half
//TODO: figure out power supply for all 6 fans
//TODO: see how much speed variability there is between fans
//TODO: confirm that setValue line works as expected and that casting doesn't cause issues
//TODO: determine gain for proportional control

#include "TimerOne.h" //Timer 1 library

//Pin Assignments
int TACH_PIN[] = {2,3,21}; //digital pins that can be converted to interrupt pins: 2,3,21,20,19,18 for INT0-5 respectively
int PWM_PIN[] = {13,14,15};

//Constants
const double TICKS_PER_REV = 2; 
const int SERIAL_MESSAGE_LENGTH = 1;
const int NUMBER_LEVELS = 3;
const int BAUD_RATE = 9600;
const long TIMER_LENGTH = 1000000; //length of timer in microseconds
const double TIMER_LENGTH_SECONDS = (double) (TIMER_LENGTH/1000000);

//Variables
int targetSpeed[] = {100,100,100}; //target speeds for each level of fans. To be set by computer (pi) control (over serial?)  
int setValue[] = {0,0,0};
int rxVal = 0;

volatile double ticks[] = {0,0,0}; //counter for tachometer
double gain = 1; //proportional control gain for fan control

//three separate interrupt service routines for each level of fan
void rpm0(){ 
  ticks[0]++;
}
void rpm1(){
  ticks[1]++;
}
void rpm2(){
  ticks[2]++;
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
    Serial.print("setValue = ");
    Serial.print(setValue[i]);
    Serial.print("; Speed = ");
    Serial.println(ticks[i]*60/(TIMER_LENGTH_SECONDS*TICKS_PER_REV));
    ticks[i] = 0; //reset value    
  }
  interrupts(); //re-enable interrupts
}

void setup() {
  pinMode(TACH_PIN[0], INPUT); // motor speed input for level 1
  pinMode(TACH_PIN[1], INPUT); // motor speed input for level 2
  pinMode(TACH_PIN[2], INPUT); // motor speed input for level 3
  pinMode(PWM_PIN[0], OUTPUT); // motor speed output for level 1
  pinMode(PWM_PIN[1], OUTPUT); // motor speed output for level 2
  pinMode(PWM_PIN[2], OUTPUT); // motor speed output for level 3
  Serial.begin(BAUD_RATE); 

  Timer1.initialize(TIMER_LENGTH); //initializes timer1 with a 1s (1 million microsecond) period (max with a single timer/counter is ~8.3 seconds)
  Timer1.attachInterrupt(controlMotors); //timer1 interrupt which controls motors based on speed
  
  attachInterrupt(digitalPinToInterrupt(TACH_PIN[0]),rpm0,RISING); //tachometer 0 interrupt  
  attachInterrupt(digitalPinToInterrupt(TACH_PIN[1]),rpm1,RISING); //tachometer 1 interrupt
  attachInterrupt(digitalPinToInterrupt(TACH_PIN[2]),rpm2,RISING); //tachometer 2 interrupt

  interrupts();
  
  analogWrite(PWM_PIN[0],rxVal); //set starting value to 0
}

void loop() {
  //moved writing writing motor speed into main loop to slim down interrupts and reduce system lag
//  analogWrite(PWM_PIN[0], setValue[0]);
//  analogWrite(PWM_PIN[1], setValue[1]);
//  analogWrite(PWM_PIN[2], setValue[2]); 
delay(200);
  if(Serial.available() > 0){
    //when enough bytes have been sent to serial port, process message
    int numBits = Serial.available();
    Serial.print("there are x bits: ");
    Serial.println(numBits);
    rxVal = 0;
    int readVal = 0;
    int decMult = 1;
    int mult=1;
    for(int i=numBits;i>0;i--){
      readVal = Serial.read();
      if(readVal !=10){
      Serial.print(readVal);
      Serial.print(" ");
      mult = 1;
      for(int j=i-2;j>0;j--){
        mult = mult*10;
      }
      Serial.println((readVal-48)*mult);
      rxVal += (readVal-48)*mult;
      }      
    }
    Serial.print("I received: ");
    Serial.println(rxVal);
    analogWrite(PWM_PIN[0],rxVal);
  }
  
//  Serial.print(); //print to debug or communicate... 
}

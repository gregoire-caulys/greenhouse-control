//TODO: Integrate previous control code to current code
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
//TODO: determine if sensor data should be sent to the pi
//TODO: determine if there will be seperate sensors per level of the greenhouse
//TODO: assign pins for humidity sensor, CO2 sensor, moisture sensor, light sensor, waterflow sensor?
//TODO: confirm that same sensors are being used as in FinalReport1.pdf
//TODO: find bounds for fan speed (e.g. max speed, stall speed)

//Sensors: 
//Grove - Temp/Humidity Sensor SHT31 
//      - i2c coms. address = 0x44 
//      - connect SCL to i2c clock (Analog 5) and sda to i2c data (analog 4)
//      - library here https://github.com/Seeed-Studio/Grove_SHT31_Temp_Humi_Sensor
//      - additional setup instructions here: http://wiki.seeedstudio.com/Grove-TempAndHumi_Sensor-SHT31/

//Grove - Sunlight Sensor
//      - also i2c. address = 0x60
//      - library here: https://github.com/SeeedDocument/Grove-Sunlight_Sensor/raw/master/res/Grove_Sunlight_Sensor-master.zip
//      - more info here: http://wiki.seeedstudio.com/Grove-Sunlight_Sensor/

#include <TimerOne.h> //Timer 1 library
#include <ArduinoJson.h> //Arduino JSON message parser
#include <Wire.h>
#include <SI114X.h>
#include <SHT31.h>
//#include <DHT.h>


int TACH_PIN[] = {2,3,21}; //digital pins that can be converted to interrupt pins: 2,3,21,20,19,18 for INT0-5 respectively
int PWM_PIN[] = {13,14,15}; 
int TEMP_PIN[] = {A1,A2,A3}; // A1,A2,A3 the output pin of LM35 <- not sure what this comment means.. pulled from previous code

//Constants
const double TICKS_PER_REV = 2; 
const int SERIAL_MESSAGE_LENGTH = 1;
const int NUMBER_LEVELS = 3;
const int BAUD_RATE = 9600;
const long TIMER_LENGTH = 1000000; //length of timer in microseconds
const double TIMER_LENGTH_SECONDS = (double) (TIMER_LENGTH/1000000);
const double TEMP_SENSOR_CONVERSION = 0.48828125;

//Variables
int target_speed[] = {100,100,100}; //target speeds for each level of fans
int set_value[] = {0,0,0};
int rx_val = 0;

double target_temp[] = {21,21,21}; //target temperature, degrees C
int target_hum[] = {50,50,50}; //target humidity, [%]?
double meas_temp[] = {0,0,0};

volatile double ticks[] = {0,0,0}; //counter for tachometer
double speed_gain = 1; //proportional control gain for fan speed control
double temp_gain = 1; //proportional control gain for temperature control

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
    //motor proportional speed control:
    set_value[i] = (int) (ticks[i]/(TIMER_LENGTH_SECONDS*TICKS_PER_REV)-target_speed[i])*speed_gain; 
    //check to make sure setValue is within valid range. Correct if not.
    if(set_value[i] >255){
    set_value[i] = 255;
    }
    else if(set_value[i] <0){
      set_value[i] = 0;
    }    
    Serial.print("set_value = ");
    Serial.print(set_value[i]);
    Serial.print("; Speed = ");
    Serial.println(ticks[i]*60/(TIMER_LENGTH_SECONDS*TICKS_PER_REV));
    ticks[i] = 0; //reset value    
  }
  interrupts(); //re-enable interrupts
}

void setup() {
  for(int i = 0;i<NUMBER_LEVELS;i++){
  pinMode(TACH_PIN[i], INPUT); // motor speed input
  pinMode(PWM_PIN[i], OUTPUT); // motor speed output
  pinMode(TEMP_PIN[i], INPUT); // temperature sensor input
  }
  
  Serial.begin(BAUD_RATE); 

  Timer1.initialize(TIMER_LENGTH); //initializes timer1 with a 1s (1 million microsecond) period (max with a single timer/counter is ~8.3 seconds)
  Timer1.attachInterrupt(controlMotors); //timer1 interrupt which controls motors based on speed
  
  attachInterrupt(digitalPinToInterrupt(TACH_PIN[0]),rpm0,RISING); //tachometer 0 interrupt  
  attachInterrupt(digitalPinToInterrupt(TACH_PIN[1]),rpm1,RISING); //tachometer 1 interrupt
  attachInterrupt(digitalPinToInterrupt(TACH_PIN[2]),rpm2,RISING); //tachometer 2 interrupt

  interrupts();
  
  analogWrite(PWM_PIN[0],rx_val); //set starting value to 0
}

void loop() {
  delay(200);
  controlTemp();  //greenhouse proportional temperature control
  
  if(Serial.available() > 0){
    //when enough bytes have been sent to serial port, process message
    int num_bits = Serial.available();
    Serial.print("there are x bits: ");
    Serial.println(num_bits);
    rx_val = 0;
    int read_val = 0;
    int dec_mult = 1;
    int mult=1;
    for(int i=num_bits;i>0;i--){
      read_val = Serial.read();
      if(read_val !=10){
      Serial.print(read_val);
      Serial.print(" ");
      mult = 1;
      for(int j=i-2;j>0;j--){
        mult = mult*10;
      }
      Serial.println((read_val-48)*mult);
      rx_val += (read_val-48)*mult;
      }      
    }
    Serial.print("I received: ");
    Serial.println(rx_val);
    analogWrite(PWM_PIN[0],rx_val);
  }
}

void controlTemp() {  // get the temperature and convert it to celsius
  for(int i=1;i<NUMBER_LEVELS;i++){    
    double temp = (double) analogRead(TEMP_PIN[i]);
    meas_temp[i] =  temp * TEMP_SENSOR_CONVERSION;
    set_value[i] = (target_temp[i]-meas_temp[i])*temp_gain;
  }
}

 /** 
PROJECT :             Caulys Project
FILENAME :            CO-EPFL.ino
 
DESCRIPTION :
    Communication with sensors, actuators and serial communication 
    with the connected raspberry pi board.
  
AUTHORS : 
   Gregoire Gentile
   Simon Honigmann
   
*/

// Include libraries
#include <TimerOne.h> //Timer 1 library
#include <SoftwareSerial.h>
#include "sensors.h"
#include <Wire.h>
#include "SI114X.h" //Light
#include "SHT31.h" //Temp&hum 


#define SHT31_ADDR_OFF   0x44
#define SHT31_ADDR_ON    0x45



//______________________ PIN CONNECTIONS _____________________________
//sensors
const int TEMPHUM_PIN[] = {2, 3, 4}; //outside, level 1, level 2 temp/hum sensors.

//const int CO2_PIN[] = {5,6,7}; //outside, level 1, level 2 CO2 sensors.
  
//actuators
const int FAN_PIN[] = {9,10};
const int LED_PIN[] = {11,12};
const int PUMP_PIN = 13; 


int k = 0;
int inbyte = 0;



//______________________ CONSTANT VALUES _____________________________


const int BAUD_RATE = 9600;

const int NUMBER_LEVELS = 2; //number of levels of the greenhouse in CO building

unsigned long PERIOD = 10000; //period in milliseconds

// Pump and Lights values
unsigned long TIME_LED[]={0,0};
unsigned long TIME_PUMP=0;

// Time factor = 60minutes*60seconds*1000milliseconds/1000millisecondperperiod=360;

unsigned long TIME_LED_ON= {(unsigned long)16 * 3600, (unsigned long)16 * 3600} ; // Number before time factor is in hours
unsigned long TIME_LED_OFF={(unsigned long)8 * 3600, (unsigned long)8 * 3600} ;
unsigned long TIME_PUMP_ON=1*360;
unsigned long TIME_PUMP_OFF=5*360;

double DIFF_TEMP[] = {0, 0}; //diff betwenn level 1 and out, level 2 and out
double DIFF_HUM[] = {0, 0};
int rx_val = 0;

double TARGET_TEMP[] = {0,21, 21}; //target temperature, degrees C ; 0 at the beginning for array substraction and difference consistency (see function control fans)
double TARGET_HUM[] = {0,50, 50}; //target relative humidity, %
double TARGET_CO2[] = {0,0, 0}; //target CO2, ppm

double WEIGHT_TEMP=0.4; //weights between 0 and 1
double WEIGHT_HUM=0.5;
double WEIGHT_CO2=0.7;



double MEAS_TEMP[] = {0, 0, 0}; //out, level 1, level 2
double MEAS_HUM[] = {0, 0, 0}; //out, level 1, level 2
double MEAS_CO2[] = {0, 0, 0}; //out, level 1, level 2

double TEMP_GAIN = 1; //proportional control gain for temperature control
double HUM_GAIN = 1; //proportional control gain for humidity control
double CO2_GAIN = 1; //proportional control gain for CO2 control



//______________________ SENSORS OBJECTS _____________________________
// Declaration of sensor object instances

SHT31 sht31 = SHT31(); //check if need of this, probably not useful
LightSensor light_sensor_out("light_out");
TempHumSensor temp_hum_sensor[] = {TempHumSensor("temp_hum_out"), TempHumSensor("temp_hum_in1"), TempSensor("temp_hum_in2")}; //any error occuring when more initialized sensors whereas less connected (for future modular version) ? 
// add CO2 when working

void setup() 
  {  
  Serial.begin(BAUD_RATE);
  delay(500);

  
  // Initialize sensors
 light_sensor_out.init();
  for (int i = 0; i <= NUMBER_LEVELS; i++) {
    temp_hum_sensor[i].init(SHT31_ADDR_OFF); //TODO: FIGURE OUT HOW TO DEAL WITH MULTIPLE SENSOR INITIALIZATIONS
    //co2_sensor[i].init();
  }

  
  
  
//______________________ SET PIN MODES _____________________________


//Sensors

  for (int i = 0; i <= NUMBER_LEVELS; i++) 
  {
    pinMode(TEMPHUM_PIN[i], OUTPUT);
    //pinMode(CO2_PIN[i], OUTPUT);
  }

  // Actuators
 pinMode(PUMP_PIN, OUTPUT);
 
  for (int i = 0; i < NUMBER_LEVELS; i++) 
  {
    pinMode(FAN_PIN[i], OUTPUT);
    pinMode(LED_PIN[i], OUTPUT);
  }

}


void loop() 
{

// Control of the fans, the lights and the pump
  ControlFans();
  ControlLights();
  ControlPump();
  
  if(Serial.available()) 
  {
    char instruction = Serial.read();
    
    // case s : send sensor measurements
    if(instruction == 's') 
      {
        String measurements[SENSORS_NB] = 
        {light_sensor_out.get_measurements(),
        // NB : the two temp_hum sensors take each 100ms
        // to get measurements, 100x slower than the others
        Temp_Hum[0],Temp_Hum[1],Temp_Hum[2]};
     
        Serial.println(build_json(measurements));
      }
  }

 delay(Period);

}

void GetMeasures ()
{
  for (int i = 0; i <= NUMBER_LEVELS; i++) 
  {
    digitalWrite(TEMPHUM_PIN[i], HIGH);
    MEAS_TEMP[i] = temp_hum_sensor[i].getTemperature(SHT31_ADDR_ON);
    MEAS_HUM[i] = temp_hum_sensor[i].getHumidity(SHT31_ADDR_ON);
    //MEAS_CO2[i] = 
    String Temp_Hum[i]=temp_hum_sensor[i].get_measurements();
    digitalWrite(TEMPHUM_PIN[i], LOW);
  }
  
}

void ControlFans() 
{ //Every 1 second, modify pwm to each pair of fans; get the temperature and convert it to celsius

  GetMeasures();

  DIFF_TEMP[]=DiffArrays(TARGET_TEMP[],MEAS_TEMP[],NUMBER_LEVELS); //define the difference between target and inside conditions. Index equal to the level (starts at 1)
  DIFF_HUM[]=DiffArrays(TARGET_HUM[],MEAS_HUM[],NUMBER_LEVELS);
  //DIFF_CO2=Diff_Arrays(TARGET_CO2,MEAS_CO2,NUMBER_LEVELS);
  
  for (int i=1;i < NUMBER_LEVELS; i++)
  {
     DIFF_TEMP_OUTIN[i]=MEAS_TEMP[0]-MEAS_TEMP[i]; //define the difference between outside and inside conditions for each level
     DIFF_HUM_OUTIN[i]=MEAS_HUM[0]-MEAS_TEMP[i];
     //DIFF_HUM_OUTIN[i]=MEAS_HUM[0]-MEAS_TEMP[i+1];
  }
 

  for (int i = 1; i <= NUMBER_LEVELS; i++)
  {

    if(((DIFF_TEMP[i]<0) && (DIFF_TEMP_OUTIN[i]>0))||((DIFF_TEMP[i]>0) && (DIFF_HUM_OUTIN[i]<DIFF_TEMP[i]))) //Case of needing a cooler or a heater : we do nothing 
    {
      SPEED_TEMP[i]=0;
    }
    
    if(((DIFF_HUM[i]<0) && (DIFF_HUM_OUTIN[i]>0))||((DIFF_HUM[i]>0) && (DIFF_HUM_OUTIN[i]<DIFF_HUM[i]))) //Case of needing a humidifier or a dryer : we do nothing 
    {
      SPEED_HUM[i]=0;
    }
    
   //if(((DIFF_CO2[i]<0) && (DIFF_CO2_OUTIN[i]>0))||((DIFF_CO2[i]>0) && (DIFF_CO2_OUTIN[i]<DIFF_CO2[i]))) //Case of needing a CO2 cleaner or a CO2 creator : we do nothing 
    //{
      //SPEED_CO2[i]=0;
    //}    

    
    if(((DIFF_TEMP[i]<0) && (DIFF_TEMP_OUTIN[i]<0))||((DIFF_TEMP[i]>0) && (DIFF_TEMP_OUTIN[i]>0))) //Outside conditions enable to increase or decrease and get closer to target
    {
      SPEED_TEMP[i]=abs(WEIGHT_TEMP*DIFF_TEMP[i]);
    }
        
    if(((DIFF_HUM[i]<0) && (DIFF_HUM_OUTIN[i]<0))||((DIFF_HUM[i]>0) && (DIFF_HUM_OUTIN[i]>0))) //Outside conditions enable to increase or decrease and get closer to target
    {
      SPEED_HUM[i]=abs(WEIGHT_HUM*DIFF_HUM[i]);
    }

    //if(((DIFF_CO2[i]<0) && (DIFF_CO2_OUTIN[i]<0))||((DIFF_CO2[i]>0) && (DIFF_CO2_OUTIN[i]>0))) //Outside conditions enable to increase or decrease and get closer to target
    //{
      //SPEED_CO2[i]=abs(WEIGHT_CO2*DIFF_CO2[i]);
    //}

    set_value[i]=(SPEED_TEMP[i] + SPEED_HUM[i] + SPEED_CO2[i])/(DIFF_TEMP[i] + DIFF_HUM[i] + DIFF_CO2[i])*255 ; //make a total of 1*255

    analogWrite(FAN_PIN[],set_value[i]);
    
    //check to make sure setValue is within valid range. Correct if not.
    if (set_value[i] > 255) 
    {
      set_value[i] = 255;
    }
    else if (set_value[i] > 10 && set_value[i] < 25) 
    {
      set_value[i] = 25; //stalling was found to occur at pwm = 10. Startup required pwm of at least 20. Setting overall min to 25 for simplicity.     }
    }
    else if (set_value[i] <= 10) 
    {
      set_value[i] = 0; //For very low commands. Simply turn off the fan
    }
    if (DEBUG_MODE == 1) 
    {
      //debugPrint1(i);
    }
    
  }
}


void ControlLights() 
{
  
  for (int i = 0; i < NUMBER_LEVELS; i++) 
  {
    TIME_LED[i] = (TIME_LED[i]++) % (TIME_LED_OFF[i] + TIME_LED_ON[i]); //modulo operator will reset timer counter when it overflows
    
    if (TIME_LED[i] <= TIME_LED_ON[i]) 
    {
      digitalWrite(LED_PIN[i], HIGH);
    
    }
    else if ((TIME_LED[i] > TIME_LED_ON[i]) && (TIME_LED[i] <= (TIME_LED_OFF[i] + TIME_LED_ON[i]))) 
    {
      digitalWrite(LIGHT_PIN[i], LOW);
    }
    
  }
}

void ControlPump() 
{
  TIME_PUMP = (TIME_PUMP++) % (TIME_PUMP_ON + TIME_PUMP_OFF);
  if (TIME_PUMP <= TIME_PUMP_ON) 
  {
    digitalWrite(PUMP_PIN, HIGH);
  }
  else if ((TIME_PUMP > TIME_PUMP_ON) && (TIME_PUMP <= (TIMPE_PUMP_OFF + TIME_PUMP_ON))) 
  {
    digitalWrite(PUMP_PIN, LOW);
  }
}


void DiffArrays(double Array1[], double Array2[], double Length) //function to subtract two arrays
{
  int i;
  double Result[Length];
  
  for (i=1; i <= Length; i++)
  {
    Result[i] = Array1[i]-Array2[i];
  }
  return Result[];
}

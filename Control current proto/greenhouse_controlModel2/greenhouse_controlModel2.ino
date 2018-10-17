 /** 
PROJECT :             Greenhouse Project
FILENAME :            greenhouse_control.ino
 
DESCRIPTION :
    Communication with sensors, actuators and serial communication 
    with the connected raspberry pi board.
  
AUTHORS : 
   Gregoire Gentile
*/

// Include libraries
#include <SoftwareSerial.h>
#include "sensors.h"
#include <Wire.h>
#include <SI114X.h>
#include <SHT31.h>
#include <DHT.h>


#define SHT31_ADDR        0x44
#define SHT31_ADDR_OUT    0x45

// Number of actuators
#define RELAYS_NB         6

// Pins on which the relays are plugged
// correspondingly : fan, light out, heat, light, pump1, pump2
const int relay_pin[RELAYS_NB] = {2,6,8,4,10,12};

int k = 0;
int inbyte = 0;



SHT31 sht31 = SHT31();
// Declaration of sensor object instances
LightSensor light_sensor_1("light_win");
TempHumSensor temp_hum_sensor_out("temp_hum_out");
TempHumSensor temp_hum_sensor_in("temp_hum_in");



unsigned long Period = 10000;

unsigned long TimeLight=0;
unsigned long TimeLightOut=0;
unsigned long TimePump1=0;
unsigned long TimePump2=0;

// Facteur temps =60minutes*60seconds*1000milliseconds/1000millisecondperperiod=360;

unsigned long TimeLightOn=16*360; // chiffre avant le facteur represente temps actviation en heures
unsigned long TimeLightOff=8*360;

unsigned long TimeLightOutOn=16*360; // chiffre avant le facteur represente temps actviation en heures
unsigned long TimeLightOutOff=8*360;

unsigned long TimePump1On=1*360;
unsigned long TimePump1Off=5*360;

unsigned long TimePump2On=1*360;
unsigned long TimePump2Off=5*360;


void setup() 
  {  
  Serial.begin(9600);
  
  delay(500);
  
  // Initialize sensors
  light_sensor_1.init();
  temp_hum_sensor_in.init(SHT31_ADDR);
  temp_hum_sensor_out.init(SHT31_ADDR_OUT);
  
  SoftwareSerial sensor(50,51);
  
  // Set pins mode
  for(k=0; k<RELAYS_NB; k++) {
    pinMode(relay_pin[k], OUTPUT);
  }
  
}


void loop() {

  TimeLight=TimeLight+1;
  TimeLightOut=TimeLightOut+1;
  TimePump1=TimePump1+1;
  TimePump2=TimePump2+1;


  TimeLight = ActivationFunction(TimeLightOn,TimeLightOff,TimeLight,3);
  TimeLightOut = ActivationFunction(TimeLightOutOn,TimeLightOutOff,TimeLightOut,1);
  TimePump1 = ActivationFunction(TimePump1On,TimePump1Off,TimePump1,4);
  TimePump2 = ActivationFunction(TimePump2On,TimePump2Off,TimePump2,5);

   Serial.println(TimeLight);
   Serial.println(TimeLightOut);


  digitalWrite(relay_pin[0], HIGH); //activates the fans by default
 
  
  if(Serial.available()) 
  {
    char instruction = Serial.read();
    
    // case s : send sensor measurements
    if(instruction == 's') 
      {
        String measurements[SENSORS_NB] = 
        {light_sensor_1.get_measurements(),
        // NB : the two temp_hum sensors take each 100ms
        // to get measurements, 100x slower than the others
        temp_hum_sensor_in.get_measurements(), 
        temp_hum_sensor_out.get_measurements()};
     
        Serial.println(build_json(measurements));
      }
  }

 delay(Period);

}

unsigned long ActivationFunction(unsigned long TimeOn, unsigned long TimeOff, unsigned long Time, int RelayNumber)
{
  
  if (Time<=TimeOn)
    {
      digitalWrite(relay_pin[RelayNumber], HIGH);
    }
   else if ((Time>TimeOn) && (Time<=(TimeOff+TimeOn)))
    {
      digitalWrite(relay_pin[RelayNumber], LOW);
    }
   else if (Time>(TimeOff+TimeOn))
    {
      Time = 0;
    }
   
   return Time;
   
}

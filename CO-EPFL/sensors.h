
/** 
PROJECT :             Greenhouse Project
FILENAME :            sensors.h
  
DESCRIPTION :
    Classes for sensors
  
AUTHORS : 
    Gaspard Feuvray, Gregoire Gentile
*/

#ifndef sensors_h
#define sensors_h

#include "Arduino.h"
#include "/usr/share/arduino/libraries/Wire/Wire.h"
#include "/usr/share/arduino/libraries/SoftwareSerial/SoftwareSerial.h"
#include "/home/pi/sketchbook/libraries/SI114X/SI114X.h"
#include "/home/pi/sketchbook/libraries/SHT/SHT31.h"

#define SENSORS_NB 3

String build_json(String sensor_measurements[SENSORS_NB]);

//______________________ LIGHT _____________________________

class LightSensor {
  String _name;
  SI114X SI1145;
  
public:
  LightSensor(String name);
  bool init();
  String get_measurements();
  String get_name();
};

//____________________ TEMP HUMID SHT __________________________

class TempHumSensor{
  String _name;
  SHT31 sht31;
  
public:
  int _temperature;
  TempHumSensor(String name);
  bool init(int i2c_adr);
  String get_measurements();
  float get_temp(int i2c_adr);
  float get_hum(int i2c_adr);
  String get_name();
  
};
  


//______________________ CO2 __________________________

class Co2Sensor {
  String _name;
  
  // unsigned char _dataRevice[9];
  int _temperature;
  int _CO2PPM;
  SoftwareSerial _sensor;
  
  
public:
  Co2Sensor(String name, uint8_t tx, uint8_t rx);
  void init();
  String get_measurements();
  String get_name();
  bool data_receive();
};


#endif

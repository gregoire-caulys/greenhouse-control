/** 
PROJECT :             Greenhouse Project
FILENAME :            sensors.cpp
  
DESCRIPTION :
    Classes for sensors
  
AUTHORS : 
    Gaspard Feuvray, Gregoire Gentile
*/
#include "sensors.h"

// Writes all measurements in json format
String build_json(String sensor_measurements[SENSORS_NB]) {
  String str = "*{";
  for(int i=0; i<SENSORS_NB; i++) {
    // add delimiter
    if(i!=0) {
      str = str + ",";
    }
    // add sensor
    str = str + sensor_measurements[i];
  }
  return str+"}*"; 
}


//______________________ LIGHT _____________________________
// I2C communication

LightSensor::LightSensor(String name) : _name(name), SI1145(SI114X()) {
}
 
String LightSensor::get_measurements() {
  String vis = String(SI1145.ReadVisible());
  String ir = String(SI1145.ReadIR());
  char uv_str[8] = "";
  dtostrf((float)SI1145.ReadUV()/100, 5, 3, uv_str);
  String uv(uv_str);

  return "\""+_name+"\":{\"vis\":"+vis+",\"ir\":"+ir+",\"uv\":"+uv+"}";
}

String LightSensor::get_name() {
  return _name;
}

bool LightSensor::init() {
  return SI1145.Begin();
}


//____________________ TEMP HUMID SHT __________________________
// I2C communication

TempHumSensor::TempHumSensor(String name): _name(name), sht31(SHT31()){
}

String TempHumSensor::get_measurements() {
  
  char humidity_str[8] = "";
  dtostrf(sht31.getHumidity(),7,2, humidity_str);
  String humidity = String(humidity_str);
  
  char temperature_str[8] = "";
  dtostrf(sht31.getTemperature(), 7, 2, temperature_str);
  String temperature = String(temperature_str);
  _temperature = temperature.toInt();
  
  return "\""+_name+"\":{\"hum\":"+humidity+",\"temp\":"+temperature+"}";
}

String TempHumSensor::get_name() {
  return _name;
}

float TempHumSensor::get_temp(int i2c_adr) {
  return sht31.getTemperature(i2c_adr);
}

float TempHumSensor::get_hum(int i2c_adr) {
  return sht31.getHumidity(i2c_adr);
}

bool TempHumSensor::init(int i2c_adr) {
   return sht31.begin(i2c_adr);
}

//____________________ CO2 __________________________
// I2C communication

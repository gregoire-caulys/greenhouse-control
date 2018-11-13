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
 _temperature = 21;}

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

bool TempHumSensor::init(int i2c_adr) {
   return sht31.begin(i2c_adr);
}


//____________________ TEMP HUMID DHT __________________________
// analog communication




//____________________ MOISTURE __________________________
// analog communication

MoistureSensor::MoistureSensor(String name, int pin) : _name(name), _pin(pin) {
}

String MoistureSensor::get_measurements() {
  int moisture = analogRead(_pin);
  return "\""+_name+"\":"+String(moisture);
}

String MoistureSensor::get_name() {
  return _name;
}

void MoistureSensor::init() {
}

//____________________ FLOWMETER __________________________
// analog communication

FlowMeter::FlowMeter(String name, int pin) : _name(name), _pin(pin) {
}

String FlowMeter::get_measurements() {
  int flow = analogRead(_pin);
  return "\""+_name+"\":"+String(flow);
}

String FlowMeter::get_name() {
  return _name;
}

void FlowMeter::init() {
}



//______________________ CO2 __________________________
// UART serial communication

const unsigned char cmd_get_sensor[] =
{
    0xff, 0x01, 0x86, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x79
};

Co2Sensor::Co2Sensor(String name, uint8_t tx, uint8_t rx): _name(name), _sensor(tx, rx) {
}

String Co2Sensor::get_measurements() {
  return "\""+_name +"\":{\"temp\":"+String(_temperature)+",\"co2\":"+String(_CO2PPM)+"}";
}

String Co2Sensor::get_name() {
  return _name;
}

void Co2Sensor::init() {
  _sensor.begin(9600);
}

bool Co2Sensor::data_receive()
{
    byte data[9];
    int i = 0;

    //transmit command data
    for(i=0; i<sizeof(cmd_get_sensor); i++)
    {
        _sensor.write(cmd_get_sensor[i]);
    }
    delay(10);
    //begin receiving data
    if(_sensor.available())
    {
        while(_sensor.available())
        {
            for(int i=0;i<9; i++)
            {
                data[i] = _sensor.read();
            }
        }
    }

    for(int j=0; j<9; j++)
    {
        Serial.print(data[j]);
        Serial.print(" ");
    }
    Serial.println("");

    if((i != 9) || (1 + (0xFF ^ (byte)(data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7]))) != data[8])
    {
        return false;
    }

    _CO2PPM = (int)data[2] * 256 + (int)data[3];
    _temperature = (int)data[4] - 40;

    return true;
}

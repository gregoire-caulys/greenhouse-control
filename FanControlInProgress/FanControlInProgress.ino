
//TODO: change motor control to use proper temperature value
//TODO: Integrate previous control code to current code
//TODO: Remove use of tachometers. Shouldn't need.
//TODO: recieve motor speed command from Pi
//TODO: figure out power supply for all 6 fans
//TODO: confirm that setValue line works as expected and that casting doesn't cause issues
//TODO: determine gain for proportional control
//TODO: replace debug serial printing with actual communication to pi
//TODO: make sure code can handle relays for all 3 levels
//TODO: see if header or cpp file need to be changed to handle 3 levels of sensors

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

//CO2 Sensor
//      - address = 0x5A
//      - will also need to pull address pin up(?) to interface with multiple sensors



//Included Libraries:
#include <TimerOne.h> //Timer 1 library
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SI114X.h>
#include <SHT31.h>
#include <DHT.h>
#include "sensors.h"

//Defined Addresses:
#define SHT31_ADDR_OFF        0x44
#define SHT31_ADDR_ON    0x45

#define HUMID_PIN         0
#define HEAT_PIN          1
#define PUMP_PIN_1        2
#define PUMP_PIN_2        3

//Constants
const int SERIAL_MESSAGE_LENGTH = 1;
const int NUMBER_LEVELS = 3;
const int BAUD_RATE = 9600;
const long TIMER_LENGTH = 1000000; //length of timer in microseconds
const double TIMER_LENGTH_SECONDS = (double) (TIMER_LENGTH / 1000000);
const double TEMP_SENSOR_CONVERSION = 0.48828125;
const int RELAYS_NB = 4; //not including relays per floor, e.g. for lights. This are handled seperately

const int DEBUG_MODE = 1;

//Pin Definitions
//These pin assignments are arbitrary - can change to other digital pins if necessary
const int RELAY_PIN[RELAYS_NB] = {53,51,49,47}; // Pins for: humid, heat, pump1, pump2
const int LIGHT_PIN[] = {22, 24, 26};
const int FAN_PIN[] = {8,9,5};
const int TEMP_PIN[] = {31, 33, 35}; //digital pins to toggle address of the temp/hum sensors.
const int TEMP_OUT_PIN = 37; //for outside temp sensor
//const int CO2_PIN[] = {41,43,45};

//Variables
int k = 0;
int inbyte = 0;

int set_value[] = {0, 0, 0};
int rx_val = 0;
double target_temp[] = {21, 21, 21}; //target temperature, degrees C
int target_hum[] = {50, 50, 50}; //target humidity, [%]? NOT CURRENTLY USED
double meas_temp[] = {0, 0, 0}; 
double temp_gain = 1; //proportional control gain for temperature control

unsigned long TimeLight[] = {0, 0, 0};
unsigned long TimeLightOn[] = {(unsigned long)16 * 3600, (unsigned long)16 * 3600, (unsigned long)16 * 3600}; // chiffre avant le facteur represente temps actviation en heures
unsigned long TimeLightOff[] = {(unsigned long)8 * 3600, (unsigned long)8 * 3600, (unsigned long)8 * 3600}; //need to cast in matrix to avoid int overflow
unsigned long TimePump1 = 0;
unsigned long TimePump1On = 1 * 3600;
unsigned long TimePump1Off = 5 * 3600;
unsigned long TimePump2 = 0;
unsigned long TimePump2On = 1 * 3600;
unsigned long TimePump2Off = 5 * 3600;

// Declaration of sensor object instances
SHT31 sht31 = SHT31();
LightSensor light_sensor[] = {LightSensor("light_win0"), LightSensor("light_win1"), LightSensor("light_win2")};
TempHumSensor temp_hum_sensor_in[] = {TempHumSensor("temp_hum_in0"), TempHumSensor("temp_hum_in1"), TempHumSensor("temp_hum_in2")};
TempHumSensor temp_hum_sensor_out("temp_hum_out");


//Setup
void setup() {
  Serial.begin(BAUD_RATE);
  delay(500);

  // initialize sensors
  for (int i = 0; i < NUMBER_LEVELS; i++) {
    light_sensor[i].init();
    temp_hum_sensor_in[i].init(SHT31_ADDR_ON); //TODO: FIGURE OUT HOW TO DEAL WITH MULTIPLE SENSOR INITIALIZATIONS
    //co2_sensor[i].init();
  }
  temp_hum_sensor_out.init(SHT31_ADDR_ON);

  //set pins
  for (int i = 0; i < NUMBER_LEVELS; i++) {
    pinMode(FAN_PIN[i], OUTPUT); // motor speed output
    pinMode(LIGHT_PIN[i], OUTPUT); // pins for relay light control
    pinMode(TEMP_PIN[i], OUTPUT); // pins for toggling I2C address of temp/hum sensor
    //pinMode(CO2_PIN[i],OUTPUT); //pins for toggling I2C address of C02 sensor
  }
  for (k = 0; k < RELAYS_NB; k++) {
    pinMode(RELAY_PIN[k], OUTPUT);
  }
  pinMode(TEMP_OUT_PIN, OUTPUT);
  //set starting value to 0
  analogWrite(FAN_PIN[0], rx_val);
  digitalWrite(TEMP_PIN[0], LOW); //just in case
  digitalWrite(TEMP_PIN[1], LOW);
  digitalWrite(TEMP_PIN[2], LOW);
  digitalWrite(TEMP_OUT_PIN, LOW);

  //Timer setup for interrupt driven control:
  Timer1.initialize(TIMER_LENGTH); //initializes timer1 with a 1s (1 million microsecond) period (max with a single timer/counter is ~8.3 seconds)
  Timer1.attachInterrupt(controlGreenHouse); //timer1 interrupt which controls fans based on temp
  interrupts(); //enable interrupts
}

//Interrupt Functions:
void controlGreenHouse() { //interrupt driven control of all actuators
  controlFans();
  controlLights();
  controlPumps();
}

void controlFans() { //Every 1 second, modify pwm to each pair of fans; get the temperature and convert it to celsius
  for (int i = 0; i < NUMBER_LEVELS; i++) {
    double temp = 0;//(double) analogRead(TEMP_PIN[i]); //TODO CHANGE THIS
    meas_temp[i] =  temp * TEMP_SENSOR_CONVERSION;
  }

  for (int i = 0; i < NUMBER_LEVELS; i++)
  {
    //motor proportional speed control:
    set_value[i] = (target_temp[i] - meas_temp[i]) * temp_gain;
    //check to make sure setValue is within valid range. Correct if not.
    if (set_value[i] > 255) {
      set_value[i] = 255;
    }
    else if (set_value[i] > 10 && set_value[i] < 25) {
      set_value[i] = 25; //stalling was found to occur at pwm = 10. Startup required pwm of at least 20. Setting overall min to 25 for simplicity.     }
    }
    else if (set_value[i] <= 10) {
      set_value[i] = 0; //For very low commands. Simply turn off the fan
    }
    if (DEBUG_MODE == 1) {
      //debugPrint1(i);
    }
  }
}

void controlLights() {
  for (int i = 0; i < NUMBER_LEVELS; i++) {
    TimeLight[i] = (TimeLight[i]++) % (TimeLightOff[i] + TimeLightOn[i]); //modulo operator will reset timer counter when it overflows
    if (TimeLight[i] <= TimeLightOn[i]) {
      digitalWrite(LIGHT_PIN[i], HIGH);
    }
    else if ((TimeLight[i] > TimeLightOn[i]) && (TimeLight[i] <= (TimeLightOff[i] + TimeLightOn[i]))) {
      digitalWrite(LIGHT_PIN[i], LOW);
    }
  }
}

void controlPumps() {
  controlPump1();
  controlPump2();
}

void controlPump1() {
  TimePump1 = (TimePump1++) % (TimePump1On + TimePump1Off);
  if (TimePump1 <= TimePump1On) {
    digitalWrite(RELAY_PIN[PUMP_PIN_1], HIGH);
  }
  else if ((TimePump1 > TimePump1On) && (TimePump1 <= (TimePump1Off + TimePump1On))) {
    digitalWrite(RELAY_PIN[PUMP_PIN_1], LOW);
  }
}

void controlPump2() {
  TimePump2 = (TimePump2++) % (TimePump2On + TimePump2Off);
  if (TimePump2 <= TimePump2On) {
    digitalWrite(RELAY_PIN[PUMP_PIN_2], HIGH);
  }
  else if ((TimePump2 > TimePump2On) && (TimePump2 <= (TimePump2Off + TimePump2On))) {
    digitalWrite(RELAY_PIN[PUMP_PIN_2], LOW);
  }
}

void debugPrint1(int i) {
  Serial.print("set_value = ");
  Serial.println(set_value[i]);
}

void debugPrint2() {
  Serial.print("I received: ");
  Serial.println(rx_val);
}

void debugControl() {
  //when enough bytes have been sent to serial port, process message
  int num_bytes = Serial.available();
  Serial.print("there are x bytes: ");
  Serial.println(num_bytes);
  rx_val = 0;
  int read_val = 0;
  int dec_mult = 1;
  int mult = 1;
  for (int i = num_bytes; i > 0; i--) {
    read_val = Serial.read();
    if (read_val != 10) {
      Serial.print(read_val);
      Serial.print(" ");
      mult = 1;
      for (int j = i - 2; j > 0; j--) {
        mult = mult * 10;
      }
      Serial.println((read_val - 48)*mult);
      rx_val += (read_val - 48) * mult;
    }
  }
  debugPrint2();
  analogWrite(FAN_PIN[0], rx_val);
  delay(2000);
  analogWrite(FAN_PIN[1], rx_val);
  delay(2000);
  analogWrite(FAN_PIN[2], rx_val);
  delay(2000);
}

void debugSensors() {
  //send sensor vals for: light sens 0-2, temp_hum sens 0-2, temp_hum_out

  //read all temp sensors:
  digitalWrite(TEMP_PIN[0], HIGH);
  
  String temp0 = temp_hum_sensor_in[0].get_measurements();
  digitalWrite(TEMP_PIN[0], LOW);
  digitalWrite(TEMP_PIN[1], HIGH);
  String temp1 = temp_hum_sensor_in[1].get_measurements();
  digitalWrite(TEMP_PIN[1], LOW);
  digitalWrite(TEMP_PIN[2], HIGH);
  String temp2 = temp_hum_sensor_in[2].get_measurements();
  digitalWrite(TEMP_PIN[2], LOW);
  digitalWrite(TEMP_OUT_PIN, HIGH);
  String tempOut = temp_hum_sensor_in[2].get_measurements();
  digitalWrite(TEMP_OUT_PIN, LOW);
  String measurements[SENSORS_NB] =
  {
    //light_sensor[0].get_measurements(),
    //light_sensor[1].get_measurements(),
    //light_sensor[2].get_measurements(),
    // NB : the two temp_hum sensors take each 100ms
    // to get measurements, 100x slower than the others
    temp0,temp1,temp2,tempOut};
  Serial.println(build_json(measurements));
}

void loop() {
  //delay(200);
  if (DEBUG_MODE == 1) {

    if (Serial.available() > 3) {
      debugControl();
    }
    //debugSensors();
  }
  else {

    if (Serial.available()) {
      char instruction = Serial.read();

      // case s : send sensor measurements
      if (instruction == 's')
      {
        //send sensor vals for: light sens 0-2, temp_hum sens 0-2, temp_hum_out
        String measurements[SENSORS_NB] =
        { light_sensor[0].get_measurements(),
          light_sensor[1].get_measurements(),
          light_sensor[2].get_measurements(),
          // NB : the two temp_hum sensors take each 100ms
          // to get measurements, 100x slower than the others
          temp_hum_sensor_in[0].get_measurements(),
          temp_hum_sensor_in[1].get_measurements(),
          temp_hum_sensor_in[2].get_measurements(),
          temp_hum_sensor_out.get_measurements()
        };

        Serial.println(build_json(measurements));
      }
    }
  }
}

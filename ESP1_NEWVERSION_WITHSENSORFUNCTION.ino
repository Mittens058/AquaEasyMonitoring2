//for blynk
#define BLYNK_TEMPLATE_ID "TMPL6-kTjpHlS"
#define BLYNK_TEMPLATE_NAME "AquaEasy Monitoring"
#define BLYNK_AUTH_TOKEN "n_nR6fFi1MIEU5bvtcOj1VEqK8qsqaDg"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "TP-Link_Extender";  // type your wifi name
char pass[] = "";  // type your wifi password

BlynkTimer timer; //import blynk timer function
int timedelay = 500;
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DFRobot_ESP_PH_WITH_ADC.h>
#include <EEPROM.h>
#include "GravityTDS.h"

GravityTDS gravityTds;
float temperature = 27;
float saltvalue;

// Pin definitions
#define SENSOR_PIN 32                 // DS18B20 sensor's input pin
// MOVE TO ESP#2 #define PH_PIN 32    // Analog input pin for pH sensor
#define TdsSensorPin 34             // Analog input pin for TDS sensor
#define DO_PIN 35                     // input pin for DO sensor    
#define HEATER_PIN 16                 // output pin to water heater
#define AERATOR_PIN 17                // output pin to air pump
//#define WP_UV_PIN 18                  // output pin to extension (water pump + UV )
//#define SERVO_PIN 19                  // output pin for servo motor to control feeder

#define VREF 3300    //VREF (mv)
#define ADC_RES 4096 //ADC Resolution

//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0

#define READ_TEMP (25) //Current water temperature ℃, Or temperature sensor function

//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (167) //mv
#define CAL1_T (25)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300) //mv
#define CAL2_T (15)   //℃

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

// Relay and motor control variables
bool activateRelay = false;
bool activateMotor = false;
unsigned long relayActivationTime = 0;
unsigned long motorActivationTime = 0;

// Temperature sensor setup
OneWire oneWire(SENSOR_PIN);
DallasTemperature DS18B20(&oneWire);

// pH sensor setup
DFRobot_ESP_PH_WITH_ADC ph;

//SETUP FOR MANUAL/AUTO

BLYNK_WRITE(V10){ // maanual control
  int mode = param.asInt(); // assigning incoming value from pin V10 to a variable
}

BLYNK_WRITE(V6) // this command is listening when something is written to V1
{
  int heater_switch = param.asInt(); // assigning incoming value from pin V1 to a variable

  if(heater_switch == 1){
    digitalWrite(HEATER_PIN, HIGH);
  }
  else{
    digitalWrite(HEATER_PIN, LOW);
  }
}
BLYNK_WRITE(V5) // this command is listening when something is written to V1
{
  int aerator_switch = param.asInt(); // assigning incoming value from pin V1 to a variable

  if(aerator_switch == 1){
    digitalWrite(AERATOR_PIN, HIGH);
  }
  else{
    digitalWrite(AERATOR_PIN, LOW);
  }
}

//SETUP FOR MANUAL/AUTO
bool mode = 0; // Initialize mode outside BLYNK_WRITE(V10)
bool heater_switch = 0; // Initialize heater_switch outside BLYNK_WRITE(V6)
bool aerator_switch = 0; // Initialize aerator_switch outside BLYNK_WRITE(V5)



void setup() {
  Serial.begin(115200);
  // Temperature sensor setup
  DS18B20.begin();
  //TDS setup
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(3.3); //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(4096); //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin(); //initialization
  // pH sensor setup
  //ph.begin();
  // Relay setup
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(AERATOR_PIN, OUTPUT);
  //pinMode(WP_UV_PIN, OUTPUT);
  //pinMode(SERVO_PIN, OUTPUT);

  digitalWrite(HEATER_PIN, LOW);
  digitalWrite(AERATOR_PIN, LOW);
  //digitalWrite(SERVO_PIN, LOW);
  //digitalWrite(SERVO_PIN, LOW);

  Blynk.begin(auth, ssid, pass);
  timer.setInterval(1000L, sendSensor);
}

void sendSensor(){
  // temperature read
  DS18B20.requestTemperatures();
  float temperatureC = DS18B20.getTempCByIndex(0);
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.print(" °C\t");
  Blynk.virtualWrite(V0,temperatureC);


  // dissolved oxygen read
  Temperaturet = (uint8_t)READ_TEMP;
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
  float DOValue = readDO(ADC_Voltage, Temperaturet);
  //Serial.print("Temperaturet:\t" + String(Temperaturet) + "\t");
  //Serial.print("ADC RAW:\t" + String(ADC_Raw) + "\t");
  //Serial.print("ADC Voltage:\t" + String(ADC_Voltage) + "\t");
  //Serial.println("DO:\t" + String(readDO(ADC_Voltage, Temperaturet)) + "\t"); //string
  Serial.print("DO:\t");
  Serial.print(DOValue/1000);
  Serial.print("mg/L\t");
  Blynk.virtualWrite(V1,DOValue/1000);

  // Salinity read
  //temperature = readTemperature(); //add your temperature sensor and read it
  gravityTds.setTemperature(temperature); // set the temperature and execute temperature compensation
  gravityTds.update(); //sample and calculate
  float tdsValue = gravityTds.getTdsValue(); // then get the value
  // Perform actions based on the mapped value using an if-else statement
  if (tdsValue<=500.00) 
  {
    float saltvalue = tdsValue/1000;
    Serial.print("Salinity: ");
    Serial.println(saltvalue); 
    Blynk.virtualWrite(V2, saltvalue); //can change tds value to ppt value based on needed 
  } 
  else if (tdsValue > 500.00 && tdsValue <= 700.00) 
  {
    float saltvalue = ((500+((tdsValue-500)/30)*100))/1000;
    Serial.print("Salinity: ");
    Serial.println(saltvalue);  
    Blynk.virtualWrite(V2, saltvalue); //can change tds value to ppt value based on needed
  } 
  else if (tdsValue > 700.00 && tdsValue <= 1000.00) 
  {
    float saltvalue = ((1000+((tdsValue-700)/20)*100))/1000;
    Serial.print("Salinity: ");
    Serial.println(saltvalue);
    Blynk.virtualWrite(V2, saltvalue); //can change tds value to ppt value based on needed
  } 
  else
  {
    float saltvalue = (2700+((tdsValue - 1000)/10)*100)/1000;
    Serial.print("Salinity:\t");
    Serial.println(saltvalue);  
    Blynk.virtualWrite(V2, saltvalue); //can change tds value to ppt value based on needed
  } 

  delay(timedelay);

}
void loop(){
  
  Blynk.run();
  timer.run();
}
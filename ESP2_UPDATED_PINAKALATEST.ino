// for blynk
#define BLYNK_TEMPLATE_ID "TMPL6-kTjpHlS"
#define BLYNK_TEMPLATE_NAME "AquaEasy Monitoring"
#define BLYNK_AUTH_TOKEN "n_nR6fFi1MIEU5bvtcOj1VEqK8qsqaDg"

#define BLYNK_PRINT Serial

//install libraries
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP32Servo.h> // Library for Servo motor
#include <Wire.h>
#include <RTClib.h> // Library for RTC
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Distinguish RTC
RTC_DS3231 rtc;

// Set time feeder
const int oneHour = 19;
const int oneMin = 21;
const int oneSec = 0;
const int twoHour = 19;
const int twoMin = 22;
const int twoSec = 0;
const int threeHour = 19;
const int threeMin = 23;
const int threeSec = 0;

char days[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire, -1);

// Variables for servo motor
int pos = 180;
int servoPin = 9;

// Distinguish servo motor name
Servo servomotor;

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "TP-Link_Extender";  // type your wifi name
char pass[] = "";  // type your wifi password

BlynkTimer timer; //import blynk timer function
int timedelay = 500;

#include <DFRobot_ESP_PH_WITH_ADC.h>

#define PH_PIN 32         // Analog pin for pH sensor

const int trigPin = 33;
const int echoPin = 35;

#define VREF 3.3    //VREF (mv)
#define ADC_RES 4096 //ADC Resolution

#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

// pH sensor setup
DFRobot_ESP_PH_WITH_ADC ph;

//MAKE VARIABLES FOR PINS
const int inlet_valve_relay = 15;
const int pump_out_relay = 16;

const int filter_pump = 17;

//VALVE = 8, SMALL PUMP = 9, FILTER = 7, MANUAL CONTROL = 10


BLYNK_WRITE(V10){
  bool mode = param.asInt();
}

BLYNK_WRITE(V8){ // REFILL 
  bool inlet_switch = param.asInt();
  if (inlet_switch == 1){
    digitalWrite(inlet_valve_relay, HIGH);
  }
  else{
    digitalWrite(inlet_valve_relay, LOW);
  }
}

BLYNK_WRITE(V9){
  bool pump_out_switch = param.asInt();
}

BLYNK_WRITE(V7) { //MANUAL OVERRIDE OF RAS, TURN ON FO2R ONE HOUR AND AUTO OFF
  bool filter_switch = param.asInt();

  if(filter_switch == 1){
    digitalWrite(filter_pump, HIGH);
  }
  else{
    digitalWrite(filter_pump, LOW);
  }
}
int percentage_level;

//define blynk switch variable
bool mode;
bool inlet_switch;
bool pump_out_switch;
bool filter_switch;


void setup() {
  Serial.begin(115200);
  // pH sensor setup
  ph.begin();

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output` 
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(inlet_valve_relay, OUTPUT); // Sets the inlet as an Output`
  pinMode(pump_out_relay, OUTPUT); // Sets the pump as an Output`
  pinMode(filter_pump, OUTPUT); // Sets the pump as an Output`

  //SET INITIAL RELAY STATE AT O
  digitalWrite(inlet_valve_relay, LOW);
  digitalWrite(pump_out_relay, LOW);
  digitalWrite(filter_pump, LOW);

  Blynk.begin(auth, ssid, pass);
  timer.setInterval(1000L, sendSensor);

  // Check if the RTC is connected properly
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  // Check if the RTC lost power and if so, set the time
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  servomotor.attach(servoPin);
    // Initialize the rtc object
  
  
  rtc.begin();

}

void sendSensor() {

  // pH reading
  float voltagePH = analogRead(PH_PIN) / 4096.0 * 3300;
  float pHValue = ph.readPH(voltagePH, 25);
  ph.calibration(voltagePH, 25);

  Serial.print("pH: ");
  Serial.print(pHValue);
  Serial.print("\t");
  Blynk.virtualWrite(V3,pHValue);
  delay(timedelay);

  delay(1000); // Delay for 1 second


  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
  
  // Convert to inches
  distanceInch = distanceCm * CM_TO_INCH;

  int sonicdistance = 8;

  float percentage_level = 100 - ((distanceInch-4)/(sonicdistance))*100;
  
  // Prints the distance in the Serial Monitor
  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);
  Serial.print("Distance (inch): ");
  Serial.println(distanceInch);
  Blynk.virtualWrite(V4,percentage_level);


  Serial.print("Water Level: ");
  Serial.println(percentage_level);
  delay(1000);
    

    //PH CONTROL VALUES
  if(pHValue < 7 && pHValue >=8.5 && mode == 0){
    //FOR WATER CHANGE
    static bool draining = true;  // Flag to track the draining/filling state
    if (draining)
    {
      // Water level is above 50%, drain the water
      digitalWrite(pump_out_relay, HIGH);
      digitalWrite(inlet_valve_relay, LOW);
      Serial.println("Draining water");
      Blynk.virtualWrite(V9,1);
      Blynk.virtualWrite(V8,0);

      if (percentage_level <= 50)
      {
        draining = false;  // Switch to filling state
        Serial.println("Water level reached 50%, switching to filling");
      }

      // Water level is below 80%, fill the water
      digitalWrite(inlet_valve_relay, HIGH);
      digitalWrite(pump_out_relay, LOW);
      Serial.println("Filling water");
      Blynk.virtualWrite(V9,0);
      Blynk.virtualWrite(V8,1);

      if (percentage_level >= 80)
      {
        draining = true;  // Switch to draining state
        Serial.println("Water level reached 80%, switching to draining");
      }
    }
  }


  else{
    digitalWrite(inlet_valve_relay, LOW);
    digitalWrite(pump_out_relay, LOW);
  }
}


void loop(){
  
  Blynk.run();
  timer.run();

  // Time display for Serial Monitor
  DateTime now = rtc.now();

  // Loop for servo motor
  if (now.hour() == oneHour && now.minute() == oneMin && now.second() == oneSec) {
    for (pos = 180; pos >= 0; pos = pos - 10) {
      servomotor.write(pos);
      delay(100);
    }

    for (pos = 0; pos <= 180; pos = pos + 1) {
      servomotor.write(pos);
    }
  }
  if (now.hour() == twoHour && now.minute() == twoMin && now.second() == twoSec) {
    for (pos = 180; pos >= 0; pos = pos - 10) {
      servomotor.write(pos);
      delay(100);
    }

    for (pos = 0; pos <= 180; pos = pos + 1) {
      servomotor.write(pos);
    }
  }

  if (now.hour() == threeHour && now.minute() == threeMin && now.second() == threeSec) {
    for (pos = 180; pos >= 0; pos = pos - 10) {
      servomotor.write(pos);
      delay(100);
    }

    for (pos = 0; pos <= 180; pos = pos + 5) {
      servomotor.write(pos);
    }
  }

  delay(1000);

}



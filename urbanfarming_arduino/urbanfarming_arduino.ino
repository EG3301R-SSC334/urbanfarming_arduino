#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DFRobot_EC.h>

#define EC_PIN A1
#define FLOAT_1 2
#define FLOAT_2 3
#define FLOAT_3 4
#define MAIN_PUMP 5

struct __attribute__((__packed__)) sensorData {
  float EC;
  float pH;
  float temp;
  float humidity;
  byte contA;
  byte contB;
  byte contC;
};

const int data_union_size = sizeof(sensorData);

union dataPacket_t {
  sensorData sensors;
  byte byteArray[data_union_size];
};

dataPacket_t data;

const long readInterval = 3000;         // 30 sec
const long postInterval = 6000;        // 10 min
const long peristalticDuration = 1000;   // 1 sec
const long pumpOnDuration = 300000;      // 5 min
const long pumpOffDuration = 720000;     // 12 min

unsigned long currentMillis = 0;
unsigned long previousReadMillis = 0;
unsigned long previousPostMillis = 0;
unsigned long previousPeristalticMillis = 0;
unsigned long previousPumpMillis = 0;

byte pumpState = LOW;
float voltage,ecValue,temperature, humidity, pH;
byte contA = LOW, contB = LOW, contC = LOW;

Adafruit_BME280 bme;
DFRobot_EC ec;

void setup() {
  Serial.begin(9600);
  ec.begin();
  bme.begin();
  pinMode(FLOAT_1, INPUT_PULLUP);
  pinMode(FLOAT_2, INPUT_PULLUP);
  pinMode(FLOAT_3, INPUT_PULLUP);
  pinMode(6, OUTPUT);
}

void loop() {
  currentMillis = millis();
  pump();
  readData();
  postData();
  readCommand();
}

void pump() {
  if (pumpState == LOW) {
    if (currentMillis - previousPumpMillis >= pumpOffDuration) {
      pumpState == HIGH;
      digitalWrite(MAIN_PUMP, pumpState);
      previousPumpMillis += pumpOffDuration;
    }
  } else {
    if (currentMillis - previousPumpMillis >= pumpOnDuration) {
      pumpState == LOW;
      digitalWrite(MAIN_PUMP, pumpState);
      previousPumpMillis += pumpOnDuration;
    }
  }
}

void readData() {
  if (currentMillis - previousReadMillis >= readInterval) {
//    temperature = bme.readTemperature();
//    humidity = bme.readHumidity();
//    voltage = analogRead(EC_PIN)/1024.0*5000;
//    ecValue = ec.readEC(voltage, temperature);
//    contA = digitalRead(FLOAT_1);
//    contB = digitalRead(FLOAT_2);
//    contC = digitalRead(FLOAT_3);
    temperature = 25.5;
    humidity = 70;
    voltage = 3000;
    pH = 6.5;
    ecValue = ec.readEC(voltage, temperature);
    contA = digitalRead(FLOAT_1);
    contB = digitalRead(FLOAT_2);
    contC = digitalRead(FLOAT_3);
    previousReadMillis += readInterval;
  }
}

void postData() {
  if (currentMillis - previousPostMillis >= postInterval) {
    data.sensors.temp = temperature;
    data.sensors.EC = ecValue;
    data.sensors.humidity = humidity;
    data.sensors.pH = pH;
    data.sensors.contA = contA;
    data.sensors.contB = contB;
    data.sensors.contC = contC;

    for (int i=0; i < data_union_size; i++) {
      if (data.byteArray[i] <16) {
        Serial.print(0);
      }
      Serial.print(data.byteArray[i], HEX);
    }
    Serial.print("\n");
    
    previousPostMillis += postInterval;
  }
}

void readCommand() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    
//  check length of string
//  split string into components
//  do stuff
  }
}

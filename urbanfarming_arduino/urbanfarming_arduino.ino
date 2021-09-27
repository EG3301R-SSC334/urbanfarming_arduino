#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "DFRobot_EC.h"

#define EC_PIN A1
#define FLOAT_1 2
#define FLOAT_2 3
#define FLOAT_3 4
#define MAIN_PUMP 5

const int readInterval = 30000;         // 30 sec
const int postInterval = 600000;        // 10 min
const int peristalticDuration = 1000;   // 1 sec
const int pumpOnDuration = 300000;      // 5 min
const int pumpOffDuration = 720000;     // 12 min
const int BUFFER_SIZE = 32;
char buf[BUFFER_SIZE];

unsigned long currentMillis = 0;
unsigned long previousReadMillis = 0;
unsigned long previousPostMillis = 0;
unsigned long previousPeristalticMillis = 0;
unsigned long previousPumpMillis = 0;

byte pumpState = LOW;
float voltage,ecValue,temperature;
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
    temperature = bme.readTemperature();
    voltage = analogRead(EC_PIN)/1024.0*5000;
    ecValue = ec.readEC(voltage, temperature);
    contA = digitalRead(FLOAT_1);
    contB = digitalRead(FLOAT_2);
    contC = digitalRead(FLOAT_3);
    previousReadMillis += readInterval;
  }
}

void postData() {
  if (currentMillis - previousPostMillis >= postInterval) {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println("C");
    
    Serial.print("Humidity: ");
    Serial.print(bme.readHumidity());
    Serial.println("%");
    
    Serial.print("EC: ");
    Serial.print(ecValue, 2);
    Serial.println("ms/cm");
  
    if (contA == LOW) {
      Serial.println("Container A low");
    }
    if (contB == LOW) {
      Serial.println("Container B low");
    }
    if (contC == LOW) {
      Serial.println("Container C low");
    }
    
    Serial.println();
    previousPostMillis += postInterval;
  }
}

void readCommand() {
  if (Serial.available() >= BUFFER_SIZE) {
    Serial.readBytes(buf, BUFFER_SIZE);
    Serial.print(buf);
  }
}

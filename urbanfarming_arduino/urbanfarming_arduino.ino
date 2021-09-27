#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "DFRobot_EC.h"

#define EC_PIN A1
#define FLOAT_1 2
#define FLOAT_2 3
#define FLOAT_3 4

float voltage,ecValue,temperature;

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
  temperature = bme.readTemperature();
  voltage = analogRead(EC_PIN)/1024.0*5000;
  ecValue = ec.readEC(voltage, temperature);
  
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("C");
  
  Serial.print("Humidity: ");
  Serial.print(bme.readHumidity());
  Serial.println("%");
  
  Serial.print("EC: ");
  Serial.print(ecValue, 2);
  Serial.println("ms/cm");

  if (digitalRead(FLOAT_1) == LOW) {
    Serial.println("Container 1 low");
  }

  if (digitalRead(FLOAT_2) == LOW) {
    Serial.println("Container 2 low");
  }

  if (digitalRead(FLOAT_3) == LOW) {
    Serial.println("Container 3 low");
  }
  
  Serial.println();
  delay(1000);
  ec.calibration(voltage,temperature);
}

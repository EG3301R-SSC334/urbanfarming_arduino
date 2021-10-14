#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DFRobot_EC.h>

#define EC_PIN A1

#define FLOAT_1     2
#define FLOAT_2     3
#define FLOAT_3     4
#define LIGHT_PIN   5
#define MAIN_PUMP   6
#define PUMP_1      7
#define PUMP_2      8
#define PUMP_3      9

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

const long dataInterval = 30000;    // 30 sec
long nutrientDuration = 1000;       // 1 sec
long waterDuration = 1000;          // 1 sec
long pumpOnDuration = 300000;       // 5 min
long pumpOffDuration = 720000;      // 12 min

unsigned long currentMillis = 0;
unsigned long previousDataMillis = 0;
unsigned long previousNutrientMillis = 0;
unsigned long previousWaterMillis = 0;
unsigned long previousPumpMillis = 0;

int nutrientControl = 0;
int waterControl = 0;

int nutrientState = HIGH;
int waterState = HIGH;
int pumpState = HIGH; // HIGH is OFF
float voltage,ecValue,temperature, humidity, pH;
int contA = LOW, contB = LOW, contC = LOW;

Adafruit_BME280 bme;
DFRobot_EC ec;

void setup() {
  Serial.begin(9600);
  ec.begin();
  bme.begin();
  pinMode(FLOAT_1, INPUT_PULLUP);
  pinMode(FLOAT_2, INPUT_PULLUP);
  pinMode(FLOAT_3, INPUT_PULLUP);
  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, HIGH);
  pinMode(MAIN_PUMP, OUTPUT);
  digitalWrite(MAIN_PUMP, HIGH);
  pinMode(PUMP_1, OUTPUT);
  digitalWrite(PUMP_1, HIGH);
  pinMode(PUMP_2, OUTPUT);
  digitalWrite(PUMP_2, HIGH);
  pinMode(PUMP_3, OUTPUT);
  digitalWrite(PUMP_3, HIGH);
}

void loop() {
  currentMillis = millis();
  pump();
  nutrient();
  water();
  readData();
  readCommand();
}

void pump() {
  if (pumpState == LOW) {
    if (currentMillis - previousPumpMillis >= pumpOnDuration) {
      pumpState = HIGH;
      digitalWrite(MAIN_PUMP, pumpState);
      previousPumpMillis += pumpOnDuration;
    }
  } else {
    if (currentMillis - previousPumpMillis >= pumpOffDuration) {
      pumpState = LOW;
      digitalWrite(MAIN_PUMP, pumpState);
      previousPumpMillis += pumpOffDuration;
    }
  }
}

void nutrient() {
  if (nutrientControl == 1) {
    if (nutrientState == LOW) {
      if (currentMillis - previousNutrientMillis >= nutrientDuration) {
        nutrientState = HIGH;
        nutrientControl = 0;
        digitalWrite(PUMP_1, nutrientState);
        digitalWrite(PUMP_2, nutrientState);
        previousNutrientMillis += nutrientDuration;
      }
    } else {
      previousPumpMillis = currentMillis;
      nutrientState = LOW;
      digitalWrite(PUMP_1, nutrientState);
      digitalWrite(PUMP_2, nutrientState);
    }
  } else {
    if (nutrientState == LOW) {
      nutrientState == HIGH;
      digitalWrite(PUMP_1, nutrientState);
      digitalWrite(PUMP_2, nutrientState);
    }
  }
}

void water() {
  if (waterControl == 1) {
    if (waterState == LOW) {
      if (currentMillis - previousWaterMillis >= waterDuration) {
        waterState = HIGH;
        waterControl = 0;
        digitalWrite(PUMP_3, waterState);
        previousWaterMillis += waterDuration;
      }
    } else {
      previousPumpMillis = currentMillis;
      waterState = LOW;
      digitalWrite(PUMP_3, waterState);
    }
  } else {
    if (waterState == LOW) {
      waterState == HIGH;
      digitalWrite(PUMP_3, waterState);
    }
  }
}

void readData() {
  if (currentMillis - previousDataMillis >= dataInterval) {
    voltage = analogRead(EC_PIN)/1024.0*5000;
    data.sensors.temp = bme.readTemperature();
    data.sensors.EC = ec.readEC(voltage, temperature);
    data.sensors.humidity = bme.readHumidity();
    data.sensors.pH = pH;
    data.sensors.contA = digitalRead(FLOAT_1);
    data.sensors.contB = digitalRead(FLOAT_2);
    data.sensors.contC = digitalRead(FLOAT_3);

    for (int i=0; i < data_union_size; i++) {
      if (data.byteArray[i] <16) {
        Serial.print(0);
      }
      Serial.print(data.byteArray[i], HEX);
    }
    Serial.print("\n");
    
    previousDataMillis += dataInterval;
  }
}

void readCommand() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    char control = line.charAt(0);
    if (control == '1') {
      pumpOnDuration = line.substring(1, 7).toInt();
      pumpOffDuration = line.substring(7, 13).toInt();
    } else if (control == '2') {
      int pump = line.charAt(1);
      if (pump == 1) {
        nutrientDuration = line.substring(2,8).toInt();
        nutrientControl = 1;
      } else if (pump == 2) {
        waterDuration = line.substring(2,8).toInt();
        waterControl = 1;
      }
    } else if (control == '3') {
        char state = line.charAt(1);
        if (state == '1') {
          digitalWrite(LIGHT_PIN, HIGH);
        } else if (state == '0') {
          digitalWrite(LIGHT_PIN, LOW);
        }
    }
  }
}

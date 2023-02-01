#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Stepper.h>

#include "Timer.h"

#define DHTPIN 5
#define DHTTYPE DHT11

#define TRIGPIN 9
#define ECHOPIN 10

#define LIGHT_WHITE 18
#define LIGHT_COLORS 19

#define MOTOR_DIR 7
#define MOTOR_STEP 8

#define IN1 24
#define IN2 25
#define IN3 29
#define IN4 30

#define SOILMOIST 6

const int stepsPerRevolution = 2048;

DHT_Unified dht(DHTPIN, DHTTYPE);

struct __attribute__((packed)) sensorData {
  char control;
  int8_t light;
  float temperature;
  int8_t humidity;
  int8_t soilMoisture;
  int8_t waterTank;
  int8_t irrigation;
};
struct __attribute__((packed)) controls {
  char control;
  int8_t lightPower;
  int8_t irrigation;
};

sensorData data;
controls control;

int newData = 0;

int sensorCounter = 0;

int motor_counter = 0;

int waterCounter = 0;
long duration, cm;

bool shouldSendData = false;
bool shouldFetchControl = false;
bool startWatering = false;

Timer controlTimer;
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);
void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);

  pinMode(SOILMOIST, INPUT);

  pinMode(TRIGPIN, OUTPUT);  // Sets the TRIGPIN as an Output
  pinMode(ECHOPIN, INPUT);   // Sets the echoPin as an Input

  pinMode(LIGHT_WHITE, OUTPUT);
  pinMode(LIGHT_COLORS, OUTPUT);

  controlTimer.begin(controlHandler, 1000);
  myStepper.setSpeed(5);

  controlTimer.start();
  dht.begin();
}

void loop() {
  if (shouldSendData) {
    readTempHum();
    readSoilMoisture();
    readWaterLevel();
    data.control = '<';
    data.light = control.lightPower;
    Serial1.write((uint8_t*)&data, sizeof(data));
    shouldSendData = false;
    if (data.soilMoisture < 20) {
      startIrrigation();
    }
  }
  if (shouldFetchControl) {
    readControls();
    if (control.irrigation == 1) {
      control.irrigation = 0;
      startIrrigation();
    }
    controlLight();
    shouldFetchControl = false;
  }
  handleIrrigation();
}

void controlHandler() {
  shouldFetchControl = true;
  sensorCounter++;
  if (startWatering) {
    waterCounter++;
  }
  if (sensorCounter >= 4) {
    shouldSendData = true;
    sensorCounter = 0;
  }
}

void startIrrigation() {
  startWatering = true;
  myStepper.step(-512);
}

void handleIrrigation() {
  if (waterCounter >= 6) {
    stopIrrigation();
  }
}

bool shouldWaterPlant() {
  bool returnVal = false;
  if (waterCounter < 6 && startWatering) {
    returnVal = true;
  } else {
    startWatering = false;
    returnVal = false;
  }

  return returnVal;
}

void stopIrrigation() {
  waterCounter = 0;
  control.irrigation = 0;
  startWatering = false;
  myStepper.step(512);
}

void controlLight() {
  float value = control.lightPower * 2.55;
  analogWrite(LIGHT_COLORS, value);
  analogWrite(LIGHT_WHITE, value);
}

void readControls() {
  controls data;
  int counter = 0;
  if (Serial1.available() > 0 && newData == 0) {
    if (Serial1.read() == '>') {
      newData = 1;
      int8_t* ptr = (int8_t*)&(data);
      ptr++;
      int counter = 0;
      int finish = 0;
      while (counter < sizeof(controls) - 1) {
        while (Serial1.available() <= 0)  //waits until there is new input
        {
          //do nothing
        }
        *ptr = Serial1.read();
        ptr++;
        counter++;
      }
      control.lightPower = data.lightPower;
      control.irrigation = data.irrigation;

      newData = 0;
      counter = 0;
    } else {
      //do nothing
    }
  }
}


void readTempHum() {
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    //Serial.println(F("Error reading temperature!"));
  } else {
    data.temperature = event.temperature;
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    //Serial.println(F("Error reading humidity!"));
  } else {
    data.humidity = event.relative_humidity;
  }
}

//  This function returns the analog soil moisture measurement
void readSoilMoisture() {
  // digitalWrite(sensorPower, HIGH);  // Turn the sensor ON                      //
  int val = analogRead(SOILMOIST);  // Read the analog value form sensor
  int moisture_percentage = 100 - (int)(((float)(val / 1023.00)) * 100);
  data.soilMoisture = moisture_percentage;
}

void readWaterlevel() {
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);
  pinMode(ECHOPIN, INPUT);
  duration = pulseIn(echoPin, HIGH);
  cm = (duration / 2) / 29.1;  // Divide by 29.1 or multiply by 0.0343
  int_8 precentage = (12-(float)cm / 12) * 100;
  data.waterTank = precentage;
}

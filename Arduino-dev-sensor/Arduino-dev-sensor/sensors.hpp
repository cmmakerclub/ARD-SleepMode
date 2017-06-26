#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

float _volume, _pitch, _roll, _batt, V_batt;
float _temp, _humid, _lat, _lon, _alt, _soundStatus;
uint16_t _lidStatus, _flameStatus, _press, _light, _carbon, _methane;
int _rssi;

float _tempBME, _humidBME, _pressBME;

#define ECHO  5
#define TRIG  7
extern uint8_t LED;
long duration;

void readDistance() {
  uint32_t sum = 0 ;
  for (int i = 1; i <= 3; i++) {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(5);
    digitalWrite(TRIG, LOW);

    pinMode(ECHO, INPUT);
    duration = pulseIn(ECHO, HIGH);

    if (_volume <= 20) {
      _volume = 20;
    } else if (_volume >= 500) {
      _volume = 500;
    }

    _volume = (duration / 2) / 29.1;
    sum += _volume;
  }
  _volume = sum / 3;
  Serial.print("_volume = ");
  Serial.println(_volume);
}


void readAllSensors() {
    _temp = bme.readTemperature();
    _humid = bme.readHumidity();
    _press = bme.readPressure() / 100.0F;

    readDistance();

    pinMode(A0, INPUT);
    _batt = analogRead(A0);

    // Serial.println("=== BME ===");
    // Serial.print("T = ");
    // Serial.print(_temp);
    // Serial.print(" H = ");
    // Serial.print(_humid);
    // Serial.print(" P = ");
    // Serial.print(_press);
    // Serial.print(" B = ");
    // Serial.print(_batt);
    // Serial.print(" D = ");
    // Serial.println(_volume);
    // Serial.println(millis() / 1000);

    digitalWrite(LED, HIGH);
}

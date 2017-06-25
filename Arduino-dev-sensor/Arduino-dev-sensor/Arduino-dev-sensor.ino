#include <Arduino.h>
#include <SPI.h>
#include "TEE_UC20.h"
#include "internet.h"
#include "gnss.h"
#include "CMMC_Interval.hpp"
#include "tcp.h"
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Sleep_n0m1.h>
#include "File.h"
#include "http.h"
#include <ArduinoJson.h>

HTTP http;
Sleep sleepCtrl;
float eepromFloatInitializedByte = 0.000f;
int eeAddress = 0;

struct EEPROMStructure {
  double lat;
  double lng;
  uint32_t sleepTimeS;
};

#define SEALEVELPRESSURE_HPA (1013.25)
// bool ret = tcp.Open("sock.traffy.xyz","10777");
//  bool ret = tcp.Open("api.traffy.xyz", "10777");
// bool ret = tcp.Open("red.cmmc.io","9991");
#define TCP_SERVER_ENDPOINT "128.199.143.200"
#define TCP_SERVER_PORT     "10777"
Adafruit_BME280 bme; // I2C

GNSS gps;
INTERNET net;
TCP tcp;
UC_FILE file;

CMMC_Interval interval2;

// JSON
StaticJsonBuffer<200> jsonBuffer;

#define SHOW_RAM 1
#define DEBUG_SERIAL 1

#if SHOW_RAM
#include <MemoryFree.h>
#endif

#define APN "internet"
//#define APN "bmta.fleet"
#define USER ""
#define PASS ""

#define BINID            "91"
//#define APPID           "SmartTrash"


//AltSoftSerial mySerial;

#define LED 13
#define MODE_PIN A4
#define SS_pin                A5
#define _volume_OK            (1<<0)
#define _lidStatus_OK         (1<<1)
#define _temp_OK              (1<<2)
#define _humid_OK             (1<<3)
#define _flameStatus_OK       (1<<4)
#define _soundStatus_OK       (1<<5)
#define _carbon_OK            (1<<6)
#define _methane_OK           (1<<7)
#define _light_OK             (1<<8)
#define _pitch_OK             (1<<9)
#define _roll_OK              (1<<10)
#define _press_OK             (1<<11)
#define _batt_OK              (1<<12)


#define addLat  1
#define addLon  2


#define ECHO  5
#define TRIG  7
long duration;


volatile char GNSS_data[58] = "";
String gps_data = "";
String gps_lat = "";
String gps_lon = "";
String gps_alt = "";

float _volume, _pitch, _roll, _batt, V_batt;
float _temp, _humid, _lat, _lon, _alt, _soundStatus;
uint16_t _lidStatus, _flameStatus, _press, _light, _carbon, _methane;

float _tempBME, _humidBME, _pressBME;

uint8_t gpsCounter = 0;
uint8_t stmTime = 10;
float localSleepTime = 5;
uint32_t machineCycle = 0;

#if DEBUG_SERIAL
void debug(String data) {
  Serial.println(data);
}
#endif

void setEEProm() {
  EEPROM.get(eeAddress, eepromFloatInitializedByte);
  Serial.println(eepromFloatInitializedByte, 3);

  // intialize EEPROM
  if (eepromFloatInitializedByte != 123.456f) {
    eepromFloatInitializedByte = 123.456f;
    // write first signature byte
    EEPROM.put(eeAddress, eepromFloatInitializedByte);
    // intialize eeprom structure
    EEPROMStructure defaultEEPROMValue = { 0.0f, 0.0f, 10 };
    eeAddress += sizeof(float);

    // write default value to eeprom
    EEPROM.put(eeAddress, defaultEEPROMValue);
    eeAddress += sizeof(EEPROMStructure);
  }
  // load eeprom
  else {
    // eeAddress = sizeof(float);
    EEPROMStructure eepromCached;
    EEPROM.get(0+sizeof(float), eepromCached);
    Serial.println("Read custom object from EEPROM: ");
    Serial.println(eepromCached.lat);
    Serial.println(eepromCached.lng);
    Serial.println(eepromCached.sleepTimeS);
  }
}


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


String netpieJsonString;
long getSleepTimeFromNetpie() {
    http.begin(1);
    Serial.println(F("Send HTTP GET"));
    http.url("http://api.netpie.io/topic/SmartTrash/time?retain&auth=YGO1C5bATVNctTE:wN7khNDXgadngRN5WxMGMc7z0");
    Serial.println(http.get());
    Serial.println(F("Clear data in RAM"));
    file.Delete(RAM,"*");
    Serial.println(F("Save HTTP Response To RAM"));
    http.SaveResponseToMemory(RAM,"netpie.json");
    Serial.println(F("Read data in RAM"));

    // clear String
    netpieJsonString = "";
    read_file(RAM, "netpie.json");
    Serial.println("READ FILE JSON");
    Serial.println(netpieJsonString);
    DynamicJsonBuffer jsonBuffer;
    JsonArray& root = jsonBuffer.parseArray(netpieJsonString.c_str());

    // Test if parsing succeeds.
    if (!root.success()) {
      Serial.println("parseObject() failed");
      return;
    }
    else {
      Serial.println("parsed OK.");
      JsonObject& netpieJsonObject = root[0];
      Serial.print("TIME PAYLOAD: ");
      const char* payload = netpieJsonObject["payload"];
      const char* topic = netpieJsonObject["topic"];
      const char* lastUpdated = netpieJsonObject["lastUpdated"];
      long payloadInt = String(payload).toInt();


      Serial.print("payload: ");
      Serial.println(payload);

      Serial.print("payloadInt: ");
      Serial.println(payloadInt);

      Serial.print("topic: ");
      Serial.println(topic);

      Serial.print("lastUpdated: ");
      Serial.println(lastUpdated);
      return payloadInt;
    }
  };

//////////////////////////////mainSETUP////////////////////////////////
bool open_tcp();

void data_out(char data)
{
  netpieJsonString += String(data);
}

void read_file(String pattern, String file_name)
{
  file.DataOutput =  data_out;
  file.ReadFile(pattern, file_name);
}

void setup()  {
  Serial.begin(9600);
  Serial2.begin(9600);  //  serial to stm

  Serial.println(millis() / 1000);
#if DEBUG_SERIAL
  Serial.println(F("Program Start."));
#endif
#if SHOW_RAM
  Serial.print(F("freeMemory()="));
  Serial.println(freeMemory());
#endif
  pinMode(LED, OUTPUT);
  pinMode(SS_pin, OUTPUT);
  pinMode(MODE_PIN, INPUT_PULLUP);
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);

  setEEProm();
  bme.begin();  // bme sensor begin

  int z = 0;
  while (z < 5) {
    digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                // wait for a second
    digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
    delay(10);
    z++;
  }

  if (digitalRead(MODE_PIN) == HIGH) {
    // localSleepTime 1 hour
    localSleepTime = 60;
#if DEBUG_SERIAL
    Serial.println("HIGH");
#endif
  } else {
    // localSleepTime 3 minute 3 * 60 = 180
    localSleepTime = 180 + 1000;
#if DEBUG_SERIAL
    Serial.println("LOW");
#endif
  }


  /////////////////////////////3G//////////////////////////////////
#if DEBUG_SERIAL
  gsm.Event_debug = debug;
#endif
  gsm.begin(&Serial3, 9600); // myserial
  gsm.PowerOn();
  while (gsm.WaitReady()) {}

  Serial.print(F("GetOperator --> "));
  Serial.println(gsm.GetOperator());
  Serial.print(F("SignalQuality --> "));
  _light = gsm.SignalQuality();
  Serial.println(gsm.SignalQuality());

  Serial.println(F("Disconnect net"));
  net.DisConnect();
  net.Configure(APN, USER, PASS);
  net.Connect();

  Serial.println(F("Show My IP"));
  Serial.println(net.GetIP());
  Serial.println(F("Start HTTP"));

#if DEBUG_SERIAL
  Serial.println(F("NET Connected"));
#endif
  long sleepTimeFromNetpie = getSleepTimeFromNetpie();
  Serial.print("SLEEP TIME [NETPIE] = ");
  Serial.println(sleepTimeFromNetpie);
  //////////////////////////////GPS//////////////////////////////
  gps.Start();
  gps.EnableNMEA();
  gps_data = gps.GetNMEA("GGA");
  gpsCounter = 0;
  bool gps_linked = true;
  uint32_t gpsTimeoutNextTick = millis() + 50 * 1000L ;
  while ((gps_data.substring(0, 8) == "$GPGGA,," ||
          gps_data.substring(0, 8) == "Please W")) {
    gps_data = gps.GetNMEA("GGA");

    // Serial.print("GNSS_data = ");
    // Serial.println(gps_data);

    digitalWrite(LED, HIGH);
    delay(5);
    digitalWrite(LED, LOW);
    delay(100);
    gpsCounter += 1;
    if (gpsCounter%5 == 0) {
      Serial.print(gpsCounter);
      Serial.println(" Wating GPS...");
    }

    // gps timeout
    if (millis() > gpsTimeoutNextTick) {
      Serial.println("GPS Timeout..");
      gps_lat = "0.0";
      gps_lon = "0.0";
      gps_alt = "0.0";
      gps_linked = false;
      break;
    }
  }

  // action after finish GPS searching...
  if (gps_linked) {
    if (gps_data.substring(0, 6) == "$GPGGA") {
      gps_data.toCharArray(GNSS_data, 58);

      String gps_cal_s = gps_data.substring(18, 27);
      float gps_cal_f = gps_cal_s.toFloat();
      gps_cal_f /= 60.0f;
      uint32_t gps_cal_l = gps_cal_f * 10000000;

      gps_lat += GNSS_data[16] ;
      gps_lat += GNSS_data[17] ;
      gps_lat += ".";
      gps_lat += gps_cal_l;
      gps_lat += GNSS_data[28] ;

      gps_cal_s = gps_data.substring(33, 42);
      gps_cal_f = gps_cal_s.toFloat();
      gps_cal_f /= 60.0f;
      gps_cal_l = gps_cal_f * 10000000;

      gps_lon += GNSS_data[30] ;
      gps_lon += GNSS_data[31] ;
      gps_lon += GNSS_data[32] ;
      gps_lon += ".";
      gps_lon += gps_cal_l;
      gps_lon += GNSS_data[43] ;

      gps_alt += GNSS_data[54];
      gps_alt += GNSS_data[55];
      gps_alt += GNSS_data[56];
      // gps_alt += GNSS_data[57];
      // gps_alt += GNSS_data[58];

      Serial.println(F("Stop GPS"));
      gps.Stop();
      gps.DisableNMEA();

      // cache GPS Information
      // EEPROMStructure gpsValue = { gps_lat.toDouble(), gps_lon.toDouble() };
      // eeAddress += sizeof(eepromFloatInitializedByte);
      // EEPROM.put(eeAddress, gpsValue);
      // Serial.println("update GPS cache...");
      // Serial.print(gpsValue.lat);
      // Serial.print(F("  "));
      // Serial.print(gpsValue.lng);
      // Serial.print(F("  "));
      // Serial.println(gps_alt);
      delay(1000);
    }
  }
  else {
      // NO GPS LINK: LOAD LAT, LNG from EEPROM
      uint32_t eeAddress = sizeof(eepromFloatInitializedByte);
      EEPROMStructure eepromCached;
      EEPROM.get(0+eeAddress, eepromCached);

      // gps_lat = String(eepromCached.lat);
      // gps_lon = String(eepromCached.lng);
      // localSleepTime = eepromCached.sleepTimeS;

      Serial.print("Read EEPROM : ");
      Serial.print(eepromCached.lat);
      Serial.print("  ");
      Serial.print(eepromCached.lng);
      Serial.print("  ");
      Serial.println(localSleepTime);
  }

  Serial.println(millis() / 1000);

}

bool open_tcp() {
  Serial.println();
  bool ret = tcp.Open(TCP_SERVER_ENDPOINT, TCP_SERVER_PORT);
  return ret;
}

bool dirty = false;
static uint32_t nextTick;

//////////////////////////////mainLOOP////////////////////////////////
void loop() {
  float mq4_co = 0.0, mq9_ch4 = 0.0;

  //  if (dirty) {
  if (1) {
    _temp = bme.readTemperature();
    _humid = bme.readHumidity();
    _press = bme.readPressure() / 100.0F;

    readDistance();

    pinMode(A0, INPUT);
    _batt = analogRead(A0);

    Serial.println("=== BME ===");
    Serial.print("T = ");
    Serial.print(_temp);
    Serial.print(" H = ");
    Serial.print(_humid);
    Serial.print(" P = ");
    Serial.print(_press);
    Serial.print(" B = ");
    Serial.print(_batt);
    Serial.print(" D = ");
    Serial.println(_volume);
    Serial.println(millis() / 1000);

    digitalWrite(LED, HIGH);
#if DEBUG_SERIAL
    Serial.print(F("print : "));
#endif
    // _volume++;
    // publish("/" APPID "/gearname/" BINID "/data1", buffer, false);
    String data1 = String (BINID ":");
    String data_s = String(_volume) + "," + String(_lidStatus) + "," + String(_temp) + ","
                    + String(_humid) + "," + String(_flameStatus);
    data1 += data_s;
#if DEBUG_SERIAL
    Serial.println(data1);
#endif
    // DATA2 Preparation
    String data2 = String (BINID ":");
    data_s = String(_pitch) + "," + String(_roll) + "," + String(_press) + "," + String(_batt);
    data2 += data_s;
#if DEBUG_SERIAL
    Serial.println(data2);
#endif
    // DATA3 Preparation
    String data3 = String (BINID ":");
    data_s = String(_soundStatus) + "," + String(mq4_co) + "," +
             String(mq9_ch4) + "," + String(_light) + "," + String(localSleepTime) + "," + String(millis() / 1000.00) + "," +
             String(_methane) + "," +
             String(_carbon);
    data3 += data_s;
#if DEBUG_SERIAL
    Serial.println(data3);
#endif
    // DATA4 Preparation
    String data4 = String (BINID ":");
    data_s = gps_lat + "," + gps_lon + "," + gps_alt;
    data4 += data_s;
    Serial.println(data4);

    bool tcpOpenFailed = false;
    int count_down = 100;
    while ( !open_tcp() && count_down ) {
#if DEBUG_SERIAL
      Serial.println("retry open tcp...");
#endif
      count_down--;
      delay(1000);
    }
    if (count_down == 0) {
      tcpOpenFailed = true;
#if DEBUG_SERIAL
      Serial.println("TCP Open failed...");
#endif
    }
    else {
      count_down = 100;
      while ( !tcp.StartSend() && count_down ) {
#if DEBUG_SERIAL
        Serial.println("retry send");
#endif
        count_down--;
        delay(500);
      }
      if (count_down) {
        tcp.println(data1);
        tcp.println(data2);
        tcp.println(data3);
        tcp.print(data4);
        tcp.StopSend();

        Serial.println(millis() / 1000);
      }
      count_down = 30;
      while ( !tcp.Close() && count_down ) {
        count_down--;
        delay(500);
#if DEBUG_SERIAL
        Serial.println("close tcp");
#endif
      }
    }

    // writeSleep to STM
    Serial2.write(stmTime);
    delay(1000);
    Serial2.write(stmTime);
    delay(1000);
    Serial2.write(stmTime);
    delay(1000);

    dirty = false;
    Serial.println(F("Sent..."));
  }

  Serial.println(millis() / 1000);
  Serial.println(F("gsm PowerOff zzZ"));
  // Serial.print("sleep for");
  // Serial.print(eepromCached.sleepTimeS);

  gsm.PowerOff();
  //  sleep.pwrSaveMode();
  sleepCtrl.pwrDownMode();
  // STM Sleep for n seconds
  Serial.println(millis());
  // sleep.sleepDelay(sleepTime); // 300000 = 5 minute
  Serial.println(millis());
  // Arduino Reset
  asm volatile ("  jmp 0");

}

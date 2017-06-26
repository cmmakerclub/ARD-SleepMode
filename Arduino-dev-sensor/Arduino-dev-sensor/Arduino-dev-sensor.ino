#include <Arduino.h>
#include <SPI.h>
#include "TEE_UC20.h"
#include "internet.h"
#include "CMMC_Interval.hpp"
#include "tcp.h"
#include <EEPROM.h>
#include <Wire.h>
#include <Sleep_n0m1.h>
#include "File.h"
#include "http.h"
#include <ArduinoJson.h>


uint8_t LED = 13;
#define MODE_PIN A4

#include "./sensors.hpp"
#include "gps.hpp"

struct EEPROMStructure {
  double lat;
  double lng;
  uint32_t sleepTimeS;
};

HTTP http;
Sleep sleepCtrl;
float eepromFloatInitializedByte = 0.000f;
EEPROMStructure globalCachedEEPROM;
int eeAddress = 0;

extern bool gotGPSLocation;
// bool ret = tcp.Open("sock.traffy.xyz","Connecting to... ");
//  bool ret = tcp.Open("api.traffy.xyz", "10777");
String TCP_SERVER_ENDPOINT = "128.199.143.200";
String TCP_SERVER_PORT     = "10777";

INTERNET net;
TCP tcp;
UC_FILE file;

long globalSleepTimeFromNetpieInMemory = 10;

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

// uint8_t stmSleepTimeS = 10;

#if DEBUG_SERIAL
void debug(String data) {
  // Serial.println(data);
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

    // LOAD EEPROM to global cache
    Serial.println("Initialized EEPROM");
    Serial.println("LOAD EEPROM to globalCachedEEPROM");
    EEPROM.get(sizeof(float), globalCachedEEPROM);
    printEEPROMInformation();
    delay(1111);
  }
  else /* load EEPROM */ {
    // eeAddress = sizeof(float);
    Serial.println("========================");
    Serial.println("LOADING CACHED IN EEPROM");
    Serial.println("========================");
    EEPROM.get(0+sizeof(float), globalCachedEEPROM);
    printEEPROMInformation();
    delay(1111);
  }
}


String netpieJsonString;
long getSleepTimeFromNetpie() {
    Serial.println(F("Send HTTP GET"));
    http.url("http://api.netpie.io/topic/SmartTrash/time?retain&auth=YGO1C5bATVNctTE:wN7khNDXgadngRN5WxMGMc7z0");
    Serial.println(http.get());
    // Serial.println(F("Clear data in RAM"));
    file.Delete(RAM,"*");
    Serial.println(F("Save HTTP Response To RAM"));
    http.SaveResponseToMemory(RAM,"netpie.json");
    // Serial.println(F("Read data in RAM"));

    // clear String
    netpieJsonString = "";
    read_file(RAM, "netpie.json");
    // Serial.println("READ FILE JSON");
    Serial.println(netpieJsonString);
    DynamicJsonBuffer jsonBuffer;
    JsonArray& root = jsonBuffer.parseArray(netpieJsonString.c_str());

    // Test if parsing succeeds.
    if (!root.success()) {
      Serial.println("parseObject() failed");
      return;
    }
    else {
      // Serial.println("parsed OK.");
      JsonObject& netpieJsonObject = root[0];
      // Serial.print("TIME PAYLOAD: ");
      const char* payload = netpieJsonObject["payload"];
      const char* topic = netpieJsonObject["topic"];
      const char* lastUpdated = netpieJsonObject["lastUpdated"];
      long payloadInt = String(payload).toInt();
      // Serial.print("payload: ");
      // Serial.println(payload);
      //
      Serial.print("payloadInt: ");
      Serial.println(payloadInt);
      //
      // Serial.print("topic: ");
      // Serial.println(topic);
      //
      // Serial.print("lastUpdated: ");
      // Serial.println(lastUpdated);
      return payloadInt;
    }
  };

//////////////////////////////mainSETUP////////////////////////////////
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

  Serial.println(F("Program Start."));
  Serial.print(F("freeMemory()="));
  Serial.println(freeMemory());
  pinMode(LED, OUTPUT);
  pinMode(MODE_PIN, INPUT_PULLUP);

  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);

  setEEProm();
  bme.begin();  // bme sensor begin

  // Just blink when program started
  int z = 0;
  while (z < 5) {
    digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                // wait for a second
    digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
    delay(10);
    z++;
  }

// GPIO CONFIGURATION
//   if (digitalRead(MODE_PIN) == HIGH) {
//     localSleepTime = 60;
// #if DEBUG_SERIAL
//     Serial.println("HIGH");
// #endif
//   } else {
//     // localSleepTime 3 minute 3 * 60 = 180
//     localSleepTime = 180 + 1000;
// #if DEBUG_SERIAL
//     Serial.println("LOW");
// #endif
//   }

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
  _rssi= gsm.SignalQuality();
  Serial.println(gsm.SignalQuality());

  Serial.println(F("Disconnect net"));
  bool netDisConnectStatus = net.DisConnect();
  Serial.print("netDisconnect Status = ");
  Serial.println(netDisConnectStatus);
  if (netDisConnectStatus) {
    Serial.println("NET DISCONNECTED OK.");
  }
  else {
    Serial.println("NET DISCONNECTED FAILED.");
  }
  net.Configure(APN, USER, PASS);
  bool netConnectStatus = net.Connect();
  Serial.print("netConnectStatus = ");
  Serial.println(netConnectStatus);
  if (netConnectStatus == 0) {
    Serial.println("net.Connect failed.");
    // asm volatile ("  jmp 0");
  }
  else {
    Serial.println(F("NET Connected"));
  }

  Serial.println(F("Show My IP"));
  Serial.println(net.GetIP());


  globalSleepTimeFromNetpieInMemory = getSleepTimeFromNetpie();
  Serial.print("SLEEP TIME [NETPIE] = ");
  Serial.println(globalSleepTimeFromNetpieInMemory);

  //////////////////////////////GPS//////////////////////////////

  startGPSService();

  Serial.println(millis() / 1000);
  http.begin(1);

}

String globalData0Version;
String globalData1;
String globalData2;
String globalData3;
String globalData4GPS;
String globalData5;

void builDataStringForTCPSocket() {
    float mq4_co = 0.0, mq9_ch4 = 0.0;
    String data_s;
    globalData0Version = String (BINID ":2,2,2,2,2");
    {
      globalData1 = String (BINID ":");
     data_s = String(_volume) + "," + String(_lidStatus) + "," + String(_temp) + ","
                      + String(_humid) + "," + String(_flameStatus);
      globalData1 += data_s;
      #if DEBUG_SERIAL
          Serial.println(globalData1);
      #endif
      // DATA2 Preparation
      globalData2 = String (BINID ":");
      String _battery_percent = "0";
      data_s = String(_pitch) + "," + String(_roll) + "," + String(_press) + "," + String(_battery_percent);
      globalData2 += data_s;
      #if DEBUG_SERIAL
          Serial.println(globalData2);
      #endif
    }
    { // DATA3 Preparation
      globalData3 = String (BINID ":");
      data_s = String(_soundStatus) + "," + String(mq4_co) + "," +
               String(mq9_ch4) + "," + String(_light) + "," + String(globalSleepTimeFromNetpieInMemory*60) + "," + String(millis() / 1000.00) + "," +
               String(_methane) + "," +
               String(_carbon);
      globalData3 += data_s;
      #if DEBUG_SERIAL
          Serial.println(globalData3);
      #endif
  }
  {
      // DATA4 Preparation
      globalData4GPS = String (BINID ":");
      data_s = gps_lat + "," + gps_lon + "," + gps_alt;
      globalData4GPS += data_s;
      Serial.println(globalData4GPS);

      // DATA5 Preparation
      globalData5 = String(BINID ":") + _rssi +"," + _batt;
      Serial.println(globalData5);
  }
}
bool open_tcp() {
  Serial.println("===========");
  Serial.println("open_tcp");
  Serial.print("Connecting to... ");
  Serial.print(TCP_SERVER_ENDPOINT);
  Serial.print(":");
  Serial.println(TCP_SERVER_PORT);
  Serial.println("===========");
  bool ret = tcp.Open(TCP_SERVER_ENDPOINT, TCP_SERVER_PORT);
  Serial.print("RESULT = ");
  Serial.println(ret);
  delay(50);
  return ret;
}



bool writeDataStringToTCPSocket() {
        /*
      data0,version
          91:2,2,2,2,2
      data1:dist,lid,temp,humid,flame
          91:61.00,0,27.56,52.87,0
      data2:pitch,roll,pressure,batt
          91:0.00,0.00,969,621.00
       data3:sound,mq4,mq9,lux,sleep,millis(),ch4,co
          91:0.00,0.00,0.00,25,10,60.50,0,0
       data4:lat,lng,alt
          91:18.7828670N,098.9788563E,268
        */
        tcp.println(globalData0Version);
        tcp.println(globalData1);
        tcp.println(globalData2);
        tcp.println(globalData3);
        tcp.println(globalData4GPS);
        tcp.print(globalData5);
        tcp.StopSend();
}

void sendSleepTimeInSecondToSTM32InS(uint8_t stmSleepTimeS) {
  // writeSleep to STM
  Serial.print("Send stemSleepTimeToSTM => ");
  Serial.println(stmSleepTimeS);

  Serial2.write(stmSleepTimeS);
  delay(1000);
  Serial2.write(stmSleepTimeS);
  delay(1000);
  Serial2.write(stmSleepTimeS);
  delay(1000);
  while(Serial2.available()) {
    Serial.println(Serial2.read());
  }
  Serial.println(F("Sent..."));
}

void printEEPROMInformation() {
  Serial.println("====================");
  Serial.println("  CACHED EEPROM  ");
  Serial.println("====================");
  Serial.print("globalCachedEEPROM.lat= ");
  Serial.println(globalCachedEEPROM.lat);
  Serial.print("globalCachedEEPROM.lng= ");
  Serial.println(globalCachedEEPROM.lng);
  Serial.print("globalCachedEEPROM.sleepTimeS = ");
  Serial.println(globalCachedEEPROM.sleepTimeS);
  Serial.println("====================");
  Serial.println("====================");
}

void sleepArduino(uint32_t sleepTimeInMs) {
  Serial.println(F("gsm PowerOff zzZ"));
  // Serial.print("sleep for");
  gsm.PowerOff();
  //  sleep.pwrSaveMode();
  sleepCtrl.pwrDownMode();
  // STM Sleep for n seconds
  Serial.print("Being sleep for ..");
  Serial.println(sleepTimeInMs);
  Serial.println(millis());
  sleepCtrl.sleepDelay(sleepTimeInMs); // in MS
  // Arduino Reset
  asm volatile ("  jmp 0");
}

void sendDataOverTCPSocket() {
  if (open_tcp()) {
    if (tcp.StartSend()) {
      writeDataStringToTCPSocket();
    }
    while(!tcp.Close()){
      Serial.println("Closing tcp...");
      delay(10);
    }
    delay(1000);
  }
}

//////////////////////////////mainLOOP////////////////////////////////
void loop() {
  // should be realtime mode
  // Serial.print("globalSleepTimeFromNetpieInMemory")
  while (globalSleepTimeFromNetpieInMemory == 0) {
    readAllSensors();
    builDataStringForTCPSocket();
    long freshSleepTimeFromNetpie = getSleepTimeFromNetpie();
    if (globalSleepTimeFromNetpieInMemory != freshSleepTimeFromNetpie) {
      Serial.println("GOT NEW SLEEP TIME");
      globalSleepTimeFromNetpieInMemory = freshSleepTimeFromNetpie;
    }
    sendDataOverTCPSocket();
    if (gotGPSLocation == false) {
      startGPSService();
    }
    delay(10*1000);
  }

  readAllSensors();
  builDataStringForTCPSocket();
  sendDataOverTCPSocket();

  Serial.println("Being sleep...");
  // TODO: must multiply by 60
  sendSleepTimeInSecondToSTM32InS(globalSleepTimeFromNetpieInMemory/60);
  sleepArduino(globalSleepTimeFromNetpieInMemory * 1000L);
}

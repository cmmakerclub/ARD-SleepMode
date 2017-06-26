#include <Arduino.h>
#include "gnss.h"

GNSS gps;
float GPS_SEARCH_TIMEOUT_S = 10;
extern uint8_t LED;

// volatile char GNSS_data[58] = "";
char GNSS_data[58] = "";
String gps_data = "";
String gps_lat = "";
String gps_lon = "";
String gps_alt = "";
uint8_t gpsCounter = 0;

void convertGPSRawDataToLatLng() {
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
    delay(1000);
  }
}

void startGPSService() {
  gps.Start();
  gps.EnableNMEA();
  gps_data = gps.GetNMEA("GGA");
  gpsCounter = 0;
  // assume got gps unless searching timeout.
  bool gps_linked = true;
  uint32_t gpsTimeoutNextTick = millis() + GPS_SEARCH_TIMEOUT_S*1000L;
  Serial.print("GPS TIMEOUT NEXTICK = ");
  Serial.println(gpsTimeoutNextTick);
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
    if (gpsCounter%10 == 0) {
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
  delay(1111);
  if (gps_linked) {
    Serial.println("FOUND GPS....");
    Serial.println("COVERTING to Coordinate....");
    convertGPSRawDataToLatLng();
    /*
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
    */
  }
  else {
    Serial.println("GPS TIMEOUT..");
  }
}

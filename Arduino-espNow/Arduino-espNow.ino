/*
  Serial Event example

  When new serial data arrives, this sketch adds it to a String.
  When a newline is received, the loop prints the string and
  clears it.

  A good test for this is to try it with a GPS receiver
  that sends out NMEA 0183 sentences.

  Created 9 May 2011
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/SerialEvent

*/

///////////////////////////////////////////////////////////////////////////////
struct NODEStructure {
  uint8_t mac[6];
  uint8_t buff[128];
  uint8_t len;
};

#define tmpSize  128
uint8_t tmp[tmpSize];
uint8_t tmp_index;

NODEStructure Node1;
NODEStructure Node2;
NODEStructure Node3;
NODEStructure Node4;


void serialEvent2() {
  while (Serial2.available()) {
    tmp[tmp_index] = Serial2.read();
    Serial.print(tmp[tmp_index], HEX);
    Serial.print(" ");
    tmp_index++;
    if (tmp_index >= tmpSize)tmp_index = 0;
  }
  MassageAnalysis();
}

uint8_t MassageAnalysis() {
  uint8_t mac[6] = {0};
  for (int i = tmp_index; i >= 2; i--) {
    if ((tmp[i] == 0x0A) && (tmp[i - 1] == 0x0D)) {
      tmp_index = 0;
      for (int j = 0; j < i; j++) {
        if ((tmp[j] == 0xFC) && (tmp[j + 1] == 0xFD)) {
          uint8_t sum = 0;
          for (uint8_t k = 0; k <=  (i - j - 3); k++) {
            sum ^= tmp[j + k];
          }
          if (sum == tmp[i - j - 2]) {
            memcpy(&mac, &tmp[j + 8], 6);
            MassageSave(mac, &tmp[j], i - j + 1);
            return 1;
          } else {
            return 0;
          }
        }
      }
    }
  }
  return 0;
}

uint8_t CheckMac(uint8_t* mac1, uint8_t* mac2) {
  uint8_t val = 0x01;
  for (int i = 0; i < 6; i++) {
    if (mac1[i] != mac2[i]) {
      return 0;
    }
  }
  return 1;
}

void MassageSave(uint8_t* tmp, uint8_t* data, uint8_t len) {
  static int index;
  if (CheckMac(tmp, Node1.mac)) {
    memset(Node1.buff, 0, sizeof(Node1.buff));
    memcpy(&Node1.buff, data, len);
    Node1.len = len;
  } else if (CheckMac(tmp, Node2.mac)) {
    memset(Node2.buff, 0, sizeof(Node2.buff));
    memcpy(&Node2.buff, data, len);
    Node2.len = len;
  } else if (CheckMac(tmp, Node3.mac)) {
    memset(Node3.buff, 0, sizeof(Node3.buff));
    memcpy(&Node3.buff, data, len);
    Node3.len = len;
  } else if (CheckMac(tmp, Node4.mac)) {
    memset(Node4.buff, 0, sizeof(Node4.buff));
    memcpy(&Node4.buff, data, len);
    Node4.len = len;
  } else {
    switch (index) {
      case 0:
        memcpy(&Node1.mac, tmp, 6);
        memset(Node1.buff, 0, sizeof(Node1.buff));
        memcpy(&Node1.buff, data, len);
        Node1.len = len;
        break;
      case 1:
        memcpy(&Node2.mac, tmp, 6);
        memset(Node2.buff, 0, sizeof(Node2.buff));
        memcpy(&Node2.buff, data, len);
        Node2.len = len;
        break;
      case 2:
        memcpy(&Node3.mac, tmp, 6);
        memset(Node3.buff, 0, sizeof(Node3.buff));
        memcpy(&Node3.buff, data, len);
        Node3.len = len;
        break;
      case 3:
        memcpy(&Node4.mac, tmp, 6);
        memset(Node4.buff, 0, sizeof(Node4.buff));
        memcpy(&Node4.buff, data, len);
        Node4.len = len;
        break;
    }
    index++;
    if (index == 4) index = 0;
  }
}


///////////////////////////////////////////////////////////////////////////////

void setup() {
  // initialize serial:
  Serial.begin(115200);
  Serial2.begin(9600);
  // reserve 200 bytes for the inputString:

}

void loop() {
  // print the string when a newline arrives:
  delay(1000);
  Serial.print("\n");
  //  MassageAnalysis();
  for (int i = 0; i < tmp_index; i++) {
    Serial.print(tmp[i], HEX);
    Serial.print(" ");
  }

  Serial.print("\n");
  Serial.print("\n");

  Serial.print("MAC1 ");
  for (int i = 0; i < 6; i++) {
    Serial.print(Node1.mac[i], HEX);
    Serial.print(" ");
  }
  Serial.print("\t");  Serial.print("\t");
  for (int i = 0; i < Node1.len; i++) {
    Serial.print(Node1.buff[i], HEX);
    Serial.print(" ");
  }

  Serial.print("\n");
  Serial.print("MAC2 ");
  for (int i = 0; i < 6; i++) {
    Serial.print(Node2.mac[i], HEX);
    Serial.print(" ");
  }
  Serial.print("\t");  Serial.print("\t");
  for (int i = 0; i < Node2.len; i++) {
    Serial.print(Node2.buff[i], HEX);
    Serial.print(" ");
  }

  Serial.print("\n");
  Serial.print("MAC3 ");
  for (int i = 0; i < 6; i++) {
    Serial.print(Node3.mac[i], HEX);
    Serial.print(" ");
  }
  Serial.print("\t");  Serial.print("\t");
  for (int i = 0; i < Node3.len; i++) {
    Serial.print(Node3.buff[i], HEX);
    Serial.print(" ");
  }

  Serial.print("\n");
  Serial.print("MAC4 ");
  for (int i = 0; i < 6; i++) {
    Serial.print(Node4.mac[i], HEX);
    Serial.print(" ");
  }
  Serial.print("\t");  Serial.print("\t");
  for (int i = 0; i < Node4.len; i++) {
    Serial.print(Node4.buff[i], HEX);
    Serial.print(" ");
  }

  Serial.print("\n");
}

/*
  SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/



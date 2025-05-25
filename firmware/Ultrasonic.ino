#include <ACAN2515.h>
#include <SPI.h>
#define lowSelect 3
#define midSelect 4
#define highSelect 5
static const byte MCP2515_CS  = 17 ;
static const byte MCP2515_INT = 15 ;

ACAN2515 can (MCP2515_CS, SPI, MCP2515_INT) ; // You can use SPI2, SPI3, if provided by your microcontroller

const uint32_t QUARTZ_FREQUENCY = 16 * 1000 * 1000 ; // 16 MHz

int currentSensor = 1;
char dist[9];
int bits;

// For averaging a 32 byte sequence
// double getData(const char *array) {
//   double data = 0;
//   int count = 0;
//   for (int i = 0; array[i] != '\0'; i++) {
//     if (array[i] == 'R') {
//       count++;
//       data += (array[i+1]-'0')*100 + (array[i+2]-'0')*10 + (array[i+3]-'0');
//     }
//   }
//   return data/count;
// }

int getData(const char *array) {
  int data = 0;
  if (array[0] == 'R') {
    data += (array[1]-'0')*100 + (array[2]-'0')*10 + (array[3]-'0');
  }
  else if (array[1] == 'R') {
    data += (array[2]-'0')*100 + (array[3]-'0')*10 + (array[4]-'0');
  }
  else if (array[2] == 'R') {
    data += (array[3]-'0')*100 + (array[4]-'0')*10 + (array[5]-'0');
  }
  else if (array[3] == 'R') {
    data += (array[4]-'0')*100 + (array[5]-'0')*10 + (array[6]-'0');
  }
  else if (array[4] == 'R') {
    data += (array[5]-'0')*100 + (array[6]-'0')*10 + (array[7]-'0');
  }
  else if (array[5] == 'R') {
    data += (array[6]-'0')*100 + (array[7]-'0')*10 + (array[0]-'0');
  }
  else if (array[6] == 'R') {
    data += (array[7]-'0')*100 + (array[0]-'0')*10 + (array[1]-'0');
  }
  else if (array[7] == 'R') {
    data += (array[0]-'0')*100 + (array[1]-'0')*10 + (array[2]-'0');
  }
  else if (array[0] == '0') {
    data += (array[1]-'0')*10 + (array[2]-'0');
  }
  return data;
}

int changeSensor() {
    if (digitalRead(lowSelect) == 0 && digitalRead(midSelect) == 0) {
        digitalWrite(lowSelect, HIGH); return 2;
      }
    else if (digitalRead(lowSelect) == 1 && digitalRead(midSelect) == 0) {
         digitalWrite(midSelect, HIGH); digitalWrite(lowSelect, LOW); return 3;
      }
    else if (digitalRead(lowSelect) == 0 && digitalRead(midSelect) == 1) {
        digitalWrite(lowSelect, HIGH); return 4;
      }
    else if (digitalRead(lowSelect) == 1 && digitalRead(midSelect) == 1) {
        digitalWrite(lowSelect, LOW); digitalWrite(midSelect, LOW); return 1;
      }
    else {return 0;} 
}

void setup() {
  SPI.begin();
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 125 * 1000);
  settings.mRequestedMode = ACAN2515Settings::LoopBackMode;
  const uint16_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
  pinMode(lowSelect, OUTPUT);
  digitalWrite(lowSelect, LOW);
  pinMode(midSelect, OUTPUT);
  digitalWrite(midSelect, LOW);
  pinMode(highSelect, OUTPUT);
  digitalWrite(highSelect, LOW);
  Serial1.setRX(1);
  Serial1.setInvertRX(true);
  Serial1.begin(9600);
  Serial.begin(9600);
}

void loop() {

  if (Serial1.available() >= 7) {
    Serial1.readBytes(dist, 8);
    bits = getData(dist);
    currentSensor = changeSensor();
    Serial.println(bits);
    Serial.println(currentSensor);
    CANMessage message;
    message.id = 20;
    message.data[0] = currentSensor;
    message.data[1] = bits;
    bool isCANSent = can.tryToSend(message);
    Serial.println(isCANSent);
  }

  // if (Serial1.available() >= 31) {
  //   Serial1.readBytes(dist, 32);
  //   //dist[31] = '\0';
  //   Serial.println(dist);
  //   Serial.println(getData(dist));
  //   changeSensor();
  // }

}

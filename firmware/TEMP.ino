// The read function indicates that the switching is occuring properly. First, fix the cable and consider adding a delay

#define lowSelect 3
#define midSelect 4
#define highSelect 5

int data = 0;
int senseID = 0;
char dist[33];
char bits;

double getData(const char *array) {
  double data = 0;
  int count = 0;
  for (int i = 0; array[i] != '\0'; i++) {
    if (array[i] == 'R') {
      count++;
      data += (array[i+1]-'0')*100 + (array[i+2]-'0')*10 + (array[i+3]-'0');
    }
  }
  return data/count;
}

void changeSensor() {
    if (digitalRead(lowSelect) == 0 && digitalRead(midSelect) == 0) {
        digitalWrite(lowSelect, HIGH); Serial.println("Sensor 2");
      }
    else if (digitalRead(lowSelect) == 1 && digitalRead(midSelect) == 0) {
         digitalWrite(midSelect, HIGH); digitalWrite(lowSelect, LOW); Serial.println("Sensor 3");
      }
    else if (digitalRead(lowSelect) == 0 && digitalRead(midSelect) == 1) {
        digitalWrite(lowSelect, HIGH); Serial.println("Sensor 4");
      }
    else if (digitalRead(lowSelect) == 1 && digitalRead(midSelect) == 1) {
        digitalWrite(lowSelect, LOW); digitalWrite(midSelect, LOW); Serial.println("Sensor 1");
      } 
}

void setup() {
  //ACAN2515Settings settings (QUARTZ_FREQUENCY, 125 * 1000);
 // settings.mRequestedMode = ACAN2515Settings::LoopBackMode;
  //const uint16_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
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
    //dist[31] = '\0';
    Serial.println(dist);
    Serial.println(getData(dist));
    changeSensor();
  }

  // if (Serial1.available() >= 31) {
  //   Serial1.readBytes(dist, 32);
  //   //dist[31] = '\0';
  //   Serial.println(dist);
  //   Serial.println(getData(dist));
  //   changeSensor();
  // }

}
#define PIN1 6
#define PIN2 5

#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {
  Serial.begin(9600);
  Serial.println("Start");

  mySerial.begin(9600);
  mySerial.println("START");

  pinMode(PIN1, OUTPUT);
  pinMode(PIN2, OUTPUT);

  digitalWrite(PIN1, LOW);
}

void loop() // run over and over
{
  if (mySerial.available()) {
    char c = mySerial.read();
    Serial.write(c);
    analogWrite(PIN2, (int)c);
  }
  if (Serial.available()) {
    mySerial.write(Serial.read());
  }
}

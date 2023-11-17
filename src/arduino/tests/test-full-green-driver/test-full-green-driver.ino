#define DIR1 2
#define PWM1 3

#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {
  Serial.begin(9600);
  Serial.println("Start");

  mySerial.begin(9600);
  mySerial.println("START");

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);

  digitalWrite(DIR1, LOW);
}

void loop() // run over and over
{
  if (mySerial.available()) {
    char c = mySerial.read();
    Serial.write(c);
    analogWrite(PWM1, (int)c);
  }
  if (Serial.available()) {
    mySerial.write(Serial.read());
  }
}

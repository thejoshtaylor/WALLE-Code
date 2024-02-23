/*Sample code for the Robot Power Wasp. 

 This ESC is controlled using RC signals, with pulses
  ranging from 1000 - 2000 microseconds.

  The main loop of this program holds the actuator still for 1 second, extends for 2 seconds,
  stops for 1 second, retracts for 2 seconds, and repeats.

  Modified by Progressive Automations, using the original example code "Sweep" from the 
  Arduino example libraries. 
  
  Hardware:
  - 1 Wasp Controller
  - Arduino Uno
  
  Wiring:
  Control side:
  - Connect the red/black to +5v and GND
  - Connect the yellow wire to your signal pin on the Arduino (in this example, pin 9)
  Power Side:
  - Connect the +/- of the motors power supply to the +/- connections on the Wasp
  - Connect the +/- of the actuator to the remaining two connections

  This example code is in the public domain.
*/ 

#define LINEAR_ACTUATOR_PIN 33
#define POT_PIN 41

#include <AutoPID.h>
#include <Servo.h>  
Servo myservo;  // create servo object to control a servo 
                // twelve servo objects can be created on most boards
 
int pos = 0;    // variable to store the servo position

float minLen = 18.5;
float maxLen = 0.5;

#define OUTPUT_MIN -500
#define OUTPUT_MAX 500
#define KP 600
#define KI 0
#define KD 10

double position, target, moveSpeed;

AutoPID myPID(&position, &target, &moveSpeed, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

 
void setup()
{ 
  Serial.begin(9600);

  myservo.attach(SERVO_PIN);  // attaches the servo on the pin to the servo object 

  pinMode(POT_PIN, INPUT);

  myPID.setTimeStep(1);
  // myPID.setBangBang(1.5);
  target = 12;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float getLength()
{
  int potValue = analogRead(POT_PIN);
  float len = mapfloat(potValue, 0, 1023, 0.0, 18.0);
  if (len < minLen)
  {
    minLen = len;
  }
  if (len > maxLen)
  {
    maxLen = len;
  }
  return len;
}
 
void loop() 
{
  position = getLength();
  // target = 12.0; // 9 inches
  myPID.run(); //call every loop, updates automatically at certain time interval
  // analogWrite(OUTPUT_PIN, outputVal);

  if (fabs(moveSpeed) < 10)
    moveSpeed = 0;

  myservo.writeMicroseconds(-moveSpeed + 1500); // stop signal

  if (millis() % 10000 == 0)
  {
    if (target == 12)
      target = 3;
    else if (target == 3)
      target = 9;
    else if (target == 9)
      target = 12;
  }

  // unsigned long start = millis();
  if (millis() % 100 == 0)
  {
    Serial.print("Length: ");
    Serial.print(position);
    Serial.print(" -> [");
    Serial.print(minLen);
    Serial.print(", ");
    Serial.print(maxLen);
    Serial.print("] -> ");
    Serial.println(moveSpeed + 1500);
  //   delay(10);
  }

  // myservo.writeMicroseconds(2000); // full speed forwards signal

  // start = millis();
  // while (millis() - start < 10000)
  // {
  //   Serial.print("Length: ");
  //   Serial.print(getLength());
  //   Serial.print(" -> [");
  //   Serial.print(minLen);
  //   Serial.print(", ");
  //   Serial.print(maxLen);
  //   Serial.println("]");
  //   delay(10);
  // }
  
  // myservo.writeMicroseconds(1500); // stop signal

  // start = millis();
  // while (millis() - start < 1000)
  // {
  //   Serial.print("Length: ");
  //   Serial.print(getLength());
  //   Serial.print(" -> [");
  //   Serial.print(minLen);
  //   Serial.print(", ");
  //   Serial.print(maxLen);
  //   Serial.println("]");
  //   delay(10);
  // }
  
  // myservo.writeMicroseconds(1000); // full speed reverse signal
  
  // start = millis();
  // while (millis() - start < 10000)
  // {
  //   Serial.print("Length: ");
  //   Serial.print(getLength());
  //   Serial.print(" -> [");
  //   Serial.print(minLen);
  //   Serial.print(", ");
  //   Serial.print(maxLen);
  //   Serial.println("]");
  //   delay(10);
  // }
}
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

#define SERVO_PIN 37

#include <Servo.h>  
Servo myservo;  // create servo object to control a servo 
                // twelve servo objects can be created on most boards
 
int pos = 0;    // variable to store the servo position

 
void setup()
{ 
  Serial.begin(9600);

  myservo.attach(SERVO_PIN);  // attaches the servo on the pin to the servo object
}

void loop() 
{
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
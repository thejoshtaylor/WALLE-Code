// X1 WALL-E 2024
// by Joshua Taylor
// 2024-01-27

// Designed for Teensy 4.1

// Pin definitions
#define PWM1 11
#define PWM2 12

#define LED 13

#define MAX_VALUE 100
#define MS_PER_CYCLE 10000

#define DELAY_PER_STEP MS_PER_CYCLE / (MAX_VALUE * 4)

#include <Servo.h>
Servo drive1;
Servo drive2;

void setup() {
  
  pinMode(LED, OUTPUT);

  drive1.attach(PWM1);
  drive1.writeMicroseconds(1500); // stop signal
  drive2.attach(PWM2);
  drive2.writeMicroseconds(1500); // stop signal
}

void loop() // run over and over
{
  // Wait 5 seconds
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }

  // Start motion
  
  digitalWrite(LED, HIGH);

  // Spin up
  for (int i = 0; i < MAX_VALUE; i++)
  {
    drive1.writeMicroseconds(1500 + i);
    drive2.writeMicroseconds(1500 + i);
    delay(DELAY_PER_STEP);
  }
  delay(5000);

  // Spin down
  for (int i = MAX_VALUE; i >= 0; i--)
  {
    drive1.writeMicroseconds(1500 + i);
    drive2.writeMicroseconds(1500 + i);
    delay(DELAY_PER_STEP);
  }
  
  // Start going backwards
  digitalWrite(LED, LOW);
  for (int i = 0; i >= -MAX_VALUE; i--)
  {
    drive1.writeMicroseconds(1500 + i);
    drive2.writeMicroseconds(1500 + i);
    delay(DELAY_PER_STEP);
  }
  delay(1000);

  // Go back to zero
  for (int i = -MAX_VALUE; i <= 0; i++)
  {
    drive1.writeMicroseconds(1500 + i);
    drive2.writeMicroseconds(1500 + i);
    delay(DELAY_PER_STEP);
  }
  delay(1000);
}

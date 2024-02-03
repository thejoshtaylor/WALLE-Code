#define PIN 33
#define LED 13

#define MAX_VALUE 500
#define MS_PER_CYCLE 20000

#define DELAY_PER_STEP MS_PER_CYCLE / (MAX_VALUE * 4)

#include <Servo.h>  
Servo servo;

void setup()
{
  pinMode(LED, OUTPUT);

  servo.attach(PIN);
  servo.writeMicroseconds(1500); // stop signal
}

void loop()
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
    servo.writeMicroseconds(1500 + i);
    delay(DELAY_PER_STEP);
  }
  delay(1000);

  // Spin down
  for (int i = MAX_VALUE; i >= 0; i--)
  {
    servo.writeMicroseconds(1500 + i);
    delay(DELAY_PER_STEP);
  }
  
  // Start going backwards
  digitalWrite(LED, LOW);
  for (int i = 0; i >= -MAX_VALUE; i--)
  {
    servo.writeMicroseconds(1500 + i);
    delay(DELAY_PER_STEP);
  }
  delay(1000);

  // Go back to zero
  for (int i = -MAX_VALUE; i <= 0; i++)
  {
    servo.writeMicroseconds(1500 + i);
    delay(DELAY_PER_STEP);
  }
  delay(1000);
}

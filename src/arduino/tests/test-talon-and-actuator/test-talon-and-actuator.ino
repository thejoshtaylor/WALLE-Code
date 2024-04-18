// X1 WALL-E 2024
// by Joshua Taylor
// 2024-02-16

// Designed for Teensy 4.1

#define LINEAR_ACTUATOR_PIN 28
#define POT_PIN 14

#include <AutoPID.h>
#include <Servo.h>
Servo actuatorServo;
Servo talonServo;

// Pin definitions
#define DIR1 20
#define PWM1 18
#define DIR2 21
#define PWM2 19

#define TALON 36

// All variables that we're going to receive from the RPi
short leftSpeed = 0;
short rightSpeed = 0;

short leftArmLength = 0;
short rightArmLength = 0;

short leftArmAngle = 0;
short rightArmAngle = 0;

short leftWingAngle = 0;
short rightWingAngle = 0;

short leftHandAngle = 0;
short rightHandAngle = 0;

bool bigFaceLatch = true;

// Introductory message
char intro[] = {0x55, 0x4f, 0x02, 0x39, 0x81, 0xc4};
char terminator[] = {0x0c, 0xf7, 0x13, 0x85, 0x3f, 0x5a};

// Message read status
char status = 0;

// Message type
char type = 0;

// Message data
char data[32];

// Checksum
char checksum = 0;

// Last message time
unsigned long lastMessage = 0;

int pos = 0; // variable to store the servo position

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

  Serial.begin(115200);
  Serial.println("Start");

  Serial1.begin(115200);

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);

  // Linear actuator
  actuatorServo.attach(LINEAR_ACTUATOR_PIN);

  pinMode(POT_PIN, INPUT);

  myPID.setTimeStep(50);
  // myPID.setBangBang(1.5);
  target = 0;

  // Shredder
  talonServo.attach(TALON);
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

void loop() // run over and over
{
  position = getLength();
  // target = 12.0; // 9 inches
  myPID.run();

  if (fabs(moveSpeed) < 10)
    moveSpeed = 0;

  actuatorServo.writeMicroseconds(-moveSpeed + 1500); // stop signal

  // Get message from rpi
  if (Serial1.available())
  {
    lastMessage = millis();
    // Determine what part of the message we're supposed to read
    // 0-5: Intro
    if (status < 6)
    {
      char in;
      // Wait for the first byte of the message
      if ((in = Serial1.read()) == intro[status])
        status++;
      else
      {
        Serial.print("BAD INTRO, ");
        Serial.print(in);
        Serial.print(" != ");
        Serial.println(intro[status]);
        status = 255;
      }
    }
    // 6: Type
    else if (status == 6)
    {
      type = Serial1.read();
      status++;
    }
    // 7-38: Data
    else if (status >= 7 && status < 39)
    {
      data[status - 7] = Serial1.read();
      status++;
    }
    // 39: Checksum
    else if (status == 39)
    {
      checksum = Serial1.read();

      // Checksum is correct
      // Calculate sum of all bytes in message
      char sum = 0;
      for (int i = 0; i < 32; i++)
        sum += data[i];

      // Checksum is incorrect
      if (sum != checksum)
      {
        Serial.println("BAD CHECKSUM");
        status = 255;
      }
      else
        status++;
    }
    // 40-45: Terminator
    else if (status >= 40 && status < 46)
    {
      // Wait for the first byte of the message
      if (Serial1.read() == terminator[status - 40])
        status++;
      else
      {
        Serial.println("BAD TERMINATOR");
        status = 255;
      }
    }

    // Complete message indicator
    if (status == 46)
    {
      // Handle the message
      if (type == 1)
      {
        // TURN OFF THE SHREDDER
        talonServo.write(1500);

        // Handle the message
        Serial.print("GOT MESSAGE\t");
        // Normal operation
        leftSpeed = (data[0] << 8) | data[1];
        Serial.print(leftSpeed);
        Serial.print("\t");
        rightSpeed = (data[2] << 8) | data[3];
        Serial.print(rightSpeed);
        Serial.print("\t");

        leftArmLength = (data[4] << 8) | data[5];

        target = mapfloat(leftArmLength, -32768, 32767, 0.0, 18.0);
        Serial.print(target);
        Serial.print("\t");
        Serial.println(position);

        rightArmLength = (data[6] << 8) | data[7];

        leftArmAngle = (data[8] << 8) | data[9];
        rightArmAngle = (data[10] << 8) | data[11];

        leftWingAngle = (data[12] << 8) | data[13];
        rightWingAngle = (data[14] << 8) | data[15];

        leftHandAngle = (data[16] << 8) | data[17];
        rightHandAngle = (data[18] << 8) | data[19];

        bigFaceLatch = data[20] == 1;
      }
      // Handle shredder message
      else if (type == 2)
      {
        Serial.print("GOT SHREDDER MESSAGE\t");

        // Check for the magic number
        int doubleCheck = (data[0] << 24) | (data[8] << 16) | (data[3] << 8) | data[19];
        if (doubleCheck == 0xdeadbeef)
        {
          int speed = (data[4] << 8) | data[5];

          // 0 means backwards, 1 means forwards
          int direction = data[6];

          // Validate message
          if (speed < -32768 || speed > 32767 || direction < 0 || direction > 1)
          {
            Serial.println("BAD SHREDDER MESSAGE (2)");
          }
          else
          {

            int signal = map(speed, -32768, 32767, 0, 100);

            signal = direction == 0 ? 1500 - signal : 1500 + signal;

            Serial.println(signal);
            talonServo.write(signal);
          }
        }
        else
        {
          Serial.println("BAD SHREDDER MESSAGE");
        }
      }

      // Reset status
      status = 0;
    }
  }

  // Check for timeout
  if (millis() - lastMessage > 1000 && status != 0)
  {
    Serial.println("TIMEOUT");
    status = 255;
  }

  // Bad message indicator
  if (status == 255)
  {
    // TURN OFF THE SHREDDER
    talonServo.write(1500);

    // Set all values to zero
    leftSpeed = 0;
    rightSpeed = 0;

    leftArmLength = -32768;
    rightArmLength = -32768;

    leftArmAngle = 0;
    rightArmAngle = 0;

    leftWingAngle = 0;
    rightWingAngle = 0;

    leftHandAngle = 0;
    rightHandAngle = 0;

    bigFaceLatch = true;

    // Reset status
    Serial.println("BAD MESSAGE");
    status = 0;
  }

  digitalWrite(DIR1, leftSpeed >= 0);
  digitalWrite(DIR2, rightSpeed >= 0);

  analogWrite(PWM1, map(abs(leftSpeed), 0, 32678, 0, 255));
  analogWrite(PWM2, map(abs(rightSpeed), 0, 32678, 0, 255));
}

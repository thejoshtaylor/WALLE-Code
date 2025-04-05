// X1 WALL-E 2024
// by Joshua Taylor
// 2024-05-31

// Designed for Teensy 4.1

#include <AutoPID.h>
#include <Servo.h>

// Pin definitions
#define E_STOP_PIN 2

#define L_FOREARM_ACTUATOR_PIN 28
#define L_FOREARM_POT_PIN 14

#define L_ELEV_ACTUATOR_PIN 29
#define L_ELEV_POT_PIN 15

#define R_FOREARM_ACTUATOR_PIN 30
#define R_FOREARM_POT_PIN 16

#define R_ELEV_ACTUATOR_PIN 31
#define R_ELEV_POT_PIN 17

#define R_WRISTBRO_PIN 33
#define L_WRISTBRO_PIN 32


// Tank drive
#define DIR1 20
#define PWM1 18
#define DIR2 21
#define PWM2 19

#define TALON 36

// Servo objects
Servo leftArmServo;
Servo leftElevServo;
Servo rightArmServo;
Servo rightElevServo;
Servo talonServo;
Servo leftwristbro;
Servo rightwristbro;

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

#define OUTPUT_MIN -500
#define OUTPUT_MAX 500
#define KP 600
#define KI 0
#define KD 10

double currentLeftLength, leftTarget, leftMoveSpeed;
double currentRightLength, rightTarget, rightMoveSpeed;

AutoPID leftPID(&currentLeftLength, &leftTarget, &leftMoveSpeed, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID rightPID(&currentRightLength, &rightTarget, &rightMoveSpeed, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

unsigned long lastHeartbeat = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("Start");

  Serial1.begin(115200);

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  pinMode(E_STOP_PIN, INPUT_PULLUP);

  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);

  // Linear actuator
  leftArmServo.attach(L_FOREARM_ACTUATOR_PIN);
  leftElevServo.attach(L_ELEV_ACTUATOR_PIN);
  rightArmServo.attach(R_FOREARM_ACTUATOR_PIN);
  rightElevServo.attach(R_ELEV_ACTUATOR_PIN);
  //leftwristbro.attach(L_WRISTBRO_PIN);
  //rightwristbro.attach(R_WRISTBRO_PIN);

  pinMode(L_FOREARM_POT_PIN, INPUT);
  pinMode(L_ELEV_POT_PIN, INPUT);
  pinMode(R_FOREARM_POT_PIN, INPUT);
  pinMode(R_ELEV_POT_PIN, INPUT);

  leftPID.setTimeStep(50);
  // leftPID.setBangBang(1.5);
  leftTarget = 0;

  rightPID.setTimeStep(50);
  // rightPID.setBangBang(1.5);
  rightTarget = 0;

  // Shredder
  talonServo.attach(TALON);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float getLinearLength(int pin, float extent)
{
  int potValue = analogRead(pin);
  float len = mapfloat(potValue, 0, 1023, 0.0, extent);
  return len;
}

void loop() // run over and over
{
  // Print heartbeat to serial every second
  if (millis() - lastHeartbeat > 1000)
  {
    Serial.println("Heartbeat");
    lastHeartbeat = millis();
  }

  currentLeftLength = getLinearLength(L_FOREARM_POT_PIN, 18.0);
  currentRightLength = getLinearLength(R_FOREARM_POT_PIN, 18.0);
  leftPID.run();
  rightPID.run();

  if (fabs(leftMoveSpeed) < 10)
    leftMoveSpeed = 0;

  if (fabs(rightMoveSpeed) < 10)
    rightMoveSpeed = 0;

  leftArmServo.writeMicroseconds(leftMoveSpeed + 1500); // stop signal
  rightArmServo.writeMicroseconds(rightMoveSpeed + 1500); // stop signal

  if (digitalRead(E_STOP_PIN) == HIGH)
  {
    Serial.println("E-STOP");
    status = 255;
  }
  // Get message from rpi
  else if (Serial1.available())
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
        // Normal operation
        leftSpeed = (data[0] << 8) | data[1];
        rightSpeed = (data[2] << 8) | data[3];

        leftArmLength = (data[4] << 8) | data[5];
        leftTarget = mapfloat(leftArmLength, -32768, 32767, 0.0, 18.0);

        rightArmLength = (data[6] << 8) | data[7];
        rightTarget = mapfloat(rightArmLength, -32768, 32767, 0.0, 18.0);

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
          short speed = (data[4] << 8) | data[5];
          Serial.print(speed);
          Serial.print("\t");

          // 0 means backwards, 1 means forwards
          int direction = data[6];
          Serial.print(direction);
          Serial.print("\t");

          // Validate message
          if (speed < -32768 || speed > 32767 || direction < 0 || direction > 1)
          {
            Serial.println("BAD SHREDDER MESSAGE (2)");
            talonServo.write(1500);
          }
          else
          {

            int signal = map(speed, -32768, 32767, 0, 500);

            signal = direction == 0 ? 1500 - signal : 1500 + signal;

            Serial.println(signal);
            talonServo.write(signal);
          }
        }
        else
        {
          Serial.println("BAD SHREDDER MESSAGE");
          talonServo.write(1500);
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

  int leftAngleSpeed = map(leftArmAngle, -32768, 32767, -500, 500);
  leftElevServo.writeMicroseconds(leftAngleSpeed + 1500);

  int rightAngleSpeed = map(rightArmAngle, -32768, 32767, -500, 500);
  rightElevServo.writeMicroseconds(rightAngleSpeed + 1500);

  //leftwristbro.write(leftHandAngle);
  //rightwristbro.write(rightHandAngle);

  digitalWrite(DIR1, leftSpeed >= 0);
  digitalWrite(DIR2, rightSpeed >= 0);

  analogWrite(PWM1, map(abs(leftSpeed), 0, 32678, 0, 255));
  analogWrite(PWM2, map(abs(rightSpeed), 0, 32678, 0, 255));
}

#include <math.h>
#include <AutoPID.h>
#include <Servo.h>

#define L_FOREARM_ACTUATOR_PIN 28
#define L_FOREARM_POT_PIN 14

#define L_ELEV_ACTUATOR_PIN 29
#define L_ELEV_POT_PIN 15

#define R_FOREARM_ACTUATOR_PIN 30
#define R_FOREARM_POT_PIN 16

#define R_ELEV_ACTUATOR_PIN 31
#define R_ELEV_POT_PIN 17

Servo leftArmServo;
Servo leftElevServo;
Servo rightArmServo;
Servo rightElevServo;

const unsigned long interval = 100;  // Interval for updating calculations (in milliseconds)
unsigned long previousMillis = 0;    // Stores the last time the calculation was performed
unsigned long currentMillis = 0;
float t = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Start");
  pinMode(R_FOREARM_POT_PIN, INPUT);
  pinMode(R_ELEV_POT_PIN, INPUT);
  rightArmServo.attach(R_FOREARM_ACTUATOR_PIN);
  rightElevServo.attach(R_ELEV_ACTUATOR_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Calculate time in seconds
    t = currentMillis / 1000.0;

    // x and y positions
    float x = 10 * sin(2 * M_PI * t / 5);  // x-position input (in inches)
    float y = -15;                         // y-position input (in inches)
    float r = 10;
    float L2 = 2;

    // Calculate components of r
    float x0 = r * cos(radians(80));  // Horizontal component of r (in inches)
    float y0 = r * sin(radians(80));  // Vertical component of r (in inches)

    // Calculations
    float L3 = sqrt(x * x + y * y);       // Distance from actuator 2 base to end-effector
    float theta3 = degrees(atan(y / x));  // Angle of actuator 2 relative to horizontal
    float theta2 = 200 - theta3;          // Angle of the link connecting actuator 1 and actuator 2
    float L1 = sqrt(pow(r, 2) - 2 * L2 * (x0 * cos(radians(theta2)) + y0 * sin(radians(theta2))) + pow(L2, 2));

    // Output results to serial monitor
    /*
    Serial.println("Results:");
    Serial.print("Time (s): ");
    Serial.println(t);
    Serial.print("x: ");
    Serial.println(x);
    Serial.print("y: ");
    Serial.println(y);
    Serial.print("L3: ");
    Serial.println(L3);
    Serial.print("L1: ");
    Serial.println(L1);
    Serial.print("Theta3: ");
    Serial.println(theta3);
    Serial.print("Theta2: ");
    Serial.println(theta2);
    Serial.println();
    */
    Serial.println(L1);
    //Serial.println(L3);
    int leftAngleSpeed = map(L1, 0, 18, -500, 500);
    leftElevServo.writeMicroseconds(leftAngleSpeed + 1500);
  }
}

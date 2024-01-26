// X1 WALL-E 2024
// by Joshua Taylor
// 2024-01-26

// Designed for Teensy 4.1

#define DIR1 22
#define PWM1 23
#define DIR2 14
#define PWM2 15

int dev = -1;

void setup() {
  Serial.begin(9600);
  Serial.println("Start");

  Serial1.begin(9600);
  Serial1.println("START");

  // Enable beefy driver serial mode
  Serial2.begin(19200);
  Serial2.write(0x40);
  Serial2.write(0x40);
  Serial2.write(0x40);
  Serial2.write(0x40);

  // pinMode(DIR1, OUTPUT);
  // pinMode(PWM1, OUTPUT);
  // pinMode(DIR2, OUTPUT);
  // pinMode(PWM2, OUTPUT);

  // digitalWrite(DIR1, LOW);
  // digitalWrite(DIR2, LOW);
  delay(3000);
}

void loop() // run over and over
{
  Serial2.write(0x40);

  for (char i = 0x40; i <= 0x7F; i++) {
    Serial2.write(i);
    delay(100);
  }

  for (char i = 0x7F; i > 0; i--) {
    Serial2.write(i);
    delay(100);
  }

  for (char i = 0x01; i <= 0x40; i++) {
    Serial2.write(i);
    delay(100);
  }

  delay(3000);

  // if (Serial1.available()) {
  //   char c = Serial1.read();
  //   Serial.write(c);

  //   // If device variable is -1, we're waiting for the rpi to tell us which device
  //   if (dev == -1) {
  //     switch (c) {
  //       case 'L':
  //         dev = 0;
  //         break;
  //       case 'R':
  //         dev = 1;
  //         break;
  //       default:
  //         Serial.println("Invalid device");
  //         break;
  //     }
  //   }
  //   else if (dev == 0)
  //   {
  //     analogWrite(PWM1, (int)c);
  //     dev = -1;
  //   }
  //   else if (dev == 1)
  //   {
  //     analogWrite(PWM2, (int)c);
  //     dev = -1;
  //   }
  // }
  // if (Serial.available()) {
  //   Serial1.write(Serial.read());
  // }
}

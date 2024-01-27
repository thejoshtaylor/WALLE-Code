// X1 WALL-E 2024
// by Joshua Taylor
// 2024-01-26

// Designed for Teensy 4.1

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

void setup() {
  
  Serial.begin(115200);
  Serial.println("Start");

  Serial1.begin(115200);

}

void loop() // run over and over
{
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
        Serial.print("GOT MESSAGE\t");
        // Normal operation
        leftSpeed = (data[0] << 8) | data[1];
        Serial.print(leftSpeed);
        Serial.print("\t");
        rightSpeed = (data[2] << 8) | data[3];
        Serial.println(rightSpeed);

        leftArmLength = (data[4] << 8) | data[5];
        rightArmLength = (data[6] << 8) | data[7];

        leftArmAngle = (data[8] << 8) | data[9];
        rightArmAngle = (data[10] << 8) | data[11];

        leftWingAngle = (data[12] << 8) | data[13];
        rightWingAngle = (data[14] << 8) | data[15];

        leftHandAngle = (data[16] << 8) | data[17];
        rightHandAngle = (data[18] << 8) | data[19];

        bigFaceLatch = data[20] == 1;
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
    // Set all values to zero
    leftSpeed = 0;
    rightSpeed = 0;

    leftArmLength = 0;
    rightArmLength = 0;

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
}

#include <SPI.h>
#include <RH_RF69.h>

// Singleton instance of the radio driver
RH_RF69 rf69;

// Pin Definitions
const int motorA1 = A0;
const int motorA2 = A1;
const int motorB1 = 4;
const int motorB2 = 5;
const int motorAEnable = 6;  // PWM enable pin for motor A
const int motorBEnable = 7;  // PWM enable pin for motor B

// Variables to store motor speeds
int motorSpeedA, motorSpeedB;
int deadZone = 10;  // Define the dead zone range

void setup() {
  // Set motor pins as outputs
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorAEnable, OUTPUT);
  pinMode(motorBEnable, OUTPUT);

  // Initialize Serial Monitor (for debugging)
  Serial.begin(9600);
  while (!Serial)
    ;

  if (!rf69.init())
    Serial.println("init failed");

  // Set frequency
  if (!rf69.setFrequency(433.0))
    Serial.println("setFrequency failed");

  rf69.setTxPower(14, true);

  // Set encryption key
  uint8_t key[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
}

void loop() {
  if (rf69.available()) {
    // Should be a message for us now
    uint8_t buf[4];
    uint8_t len = sizeof(buf);

    if (rf69.recv(buf, &len)) {
      Serial.print("Received data: ");
      for (int i = 0; i < len; i++) {
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      // Decode the received data
      int forwardSpeed = buf[0] | (buf[1] << 8);
      int turnSpeed = buf[2] | (buf[3] << 8);

      // Calculate motor speeds
      motorSpeedA = forwardSpeed + turnSpeed;  // Left motors
      motorSpeedB = forwardSpeed - turnSpeed;  // Right motors

      // Constrain motor speeds to valid PWM range (-255 to 255)
      motorSpeedA = constrain(motorSpeedA, -155, 155);
      motorSpeedB = constrain(motorSpeedB, -155, 155);

      // Set motor speed using PWM
      analogWrite(motorAEnable, abs(motorSpeedA));
      analogWrite(motorBEnable, abs(motorSpeedB));

      // Handle motor directions based on sign of speed (forward or backward)
      if (motorSpeedA > 0) {
        digitalWrite(motorA1, HIGH);
        digitalWrite(motorA2, LOW);
      } else if (motorSpeedA < 0) {
        digitalWrite(motorA1, LOW);
        digitalWrite(motorA2, HIGH);
      }

      if (motorSpeedB > 0) {
        digitalWrite(motorB1, HIGH);
        digitalWrite(motorB2, LOW);
      } else if (motorSpeedB < 0) {
        digitalWrite(motorB1, LOW);
        digitalWrite(motorB2, HIGH);
      }

      // Print motor speeds for testing
      Serial.print("Motor Speed A: ");
      Serial.println(motorSpeedA);
      Serial.print("Motor Speed B: ");
      Serial.println(motorSpeedB);

      // Send a reply
      uint8_t data[] = "Data received";
      rf69.send(data, sizeof(data));
      rf69.waitPacketSent();
    } else {
      Serial.println("recv failed");
    }
  }
}

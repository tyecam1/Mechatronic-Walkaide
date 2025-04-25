#include <SoftwareSerial.h>
#include <Arduino.h>
SoftwareSerial NAV_ARDUINO(8, 7);  // RX, TX

// ✅ Motor control pins
const int EnA = 6, MotorA1 = A0, MotorA2 = A1;
const int EnB = 3, MotorB1 = 4, MotorB2 = 5;
const int EnC = 11, MotorC1 = A2, MotorC2 = A3;
const int EnD = 9, MotorD1 = 12, MotorD2 = 10;

#define DRIVE_SPEED 90
#define TURN_SPEED 75

void setup() {
  delay(3000);
  Serial.begin(9600);
  NAV_ARDUINO.begin(9600);

  pinMode(EnA, OUTPUT);
  pinMode(MotorA1, OUTPUT);
  pinMode(MotorA2, OUTPUT);
  pinMode(EnB, OUTPUT);
  pinMode(MotorB1, OUTPUT);
  pinMode(MotorB2, OUTPUT);
  pinMode(EnC, OUTPUT);
  pinMode(MotorC1, OUTPUT);
  pinMode(MotorC2, OUTPUT);
  pinMode(EnD, OUTPUT);
  pinMode(MotorD1, OUTPUT);
  pinMode(MotorD2, OUTPUT);

  Serial.println(F("✅ Motor Control Arduino Ready."));
}

void loop() {
  if (NAV_ARDUINO.available()) {

    /*String message = NAV_ARDUINO.readStringUntil('\ln');
    Serial.print("\n Recieved \n");
    Serial.print(message);  // Print received data from Mega
    */

    char command = NAV_ARDUINO.read();
    //NAV_ARDUINO.flush();  // ✅ Clears buffer to prevent stuck commands
      switch (command) {
      case 'F': 
        Serial.println(F("➡ Moving Forward"));
        moveForward();
        break;
      case 'L': 
        Serial.println(F("↪ Turning Left"));
        turnLeft();
        break;
      case 'R': 
        Serial.println(F("↩ Turning Right"));
        turnRight();
        break;
      case 'S': 
        Serial.println(F("🛑 Stopping"));
        stopMotors();
        break;
    }
    
    delay(10);
  }
}

// ✅ Moves forward
void moveForward() {
  digitalWrite(MotorA1, LOW); digitalWrite(MotorA2, HIGH);
  digitalWrite(MotorB1, HIGH); digitalWrite(MotorB2, LOW);
  digitalWrite(MotorC1, HIGH); digitalWrite(MotorC2, LOW);
  digitalWrite(MotorD1, HIGH); digitalWrite(MotorD2, LOW);
  
  analogWrite(EnA, DRIVE_SPEED);
  analogWrite(EnB, DRIVE_SPEED);
  analogWrite(EnC, DRIVE_SPEED);
  analogWrite(EnD, DRIVE_SPEED);

  delay(100);  // ✅ Prevent rapid stopping
}

// ✅ Turns left
void turnLeft() {
  digitalWrite(MotorA1, HIGH); digitalWrite(MotorA2, LOW);
  digitalWrite(MotorB1, HIGH); digitalWrite(MotorB2, LOW);
  digitalWrite(MotorC1, LOW); digitalWrite(MotorC2, HIGH);
  digitalWrite(MotorD1, HIGH); digitalWrite(MotorD2, LOW);
  
  analogWrite(EnA, TURN_SPEED);
  analogWrite(EnB, TURN_SPEED);
  analogWrite(EnC, TURN_SPEED);
  analogWrite(EnD, TURN_SPEED);

  delay(100);  // ✅ Prevent rapid switching
}

// ✅ Turns right
void turnRight() {
  digitalWrite(MotorA1, LOW); digitalWrite(MotorA2, HIGH);
  digitalWrite(MotorB1, LOW); digitalWrite(MotorB2, HIGH);
  digitalWrite(MotorC1, HIGH); digitalWrite(MotorC2, LOW);
  digitalWrite(MotorD1, LOW); digitalWrite(MotorD2, HIGH);
  
  analogWrite(EnA, TURN_SPEED);
  analogWrite(EnB, TURN_SPEED);
  analogWrite(EnC, TURN_SPEED);
  analogWrite(EnD, TURN_SPEED);

  delay(100);  // ✅ Prevent rapid switching
}

// ✅ Stops all motors
void stopMotors() {
  analogWrite(EnA, 0);
  analogWrite(EnB, 0);
  analogWrite(EnC, 0);
  analogWrite(EnD, 0);
}

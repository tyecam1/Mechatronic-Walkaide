#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>

#define ORIENT_SERVO_PIN  9   
#define LINEAR_SERVO_PIN  10  

Servo orientationServo;
Servo linearServo;

ros::NodeHandle nh;

bool emergencyRecoveryActive = false; 
char lastCommand[20] = "";  

void moveWithSequence(int orientationAngle, int linearPosition) {
  emergencyRecoveryActive = false; 
  
  linearServo.write(120); 
  delay(500);

  orientationServo.write(orientationAngle);
  delay(500);

  linearServo.write(linearPosition);
  delay(500);
}

void emergencyRecoveryMode() {
  moveWithSequence(90, 120);
  emergencyRecoveryActive = true;
  while (emergencyRecoveryActive) {
    linearServo.write(0);  
    delay(500);
    linearServo.write(60);   
    delay(500);
    nh.spinOnce();  
  }

  moveWithSequence(90, 120);
}

void commandCallback(const std_msgs::String &msg) {
  if (strncmp(msg.data, lastCommand, 10) == 0) {
    return;
  }
  strncpy(lastCommand, msg.data, sizeof(lastCommand) - 1);
  lastCommand[sizeof(lastCommand) - 1] = '\0';

  if (emergencyRecoveryActive && strncmp(msg.data, "recovery", 8) != 0) {
    emergencyRecoveryActive = false;
  }
  
  if (strncmp(msg.data, "forward", 7) == 0) {
    moveWithSequence(90, 0);
  } else if (strncmp(msg.data, "left", 4) == 0) {
    moveWithSequence(180, 45);
  } else if (strncmp(msg.data, "right", 5) == 0) {
    moveWithSequence(0, 45);
  } else if (strncmp(msg.data, "halfleft", 8) == 0) {
    moveWithSequence(135, 35);
  } else if (strncmp(msg.data, "halfright", 9) == 0) {
    moveWithSequence(45, 35);
  } else if (strncmp(msg.data, "stop", 4) == 0) {
    moveWithSequence(90, 120);
  } else if (strncmp(msg.data, "recovery", 8) == 0) {
    emergencyRecoveryMode();
  } else {
    moveWithSequence(90, 120); 
  }
}

ros::Subscriber<std_msgs::String> sub("arduino_commands", commandCallback);

void setup() {
  orientationServo.attach(ORIENT_SERVO_PIN);
  linearServo.attach(LINEAR_SERVO_PIN);

  linearServo.write(120);
  orientationServo.write(90);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}

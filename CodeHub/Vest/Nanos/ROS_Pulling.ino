#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>

#define LEFT_SERVO_PIN  9
#define RIGHT_SERVO_PIN 10

Servo leftServo;
Servo rightServo;

ros::NodeHandle nh;

void commandCallback(const std_msgs::String &msg) {
  if (strcmp(msg.data, "forward") == 0) {
    leftServo.write(180);
    rightServo.write(0);
    delay(200);
    leftServo.write(0);
    rightServo.write(180);
  } 
  else if (strcmp(msg.data, "left") == 0 || strcmp(msg.data, "halfleft") == 0) {
    leftServo.write(180);
    rightServo.write(0);
    delay(200);
    leftServo.write(180);
    rightServo.write(180);
  } 
  else if (strcmp(msg.data, "right") == 0 || strcmp(msg.data, "halfright") == 0) {
    leftServo.write(180);
    rightServo.write(0);
    delay(200);
    leftServo.write(0);
    rightServo.write(0);
  } 
  else if (strcmp(msg.data, "stop") == 0) {
    leftServo.write(180);
    rightServo.write(0);
  } 
  else {
    nh.loginfo("Unknown command received");
  }
}

ros::Subscriber<std_msgs::String> sub("arduino_commands", commandCallback);

void setup() {
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);

  leftServo.write(180);
  rightServo.write(0);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}

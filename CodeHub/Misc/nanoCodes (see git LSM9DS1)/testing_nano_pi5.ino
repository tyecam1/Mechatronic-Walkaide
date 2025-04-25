#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>

// Define servo pins
#define ORIENT_SERVO_PIN  10   // Rotary/Orientation servo
#define LINEAR_SERVO_PIN  9    // Linear servo

Servo orientationServo;
Servo linearServo;

// Create a ROS node handle
ros::NodeHandle nh;

// Callback function for the "turn" topic
void turnCallback(const std_msgs::String &msg) {
  // Check if the received string is "turn"
  if (strcmp(msg.data, "turn") == 0) {
    // Rotary servo sequence: 0° -> 180° -> 0°
    orientationServo.write(0);
    delay(500);  // Wait for motion
    orientationServo.write(180);
    delay(500);
    orientationServo.write(0);
    
    delay(1000); // Pause for one second before linear servo action

    // Linear servo sequence: 0° -> 180° -> 0°
    linearServo.write(0);
    delay(500);
    linearServo.write(180);
    delay(500);
    linearServo.write(0);
  }
}

// Create a subscriber for the "turn" topic
ros::Subscriber<std_msgs::String> sub("arduino_commands", turnCallback);

void setup() {
  // Attach servos to defined pins
  orientationServo.attach(ORIENT_SERVO_PIN);
  linearServo.attach(LINEAR_SERVO_PIN);

  // Set initial positions
  orientationServo.write(0); // Rotary servo starting at 0°
  linearServo.write(0);      // Linear servo starting at 0°

  // Initialize ROS node and subscriber
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
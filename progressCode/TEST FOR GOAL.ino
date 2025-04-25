#include <Arduino.h>
#include "Arduino_NineAxesMotion.h"  // Sensor library
#include <Wire.h>
#include <ros.h> // Includes ROS functionalities
#include <geometry_msgs/Point.h>  // For receiving goal coordinates
#include <std_msgs/Float64.h>  // For publishing Euler angles (yaw)
#include <std_msgs/Bool.h>  // For goal reached status

NineAxesMotion mySensor;  // Sensor object
const int streamPeriod = 20;  // Stream at 50Hz

// Motor control pins
const int EnA = 6, MotorA1 = A0, MotorA2 = A1;
const int EnB = 3, MotorB1 = 4, MotorB2 = 5;
const int EnC = 11, MotorC1 = A2, MotorC2 = A3;
const int EnD = 9, MotorD1 = 12, MotorD2 = 10;

// Default motor speed values
const int turn_speed = 100;
const int drive_speed = 150;  // Speed when moving forward

// Robot dimensions. In cm.
const float wheel_dist = 8.0; // Distance between wheels
ros::NodeHandle nh;  // Connects the Arduino to ROS

// Publisher for Euler angle
std_msgs::Float64 euler_angle_msg;  // Use Float64 for sending Euler angles
ros::Publisher euler_angle_pub("/arduino_yaw", &euler_angle_msg);  // Publish to /arduino_yaw

// Subscriber for goal coordinates
void goal_cb(const geometry_msgs::Point& msg) {
  // We are only receiving the goal coordinates to turn towards
  float goal_x = msg.x;
  float goal_y = msg.y;

  // Turn to face new direction based on the goal coordinates
  Serial.print("Received New Goal: X=");
  Serial.print(goal_x);
  Serial.print(", Y=");
  Serial.println(goal_y);

  turnToFaceDirection(goal_x, goal_y);  // Turn to face the goal
  driveForward();  // Move forward after turning
}

ros::Subscriber<geometry_msgs::Point> sub("goal_to_arduino", goal_cb);  // Subscribe to goal_to_arduino

// Subscriber for goal reached status
void goalReached_cb(const std_msgs::Bool &msg) {
  if (msg.data) {
    Serial.println("Goal Reached! Stopping.");
    MoveStop();  // Stop the motors when goal is reached
  }
}

// ROS subscriber to listen to goal reached message
ros::Subscriber<std_msgs::Bool> goal_reached_sub("/goal_reached", goalReached_cb);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mySensor.initSensor();
  mySensor.setOperationMode(OPERATION_MODE_NDOF);
  mySensor.setUpdateMode(MANUAL);

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
  pinMode(13, OUTPUT);

  nh.initNode();  // Initialize ROS node
  nh.advertise(euler_angle_pub);  // Advertise the new publisher
  nh.subscribe(goal_reached_sub);  // Subscribe to the goal_reached topic

  Serial.println("Waiting for goal coordinates...");
}

void loop() {
  nh.spinOnce();  // Listen for messages
  delay(10);  // Small delay to allow for incoming ROS messages
}

// Function to stop all motors
void MoveStop() {
  analogWrite(EnA, 0);
  analogWrite(EnB, 0);
  analogWrite(EnC, 0);
  analogWrite(EnD, 0);
}

// Function to turn towards the desired direction and publish Euler angle
void turnToFaceDirection(float goal_x, float goal_y) {
  mySensor.updateEuler();
  float current_heading = mySensor.readEulerHeading();
  float target_heading = atan2(goal_y, goal_x) * 180.0 / PI;
  float angle_diff = target_heading - current_heading;

  // Normalize angle_diff to range [0, 360]
  if (angle_diff < 0) {
    angle_diff += 360;
  }
  if (angle_diff >= 360) {
    angle_diff -= 360;
  }

  int turn_direction = (angle_diff > 0) ? 1 : -1;

  Serial.print("Turning to Face: ");
  Serial.print(target_heading);
  Serial.println(" degrees");

  // Turn until the robot faces the target direction
  while (abs(angle_diff) > 5) {
    Serial.print("Current Heading: ");
    Serial.print(current_heading);
    Serial.print(" | Target: ");
    Serial.print(target_heading);
    Serial.print(" | Angle Diff: ");
    Serial.println(angle_diff);

    digitalWrite(MotorA1, turn_direction == 1 ? LOW : HIGH);
    digitalWrite(MotorA2, turn_direction == 1 ? HIGH : LOW);
    digitalWrite(MotorB1, turn_direction == 1 ? HIGH : LOW);
    digitalWrite(MotorB2, turn_direction == 1 ? LOW : HIGH);
    digitalWrite(MotorC1, turn_direction == 1 ? LOW : HIGH);
    digitalWrite(MotorC2, turn_direction == 1 ? HIGH : LOW);
    digitalWrite(MotorD1, turn_direction == 1 ? HIGH : LOW);
    digitalWrite(MotorD2, turn_direction == 1 ? LOW : HIGH);

    analogWrite(EnA, turn_speed);
    analogWrite(EnB, turn_speed);
    analogWrite(EnC, turn_speed);
    analogWrite(EnD, turn_speed);

    delay(100);
    mySensor.updateEuler();
    current_heading = mySensor.readEulerHeading();
    angle_diff = target_heading - current_heading;

    // Normalize angle_diff to range [0, 360]
    if (angle_diff < 0) {
      angle_diff += 360;
    }
    if (angle_diff >= 360) {
      angle_diff -= 360;
    }

    // Publish the current Euler angle to ROS during the turn
    euler_angle_msg.data = current_heading;  // Set the Euler angle
    euler_angle_pub.publish(&euler_angle_msg);  // Publish the Euler angle

    // Serial output to monitor
    Serial.print("Current Heading Published: ");
    Serial.println(current_heading);
  }

  Serial.println("Turn complete!");
  MoveStop();
}

// Function to drive forward for 1 second
void driveForward() {
  Serial.println("Driving forward...");

  digitalWrite(MotorA1, HIGH);
  digitalWrite(MotorA2, LOW);
  digitalWrite(MotorB1, HIGH);
  digitalWrite(MotorB2, LOW);
  digitalWrite(MotorC1, HIGH);
  digitalWrite(MotorC2, LOW);
  digitalWrite(MotorD1, HIGH);
  digitalWrite(MotorD2, LOW);

  analogWrite(EnA, drive_speed);
  analogWrite(EnB, drive_speed);
  analogWrite(EnC, drive_speed);
  analogWrite(EnD, drive_speed);

  delay(1000);  // Move forward for 1 second
  MoveStop();
}



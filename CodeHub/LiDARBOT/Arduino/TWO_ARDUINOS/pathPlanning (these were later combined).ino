#include <Wire.h>
#include <ros.h>
#include <geometry_msgs/Point.h>  // Goal position from ROS
#include "Arduino_NineAxesMotion.h"  // IMU for orientation

#define MOTOR_ARDUINO Serial1  // ✅ Use Serial1 on Mega for motor communication
#define TURN_TIMEOUT 5000  // ✅ Max time allowed for turning (ms)

NineAxesMotion mySensor;
ros::NodeHandle nh;

float goal_x = 0.0, goal_y = 0.0;
float robot_x = 0.0, robot_y = 0.0;  // ✅ Store robot's current position
const float GOAL_THRESHOLD = 0.2;  // ✅ Distance threshold for reaching goal
char move;


// ✅ Callback for receiving goal coordinates from Raspberry Pi (ROS)
void goal_cb(const geometry_msgs::Point& msg) {
  goal_x = msg.x;
  goal_y = msg.y;

  Serial.print(F("New Goal Received: X="));
  Serial.print(goal_x);
  Serial.print(F(", Y="));
  Serial.println(goal_y);

  moveToGoal();
}

ros::Subscriber<geometry_msgs::Point> goal_sub("/goal_to_arduino", goal_cb);

void setup() {
  delay(2000);  // Allow Serial to initialize
  Serial.begin(9600);
  MOTOR_ARDUINO.begin(9600);
  MOTOR_ARDUINO.println("Navigation Arduino (Mega) Ready.");

  Wire.begin();
  mySensor.initSensor();
  mySensor.setOperationMode(OPERATION_MODE_NDOF);
  mySensor.setUpdateMode(MANUAL);

  nh.initNode();
  nh.subscribe(goal_sub);

  Serial.println("✅ Navigation Arduino (Mega) Ready.");
}

void loop() {
  //nh.spinOnce();
  //MOTOR_ARDUINO.println("Hello from Mega! via Serial1 \n");  // Send test to Uno
  if (Serial.available()>0) {
    move = Serial.read();
    MOTOR_ARDUINO.println(move);
    Serial.println("Sent to Uno");
  }
  //MOTOR_ARDUINO.println("F");  // Send test to Uno
  delay(10);
}

// ✅ Moves toward the goal by turning first, then moving forward
void moveToGoal() {
  mySensor.updateEuler();
  float current_heading = mySensor.readEulerHeading();
  
  // ✅ Compute the heading angle relative to the robot’s current position
  float dx = goal_x - robot_x;
  float dy = goal_y - robot_y;
  float target_heading = atan2(dy, dx) * 180.0 / PI;
  float angle_diff = normalizeAngle(target_heading - current_heading);

  Serial.print(F("Current Heading: ")); Serial.print(current_heading);
  Serial.print(F(" | Target: ")); Serial.print(target_heading);
  Serial.print(F(" | Angle Diff: ")); Serial.println(angle_diff);

  if (abs(angle_diff) > 5) {
    turnToAngle(angle_diff);
  }

  // ✅ Move forward until goal is reached
  Serial.println(F("➡ Moving Forward"));
  MOTOR_ARDUINO.write('F');  

  while (!goalReached()) {
    delay(100);
  }

  MOTOR_ARDUINO.write('S');  // ✅ Stop when goal is reached
  Serial.println(F("🏁 Goal Reached! Stopping."));
}

// ✅ Turns to the correct angle
void turnToAngle(float angle_diff) {
  char turn_command = (angle_diff > 0) ? 'L' : 'R';

  Serial.print(F("↪ Turning "));
  Serial.println((turn_command == 'L') ? "Left" : "Right");

  unsigned long start_time = millis();  // ✅ Start timeout tracking

  while (abs(angle_diff) > 5) {
    MOTOR_ARDUINO.write(turn_command);
    delay(100);

    mySensor.updateEuler();
    float current_heading = mySensor.readEulerHeading();
    
    // ✅ Recalculate angle difference dynamically
    float dx = goal_x - robot_x;
    float dy = goal_y - robot_y;
    float new_target_heading = atan2(dy, dx) * 180.0 / PI;
    angle_diff = normalizeAngle(new_target_heading - current_heading);

    if (millis() - start_time > TURN_TIMEOUT) {
      Serial.println(F("⏳ Timeout: Turn took too long! Stopping."));
      break;  // ✅ Prevent infinite turn loops
    }
  }

  Serial.println(F("✔ Turn complete."));
  MOTOR_ARDUINO.write('S');  // ✅ Stop turning
}

// ✅ Function to check if goal is reached
bool goalReached() {
  // ✅ Replace hardcoded `(0,0)` with actual robot position
  mySensor.updateEuler();  // ✅ Get real-time orientation data

  float distance = sqrt(pow(goal_x - robot_x, 2) + pow(goal_y - robot_y, 2));
  return distance < GOAL_THRESHOLD;
}

// ✅ Normalizes angles to range [-180, 180]
float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

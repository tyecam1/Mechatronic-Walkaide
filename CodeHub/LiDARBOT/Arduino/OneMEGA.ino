
#include <Wire.h>
#include <ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include "Arduino_NineAxesMotion.h"
#include <math.h>

// ===== Motor Control Pin Mapping for Mega =====
// Analog motor pins remapped: A1->A13, A0->A12; A2->A14, A3->A15
// Digital PWM pins remapped: ~3->23, 4->25, ~5->27, ~6->29, ~9->47, ~10->49, ~11->51, 12->53
const int EnA = 3, MotorA1 = A13, MotorA2 = A12;
const int EnB = 5, MotorB1 = 25, MotorB2 = 27;
const int EnC = 4, MotorC1 = A14, MotorC2 = A15;
const int EnD = 2, MotorD1 = 53, MotorD2 = 49;
const int maxPWM = 255;

// ===== APF and ROS Settings =====
#define MAX_OBSTACLES 3
#define GOAL_THRESHOLD 0.5
#define HEADING_DEADZONE 0.1  // Deadzone threshold in radians to reduce left/ right oscillation

// Define how long an obstacle remains "fresh" (in ms)
const unsigned long OBSTACLE_BUFFER_TIME = 200;  // adjust as needed

ros::NodeHandle nh;

// Goal and odometry variables
float goal_x = 0.0, goal_y = 0.0;
bool goal_received = false;
float robot_x = 0.0, robot_y = 0.0;
float yaw = 0.0;

// Obstacle structure with a timestamp
struct Obstacle {
  float x, y;
  unsigned long timestamp;
};
Obstacle obstacles[MAX_OBSTACLES];
int num_obstacles = 0;

// APF parameters (tune as needed)
const float k_att = 1.0;
const float k_rep = 2.0;   // Same ratio as Pi5
const float linear_scaling = 0.37;    // Adjust if needed
const float angular_scaling = 0.4; // Adjust if needed
const float MAX_VEL = 0.34; 
const float MAX_ANG_VEL = 0.45;

// Safety timeout
unsigned long last_update = 0;
const unsigned long TIMEOUT_MS = 2000; // 2 seconds

// ROS publishers
geometry_msgs::Twist vel_msg;
ros::Publisher vel_pub("/cmd_vel", &vel_msg);
std_msgs::Float32 yaw_msg;
ros::Publisher yaw_pub("/arduino/yaw", &yaw_msg);
std_msgs::String debug_msg;
ros::Publisher debug_pub("/arduino/debug_msg", &debug_msg);

// IMU sensor instance
NineAxesMotion mySensor;

// ===== Callback Functions =====
// Goal callback: updates goal and resets timer
void goalCallback(const geometry_msgs::Point& msg) {
  goal_x = msg.x;
  goal_y = msg.y;
  goal_received = true;
  last_update = millis();
  Serial.print("Received goal: ");
  Serial.print(goal_x);
  Serial.print(", ");
  Serial.println(goal_y);
}
ros::Subscriber<geometry_msgs::Point> goal_sub("/goal_point", goalCallback);

// New callback for simple pose messages:
void poseCallback(const geometry_msgs::Point& msg) {
  robot_x = -msg.y;
  robot_y = msg.x;
  last_update = millis();
}
ros::Subscriber<geometry_msgs::Point> pose_sub("/pose", poseCallback);

// In-place filtering: remove stale obstacles from the buffer.
void updateObstacleBuffer() {
  unsigned long currentMillis = millis();
  int j = 0;
  // Loop over existing obstacles and copy only those that are still fresh.
  for (int i = 0; i < num_obstacles; i++) {
    if (currentMillis - obstacles[i].timestamp < OBSTACLE_BUFFER_TIME) {
      // Keep this obstacle by copying it to index j.
      if (i != j) {
        obstacles[j] = obstacles[i];
      }
      j++;
    }
  }
  num_obstacles = j;
}

// Obstacle callback: stores obstacles with their timestamp
void obstacleCallback(const geometry_msgs::Point& msg) {
  if (num_obstacles < MAX_OBSTACLES) {
    obstacles[num_obstacles].x = msg.x;
    obstacles[num_obstacles].y = msg.y;
    obstacles[num_obstacles].timestamp = millis();
    num_obstacles++;
    last_update = millis();
  }
}

ros::Subscriber<geometry_msgs::Point> obstacle_sub("/obstacles_to_arduino", obstacleCallback);

// ===== Motor Control Functions =====
void setMotor(int pwm, int pwmPin, int motorPin1, int motorPin2) {
  if (pwm >= 0) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(pwmPin, pwm);
  } else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    analogWrite(pwmPin, -pwm);
  }
}

void controlMotors(float lin_vel, float ang_vel) {
  int leftPWM = (lin_vel - ang_vel) * maxPWM;
  int rightPWM = (lin_vel + ang_vel) * maxPWM;
  
  leftPWM = constrain(leftPWM, -maxPWM, maxPWM);
  rightPWM = constrain(rightPWM, -maxPWM, maxPWM);
  
  // Set left side motors (EnA and EnC)
  setMotor(leftPWM, EnA, MotorA1, MotorA2);
  setMotor(leftPWM, EnC, MotorC1, MotorC2);
  // Set right side motors (EnB and EnD)
  setMotor(rightPWM, EnB, MotorB1, MotorB2);
  setMotor(rightPWM, EnD, MotorD1, MotorD2);
}

// ===== Function to Stop the Robot =====
void stopRobot() {
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(&vel_msg);
  Serial.println("Stopping motors.");
  
  // Ensure all motor PWM outputs are zero
  analogWrite(EnA, 0);
  analogWrite(EnB, 0);
  analogWrite(EnC, 0);
  analogWrite(EnD, 0);
}

// ===== APF Computation =====
// Compute APF command and output linear and angular velocities via reference parameters
void computeAPF(float &lin_vel, float &ang_vel) {
  if (!goal_received) {
    stopRobot();
    lin_vel = 0;
    ang_vel = 0;
    return;
  }
  
  float d_x = goal_x - robot_x;
  float d_y = goal_y - robot_y;
  float distance_to_goal = sqrt(d_x*d_x + d_y*d_y);
  
  if (distance_to_goal < GOAL_THRESHOLD) {
    stopRobot();
    Serial.println("Goal reached.");
    lin_vel = 0;
    ang_vel = 0;
    return;
  }
  
  // Attractive force
  float F_att_x = k_att * (goal_x - robot_x);
  float F_att_y = k_att * (goal_y - robot_y);
  
  // Repulsive force
  float F_rep_x = 0.0, F_rep_y = 0.0;
  float d0 = 4.0; // influence range
  float robot_heading = yaw * (PI / 180.0);
  for (int i = 0; i < num_obstacles; i++) {
    // Vector from robot to obstacle for angle calculation
    float dx_obs = obstacles[i].x - robot_x;
    float dy_obs = obstacles[i].y - robot_y;
    
    // Calculate the relative angle between robot heading and obstacle direction
    float obs_angle = atan2(dy_obs, dx_obs);
    float angle_diff = obs_angle - robot_heading;
    while (angle_diff > M_PI) angle_diff -= 2*M_PI;
    while (angle_diff < -M_PI) angle_diff += 2*M_PI;
    
    // Smooth scaling: factor = 1 + max(cos(angle_diff), 0)
    // (Directly ahead: factor=2; at 90° or more: factor=1)
    float factor = 1.0 + fmax(0.0, cos(angle_diff));
    
    // Compute repulsive force (from obstacle to robot)
    float dx = robot_x - obstacles[i].x;
    float dy = robot_y - obstacles[i].y;
    float dist = sqrt(dx*dx + dy*dy);
    if (dist > 0 && dist < d0) {
      float rep_mag = factor * k_rep * (1.0/dist - 1.0/d0) / (dist*dist);
      F_rep_x += rep_mag * (dx/dist);
      F_rep_y += rep_mag * (dy/dist);
    }
  }
  
  float F_total_x = F_att_x + F_rep_x;
  float F_total_y = F_att_y + F_rep_y;
  
  float desired_heading = atan2(F_total_y, F_total_x);
  float heading_error = desired_heading - robot_heading;
  while (heading_error > M_PI) heading_error -= 2*M_PI;
  while (heading_error < -M_PI) heading_error += 2*M_PI;
  
  if (fabs(heading_error) < HEADING_DEADZONE) {
    heading_error = 0;
  }
  
  lin_vel = linear_scaling * sqrt(F_total_x*F_total_x + F_total_y*F_total_y);
  ang_vel = angular_scaling * heading_error;
  
  if (lin_vel > MAX_VEL) lin_vel = MAX_VEL;
  if (ang_vel > MAX_ANG_VEL) ang_vel = MAX_ANG_VEL;
  if (ang_vel < -MAX_ANG_VEL) ang_vel = -MAX_ANG_VEL;
  
  vel_msg.linear.x = lin_vel;
  vel_msg.angular.z = ang_vel;
  vel_pub.publish(&vel_msg);
  
  publishDebugInfo(F_att_x, F_att_y, F_rep_x, F_rep_y, F_total_x, F_total_y, heading_error, lin_vel, ang_vel);
  
  num_obstacles = 0;
}

// ===== Debug Publisher =====
void publishDebugInfo(float F_att_x, float F_att_y, float F_rep_x, float F_rep_y, 
                      float F_total_x, float F_total_y, float heading_error, 
                      float lin_vel, float ang_vel) {
  String debugStr = "";
  debugStr += "Pos(" + String(robot_x, 2) + ", " + String(robot_y, 2) + ") ";
  debugStr += "Goal(" + String(goal_x, 2) + ", " + String(goal_y, 2) + ") ";
  debugStr += "Yaw(" + String(yaw, 2) + ") ";
  debugStr += "F_att(" + String(k_att * (goal_x - robot_x), 2) + ", " 
                        + String(k_att * (goal_y - robot_y), 2) + ") ";
  debugStr += "F_rep(" + String(F_rep_x, 2) + ", " + String(F_rep_y, 2) + ") ";
  debugStr += "F_total(" + String(F_total_x, 2) + ", " + String(F_total_y, 2) + ") ";
  debugStr += "HeadErr(" + String(heading_error, 2) + ") ";
  debugStr += "Cmd(" + String(lin_vel, 2) + ", " + String(ang_vel, 2) + ") ";
  debugStr += "Timeout(" + String(millis() - last_update) + ") ";
  
  // Append obstacle information from the buffer
  debugStr += "Obs[";
  for (int i = 0; i < num_obstacles; i++) {
      debugStr += "(" + String(obstacles[i].x, 2) + "," 
                        + String(obstacles[i].y, 2) + ")";
      if (i < num_obstacles - 1) {
         debugStr += ",";
      }
  }
  debugStr += "]";
  
  std_msgs::String msg;
  msg.data = debugStr.c_str();
  debug_pub.publish(&msg);
}




void setup() {
  Serial.begin(57600);
  // Set motor control pins as outputs
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
  
  Wire.begin();
  mySensor.initSensor();
  mySensor.setOperationMode(OPERATION_MODE_NDOF);
  mySensor.setUpdateMode(MANUAL);
  
  nh.initNode();
  nh.subscribe(goal_sub);
  nh.subscribe(pose_sub);

  nh.subscribe(obstacle_sub);
  nh.advertise(vel_pub);
  nh.advertise(debug_pub);
  nh.advertise(yaw_pub);
  
  // Immediately stop motors on startup
  stopRobot();
  last_update = millis();
}

void loop() {
  nh.spinOnce();
  
  // Update IMU-based yaw
  mySensor.updateEuler();
  yaw = -mySensor.readEulerHeading();

  // Publish a heartbeat debug message
  {
    std_msgs::String heartbeat;
    String hb = "Heartbeat: " + String(millis());
    heartbeat.data = hb.c_str();
    debug_pub.publish(&heartbeat);
  }
  
  // Check for communication timeout
  if (millis() - last_update > TIMEOUT_MS) {
    stopRobot();
    last_update = millis();  // Reset the timer so system can resume when data returns
  } else {
    float lin_vel, ang_vel;
    updateObstacleBuffer();
    computeAPF(lin_vel, ang_vel);
    controlMotors(lin_vel, ang_vel);
  }
  
  delay(100);
}
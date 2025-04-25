#include <Arduino_LSM9DS1.h>   // Updated LSM9DS1 library by Femme Verbeek
#include <MadgwickAHRS.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>

Madgwick filter;
ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("nano/imu_corrected", &imu_msg);  // No queue size

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU initialized");

  // Set continuous mode before configuring sensor parameters
  IMU.setContinuousMode();
  
  // Accelerometer configuration:
  // - Full Scale: setting 2 → ±4g
  // - Output Data Rate: setting 4 → 238 Hz
  // - Calibration: offsets and slopes as provided
  IMU.setAccelFS(2);
  IMU.setAccelODR(4);
  IMU.setAccelOffset(-0.013006, -0.007999, -0.024579);
  IMU.setAccelSlope (0.994657, 0.993976, 1.005728);
  
  // Gyroscope configuration:
  // - Full Scale: setting 2 → ±1000°/s
  // - Output Data Rate: setting 4 → 238 Hz
  // - Calibration: offsets and slopes as provided
  IMU.setGyroFS(2);
  IMU.setGyroODR(4);
  IMU.setGyroOffset (1.003296, 3.608795, 1.472778);
  IMU.setGyroSlope (1.181707, 1.131963, 1.142341);
  
  // Magnetometer configuration:
  // - Full Scale: setting 0 → ±400 µT (best for typical indoor fields)
  // - Output Data Rate: setting 6 → 40 Hz
  // - Calibration: offsets and slopes as provided
  IMU.setMagnetFS(0);
  IMU.setMagnetODR(6);
  IMU.setMagnetOffset(0.969238, 19.859009, 18.295288);
  IMU.setMagnetSlope (0.582433, 0.574546, 0.561023);
  
  // Update the Madgwick filter to match the accelerometer/gyroscope frequency (238 Hz)
  filter.begin(238.0);

  nh.initNode();
  nh.advertise(imu_pub);
}

void loop() {
  // Check that all sensors have data available
  if (IMU.accelAvailable() && IMU.gyroAvailable() && IMU.magnetAvailable()) {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    
    // Read sensor data with calibration applied internally
    IMU.readAccel(ax, ay, az);
    IMU.readGyro(gx, gy, gz);
    IMU.readMagnet(mx, my, mz);
    
    // Convert gyroscope readings from degrees/sec to radians/sec
    gx *= (PI / 180.0f);
    gy *= (PI / 180.0f);
    gz *= (PI / 180.0f);
    
    // Update the Madgwick filter with the calibrated sensor data
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    
    // Populate and publish the IMU ROS message
    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = "imu_link";
    
    imu_msg.orientation.w = filter.getQ0();
    imu_msg.orientation.x = filter.getQ1();
    imu_msg.orientation.y = filter.getQ2();
    imu_msg.orientation.z = filter.getQ3();
    
    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;
    
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;
    
    imu_pub.publish(&imu_msg);
  }
  
  nh.spinOnce();  // Process ROS messages
}

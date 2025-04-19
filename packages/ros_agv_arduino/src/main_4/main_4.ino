#include <Arduino.h>
#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/JointState.h>
#include "motor.h"

// --- Encoder and Pin Definitions ---
#define ENCODER_COUNTS_PER_REV 576.0

// Encoder Pins (A channels with interrupts, B channels for direction)
#define ENC1_A 2
#define ENC1_B 26
#define ENC2_A 3
#define ENC2_B 42
#define ENC3_A 18
#define ENC3_B 44
#define ENC4_A 19
#define ENC4_B 52

// --- Global Encoder Variables ---
volatile long encoder1_ticks = 0;
volatile long encoder2_ticks = 0;
volatile long encoder3_ticks = 0;
volatile long encoder4_ticks = 0;

// --- Motor Control Parameters ---
#define PID_UPDATE_FREQ 20.0
#define PUBLISH_FREQ 20.0
const double updateInterval = 1000.0/PID_UPDATE_FREQ;

// --- PID Controllers ---
double kp = 10.0, ki = 13.0, kd = 0.0;

// Motor 1 (Front Left)
double setpoint1, input1, output1;
PID pid1(&input1, &output1, &setpoint1, kp, ki, kd, DIRECT);

// Motor 2 (Front Right)
double setpoint2, input2, output2;
PID pid2(&input2, &output2, &setpoint2, kp, ki, kd, DIRECT);

// Motor 3 (Rear Left)
double setpoint3, input3, output3;
PID pid3(&input3, &output3, &setpoint3, kp, ki, kd, DIRECT);

// Motor 4 (Rear Right)
double setpoint4, input4, output4;
PID pid4(&input4, &output4, &setpoint4, kp, ki, kd, DIRECT);

// --- Motor Objects ---
Motor motorFL(8, 10, 9);   // Front Left
Motor motorFR(13, 11, 12); // Front Right
Motor motorRL(7, 6, 5);    // Rear Left
Motor motorRR(4, 3, 2);    // Rear Right

// --- ROS Configuration ---
ros::NodeHandle nh;
sensor_msgs::JointState jointMsg;
const char* jointNames[4] = {"fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel"};
float jointPositions[4] = {0};
float jointVelocities[4] = {0};

void velocityCallback(const geometry_msgs::Vector3Stamped& cmd) {
  // Example mapping: 
  // cmd.vector.x = linear velocity
  // cmd.vector.y = angular velocity
  // Convert to differential drive wheel velocities
  float base_width = 0.3; // Robot track width in meters
  float linear = cmd.vector.x;
  float angular = cmd.vector.y;
  
  setpoint1 = setpoint3 = linear - (angular * base_width/2);
  setpoint2 = setpoint4 = linear + (angular * base_width/2);
}

ros::Subscriber<geometry_msgs::Vector3Stamped> cmdSub("cmd_vel", &velocityCallback);
ros::Publisher jointPub("joint_states", &jointMsg);

// --- Encoder ISRs ---
void readEncoder1() { 
  digitalRead(ENC1_B) ? encoder1_ticks-- : encoder1_ticks++;
}
void readEncoder2() {
  digitalRead(ENC2_B) ? encoder2_ticks-- : encoder2_ticks++;
}
void readEncoder3() {
  digitalRead(ENC3_B) ? encoder3_ticks-- : encoder3_ticks++;
}
void readEncoder4() {
  digitalRead(ENC4_B) ? encoder4_ticks-- : encoder4_ticks++;
}

void setup() {
  Serial.begin(57600);
  nh.initNode();
  
  // Encoder Setup
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  // ... Repeat for all encoder pins
  
  attachInterrupt(digitalPinToInterrupt(ENC1_A), readEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), readEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), readEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4_A), readEncoder4, CHANGE);

  // PID Setup
  pid1.SetSampleTime(updateInterval);
  pid1.SetOutputLimits(-255, 255);
  // ... Repeat for all PIDs
  
  // ROS Setup
  jointMsg.name_length = 4;
  jointMsg.velocity_length = 4;
  jointMsg.position_length = 4;
  jointMsg.name = (char**)jointNames;
  jointMsg.velocity = jointVelocities;
  jointMsg.position = jointPositions;
  
  nh.advertise(jointPub);
  nh.subscribe(cmdSub);
}

void updatePID() {
  static long lastTicks[4] = {0};
  static unsigned long lastTime = millis();
  
  double dt = (millis() - lastTime) / 1000.0;
  dt = max(dt, 0.001);
  
  // Calculate velocities
  input1 = (encoder1_ticks - lastTicks[0]) * (2*PI/ENCODER_COUNTS_PER_REV) / dt;
  input2 = (encoder2_ticks - lastTicks[1]) * (2*PI/ENCODER_COUNTS_PER_REV) / dt;
  input3 = (encoder3_ticks - lastTicks[2]) * (2*PI/ENCODER_COUNTS_PER_REV) / dt;
  input4 = (encoder4_ticks - lastTicks[3]) * (2*PI/ENCODER_COUNTS_PER_REV) / dt;
  
  // Store current values
  memcpy(lastTicks, &encoder1_ticks, sizeof(lastTicks));
  lastTime = millis();
  
  // Compute PID outputs
  pid1.Compute();
  pid2.Compute();
  pid3.Compute();
  pid4.Compute();
  
  // Update motors
  motorFL.setSpeed(abs(output1));
  motorFR.setSpeed(abs(output2));
  motorRL.setSpeed(abs(output3));
  motorRR.setSpeed(abs(output4));
  
  output1 > 0 ? motorFL.forward() : motorFL.backward();
  output2 > 0 ? motorFR.forward() : motorFR.backward();
  output3 > 0 ? motorRL.forward() : motorRL.backward();
  output4 > 0 ? motorRR.forward() : motorRR.backward();
}

void loop() {
  static unsigned long lastPID = 0, lastPub = 0;
  
  if(millis() - lastPID >= updateInterval) {
    updatePID();
    lastPID = millis();
  }
  
  if(millis() - lastPub >= 1000/PUBLISH_FREQ) {
    // Update joint states
    for(int i=0; i<4; i++) {
      jointPositions[i] += jointVelocities[i] * (millis() - lastPub)/1000.0;
    }
    jointMsg.header.stamp = nh.now();
    jointPub.publish(&jointMsg);
    lastPub = millis();
  }
  
  nh.spinOnce();
}

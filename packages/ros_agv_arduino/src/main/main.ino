/**
 * Low-level Embedded Motor Controller for the Differential Drive AGV
 * Using PID_v1 library
 * 
 * Controls and monitors speed of DC motors using PID controllers and L298N
 * motor drivers. Communicates with a high-level controller using ROSSerial.
 */

#include <Arduino.h>
#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/JointState.h>
#include "motor.h"
#include <Encoder.h>

// --- Encoder and Pin Definitions ---
#define ENC_COUNT_REV 1620.0

#define motorAHallA 2
#define motorAHallB 3
#define motorBHallA 18
#define motorBHallB 19

// --- Update Frequencies ---
#define PID_UPDATE_FREQ 20.0          // Hz for PID updates
#define PUBLISH_UPDATE_SPEED 20.0     // Hz for publishing joint state

// --- Global Encoder & Timing Variables ---
int previousMotorATicks = 0;
int previousMotorBTicks = 0;
long prev_update_time = 0;
long lastUpdate = 0;
long lastPublish = 0;
double updateRate = (1.0 / PID_UPDATE_FREQ) * 1000.0;  // in milliseconds

// --- Motor Angle (for JointState publisher) ---
float motorAAngle = 0;
float motorBAngle = 0;

// --- PID Control Variables for Motor A ---
// In PID_v1, the first argument is Input (measured value),
// second is Output (control command), third is Setpoint (desired value).
double velSetPointA = 0;       // desired angular velocity (rad/s)
double velMotorAInput = 0;     // measured velocity (rad/s) for motor A (PID input)
double pwmMotorA = 0;          // computed PWM output

// --- PID Control Variables for Motor B ---
double velSetPointB = 0;       // desired angular velocity (rad/s)
double velMotorBInput = 0;     // measured velocity (rad/s) for motor B (PID input)
double pwmMotorB = 0;          // computed PWM output


// --- PID Tuning Constants ---
double kp = 10.0;
double ki = 13.0;
double kd = 2.0;
// Note: PID_v1 does not include a built‐in feed‐forward term.

// --- PID Controllers ---
// Order: (&Input, &Output, &Setpoint, kp, ki, kd, DIRECT)
PID pidMotorA(&velMotorAInput, &pwmMotorA, &velSetPointA, kp, ki, kd, DIRECT);
PID pidMotorB(&velMotorBInput, &pwmMotorB, &velSetPointB, kp, ki, kd, DIRECT);

// --- Instantiate Encoders and Motors ---
Encoder motorAEnc(motorAHallA, motorAHallB);
Encoder motorBEnc(motorBHallA, motorBHallB);

Motor motorA = Motor(8, 10, 9);
Motor motorB = Motor(13, 11, 12);

// --- ROS Messaging Setup ---
geometry_msgs::Vector3Stamped speedCtrlMsg;
sensor_msgs::JointState jointStatesMsg;
float jointStatePosition[2];
float jointStateVelocity[2];
float jointStateEffort[2];
// To avoid the string literal conversion warning, use const char*
const char* jointStateNames[2] = {"left_wheel_joint", "right_wheel_joint"};

ros::NodeHandle nh;
ros::Publisher jointStates("joint_states_control", &jointStatesMsg);

// --- ROS Topic Callback --- 
// Reads a Vector3Stamped message and maps the values to desired setpoints.
void speedCtrlCallback(const geometry_msgs::Vector3Stamped& msg) {
  // Example mapping (adjust as needed):
  velSetPointB = msg.vector.x;
  velSetPointA = msg.vector.y;
}
ros::Subscriber<geometry_msgs::Vector3Stamped> 
    speedCtrlSub("angular_velocity_cmd", speedCtrlCallback);

// --- Function Prototypes ---
void calculateMotorVelocity();
void publishState();
void updateMotorControls();

// --- Arduino setup() ---
void setup() {
  Serial.begin(57600);
  nh.initNode();
  // If using Serial, wait until connection established:
  while (!nh.connected()) {
    nh.spinOnce();
  }
  nh.loginfo("Initialize Embedded Motor Controller");

  // --- ROS Joint State Setup ---
  jointStatesMsg.name = (char**)jointStateNames;  // cast acceptable for ROS msg
  jointStatesMsg.name_length = 2;
  jointStatesMsg.position_length = 2;
  jointStatesMsg.velocity_length = 2;
  jointStatesMsg.effort_length = 2;
  
  nh.subscribe(speedCtrlSub);
  nh.advertise(jointStates);

  // --- Initialize Motors ---
  motorA.setSpeed(0);
  motorA.forward();
  
  motorB.setSpeed(0);
  motorB.forward();

  // --- Configure PID Controllers ---
  pidMotorA.SetSampleTime(updateRate);
  pidMotorA.SetOutputLimits(-175, 175);
  pidMotorA.SetMode(AUTOMATIC);

  pidMotorB.SetSampleTime(updateRate);
  pidMotorB.SetOutputLimits(-175, 175);
  pidMotorB.SetMode(AUTOMATIC);
  
  prev_update_time = millis();
  nh.loginfo("Initialization Complete");
}

// --- Arduino loop() ---
void loop() {
  // Update measured velocities from encoders
  calculateMotorVelocity();

  // Set the PID "Input" variables to the measured velocities.
  velMotorAInput = velMotorAInput;  // Already updated in calculateMotorVelocity()
  velMotorBInput = velMotorBInput;  // Same here

  // Compute new PID outputs
  pidMotorA.Compute();
  pidMotorB.Compute();

  // Update motor directions and speeds for Motor A
  if (velSetPointA == 0) {
    motorA.stop();
    pwmMotorA = 0;
  } else {
    if (velSetPointA > 0) {
      motorA.forward();
    } else {
      motorA.backward();
    }
    motorA.setSpeed(constrain(abs((int)pwmMotorA), 0, 255));
  }

  // Update motor directions and speeds for Motor B
  if (velSetPointB == 0) {
    motorB.stop();
    pwmMotorB = 0;
  } else {
    if (velSetPointB > 0) {
      motorB.forward();
    } else {
      motorB.backward();
    }
    motorB.setSpeed(constrain(abs((int)pwmMotorB), 0, 255));
  }

  // Run motor control loop at the defined update rate
  if (millis() - lastUpdate >= updateRate) {
    lastUpdate = millis();
    // Optionally add further control tasks here
  }

  // Publish joint state at defined publish rate
  if (millis() - lastPublish >= updateRate) {
    publishState();
    lastPublish = millis();
  }

  nh.spinOnce();
}

// --- Helper Function Definitions ---

// Reads encoder ticks and calculates angular velocity and updates motor angle.
void calculateMotorVelocity() {
  int motorATicks = motorAEnc.read();
  int motorBTicks = motorBEnc.read();

  long deltaMotorA = (motorATicks - previousMotorATicks);
  long deltaMotorB = (motorBTicks - previousMotorBTicks);

  double deltaAngleMotorA = ((double) deltaMotorA) * (2.0 * PI / ENC_COUNT_REV);
  double deltaAngleMotorB = ((double) deltaMotorB) * (2.0 * PI / ENC_COUNT_REV);

  previousMotorATicks = motorATicks;
  previousMotorBTicks = motorBTicks;

  double dt = (millis() - prev_update_time) / 1000.0;
  if (dt <= 0) dt = 0.001;

  // Here, we interpret the change in angle over time as the measured velocity.
  // (In this example, we simply store it back into the same variable used as PID input.)
  velMotorAInput = deltaAngleMotorA / dt;
  velMotorBInput = deltaAngleMotorB / dt;

  // Update cumulative motor angles (for publishing)
  motorAAngle -= deltaAngleMotorA;
  motorBAngle -= deltaAngleMotorB;
  prev_update_time = millis();
}

// Publishes the current joint state to ROS.
void publishState() {
  // For this example, we assume motorB is left and motorA is right wheel.
  jointStateVelocity[0] = velMotorBInput;
  jointStateVelocity[1] = velMotorAInput;
  jointStatePosition[0] = motorBAngle;
  jointStatePosition[1] = motorAAngle;
  jointStateEffort[0] = 0;
  jointStateEffort[1] = 0;
  
  jointStatesMsg.header.stamp = nh.now();
  jointStatesMsg.position = jointStatePosition;
  jointStatesMsg.velocity = jointStateVelocity;
  jointStatesMsg.effort = jointStateEffort;
  jointStates.publish(&jointStatesMsg);
}

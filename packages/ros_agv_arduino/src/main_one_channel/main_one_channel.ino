#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "motor.h"

// Encoder Ticks per Revolution
#define ENC_COUNT_REV 1620.0

// Hall Sensor Pins (using only one channel per encoder)
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 18

// Update Frequency (in Hz)
#define PID_UPDATE_FREQ 20.0
#define PUBLISH_FREQ 10.0

// Wheelbase and Wheel Radius
#define WHEELBASE 0.2
#define WHEEL_RADIUS 0.032

// Motor Control
Motor motorA(8, 9, 10);
Motor motorB(13, 12, 11);

// ROS NodeHandle and Subscriber
ros::NodeHandle nh;

// Velocity Setpoints and PID variables
double velLeftSetpoint = 0;
double velRightSetpoint = 0;
double velLeftCurrent = 0;
double velRightCurrent = 0;

// Encoder pulse counters
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

// PID Control variables
double Kp = 1.8;
double Ki = 5.0;
double Kd = 0.01;
double leftIntegral = 0;
double rightIntegral = 0;
double leftPrevError = 0;
double rightPrevError = 0;

// Position tracking
float leftWheelPos = 0;
float rightWheelPos = 0;

// ROS message objects
sensor_msgs::JointState jointStateMsg;
char* jointNames[2] = {"left_wheel_joint", "right_wheel_joint"};
float jointPos[2] = {0, 0};
float jointVel[2] = {0, 0};

// ROS publisher
ros::Publisher jointPub("joint_states", &jointStateMsg);

void encoderISR_A() {
  encoderCountA++;
}

void encoderISR_B() {
  encoderCountB++;
}

void cmdVelCallback(const geometry_msgs::Twist& msg) {
    double linear_vel = msg.linear.x;
    double angular_vel = msg.angular.z;

    // Differential drive kinematics
    velLeftSetpoint = (linear_vel - (angular_vel * WHEELBASE / 2.0)) / WHEEL_RADIUS;
    velRightSetpoint = (linear_vel + (angular_vel * WHEELBASE / 2.0)) / WHEEL_RADIUS;
}

ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCallback);

void setup() {
    Serial.begin(115200);
    
    // Initialize ROS node
    nh.initNode();
    nh.subscribe(cmdVelSub);
    
    // Set up joint state publisher
    jointStateMsg.header.frame_id = "base_link";
    jointStateMsg.name = jointNames;
    jointStateMsg.position = jointPos;
    jointStateMsg.velocity = jointVel;
    jointStateMsg.name_length = 2;
    jointStateMsg.position_length = 2;
    jointStateMsg.velocity_length = 2;
    nh.advertise(jointPub);
    
    // Initialize motors
    motorA.setSpeed(0);
    motorA.forward();
    motorB.setSpeed(0);
    motorB.forward();
    
    // Set up encoder interrupts
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR_A, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderISR_B, RISING);
    
    Serial.println("Differential Drive Robot Initialized");
}

void calculateVelocity() {
    static unsigned long lastVelCalcTime = 0;
    static long prevCountA = 0;
    static long prevCountB = 0;
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastVelCalcTime) / 1000.0;
    
    if (deltaTime > 0.01) {
        noInterrupts();
        long currentCountA = encoderCountA;
        long currentCountB = encoderCountB;
        interrupts();
        
        long deltaA = currentCountA - prevCountA;
        long deltaB = currentCountB - prevCountB;
        
        float leftRadians = (deltaA / ENC_COUNT_REV) * 2 * PI;
        float rightRadians = (deltaB / ENC_COUNT_REV) * 2 * PI;
        
        velLeftCurrent = leftRadians / deltaTime;
        velRightCurrent = rightRadians / deltaTime;
        
        leftWheelPos += leftRadians;
        rightWheelPos += rightRadians;
        
        prevCountA = currentCountA;
        prevCountB = currentCountB;
        lastVelCalcTime = currentTime;
    }
}

void updatePID() {
    // Calculate errors
    double leftError = velLeftSetpoint - velLeftCurrent;
    double rightError = velRightSetpoint - velRightCurrent;
    
    // Calculate integral term with anti-windup
    leftIntegral = constrain(leftIntegral + leftError, -50, 50);
    rightIntegral = constrain(rightIntegral + rightError, -50, 50);
    
    // Calculate derivative term
    double leftDerivative = leftError - leftPrevError;
    double rightDerivative = rightError - rightPrevError;
    
    // Calculate PID output
    double leftOutput = Kp * leftError + Ki * leftIntegral + Kd * leftDerivative;
    double rightOutput = Kp * rightError + Ki * rightIntegral + Kd * rightDerivative;
    
    // Convert to PWM values
    int pwmLeft = constrain(leftOutput, -255, 255);
    int pwmRight = constrain(rightOutput, -255, 255);
    
    // Apply to motors
    motorA.setSpeed(abs(pwmLeft));
    if (pwmLeft >= 0) motorA.forward();
    else motorA.backward();
    
    motorB.setSpeed(abs(pwmRight));
    if (pwmRight >= 0) motorB.forward();
    else motorB.backward();
    
    // Save errors for next iteration
    leftPrevError = leftError;
    rightPrevError = rightError;
}

void publishJointStates() {
    jointStateMsg.header.stamp = nh.now();
    jointPos[0] = leftWheelPos;
    jointPos[1] = rightWheelPos;
    jointVel[0] = velLeftCurrent;
    jointVel[1] = velRightCurrent;
    jointPub.publish(&jointStateMsg);
}

void loop() {
    static unsigned long lastPidUpdate = 0;
    static unsigned long lastPublish = 0;
    unsigned long currentTime = millis();
    
    calculateVelocity();
    
    if (currentTime - lastPidUpdate >= (1000 / PID_UPDATE_FREQ)) {
        updatePID();
        lastPidUpdate = currentTime;
    }
    
    if (currentTime - lastPublish >= (1000 / PUBLISH_FREQ)) {
        publishJointStates();
        lastPublish = currentTime;
    }
    
    nh.spinOnce();
    delay(1);
}

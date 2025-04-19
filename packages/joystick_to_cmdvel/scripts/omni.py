#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class MecanumDrive:
    def __init__(self):
        # Publishers for wheel velocities
        self.front_left_pub = rospy.Publisher('/front_left_wheel_velocity', Float32, queue_size=10)
        self.front_right_pub = rospy.Publisher('/front_right_wheel_velocity', Float32, queue_size=10)
        self.back_left_pub = rospy.Publisher('/back_left_wheel_velocity', Float32, queue_size=10)
        self.back_right_pub = rospy.Publisher('/back_right_wheel_velocity', Float32, queue_size=10)

        # Subscriber for joystick inputs
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Robot parameters (adjust based on your robot's geometry)
        self.wheel_radius = 0.05  # Radius of the wheels in meters
        self.wheel_base = 0.3     # Distance between front and back wheels in meters
        self.track_width = 0.3    # Distance between left and right wheels in meters

        # Joystick configuration (adjust for your controller)
        self.JOYSTICK_DEADZONE = 0.1
        self.AXIS_L_STICK_H = 0   # Left stick horizontal (strafing)
        self.AXIS_L_STICK_V = 1   # Left stick vertical (forward/backward)
        self.AXIS_R_STICK_H = 3   # Right stick horizontal (rotation)

        # Speed parameters
        self.base_speed = 1.0     # Base speed scaling factor
        self.max_speed = 2.0      # Maximum speed scaling factor

    def joy_callback(self, data):
        # Apply deadzone to joystick inputs
        linear_x = self.apply_deadzone(data.axes[self.AXIS_L_STICK_V]) * self.base_speed
        linear_y = self.apply_deadzone(data.axes[self.AXIS_L_STICK_H]) * self.base_speed
        angular_z = self.apply_deadzone(data.axes[self.AXIS_R_STICK_H]) * self.base_speed

        # Dynamically scale speed based on joystick position
        speed_scaling_factor = max(abs(linear_x), abs(linear_y), abs(angular_z))
        scaled_speed = self.base_speed + (self.max_speed - self.base_speed) * speed_scaling_factor

        linear_x *= scaled_speed
        linear_y *= scaled_speed
        angular_z *= scaled_speed

        # Compute wheel velocities using mecanum kinematics
        front_left_velocity = (linear_x - linear_y - angular_z * (self.wheel_base + self.track_width)) / self.wheel_radius
        front_right_velocity = (linear_x + linear_y + angular_z * (self.wheel_base + self.track_width)) / self.wheel_radius
        back_left_velocity = (linear_x + linear_y - angular_z * (self.wheel_base + self.track_width)) / self.wheel_radius
        back_right_velocity = (linear_x - linear_y + angular_z * (self.wheel_base + self.track_width)) / self.wheel_radius

        # Publish wheel velocities
        self.front_left_pub.publish(front_left_velocity)
        self.front_right_pub.publish(front_right_velocity)
        self.back_left_pub.publish(back_left_velocity)
        self.back_right_pub.publish(back_right_velocity)

    def apply_deadzone(self, value):
        return 0.0 if abs(value) < self.JOYSTICK_DEADZONE else value

if __name__ == '__main__':
    rospy.init_node('mecanum_drive')
    rospy.loginfo("Mecanum drive node started.")
    mecanum_drive = MecanumDrive()
    rospy.spin()

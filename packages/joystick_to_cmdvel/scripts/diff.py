#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickToCmdVel:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

        # ====== Adjustable Parameters ======
        # Axis mappings (adjust according to your joystick layout)
        self.axis_linear_x = 1      # Left stick vertical axis (forward/backward)
        self.axis_linear_y = 0      # Left stick horizontal axis (left/right)
        self.axis_angular_z = 3     # Right stick horizontal axis (turning)

        # Trigger mappings (L2 and R2)
        self.axis_trigger_L2 = 2    # Left trigger
        self.axis_trigger_R2 = 5    # Right trigger

        # D-Pad button mappings for faster movement (adjusted for YOUR joystick!)
        self.button_dpad_up    = None   # Set correct index or None if unavailable
        self.button_dpad_down  = None
        self.button_dpad_left  = None
        self.button_dpad_right = None

        # Scaling factors for normal movement (maximum speed when joystick is fully deflected)
        self.scale_linear_x = 5.5
        self.scale_linear_y = 4.5
        self.scale_angular_z = 4.0

        # Scaling factors when D-Pad buttons are pressed (fast mode)
        self.fast_scale_linear_x = 3.0
        self.fast_scale_linear_y = 1.0

        # Logarithmic transformation factor (tune this to adjust the "feel")
        self.log_factor = 9.0  # When joystick value is 1, math.log1p(log_factor*1)/math.log1p(log_factor) = 1

    def log_transform(self, value, factor):
        # Map joystick value from [-1, 1] to a logarithmic curve.
        # The result is in the same range as the input.
        return math.copysign(math.log1p(factor * abs(value)) / math.log1p(factor), value)

    def joy_callback(self, data):
        twist = Twist()

        # ====== Read Trigger Values ======
        trigger_L2_value = data.axes[self.axis_trigger_L2] if len(data.axes) > self.axis_trigger_L2 else -1.0
        trigger_R2_value = data.axes[self.axis_trigger_R2] if len(data.axes) > self.axis_trigger_R2 else -1.0

        trigger_L2_normalized = (trigger_L2_value + 1) / 2.0
        trigger_R2_normalized = (trigger_R2_value + 1) / 2.0

        rospy.logdebug("Trigger L2: {:.2f}, Trigger R2: {:.2f}".format(trigger_L2_normalized, trigger_R2_normalized))

        # ====== Check if any D-Pad button is pressed for fast mode ======
        fast_mode_enabled = False
        button_indices = [self.button_dpad_up, self.button_dpad_down,
                          self.button_dpad_left, self.button_dpad_right]

        for idx in button_indices:
            if idx is not None and idx < len(data.buttons):
                if data.buttons[idx]:
                    fast_mode_enabled = True
                    break

        linear_x_scale = self.fast_scale_linear_x if fast_mode_enabled else self.scale_linear_x
        linear_y_scale = self.fast_scale_linear_y if fast_mode_enabled else self.scale_linear_y

        # ====== Linear Movement (Left Joystick) ======
        if len(data.axes) > self.axis_linear_x:
            # Apply logarithmic scaling to the axis input before scaling by the maximum speed.
            twist.linear.x = self.log_transform(data.axes[self.axis_linear_x], self.log_factor) * linear_x_scale
        else:
            twist.linear.x = 0.0

        if len(data.axes) > self.axis_linear_y:
            twist.linear.y = self.log_transform(data.axes[self.axis_linear_y], self.log_factor) * linear_y_scale
        else:
            twist.linear.y = 0.0

        # ====== Rotation Movement (Right Joystick Horizontal Axis) ======
        if len(data.axes) > self.axis_angular_z:
            twist.angular.z = self.log_transform(data.axes[self.axis_angular_z], self.log_factor) * self.scale_angular_z
        else:
            twist.angular.z = 0.0

        # Publish cmd_vel message
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('joystick_to_cmdvel')
    rospy.loginfo("Joystick to cmd_vel node started safely.")
    joystick_to_cmdvel = JoystickToCmdVel()
    rospy.spin()

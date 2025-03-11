#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class XboxJoystickController:
    def __init__(self):
        rospy.init_node('xbox_joystick_controller')

        # Publisher to send cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribe to joystick inputs
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Scaling factors for joystick inputs
        self.linear_scale = 1.0  # Adjust for surge & sway
        self.heave_scale = 1.0   # Adjust for heave
        self.yaw_scale = 1.0     # Adjust for yaw

        # Axis mappings for Xbox controller
        self.surge_axis = 1  # Left stick Up/Down
        self.sway_axis = 0   # Left stick Left/Right
        self.heave_up_trigger = 5  # Right trigger (increases heave)
        self.heave_down_trigger = 2  # Left trigger (decreases heave)
        self.yaw_axis = 3  # Right stick Left/Right
        self.min_control_input = 0

    def joy_callback(self, joy_msg):
        twist = Twist()

        # Map joystick axes to surge, sway, and yaw
        twist.linear.x = self.linear_scale * joy_msg.axes[self.surge_axis]  # Forward/backward
        twist.linear.y = self.linear_scale * joy_msg.axes[self.sway_axis]   # Strafe left/right
        twist.angular.z = self.yaw_scale * joy_msg.axes[self.yaw_axis]      # Rotate left/right

        # Heave (Up/Down)
        # Triggers start at 1 (neutral), move to -1 when pressed
        heave_up = (1 - joy_msg.axes[self.heave_up_trigger]) / 2  # Convert range from [1, -1] to [0, 1]
        heave_down = (1 - joy_msg.axes[self.heave_down_trigger]) / 2  # Convert range from [1, -1] to [0, 1]
        twist.linear.z = self.heave_scale * (heave_up - heave_down)  # Combine trigger inputs

        # Debugging: Print values for inspection
        rospy.loginfo(f"Linear x: {twist.linear.x}, Linear y: {twist.linear.y}, Angular z: {twist.angular.z}, Linear z (Heave): {twist.linear.z}")

        # Only publish if there is some non-zero motion
        if abs(twist.linear.x) >= self.min_control_input or abs(twist.linear.y) >= self.min_control_input or abs(twist.angular.z) >= self.min_control_input or abs(twist.linear.z) >= self.min_control_input:
            self.cmd_vel_pub.publish(twist)
        else:
            rospy.loginfo("No significant motion detected. Not publishing.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = XboxJoystickController()
    node.run()


#!/usr/bin/env python3
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import rospy

class PX4Interface:
    def __init__(self):
        rospy.init_node('px4_interface')
        
        # Publishers
        self.rc_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/joy', Joy, self.joy_callback)  # Direct joy subscription for lights
        
        # Arming and mode services
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # Xbox controller button mappings for lights
        self.button_x = 2  # X button index
        self.button_b = 1  # B button index
        
        # Channel assignments
        self.roll_channel = 0      # Maps to sway (linear.y)
        self.pitch_channel = 1     # Maps to surge (linear.x)
        self.throttle_channel = 2  # Maps to heave (linear.z)
        self.yaw_channel = 3       # Maps to yaw (angular.z)
        self.forward_channel = 4   # Additional forward control
        self.lateral_channel = 5   # Additional lateral control
        self.lights_level_1 = 8    # Light 1 control (X button)
        self.lights_level_2 = 9    # Light 2 control (B button)
        
        # RC signal ranges
        self.rc_min = 1100
        self.rc_mid = 1500
        self.rc_max = 1900
        
        # Light states
        self.light_1_state = False
        self.light_2_state = False
        
        # Previous button states for edge detection
        self.prev_x_button = False
        self.prev_b_button = False
        
        self.last_twist = Twist()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        
        # Initialize the vehicle
        self.arm_and_set_mode()

    def arm_and_set_mode(self):
        """Arms the vehicle and sets mode to MANUAL"""
        rospy.loginfo("Setting mode to MANUAL...")
        self.set_mode_client(base_mode=0, custom_mode="MANUAL")
        rospy.loginfo("Setting COM_RC_IN_MODE to 1...")
        rospy.loginfo("Arming Pixhawk...")
        success = self.arming_client(True)
        if success.success:
            rospy.loginfo("Vehicle armed successfully!")
        else:
            rospy.logwarn("Failed to arm vehicle!")

    def joy_callback(self, joy_msg):
        """Handle Xbox controller input for lights"""
        # X button for Light 1 (toggle on rising edge)
        x_button = bool(joy_msg.buttons[self.button_x])
        if x_button and not self.prev_x_button:
            self.light_1_state = not self.light_1_state
            rospy.loginfo(f"Light 1 {'ON' if self.light_1_state else 'OFF'}")
        self.prev_x_button = x_button
        
        # B button for Light 2 (toggle on rising edge)
        b_button = bool(joy_msg.buttons[self.button_b])
        if b_button and not self.prev_b_button:
            self.light_2_state = not self.light_2_state
            rospy.loginfo(f"Light 2 {'ON' if self.light_2_state else 'OFF'}")
        self.prev_b_button = b_button

    def cmd_vel_callback(self, twist_msg):
        """Handle command velocity messages"""
        self.last_twist = twist_msg

    def scale_to_rc(self, value):
        """Scale from -1:1 to RC range"""
        return int(self.rc_mid + (value * (self.rc_max - self.rc_mid)))

    def timer_callback(self, event):
        """Timer callback to publish RC messages"""
        self.process_and_publish_rc(self.last_twist)

    def process_and_publish_rc(self, twist_msg):
        """Process twist messages and publish RC override"""
        rc_msg = OverrideRCIn()
        rc_msg.channels = [65535] * 18  # Initialize all channels to no override

        # Convert twist commands to RC values
        roll = self.scale_to_rc(twist_msg.linear.y)    # Sway
        pitch = self.scale_to_rc(twist_msg.linear.x)   # Surge
        throttle = self.scale_to_rc(twist_msg.linear.z) # Heave
        yaw = self.scale_to_rc(twist_msg.angular.z)    # Yaw

        # Additional forward/lateral controls (if needed)
        forward = self.scale_to_rc(twist_msg.linear.x)
        lateral = self.scale_to_rc(twist_msg.linear.y)

        # Apply limits and set channel values
        rc_msg.channels[self.roll_channel] = max(self.rc_min, min(self.rc_max, roll))
        rc_msg.channels[self.pitch_channel] = max(self.rc_min, min(self.rc_max, pitch))
        rc_msg.channels[self.throttle_channel] = max(self.rc_min, min(self.rc_max, throttle))
        rc_msg.channels[self.yaw_channel] = max(self.rc_min, min(self.rc_max, yaw))
        rc_msg.channels[self.forward_channel] = max(self.rc_min, min(self.rc_max, forward))
        rc_msg.channels[self.lateral_channel] = max(self.rc_min, min(self.rc_max, lateral))

        # Set light channels based on button states
        rc_msg.channels[self.lights_level_1] = self.rc_max if self.light_1_state else self.rc_min
        rc_msg.channels[self.lights_level_2] = self.rc_max if self.light_2_state else self.rc_min

        # Publish the RC message
        self.rc_pub.publish(rc_msg)

    def run(self):
        """Main run loop"""
        rospy.spin()

if __name__ == '__main__':
    node = PX4Interface()
    node.run()

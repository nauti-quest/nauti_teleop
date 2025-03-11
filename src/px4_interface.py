from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import Twist
import rospy

class PX4Interface:
    def __init__(self):
        rospy.init_node('px4_interface')
        self.rc_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Arming and mode services
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.arm_and_set_mode()

        self.roll_channel = 0
        self.pitch_channel = 1
        self.throttle_channel = 2
        self.yaw_channel = 3
        self.forward_channel = 4
        self.lateral_channel = 5
        self.lights_level_1 = 8
        self.lights_level_2 = 9

        self.rc_min = 1250  #1100
        self.rc_mid = 1500  
        self.rc_max = 1750 #1900

        self.last_twist = Twist()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)  

    def arm_and_set_mode(self):
        """ Arms the vehicle and sets mode to MANUAL """
        rospy.loginfo("Setting mode to MANUAL...")
        self.set_mode_client(base_mode=0, custom_mode="MANUAL")
            # Set COM_RC_IN_MODE to 1
        rospy.loginfo("Setting COM_RC_IN_MODE to 1...")
        rospy.loginfo("Arming Pixhawk...")
        success = self.arming_client(True)
        if success.success:
            rospy.loginfo("Vehicle armed successfully!")
        else:
            rospy.logwarn("Failed to arm vehicle!")

    def cmd_vel_callback(self, twist_msg):
        self.last_twist = twist_msg

    def scale_to_rc(self, value):
        return int(self.rc_mid + (value * (self.rc_max - self.rc_mid))) 

    def timer_callback(self, event):
        self.process_and_publish_rc(self.last_twist)

    def process_and_publish_rc(self, twist_msg):
        rc_msg = OverrideRCIn()
        rc_msg.channels = [65535] * 18  
        
        forward = self.scale_to_rc(twist_msg.linear.x)
        lateral = self.scale_to_rc(twist_msg.linear.y)
        #roll = self.scale_to_rc(twist_msg.angular.y)  
        #pitch = self.scale_to_rc(twist_msg.angular.x)  
        throttle = self.scale_to_rc(twist_msg.linear.z)
        yaw = self.scale_to_rc(twist_msg.angular.z)
        rc_msg.channels[self.forward_channel] = max(self.rc_min, min(self.rc_max, forward))
        rc_msg.channels[self.lateral_channel] = max(self.rc_min, min(self.rc_max, lateral))
        #rc_msg.channels[self.roll_channel] = max(self.rc_min, min(self.rc_max, roll))
        #rc_msg.channels[self.pitch_channel] = max(self.rc_min, min(self.rc_max, pitch))
        rc_msg.channels[self.throttle_channel] = max(self.rc_min, min(self.rc_max, throttle))
        rc_msg.channels[self.yaw_channel] = max(self.rc_min, min(self.rc_max, yaw))

        self.rc_pub.publish(rc_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PX4Interface()
    node.run()

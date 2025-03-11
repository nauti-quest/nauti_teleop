# """
# Example of how to use RC_CHANNEL_OVERRIDE messages to force input channels
# in Ardupilot. These effectively replace the input channels (from joystick
# or radio), NOT the output channels going to thrusters and servos.
# """

# # Import mavutil
# from pymavlink import mavutil
# import time

# # Create the connection
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# # Wait a heartbeat before sending commands
# master.wait_heartbeat()

# # Create a function to send RC values
# # More information about Joystick channels
# # here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
# def set_rc_channel_pwm(channel_id, pwm=1500):
#     """ Set RC channel pwm value
#     Args:
#         channel_id (TYPE): Channel ID
#         pwm (int, optional): Channel pwm value 1100-1900
#     """
#     if channel_id < 1 or channel_id > 18:
#         print("Channel does not exist.")
#         return

#     # Mavlink 2 supports up to 18 channels:
#     # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
#     rc_channel_values = [65535 for _ in range(8)]
#     rc_channel_values[channel_id - 1] = pwm
#     master.mav.rc_channels_override_send(
#         master.target_system,                # target_system
#         master.target_component,             # target_component
#         *rc_channel_values)                  # RC channel list, in microseconds.

# channel = 0
# while True: 
#     channel = (channel + 1) % 6
#     set_rc_channel_pwm(channel, 1300)
#     print("channel pwm changed for channel ", channel)
#     time.sleep(5)
#     set_rc_channel_pwm(channel, 1500)


"""
Example of how to connect pymavlink to an autopilot via an UDP connection
"""

# Disable "Bare exception" warning
# pylint: disable=W0702

import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
#  If using a companion computer
#  the default connection is available
#  at ip 192.168.2.1 and the port 14550
# Note: The connection is done with 'udpin' and not 'udpout'.
#  You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
#  uses a 'udpbcast' (client) and not 'udpin' (server).
#  If you want to use QGroundControl in parallel with your python script,
#  it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
#  E.g: --out udpbcast:192.168.2.255:yourport
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Make sure the connection is valid
master.wait_heartbeat()

# Get some information !
while True:
    try:
        print(master.recv_match().to_dict())
    except:
        pass
    time.sleep(0.1)
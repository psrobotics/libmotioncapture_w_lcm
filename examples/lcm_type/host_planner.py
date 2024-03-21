import socket
import numpy as np
import struct
import math
import time

# lcm init
import lcm
from exlcm import motion_t

def send_udp_message(message, ip, port):
    """
    Send a UDP message to the specified IP address and port.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(message, (ip, port))

# lcm handle function
def my_handler(channel, data):
    msg = motion_t.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   timestamp   = %s" % str(msg.timestamp))
    print("   latency   = %s" % str(msg.latency))
    print("   position    = %s" % str(msg.position))
    print("   orientation = %s" % str(msg.orientation))
    print("   rb_name        = '%s'" % msg.rb_name)
    print("   enabled     = %s" % str(msg.enabled))
    print("")


# setup lcm
lc = lcm.LCM()
subscription = lc.subscribe("VICON_LCM", my_handler)
print("lcm init done!")

# setup socket
ip = "192.168.123.15"  # destination IP of the jetson NX onboard
port = 8088  # replace with the destination port number

# command params, some with default trotting gait
cmd_x_vel = 0.0
cmd_y_vel = 0.0
cmd_yaw_vel = 0.0
cmd_height = 0.0 # -0.3~0.3, with 0 as normal height
cmd_freq = 2.0 # 2.0~4.0
cmd_phase = 0.5 # gait phase
cmd_offset = 0.0 # gait offset
cmd_bound = 0.0
cmd_duration = 0.5
cmd_footswing = 0.08 # foot swing height max(0,adj)*0.32+0.03
cmd_ori_pitch = 0.0 # -0.4~0.4
cmd_ori_roll = 0.0
cmd_stance_width = 0.33 # fpp width
cmd_stance_length = 0.40 # fpp len
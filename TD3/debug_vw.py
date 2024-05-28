import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from collections import deque


# Create a figure and axis
fig, axs = plt.subplots(2)
axs[0].grid()
axs[1].grid()
# axs[0].set_title('v')
# axs[1].set_title('w')

# Create an empty scatter plot object
odom_v_list = deque(maxlen=100)
cmd_v_list = deque(maxlen=100)
odom_w_list = deque(maxlen=100)
cmd_w_list = deque(maxlen=100)
sc_odom_v, = axs[0].plot([], [])
sc_cmd_v, = axs[0].plot([], [])
sc_odom_w, = axs[1].plot([], [])
sc_cmd_w, = axs[1].plot([], [])
sc_odom_v.set_label('odom_v')
sc_cmd_v.set_label('cmd_v')
sc_odom_w.set_label('odom_w')
sc_cmd_w.set_label('cmd_w')
axs[0].legend(loc='upper left')
axs[1].legend(loc='upper left')

odom_v, odom_w, cmd_v, cmd_w = 0, 0, 0, 0

# Animation function: this is called sequentially
def plot(a):
    global odom_v, odom_w, cmd_v, cmd_w
    try:
        cmd_msg = rospy.wait_for_message('/r1/cmd_vel', Twist, timeout=0.25)
        cmd_v = cmd_msg.linear.x
        cmd_w = cmd_msg.angular.z
    except:
        pass
    try:
        odom_msg = rospy.wait_for_message('/r1/odom', Odometry, timeout=0.25)
        odom_v = odom_msg.twist.twist.linear.x
        odom_w = -odom_msg.twist.twist.angular.z
    except:
        pass
    
    odom_v_list.append(odom_v)
    odom_w_list.append(odom_w)
    cmd_v_list.append(cmd_v)
    cmd_w_list.append(cmd_w)

    sc_odom_v.set_data(np.arange(len(odom_v_list)), odom_v_list)
    sc_cmd_v.set_data(np.arange(len(cmd_v_list)), cmd_v_list)
    sc_odom_w.set_data(np.arange(len(odom_w_list)), odom_w_list)
    sc_cmd_w.set_data(np.arange(len(cmd_w_list)), cmd_w_list)

    axs[0].relim()
    axs[0].autoscale_view(True, True, True)
    axs[1].relim()
    axs[1].autoscale_view(True, True, True)

rospy.init_node('hi', anonymous=True)

# Create the animation object
# ani = FuncAnimation(fig, plot, fargs=(data,), frames=4, interval=100, repeat=True)
ani = FuncAnimation(fig, plot, frames=4, interval=100, repeat=True)

# Show the animation
plt.show()

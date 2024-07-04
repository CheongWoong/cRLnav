# additional reference: https://github.com/erlerobot/gym-gazebo/blob/master/gym_gazebo/envs/erlecopter/gazebo_erlecopter_hover.py

import math
import os
import random
import subprocess
import time
from os import path

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from squaternion import Quaternion
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.35
TIME_DELTA = 0.2
ALTITUDE = 3.0
ACCELERATION_LIMIT = np.array([1.0, 1.0, 1.0]) # m/s, rad/s, m/s
ACCELERATION_LIMIT *= TIME_DELTA


# Check if the random goal position is located on an obstacle and do not accept it if it is
def check_pos(x, y):
    goal_ok = True

    return goal_ok


class GazeboEnv:
    """Superclass for all Gazebo environments."""

    def __init__(self, environment_dim):
        self.invalid_action_clipping = True

        self.environment_dim = environment_dim
        self.odom_x = 0
        self.odom_y = 0

        self.goal_x = 1
        self.goal_y = 0.0

        self.upper = 5.0
        self.lower = -5.0
        self.last_odom = None

        self.set_self_state = ModelState()
        self.set_self_state.model_name = "iris"
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 0.0
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0

        port = "11311"

        # subprocess.Popen(["roslaunch", "-p", port, "px4", "mavros_posix_sitl.launch"])
        # print("mavros_posix_sitl launched!")

        rospy.init_node("gym", anonymous=True)

        # Set up the ROS publishers and subscribers
        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.set_state = rospy.Publisher(
            "gazebo/set_model_state", ModelState, queue_size=10
        )
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.reset_proxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.publisher = rospy.Publisher("goal_point", MarkerArray, queue_size=3)
        self.publisher2 = rospy.Publisher("linear_velocity", MarkerArray, queue_size=1)
        self.publisher3 = rospy.Publisher("angular_velocity", MarkerArray, queue_size=1)
        self.odom = rospy.Subscriber(
            "/mavros/local_position/odom", Odometry, self.odom_callback, queue_size=1
        )

        self.mode_proxy = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.arm_proxy = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.takeoff_proxy = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)

        countdown = 5
        while countdown > 0:
            print ("Taking off in in %ds"%countdown)
            countdown -= 1
            time.sleep(1)

        # Set OFFBOARD mode
        rospy.wait_for_service('mavros/set_mode')
        try:
            self.mode_proxy(0, 'OFFBOARD')
        except (rospy.ServiceException) as e:
            print ("mavros/set_mode service call failed: %s"%e)

        # # Arm throttle
        # print("Arming")
        # rospy.wait_for_service('mavros/cmd/arming')
        # try:
        #     self.arm_proxy(True)
        # except Exception as e:
        #     print ("mavros/set_mode service call failed: %s"%e)
        # time.sleep(3)

        # # Takeoff
        # print("Taking off...")
        # rospy.wait_for_service('mavros/set_mode')
        # try:
        #     self.mode_proxy(0, 'AUTO.TAKEOFF')
        # except Exception as e:
        #     print ("mavros/set_mode service call failed: %s"%e)
        # time.sleep(5)

        # # Set OFFBOARD mode
        # rospy.wait_for_service('mavros/set_mode')
        # try:
        #     self.mode_proxy(0, 'OFFBOARD')
        # except (rospy.ServiceException) as e:
        #     print ("mavros/set_mode service call failed: %s"%e)

    def odom_callback(self, od_data):
        self.last_odom = od_data

    # Perform an action and read a new state
    def step(self, action):
        target = False

        # Publish the robot action
        vel_cmd = Twist()
        ###############################################################
        # invalid action masking (clipping) with acceleration limit and linear velocity near the goal
        if self.invalid_action_clipping:
            prev_action = self.prev_state[-3:]
            action_diff = action - prev_action
            action_diff = np.clip(action_diff, -ACCELERATION_LIMIT, ACCELERATION_LIMIT)
            action = prev_action + action_diff

            ### Maybe do this only at inference time
            # dist = self.prev_state[0]
            # lin_vel_limit = dist / 5
            # action[0] = np.clip(action[0], 0, lin_vel_limit)
        ###############################################################
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        vel_cmd.linear.z = action[2]
        self.vel_pub.publish(vel_cmd)
        self.publish_markers(action)

        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        # propagate state for TIME_DELTA seconds
        time.sleep(TIME_DELTA)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            pass
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

        done, collision = False, False

        # Calculate robot heading from odometry data
        self.odom_x = self.last_odom.pose.pose.position.x
        self.odom_y = self.last_odom.pose.pose.position.y
        self.odom_z = self.last_odom.pose.pose.position.z
        quaternion = Quaternion(
            self.last_odom.pose.pose.orientation.w,
            self.last_odom.pose.pose.orientation.x,
            self.last_odom.pose.pose.orientation.y,
            self.last_odom.pose.pose.orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

        # Calculate distance to the goal from the robot
        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        # Calculate the relative angle between the robots heading and heading toward the goal
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Detect if the goal has been reached and give a large positive reward
        if distance < GOAL_REACHED_DIST:
            target = True
            done = True

        altitude = self.odom_z
        if altitude < 1:
            done = True
        robot_state = [distance, theta, altitude - ALTITUDE, action[0], action[1], action[2]]
        state = np.array(robot_state)
        reward = self.get_reward(target, collision, action, altitude)
        self.prev_state = np.array(robot_state)
        return state, reward, done, target

    def reset(self):

        # Landing
        print("Landing...")
        rospy.wait_for_service('mavros/set_mode')
        try:
            self.mode_proxy(0, 'AUTO.LAND')
        except Exception as e:
            print ("mavros/set_mode service call failed: %s"%e)
        time.sleep(5)

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service("/gazebo/reset_world")
        try:
            self.reset_proxy()

        except rospy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")

        angle = np.random.uniform(-np.pi, np.pi)
        quaternion = Quaternion.from_euler(0.0, 0.0, angle)
        object_state = self.set_self_state

        x = 0
        y = 0
        position_ok = False
        while not position_ok:
            x = np.random.uniform(-4.5, 4.5)
            y = np.random.uniform(-4.5, 4.5)
            position_ok = check_pos(x, y)
        object_state.pose.position.x = x
        object_state.pose.position.y = y
        object_state.pose.position.z = 0.1 # ALTITUDE
        object_state.pose.orientation.x = quaternion.x
        object_state.pose.orientation.y = quaternion.y
        object_state.pose.orientation.z = quaternion.z
        object_state.pose.orientation.w = quaternion.w
        self.set_state.publish(object_state)

        self.odom_x = object_state.pose.position.x
        self.odom_y = object_state.pose.position.y
        self.odom_z = object_state.pose.position.z

        # Arm throttle
        print("Arming")
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            self.arm_proxy(True)
        except Exception as e:
            print ("mavros/set_mode service call failed: %s"%e)
        time.sleep(3)

        # Takeoff
        print("Taking off...")
        rospy.wait_for_service('mavros/set_mode')
        try:
            self.mode_proxy(0, 'AUTO.TAKEOFF')
        except Exception as e:
            print ("mavros/set_mode service call failed: %s"%e)
        time.sleep(5)

        # Set OFFBOARD mode
        rospy.wait_for_service('mavros/set_mode')
        try:
            self.mode_proxy(0, 'OFFBOARD')
        except (rospy.ServiceException) as e:
            print ("mavros/set_mode service call failed: %s"%e)

        # set a random goal in empty space in environment
        self.change_goal()
        self.publish_markers([0.0, 0.0])

        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        time.sleep(TIME_DELTA)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y

        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle

        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        altitude = self.odom_z
        robot_state = [distance, theta, altitude - ALTITUDE, 0.0, 0.0, 0.0]
        self.prev_state = np.array(robot_state)
        state = np.array(robot_state)
        return state

    def change_goal(self):
        # Place a new goal and check if its location is not on one of the obstacles
        if self.upper < 10:
            self.upper += 0.004
        if self.lower > -10:
            self.lower -= 0.004

        goal_ok = False

        while not goal_ok:
            self.goal_x = self.odom_x + random.uniform(self.upper, self.lower)
            self.goal_y = self.odom_y + random.uniform(self.upper, self.lower)
            goal_ok = check_pos(self.goal_x, self.goal_y)

    def publish_markers(self, action):
        # Publish visual data in Rviz
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0

        markerArray.markers.append(marker)

        self.publisher.publish(markerArray)

        markerArray2 = MarkerArray()
        marker2 = Marker()
        marker2.header.frame_id = "odom"
        marker2.type = marker.CUBE
        marker2.action = marker.ADD
        marker2.scale.x = abs(action[0])
        marker2.scale.y = 0.1
        marker2.scale.z = 0.01
        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.pose.position.x = 5
        marker2.pose.position.y = 0
        marker2.pose.position.z = 0

        markerArray2.markers.append(marker2)
        self.publisher2.publish(markerArray2)

        markerArray3 = MarkerArray()
        marker3 = Marker()
        marker3.header.frame_id = "odom"
        marker3.type = marker.CUBE
        marker3.action = marker.ADD
        marker3.scale.x = abs(action[1])
        marker3.scale.y = 0.1
        marker3.scale.z = 0.01
        marker3.color.a = 1.0
        marker3.color.r = 1.0
        marker3.color.g = 0.0
        marker3.color.b = 0.0
        marker3.pose.orientation.w = 1.0
        marker3.pose.position.x = 5
        marker3.pose.position.y = 0.2
        marker3.pose.position.z = 0

        markerArray3.markers.append(marker3)
        self.publisher3.publish(markerArray3)

    @staticmethod
    def get_reward(target, collision, action, altitude):
        if target:
            return 100.0
        elif collision or altitude < 1:
            return -100.0
        else:
            r3 = lambda x: abs(x - ALTITUDE)
            return action[0] / 2 - abs(action[1]) / 2 - abs(action[2]) / 2 - r3(altitude) / 2
#!/usr/bin/env python

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn
from tools.sample_scan import SampleScan

class Env():
    def __init__(self, action_size, random_goal_position):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.lastPose = Pose()
        self.pub_cmd_vel = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('/robot1/odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        
        self.respawn_goal = Respawn(random_goal_position)

        self.sample_scan = SampleScan(10)

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)

        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        
        invertHeadging = False
        if self.lastPose == odom.pose.pose:
            invertHeading = True;
            rospy.loginfo("Same pose")
        self.lastPose == odom.pose.pose

        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        if invertHeadging == True:
            self.heading = - round(heading, 2)
        else:
            self.heading = round(heading, 2)

    
    def getState(self, scan):
        scan_range = []
        heading = self.heading
        min_range = 0.17
        done = False
                
        scan_range = self.sample_scan.clean_data(scan)

        if min_range > min(scan_range) > 0:
            done = True

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance < 0.2:
            self.get_goalbox = True

        return scan_range + [heading, current_distance], done
    
    # reward the actions where the heading points to the target
    def heading_reward(self, heading):
        yaw_reward = []
        for i in range(self.action_size):
            angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            #print("i: ", i, " angle: ", angle, " tr: ", tr)
            yaw_reward.append(tr)

        return yaw_reward


    def setReward(self, state, collision, action):
        yaw_reward = []
        current_distance = state[-1]
        heading = state[-2]

        yaw_reward = self.heading_reward(heading)

        distance_rate = 2 ** (current_distance / self.goal_distance)
        reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate)

        if collision:
            rospy.loginfo("Collision!!")
            reward = -100
            self.pub_cmd_vel.publish(Twist()) # stop

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 200
            self.pub_cmd_vel.publish(Twist()) #stop
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True)
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False

        return reward


    def step(self, action):
        max_angular_vel = 1.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15
        vel_cmd.angular.z = ang_vel
       
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('robot1_base_scan', LaserScan, timeout=5)
            except:
                pass

        data = self.sample_scan.get_sample_from_laser_scan(data.ranges)

        state, collision = self.getState(data)
        reward = self.setReward(state, collision, action)

        return np.asarray(state), reward, collision, self.get_goalbox

    def reset(self):
        
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('robot1_base_scan', LaserScan, timeout=50)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(False)
            self.initGoal = False

        data = self.sample_scan.get_sample_from_laser_scan(data.ranges)
        self.goal_distance = self.getGoalDistace()
        state, done = self.getState(data)

        return np.asarray(state)
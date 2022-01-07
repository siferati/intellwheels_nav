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

class Env():
    def __init__(self, action_size):
        self.goal_x = 0
        self.goal_y = 0
        
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        
        self.heading_r1 = 0
        self.position_r1 = Pose()

        self.heading_r2 = 0
        self.position_r2 = Pose()

        self.distance_to_chair1 = 1000;
        
        self.pub_cmd_vel_r1 = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=5)
        self.pub_cmd_vel_r2 = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=5)
        
        self.sub_odom_r1 = rospy.Subscriber('/robot1/odom', Odometry, self.getOdometryRobot1)
        self.sub_odom_r2 = rospy.Subscriber('/robot1/odom', Odometry, self.getOdometryRobot2)

        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        
        self.respawn_goal = Respawn()

    def getDistance(self, pos_x, pos_y, goal_x, goal_y):
        #goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        return round(math.hypot(goal_x - pos_x, goal_y - pos_y), 2)

    def getOdometryRobot1(self, odom):
        position_r1, self.heading_r1 = self.getOdometry(odom)
        # avoid get stuck in a corner
        if self.position_r1 == position_r1:
            self.heading_r1 = - self.heading_r1
        self.position_r1 = position_r1

    def getOdometryRobot2(self, odom):
        position_r2, self.heading_r2 = self.getOdometry(odom)
        # avoid get stuck in a corner
        if self.position_r2 == position_r2:
            self.heading_r2 = - self.heading_r2
        self.position_r2 = position_r2

    def getOdometry(self, odom):
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - position.y, self.goal_x - position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        heading = round(heading, 2)

        return position, heading

    def getState(self, scan):
        scan_range = []
        heading = self.heading_r1
        min_range = 0.13
        done = False
        
        #this need more work

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        if min_range > min(scan_range) > 0:
            rospy.loginfo("Wall collision !!")
            done = True

        current_distance_to_goal = self.getDistance(self.goal_x, self.position_r1.x, self.goal_y, self.position_r1.y)
        if current_distance_to_goal < 0.2:
            self.get_goalbox = True
        
        #print distance bewtween the robots
        chair_get_close = False
        current_distance_to_chair1 = self.getDistance(self.position_r1.x, self.position_r2.x, self.position_r1.y, self.position_r2.y) 
        if(current_distance_to_chair1 < self.distance_to_chair1):
            self.distance_to_chair1 = current_distance_to_chair1
            chair_get_close = True;

        chair_collision = False
        if(current_distance_to_chair1 < 0.5):
            chair_collision = True

        #rospy.loginfo("Distance to chair 1 = %f", current_distance_to_chair1)

        return scan_range + [heading, current_distance_to_goal], done, chair_collision, chair_get_close

    def setReward(self, state, wall_collision, chair_collision, chair_get_close ,action):
        yaw_reward = []
        current_distance = state[-1]
        heading = state[-2]

        #chair_collision = False

        for i in range(5):
            angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            yaw_reward.append(tr)

        distance_rate = 2 ** (current_distance / self.goal_distance)
        reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate)

        if wall_collision:
            rospy.loginfo("Collision!!")
            reward = -200
            self.pub_cmd_vel_r1.publish(Twist()) # stop

        if chair_collision:
            rospy.loginfo("Possible chair collision!!")
            reward = -200
            self.pub_cmd_vel_r2.publish(Twist()) # stop

        if chair_collision:
            rospy.loginfo("Possible chair collision!!")
            reward = -200
            self.pub_cmd_vel_r2.publish(Twist()) # stop

        if chair_get_close:
            #rospy.loginfo("Chair2 is getting close to Chair1")
            reward = 200
        else: 
            #rospy.loginfo("Chair2 is getting far to Chair1")
            reward = -200


        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 200
            self.pub_cmd_vel_r1.publish(Twist()) #stop
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getDistance(self.goal_x, self.position_r1.x, self.goal_y, self.position_r1.y)
            self.get_goalbox = False

        return reward

    
    def checkIfIsInSamePosition(self):
        return False
    
    def step(self, action):
        
        max_angular_vel = 1.5

        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15
        vel_cmd.angular.z = ang_vel

    

       
        # publish the new speed
        self.pub_cmd_vel_r1.publish(vel_cmd)

        #print ("Action ", ang_vel)

        # mimic the movement
        self.pub_cmd_vel_r2.publish(vel_cmd)

        # print ("New speed: ", vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('robot1_base_scan', LaserScan, timeout=5)
            except:
                pass

        #print("Data: ", data)

        state, done, chair_collision, chair_get_close = self.getState(data)
        reward = self.setReward(state, done, chair_collision, chair_get_close , action)
        return np.asarray(state), reward, done

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
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False

        self.goal_distance = self.getDistance(self.goal_x, self.position_r1.x, self.goal_y, self.position_r1.y)
        state, _, _ , _= self.getState(data)

        return np.asarray(state)
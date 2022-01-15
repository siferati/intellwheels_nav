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
from tools.sample_scan import SampleScan

class Env():
    def __init__(self, action_size):
        
        self.action_size = action_size
        self.initGoal = True
        self.get_goal = False
        
        # pose of robot1
        self.pose_r1 = Pose()
        self.pose_r1.position.y = 0.0
        self.pose_r1.position.z = 0.0
        self.pose_r1.orientation.x = 0.0
        self.pose_r1.orientation.y = 0.0
        self.pose_r1.orientation.z = 0.0
        self.pose_r1.orientation.w = 1.0

        self.heading_r2 = 0
        self.pose_r2 = Pose()
        self.pose_r2.position.x = 0.0
        self.pose_r2.position.y = 0.0
        self.pose_r2.position.z = 0.0
        self.pose_r2.orientation.x = 0.0
        self.pose_r2.orientation.y = 0.0
        self.pose_r2.orientation.z = 0.0
        self.pose_r2.orientation.w = 1.0

        self.distance_to_chair1 = 1000;
        
        self.pub_cmd_vel_r2 = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=5)
        
        self.sub_odom_r1 = rospy.Subscriber('/robot1/odom', Odometry, self.getOdometryRobot1)
        self.sub_odom_r2 = rospy.Subscriber('/robot2/odom', Odometry, self.getOdometryRobot2)

        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

        self.sample_scan = SampleScan(10)
        

    def getDistance(self, pos_x, pos_y, goal_x, goal_y):
        #goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        return round(math.hypot(goal_x - pos_x, goal_y - pos_y), 2)

    def getOdometryRobot1(self, odom):
        self.pose_r1 = odom.pose.pose

    def getOdometryRobot2(self, odom):
        if self.pose_r2 == odom.pose.pose:
            self.heading_r2 = - self.heading_r2
        else:
            self.pose_r2 == odom.pose.pose    
            self.heading_r2 = self.getHeadingToRobot1(odom)

    def getHeadingToRobot1(self, odom):
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.pose_r1.position.y - position.y, self.pose_r1.position.x - position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        heading = round(heading, 2)

        return heading


    def getState(self, scan):
        scan_range = []
        heading = self.heading_r2
        min_range = 0.5
        min_distance_to_chair = 1.4
        max_safe_zone = 1.6
        collision = False
        
        scan_range = self.sample_scan.clean_data(scan)

        if min_range > min(scan_range) > 0:
            rospy.loginfo("Collision !!")
            collision = True
               
        chair_safe_zone = False
        current_distance_to_chair1 = self.getDistance(self.pose_r1.position.x, self.pose_r2.position.x, self.pose_r1.position.y, self.pose_r2.position.y) 
        
        if current_distance_to_chair1 < min_distance_to_chair:
            self.get_goal = True

        if(current_distance_to_chair1 < self.distance_to_chair1):
            self.distance_to_chair1 = current_distance_to_chair1
            if(self.distance_to_chair1 >= min_distance_to_chair and self.distance_to_chair1 <= max_safe_zone):
                chair_safe_zone = True
        
        rospy.loginfo("[ %f - %f ] and [ %f - %f ]  Dst chair 1 = %f .... safe[ %f - %f ]",
                        self.pose_r1.position.x,
                        self.pose_r1.position.y,
                        self.pose_r2.position.x,
                        self.pose_r2.position.y,
                        current_distance_to_chair1, 
                        min_distance_to_chair, 
                        max_safe_zone )
        
        return scan_range + [heading, current_distance_to_chair1], collision, chair_safe_zone

    # reward the actions where the heading points to the target
    def heading_reward(self, heading):
        yaw_reward = []
        for i in range(self.action_size):
            angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            yaw_reward.append(tr)

        return yaw_reward

    def setReward(self, state, collision, chair_safe_zone ,action):
        yaw_reward = []
        current_distance = state[-1]
        heading = state[-2]

        reward = 0
        if(self.goal_distance != 0):

            yaw_reward = self.heading_reward(heading)

            distance_rate = 2 ** (current_distance / self.goal_distance)
            reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate)

            if collision:
                rospy.loginfo("Collision with an object stop the chair !!")
                reward = -100
                self.pub_cmd_vel_r2.publish(Twist()) # stop
            
            #if chair_collision:
            #    #rospy.loginfo("Possible chair collision!!")
            #    reward = -100
            #    self.pub_cmd_vel_r2.publish(Twist()) # stop
            
            if chair_safe_zone:
                rospy.loginfo("Chair2 is in safe zone related to Chair1")
                reward = 10
                #self.pub_cmd_vel_r2.publish(Twist()) #Todo: maybe should not stop             
            

            if self.get_goal:
                rospy.loginfo("Find chair")
                reward = 100
                #self.pub_cmd_vel_r2.publish(Twist()) #stop
                self.goal_distance = self.getDistance(self.pose_r1.position.x, self.pose_r2.position.x, self.pose_r1.position.y, self.pose_r2.position.y)
                self.get_goal = False

        return reward
    
    
    def step(self, action):
        
        max_angular_vel = 1.5

        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15
        vel_cmd.angular.z = ang_vel

        # mimic the movement
        self.pub_cmd_vel_r2.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('robot2_base_scan', LaserScan, timeout=5)
            except:
                pass

        data = self.sample_scan.get_sample_from_laser_scan(data.ranges)
        state, collision, chair_safe_zone = self.getState(data)

        if(collision or self.get_goal):
            self.get_goal = False

        reward = self.setReward(state, collision, chair_safe_zone, action)

        return np.asarray(state), reward, collision, self.get_goal

    def reset(self):
        
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('robot2_base_scan', LaserScan, timeout=50)
            except:
                pass

        if self.initGoal:
            self.initGoal = False

        data = self.sample_scan.get_sample_from_laser_scan(data.ranges)
        self.goal_distance = self.getDistance(self.pose_r1.position.x, self.pose_r2.position.x, self.pose_r1.position.y, self.pose_r2.position.y)
        state, _, _ = self.getState(data)

        return np.asarray(state)
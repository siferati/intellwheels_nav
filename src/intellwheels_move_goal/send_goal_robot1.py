#!/usr/bin/env python


import rospy
import actionlib
from multiprocessing import Process
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def movebase_client(topic, x, y):

    client = actionlib.SimpleActionClient(topic,MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


def send_goals(node, topic, x, y):
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client(topic, x, y)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


if __name__ == '__main__':
    node = rospy.get_param('send_goal_robot1/node')
    topic = rospy.get_param('send_goal_robot1/topic')
    x_pos = rospy.get_param('send_goal_robot1/x')
    y_pos = rospy.get_param('send_goal_robot1/y')
    send_goals(node, topic, x_pos, y_pos)
    
    
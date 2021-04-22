#include "../include/OdomTwister.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

OdomTwister::OdomTwister(ros::NodeHandle& nh)
: 	pub(nh.advertise<nav_msgs::Odometry>("odom_twisted", 10)),
	sub(nh.subscribe("odom", 10, &OdomTwister::callback, this))
{ }

double OdomTwister::quaternion_msg_to_yaw(const geometry_msgs::Quaternion& msg)
{
	tf2::Quaternion q;
	tf2::fromMsg(msg, q);
	tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
	return yaw;
}

void OdomTwister::callback(const nav_msgs::OdometryPtr& msg)
{	
	// calcs
	double dt = (msg->header.stamp - this->prev_stamp).toSec();
	double ori = this->quaternion_msg_to_yaw(msg->pose.pose.orientation);
	double prev_dist = tf2::Vector3(this->prev_pos.x, this->prev_pos.y, this->prev_pos.z).length();
	double dist = tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z).length();

	// add twist to msg (always assume it's going forward)
	msg->twist.twist.linear.x = abs((dist - prev_dist) / dt);
	msg->twist.twist.angular.z = (ori - this->prev_ori) / dt;

	// update
	this->prev_pos = msg->pose.pose.position;
	this->prev_ori = ori;
	this->prev_stamp = msg->header.stamp;

	pub.publish(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_twister");
	ros::NodeHandle nh;
	OdomTwister twister(nh);
	ros::spin();

	return 0;
}
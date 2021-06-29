#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
 
class OdomTwister
{
public:
	OdomTwister(ros::NodeHandle& nh);

private:
	/** Subscriber to odom topic. */
	const ros::Subscriber sub;

	/** Publisher of twisted odom topic. */
	const ros::Publisher pub;

	/** Position of the last received message. */
	geometry_msgs::Point prev_pos;

	/** Orientation (yaw) of the last received message. */
	double prev_ori;

	/** Timestamp of the last received message. */
	ros::Time prev_stamp;

	/** Callback of odom message. */
	void callback(const nav_msgs::OdometryPtr& msg);

	/** Converts the given quaternation to yaw. */
	double quaternion_msg_to_yaw(const geometry_msgs::Quaternion& quaternion);
};
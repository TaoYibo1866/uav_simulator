#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

using sensor_msgs::Joy;
using geometry_msgs::Twist;

ros::Publisher cmd_pub;

void joyCb(Joy msg)
{
	// F710 Direct Input Mode
	Twist cmd;
	cmd.linear.x = 5.0 * msg.axes[3];
	cmd.linear.y = 5.0 * msg.axes[2];
  cmd.linear.z = 5.0 * msg.axes[1];
	cmd.angular.z = 90 * msg.axes[0] * M_PI / 180.0;
	cmd_pub.publish(cmd); 
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "m100_joystick");
	ros::NodeHandle nh;
	ros::Subscriber joy_sub = nh.subscribe<Joy>("joy", 1, joyCb);
	cmd_pub = nh.advertise<Twist>("cmd_vel", 1);
	ros::spin();
	return 0;
}
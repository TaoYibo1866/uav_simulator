#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

#define STATE_PERIOD 0.002

using sensor_msgs::Joy;
using nav_msgs::Odometry;

class JoyControl
{
public:
  JoyControl();
  ~JoyControl();
private:
  void joyCb(Joy msg);
  void stateCb(Odometry msg);
private:
	ros::Publisher cmd_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber state_sub_;
  Eigen::Vector3d cmd_pos_;
  Eigen::Vector3d cmd_vel_b_;
  double cmd_yawrate_;
  double cmd_yaw_;
};

JoyControl::JoyControl()
{
  ros::NodeHandle nh;
  cmd_pub_ = nh.advertise<Joy>("trajectory", 1);
  joy_sub_ = nh.subscribe<Joy>("joy", 1, &JoyControl::joyCb, this);
  state_sub_ = nh.subscribe<Odometry>("state", 1, &JoyControl::stateCb, this);
  cmd_pos_ = Eigen::Vector3d::Zero();
  cmd_vel_b_ = Eigen::Vector3d::Zero();
  cmd_yaw_ = 0;
  cmd_yawrate_ = 0;
}

JoyControl::~JoyControl() {}

void JoyControl::joyCb(Joy msg)
{
  cmd_vel_b_(0) = 5.0 * msg.axes[3];
	cmd_vel_b_(1) = 5.0 * msg.axes[2];
  cmd_vel_b_(2) = 5.0 * msg.axes[1];
	cmd_yawrate_ = 180 * msg.axes[0] * M_PI / 180.0;
}

void JoyControl::stateCb(Odometry msg)
{
  static bool initialized = false;

  Eigen::Quaterniond q(msg.pose.pose.orientation.w,
                         msg.pose.pose.orientation.x,
                         msg.pose.pose.orientation.y,
                         msg.pose.pose.orientation.z);
  Eigen::Matrix3d R(q);
  
  Eigen::Vector3d b1c, b2c, b3c;
  Eigen::Vector3d b1 = R.col(0);
  b3c = Eigen::Vector3d::UnitZ();
  b2c = b3c.cross(b1).normalized();
  b1c = b2c.cross(b3c).normalized();
  Eigen::Matrix3d Rc;
  Rc << b1c, b2c, b3c;

  if (!initialized)
  {
    initialized = true;
    cmd_pos_(0) = msg.pose.pose.position.x;
    cmd_pos_(1) = msg.pose.pose.position.y;
    cmd_pos_(2) = msg.pose.pose.position.z;
    cmd_yaw_ = atan2(b1c(1), b1c(0));
  }

  Eigen::Vector3d cmd_vel = Rc * cmd_vel_b_;
  cmd_pos_ = cmd_pos_ + cmd_vel * STATE_PERIOD;
  cmd_yaw_ = cmd_yaw_ + cmd_yawrate_ * STATE_PERIOD;
  static Eigen::Vector3d prev_cmd_vel = cmd_vel;
  Eigen::Vector3d cmd_acc = (cmd_vel - prev_cmd_vel) / STATE_PERIOD;
  prev_cmd_vel = cmd_vel;

  Joy cmd;
  cmd.axes.push_back(cmd_pos_(0));
  cmd.axes.push_back(cmd_pos_(1));
  cmd.axes.push_back(cmd_pos_(2));

  cmd.axes.push_back(cmd_vel(0));
  cmd.axes.push_back(cmd_vel(1));
  cmd.axes.push_back(cmd_vel(2));

  cmd.axes.push_back(cmd_acc(0));
  cmd.axes.push_back(cmd_acc(1));
  cmd.axes.push_back(cmd_acc(2));

  cmd.axes.push_back(cmd_yaw_);

  cmd_pub_.publish(cmd);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "joy_control");
	JoyControl joy_control;
  ros::spin();
	return 0;
}
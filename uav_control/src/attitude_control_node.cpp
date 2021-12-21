#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>

#define J11 0.01
#define J22 0.01
#define J33 0.016
#define J12 0
#define J13 0
#define J23 0
#define KR1 3
#define KR2 3
#define KR3 1
#define KOM1 0.5
#define KOM2 0.5
#define KOM3 0.2
#define POSITIVE(x) (x < 0 ? 0 : x)

using geometry_msgs::Twist;
using geometry_msgs::Pose;
using nav_msgs::Odometry;

class AttitudeControl
{
public:
  AttitudeControl();
  ~AttitudeControl();
private:
  void cmdCb(Pose msg);
  void stateCb(Odometry msg);
private:
  Eigen::Matrix3d Rd_;
  Eigen::Vector3d fd_;
  ros::Subscriber state_sub_;
  ros::Subscriber cmd_sub_;
  ros::Publisher ctrl_pub_;
};

AttitudeControl::AttitudeControl()
{
  ros::NodeHandle nh;
  state_sub_ = nh.subscribe<Odometry>("state", 1, &AttitudeControl::stateCb, this);
  cmd_sub_ = nh.subscribe<Pose>("attitude_force", 1, &AttitudeControl::cmdCb, this);
  ctrl_pub_ = nh.advertise<Twist>("actuation", 1);
  Rd_ = Eigen::Matrix3d::Identity();
  fd_ = Eigen::Vector3d::Zero();
}

AttitudeControl::~AttitudeControl() {}

void AttitudeControl::cmdCb(Pose msg)
{
  Eigen::Quaterniond qd(msg.orientation.w,
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z);
  Rd_ = qd.matrix();
  fd_(0) = msg.position.x;
  fd_(1) = msg.position.y;
  fd_(2) = msg.position.z;
}

void AttitudeControl::stateCb(Odometry msg)
{
  Eigen::Quaterniond q(msg.pose.pose.orientation.w,
                       msg.pose.pose.orientation.x, 
                       msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z);
  Eigen::Matrix3d R  = q.matrix();

  double R11 = R(0, 0);
  double R12 = R(0, 1);
  double R13 = R(0, 2);
  double R21 = R(1, 0);
  double R22 = R(1, 1);
  double R23 = R(1, 2);
  double R31 = R(2, 0);
  double R32 = R(2, 1);
  double R33 = R(2, 2);

  double Rd11 = Rd_(0, 0);
  double Rd12 = Rd_(0, 1);
  double Rd13 = Rd_(0, 2);
  double Rd21 = Rd_(1, 0);
  double Rd22 = Rd_(1, 1);
  double Rd23 = Rd_(1, 2);
  double Rd31 = Rd_(2, 0);
  double Rd32 = Rd_(2, 1);
  double Rd33 = Rd_(2, 2);

  double thrust = POSITIVE(R13 * fd_(0) + R23 * fd_(1) + R33 * fd_(2));
  double psi = 0.5 * (3.0 - (Rd11 * R11 + Rd21 * R21 + Rd31 * R31 + 
                             Rd12 * R12 + Rd22 * R22 + Rd32 * R32 +
                             Rd13 * R13 + Rd23 * R23 + Rd33 * R33));
  if (psi > 1)
  {
    ROS_WARN("psi > 1, value = %lf", psi);
  }

  double eR1 = 0.5 * (R12 * Rd13 - R13 * Rd12 + R22 * Rd23 - R23 * Rd22 + R32 * Rd33 - R33 * Rd32);
  double eR2 = 0.5 * (R13 * Rd11 - R11 * Rd13 + R23 * Rd21 - R21 * Rd23 + R33 * Rd31 - R31 * Rd33);
  double eR3 = 0.5 * (R11 * Rd12 - R12 * Rd11 + R21 * Rd22 - R22 * Rd21 + R31 * Rd32 - R32 * Rd31);

  double Om1 = msg.twist.twist.angular.x;
  double Om2 = msg.twist.twist.angular.y;
  double Om3 = msg.twist.twist.angular.z;

  double eOm1 = Om1;
  double eOm2 = Om2;
  double eOm3 = Om3;

  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  J(0, 0) = J11;
  J(0, 1) = J12;
  J(0, 2) = J13;
  J(1, 0) = J12;
  J(1, 1) = J22;
  J(1, 2) = J23;
  J(2, 0) = J13;
  J(2, 1) = J23;
  J(2, 2) = J33;
  Eigen::Vector3d Om;
  Om(0) = Om1;
  Om(1) = Om2;
  Om(2) = Om3;
  Eigen::Vector3d comp = Om.cross(J * Om);

  Twist ctrl;
  ctrl.linear.z = thrust;
  ctrl.angular.x = -KR1   * eR1 - KOM1 * eOm1 + comp(0);
  ctrl.angular.y = -KR2   * eR2 - KOM2 * eOm2 + comp(1);
  ctrl.angular.z = -KR3   * eR3 - KOM3 * eOm3 + comp(2);

  ctrl_pub_.publish(ctrl);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "attitude_control");
  AttitudeControl attitude_control;
  ros::spin();
  return 0;
}
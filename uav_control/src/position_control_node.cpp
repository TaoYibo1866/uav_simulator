#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include <Eigen/Eigen>

#define KX1 8
#define KX2 8
#define KX3 17
#define KV1 6
#define KV2 6
#define KV3 10
#define MASS 2.70703
#define G 9.8
#define MAX_TILT M_PI / 4

using nav_msgs::Odometry;
using geometry_msgs::Pose;
using sensor_msgs::Joy;

class PositionControl
{
public:
  PositionControl();
  ~PositionControl();
private:
  void stateCb(Odometry msg);
  void cmdCb(Joy msg);
private:
  ros::Publisher ctrl_pub_;
  ros::Subscriber cmd_sub;
  ros::Subscriber state_sub_;
  double des_yaw_;
  Eigen::Vector3d des_pos_;
  Eigen::Vector3d des_vel_;
  Eigen::Vector3d des_acc_;
};

PositionControl::PositionControl()
{
  ros::NodeHandle nh;
  ctrl_pub_ = nh.advertise<Pose>("attitude_force", 1);
  cmd_sub = nh.subscribe<Joy>("trajectory", 1, &PositionControl::cmdCb, this, ros::TransportHints().tcpNoDelay());
  state_sub_ = nh.subscribe<Odometry>("state", 1, &PositionControl::stateCb, this, ros::TransportHints().tcpNoDelay());
  des_yaw_ = 0;
  des_pos_ = Eigen::Vector3d::Zero();
  des_vel_ = Eigen::Vector3d::Zero();
  des_acc_ = Eigen::Vector3d::Zero();
}

PositionControl::~PositionControl() {}

void PositionControl::cmdCb(Joy msg)
{
  if (msg.axes.size() != 10)
  {
    ROS_WARN("axes.size != 10");
    return;
  }

  des_yaw_ = msg.axes[9];
  des_pos_.x() = msg.axes[0];
  des_pos_.y() = msg.axes[1];
  des_pos_.z() = msg.axes[2];
  des_vel_.x() = msg.axes[3];
  des_vel_.y() = msg.axes[4];
  des_vel_.z() = msg.axes[5];
  des_acc_.x() = msg.axes[6];
  des_acc_.y() = msg.axes[7];
  des_acc_.z() = msg.axes[8];
}

void PositionControl::stateCb(Odometry msg)
{
  Eigen::Vector3d pos(msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      msg.pose.pose.position.z);
  Eigen::Vector3d vel(msg.twist.twist.linear.x,
                      msg.twist.twist.linear.y,
                      msg.twist.twist.linear.z);
  Eigen::Vector3d pos_err = des_pos_ - pos;
  Eigen::Vector3d vel_err = des_vel_ - vel;
  Eigen::Vector3d kx(KX1, KX2, KX3);
  Eigen::Vector3d kv(KV1, KV2, KV3);
  Eigen::Vector3d force = kx.asDiagonal() * pos_err + kv.asDiagonal() * vel_err + MASS * des_acc_ + MASS * G * Eigen::Vector3d::UnitZ();

  // limit tilt angle to MAX_TILT
  double c = cos(MAX_TILT);
  if (Eigen::Vector3d(0, 0, 1).dot(force.normalized()) < c) {
    Eigen::Vector3d f = force - MASS * G * Eigen::Vector3d::UnitZ();
    double nf = f.norm();
    double A = c * c * nf * nf - f(2) * f(2);
    double B = 2 * (c * c - 1) * f(2) * MASS * G;
    double C = (c * c - 1) * MASS * MASS * G * G;
    double s = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
    force = s * f + MASS * G * Eigen::Vector3d::UnitZ();
  }

  Eigen::Vector3d b1c, b2c, b3c;
  Eigen::Vector3d b1d(cos(des_yaw_), sin(des_yaw_), 0);
  if (force.norm() < 1e-7)
    b3c = Eigen::Vector3d::UnitZ();
  else
    b3c = force.normalized();
  b2c = b3c.cross(b1d).normalized();
  b1c = b2c.cross(b3c).normalized();
  Eigen::Matrix3d Rc;
  Rc << b1c, b2c, b3c;
  Eigen::Quaterniond qc(Rc);

  Pose ctrl;
  ctrl.position.x = force(0);
  ctrl.position.y = force(1);
  ctrl.position.z = force(2);
  ctrl.orientation.x = qc.x();
  ctrl.orientation.y = qc.y();
  ctrl.orientation.z = qc.z();
  ctrl.orientation.w = qc.w();
  ctrl_pub_.publish(ctrl);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_control");
  PositionControl position_control;
  ros::spin();
  return 0;
}
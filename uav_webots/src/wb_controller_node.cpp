#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/Twist.h>
#include <webots/Robot.hpp>
#include <webots/Gyro.hpp>
#include <webots/GPS.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Eigen>

using namespace webots;
using namespace Eigen;
using rosgraph_msgs::Clock;
using nav_msgs::Odometry;
using geometry_msgs::Twist;

#define STATE_PERIOD_MS 2
#define KF           2.55e-5 //  N  per rad/s
#define KM           5.1e-7  //  Nm per rad/s
#define ARM_LENGTH   0.22978 //  m
#define POSITIVE(x) x < 0 ? 0 : x

void cmdCb(Twist::ConstPtr msg, Motor* motor1, Motor* motor2, Motor* motor3, Motor* motor4)
{
  double f = msg->linear.z;
  double M1 = msg->angular.x;
  double M2 = msg->angular.y;
  double M3 = msg->angular.z;
  M1 = M1 / ARM_LENGTH;
  M2 = M2 / ARM_LENGTH;
  M3 = M3 / (KM / KF);
  double f1 = POSITIVE(0.25 * (f - M1 - M2 - M3));
  double f2 = POSITIVE(0.25 * (f + M1 - M2 + M3));
  double f3 = POSITIVE(0.25 * (f + M1 + M2 - M3));
  double f4 = POSITIVE(0.25 * (f - M1 + M2 + M3));
  double w1 =  sqrt(f1 / KF);
  double w2 = -sqrt(f2 / KF);
  double w3 =  sqrt(f3 / KF);
  double w4 = -sqrt(f4 / KF);
  motor1->setVelocity(w1);
  motor2->setVelocity(w2);
  motor3->setVelocity(w3);
  motor4->setVelocity(w4);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wb_controller");

  // webots setup
  Robot* robot = new Robot;
  int timestep = (int)robot->getBasicTimeStep();
  Motor* motor1 = robot->getMotor("front_right_motor");
  Motor* motor2 = robot->getMotor("front_left_motor");
  Motor* motor3 = robot->getMotor("rear_left_motor");
  Motor* motor4 = robot->getMotor("rear_right_motor");
  motor1->setPosition(INFINITY);
  motor2->setPosition(INFINITY);
  motor3->setPosition(INFINITY);
  motor4->setPosition(INFINITY);
  motor1->setVelocity(0);
  motor2->setVelocity(0);
  motor3->setVelocity(0);
  motor4->setVelocity(0);
  GPS* gps = robot->getGPS("gps");
  gps->enable(STATE_PERIOD_MS);
  Gyro* gyro = robot->getGyro("gyro");
  gyro->enable(STATE_PERIOD_MS);
  InertialUnit* imu = robot->getInertialUnit("inertial unit");
  imu->enable(STATE_PERIOD_MS);

  // ros setup
  ros::NodeHandle nh;
  ros::Publisher clock_pub = nh.advertise<Clock>("/clock", 1);
  ros::Publisher state_pub = nh.advertise<Odometry>("state", 1);
  ros::Subscriber cmd_sub = nh.subscribe<Twist>("actuation", 1, boost::bind(cmdCb, _1, motor1, motor2, motor3, motor4));

  while (ros::ok() && robot->step(timestep) != -1)
  {
    double t = robot->getTime();
    Clock clock;
    clock.clock.fromSec(t);
    clock_pub.publish(clock);

    if ((int)(t * 1000) % STATE_PERIOD_MS == 0)
    {
      const double* gps_data = gps->getValues();
      const double* imu_data = imu->getQuaternion();
      const double* gyro_data = gyro->getValues();
      double x =  gps_data[0];
      double y = -gps_data[2];
      double z =  gps_data[1];
      static double prev_x = x;
      static double prev_y = y;
      static double prev_z = z;
      double vx = (x - prev_x) * 1e3 / timestep;
      double vy = (y - prev_y) * 1e3 / timestep;
      double vz = (z - prev_z) * 1e3 / timestep;
      prev_x = x;
      prev_y = y;
      prev_z = z;
      double Om1 = gyro_data[0];
      double Om2 = gyro_data[1];
      double Om3 = gyro_data[2];

      Quaterniond q = Quaterniond(cos(M_PI / 4), sin(M_PI / 4), 0, 0) * Quaterniond(imu_data[3], imu_data[0], imu_data[1], imu_data[2]);

      Odometry state;
      state.header.stamp = clock.clock;
      state.pose.pose.position.x = x;
      state.pose.pose.position.y = y;
      state.pose.pose.position.z = z;
      state.pose.pose.orientation.x = q.x();
      state.pose.pose.orientation.y = q.y();
      state.pose.pose.orientation.z = q.z();
      state.pose.pose.orientation.w = q.w();
      state.twist.twist.linear.x = vx;
      state.twist.twist.linear.y = vy;
      state.twist.twist.linear.z = vz;
      state.twist.twist.angular.x = Om1;
      state.twist.twist.angular.y = Om2;
      state.twist.twist.angular.z = Om3;
      state_pub.publish(state);
    }
    
    ros::spinOnce();
  }
  delete robot;
  return 0;
}
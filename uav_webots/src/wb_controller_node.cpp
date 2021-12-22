#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/Joy.h>
#include <webots/Robot.hpp>
#include <webots/Gyro.hpp>
#include <webots/GPS.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/LED.hpp>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Eigen>

using namespace webots;
using namespace Eigen;
using rosgraph_msgs::Clock;
using nav_msgs::Odometry;
using sensor_msgs::Joy;

#define STATE_PERIOD_MS 2

void cmdCb(Joy::ConstPtr msg, Motor* motor1, Motor* motor2, Motor* motor3, Motor* motor4)
{
  if (msg->axes.size() != 4)
  {
    ROS_WARN("axes.size() != 4");
    return;
  }
  motor1->setVelocity(msg->axes[0]);
  motor2->setVelocity(msg->axes[1]);
  motor3->setVelocity(msg->axes[2]);
  motor4->setVelocity(msg->axes[3]);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wb_controller");
  int publish_clock = 1;
  if (argc == 2)
    publish_clock = atoi(argv[1]);
  ROS_INFO("publish_clock = %d", publish_clock);

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
  LED* led = robot->getLED("led");
  led->set(1);

  // ros setup
  ros::NodeHandle nh;
  ros::Publisher clock_pub = nh.advertise<Clock>("/clock", 1);
  ros::Publisher state_pub = nh.advertise<Odometry>("state", 1);
  ros::Subscriber cmd_sub = nh.subscribe<Joy>("actuation", 1, boost::bind(cmdCb, _1, motor1, motor2, motor3, motor4), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

  while (ros::ok() && robot->step(timestep) != -1)
  {
    double t = robot->getTime();
    Clock clock;
    clock.clock.fromSec(t);
    if (publish_clock)
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
      double vx = (x - prev_x) * 1e3 / STATE_PERIOD_MS;
      double vy = (y - prev_y) * 1e3 / STATE_PERIOD_MS;
      double vz = (z - prev_z) * 1e3 / STATE_PERIOD_MS;
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
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <webots/Robot.hpp>
#include <webots/Gyro.hpp>
#include <webots/GPS.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace webots;
using namespace rosgraph_msgs;
using namespace nav_msgs;

int main(int argc, char** argv)
{
  // ros setup
  ros::init(argc, argv, "wb_m100_controller");
  ros::NodeHandle nh;
  ros::Publisher clock_pub = nh.advertise<Clock>("/clock", 1);

  // webots setup
  Robot* robot = new Robot;
  int timestep = (int)robot->getBasicTimeStep();
  Gyro* gyro = robot->getGyro("gyro");
  gyro->enable(timestep);
  GPS* gps = robot->getGPS("gps");
  gps->enable(timestep);
  InertialUnit* imu = robot->getInertialUnit("inertial unit");
  imu->enable(timestep);
  Motor* motor1 = robot->getMotor("front_right_motor");
  Motor* motor2 = robot->getMotor("front_left_motor");
  Motor* motor3 = robot->getMotor("rear_left_motor");
  Motor* motor4 = robot->getMotor("rear_right_motor");
  motor1->setPosition(INFINITY);
  motor2->setPosition(INFINITY);
  motor3->setPosition(INFINITY);
  motor4->setPosition(INFINITY);

  while (ros::ok() && robot->step(timestep) != -1)
  {
    Clock clock;
    clock.clock.fromSec(robot->getTime());
    clock_pub.publish(clock);

    double t = robot->getTime();
    const double* gps_data = gps->getValues();
    const double* imu_data = imu->getQuaternion();
    const double* gyro_data = gyro->getValues();
    tf2::Transform pose;
    pose.setOrigin(tf2::Vector3(gps_data[0], -gps_data[2], gps_data[1]));
    pose.setRotation(tf2::Quaternion(cos(M_PI / 4) , 0             , 0             , sin(M_PI / 4) )
                  * tf2::Quaternion(imu_data[0], imu_data[1], imu_data[2], imu_data[3]));
    // x: location
    tf2::Vector3 x(gps_data[0], -gps_data[2], gps_data[1]);
    // q, R: rotation
    tf2::Quaternion q = tf2::Quaternion(cos(M_PI / 4) , 0             , 0             , sin(M_PI / 4) )
                      * tf2::Quaternion(imu_data[0], imu_data[1], imu_data[2], imu_data[3]);
    tf2::Matrix3x3 R(q);
    // RPY
    double roll, pitch, yaw;
    R.getRPY(roll, pitch, yaw);
    // omega: angular velocity in the body-fixed frame
    tf2::Vector3 omega(gyro_data[0], gyro_data[1], gyro_data[2]);

    ROS_INFO("position: x=%.1lf, y=%.1lf, z=%.1lf", x.x(), x.y(), x.z());
    ROS_INFO("orientation: roll=%.1lf, pitch=%.1lf, yaw=%.1lf", roll, pitch, yaw);
    ROS_INFO("angular_velocity: x=%.1lf, y=%.1lf, z=%.1lf", omega.x(), omega.y(), omega.z());

    motor1->setVelocity(490);
    motor2->setVelocity(-489);
    motor3->setVelocity(490);
    motor4->setVelocity(-489);
    ros::spinOnce();
  }
  delete robot;
  return 0;
}
//
// 里程计：用四个轮子的实际转速反推底盘速度，积分成位姿，发布 /odom 和 tf。
//
// 对应考核要求 6「使用正运动学实现里程计」。
//

#ifndef HERO_CHASSIS_CONTROLLER_CHASSIS_ODOMETRY_H_
#define HERO_CHASSIS_CONTROLLER_CHASSIS_ODOMETRY_H_

#include <array>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include "hero_chassis_controller/mecanum_kinematics.h"

namespace hero_chassis_controller {

class ChassisOdometry {
 public:
  ChassisOdometry(const MecanumKinematics &kinematics, ros::NodeHandle &controller_nh);

  // 每个控制周期调用一次。wheel_velocity 是四个轮子的实测角速度（rad/s）。
  void Update(const std::array<double, kWheelCount> &wheel_velocity, const ros::Time &time,
              const ros::Duration &period);

  // 把里程计清零。换仿真世界或重定位时调用。
  void Reset();

  double x() const { return x_; }
  double y() const { return y_; }
  double yaw() const { return yaw_; }

 private:
  void Publish(const ros::Time &time, double vx, double vy, double wz);

  MecanumKinematics kinematics_;

  ros::Publisher odom_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};

  // 二维位姿。麦轮底盘在平面上运动，轮子里程计推不出高度，所以只维护这三个量。
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  nav_msgs::Odometry odom_msg_;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_CHASSIS_ODOMETRY_H_

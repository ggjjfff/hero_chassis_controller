// Copyright 2026 Guo Jiafeng
//
// Licensed under the BSD-3-Clause License (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://opensource.org/licenses/BSD-3-Clause
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hero_chassis_controller/chassis_odometry.h"

#include <cmath>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

namespace hero_chassis_controller {

ChassisOdometry::ChassisOdometry(const MecanumKinematics &kinematics, ros::NodeHandle &controller_nh)
    : kinematics_(kinematics) {
  controller_nh.param<std::string>("odom_frame", odom_frame_, "odom");
  controller_nh.param<std::string>("base_frame", base_frame_, "base_link");
  controller_nh.param<bool>("publish_tf", publish_tf_, true);

  // 队列深度取 1：里程计只关心最新值，攒着旧数据没有意义。
  // 话题名用绝对路径 "/odom"：controller_nh 是 /<controller_name>，
  // 相对名会变成 /<controller_name>/odom，而 nav 栈和 RViz 都看全局 /odom。
  odom_pub_ = controller_nh.advertise<nav_msgs::Odometry>("/odom", 1);

  odom_msg_.header.frame_id = odom_frame_;
  odom_msg_.child_frame_id = base_frame_;

  // 协方差：这里不建模打滑，给一个「比较确定」的量级即可。
  // 位姿协方差对角线上是 (x, y, z, roll, pitch, yaw) 的方差。
  odom_msg_.pose.covariance[0] = 0.01;
  odom_msg_.pose.covariance[7] = 0.01;
  odom_msg_.pose.covariance[14] = 1e6;  // z 不可观测，给大值表示「别信我」
  odom_msg_.pose.covariance[21] = 1e6;  // roll 同理
  odom_msg_.pose.covariance[28] = 1e6;  // pitch 同理
  odom_msg_.pose.covariance[35] = 0.05;

  odom_msg_.twist.covariance[0] = 0.01;
  odom_msg_.twist.covariance[7] = 0.01;
  odom_msg_.twist.covariance[14] = 1e6;
  odom_msg_.twist.covariance[21] = 1e6;
  odom_msg_.twist.covariance[28] = 1e6;
  odom_msg_.twist.covariance[35] = 0.05;
}

void ChassisOdometry::Update(const std::array<double, kWheelCount> &wheel_velocity, const ros::Time &time,
                             const ros::Duration &period) {
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  kinematics_.ForwardKinematics(wheel_velocity, &vx, &vy, &wz);

  // 关键点：正运动学给出的是【车体坐标系】下的速度，
  // 而位姿是【世界坐标系】下的。所以不能直接 x += vx * dt，
  // 必须先用当前的 yaw 把速度转到世界系。
  //
  // 用中点法（RK2）而不是欧拉法：先按半个周期后的航向角积分位移。
  // 原地转向的时候两者差别很小，但边走边转的时候中点法明显更准，
  // 代价只是多算一次三角函数。
  const double kDt = period.toSec();
  const double kYawMid = yaw_ + wz * kDt / 2.0;
  const double kCos = std::cos(kYawMid);
  const double kSin = std::sin(kYawMid);

  // 车体系 → 世界系：绕 z 轴旋转 yaw。
  //   world_x =  cos * vx - sin * vy
  //   world_y =  sin * vx + cos * vy
  // 麦轮有 vy，所以这一项不能省；差速底盘才可以省。
  x_ += (kCos * vx - kSin * vy) * kDt;
  y_ += (kSin * vx + kCos * vy) * kDt;
  yaw_ += wz * kDt;

  // 把 yaw 折到 (-pi, pi]，避免长时间运行后数值无限增长。
  if (yaw_ > M_PI) {
    yaw_ -= 2.0 * M_PI;
  } else if (yaw_ <= -M_PI) {
    yaw_ += 2.0 * M_PI;
  }

  Publish(time, vx, vy, wz);
}

void ChassisOdometry::Publish(const ros::Time &time, double vx, double vy, double wz) {
  odom_msg_.header.stamp = time;

  odom_msg_.pose.pose.position.x = x_;
  odom_msg_.pose.pose.position.y = y_;
  odom_msg_.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_);
  odom_msg_.pose.pose.orientation.x = q.x();
  odom_msg_.pose.pose.orientation.y = q.y();
  odom_msg_.pose.pose.orientation.z = q.z();
  odom_msg_.pose.pose.orientation.w = q.w();

  odom_msg_.twist.twist.linear.x = vx;
  odom_msg_.twist.twist.linear.y = vy;
  odom_msg_.twist.twist.linear.z = 0.0;
  odom_msg_.twist.twist.angular.x = 0.0;
  odom_msg_.twist.twist.angular.y = 0.0;
  odom_msg_.twist.twist.angular.z = wz;

  odom_pub_.publish(odom_msg_);

  if (!publish_tf_) {
    return;
  }
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = time;
  transform.header.frame_id = odom_frame_;
  transform.child_frame_id = base_frame_;
  transform.transform.translation.x = x_;
  transform.transform.translation.y = y_;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  tf_broadcaster_.sendTransform(transform);
}

void ChassisOdometry::Reset() {
  x_ = 0.0;
  y_ = 0.0;
  yaw_ = 0.0;
}

}  // namespace hero_chassis_controller

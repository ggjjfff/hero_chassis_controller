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

//
// 麦克纳姆轮底盘的正/逆运动学 —— 纯数学，不依赖 ROS。
//
// 之所以把它单独拎出来：运动学一旦混进控制器里，公式错、符号错、PID 错
// 三件事就缠在一起，只能靠「改一版、编译、起 Gazebo、看车乱走」来猜。
// 拆出来之后，公式对不对用单元测试 1 秒就能验证（见 test/test_kinematics.cpp）。
//

#ifndef HERO_CHASSIS_CONTROLLER_MECANUM_KINEMATICS_H_
#define HERO_CHASSIS_CONTROLLER_MECANUM_KINEMATICS_H_

#include <array>

namespace hero_chassis_controller {

// 四个轮子的固定顺序。全工程统一用这个下标，不要各处自己写 0/1/2/3。
enum WheelIndex : int {
  kLeftFront = 0,
  kRightFront = 1,
  kLeftBack = 2,
  kRightBack = 3,
  kWheelCount = 4,
};

class MecanumKinematics {
 public:
  // wheel_radius : 轮子半径 R，单位 m
  // wheel_base   : 前后轴距（x 方向），单位 m
  // wheel_track  : 左右轮距（y 方向），单位 m
  //
  // 参数取自 rm_description_for_task/urdf/hero/chassis.urdf.xacro：
  //   wheel_radius = 0.07625, wheel_base = 0.4, wheel_track = 0.4
  MecanumKinematics(double wheel_radius, double wheel_base, double wheel_track);

  // 逆运动学：底盘想要的速度 → 四个轮子的期望角速度（rad/s）
  //   vx : 前进速度 m/s，正 = 向车头方向
  //   vy : 横移速度 m/s，正 = 向车身左侧（ROS 约定 y 轴朝左）
  //   wz : 自转角速度 rad/s，正 = 逆时针（从上往下看）
  //
  // 返回顺序与 WheelIndex 一致。
  std::array<double, kWheelCount> InverseKinematics(double vx, double vy, double wz) const;

  // 正运动学：四个轮子的实际角速度 → 底盘速度。里程计用它。
  // 它是逆运动学的严格反函数（单元测试里用随机数验证了互逆性）。
  void ForwardKinematics(const std::array<double, kWheelCount> &wheel_velocity,
                         double *vx, double *vy, double *wz) const;

  double wheel_radius() const { return wheel_radius_; }
  double wheel_base() const { return wheel_base_; }
  double wheel_track() const { return wheel_track_; }

  // 麦轮公式里的几何常数 K = (wheel_base + wheel_track) / 2。
  // 它就是「从车心到某个轮子的曼哈顿距离」，即 |lx| + |ly|。
  double k() const { return k_; }

 private:
  double wheel_radius_;
  double wheel_base_;
  double wheel_track_;
  double k_;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_MECANUM_KINEMATICS_H_

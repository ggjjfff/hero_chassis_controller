#include "hero_chassis_controller/mecanum_kinematics.h"

#include <cmath>

namespace hero_chassis_controller {

MecanumKinematics::MecanumKinematics(double wheel_radius, double wheel_base, double wheel_track)
    : wheel_radius_(wheel_radius),
      wheel_base_(wheel_base),
      wheel_track_(wheel_track),
      k_((wheel_base + wheel_track) / 2.0) {}

std::array<double, kWheelCount> MecanumKinematics::InverseKinematics(double vx, double vy, double wz) const {
  // 辊子呈 45 度交叉的「X 型」布局，四个轮子的接地点对 vx / vy / wz 的敏感方向各自不同，
  // 所以出现了下面这组固定的正负号。推导见 doc/derivation.md。
  //
  //            vx    vy    wz
  //   LF      +1    -1    -K
  //   RF      +1    +1    +K
  //   LB      +1    +1    -K
  //   RB      +1    -1    +K
  //
  // 自检：设 vy=wz=0，四个轮子同速 → 直线前进，符合直觉。
  //       设 vx=wz=0、vy>0，LF 与 RB 反转、RF 与 LB 正转 → 横向左移。
  const double kOmega = k_ * wz;
  std::array<double, kWheelCount> wheel_velocity{};
  wheel_velocity[kLeftFront] = (vx - vy - kOmega) / wheel_radius_;
  wheel_velocity[kRightFront] = (vx + vy + kOmega) / wheel_radius_;
  wheel_velocity[kLeftBack] = (vx + vy - kOmega) / wheel_radius_;
  wheel_velocity[kRightBack] = (vx - vy + kOmega) / wheel_radius_;
  return wheel_velocity;
}

void MecanumKinematics::ForwardKinematics(const std::array<double, kWheelCount> &wheel_velocity,
                                          double *vx, double *vy, double *wz) const {
  if (vx == nullptr || vy == nullptr || wz == nullptr) {
    return;
  }
  // 上面那个 4x3 矩阵的最小二乘反解（这个矩阵的行两两正交，所以反解就是按列求和再除以 4）。
  const double kRadius = wheel_radius_;
  *vx = kRadius * (wheel_velocity[kLeftFront] + wheel_velocity[kRightFront] +
                   wheel_velocity[kLeftBack] + wheel_velocity[kRightBack]) /
        4.0;
  *vy = kRadius * (-wheel_velocity[kLeftFront] + wheel_velocity[kRightFront] +
                   wheel_velocity[kLeftBack] - wheel_velocity[kRightBack]) /
        4.0;
  *wz = kRadius * (-wheel_velocity[kLeftFront] + wheel_velocity[kRightFront] -
                   wheel_velocity[kLeftBack] + wheel_velocity[kRightBack]) /
       (4.0 * k_);
}

}  // namespace hero_chassis_controller

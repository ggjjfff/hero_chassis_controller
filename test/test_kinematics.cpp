// Copyright 2026 Hero Chassis Team
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
// 运动学单元测试
//
// 跑法（在工作区根目录）：
//     catkin build hero_chassis_controller --no-deps
//     ./devel/lib/hero_chassis_controller/test_mecanum_kinematics
// 或者：
//     catkin test hero_chassis_controller
//
// 为什么值得写这几十行：
//   运动学的错误有两类 —— 公式错、符号错。两者都会让车「乱走」，
//   但在 Gazebo 里看起来一模一样，只能靠反复试。
//   在单元测试里，两类错误各自有明确的断言，一次就能定位。
//

#include <cmath>

#include <gtest/gtest.h>

#include "hero_chassis_controller/mecanum_kinematics.h"

namespace {

using hero_chassis_controller::kLeftBack;
using hero_chassis_controller::kLeftFront;
using hero_chassis_controller::kRightBack;
using hero_chassis_controller::kRightFront;
using hero_chassis_controller::MecanumKinematics;

// 参数取自 rm_description_for_task/urdf/hero/chassis.urdf.xacro
constexpr double kRadius = 0.07625;
constexpr double kBase = 0.4;
constexpr double kTrack = 0.4;
// K = (0.4 + 0.4) / 2
constexpr double kK = 0.4;

// 浮点比较的容差。公式里只有加减乘除，误差应该远小于这个值。
constexpr double kEps = 1e-9;

class MecanumKinematicsTest : public ::testing::Test {
 protected:
  MecanumKinematics kinematics_{kRadius, kBase, kTrack};
};

// 几何常数 K 的定义：从车心到某个轮子的曼哈顿距离。
TEST_F(MecanumKinematicsTest, GeometricConstantIsHalfOfBasePlusTrack) {
  EXPECT_NEAR(kinematics_.k(), kK, kEps);
  EXPECT_NEAR(kinematics_.k(), (kBase + kTrack) / 2.0, kEps);
}

// 零指令必须给出零转速。否则遥控松开摇杆车还在爬，是很典型的一类 bug。
TEST_F(MecanumKinematicsTest, ZeroCommandGivesZeroWheelVelocity) {
  const auto w = kinematics_.InverseKinematics(0.0, 0.0, 0.0);
  for (int i = 0; i < hero_chassis_controller::kWheelCount; ++i) {
    EXPECT_NEAR(w[i], 0.0, kEps) << "wheel index " << i;
  }
}

// 纯前进：四个轮子同速同向，且 w = v / R。
TEST_F(MecanumKinematicsTest, PureForwardMakesAllWheelsSpinAlike) {
  const double kVx = 0.2;
  const auto w = kinematics_.InverseKinematics(kVx, 0.0, 0.0);
  const double kExpected = kVx / kRadius;

  EXPECT_GT(kExpected, 0.0);  // 正转 = 车往前，方向别搞反
  EXPECT_NEAR(w[kLeftFront], kExpected, kEps);
  EXPECT_NEAR(w[kRightFront], kExpected, kEps);
  EXPECT_NEAR(w[kLeftBack], kExpected, kEps);
  EXPECT_NEAR(w[kRightBack], kExpected, kEps);
}

// 纯后退：和前进只差一个负号。
TEST_F(MecanumKinematicsTest, PureBackwardIsNegativeOfForward) {
  const auto forward = kinematics_.InverseKinematics(0.2, 0.0, 0.0);
  const auto backward = kinematics_.InverseKinematics(-0.2, 0.0, 0.0);
  for (int i = 0; i < hero_chassis_controller::kWheelCount; ++i) {
    EXPECT_NEAR(backward[i], -forward[i], kEps) << "wheel index " << i;
  }
}

// 横向左移：这是麦克纳姆轮存在的理由，也是符号最容易错的地方。
// X 型布局下，LF 与 RB 同手性、RF 与 LB 同手性，两组方向相反。
TEST_F(MecanumKinematicsTest, StrafeLeftAlternatesWheelDirections) {
  const double kVy = 0.2;
  const auto w = kinematics_.InverseKinematics(0.0, kVy, 0.0);
  const double kMagnitude = kVy / kRadius;

  // 左移：LF 倒转、RF 正转、LB 正转、RB 倒转
  EXPECT_NEAR(w[kLeftFront], -kMagnitude, kEps);
  EXPECT_NEAR(w[kRightFront], kMagnitude, kEps);
  EXPECT_NEAR(w[kLeftBack], kMagnitude, kEps);
  EXPECT_NEAR(w[kRightBack], -kMagnitude, kEps);

  // 用一句话再验一遍这个模式：LF 与 RB 同号，且与 RF、LB 反号。
  EXPECT_NEAR(w[kLeftFront], w[kRightBack], kEps);
  EXPECT_NEAR(w[kRightFront], w[kLeftBack], kEps);
  EXPECT_NEAR(w[kLeftFront], -w[kRightFront], kEps);
}

// 原地逆时针：左侧两个轮子倒转、右侧两个正转，形成一对力偶。
TEST_F(MecanumKinematicsTest, SpinCounterClockwiseTurnsLeftSideBackward) {
  const double kWz = 0.5;
  const auto w = kinematics_.InverseKinematics(0.0, 0.0, kWz);
  const double kMagnitude = kK * kWz / kRadius;

  EXPECT_LT(w[kLeftFront], 0.0);   // 左侧后退
  EXPECT_LT(w[kLeftBack], 0.0);
  EXPECT_GT(w[kRightFront], 0.0);  // 右侧前进
  EXPECT_GT(w[kRightBack], 0.0);

  EXPECT_NEAR(w[kLeftFront], -kMagnitude, kEps);
  EXPECT_NEAR(w[kRightFront], kMagnitude, kEps);
}

// 原地顺时针：逆时针的镜像。
TEST_F(MecanumKinematicsTest, SpinClockwiseIsMirrorOfCounterClockwise) {
  const auto ccw = kinematics_.InverseKinematics(0.0, 0.0, 0.5);
  const auto cw = kinematics_.InverseKinematics(0.0, 0.0, -0.5);
  for (int i = 0; i < hero_chassis_controller::kWheelCount; ++i) {
    EXPECT_NEAR(cw[i], -ccw[i], kEps) << "wheel index " << i;
  }
}

// 正运动学必须是逆运动学的反函数。
// 这条一过，里程计算出来的速度就和指令对得上，不用再怀疑「odom 飘」是公式问题。
TEST_F(MecanumKinematicsTest, ForwardIsExactInverseOfInverse) {
  // 用确定的几组值，而不是随机数：测试失败时能直接看出是哪组参数出的问题。
  const double kCases[][3] = {
      {0.2, 0.0, 0.0},   {0.0, 0.2, 0.0},   {0.0, 0.0, 0.5},   {-0.3, 0.1, -0.4},
      {1.0, 1.0, 1.0},   {-2.0, 0.5, 3.0},  {0.0, 0.0, 0.0},   {1.5, -1.5, -2.5},
  };

  for (const auto &c : kCases) {
    const double kVx = c[0];
    const double kVy = c[1];
    const double kWz = c[2];

    const auto w = kinematics_.InverseKinematics(kVx, kVy, kWz);

    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    kinematics_.ForwardKinematics(w, &vx, &vy, &wz);

    EXPECT_NEAR(vx, kVx, 1e-8) << "vx round trip failed for input (" << kVx << ", " << kVy
                               << ", " << kWz << ")";
    EXPECT_NEAR(vy, kVy, 1e-8) << "vy round trip failed for input (" << kVx << ", " << kVy
                               << ", " << kWz << ")";
    EXPECT_NEAR(wz, kWz, 1e-8) << "wz round trip failed for input (" << kVx << ", " << kVy
                               << ", " << kWz << ")";
  }
}

// 单位 sanity check：1 m/s 前进时，轮子转 1/R rad/s。
// 换成 rpm 大约是 125 rpm，和真实底盘电机的量级对得上。
TEST_F(MecanumKinematicsTest, ForwardKinematicsReconstructsKnownSpeed) {
  const double kVx = 1.0;
  const auto w = kinematics_.InverseKinematics(kVx, 0.0, 0.0);

  // 手工验算：轮子每秒转 w/(2*pi) 圈，每圈走 2*pi*R，所以线速度 = w*R。
  EXPECT_NEAR(w[kLeftFront] * kRadius, kVx, kEps);

  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  kinematics_.ForwardKinematics(w, &vx, &vy, &wz);
  EXPECT_NEAR(vx, kVx, 1e-9);
  EXPECT_NEAR(vy, 0.0, kEps);
  EXPECT_NEAR(wz, 0.0, kEps);
}

// 传空指针不能崩。里程计里如果不小心传错，宁可什么都不做也不要段错误。
TEST_F(MecanumKinematicsTest, ForwardKinematicsToleratesNullPointers) {
  const auto w = kinematics_.InverseKinematics(0.1, 0.2, 0.3);
  double vx = 0.0;
  EXPECT_NO_THROW(kinematics_.ForwardKinematics(w, nullptr, nullptr, nullptr));
  EXPECT_NO_THROW(kinematics_.ForwardKinematics(w, &vx, nullptr, nullptr));
}

}  // namespace

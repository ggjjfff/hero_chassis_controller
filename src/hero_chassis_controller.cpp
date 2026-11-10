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

#include "hero_chassis_controller/hero_chassis_controller.h"

#include <algorithm>

#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace hero_chassis_controller {
namespace {

// 把 value 限制在 [low, high] 内。
double Clamp(double value, double low, double high) { return std::max(low, std::min(high, value)); }

}  // namespace

bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                 ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  if (!LoadJointHandles(effort_joint_interface, controller_nh)) {
    return false;
  }

  controller_nh.param("effort_limit", effort_limit_, 5.0);
  controller_nh.param("max_linear_velocity", max_linear_velocity_, 2.0);
  controller_nh.param("max_angular_velocity", max_angular_velocity_, 3.0);
  controller_nh.param<std::string>("velocity_mode", velocity_mode_, "chassis");
  controller_nh.param<std::string>("world_frame", world_frame_, "odom");
  controller_nh.param<std::string>("base_frame", base_frame_, "base_link");

  if (velocity_mode_ != "chassis" && velocity_mode_ != "world") {
    ROS_ERROR_STREAM("velocity_mode must be 'chassis' or 'world', got '" << velocity_mode_ << "'");
    return false;
  }

  LoadKinematicsParams(controller_nh);

  if (!InitPids(controller_nh)) {
    return false;
  }

  odometry_.reset(new ChassisOdometry(*kinematics_, controller_nh));

  // 只有 world 模式才需要 tf。少构造一个监听线程，也少一处出错的可能。
  if (velocity_mode_ == "world") {
    tf_buffer_.reset(new tf2_ros::Buffer());
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
    ROS_INFO_STREAM("velocity_mode=world, /cmd_vel is interpreted in frame '"
                    << world_frame_ << "' and transformed into '" << base_frame_ << "'");
  } else {
    ROS_INFO("velocity_mode=chassis, /cmd_vel is interpreted in the base frame");
  }

  // 注意话题名用绝对路径 "/cmd_vel"：
  // controller_nh 是 /<controller_name>，相对名会变成 /<controller_name>/cmd_vel。
  // teleop 和 nav 栈都在全局 /cmd_vel 上发指令，所以这里必须用 root_nh + 绝对名。
  cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1,
                                                         &HeroChassisController::CmdVelCallback, this);

  ROS_INFO_STREAM("hero_chassis_controller ready: R=" << kinematics_->wheel_radius()
                                                      << " m, base=" << kinematics_->wheel_base()
                                                      << " m, track=" << kinematics_->wheel_track()
                                                      << " m, K=" << kinematics_->k() << " m");
  return true;
}

bool HeroChassisController::LoadJointHandles(
    hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &controller_nh) {
  // 配置项名 → 默认关节名。默认值与 rm_description_for_task 的 urdf 一致。
  const std::array<std::string, kWheelCount> kParamKeys = {
      "left_front_joint", "right_front_joint", "left_back_joint", "right_back_joint"};
  const std::array<std::string, kWheelCount> kDefaultNames = {
      "left_front_wheel_joint", "right_front_wheel_joint", "left_back_wheel_joint", "right_back_wheel_joint"};

  for (int i = 0; i < kWheelCount; ++i) {
    std::string joint_name;
    controller_nh.param<std::string>(kParamKeys[i], joint_name, kDefaultNames[i]);
    try {
      joints_[i] = effort_joint_interface->getHandle(joint_name);
    } catch (const hardware_interface::HardwareInterfaceException &error) {
      // 常见原因：urdf 里关节名拼错，或者 transmission 里写的是 VelocityJointInterface
      // 而不是 EffortJointInterface。这里把关节名打出来，省得去猜。
      ROS_ERROR_STREAM("failed to get effort handle for joint '" << joint_name << "': " << error.what());
      return false;
    }
  }
  return true;
}

void HeroChassisController::LoadKinematicsParams(ros::NodeHandle &controller_nh) {
  // 默认值取自 rm_description_for_task/urdf/hero/chassis.urdf.xacro。
  double wheel_radius = 0.07625;
  double wheel_base = 0.4;
  double wheel_track = 0.4;
  controller_nh.param("wheel_radius", wheel_radius, wheel_radius);
  controller_nh.param("wheel_base", wheel_base, wheel_base);
  controller_nh.param("wheel_track", wheel_track, wheel_track);

  kinematics_.reset(new MecanumKinematics(wheel_radius, wheel_base, wheel_track));
}

bool HeroChassisController::InitPids(ros::NodeHandle &controller_nh) {
  // 每个轮子一个独立的 PID，参数在 yaml 里各自一段：
  //   /hero_chassis_controller/left_front_pid/{p, i, d, i_clamp_min, i_clamp_max}
  //
  // Pid::init(nh) 会读该命名空间下的参数，并且顺带把 dynamic_reconfigure 服务建好，
  // 之后可以用 rqt_reconfigure 在线调参，不必重编译、不必重启。
  // 四个轮子各用一个子 NodeHandle，是为了让四个 dyn-reconf 服务的名字不撞车。
  const std::array<std::string, kWheelCount> kPidKeys = {
      "left_front_pid", "right_front_pid", "left_back_pid", "right_back_pid"};

  for (int i = 0; i < kWheelCount; ++i) {
    ros::NodeHandle pid_nh(controller_nh, kPidKeys[i]);
    if (!pids_[i].init(pid_nh)) {
      ROS_ERROR_STREAM("failed to load PID parameters for '" << kPidKeys[i] << "'");
      return false;
    }
  }
  return true;
}

void HeroChassisController::starting(const ros::Time &time) {
  (void)time;
  geometry_msgs::Twist zero;
  cmd_vel_buffer_.writeFromNonRT(zero);
  cmd_vel_received_ = false;

  for (auto &pid : pids_) {
    pid.reset();
  }
  if (odometry_) {
    odometry_->Reset();
  }
  ROS_INFO("hero_chassis_controller started");
}

void HeroChassisController::stopping(const ros::Time &time) {
  (void)time;
  // 控制器被切走时把力矩清零，否则轮子会带着最后一次的指令继续转。
  for (auto &joint : joints_) {
    joint.setCommand(0.0);
  }
  ROS_INFO("hero_chassis_controller stopped");
}

void HeroChassisController::CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  // 订阅线程写、控制线程读，用 RealtimeBuffer 做无锁交换。
  cmd_vel_buffer_.writeFromNonRT(*msg);

  // 只在第一次收到时提示一次。这条日志的价值在于区分两种「车不动」：
  // 一种是根本没收到指令（话题名不对、控制器没起），
  // 另一种是收到了但 PID/符号有问题。没有这条日志只能靠猜。
  // exchange 是原子的「写入并返回旧值」，所以只会打印一次。
  if (!cmd_vel_received_.exchange(true)) {
    ROS_INFO("first /cmd_vel received, chassis is now driven by external commands");
  }
}

std::array<double, 3> HeroChassisController::ResolveCommand(const geometry_msgs::Twist &cmd) {
  double vx = cmd.linear.x;
  double vy = cmd.linear.y;
  double wz = cmd.angular.z;

  if (velocity_mode_ != "world" || !tf_buffer_) {
    // chassis 模式：/cmd_vel 本来就是车体系下的速度，限幅后直接用。
    return {{Clamp(vx, -max_linear_velocity_, max_linear_velocity_),
             Clamp(vy, -max_linear_velocity_, max_linear_velocity_),
             Clamp(wz, -max_angular_velocity_, max_angular_velocity_)}};
  }

  // world 模式：/cmd_vel 是 「world_frame_」 下的速度。
  // 要让底盘按世界系方向走，就得先把它转到车体系，再交给逆运动学。
  //
  // 这里用 ros::Time(0) 取「最新可用」的变换，而不是 time：
  // update() 拿到的 time 是这一周期的时刻，tf 里 odom->base_link 可能还没发布到那个时刻，
  // 查询会抛 ExtrapolationException。平面运动用最新变换完全够。
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(base_frame_, world_frame_, ros::Time(0));
  } catch (const tf2::TransformException &error) {
    // tf 还没就绪（比如刚好在启动瞬间）。这一周期退化成「停在原地」，
    // 比拿着错误的方向乱冲安全。
    ROS_WARN_STREAM_THROTTLE(1.0, "cannot transform " << world_frame_ << " -> " << base_frame_
                                                       << ": " << error.what() << ", holding still");
    return {{0.0, 0.0, 0.0}};
  }

  // 角速度在平面运动里与参考系无关，不需要变换；只有平移向量需要。
  geometry_msgs::Vector3Stamped velocity_in_world;
  velocity_in_world.header.frame_id = world_frame_;
  velocity_in_world.header.stamp = ros::Time(0);
  velocity_in_world.vector.x = vx;
  velocity_in_world.vector.y = vy;
  velocity_in_world.vector.z = 0.0;

  geometry_msgs::Vector3Stamped velocity_in_body;
  tf2::doTransform(velocity_in_world, velocity_in_body, transform);

  return {{Clamp(velocity_in_body.vector.x, -max_linear_velocity_, max_linear_velocity_),
           Clamp(velocity_in_body.vector.y, -max_linear_velocity_, max_linear_velocity_),
           Clamp(wz, -max_angular_velocity_, max_angular_velocity_)}};
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
  const geometry_msgs::Twist cmd = *cmd_vel_buffer_.readFromRT();
  const std::array<double, 3> body_velocity = ResolveCommand(cmd);

  // ① 逆运动学：车想怎么走 → 四个轮子各转多快
  const std::array<double, kWheelCount> desired_velocity =
      kinematics_->InverseKinematics(body_velocity[0], body_velocity[1], body_velocity[2]);

  // ② 读四个轮子的实测角速度
  std::array<double, kWheelCount> actual_velocity{};
  for (int i = 0; i < kWheelCount; ++i) {
    actual_velocity[i] = joints_[i].getVelocity();
  }

  // ③ 每个轮子一个 PID：速度误差 → 力矩
  //    注意 PID 的输入是「速度差」，输出是「力矩」，这是速度环而不是位置环。
  //    computeCommand 的 error 约定是 (target - state)，正好是「期望减实测」。
  //    （不要用 updatePid：它标了 deprecated，而且 error 约定是反的。）
  for (int i = 0; i < kWheelCount; ++i) {
    const double error = desired_velocity[i] - actual_velocity[i];
    const double effort = Clamp(pids_[i].computeCommand(error, period), -effort_limit_, effort_limit_);
    joints_[i].setCommand(effort);
  }

  // ④ 正运动学 + 积分：实测轮速 → 里程计
  if (odometry_) {
    odometry_->Update(actual_velocity, time, period);
  }
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase);

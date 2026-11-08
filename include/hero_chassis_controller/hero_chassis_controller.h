//
// 麦克纳姆轮底盘控制器
//
// 数据流：
//   /cmd_vel  ──►（可选：tf 把世界系速度转到车体系）──► 逆运动学
//                                                          │
//                                    四个轮子的期望角速度 ──┘
//                                                          │
//                                    四个 control_toolbox::Pid
//                                                          │
//                                    四个轮子的期望力矩 ──► setCommand
//                                                          │
//                                    实测轮速 ──► 正运动学 ──► 里程计
//

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H_
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H_

#include <array>
#include <atomic>
#include <memory>
#include <string>

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "hero_chassis_controller/chassis_odometry.h"
#include "hero_chassis_controller/mecanum_kinematics.h"

namespace hero_chassis_controller {

class HeroChassisController
    : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &root_nh,
            ros::NodeHandle &controller_nh) override;

  void starting(const ros::Time &time) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  void stopping(const ros::Time &time) override;

 private:
  // /cmd_vel 的回调在 ROS 的订阅线程里跑，而 update() 在实时控制线程里跑。
  // 两者不能直接共享一个 geometry_msgs::Twist 变量（会是数据竞争），
  // 也不能在实时线程里加锁（会破坏实时性）。
  // RealtimeBuffer 的写法是「实时侧无锁读、非实时侧加锁写」。
  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

  bool LoadJointHandles(hardware_interface::EffortJointInterface *effort_joint_interface,
                        ros::NodeHandle &controller_nh);
  void LoadKinematicsParams(ros::NodeHandle &controller_nh);
  bool InitPids(ros::NodeHandle &controller_nh);

  // 把 /cmd_vel 里的速度按 velocity_mode_ 解释，返回车体坐标系下的 (vx, vy, wz)。
  std::array<double, 3> ResolveCommand(const geometry_msgs::Twist &cmd);

  std::array<hardware_interface::JointHandle, kWheelCount> joints_;

  std::unique_ptr<MecanumKinematics> kinematics_;
  std::unique_ptr<ChassisOdometry> odometry_;
  std::array<control_toolbox::Pid, kWheelCount> pids_;

  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;
  ros::Subscriber cmd_vel_sub_;

  // velocity_mode_ == "world" 时才会用到 tf。
  // 这两个成员在 "chassis" 模式下不构造，省掉一个 tf 监听线程。
  std::string velocity_mode_;
  std::string world_frame_;
  std::string base_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  double max_linear_velocity_{2.0};
  double max_angular_velocity_{3.0};
  double effort_limit_{5.0};

  // 跨线程读写，用 atomic 而不是裸 bool，避免编译器把它优化进寄存器。
  std::atomic<bool> cmd_vel_received_{false};
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H_

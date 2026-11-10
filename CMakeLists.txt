cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

## C++14：control_toolbox 和 tf2 在 noetic 下都用 C++14 编译，保持一致。
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 打开常用警告，但不设 -Werror。
##
## -Werror 会把「一条无害的警告」直接升级成编译失败。示例包
## simple_chassis_controller 里用了它，但那是作者在固定的工具链上调通的。
## 本项目要在未知版本的 gcc 上编，且编译一次要点时间，
## 让警告拦住编译进程不划算 —— 先编过、先跑起来，再回头清警告。
## 需要严格模式时手动加：
##     catkin build hero_chassis_controller --cmake-args -DCMAKE_CXX_FLAGS=-Werror
add_definitions(-Wall)

find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    roslint
    controller_interface
    hardware_interface
    pluginlib
    control_toolbox
    realtime_tools
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    urdf
    )

catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    roslint
    controller_interface
    hardware_interface
    pluginlib
    control_toolbox
    realtime_tools
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    urdf
    )

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

## 运动学拆成独立源文件，好处是单元测试可以只链接它，
## 不必把整个控制器（连同 tf2、pluginlib）拖进来。
add_library(${PROJECT_NAME}_kinematics
    src/mecanum_kinematics.cpp
    )
target_include_directories(${PROJECT_NAME}_kinematics PUBLIC include)

## 控制器本体，编成 pluginlib 加载的动态库。
## 运动学通过链接 _kinematics 复用，不重复编译一份。
add_library(${PROJECT_NAME}
    src/hero_chassis_controller.cpp
    src/chassis_odometry.cpp
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_kinematics
    ${catkin_LIBRARIES}
    )

##############
## Testing  ##
##############

if (CATKIN_ENABLE_TESTING)
  ## 运动学单元测试：不依赖 ROS、不依赖 Gazebo，1 秒内出结果。
  ## 这是整个工程里性价比最高的一道防线 —— 公式错在这一步就拦住，
  ## 不用等到「起 Gazebo 看车乱走」才发现。
  catkin_add_gtest(test_mecanum_kinematics test/test_kinematics.cpp)
  if (TARGET test_mecanum_kinematics)
    target_link_libraries(test_mecanum_kinematics ${PROJECT_NAME}_kinematics)
  endif()
endif ()

##########################
## Static code analysis ##
##########################

## roslint 会按 ROS C++ Style Guide 检查缩进、行宽、行尾空格、include 顺序。
## 对应考核要求 3「正确的代码规范」。
roslint_cpp()

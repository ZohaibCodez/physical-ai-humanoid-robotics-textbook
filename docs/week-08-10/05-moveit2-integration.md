---
sidebar_position: 5
title: MoveIt2 Integration
---

# MoveIt2 Integration

## Learning Objectives

By the end of this chapter, you will be able to:

- Set up MoveIt2 for robot motion planning
- Create a C++ ROS 2 node for motion planning
- Plan and execute trajectories with collision avoidance
- Integrate MoveIt2 with custom robot models

## Introduction

MoveIt2 is the state-of-the-art motion planning framework for ROS 2. It provides advanced capabilities for manipulation, including collision checking, inverse kinematics, and trajectory execution.

## C++ ROS 2 Node Example

Here's a complete C++ example for motion planning with MoveIt2:

```cpp
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class MotionPlanningNode : public rclcpp::Node
{
public:
  MotionPlanningNode() : Node("motion_planning_node")
  {
    // Initialize MoveGroup interface for the planning group
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm");
    
    // Configure planning parameters
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(0.1);
    move_group_->setMaxAccelerationScalingFactor(0.1);
    
    RCLCPP_INFO(this->get_logger(), "Motion planning node initialized");
  }

  bool planToTarget(const geometry_msgs::msg::Pose& target_pose)
  {
    // Set the target pose
    move_group_->setPoseTarget(target_pose);
    
    // Plan the motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == 
                    moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Planning successful! Executing...");
      move_group_->execute(plan);
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
      return false;
    }
  }

  void addCollisionObject()
  {
    // Create a collision object (table)
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_->getPlanningFrame();
    collision_object.id = "table";
    
    // Define the table shape (box)
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.0;  // length (x)
    primitive.dimensions[1] = 1.0;  // width (y)
    primitive.dimensions[2] = 0.1;  // height (z)
    
    // Set the table pose
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.05;
    
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    
    // Add the object to the planning scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.addCollisionObjects(collision_objects);
    
    RCLCPP_INFO(this->get_logger(), "Collision object added to planning scene");
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionPlanningNode>();
  
  // Add collision objects
  node->addCollisionObject();
  
  // Create a target pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.3;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.5;
  
  // Plan and execute
  node->planToTarget(target_pose);
  
  rclcpp::shutdown();
  return 0;
}
```

## Compilation Instructions

### CMakeLists.txt Configuration

Add these dependencies to your `CMakeLists.txt`:

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(motion_planning_node src/motion_planning_node.cpp)

ament_target_dependencies(motion_planning_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
)

install(TARGETS
  motion_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

### package.xml Configuration

Add these dependencies:

```xml
<depend>rclcpp</depend>
<depend>moveit_ros_planning_interface</depend>
<depend>moveit_msgs</depend>
<depend>geometry_msgs</depend>
```

### Build and Run

```bash
# Build the workspace
cd ~/ros2_ws
colcon build --packages-select your_package_name

# Source the workspace
source install/setup.bash

# Launch MoveIt2 with your robot
ros2 launch your_robot_moveit_config demo.launch.py

# In another terminal, run your node
ros2 run your_package_name motion_planning_node
```

## Key Features

**Motion Planning**:
- Uses OMPL (Open Motion Planning Library) planners
- Supports multiple planning algorithms (RRT, PRM, etc.)
- Automatic collision checking

**Collision Avoidance**:
- Adds objects to planning scene
- Automatically avoids collisions during planning
- Supports self-collision checking

**Trajectory Execution**:
- Smooth trajectory generation
- Velocity/acceleration limits
- Real-time trajectory tracking

## Summary

MoveIt2 provides a powerful framework for robot manipulation. The C++ API gives you fine-grained control over motion planning, collision checking, and trajectory execution.

**Key Takeaways**:
- MoveIt2 simplifies complex motion planning tasks
- C++ interface provides high-performance control
- Collision checking is automatic and robust
- Integration with custom robots requires proper URDF setup

---

**Next Chapter**: [Navigation Stack](./06-navigation-stack.md) →

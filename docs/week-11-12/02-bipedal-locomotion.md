---
sidebar_position: 2
title: Bipedal Locomotion
---

# Bipedal Locomotion

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the gait cycle and phases of walking
- Implement Zero Moment Point (ZMP) based walking
- Generate walking trajectories for humanoid robots
- Control balance during bipedal locomotion

## Introduction

Bipedal locomotion is one of the most challenging problems in robotics. Unlike wheeled robots, humanoid robots must constantly manage balance while moving, making walking a complex dynamic control problem.

## Gait Cycle Phases

The human walking cycle consists of:

1. **Stance Phase** (60%): Foot in contact with ground
2. **Swing Phase** (40%): Foot moving forward
3. **Double Support** (10%): Both feet on ground
4. **Single Support** (50%): One foot on ground

## Zero Moment Point (ZMP) Walking

The ZMP is the point on the ground where the total inertia force equals zero. Stable walking requires keeping the ZMP inside the support polygon.

### Complete ZMP Walking Controller

<details>
<summary><strong>Click to expand: Full ZMP Walking Implementation (Python)</strong></summary>

```python
import numpy as np
from dataclasses import dataclass
from typing import Tuple, List
import matplotlib.pyplot as plt

@dataclass
class GaitParameters:
    """Parameters for bipedal gait generation"""
    step_length: float = 0.2  # meters
    step_height: float = 0.05  # meters
    step_duration: float = 0.8  # seconds
    double_support_ratio: float = 0.1  # 10% of cycle
    com_height: float = 0.8  # meters (center of mass)
    foot_width: float = 0.1  # meters
    foot_length: float = 0.2  # meters
    
class ZMPWalkingController:
    """
    Zero Moment Point (ZMP) based walking controller for humanoid robots.
    
    This implementation generates stable walking trajectories by ensuring
    the ZMP stays within the support polygon throughout the gait cycle.
    """
    
    def __init__(self, params: GaitParameters):
        self.params = params
        self.g = 9.81  # gravity (m/s^2)
        self.current_step = 0
        self.trajectory_cache = {}
        
    def compute_zmp_trajectory(self, num_steps: int = 4) -> np.ndarray:
        """
        Compute ZMP reference trajectory for walking.
        
        Args:
            num_steps: Number of steps to plan
            
        Returns:
            zmp_trajectory: Array of ZMP positions [x, y] over time
        """
        dt = 0.01  # time step (100 Hz control)
        step_samples = int(self.params.step_duration / dt)
        total_samples = step_samples * num_steps
        
        zmp_trajectory = np.zeros((total_samples, 2))
        
        for step in range(num_steps):
            start_idx = step * step_samples
            end_idx = start_idx + step_samples
            
            # Alternate between left and right foot
            if step % 2 == 0:
                # Right foot support
                foot_y = -self.params.foot_width / 2
            else:
                # Left foot support
                foot_y = self.params.foot_width / 2
            
            # Move ZMP forward during step
            foot_x = np.linspace(
                step * self.params.step_length,
                (step + 1) * self.params.step_length,
                step_samples
            )
            
            zmp_trajectory[start_idx:end_idx, 0] = foot_x
            zmp_trajectory[start_idx:end_idx, 1] = foot_y
            
        return zmp_trajectory
    
    def compute_com_trajectory(self, zmp_trajectory: np.ndarray) -> np.ndarray:
        """
        Compute Center of Mass (CoM) trajectory from ZMP reference.
        
        Uses inverted pendulum model to ensure dynamic stability.
        
        Args:
            zmp_trajectory: ZMP reference positions
            
        Returns:
            com_trajectory: CoM positions and velocities
        """
        dt = 0.01
        num_samples = len(zmp_trajectory)
        
        # Initialize CoM state [x, y, vx, vy]
        com_state = np.zeros((num_samples, 4))
        com_state[0, :2] = zmp_trajectory[0]  # Start at first ZMP position
        
        # Natural frequency of inverted pendulum
        omega = np.sqrt(self.g / self.params.com_height)
        
        for t in range(num_samples - 1):
            # Current state
            x, y, vx, vy = com_state[t]
            zmp_x, zmp_y = zmp_trajectory[t]
            
            # Inverted pendulum dynamics
            # Acceleration: a = omega^2 * (CoM - ZMP)
            ax = omega**2 * (x - zmp_x)
            ay = omega**2 * (y - zmp_y)
            
            # Integrate with Euler method
            com_state[t+1, 0] = x + vx * dt
            com_state[t+1, 1] = y + vy * dt
            com_state[t+1, 2] = vx + ax * dt
            com_state[t+1, 3] = vy + ay * dt
            
        return com_state
    
    def generate_foot_trajectory(
        self,
        start_pos: np.ndarray,
        end_pos: np.ndarray,
        duration: float,
        is_swing: bool = True
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate smooth foot trajectory using cubic splines.
        
        Args:
            start_pos: Starting foot position [x, y, z]
            end_pos: Ending foot position [x, y, z]
            duration: Duration of motion
            is_swing: Whether this is swing phase (lift foot)
            
        Returns:
            positions: Foot positions over time
            velocities: Foot velocities
            accelerations: Foot accelerations
        """
        dt = 0.01
        num_samples = int(duration / dt)
        t = np.linspace(0, duration, num_samples)
        
        positions = np.zeros((num_samples, 3))
        velocities = np.zeros((num_samples, 3))
        accelerations = np.zeros((num_samples, 3))
        
        for axis in range(3):
            # Cubic polynomial coefficients for smooth motion
            # p(t) = a0 + a1*t + a2*t^2 + a3*t^3
            # Boundary conditions: p(0)=start, p(T)=end, v(0)=0, v(T)=0
            
            T = duration
            a0 = start_pos[axis]
            a1 = 0  # zero initial velocity
            a2 = 3*(end_pos[axis] - start_pos[axis]) / T**2
            a3 = -2*(end_pos[axis] - start_pos[axis]) / T**3
            
            # Position, velocity, acceleration
            positions[:, axis] = a0 + a1*t + a2*t**2 + a3*t**3
            velocities[:, axis] = a1 + 2*a2*t + 3*a3*t**2
            accelerations[:, axis] = 2*a2 + 6*a3*t
        
        # Add swing height (parabolic arc) if swing phase
        if is_swing:
            swing_height = self.params.step_height * (
                4 * (t / duration) * (1 - t / duration)
            )
            positions[:, 2] += swing_height
            
        return positions, velocities, accelerations
    
    def check_stability(self, com_pos: np.ndarray, support_foot: str) -> bool:
        """
        Check if current CoM position maintains stability.
        
        Stability criterion: CoM projection must be inside support polygon.
        
        Args:
            com_pos: Center of mass position [x, y, z]
            support_foot: 'left' or 'right'
            
        Returns:
            is_stable: Whether the robot is stable
        """
        # Define support polygon (simplified as rectangle)
        if support_foot == 'right':
            foot_y = -self.params.foot_width / 2
        else:
            foot_y = self.params.foot_width / 2
            
        # Check if CoM projection is within support polygon
        x_margin = self.params.foot_length / 2 * 0.8  # 80% safety margin
        y_margin = self.params.foot_width / 2 * 0.8
        
        within_x = abs(com_pos[0]) < x_margin
        within_y = abs(com_pos[1] - foot_y) < y_margin
        
        return within_x and within_y
    
    def visualize_gait(self, com_trajectory: np.ndarray, zmp_trajectory: np.ndarray):
        """
        Visualize the planned gait trajectories.
        
        Args:
            com_trajectory: Center of mass trajectory
            zmp_trajectory: Zero moment point trajectory
        """
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        
        # Top-down view
        ax1.plot(com_trajectory[:, 0], com_trajectory[:, 1], 
                'b-', linewidth=2, label='CoM Trajectory')
        ax1.plot(zmp_trajectory[:, 0], zmp_trajectory[:, 1], 
                'r--', linewidth=2, label='ZMP Trajectory')
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.set_title('Walking Trajectory (Top View)')
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')
        
        # Side view (X-Z plane)
        time = np.arange(len(com_trajectory)) * 0.01
        ax2.plot(time, com_trajectory[:, 0], 'b-', linewidth=2, label='CoM X')
        ax2.plot(time, zmp_trajectory[:, 0], 'r--', linewidth=2, label='ZMP X')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('X Position (m)')
        ax2.set_title('Forward Progress')
        ax2.legend()
        ax2.grid(True)
        
        plt.tight_layout()
        plt.savefig('zmp_walking_trajectory.png', dpi=150)
        print("Visualization saved to zmp_walking_trajectory.png")

# Example usage
if __name__ == "__main__":
    # Create controller with default parameters
    params = GaitParameters(
        step_length=0.15,
        step_height=0.04,
        step_duration=0.8,
        com_height=0.75
    )
    
    controller = ZMPWalkingController(params)
    
    # Generate walking trajectory for 4 steps
    print("Generating ZMP trajectory...")
    zmp_traj = controller.compute_zmp_trajectory(num_steps=4)
    
    print("Computing CoM trajectory...")
    com_traj = controller.compute_com_trajectory(zmp_traj)
    
    # Check stability
    print("\nStability check:")
    stable_count = 0
    for i in range(0, len(com_traj), 100):  # Check every 1 second
        is_stable = controller.check_stability(
            com_traj[i, :3],
            'right' if (i // (0.8 * 100)) % 2 == 0 else 'left'
        )
        if is_stable:
            stable_count += 1
    
    print(f"Stability: {stable_count}/{len(com_traj)//100} checkpoints passed")
    
    # Visualize
    print("\nGenerating visualization...")
    controller.visualize_gait(com_traj, zmp_traj)
    
    print("\nWalking controller ready!")
    print(f"Step length: {params.step_length}m")
    print(f"Step duration: {params.step_duration}s")
    print(f"Walking speed: {params.step_length/params.step_duration:.2f} m/s")
```

</details>

### Key Implementation Details

**Inverted Pendulum Model**:
- Simplifies humanoid to point mass at CoM
- Dynamics: `a = ω² × (CoM - ZMP)` where `ω = √(g/h)`
- Natural frequency depends on CoM height

**Stability Criterion**:
- ZMP must stay inside support polygon (foot contact area)
- Larger safety margin = more stable but slower walking
- Double support phase provides extra stability

**Trajectory Generation**:
- Cubic splines for smooth foot motion
- Zero velocity at start/end of swing
- Parabolic swing height profile

## ROS 2 Integration

<details>
<summary><strong>Click to expand: ROS 2 Walking Controller Node</strong></summary>

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class BipedalWalkingNode(Node):
    """ROS 2 node for bipedal walking control"""
    
    def __init__(self):
        super().__init__('bipedal_walking_node')
        
        # Initialize walking controller
        self.params = GaitParameters()
        self.controller = ZMPWalkingController(self.params)
        
        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )
        
        self.com_pub = self.create_publisher(
            PoseStamped,
            '/walking/com_trajectory',
            10
        )
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Control timer (100 Hz)
        self.control_timer = self.create_timer(0.01, self.control_loop)
        
        self.get_logger().info('Bipedal walking node started')
        
        # Generate initial trajectory
        self.zmp_traj = self.controller.compute_zmp_trajectory(num_steps=4)
        self.com_traj = self.controller.compute_com_trajectory(self.zmp_traj)
        self.traj_index = 0
        
    def joint_state_callback(self, msg):
        """Process current joint states"""
        # Store current joint positions for feedback control
        self.current_joint_positions = dict(zip(msg.name, msg.position))
        
    def control_loop(self):
        """Main control loop - runs at 100 Hz"""
        if self.traj_index >= len(self.com_traj):
            self.get_logger().info('Walking trajectory completed')
            return
        
        # Get desired CoM position from trajectory
        desired_com = self.com_traj[self.traj_index, :3]
        
        # Compute inverse kinematics (simplified)
        joint_commands = self.com_to_joint_positions(desired_com)
        
        # Publish joint commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = joint_commands
        self.joint_cmd_pub.publish(cmd_msg)
        
        # Publish CoM trajectory for visualization
        com_msg = PoseStamped()
        com_msg.header.stamp = self.get_clock().now().to_msg()
        com_msg.header.frame_id = 'world'
        com_msg.pose.position.x = float(desired_com[0])
        com_msg.pose.position.y = float(desired_com[1])
        com_msg.pose.position.z = float(desired_com[2])
        self.com_pub.publish(com_msg)
        
        self.traj_index += 1
        
    def com_to_joint_positions(self, com_pos):
        """
        Convert desired CoM position to joint angles.
        This is a simplified version - real implementation needs full IK.
        """
        # Placeholder - implement full inverse kinematics here
        num_joints = 12  # Typical humanoid leg joints
        return [0.0] * num_joints

def main(args=None):
    rclpy.init(args=args)
    node = BipedalWalkingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

</details>

## Summary

Bipedal locomotion requires careful balance control using ZMP principles. The inverted pendulum model provides a computationally efficient approach for stable walking trajectory generation.

**Key Takeaways**:
- ZMP must stay within support polygon for stability
- Inverted pendulum model enables real-time control
- Gait cycle consists of swing and stance phases
- Trajectory smoothness is critical for energy efficiency

## Further Reading

- [Introduction to Humanoid Robotics](https://link.springer.com/book/10.1007/978-3-642-54536-8) by Kajita et al.
- [ZMP Walking Tutorial](https://www.inrialpes.fr/bipop/material/ZMPwalking.pdf) - INRIA

---

**Next Chapter**: [Balance and Stability](./03-balance-and-stability.md) →

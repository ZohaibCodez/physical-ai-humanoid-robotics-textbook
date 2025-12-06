# Part 2: ROS 2 Ecosystem

Welcome to Part 2! This is where theory transforms into practice. You'll install ROS 2 Humble, write your first robot nodes, and master the middleware that powers modern robotics systems.

## From Concepts to Code

In Part 1, you understood **why** Physical AI requires different approaches than software-only systems. In Part 2, you'll learn **how** to build these systems using ROS 2 (Robot Operating System 2)—the industry-standard framework for robot software development.

ROS 2 isn't just a library or framework; it's a complete ecosystem that provides:
- **Communication infrastructure** (publish-subscribe, services, actions)
- **Hardware abstraction** (sensors, actuators, controllers)
- **Development tools** (visualization, debugging, simulation integration)
- **Standard interfaces** (coordinate transforms, control messages, sensor data)
- **Community packages** (navigation, manipulation, perception)

This part builds your ROS 2 mastery from first principles through advanced patterns.

## What You'll Learn in Part 2

### Chapter 3: ROS 2 Architecture (4 lessons)
Understand the fundamental shift from ROS 1 to ROS 2. Learn about DDS middleware, Quality-of-Service policies, workspace organization, and the Colcon build system. You'll understand **why** ROS 2 makes the architectural choices it does.

**Key Questions:**
- What problems does ROS 2 solve that ROS 1 couldn't?
- How does DDS middleware enable reliable robot communication?
- How do you organize multi-package robot projects?
- What's the role of the build system in robotics development?

### Chapter 4: Nodes, Topics, and Services (4 lessons)
Master the core communication patterns in ROS 2. You'll write publishers, subscribers, services, and clients. Learn lifecycle management for robust nodes and create custom message types for your specific robot needs.

**Key Questions:**
- How do nodes communicate in distributed robot systems?
- When should you use topics vs services?
- How do you manage node lifecycle for safety-critical systems?
- How do you define custom interfaces for domain-specific data?

### Chapter 5: ActionLib and Goal-Based Control (4 lessons)
Learn goal-based communication for long-running robot tasks. Understand when actions are appropriate (vs services), implement action servers with feedback and preemption, and integrate with navigation systems.

**Key Questions:**
- What makes actions different from services?
- How do you provide feedback for multi-second robot tasks?
- How do you handle task cancellation and preemption?
- How does navigation use the action pattern?

### Chapter 6: TF2 Transformations (5 lessons)
Master coordinate frame transformations—the mathematical foundation of robot spatial reasoning. Learn the TF2 tree structure, broadcast transforms, query frame relationships, and integrate with robot state publishers.

**Key Questions:**
- How do robots track relationships between sensors, joints, and the world?
- What's the difference between static and dynamic transforms?
- How do you debug transform tree problems?
- How does TF2 integrate with URDF robot models?

## Learning Approach

Part 2 is **hands-on and code-heavy**. You'll:
- **Install ROS 2 Humble** on Ubuntu 22.04 (or WSL2)
- **Write Python nodes** following modern best practices (type hints, lifecycle)
- **Build workspaces** with multiple packages
- **Debug distributed systems** using ROS 2 tools
- **Test on simulated robots** before moving to hardware

### Prerequisites

Before starting Part 2:
- ✅ Complete Part 1 (conceptual foundations)
- ✅ Ubuntu 22.04 LTS environment (native or WSL2)
- ✅ Python 3.10+ installed
- ✅ Basic command-line comfort
- ✅ Text editor or IDE (VS Code recommended)

*See Appendix: Installation Guide for detailed setup instructions*

### Development Environment

You'll need:
- **ROS 2 Humble Hawksbill** (LTS release, supported until 2027)
- **Colcon** build tool
- **Gazebo Classic or Garden** (for simulation)
- **RViz2** (for visualization)
- **VS Code with ROS extensions** (recommended)

## Estimated Time

**⏱️ Total Time for Part 2**: 25-30 hours

- **Core lessons**: 15-20 hours (17 lessons × 1-1.5 hours each)
- **Hands-on coding**: 8-10 hours (building and testing examples)
- **Troubleshooting and exploration**: 2-3 hours

**Recommended Pace**: 2-3 lessons per week over 6-8 weeks

## Part Structure

**Chapter 3: ROS 2 Architecture** (4 lessons)
1. ROS 1 to ROS 2 Evolution
2. DDS Middleware & QoS Policies
3. Packages and Workspaces
4. Colcon Build System

**Chapter 4: Nodes, Topics, and Services** (4 lessons)
1. Nodes and Lifecycle Management
2. Publishers and Subscribers
3. Services and Clients
4. Custom Messages and Interfaces

**Chapter 5: ActionLib and Goal-Based Control** (4 lessons)
1. Actions vs Services
2. Implementing Action Servers
3. Action Clients and Feedback
4. Navigation Action Interfaces

**Chapter 6: TF2 Transformations** (5 lessons)
1. Coordinate Frames and Transforms
2. TF2 Tree Structure
3. Broadcasting Transforms
4. Listening to Transforms
5. Robot State Publisher

## Connection to Other Parts

**Building on Part 1:**
- Physical AI challenges (sensing, acting, real-time) → ROS 2 communication patterns solve these
- Embodied intelligence concepts → Implemented through ROS 2 nodes and transforms

**Preparing for Part 3:**
- ROS 2 skills → Used to control simulated robots in Gazebo and Unity
- TF2 transforms → Essential for URDF robot modeling
- Action patterns → Foundation for navigation and manipulation

**Enabling Parts 4-7:**
- ROS 2 proficiency → Required for Isaac ROS, humanoid control, conversational systems
- Communication patterns → Used throughout all advanced topics

## Real-World Context

ROS 2 powers robots from:
- **Autonomous vehicles** (Toyota, Apex.AI)
- **Warehouse robots** (Amazon Robotics, Fetch)
- **Humanoids** (Boston Dynamics Spot, Agility Robotics Digit)
- **Manipulation systems** (Universal Robots, Franka Emika)
- **Research platforms** (TurtleBot, PR2, most academic robots)

Companies using ROS 2:
- **Aerospace**: NASA (Mars rovers, lunar robots)
- **Automotive**: BMW, Bosch, Continental
- **Logistics**: Amazon, Locus Robotics, Otto Motors
- **Healthcare**: Intuitive Surgical, Johns Hopkins
- **Agriculture**: Blue River Technology, Naïo Technologies

## Success Criteria

By the end of Part 2, you will be able to:

✅ Install and configure ROS 2 Humble environments
✅ Write well-structured Python nodes with lifecycle management
✅ Implement publishers, subscribers, services, and action servers
✅ Design communication patterns for robot systems
✅ Create custom message and service definitions
✅ Build and manage multi-package workspaces with Colcon
✅ Use TF2 for coordinate frame transformations
✅ Debug distributed robot systems using ROS 2 tools
✅ Integrate with Gazebo simulation
✅ Follow ROS 2 best practices and code quality standards

## What Comes Next

After completing Part 2, you'll move to **Part 3: Simulation Environments**, where you'll:
- Build detailed robot models in URDF and Xacro
- Simulate robots in Gazebo Garden
- Create photorealistic environments in Unity Robotics Hub
- Test perception and control algorithms in simulation
- Prepare for sim-to-real transfer

But first, let's master the ROS 2 foundation.

---

**Ready to start coding?** Begin with [Chapter 3: ROS 2 Architecture](./chapter-03-ros2-architecture/index.md)

---

*Part 2 is Week 3-5 of the 13-week curriculum. Ensure ROS 2 Humble is installed before proceeding.*

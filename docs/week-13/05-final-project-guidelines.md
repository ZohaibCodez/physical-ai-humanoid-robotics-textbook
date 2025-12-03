---
sidebar_position: 5
title: Capstone Project
---

# Capstone Project: Autonomous Humanoid Robot

## Project Overview

**Goal**: Design, implement, and demonstrate an autonomous humanoid robot system that integrates perception, navigation, manipulation, and locomotion in a simulated or real environment.

**Duration**: Week 13 (intensive focus) + optional polish time  
**Team Size**: 1-3 students  
**Weight**: 25% of final grade  
**Minimum Requirements**: Must demonstrate all four core capabilities (perception, navigation, manipulation, locomotion)

---

## Project Scenario

Your humanoid robot must complete the following task:

**"Fetch and Deliver"**  
The robot starts in a home position, navigates to a target location avoiding obstacles, identifies and grasps a specific object (e.g., bottle, box), carries it while walking, and delivers it to a designated drop-off point.

### Environment Specifications
- **Space**: 5m x 5m indoor area with 3-5 obstacles
- **Objects**: At least 3 different objects to identify (use 1)
- **Terrain**: Flat surface (stairs/slopes optional for extra credit)
- **Lighting**: Standard indoor lighting (shadows acceptable)

---

## Core Requirements (Must Complete All)

### 1. Perception (25 points)

**Required Capabilities**:
- RGB-D camera or lidar for environment sensing
- Object detection using deep learning (YOLO, MobileNet, or similar)
- 6D pose estimation for grasp planning
- Obstacle detection for navigation

**Deliverables**:
- Trained or fine-tuned object detection model (minimum 80% accuracy on test set)
- Real-time processing (over 10 FPS)
- Visualization of detected objects with bounding boxes/poses

### 2. Navigation (25 points)

**Required Capabilities**:
- SLAM for map building and localization
- Path planning to goal (A*, RRT, or similar)
- Dynamic obstacle avoidance
- Localization accuracy within ±10cm

**Deliverables**:
- Generated map of environment
- Planned path visualization
- Successful navigation in 8/10 trials
- Handling of dynamic obstacles (person walking through path)

### 3. Manipulation (25 points)

**Required Capabilities**:
- Inverse kinematics for arm control
- Grasp planning based on object pose
- Pick-and-place with success rate over 70%
- Collision-free motion planning

**Deliverables**:
- Working gripper/end-effector
- Successful grasp of target object
- Stable hold during locomotion
- Precise placement at goal location

### 4. Locomotion (25 points)

**Required Capabilities**:
- Stable bipedal walking (minimum 10 steps)
- Balance control (ZMP or similar)
- Walking with object in hand
- Recovery from minor disturbances

**Deliverables**:
- Gait cycle implementation
- ZMP trajectory within support polygon
- Walking speed over 0.1 m/s
- No falls during task execution

---

## Technical Requirements

### Robot Specifications
- **DOF**: Minimum 12 (6 legs, 6 arms) + gripper
- **Sensors**: RGB-D camera or lidar + IMU
- **Platform**: Gazebo, NVIDIA Isaac Sim, or real hardware
- **Framework**: ROS 2 Humble

### Software Architecture
Your system must include:
- **Perception Node**: Object detection + localization
- **Planning Node**: Path planning + task sequencing
- **Control Nodes**: Arm control + locomotion control
- **State Machine**: High-level task coordination

### Performance Metrics
| Metric | Target | Measurement |
|--------|--------|-------------|
| Task completion time | Under 5 minutes | Timer from start to delivery |
| Navigation success rate | Greater than 80% | 8/10 trials reach goal |
| Grasp success rate | Greater than 70% | 7/10 grasps successful |
| Locomotion stability | No falls | Walking test over 20 steps |
| Perception accuracy | Greater than 80% | Correct object ID rate |

---

## Project Timeline

### Phase 1: Design (Week 13, Days 1-2)
- ✅ Define system architecture
- ✅ Select algorithms for each component
- ✅ Create URDF model or choose existing platform
- ✅ Set up simulation environment

**Deliverable**: Design document (2-3 pages) with architecture diagram

### Phase 2: Component Development (Days 3-5)
- ✅ Implement and test perception independently
- ✅ Implement and test navigation independently
- ✅ Implement and test manipulation independently
- ✅ Implement and test locomotion independently

**Deliverable**: Four working components with unit tests

### Phase 3: Integration (Days 6-7)
- ✅ Integrate perception + navigation
- ✅ Integrate manipulation + locomotion
- ✅ Full system integration with state machine
- ✅ End-to-end testing

**Deliverable**: Working integrated system

### Phase 4: Optimization & Documentation (Days 8-10)
- ✅ Performance tuning and bug fixes
- ✅ Record demonstration video
- ✅ Write final report
- ✅ Prepare presentation

**Deliverable**: Final submission package

---

## Grading Rubric (Total: 100 points)

### Core Functionality (70 points)

| Component | Excellent (18 to 25pts) | Good (14 to 17pts) | Satisfactory (10 to 13pts) | Needs Work (under 10pts) |
|-----------|---------------------|----------------|----------------------|---------------------|
| **Perception** | Robust detection, accurate poses, over 85% accuracy | Good detection, minor errors, over 75% accuracy | Basic detection, significant errors, over 60% accuracy | Poor detection or under 60% accuracy |
| **Navigation** | Reliable path planning, smooth motion, over 85% success | Good navigation, minor issues, over 70% success | Basic navigation, frequent issues | Often fails to reach goal |
| **Manipulation** | Smooth grasping, stable hold, over 75% success | Good grasps, occasional drops, over 60% success | Basic grasping, frequent failures | Rarely succeeds in grasping |
| **Locomotion** | Stable walking, good balance, handles disturbances | Walks reliably, minor stability issues | Basic walking, some instability | Frequent falls or won't walk |

### Integration & System Design (15 points)

| Criterion | Excellent (13 to 15pts) | Good (10 to 12pts) | Satisfactory (7 to 9pts) | Needs Work (under 7pts) |
|-----------|---------------------|----------------|----------------------|---------------------|
| **System Integration** | Seamless component interaction, robust state machine | Good integration, minor coordination issues | Components work but integration rough | Poor integration, components don't work together |

### Documentation & Presentation (15 points)

| Criterion | Excellent (13 to 15pts) | Good (10 to 12pts) | Satisfactory (7 to 9pts) | Needs Work (under 7pts) |
|-----------|---------------------|----------------|----------------------|---------------------|
| **Report Quality** | Comprehensive analysis, clear methodology, thorough results | Good documentation, minor gaps | Basic report, missing some details | Incomplete or unclear documentation |
| **Video Demo** | Professional presentation, clear narration, shows all features | Good demo, most features shown | Basic demo, some features missing | Poor quality or incomplete demo |

---

## Deliverables

### 1. Source Code (Required)
**Format**: Git repository (GitHub/GitLab)

**Structure**:
```
capstone-project/
├── README.md                    # Overview and setup instructions
├── package.xml                  # ROS 2 package metadata
├── launch/
│   ├── full_system.launch.py   # Launch entire system
│   ├── perception.launch.py    # Launch perception only (for testing)
│   ├── navigation.launch.py
│   ├── manipulation.launch.py
│   └── locomotion.launch.py
├── src/
│   ├── perception/             # Perception nodes
│   ├── navigation/             # SLAM, planning nodes
│   ├── manipulation/           # Arm control, grasp planning
│   ├── locomotion/             # Walking controller
│   └── state_machine/          # High-level task coordinator
├── config/
│   ├── sensors.yaml            # Sensor configurations
│   ├── navigation.yaml         # Nav2 parameters
│   └── control.yaml            # Controller gains
├── urdf/
│   └── humanoid_robot.urdf     # Robot description
├── worlds/
│   └── task_environment.world  # Gazebo world file
├── models/
│   └── object_detector.pt      # Trained model weights
└── docs/
    ├── design_document.pdf     # System design
    ├── final_report.pdf        # Final report
    └── diagrams/               # Architecture diagrams
```

### 2. Final Report (Required)
**Format**: PDF, 10-15 pages

**Required Sections**:
1. **Executive Summary** (1 page)
   - Project overview and key achievements

2. **System Architecture** (2-3 pages)
   - Overall system diagram
   - Component descriptions and interfaces
   - State machine diagram

3. **Technical Approach** (4-6 pages)
   - Perception: Algorithm choice, training details
   - Navigation: SLAM and planning algorithms
   - Manipulation: Kinematics, grasp planning
   - Locomotion: Gait generation, balance control

4. **Implementation Details** (2-3 pages)
   - ROS 2 node structure
   - Key code snippets with explanations
   - Parameter tuning process

5. **Experimental Results** (3-4 pages)
   - Performance metrics with graphs
   - Success rates for each component
   - Failure analysis and edge cases
   - Comparison to baseline or requirements

6. **Challenges and Solutions** (1-2 pages)
   - Major obstacles encountered
   - How you overcame them
   - Lessons learned

7. **Conclusion and Future Work** (1 page)
   - Summary of achievements
   - Potential improvements
   - Applications

8. **References**
   - Cited papers, libraries, resources

### 3. Demonstration Video (Required)
**Format**: MP4 or MOV, 5-8 minutes

**Required Content**:
1. **Introduction** (30 sec)
   - Team members, project title

2. **System Overview** (1 min)
   - Architecture diagram walkthrough
   - Component descriptions

3. **Individual Component Demos** (2 min)
   - Perception: Show object detection
   - Navigation: Show SLAM map + path planning
   - Manipulation: Show grasp execution
   - Locomotion: Show walking cycle

4. **Full Task Execution** (2-3 min)
   - Complete "Fetch and Deliver" run
   - Real-time or slightly sped up (2x max)
   - Narrate what's happening

5. **Results Summary** (1 min)
   - Key metrics and success rates
   - Demonstration of robustness (show multiple trials if possible)

6. **Conclusion** (30 sec)
   - Key achievements and challenges

**Technical Requirements**:
- Resolution: 1080p minimum
- Audio: Clear narration (no background music)
- Subtitles: Optional but recommended
- Hosting: YouTube (unlisted), Google Drive, or Vimeo

### 4. Live Presentation (If Required)
**Duration**: 15 minutes + 5 minutes Q&A

**Format**:
- Slides (10-15 slides)
- Live demo or video playback
- All team members participate

---

## Implementation Tips

### Getting Started
1. **Choose Your Platform**:
   - Gazebo: Easier setup, good community support
   - Isaac Sim: Better physics, photorealistic rendering
   - Real Hardware: Most impressive but higher risk

2. **Select Base Robot**:
   - Use existing URDF (ROBOTIS OP3, NAO, Atlas) or
   - Build custom humanoid (minimum viable DOF)

3. **Leverage Existing Packages**:
   - Nav2 for navigation
   - MoveIt2 for manipulation
   - ros2_control for joint management

### Development Strategy
- **Week 1**: Get basic robot walking in simulation
- **Week 1-2**: Implement perception independently
- **Week 2**: Add navigation (SLAM + planning)
- **Week 2-3**: Integrate manipulation
- **Week 3**: Full system integration and testing

### Common Pitfalls to Avoid
❌ **Overly ambitious scope** - Stick to minimum requirements first  
❌ **Late integration** - Integrate early, test often  
❌ **Ignoring edge cases** - Test failure scenarios  
❌ **Poor time management** - Don't leave video recording for last day  
❌ **Inadequate testing** - Test each component independently first  

### Recommended Tools
- **Simulation**: Gazebo Fortress or NVIDIA Isaac Sim
- **Visualization**: RViz2, rqt_graph
- **Development**: VS Code with ROS 2 extensions
- **Version Control**: Git with meaningful commits
- **Collaboration**: GitHub Projects for task tracking (if team)

---

## Extra Credit Opportunities (+10 points max)

### Advanced Features (+5 points each)
1. **Stair Climbing**: Robot walks up/down stairs (3-5 steps)
2. **Dynamic Obstacle Avoidance**: Handles moving obstacles (person walking)
3. **Multi-Object Manipulation**: Grasps and carries multiple objects
4. **Whole-Body Control**: Uses arms for balance during walking
5. **Learning-Based Control**: Uses RL or imitation learning for locomotion

### Innovation Bonus (+5 points each)
1. **Novel Algorithm**: Implement new approach with justification
2. **Sim-to-Real Transfer**: Demonstrate on real hardware (if available)
3. **Failure Recovery**: Robot recovers from failures autonomously
4. **Human-Robot Interaction**: Robot responds to voice commands or gestures

**Note**: Extra credit cannot exceed +10 total. Quality over quantity.

---

## Example Project Ideas

### Minimum Viable Project (Target: B grade)
- Simple environment with 3 obstacles
- Basic object detection (single class)
- A* path planning on known map
- Simple grasping with parallel gripper
- Static walking gait (no dynamic balance)

### Strong Project (Target: A grade)
- Complex environment with 5+ obstacles
- Multi-class object detection with pose estimation
- Nav2 stack with dynamic replanning
- Adaptive grasping based on object properties
- Dynamic walking with ZMP control

### Exceptional Project (Target: A+ with extra credit)
- Multi-level environment (stairs)
- Learning-based locomotion
- Whole-body control for manipulation
- Real-time performance optimization
- Sim-to-real transfer demonstrated

---

## Submission

**Deadline**: End of Week 13 (Friday 11:59 PM)

**Submit via Course Platform**:
1. Git repository URL (public or add instructor as collaborator)
2. Final report PDF
3. Video link (YouTube unlisted, Google Drive, or Vimeo)
4. README.md must include:
   - Setup instructions
   - Dependencies list
   - Run commands
   - Expected output
   - Troubleshooting guide

**Late Policy**: -10% per day, maximum 3 days

---

## Resources

### ROS 2 & Robotics
- [Nav2 Documentation](https://navigation.ros.org/)
- [MoveIt2 Tutorials](https://moveit.picknik.ai/humble/index.html)
- [ros2_control](https://control.ros.org/)

### Perception & AI
- [YOLOv8 for ROS 2](https://github.com/mgonzs13/yolov8_ros)
- [PyTorch](https://pytorch.org/)
- [OpenCV](https://opencv.org/)

### Humanoid Robotics
- [Humanoid Robotics Papers](https://paperswithcode.com/task/humanoid-control)
- [Whole-Body Control](https://github.com/stack-of-tasks/pinocchio)
- [Bipedal Locomotion Framework](https://github.com/ami-iit/bipedal-locomotion-framework)

### Simulation
- [Gazebo Documentation](https://gazebosim.org/docs)
- [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/)

---

## FAQ

**Q: Can we use pre-trained models for object detection?**  
A: Yes, but you must fine-tune on custom objects or demonstrate adaptation to your scenario.

**Q: What if our robot falls during the demo?**  
A: Falls are acceptable if the robot completes 7/10 trials successfully. Document failure modes in your report.

**Q: Can we use a wheeled robot instead of a humanoid?**  
A: No, bipedal locomotion is a core requirement. However, you can use simplified humanoid models.

**Q: How complex should the URDF be?**  
A: Minimum 12 DOF with proper mass/inertia. Complexity is rewarded but not required for full credit.

**Q: Can we work individually on a team project?**  
A: Yes, but team projects allow more ambitious scope. Solo projects evaluated with adjusted expectations.

**Q: What if Isaac Sim doesn't run on our hardware?**  
A: Use Gazebo or simplified Isaac Sim scenes. Contact instructor if issues persist.

---

## Getting Help

- **Office Hours**: Schedule 1-on-1 or team consultations
- **Project Check-ins**: Mandatory progress check at Day 5
- **Discord/Slack**: #capstone-project channel for questions
- **Peer Review**: Optional peer feedback sessions available

**Good luck building your autonomous humanoid!** 🤖

---

**Remember**: This project demonstrates everything you've learned. Focus on building a **working system** that integrates all concepts, rather than perfect individual components. Quality of integration matters more than theoretical perfection.


---
sidebar_position: 5
title: Assessments and Grading
---

# Assessments and Grading

## Assessment Philosophy

This course evaluates your ability to **build functional robotic systems**, not just understand theory. Assessments focus on:

1. **Working implementations** - Code that runs and demonstrates key concepts
2. **System integration** - Combining multiple ROS 2 components effectively
3. **Problem-solving** - Debugging and adapting to challenges
4. **Documentation** - Clear explanations of your design decisions

---

## Assessment Structure

| Module | Assessment Type | Weight | Due Week |
|--------|----------------|--------|----------|
| **Weeks 1-5** | ROS 2 Mini-Project | 20% | Week 5 |
| **Weeks 6-7** | Simulation Project | 15% | Week 7 |
| **Weeks 8-10** | NVIDIA Isaac Project | 20% | Week 10 |
| **Weeks 11-12** | Humanoid Control Project | 20% | Week 12 |
| **Week 13** | Capstone Project | 25% | Week 13 |

**Total**: 100%

---

## Week 5 Assessment: ROS 2 Mini-Project

### Project Requirements

**Objective**: Build a multi-node ROS 2 system that demonstrates core robotics concepts.

**Deliverables**:
1. **Source Code** - Complete ROS 2 package with:
   - At least 3 nodes (sensor, processor, actuator)
   - Topic-based communication between nodes
   - One service for configuration/commands
   - Launch file to start all nodes
   
2. **Documentation** - README.md with:
   - System architecture diagram
   - Node descriptions and interfaces
   - Build and run instructions
   - Sample output/screenshots

3. **Video Demo** - 2-3 minute screencast showing:
   - System startup
   - Data flow visualization (rqt_graph)
   - Example use case execution

### Suggested Topics
- **Mobile robot simulator** with odometry, navigation, and obstacle avoidance
- **Sensor fusion system** combining camera and IMU data
- **Robotic arm controller** with trajectory planning and execution
- **Distributed sensor network** with data aggregation

### Grading Rubric (100 points)

| Criterion | Excellent (A) | Good (B) | Satisfactory (C) | Needs Work (D/F) |
|-----------|--------------|----------|------------------|------------------|
| **Functionality (40pts)** | System works reliably, handles edge cases | Works for main use cases, minor issues | Basic functionality present, some bugs | Doesn't run or major issues |
| **Architecture (25pts)** | Well-designed, modular, follows ROS 2 best practices | Good structure, mostly follows conventions | Basic structure, some design issues | Poor structure, tightly coupled |
| **Documentation (20pts)** | Comprehensive, clear diagrams, easy to replicate | Good explanations, some details missing | Basic documentation, requires clarification | Missing key information |
| **Code Quality (15pts)** | Clean, commented, proper error handling | Readable, basic comments | Functional but messy | Hard to understand, no comments |

### Submission Guidelines
- **Format**: Git repository (GitHub/GitLab) with public access
- **Deadline**: End of Week 5 (Friday 11:59 PM)
- **Submit**: Repository URL via course platform
- **Late Policy**: -10% per day, max 3 days

---

## Week 7 Assessment: Simulation Project

### Project Requirements

**Objective**: Create a simulated robot environment in Gazebo with autonomous behavior.

**Deliverables**:
1. **Gazebo World** - Custom simulation environment with:
   - At least 3 obstacles/objects
   - Robot model with sensors (camera, lidar, or IMU)
   - Physics-based interactions

2. **Autonomous Behavior** - Robot that performs:
   - SLAM (Simultaneous Localization and Mapping)
   - Path planning to goal locations
   - Obstacle avoidance

3. **Analysis Report** - PDF document (3-5 pages) with:
   - System design and sensor choices
   - Algorithm explanations (SLAM, navigation)
   - Performance metrics (success rate, time to goal)
   - Challenges faced and solutions

### Grading Rubric (100 points)

| Criterion | Excellent (A) | Good (B) | Satisfactory (C) | Needs Work (D/F) |
|-----------|--------------|----------|------------------|------------------|
| **Simulation Setup (25pts)** | Complex environment, realistic physics | Good environment, basic physics | Simple environment, works | Environment doesn't load |
| **Autonomous Navigation (35pts)** | Reliable navigation, handles dynamic obstacles | Navigates successfully most times | Basic navigation, some failures | Doesn't navigate or frequent crashes |
| **SLAM Quality (20pts)** | Accurate map, handles loop closure | Good map with minor errors | Basic map, significant drift | No map or unusable |
| **Report (20pts)** | Thorough analysis, clear metrics | Good explanations, results shown | Basic documentation | Missing critical information |

### Submission Guidelines
- **Format**: Git repository + PDF report
- **Deadline**: End of Week 7 (Friday 11:59 PM)
- **Include**: Recorded video (3-5 min) of robot navigating

---

## Week 10 Assessment: NVIDIA Isaac Sim Project

### Project Requirements

**Objective**: Develop a perception-based manipulation task using NVIDIA Isaac Sim.

**Deliverables**:
1. **Isaac Sim Scene** - Simulated environment with:
   - Robotic manipulator (UR5, Franka, or similar)
   - Objects to manipulate (at least 3)
   - RGB-D camera for perception

2. **Perception Pipeline** - Computer vision system that:
   - Detects objects using deep learning (YOLO, PointNet, etc.)
   - Estimates 6D poses
   - Handles occlusion and lighting variations

3. **Manipulation Task** - Robot performs:
   - Pick-and-place operations
   - Collision-free trajectory planning (MoveIt2)
   - Success rate over 70% over 10 trials

4. **Technical Report** - PDF (5-7 pages) covering:
   - Perception algorithm details
   - Motion planning approach
   - Experimental results with failure analysis

### Grading Rubric (100 points)

| Criterion | Excellent (A) | Good (B) | Satisfactory (C) | Needs Work (D/F) |
|-----------|--------------|----------|------------------|------------------|
| **Perception (30pts)** | Robust detection, accurate pose estimation | Good detection, minor pose errors | Basic detection, significant errors | Detection fails frequently |
| **Manipulation (30pts)** | Smooth motions, greater than 80% success rate | Works well, 60-80% success | Basic function, less than 60% success | Doesn't complete tasks |
| **Integration (20pts)** | Seamless perception-to-motion pipeline | Good integration, minor delays | Basic integration, noticeable lag | Poor integration or crashes |
| **Report (20pts)** | In-depth analysis, clear metrics, failure discussion | Good documentation, results shown | Basic report, minimal analysis | Incomplete or unclear |

### Submission Guidelines
- **Format**: Git repository + PDF report + demo video
- **Deadline**: End of Week 10 (Friday 11:59 PM)
- **Video**: 5-minute demonstration of perception + manipulation pipeline

---

## Week 12 Assessment: Humanoid Control Project

### Project Requirements

**Objective**: Implement locomotion or manipulation on a humanoid robot (sim or real).

**Deliverables**:
1. **URDF Model** - Complete humanoid description with:
   - At least 12 DOF (arms + legs)
   - Proper mass and inertia properties
   - Visual and collision geometry

2. **Control System** - Implement one of:
   - **Option A**: Bipedal walking (at least 5 consecutive steps)
   - **Option B**: Upper-body manipulation (reach, grasp, place)
   - **Option C**: Whole-body control (locomotion + manipulation)

3. **Stability Analysis** - Demonstrate:
   - ZMP (Zero Moment Point) tracking
   - Balance recovery from small pushes
   - Joint torque limits respected

4. **Project Report** - PDF (6-8 pages) with:
   - Kinematic and dynamic model description
   - Control algorithm details (PID, MPC, RL, etc.)
   - Experimental results with graphs
   - Video frames showing key phases

### Grading Rubric (100 points)

| Criterion | Excellent (A) | Good (B) | Satisfactory (C) | Needs Work (D/F) |
|-----------|--------------|----------|------------------|------------------|
| **Model Quality (20pts)** | Accurate URDF, realistic dynamics | Good model, minor inaccuracies | Basic model, some issues | Model doesn't load or unstable |
| **Control Performance (35pts)** | Robust, smooth motions, handles disturbances | Good performance, occasional issues | Basic function, frequent failures | Doesn't achieve task |
| **Stability (25pts)** | Excellent balance, ZMP within support polygon | Good stability, minor violations | Marginal stability | Falls or unstable |
| **Report (20pts)** | Comprehensive analysis, clear methodology | Good documentation, results explained | Basic report, minimal detail | Incomplete or unclear |

### Submission Guidelines
- **Format**: Git repository + PDF report + video demo
- **Deadline**: End of Week 12 (Friday 11:59 PM)
- **Video**: 3-5 minutes showing task execution and stability tests

---

## Week 13: Capstone Project

See [Capstone Project Details](week-13/05-final-project-guidelines.md) for complete requirements and rubric.

**Summary**: Design and implement an autonomous humanoid robot that combines:
- Perception (vision, depth sensing)
- Navigation (SLAM, path planning)
- Manipulation (grasping, placing)
- Locomotion (walking, balance control)

**Weight**: 25% of final grade  
**Team Size**: 1-3 students  
**Duration**: Weeks 13 (intensive focus)

---

## Grading Scale

| Letter Grade | Percentage | Description |
|--------------|-----------|-------------|
| **A** | 90-100% | Exceptional work, exceeds expectations |
| **B** | 80-89% | Good work, meets all requirements |
| **C** | 70-79% | Satisfactory work, meets basic requirements |
| **D** | 60-69% | Below expectations, significant issues |
| **F** | Below 60% | Unsatisfactory, does not meet minimum standards |

---

## General Submission Guidelines

### Repository Structure
```
project-name/
├── README.md               # Project overview and instructions
├── package.xml             # ROS 2 package metadata
├── CMakeLists.txt          # Build configuration (if C++)
├── setup.py                # Setup file (if Python)
├── launch/                 # Launch files
│   └── demo.launch.py
├── src/                    # Source code
│   ├── node1.py
│   └── node2.py
├── config/                 # Configuration files (YAML, JSON)
├── worlds/                 # Gazebo world files (if applicable)
├── urdf/                   # Robot descriptions
├── docs/                   # Additional documentation
│   ├── architecture.md
│   └── diagrams/
└── media/                  # Screenshots, videos
    └── demo.mp4
```

### Video Requirements
- **Format**: MP4, MOV, or WEBM
- **Resolution**: Minimum 720p
- **Duration**: As specified per assessment
- **Audio**: Include narration explaining what's happening
- **Host**: YouTube (unlisted), Google Drive, or Vimeo

### Code Standards
- **Python**: Follow PEP 8 style guide
- **C++**: Follow ROS 2 C++ style guide
- **Comments**: Explain non-obvious logic
- **Error Handling**: Include try/except blocks for critical operations
- **Logging**: Use ROS 2 logging (get_logger().info(), etc.)

### Documentation Standards
- **README.md**: Must include:
  - Project description and goals
  - Prerequisites and dependencies
  - Build instructions
  - Run instructions with example commands
  - Troubleshooting common issues
- **Code comments**: Docstrings for all functions/classes
- **Diagrams**: Use draw.io, Lucidchart, or similar
- **Report PDFs**: Use proper headings, figures with captions, references

---

## Academic Integrity

- **Collaboration**: Permitted for learning, but all submitted code must be your own
- **External Code**: You may use open-source libraries, but cite all sources
- **AI Tools**: GitHub Copilot and ChatGPT are permitted as coding assistants, but you must understand all code you submit
- **Plagiarism**: Copying code from classmates or online sources without attribution results in automatic zero

---

## Late Submission Policy

- **First 24 hours**: -10% penalty
- **24-48 hours**: -20% penalty
- **48-72 hours**: -30% penalty
- **Beyond 72 hours**: Zero credit unless prior arrangement with instructor

**Extensions**: Available for medical emergencies or documented circumstances. Contact instructor at least 48 hours before deadline.

---

## Getting Help

- **Office Hours**: See course syllabus for schedule
- **Discord/Slack**: Use course channel for questions (response within 24 hours)
- **Email**: For private matters only (allow 48 hours for response)
- **Tutoring**: Peer tutoring available, schedule through course platform

---

## Assessment Tips

### For Success:
1. **Start early** - Don't wait until the last week
2. **Test incrementally** - Verify each component before integration
3. **Document as you go** - Don't leave documentation until the end
4. **Use version control** - Commit frequently with meaningful messages
5. **Ask questions** - Use office hours and discussion forums
6. **Record videos early** - Technical issues can delay final submission

### Common Mistakes to Avoid:
- ❌ Overly complex designs that are hard to debug
- ❌ Not testing on clean system (missing dependencies)
- ❌ Poor git practices (single massive commit)
- ❌ Waiting until deadline to record video
- ❌ Not reading assignment requirements carefully
- ❌ Hardcoding paths specific to your machine

---

## Frequently Asked Questions

**Q: Can I use a different simulator than Gazebo?**  
A: For Week 7, yes (Unity, Webots, PyBullet). For Week 10, NVIDIA Isaac Sim is required.

**Q: What if my robot keeps falling in simulation?**  
A: Start with static balance tests, then add motion. Use lower gains for PID control. Check mass/inertia properties.

**Q: Can I work in a team?**  
A: Weeks 1-12 are individual. Capstone project allows teams of 1-3.

**Q: What hardware do I need?**  
A: See [Hardware Requirements](appendix/hardware-recommendations.md). Minimum: laptop with 8GB RAM for simulation.

**Q: How do I submit if my repository is private?**  
A: Either make it public or add instructor as collaborator.

---

## Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Tutorials](https://gazebosim.org/docs)
- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/humble/index.html)
- [Git Best Practices](https://www.git-tower.com/learn/git/ebook/en/command-line/appendix/best-practices)

For additional support, visit the course discussion forum or attend office hours.

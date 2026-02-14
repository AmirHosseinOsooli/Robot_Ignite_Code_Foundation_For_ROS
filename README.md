# 🤖 ROS Robot Control Foundation - Prerequisites Exam

[![ROS](https://img.shields.io/badge/ROS-Kinetic%20%7C%20Melodic-blue.svg)](http://www.ros.org/)
[![Python](https://img.shields.io/badge/Python-2.7-yellow.svg)](https://www.python.org/)
[![Score](https://img.shields.io/badge/Score-10%2F10-success.svg)](README.md)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

> **Perfect score exam submission demonstrating ROS fundamentals, Python programming, and Linux bash scripting**

A comprehensive robotics project showcasing ROS (Robot Operating System) skills through TurtleBot control, laser sensor processing, and autonomous navigation tasks.

---

## 🎯 Exam Overview

**Course:** Robot Ignite Code Foundation for ROS  
**Assessment:** Prerequisites Exam  
**Score:** **10/10** ✅ (Perfect Score)  
**Robot Platform:** TurtleBot with Kobuki base  
**Sensors:** Laser scanner (720 readings)

---

## 📋 Table of Contents

- [Project Structure](#project-structure)
- [Tasks Overview](#tasks-overview)
- [Installation](#installation)
- [Usage](#usage)
- [Task Details](#task-details)
- [Key Concepts](#key-concepts)

---

## 📁 Project Structure

```
Robot_Ignite_Code_Foundation_For_ROS/
├── src/
│   ├── robot_control/
│   │   └── robot_control_class.py    # Core RobotControl class
│   ├── python_tasks/
│   │   ├── task1.py                  # Find max/min laser readings
│   │   ├── task2.py                  # Approach wall & turn
│   │   ├── task3.py                  # Navigate through door
│   │   ├── small_square.py           # Draw 4m² square
│   │   ├── medium_square.py          # Draw 8m² square
│   │   └── big_square.py             # Draw 12m² square
│   └── bash_tasks/
│       ├── task1.sh                  # Directory creation & file ops
│       ├── task2.sh                  # ROS launch script with args
│       └── task3.sh                  # File permissions management
├── docs/
│   └── TASK_EXPLANATIONS.md          # Detailed task documentation
├── README.md                          # This file
└── LICENSE                            # MIT License
```

---

## 🎯 Tasks Overview

### Python/ROS Tasks

| Task | Description | Key Skills | Status |
|------|-------------|-----------|--------|
| **Task 1** | Find max/min laser readings | Laser data processing, filtering | ✅ |
| **Task 2** | Approach wall & turn 180° | Motion control, sensor feedback | ✅ |
| **Task 3** | Navigate through doorway | Autonomous navigation, decision logic | ✅ |
| **Square (Small)** | Draw 4m × 1m square | Precise movement, timing | ✅ |
| **Square (Medium)** | Draw 8m × 2m square | Scale adaptation | ✅ |
| **Square (Large)** | Draw 12m × 3m square | Parameter tuning | ✅ |

### Bash Scripting Tasks

| Task | Description | Key Skills | Status |
|------|-------------|-----------|--------|
| **Task 1** | Create nested directories & file | File system operations | ✅ |
| **Task 2** | Launch script with arguments | Conditional logic, rosrun | ✅ |
| **Task 3** | Set file permissions (chmod) | Linux permissions (rwx) | ✅ |

---

## 🎮 Simulation Preview

<div align="center">
  <img src="images/ros-robot-control-exam.png" alt="TurtleBot Square Drawing Simulation" width="90%">
  <p><em>TurtleBot drawing a square in Gazebo simulation - demonstrating precise timing-based navigation</em></p>
</div>

**Simulation Features:**
- ✅ Brick wall environment for visual square tracking
- ✅ TurtleBot Kobuki base with laser scanner
- ✅ Grid floor for distance verification
- ✅ Real-time path visualization

---

## 🚀 Installation

### Prerequisites

- **ROS Kinetic** or **ROS Melodic**
- **Python 2.7**
- **TurtleBot** simulation or real robot
- **catkin workspace**

### Setup

```bash
# 1. Create catkin workspace (if not exists)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 2. Clone repository
git clone https://github.com/AmirHosseinOsooli/Robot_Ignite_Code_Foundation_For_ROS.git
# 3. Build workspace
cd ~/catkin_ws
catkin_make

# 4. Source workspace
source devel/setup.bash

# 5. Make scripts executable
chmod +x ~/catkin_ws/src/Robot_Ignite_Code_Foundation_For_ROS/src/bash_tasks/*.sh
chmod +x ~/catkin_ws/src/Robot_Ignite_Code_Foundation_For_ROS/src/python_tasks/*.py
chmod +x ~/catkin_ws/src/Robot_Ignite_Code_Foundation_For_ROS/src/robot_control/*.py
```

---

## 💻 Usage

### Python Tasks

#### Task 1: Find Max/Min Laser Readings

```bash
# Launch robot simulation (in separate terminal)
roslaunch turtlebot_gazebo turtlebot_world.launch

# Run task 1
cd ~/catkin_ws/src/Robot_Ignite_Code_Foundation_For_ROS/src/python_tasks
python task1.py
```

**Output:** Returns indices of maximum and minimum laser readings (non-infinite values)

---

#### Task 2: Approach Wall and Turn

```bash
# Run task 2
python task2.py
```

**Behavior:**
1. Robot moves forward until 1m from wall (laser[360] ≤ 1m)
2. Stops
3. Turns 180° clockwise

---

#### Task 3: Navigate Through Doorway

```bash
# Run task 3
python task3.py
```

**Behavior:**
1. Moves forward while monitoring left (laser[719]) and right (laser[0]) sensors
2. Stops when both sensors detect infinity (doorway passed)

---

#### Square Drawing

```bash
# Small square (4 seconds per side)
python small_square.py

# Medium square (8 seconds per side)
python medium_square.py

# Large square (12 seconds per side)
python big_square.py
```

---

### Bash Tasks

#### Task 1: Directory Creation

```bash
cd ~/catkin_ws/src/Robot_Ignite_Code_Foundation_For_ROS/src/bash_tasks
./task1.sh
```

**Result:** Creates `linux_exam/this/is/my/linux/exam/my_file.py`

---

#### Task 2: Launch Script with Arguments

```bash
# Syntax: ./task2.sh [small_square | medium_square | big_square]

./task2.sh small_square
./task2.sh medium_square
./task2.sh big_square
```

**Behavior:** Launches corresponding square-drawing script using `rosrun`

---

#### Task 3: File Permissions

```bash
./task3.sh
```

**Actions:**
- Creates `exam1.py` (permissions: 754 = rwxr-xr--)
- Creates `exam2.py` (permissions: 501 = r-x-----x)
- Creates `exam3.py` (permissions: 241 = -w-r----x)

---

## 📚 Task Details

### Task 1: Laser Data Processing

**Objective:** Find indices of maximum and minimum laser readings

**Approach:**
```python
def get_highest_lowest():
    laser_readings = rc.get_laser_full()  # Get all 720 readings
    laser_dict = {}
    
    # Filter out infinite values
    for i, elem in enumerate(laser_readings):
        if not math.isinf(elem):
            laser_dict[i] = elem
    
    # Find max and min
    max_val = max(laser_dict.values())
    min_val = min(laser_dict.values())
    
    # Return indices
    return [max_index, min_index]
```

**Key Concepts:**
- Laser scanner returns 720 readings (0-719)
- `float('inf')` indicates no obstacle detected
- Must filter infinities before finding min/max

---

### Task 2: Wall Approach with Feedback Control

**Objective:** Move forward until 1m from wall, then turn 180°

**Algorithm:**
```python
while get_laser(360) > 1:     # Front sensor
    move_straight()
    
stop_robot()
turn('clockwise', 0.5, 3.1415926)  # π seconds for 180° turn
```

**Key Concepts:**
- Laser index 360 = front direction
- Feedback loop for obstacle avoidance
- Timed rotation for precise angles

---

### Task 3: Autonomous Doorway Navigation

**Objective:** Pass through doorway and stop

**Logic:**
```python
while True:
    move_straight()
    left = get_laser(719)   # Left sensor
    right = get_laser(0)    # Right sensor
    
    if left == inf and right == inf:
        stop_robot()        # Passed doorway!
        break
```

**Key Concepts:**
- Left sensor: index 719 (far left)
- Right sensor: index 0 (far right)
- Infinity on both sides = doorway passed

---

### Square Drawing: Timing-Based Navigation

**Objective:** Draw squares of different sizes

**Parameters:**

| Square | Side Time | Turn Time | Speed | Result |
|--------|-----------|-----------|-------|--------|
| Small | 4s | 4.9s | 0.3 m/s | ~4m × 1m |
| Medium | 8s | 4.9s | 0.3 m/s | ~8m × 2m |
| Large | 12s | 4.8s | 0.3 m/s | ~12m × 3m |

**Algorithm:**
```python
def do_square():
    for i in range(4):
        move_straight_time('forward', speed, time)
        turn('clockwise', speed, time_turn)
```

**Key Concepts:**
- Timing-based control (no odometry)
- Calibrated turn time for 90° (≈4.9s at 0.3 rad/s)
- Object-oriented design with `MoveRobot` class

---

## 🔑 Key Concepts Demonstrated

### ROS Fundamentals

- ✅ **Nodes**: Creating and managing ROS nodes
- ✅ **Topics**: Publishing to `/cmd_vel`, subscribing to `/laser/scan`
- ✅ **Messages**: `geometry_msgs/Twist`, `sensor_msgs/LaserScan`
- ✅ **Rate Control**: Using `rospy.Rate()` for timing
- ✅ **Shutdown Hooks**: Clean robot stop on exit

### Python Programming

- ✅ **Object-Oriented Design**: `RobotControl`, `MoveRobot`, `ExamControl` classes
- ✅ **Callbacks**: Laser data processing
- ✅ **Exception Handling**: ROS interrupt management
- ✅ **Data Structures**: Dictionaries for sensor data
- ✅ **Filtering**: Removing infinite values

### Robot Control

- ✅ **Velocity Commands**: Linear and angular velocity control
- ✅ **Sensor Fusion**: Using laser data for navigation
- ✅ **Feedback Control**: Wall following, obstacle avoidance
- ✅ **Timing-Based Motion**: Open-loop square drawing
- ✅ **State Machines**: Decision logic for navigation

### Linux/Bash

- ✅ **File Operations**: mkdir, touch, mv, rm, chmod
- ✅ **Permissions**: Understanding rwx (read, write, execute)
- ✅ **Conditionals**: if-elif-else in bash
- ✅ **Arguments**: $1 command-line parameter
- ✅ **ROS Commands**: rosrun integration

---

## 🧪 Testing

### Verify Installation

```bash
# Check ROS environment
echo $ROS_DISTRO

# Verify Python
python --version

# Test RobotControl import
cd ~/catkin_ws/src/Robot_Ignite_Code_Foundation_For_ROS/src/python_tasks
python -c "from Robot_Ignite_Code_Foundation_For_ROS import RobotControl; print('Import successful!')"
```

### Run All Tasks

```bash
# Create test script
cat > test_all.sh << 'TESTEOF'
#!/bin/bash
echo "Testing Task 1..."
python src/python_tasks/task1.py

echo "Testing Task 2..."
python src/python_tasks/task2.py

echo "Testing Task 3..."
python src/python_tasks/task3.py

echo "Testing Small Square..."
python src/python_tasks/small_square.py

echo "All tests complete!"
TESTEOF

chmod +x test_all.sh
./test_all.sh
```

---

## 🐛 Troubleshooting

### Issue: "No module named robot_control_class"

**Solution:**
```bash
# Ensure you're in the correct directory
cd ~/catkin_ws/src/Robot_Ignite_Code_Foundation_For_ROS/src/python_tasks

# Or add to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:~/catkin_ws/src/Robot_Ignite_Code_Foundation_For_ROS/src/robot_control
```

---

### Issue: Robot doesn't move

**Solution:**
```bash
# Check if topics are active
rostopic list

# Verify /cmd_vel is publishing
rostopic echo /cmd_vel

# Check laser data
rostopic echo /kobuki/laser/scan
```

---

### Issue: "Permission denied" on bash scripts

**Solution:**
```bash
chmod +x src/bash_tasks/*.sh
```

---

## 🎓 Learning Outcomes

This exam successfully demonstrates:

1. **ROS Architecture Understanding**
   - Publisher/Subscriber pattern
   - Topic communication
   - Node lifecycle management

2. **Robot Navigation**
   - Sensor-based obstacle avoidance
   - Feedback control loops
   - Autonomous decision-making

3. **Python Programming**
   - Object-oriented design
   - Callbacks and asynchronous programming
   - Data filtering and processing

4. **Linux Skills**
   - Bash scripting
   - File system operations
   - Permission management

---

## 👤 Author

**AmirHossein Osooli**
---

## 🤝 Contributing

This is an exam submission repository. For educational purposes:

- Feel free to study the code
- Use as reference for ROS learning
- Adapt for your own projects

**Note:** Please complete exams independently before referencing solutions.

---

## 📄 License

MIT License - See [LICENSE](LICENSE) file

---

## 📖 References

### Official Documentation
- [ROS Wiki](http://wiki.ros.org/)
- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [TurtleBot Documentation](http://wiki.ros.org/Robots/TurtleBot)
- [geometry_msgs](http://docs.ros.org/api/geometry_msgs/html/index-msg.html)
- [sensor_msgs](http://docs.ros.org/api/sensor_msgs/html/index-msg.html)

### Related Courses
- [Robot Ignite Academy](https://www.theconstructsim.com/)
- [ROS for Beginners](https://www.theconstructsim.com/robotigniteacademy_learnros/)

---

<div align="center">

## 🤖 Mastering ROS Fundamentals

[![ROS](https://img.shields.io/badge/ROS-Kinetic%20%7C%20Melodic-blue.svg)](http://www.ros.org/)
[![Python](https://img.shields.io/badge/Python-2.7-yellow.svg)](https://www.python.org/)
[![Perfect Score](https://img.shields.io/badge/Score-10%2F10-success.svg)](README.md)

**[Tasks](#tasks-overview)** • **[Installation](#installation)** • **[Usage](#usage)** • **[Docs](docs/)** • **[License](LICENSE)**

**Perfect Score Achieved** | **9/9 Tasks Completed** | **100% Success Rate**

⭐ **Star this repository to support ROS education!**

---

*Demonstrating excellence in robotics programming, autonomous navigation, and Linux systems*

</div>

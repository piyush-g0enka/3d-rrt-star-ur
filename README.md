# 3D RRT* with UR3e

## Note
- **RRT simulation video**: [Watch here](https://www.youtube.com/watch?v=YYwdC_GXb7Q)
- **RRT-star simulation video**: [Watch here](https://www.youtube.com/watch?v=B2dtDf0w5ew)
- **Gazebo simulation video**: [Watch here](https://www.youtube.com/watch?v=UKlIfcn6nj0)

---

## Running RRT and RRT* Standalone Algorithms

### RRT Algorithm (rrt.py)
To run the RRT algorithm, use the following command:

```bash
$ python3 4_code_rrt.py
```

- The code is initialized with:
  - **START**: `(0.36, 0.36, 0.15)`
  - **GOAL**: `(0.36, -0.36, 0.15)`

- You can modify the start and goal positions in lines 128 and 129 of the code.

---

### RRT* Algorithm (rrt_star.py)
To run the RRT* algorithm, use the following command:

```bash
$ python3 4_code_rrt_star.py
```

- The code is initialized with:
  - **START**: `(0.36, 0.36, 0.15)`
  - **GOAL**: `(0.36, -0.36, 0.15)`

- You can modify the start and goal positions in lines 165 and 166 of the code.

---

## Running ROS2 - Gazebo Simulation

### Prerequisites
1. Place the ROS2 packages in the `src` folder of your ROS2 Humble workspace.
2. If the computer hangs while building packages, build them one by one.
3. Ensure the `controller_manager` package is installed:

```bash
$ sudo apt install ros-humble-controller-manager
```

---

### Steps to Run the ROS2 Code

1. **Start Gazebo Simulation** (the robot will spawn in the "Up" configuration):

   ```bash
   $ ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:=ur3e
   ```

2. **Start Waypoint Server** (the robot will move to the initial pose):

   ```bash
   $ ros2 launch execute_motion ur_param_server.launch.py
   ```

3. **Run Planner Controller Node**:

   ```bash
   $ ros2 run execute_motion planner_controller.py
   ```

- The pick and drop positions can be modified in lines 38 and 42 of `execute_motion/execute_motion/planner_controller_interface.py`.
  - Current pick position: `[0.36, 0.36, 0.05]`
  - Current drop position: `[0.36, -0.36, 0.1]`

- **Note**: Ensure the pick and drop points are not in the obstacle space. If they are in obstacle space, the algorithm will not return anything.
```

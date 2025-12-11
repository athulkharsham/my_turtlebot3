# 8-Week Mastery Plan: Modern Factor Graph SLAM with Nova Carter in Isaac Sim

This comprehensive 8-week plan is structured with specific tutorials, academic courses, and practical milestones to take you from theory to a working factor-based SLAM system on your simulated Nova Carter robot using GTSAM and Kimera-VIO.

## Phase 1: Theory & GTSAM Fundamentals (Weeks 1-2)
**Goal:** Understand the math behind factor graphs and run your first GTSAM optimization.

### Week 1: The Mathematics of Factor Graphs

**Primary Course (Video Series):**
*   **Professor:** Cyrill Stachniss (University of Bonn)
*   **Watch:**
    1.  [Graph-based SLAM with Pose Graphs](https://www.youtube.com/watch?v=uHbRKvD8TWg) - *Essential for understanding nodes/edges.*
    2.  [Graph-Based SLAM with Landmarks](https://www.youtube.com/watch?v=mZBdPgBtrCM) - *Bridges the gap to visual SLAM.*
    3.  [Least Squares & Gauss-Newton](https://www.youtube.com/watch?v=A8vN0rK8cwc) - *The solver backend math.*

**Core Reading:**
*   **Paper:** "Factor Graphs for Robot Perception" (Dellaert & Kaess) - Read Chapters 1-3.
*   **Tutorial PDF:** Download the [GTSAM 4.0 Tutorial](https://dongjing3309.github.io/files/gtsam-tutorial.pdf). Read pages 1-20 (Factor Graphs, Nonlinear Least Squares).

### Week 2: Hands-on GTSAM Programming

**Video Tutorial:**
*   **Watch:** "A Technical Walkthrough of the SLAM Back-end" by Air Lab CMU.
*   **Focus:** Timestamps 21:45 (Gauss Newton) and 1:07:01 (2D SLAM Example).

**Coding Assignments:**
1.  **Clone:** `git clone https://github.com/gtbook/gtsam-examples`
2.  **Run:** `SimpleRotation.py` and `PlanarSLAMExample.py`.
3.  **Exercise:** Modify the `PlanarSLAMExample` to add high noise to one odometry factor and observe how the graph distorts vs. corrects when a loop closure factor is added.
4.  **C++ Deep Dive:** Study `CombinedImuFactorsExample.cpp` in the GTSAM repo. This file is **critical** for understanding how IMU preintegration works in code.

---

## Phase 2: Kimera-VIO Study (Weeks 3-4)
**Goal:** Master the specific architecture you will deploy.

### Week 3: Architecture & Installation

**Study Material:**
*   **Paper:** "Kimera: an Open-Source Library for Real-Time Metric-Semantic SLAM" (Rosinol et al.).
    *   *Focus:* Section III (VIO) and Section IV (Pose Graph Optimization).
*   **Video:** [Kimera Presentation at ICRA 2020](https://www.youtube.com/watch?v=e6fWfULKzto).

**Action Items:**
1.  **Install:** Set up Kimera-VIO-ROS using the [Docker container](https://github.com/MIT-SPARK/Kimera-VIO-ROS#docker) to avoid dependency hell.
2.  **Dataset Test:** Download a **EuRoC dataset** bag file (e.g., V1_01_easy).
3.  **Run:** Execute the standard launch file to see it working on "perfect" data.
4.  **Analyze:** Open `rqt_graph` while it runs. Trace the topics: `/camera/infra1/image_raw` -> `feature_tracker` -> `kimera_vio_ros`.

### Week 4: Configuration Mastery

**Task:** You must understand how to configure Kimera for *new* robots (like Nova Carter).

**Study Config Files:** Look inside `params/Euroc` folder in the repository.
*   `LeftCameraParams.yaml` / `RightCameraParams.yaml`: Intrinsics/Extrinsics.
*   `ImuParams.yaml`: Gyro/Accel noise density and random walk (Crucial for GTSAM factors).

**Exercise:** Create a dummy configuration folder named `NovaCarter` and copy the EuRoC files into it. We will fill these with real values in Week 6.

---

## Phase 3: Isaac Sim & Nova Carter (Weeks 5-6)
**Goal:** Generate valid sensor data from the simulated robot.

### Week 5: Nova Carter Simulation Setup

**Tutorials:**
*   **Official:** "ROS2 Cameras" in Isaac Sim Documentation.
*   **Specific:** "Exploring Autonomous Navigation with Isaac SIM and NVIDIA Carter" (YouTube).

**Action Items:**
1.  Launch Isaac Sim and load the **Nova Carter** asset.
2.  Enable the **ROS2 Bridge** extension.
3.  Verify the sensor topics using `ros2 topic list`. You should see:
    *   `/isaac_ros/left/image_rect` (or similar)
    *   `/isaac_ros/right/image_rect`
    *   `/isaac_ros/imu`

### Week 6: Sensor Tuning & Calibration

**The Critical Step:** Standard sims often output perfect data, which breaks VIO/SLAM that *expects* noise.

**Task:**
1.  **IMU Noise:** In Isaac Sim, find the IMU prim. Add Gaussian noise to the accelerometer and gyroscope settings to match the [Bosch BMI088 datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/) (used in Nova Carter).
2.  **Camera Rate:** Ensure cameras are publishing at **20Hz+** and IMU at **200Hz+**. GTSAM needs high-frequency IMU data for preintegration.
3.  **TFs:** Run `ros2 run tf2_tools view_frames`. Ensure there is a connected tree from `base_link` -> `imu_link` and `base_link` -> `camera_left`.

---

## Phase 4: Integration (Weeks 7-8)
**Goal:** Connect the simulator to the SLAM backend.

### Week 7: The Bridge (Remapping)

**Challenge:** Kimera expects topics like `/cam0/image_raw`, but Isaac gives `/front_stereo_camera/left/image_raw`.

**Solution:** Create a custom ROS2 launch file `nova_slam.launch.py`.
*   Use `<remap>` tags to route Isaac topics to Kimera inputs.
*   **Example Remap:**
    ```python
    remappings=[
        ('left_cam/image_raw', '/front_stereo_camera/left/image_raw'),
        ('right_cam/image_raw', '/front_stereo_camera/right/image_raw'),
        ('imu', '/sensors/imu')
    ]
    ```

### Week 8: Tuning & Loop Closure

**Execution:**
1.  Launch Isaac Sim (Nova Carter).
2.  Launch your `nova_slam.launch.py`.
3.  Open **RViz2**. Visualize the `odometry` topic from Kimera.

**Troubleshooting:**
*   *Drifting wildly?* Check your IMU extrinsics (rotation matrix between IMU and Camera) in the config yaml. This is the #1 cause of VIO failure.
*   *Graph optimization slow?* Reduce the number of features in Kimera's `FrontendParams.yaml` (default is often 300-500; try 200).

**Final Exam:** Drive the Nova Carter in a loop in Isaac Sim. Verify that when you return to the start, the trajectory "snaps" together (Loop Closure).

---

## Summary of Resources

| Resource Type | Name/Link | Purpose |
| :--- | :--- | :--- |
| **Course** | [Cyrill Stachniss: Graph-Based SLAM](https://www.youtube.com/watch?v=uHbRKvD8TWg) | Theory Foundation |
| **Tutorial** | [GTSAM 4.0 PDF](https://dongjing3309.github.io/files/gtsam-tutorial.pdf) | Programming Reference |
| **Code** | [GTSAM Examples Repo](https://github.com/gtbook/gtsam-examples) | Code snippets |
| **Sim Tool** | [Isaac Sim ROS2 Camera Tutorial](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/ros2_tutorials/tutorial_ros2_camera.html) | Sensor Setup |
| **Hardware Info** | [Nova Carter Datasheet](https://robotics.segway.com/wp-content/uploads/2023/11/Nova-Carter-Product-Manual-v1.0.pdf) | Config parameters |

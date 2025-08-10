# ekf_localizer

## Overview

`ekf_localizer` is a C++ project that estimates the state of a robot (position, velocity, and attitude) by fusing data from an Inertial Measurement Unit (IMU) and Odometry using an Extended Kalman Filter (EKF).

This project is a standalone executable that does not depend on specific middleware like ROS. It is implemented in pure C++ using the Eigen library. It processes sensor data from CSV files and saves the filtered results to a CSV file, designed for easy analysis.

## Core Features

-   **State Prediction (Prediction Step)**: Predicts the next state of the robot using acceleration and angular velocity data from the IMU.
-   **State Correction (Correction Step)**: Corrects the predicted state and reduces uncertainty using linear/angular velocity measurements from Odometry.
-   **Data Synchronization and Loading**: Synchronizes and loads IMU and Odometry data, stored in multiple CSV files, based on timestamps.

## Dependencies

The following libraries are required to build and run this project:

-   **CMake** (version 3.14 or higher)
-   A compiler supporting **C++17** (e.g., GCC, Clang)
-   **Eigen3** (version 3.3 or higher)

## Build Instructions

Follow these steps to build the project:

1.  Create a `build` directory in the project root and navigate into it.

    ```bash
    mkdir build
    cd build
    ```

2.  Run `cmake` to generate the build files.

    ```bash
    cmake ..
    ```

3.  Use `make` to compile the source code.

    ```bash
    make
    ```

    After a successful build, the `ekf_localizer` executable will be created in the `build` directory.

## Data Preparation

To run the EKF, sensor data in a specific format is required. The data must be provided in CSV files, where each file consists of two columns: **timestamp** and a **single value**.

**Required CSV Files:**

The program currently reads files from the hardcoded path `src/ekf_localizer/source`. Please ensure the files are available at this location before running.

-   **IMU Data:**
    -   `imu_linear_acc_x.csv`, `imu_linear_acc_y.csv`, `imu_linear_acc_z.csv`
    -   `imu_angular_vel_x.csv`, `imu_angular_vel_y.csv`, `imu_angular_vel_z.csv`
    -   `imu_orientation_x.csv`, `imu_orientation_y.csv`, `imu_orientation_z.csv`, `imu_orientation_w.csv`
-   **Odometry Data:**
    -   `odom_position_x.csv`, `odom_position_y.csv`, `odom_position_z.csv`
    -   `odom_linear_vel_x.csv`, `odom_linear_vel_y.csv`, `odom_linear_vel_z.csv`
    -   `odom_angular_vel_x.csv`, `odom_angular_vel_y.csv`, `odom_angular_vel_z.csv`
    -   `odom_orientation_x.csv`, `odom_orientation_y.csv`, `odom_orientation_z.csv`, `odom_orientation_w.csv`

**Using Example Datasets:**

The `src/ekf_localizer/source_examples` directory contains different datasets to test the EKF. To use one of them, you need to copy the contents of a chosen dataset folder into the `src/ekf_localizer/source` directory.

For example, to use the `slope_map_b1_2` dataset, follow these steps from the `ekf_localizer` directory:

1.  First, clear the current data from the `source` directory:
    ```bash
    rm -f src/source/*.csv
    ```

2.  Then, copy the new dataset into the `source` directory:
    ```bash
    cp src/source_examples/slope_map_b1_2/* src/source/
    ```

Now you can run the simulation with the new dataset.

## How to Run

This project provides a convenient script `get_result.sh` to compile, run the EKF, and visualize the results.

### 1. Visualization Tool Setup

Before running the main script, you need to install the Python dependencies for the visualization tool.

Navigate to the `build` directory and install the required packages using `pip` and `requirements.txt`:

```bash
cd build
pip install -r requirements.txt
```

### 2. Running the Simulation and Visualization

From the `build` directory, execute the `get_result.sh` script:

```bash
./get_result.sh
```

This script performs the following actions:
1.  **Compiles the code**: Runs `make` to ensure the `ekf_localizer` executable is up-to-date.
2.  **Cleans up old results**: Removes any existing `ekf_result.csv` file.
3.  **Runs the EKF executable**: Executes `./ekf_localizer` to generate a new `ekf_result.csv` file.
4.  **Launches the visualization tool**: Starts `result_visualization.py` to display the results graphically.

### Manual Execution

Alternatively, you can run the components manually.

**Run the EKF localizer:**
From the `build` directory, run:
```bash
./ekf_localizer
```
Note: The input data path and output file path are currently hardcoded in `src/main.cpp`.
-   **Input Path**: `/Users/yeonge/workspace/share/humble/ekf-ws/src/ekf_localizer/source`
-   **Output File**: `/Users/yeonge/workspace/share/humble/ekf-ws/src/ekf_localizer/build/ekf_result.csv`

**Run the visualization tool:**
After `ekf_result.csv` is generated, run the Python script from the `build` directory:
```bash
python3 result_visualization.py
```

## Output

When the program finishes, an `ekf_result.csv` file is generated in the `build` directory. This file contains the EKF estimation results and the original sensor data for each timestamp.

**CSV File Column Descriptions:**

-   `timestamp`: Timestamp of the synchronized sensor data (seconds).
-   `ekf_pos_x, ekf_pos_y, ekf_pos_z`: Position estimated by EKF (m).
-   `ekf_vel_x, ekf_vel_y, ekf_vel_z`: Velocity estimated by EKF (m/s).
-   `ekf_roll, ekf_pitch, ekf_yaw`: Euler angles estimated by EKF (rad).
-   `ekf_ang_vel_x, ekf_ang_vel_y, ekf_ang_vel_z`: Angular velocity estimated by EKF (rad/s).
-   `odom_pos_x, odom_pos_y, odom_pos_z`: Original position from Odometry (m).
-   `odom_vel_x, odom_vel_y, odom_vel_z`: Original velocity from Odometry (m/s).
-   `odom_roll, odom_pitch, odom_yaw`: Original Euler angles from Odometry (rad).
-   `odom_ang_vel_x, odom_ang_vel_y, odom_ang_vel_z`: Original angular velocity from Odometry (rad/s).
-   `imu_acc_x, imu_acc_y, imu_acc_z`: Original acceleration from IMU (m/s^2).
-   `imu_ang_vel_x, imu_ang_vel_y, imu_ang_vel_z`: Original angular velocity from IMU (rad/s).
-   `imu_roll, imu_pitch, imu_yaw`: Original Euler angles from IMU (rad).

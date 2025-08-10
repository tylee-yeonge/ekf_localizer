#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EKF Simulation Results Visualization Tool
Visualizing each component in separate tabs
"""

import sys
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib.style as style
from mpl_toolkits.mplot3d import Axes3D
import glob

# Font settings
plt.rcParams["font.family"] = "DejaVu Sans"
plt.rcParams["font.size"] = 10
plt.rcParams["axes.unicode_minus"] = False


class EKFResultVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title("EKF Simulation Results Visualization")
        self.root.geometry("1400x900")

        # Data storage variables
        self.data = None
        self.raw_data = {}
        self.csv_files_found = []
        self.raw_data_files_found = []
        self.loaded_files = []
        self.failed_files = []
        self.csv_file = "ekf_result.csv"  # Changed from simulation_results.csv

        # Style settings
        style.use("default")

        # Setup GUI
        self.setup_gui()

        # Try to load data at startup
        self.load_data()
        self.load_raw_data()

    def setup_gui(self):
        """Setup GUI components"""
        # Menu bar
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Open CSV File", command=self.open_file)
        file_menu.add_command(label="Refresh", command=self.refresh_all_data)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

        # Status bar
        self.status_var = tk.StringVar()
        self.status_var.set("Ready")
        status_bar = tk.Label(
            self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W
        )
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

        # Create tab widget
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # Create tabs
        self.create_tabs()

    def create_tabs(self):
        """Create tabs"""
        self.tabs = {}

        # 1. Raw Data tab
        self.tabs["raw_data"] = ttk.Frame(self.notebook)
        self.notebook.add(self.tabs["raw_data"], text="Raw Data")

        # 2. IMU Data tab (NEW)
        self.tabs["imu_data"] = ttk.Frame(self.notebook)
        self.notebook.add(self.tabs["imu_data"], text="IMU Data")

        # 3. Position tab
        self.tabs["position"] = ttk.Frame(self.notebook)
        self.notebook.add(self.tabs["position"], text="Position")

        # 4. Velocity tab
        self.tabs["velocity"] = ttk.Frame(self.notebook)
        self.notebook.add(self.tabs["velocity"], text="Velocity")

        # 5. Orientation tab
        self.tabs["orientation"] = ttk.Frame(self.notebook)
        self.notebook.add(self.tabs["orientation"], text="Orientation")

        # 6. Acceleration & Angular Velocity tab
        self.tabs["bias"] = ttk.Frame(self.notebook)
        self.notebook.add(self.tabs["bias"], text="Accel & Angular Vel")

        # 7. 2D Trajectory tab
        self.tabs["trajectory"] = ttk.Frame(self.notebook)
        self.notebook.add(self.tabs["trajectory"], text="2D Trajectory")

        # 8. Error Analysis tab
        self.tabs["error"] = ttk.Frame(self.notebook)
        self.notebook.add(self.tabs["error"], text="Error Analysis")

        # 9. Statistics tab
        self.tabs["statistics"] = ttk.Frame(self.notebook)
        self.notebook.add(self.tabs["statistics"], text="Statistics")

    def load_data(self):
        """Load CSV data and convert column names for compatibility"""
        try:
            if os.path.exists(self.csv_file):
                # Load the original data
                original_data = pd.read_csv(self.csv_file)

                # Create a copy for processing
                self.data = original_data.copy()

                # Map odom values to true values and ekf values to estimated values
                column_mapping = {
                    "timestamp": "time",
                    # Position mappings
                    "odom_pos_x": "true_x",
                    "odom_pos_y": "true_y",
                    "odom_pos_z": "true_z",
                    "ekf_pos_x": "est_x",
                    "ekf_pos_y": "est_y",
                    "ekf_pos_z": "est_z",
                    # Velocity mappings
                    "odom_vel_x": "true_vx",
                    "odom_vel_y": "true_vy",
                    "odom_vel_z": "true_vz",
                    "ekf_vel_x": "est_vx",
                    "ekf_vel_y": "est_vy",
                    "ekf_vel_z": "est_vz",
                    # Orientation mappings
                    "odom_roll": "true_roll",
                    "odom_pitch": "true_pitch",
                    "odom_yaw": "true_yaw",
                    "ekf_roll": "est_roll",
                    "ekf_pitch": "est_pitch",
                    "ekf_yaw": "est_yaw",
                    # Angular velocity mappings
                    "odom_ang_vel_x": "true_vroll",
                    "odom_ang_vel_y": "true_vpitch",
                    "odom_ang_vel_z": "true_vyaw",
                    "ekf_ang_vel_x": "est_vroll",
                    "ekf_ang_vel_y": "est_vpitch",
                    "ekf_ang_vel_z": "est_vyaw",
                    # IMU data mappings (NEW)
                    "imu_acc_x": "imu_ax",
                    "imu_acc_y": "imu_ay",
                    "imu_acc_z": "imu_az",
                    "imu_ang_vel_x": "imu_wx",
                    "imu_ang_vel_y": "imu_wy",
                    "imu_ang_vel_z": "imu_wz",
                    "imu_roll": "imu_roll",
                    "imu_pitch": "imu_pitch",
                    "imu_yaw": "imu_yaw",
                }

                # Use real IMU acceleration data if available, otherwise set to zero
                for axis in ["x", "y", "z"]:
                    imu_acc_col = f"imu_acc_{axis}"
                    if imu_acc_col in original_data.columns:
                        self.data[f"true_a{axis}"] = original_data[imu_acc_col]
                        self.data[f"est_a{axis}"] = original_data[
                            imu_acc_col
                        ]  # Use same IMU data for now
                    else:
                        self.data[f"true_a{axis}"] = 0.0
                        self.data[f"est_a{axis}"] = 0.0

                # Rename columns
                for old_col, new_col in column_mapping.items():
                    if old_col in self.data.columns:
                        self.data[new_col] = self.data[old_col]

                self.status_var.set(
                    f"EKF result data loaded successfully: {len(self.data)} data points"
                )
                self.update_all_plots()
            else:
                self.status_var.set(f"File not found: {self.csv_file}")
                messagebox.showwarning("Warning", f"File not found: {self.csv_file}")
        except Exception as e:
            self.status_var.set(f"Data loading failed: {str(e)}")
            messagebox.showerror("Error", f"Data loading failed: {str(e)}")

    def load_raw_data(self):
        """Load raw sensor data from CSV files"""
        try:
            # Get current working directory
            current_dir = os.getcwd()
            print(f"Current working directory: {current_dir}")

            # Find all CSV files in the current directory
            csv_files = glob.glob("*.csv")
            print(f"Found CSV files: {csv_files}")

            raw_data_files = [
                f
                for f in csv_files
                if f not in ["simulation_results.csv", "ekf_result.csv"]
            ]
            print(f"Raw data files (excluding result files): {raw_data_files}")

            self.raw_data = {}
            loaded_files = []
            failed_files = []

            for file in raw_data_files:
                try:
                    print(f"Attempting to load: {file}")
                    df = pd.read_csv(file)
                    print(
                        f"Successfully loaded {file}: {len(df)} rows, columns: {list(df.columns)}"
                    )

                    # Extract sensor type from filename
                    sensor_name = file.replace(".csv", "")
                    self.raw_data[sensor_name] = df
                    loaded_files.append(file)

                except Exception as e:
                    print(f"Failed to load {file}: {str(e)}")
                    failed_files.append((file, str(e)))

            print(f"Successfully loaded {len(loaded_files)} files: {loaded_files}")
            if failed_files:
                print(f"Failed to load {len(failed_files)} files: {failed_files}")

            if self.raw_data:
                current_status = self.status_var.get()
                self.status_var.set(
                    f"{current_status} | Raw data: {len(self.raw_data)} sensors"
                )

            # Store information for debugging
            self.csv_files_found = csv_files
            self.raw_data_files_found = raw_data_files
            self.loaded_files = loaded_files
            self.failed_files = failed_files

        except Exception as e:
            print(f"Exception in load_raw_data: {str(e)}")
            self.csv_files_found = []
            self.raw_data_files_found = []
            self.loaded_files = []
            self.failed_files = []

    def refresh_all_data(self):
        """Refresh all data"""
        self.load_data()
        self.load_raw_data()

    def open_file(self):
        """Open file dialog"""
        filename = filedialog.askopenfilename(
            title="Select CSV File",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if filename:
            self.csv_file = filename
            self.load_data()

    def update_all_plots(self):
        """Update all plots"""
        if self.data is None:
            return

        self.plot_raw_data()
        self.plot_imu_data()  # NEW
        self.plot_position()
        self.plot_velocity()
        self.plot_orientation()
        self.plot_bias()
        self.plot_trajectory_3d()
        self.plot_error_analysis()
        self.plot_statistics()

    def plot_raw_data(self):
        """Plot raw sensor data"""
        frame = self.tabs["raw_data"]

        # Remove existing widgets
        for widget in frame.winfo_children():
            widget.destroy()

        if not self.raw_data:
            # Create a more informative message with debugging info
            info_frame = tk.Frame(frame)
            info_frame.pack(expand=True, fill=tk.BOTH, padx=20, pady=20)

            title_label = tk.Label(
                info_frame,
                text="Raw Sensor Data Status",
                font=("Arial", 14, "bold"),
                fg="blue",
            )
            title_label.pack(pady=(0, 10))

            # Current directory info
            current_dir = os.getcwd()
            dir_label = tk.Label(
                info_frame,
                text=f"Current Directory: {current_dir}",
                font=("Arial", 10),
                wraplength=600,
            )
            dir_label.pack(pady=(0, 10))

            # Found CSV files
            if hasattr(self, "csv_files_found") and self.csv_files_found:
                found_label = tk.Label(
                    info_frame,
                    text=f"Found CSV files: {', '.join(self.csv_files_found)}",
                    font=("Arial", 10),
                    wraplength=600,
                )
                found_label.pack(pady=(0, 10))
            else:
                no_files_label = tk.Label(
                    info_frame,
                    text="No CSV files found in current directory",
                    font=("Arial", 10),
                    fg="red",
                )
                no_files_label.pack(pady=(0, 10))

            # Raw data files (excluding result files)
            if hasattr(self, "raw_data_files_found") and self.raw_data_files_found:
                raw_files_label = tk.Label(
                    info_frame,
                    text=f"Raw data files found: {', '.join(self.raw_data_files_found)}",
                    font=("Arial", 10),
                    wraplength=600,
                )
                raw_files_label.pack(pady=(0, 10))

            # Successfully loaded files
            if hasattr(self, "loaded_files") and self.loaded_files:
                loaded_label = tk.Label(
                    info_frame,
                    text=f"Successfully loaded: {', '.join(self.loaded_files)}",
                    font=("Arial", 10),
                    fg="green",
                    wraplength=600,
                )
                loaded_label.pack(pady=(0, 10))

            # Failed files
            if hasattr(self, "failed_files") and self.failed_files:
                failed_label = tk.Label(
                    info_frame,
                    text="Failed to load files:",
                    font=("Arial", 10, "bold"),
                    fg="red",
                )
                failed_label.pack(pady=(0, 5))

                for file, error in self.failed_files:
                    error_label = tk.Label(
                        info_frame,
                        text=f"  • {file}: {error}",
                        font=("Arial", 9),
                        fg="red",
                        wraplength=600,
                    )
                    error_label.pack(pady=(0, 2))

            # Refresh button
            refresh_button = tk.Button(
                info_frame,
                text="Refresh Data",
                command=self.refresh_all_data,
                font=("Arial", 10),
                bg="lightblue",
            )
            refresh_button.pack(pady=(20, 10))

            return

        # Create notebook for raw data subtabs
        raw_notebook = ttk.Notebook(frame)
        raw_notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # IMU Accelerometer tab
        self.plot_imu_accel(raw_notebook)

        # IMU Gyroscope tab
        self.plot_imu_gyro(raw_notebook)

        # IMU Orientation tab
        self.plot_imu_orientation(raw_notebook)

        # Odometry tab
        self.plot_odom_data(raw_notebook)

    def plot_imu_accel(self, parent_notebook):
        """Plot IMU accelerometer data"""
        accel_frame = ttk.Frame(parent_notebook)
        parent_notebook.add(accel_frame, text="IMU Accel")

        # Check if accelerometer data exists
        accel_files = [k for k in self.raw_data.keys() if "linear_acc" in k]
        if not accel_files:
            label = tk.Label(
                accel_frame,
                text="No IMU accelerometer data found",
                font=("Arial", 12),
                justify=tk.CENTER,
            )
            label.pack(expand=True)
            return

        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle("IMU Linear Acceleration", fontsize=14, fontweight="bold")

        # Plot accelerometer data
        for i, axis in enumerate(["x", "y", "z"]):
            key = f"imu_linear_acc_{axis}"
            if key in self.raw_data:
                data = self.raw_data[key]
                axes[i].plot(data["elapsed_time"], data["value"], "b-", linewidth=1)
                axes[i].set_ylabel(f"Accel {axis.upper()} (m/s²)")
                axes[i].grid(True, alpha=0.3)
                axes[i].set_title(f"Linear Acceleration {axis.upper()}")

        axes[2].set_xlabel("Time (s)")
        plt.tight_layout()

        canvas = FigureCanvasTkAgg(fig, accel_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def plot_imu_gyro(self, parent_notebook):
        """Plot IMU gyroscope data"""
        gyro_frame = ttk.Frame(parent_notebook)
        parent_notebook.add(gyro_frame, text="IMU Gyro")

        # Check if gyroscope data exists
        gyro_files = [k for k in self.raw_data.keys() if "angular_vel" in k]
        if not gyro_files:
            label = tk.Label(
                gyro_frame,
                text="No IMU gyroscope data found",
                font=("Arial", 12),
                justify=tk.CENTER,
            )
            label.pack(expand=True)
            return

        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle("IMU Angular Velocity", fontsize=14, fontweight="bold")

        # Plot gyroscope data
        for i, axis in enumerate(["x", "y", "z"]):
            key = f"imu_angular_vel_{axis}"
            if key in self.raw_data:
                data = self.raw_data[key]
                axes[i].plot(data["elapsed_time"], data["value"], "r-", linewidth=1)
                axes[i].set_ylabel(f"Gyro {axis.upper()} (rad/s)")
                axes[i].grid(True, alpha=0.3)
                axes[i].set_title(f"Angular Velocity {axis.upper()}")

        axes[2].set_xlabel("Time (s)")
        plt.tight_layout()

        canvas = FigureCanvasTkAgg(fig, gyro_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def plot_imu_orientation(self, parent_notebook):
        """Plot IMU orientation data"""
        orient_frame = ttk.Frame(parent_notebook)
        parent_notebook.add(orient_frame, text="IMU Orientation")

        # Check if orientation data exists
        orient_files = [k for k in self.raw_data.keys() if "orientation" in k]
        if not orient_files:
            label = tk.Label(
                orient_frame,
                text="No IMU orientation data found",
                font=("Arial", 12),
                justify=tk.CENTER,
            )
            label.pack(expand=True)
            return

        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle("IMU Orientation (Quaternion)", fontsize=14, fontweight="bold")

        # Plot quaternion components
        quat_components = ["w", "x", "y", "z"]
        positions = [(0, 0), (0, 1), (1, 0), (1, 1)]

        for i, comp in enumerate(quat_components):
            key = f"imu_orientation_{comp}"
            if key in self.raw_data:
                data = self.raw_data[key]
                row, col = positions[i]
                axes[row, col].plot(
                    data["elapsed_time"], data["value"], "g-", linewidth=1
                )
                axes[row, col].set_ylabel(f"Quat {comp.upper()}")
                axes[row, col].grid(True, alpha=0.3)
                axes[row, col].set_title(f"Quaternion {comp.upper()}")
                if row == 1:  # Bottom row
                    axes[row, col].set_xlabel("Time (s)")

        plt.tight_layout()

        canvas = FigureCanvasTkAgg(fig, orient_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def plot_odom_data(self, parent_notebook):
        """Plot odometry data"""
        odom_frame = ttk.Frame(parent_notebook)
        parent_notebook.add(odom_frame, text="Odometry")

        # Check if odometry data exists
        odom_files = [
            k for k in self.raw_data.keys() if "odom" in k and "twist" not in k
        ]
        if not odom_files:
            label = tk.Label(
                odom_frame,
                text="No odometry data found",
                font=("Arial", 12),
                justify=tk.CENTER,
            )
            label.pack(expand=True)
            return

        # Try to find odometry files that match the expected patterns
        linear_vel_files = [k for k in self.raw_data.keys() if "odom_linear_vel" in k]
        angular_vel_files = [k for k in self.raw_data.keys() if "odom_angular_vel" in k]

        if not linear_vel_files and not angular_vel_files:
            label = tk.Label(
                odom_frame,
                text="No odometry velocity data found",
                font=("Arial", 12),
                justify=tk.CENTER,
            )
            label.pack(expand=True)
            return

        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle("Odometry Data", fontsize=14, fontweight="bold")

        # Plot linear velocity
        if "odom_linear_vel_x" in self.raw_data:
            data = self.raw_data["odom_linear_vel_x"]
            axes[0].plot(data["elapsed_time"], data["value"], "b-", linewidth=1)
            axes[0].set_ylabel("Linear X (m/s)")
            axes[0].grid(True, alpha=0.3)
            axes[0].set_title("Linear Velocity X")

        # Plot angular velocity
        if "odom_angular_vel_z" in self.raw_data:
            data = self.raw_data["odom_angular_vel_z"]
            axes[1].plot(data["elapsed_time"], data["value"], "r-", linewidth=1)
            axes[1].set_ylabel("Angular Z (rad/s)")
            axes[1].grid(True, alpha=0.3)
            axes[1].set_title("Angular Velocity Z")
            axes[1].set_xlabel("Time (s)")

        plt.tight_layout()

        canvas = FigureCanvasTkAgg(fig, odom_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def plot_imu_data(self):
        """Plot IMU sensor data from ekf_result.csv"""
        frame = self.tabs["imu_data"]

        # Remove existing widgets
        for widget in frame.winfo_children():
            widget.destroy()

        if self.data is None:
            label = tk.Label(frame, text="No data available", font=("Arial", 12))
            label.pack(expand=True)
            return

        # Check if IMU data columns exist
        imu_columns = [
            "imu_ax",
            "imu_ay",
            "imu_az",
            "imu_wx",
            "imu_wy",
            "imu_wz",
            "imu_roll",
            "imu_pitch",
            "imu_yaw",
        ]
        available_imu_cols = [col for col in imu_columns if col in self.data.columns]

        if not available_imu_cols:
            label = tk.Label(
                frame,
                text="No IMU data found in ekf_result.csv",
                font=("Arial", 12),
                justify=tk.CENTER,
            )
            label.pack(expand=True)
            return

        # Create notebook for IMU data subtabs
        imu_notebook = ttk.Notebook(frame)
        imu_notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        time = self.data["time"]

        # 1. IMU Accelerometer tab
        if any(col in self.data.columns for col in ["imu_ax", "imu_ay", "imu_az"]):
            accel_frame = ttk.Frame(imu_notebook)
            imu_notebook.add(accel_frame, text="Accelerometer")

            fig, axes = plt.subplots(3, 1, figsize=(12, 10))
            fig.suptitle("IMU Linear Acceleration", fontsize=14, fontweight="bold")

            for i, axis in enumerate(["x", "y", "z"]):
                col_name = f"imu_a{axis}"
                if col_name in self.data.columns:
                    axes[i].plot(time, self.data[col_name], "b-", linewidth=1.5)
                    axes[i].set_ylabel(f"Accel {axis.upper()} (m/s²)")
                    axes[i].grid(True, alpha=0.3)
                    axes[i].set_title(f"Linear Acceleration {axis.upper()}")

            axes[2].set_xlabel("Time (s)")
            plt.tight_layout()

            canvas = FigureCanvasTkAgg(fig, accel_frame)
            canvas.draw()
            canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # 2. IMU Gyroscope tab
        if any(col in self.data.columns for col in ["imu_wx", "imu_wy", "imu_wz"]):
            gyro_frame = ttk.Frame(imu_notebook)
            imu_notebook.add(gyro_frame, text="Gyroscope")

            fig, axes = plt.subplots(3, 1, figsize=(12, 10))
            fig.suptitle("IMU Angular Velocity", fontsize=14, fontweight="bold")

            for i, axis in enumerate(["x", "y", "z"]):
                col_name = f"imu_w{axis}"
                if col_name in self.data.columns:
                    axes[i].plot(time, self.data[col_name], "r-", linewidth=1.5)
                    axes[i].set_ylabel(f"Gyro {axis.upper()} (rad/s)")
                    axes[i].grid(True, alpha=0.3)
                    axes[i].set_title(f"Angular Velocity {axis.upper()}")

            axes[2].set_xlabel("Time (s)")
            plt.tight_layout()

            canvas = FigureCanvasTkAgg(fig, gyro_frame)
            canvas.draw()
            canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # 3. IMU Orientation tab
        if any(
            col in self.data.columns for col in ["imu_roll", "imu_pitch", "imu_yaw"]
        ):
            orient_frame = ttk.Frame(imu_notebook)
            imu_notebook.add(orient_frame, text="Orientation")

            fig, axes = plt.subplots(3, 1, figsize=(12, 10))
            fig.suptitle("IMU Orientation", fontsize=14, fontweight="bold")

            orient_axes = ["roll", "pitch", "yaw"]
            for i, axis in enumerate(orient_axes):
                col_name = f"imu_{axis}"
                if col_name in self.data.columns:
                    axes[i].plot(
                        time, np.degrees(self.data[col_name]), "g-", linewidth=1.5
                    )
                    axes[i].set_ylabel(f"{axis.capitalize()} (degrees)")
                    axes[i].grid(True, alpha=0.3)
                    axes[i].set_title(f"IMU {axis.capitalize()}")

            axes[2].set_xlabel("Time (s)")
            plt.tight_layout()

            canvas = FigureCanvasTkAgg(fig, orient_frame)
            canvas.draw()
            canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # 4. Combined IMU Overview tab
        overview_frame = ttk.Frame(imu_notebook)
        imu_notebook.add(overview_frame, text="Overview")

        fig, axes = plt.subplots(3, 1, figsize=(12, 12))
        fig.suptitle("IMU Data Overview", fontsize=14, fontweight="bold")

        # Plot accelerometer data
        if any(col in self.data.columns for col in ["imu_ax", "imu_ay", "imu_az"]):
            for axis, color in zip(["x", "y", "z"], ["r", "g", "b"]):
                col_name = f"imu_a{axis}"
                if col_name in self.data.columns:
                    axes[0].plot(
                        time,
                        self.data[col_name],
                        color=color,
                        label=f"Accel {axis.upper()}",
                        linewidth=1.5,
                    )
            axes[0].set_ylabel("Acceleration (m/s²)")
            axes[0].set_title("Linear Acceleration")
            axes[0].legend()
            axes[0].grid(True, alpha=0.3)

        # Plot gyroscope data
        if any(col in self.data.columns for col in ["imu_wx", "imu_wy", "imu_wz"]):
            for axis, color in zip(["x", "y", "z"], ["r", "g", "b"]):
                col_name = f"imu_w{axis}"
                if col_name in self.data.columns:
                    axes[1].plot(
                        time,
                        self.data[col_name],
                        color=color,
                        label=f"Gyro {axis.upper()}",
                        linewidth=1.5,
                    )
            axes[1].set_ylabel("Angular Velocity (rad/s)")
            axes[1].set_title("Angular Velocity")
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)

        # Plot orientation data
        if any(
            col in self.data.columns for col in ["imu_roll", "imu_pitch", "imu_yaw"]
        ):
            for axis, color in zip(["roll", "pitch", "yaw"], ["r", "g", "b"]):
                col_name = f"imu_{axis}"
                if col_name in self.data.columns:
                    axes[2].plot(
                        time,
                        np.degrees(self.data[col_name]),
                        color=color,
                        label=f"{axis.capitalize()}",
                        linewidth=1.5,
                    )
            axes[2].set_ylabel("Orientation (degrees)")
            axes[2].set_xlabel("Time (s)")
            axes[2].set_title("Orientation")
            axes[2].legend()
            axes[2].grid(True, alpha=0.3)

        plt.tight_layout()

        canvas = FigureCanvasTkAgg(fig, overview_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def plot_position(self):
        """Position plot"""
        frame = self.tabs["position"]

        # Remove existing widgets
        for widget in frame.winfo_children():
            widget.destroy()

        if self.data is None or not all(
            k in self.data.columns
            for k in ["true_x", "est_x", "true_y", "est_y", "true_z", "est_z"]
        ):
            label = tk.Label(
                frame, text="Position data not available in CSV.", font=("Arial", 12)
            )
            label.pack(expand=True)
            return

        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        fig.suptitle(
            "Position (Blue: Odom True, Red: EKF Estimated)",
            fontsize=14,
            fontweight="bold",
        )

        time = self.data["time"]

        # X Position
        axes[0].plot(time, self.data["true_x"], "b-", label="Odom X", linewidth=1.5)
        axes[0].plot(time, self.data["est_x"], "r--", label="EKF X", linewidth=1.5)
        axes[0].set_ylabel("X Position (m)")
        axes[0].set_title("X Position")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        # Y Position
        axes[1].plot(time, self.data["true_y"], "b-", label="Odom Y", linewidth=1.5)
        axes[1].plot(time, self.data["est_y"], "r--", label="EKF Y", linewidth=1.5)
        axes[1].set_ylabel("Y Position (m)")
        axes[1].set_title("Y Position")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        # Z Position
        axes[2].plot(time, self.data["true_z"], "b-", label="Odom Z", linewidth=1.5)
        axes[2].plot(time, self.data["est_z"], "r--", label="EKF Z", linewidth=1.5)
        axes[2].set_ylabel("Z Position (m)")
        axes[2].set_xlabel("Time (s)")
        axes[2].set_title("Z Position")
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

        canvas = FigureCanvasTkAgg(fig, frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def plot_velocity(self):
        """Velocity plot"""
        frame = self.tabs["velocity"]

        # Remove existing widgets
        for widget in frame.winfo_children():
            widget.destroy()

        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle(
            "Velocity (Blue: Odom True, Red: EKF Estimated)",
            fontsize=14,
            fontweight="bold",
        )

        time = self.data["time"]

        # X Velocity
        axes[0].plot(time, self.data["true_vx"], "b-", label="Odom Vx", linewidth=2)
        axes[0].plot(time, self.data["est_vx"], "r--", label="EKF Vx", linewidth=2)
        axes[0].set_ylabel("X Velocity (m/s)")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        # Y Velocity
        axes[1].plot(time, self.data["true_vy"], "b-", label="Odom Vy", linewidth=2)
        axes[1].plot(time, self.data["est_vy"], "r--", label="EKF Vy", linewidth=2)
        axes[1].set_ylabel("Y Velocity (m/s)")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        # Z Velocity
        axes[2].plot(time, self.data["true_vz"], "b-", label="Odom Vz", linewidth=2)
        axes[2].plot(time, self.data["est_vz"], "r--", label="EKF Vz", linewidth=2)
        axes[2].set_ylabel("Z Velocity (m/s)")
        axes[2].set_xlabel("Time (s)")
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout()

        canvas = FigureCanvasTkAgg(fig, frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def plot_orientation(self):
        """Orientation plot"""
        frame = self.tabs["orientation"]

        # Remove existing widgets
        for widget in frame.winfo_children():
            widget.destroy()

        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle(
            "Orientation (Blue: Odom True, Red: EKF Estimated)",
            fontsize=14,
            fontweight="bold",
        )

        time = self.data["time"]

        # Roll
        axes[0].plot(
            time,
            np.degrees(self.data["true_roll"]),
            "b-",
            label="Odom Roll",
            linewidth=2,
        )
        axes[0].plot(
            time,
            np.degrees(self.data["est_roll"]),
            "r--",
            label="EKF Roll",
            linewidth=2,
        )
        axes[0].set_ylabel("Roll (degrees)")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        # Pitch
        axes[1].plot(
            time,
            np.degrees(self.data["true_pitch"]),
            "b-",
            label="Odom Pitch",
            linewidth=2,
        )
        axes[1].plot(
            time,
            np.degrees(self.data["est_pitch"]),
            "r--",
            label="EKF Pitch",
            linewidth=2,
        )
        axes[1].set_ylabel("Pitch (degrees)")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        # Yaw
        axes[2].plot(
            time, np.degrees(self.data["true_yaw"]), "b-", label="Odom Yaw", linewidth=2
        )
        axes[2].plot(
            time,
            np.degrees(self.data["est_yaw"]),
            "r--",
            label="EKF Yaw",
            linewidth=2,
        )
        axes[2].set_ylabel("Yaw (degrees)")
        axes[2].set_xlabel("Time (s)")
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout()

        canvas = FigureCanvasTkAgg(fig, frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def plot_bias(self):
        """Acceleration and Angular Velocity plot with real IMU data"""
        frame = self.tabs["bias"]

        # Remove existing widgets
        for widget in frame.winfo_children():
            widget.destroy()

        fig, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
        fig.suptitle(
            "Acceleration & Angular Velocity (Blue: IMU/Odom, Red: EKF Estimated)",
            fontsize=14,
            fontweight="bold",
        )

        time = self.data["time"]

        # --- Linear Acceleration (now using real IMU data) ---
        accel_axes = ["x", "y", "z"]
        for i, axis in enumerate(accel_axes):
            ax = axes[0, i]
            true_col = f"true_a{axis}"
            est_col = f"est_a{axis}"

            # Use IMU data if available, otherwise use the existing data
            imu_col = f"imu_a{axis}"
            if imu_col in self.data.columns:
                ax.plot(
                    time,
                    self.data[imu_col],
                    "b-",
                    label=f"IMU Accel {axis.upper()}",
                    linewidth=1.5,
                )
                # If there's a difference between true and est, show EKF estimate
                if true_col in self.data.columns and est_col in self.data.columns:
                    ax.plot(
                        time,
                        self.data[est_col],
                        "r--",
                        label=f"EKF Accel {axis.upper()}",
                        linewidth=1.5,
                    )
            else:
                ax.plot(
                    time,
                    self.data[true_col],
                    "b-",
                    label=f"True Accel {axis.upper()}",
                    linewidth=1.5,
                )
                ax.plot(
                    time,
                    self.data[est_col],
                    "r--",
                    label=f"EKF Accel {axis.upper()}",
                    linewidth=1.5,
                )

            ax.set_title(f"Linear Acceleration {axis.upper()}")
            ax.set_ylabel("(m/s²)")
            ax.legend()
            ax.grid(True, alpha=0.3)

        # --- Angular Velocity ---
        ang_vel_axes = ["roll", "pitch", "yaw"]
        for i, axis in enumerate(ang_vel_axes):
            ax = axes[1, i]
            true_col = f"true_v{axis}"
            est_col = f"est_v{axis}"

            # Also show IMU angular velocity if available
            imu_col = f"imu_w{['x', 'y', 'z'][i]}"  # Map roll->x, pitch->y, yaw->z
            if imu_col in self.data.columns:
                ax.plot(
                    time,
                    self.data[imu_col],
                    "g:",
                    label=f"IMU Ang Vel {axis.capitalize()}",
                    linewidth=1.5,
                )

            ax.plot(
                time,
                self.data[true_col],
                "b-",
                label=f"Odom Ang Vel {axis.capitalize()}",
                linewidth=1.5,
            )
            ax.plot(
                time,
                self.data[est_col],
                "r--",
                label=f"EKF Ang Vel {axis.capitalize()}",
                linewidth=1.5,
            )
            ax.set_title(f"Angular Velocity {axis.capitalize()}")
            ax.set_ylabel("(rad/s)")
            ax.set_xlabel("Time (s)")
            ax.legend()
            ax.grid(True, alpha=0.3)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

        canvas = FigureCanvasTkAgg(fig, frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def plot_trajectory_3d(self):
        """2D trajectory plot"""
        frame = self.tabs["trajectory"]

        # Remove existing widgets
        for widget in frame.winfo_children():
            widget.destroy()

        fig, ax = plt.subplots(figsize=(12, 10))

        # --- 2D trajectory plot (X-Y plane) with progression indication ---

        # 1) Display Odom (reference) path as a faint gray line
        ax.plot(
            self.data["true_x"],
            self.data["true_y"],
            color="lightgray",
            linewidth=1.0,
            label="Odom Trajectory",
            alpha=0.6,
        )

        # 2) Draw EKF trajectory based on time (color) to represent progression
        sc = ax.scatter(
            self.data["est_x"],
            self.data["est_y"],
            c=self.data["time"],
            cmap="viridis",
            s=10,
            alpha=0.9,
            label="EKF Trajectory",
        )

        # Add a color bar
        cbar = plt.colorbar(sc, ax=ax)
        cbar.set_label("Time (s)")

        # 3) Add arrows at regular intervals to more clearly indicate the direction of travel
        arrow_step = max(1, len(self.data) // 30)  # About 30 arrows
        x = self.data["est_x"].values
        y = self.data["est_y"].values
        dx = np.diff(x)
        dy = np.diff(y)
        idx = np.arange(0, len(dx), arrow_step)
        ax.quiver(
            x[idx],
            y[idx],
            dx[idx],
            dy[idx],
            color="black",
            scale_units="xy",
            angles="xy",
            scale=1,
            width=0.003,
            alpha=0.8,
        )

        # Mark start and end points
        ax.scatter(
            self.data["true_x"].iloc[0],
            self.data["true_y"].iloc[0],
            color="green",
            s=100,
            label="Start",
            marker="o",
        )
        ax.scatter(
            self.data["true_x"].iloc[-1],
            self.data["true_y"].iloc[-1],
            color="red",
            s=100,
            label="End",
            marker="s",
        )

        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.set_title("2D Trajectory (X-Y Plane)", fontsize=14, fontweight="bold")
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis("equal")  # Equal aspect ratio for better visualization

        plt.tight_layout()

        canvas = FigureCanvasTkAgg(fig, frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def plot_error_analysis(self):
        """Error analysis plot"""
        frame = self.tabs["error"]

        # Remove existing widgets
        for widget in frame.winfo_children():
            widget.destroy()

        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle("Error Analysis (EKF vs Odom)", fontsize=14, fontweight="bold")

        time = self.data["time"]

        # Position error
        pos_error_x = self.data["est_x"] - self.data["true_x"]
        pos_error_y = self.data["est_y"] - self.data["true_y"]
        pos_error_z = self.data["est_z"] - self.data["true_z"]
        pos_error_total = np.sqrt(pos_error_x**2 + pos_error_y**2 + pos_error_z**2)

        axes[0, 0].plot(time, pos_error_x, "r-", label="X Error", linewidth=2)
        axes[0, 0].plot(time, pos_error_y, "g-", label="Y Error", linewidth=2)
        axes[0, 0].plot(time, pos_error_z, "b-", label="Z Error", linewidth=2)
        axes[0, 0].plot(time, pos_error_total, "k--", label="Total Error", linewidth=2)
        axes[0, 0].set_ylabel("Position Error (m)")
        axes[0, 0].set_title("Position Error")
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)

        # Velocity error
        vel_error_x = self.data["est_vx"] - self.data["true_vx"]
        vel_error_y = self.data["est_vy"] - self.data["true_vy"]
        vel_error_z = self.data["est_vz"] - self.data["true_vz"]
        vel_error_total = np.sqrt(vel_error_x**2 + vel_error_y**2 + vel_error_z**2)

        axes[0, 1].plot(time, vel_error_x, "r-", label="Vx Error", linewidth=2)
        axes[0, 1].plot(time, vel_error_y, "g-", label="Vy Error", linewidth=2)
        axes[0, 1].plot(time, vel_error_z, "b-", label="Vz Error", linewidth=2)
        axes[0, 1].plot(time, vel_error_total, "k--", label="Total Error", linewidth=2)
        axes[0, 1].set_ylabel("Velocity Error (m/s)")
        axes[0, 1].set_title("Velocity Error")
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)

        # Orientation error
        ori_error_roll = np.degrees(self.data["est_roll"] - self.data["true_roll"])
        ori_error_pitch = np.degrees(self.data["est_pitch"] - self.data["true_pitch"])
        ori_error_yaw = np.degrees(self.data["est_yaw"] - self.data["true_yaw"])

        axes[1, 0].plot(time, ori_error_roll, "r-", label="Roll Error", linewidth=2)
        axes[1, 0].plot(time, ori_error_pitch, "g-", label="Pitch Error", linewidth=2)
        axes[1, 0].plot(time, ori_error_yaw, "b-", label="Yaw Error", linewidth=2)
        axes[1, 0].set_ylabel("Orientation Error (degrees)")
        axes[1, 0].set_xlabel("Time (s)")
        axes[1, 0].set_title("Orientation Error")
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)

        # Error histogram
        axes[1, 1].hist(
            pos_error_total, bins=30, alpha=0.7, color="blue", edgecolor="black"
        )
        axes[1, 1].set_xlabel("Position Error (m)")
        axes[1, 1].set_ylabel("Frequency")
        axes[1, 1].set_title("Position Error Distribution")
        axes[1, 1].grid(True, alpha=0.3)

        plt.tight_layout()

        canvas = FigureCanvasTkAgg(fig, frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def plot_statistics(self):
        """Display statistics information"""
        frame = self.tabs["statistics"]

        # Remove existing widgets
        for widget in frame.winfo_children():
            widget.destroy()

        # Calculate statistics
        pos_error_x = self.data["est_x"] - self.data["true_x"]
        pos_error_y = self.data["est_y"] - self.data["true_y"]
        pos_error_z = self.data["est_z"] - self.data["true_z"]
        pos_error_total = np.sqrt(pos_error_x**2 + pos_error_y**2 + pos_error_z**2)

        vel_error_x = self.data["est_vx"] - self.data["true_vx"]
        vel_error_y = self.data["est_vy"] - self.data["true_vy"]
        vel_error_z = self.data["est_vz"] - self.data["true_vz"]
        vel_error_total = np.sqrt(vel_error_x**2 + vel_error_y**2 + vel_error_z**2)

        ori_error_roll = np.degrees(self.data["est_roll"] - self.data["true_roll"])
        ori_error_pitch = np.degrees(self.data["est_pitch"] - self.data["true_pitch"])
        ori_error_yaw = np.degrees(self.data["est_yaw"] - self.data["true_yaw"])

        # Raw data statistics
        raw_data_info = ""
        if self.raw_data:
            raw_data_info = f"""
Raw Data Information:
- Available sensors: {len(self.raw_data)}
- Sensor types: {', '.join(self.raw_data.keys())}
"""

        # Generate statistics text
        stats_text = f"""
EKF Performance Statistics (EKF vs Odom)
{'='*50}

Simulation Information:
- Total Time: {self.data['time'].iloc[-1]:.2f} seconds
- Data Points: {len(self.data)} points
- Time Interval: {np.mean(np.diff(self.data['time'])):.3f} seconds

Position Error (EKF vs Odom):
- Mean Error: {np.mean(pos_error_total):.4f} m
- Standard Deviation: {np.std(pos_error_total):.4f} m
- Maximum Error: {np.max(pos_error_total):.4f} m
- RMS Error: {np.sqrt(np.mean(pos_error_total**2)):.4f} m

Velocity Error (EKF vs Odom):
- Mean Error: {np.mean(vel_error_total):.4f} m/s
- Standard Deviation: {np.std(vel_error_total):.4f} m/s
- Maximum Error: {np.max(vel_error_total):.4f} m/s
- RMS Error: {np.sqrt(np.mean(vel_error_total**2)):.4f} m/s

Orientation Error (EKF vs Odom):
- Roll Mean Error: {np.mean(np.abs(ori_error_roll)):.4f} degrees
- Pitch Mean Error: {np.mean(np.abs(ori_error_pitch)):.4f} degrees
- Yaw Mean Error: {np.mean(np.abs(ori_error_yaw)):.4f} degrees

EKF Final State:
- Final Angular Velocity X: {self.data['est_vroll'].iloc[-1]:.6f} rad/s
- Final Angular Velocity Y: {self.data['est_vpitch'].iloc[-1]:.6f} rad/s
- Final Angular Velocity Z: {self.data['est_vyaw'].iloc[-1]:.6f} rad/s

Trajectory Information (Odom Reference):
- Start Position: ({self.data['true_x'].iloc[0]:.2f}, {self.data['true_y'].iloc[0]:.2f}, {self.data['true_z'].iloc[0]:.2f})
- End Position: ({self.data['true_x'].iloc[-1]:.2f}, {self.data['true_y'].iloc[-1]:.2f}, {self.data['true_z'].iloc[-1]:.2f})
- Total Distance: {np.sum(np.sqrt(np.diff(self.data['true_x'])**2 + np.diff(self.data['true_y'])**2 + np.diff(self.data['true_z'])**2)):.2f} m

{raw_data_info}
        """

        # Create text widget
        text_widget = tk.Text(frame, wrap=tk.WORD, font=("Courier", 10))
        text_widget.insert(tk.END, stats_text)
        text_widget.config(state=tk.DISABLED)

        # Add scrollbar
        scrollbar = tk.Scrollbar(frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        text_widget.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        text_widget.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=text_widget.yview)


def main():
    """Main function"""
    root = tk.Tk()
    app = EKFResultVisualizer(root)
    root.mainloop()


if __name__ == "__main__":
    main()

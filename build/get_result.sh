#!/bin/bash
clear
# Get the current directory path
current_dir=$(pwd)

echo "Starting code compilation"
make

echo "Code compilation complete"

rm $current_dir/ekf_result.csv
echo "ekf_result.csv file deleted"

echo "Running EKF simulator"
./ekf_localizer
echo "ekf_result.csv file created"

echo "Starting visualization"
python3 $current_dir/result_visualization.py

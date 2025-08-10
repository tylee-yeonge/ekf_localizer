# Minimum CMake version requirement
cmake_minimum_required(VERSION 3.14)

# Set project name
project(ekf_localizer)

# Set C++ standard (C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3 library
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Create executable
# Add all .cpp files here.
add_executable(
    ekf_localizer
    src/main.cpp
    src/ekf.cpp    
    src/data_loader.cpp
    src/utils.cpp
)

# Link Eigen3 library to the executable
target_include_directories(ekf_localizer PUBLIC 
    include
)

# Link Eigen3 target
target_link_libraries(ekf_localizer Eigen3::Eigen)

# Output message
message(STATUS "Eigen3 include directory: ${EIGEN3_INCLUDE_DIR}")
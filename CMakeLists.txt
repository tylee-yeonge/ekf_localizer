# CMake 최소 버전 요구사항
cmake_minimum_required(VERSION 3.14)

# 프로젝트 이름 설정
project(ekf_localizer)

# C++ 표준 설정 (C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Eigen3 라이브러리 찾기
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 실행 파일 생성
# 모든 .cpp 파일을 여기에 추가합니다.
add_executable(
    ekf_localizer
    src/main.cpp
    src/ekf.cpp    
    src/data_loader.cpp
    src/utils.cpp
)

# 실행 파일에 Eigen3 라이브러리 연결
target_include_directories(ekf_localizer PUBLIC 
    include
)

# Eigen3 타겟 연결
target_link_libraries(ekf_localizer Eigen3::Eigen)

# 출력 메시지
message(STATUS "Eigen3 include directory: ${EIGEN3_INCLUDE_DIR}")
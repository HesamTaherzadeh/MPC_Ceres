cmake_minimum_required(VERSION 3.10)
project(MPC_Solver)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

# Include directories
include_directories(${EIGEN3_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} include)

# Add executable
add_executable(MPC_Solver
    main.cpp
    src/MPC.cpp
)

# Link libraries
target_link_libraries(MPC_Solver
    ${CERES_LIBRARIES}
)
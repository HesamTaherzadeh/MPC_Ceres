# MPC_Solver

This project implements a solver for the resection problem using the Ceres Solver library. The goal is to estimate the camera pose parameters (position and orientation) given ground control points (GCPs) and image measurements.

## Overview

The code is organized into three files:

- **MPC.hpp**: Contains the class definition for the MPC solver.
- **MPC.cpp**: Implements the methods defined in MPC.hpp.
- **main.cpp**: The main function that initializes the solver and runs it.

The solver reads data from a CSV file, performs optimization for left and right images separately, and computes the Root Mean Square Error (RMSE) for the optimized parameters.

## Dependencies

- **Ceres Solver**: A nonlinear least squares optimizer.
- **Eigen**: A C++ template library for linear algebra.
- **GLog**: Google's logging library.

Make sure these libraries are installed on your system before building the project.

### Installation on Ubuntu

```bash
sudo apt-get update
sudo apt-get install libeigen3-dev libceres-dev libgoogle-glog-dev
```

## Build Instructions

1. **Create a build directory:**

   ```bash
   mkdir build
   cd build
   ```

2. **Run CMake:**

   ```bash
   cmake ..
   ```

3. **Compile the project:**

   ```bash
   make
   ```

4. **Run the executable:**

   ```bash
   ./MPC_Solver [csv_file_path]
   ```

   Replace `[csv_file_path]` with the path to your CSV data file.

## Equations

### Resection Model

The resection problem involves estimating the camera pose (position and orientation) given known 3D points and their corresponding 2D image measurements. The mathematical model is as follows:

1. **Camera Position and Orientation:**

   - Position:
     ```
     Xs(t) = X0 + X1 * t + X2 * t^2
     ```

   - Orientation angles:
     ```
     ω(t) = ω0 + ω1 * t + ω2 * t^2
     φ(t) = φ0 + φ1 * t + φ2 * t^2
     κ(t) = κ0 + κ1 * t + κ2 * t^2
     ```

2. **Rotation Matrices:**

   - Rotation about the Z-axis:
     ```
     RW =
     [ 1       0         0       ]
     [ 0  cos(ω)   -sin(ω) ]
     [ 0  sin(ω)    cos(ω) ]
     ```

   - Rotation about the X-axis:
     ```
     RP =
     [ cos(φ)    0  sin(φ) ]
     [   0       1    0    ]
     [ -sin(φ)   0  cos(φ) ]
     ```

   - Rotation about the Y-axis:
     ```
     RK =
     [ cos(κ)  -sin(κ)   0 ]
     [ sin(κ)   cos(κ)   0 ]
     [   0        0      1 ]
     ```

   - Total rotation matrix:
     ```
     R = RK * RP * RW
     ```

3. **Projection Equations:**

   - Ground point in camera coordinates:
     ```
     ΔX = Xg - Xs(t)
     ΔY = Yg - Ys(t)
     ΔZ = Zg - Zs(t)
     ```

   - Rotated coordinates:
     ```
     X' = R11 * ΔX + R12 * ΔY + R13 * ΔZ
     Y' = R21 * ΔX + R22 * ΔY + R23 * ΔZ
     Z' = R31 * ΔX + R32 * ΔY + R33 * ΔZ
     ```

   - Projected image coordinates:
     ```
     x = -f * X' / Z'
     y = -f * Y' / Z'
     ```

   Where `f` is the focal length.

### RMSE Calculation

The RMSE is calculated as:

$$
RMSE = \sqrt{\frac{1}{N} \sum_{i=1}^{N} \left( (x_{obs,i} - x_{proj,i})^2 + (y_{obs,i} - y_{proj,i})^2 \right)}
$$

Where:
- `N` is the number of observations.
- `x_obs, y_obs` are the observed image coordinates.
- `x_proj, y_proj` are the projected coordinates based on the optimized parameters.

## Troubleshooting

- **Solver Failure:** Ensure that the initial parameters are reasonable and that the data is correctly loaded.
- **NaN or Inf Values:** Check for division by zero or invalid mathematical operations in the code.
- **Data Integrity:** Verify that the CSV file is correctly formatted and contains the expected data.

## References

- **Ceres Solver Documentation:** [Ceres Solver](http://ceres-solver.org/)
- **Eigen Library:** [Eigen](https://eigen.tuxfamily.org/)
- **GLog Library:** [GLog](https://github.com/google/glog)

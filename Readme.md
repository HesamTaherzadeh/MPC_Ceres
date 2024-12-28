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
     $$
     \mathbf{X_s}(t) = X_0 + X_1 t + X_2 t^2
     $$
   - Orientation angles:
     $$
     \omega(t) = \omega_0 + \omega_1 t + \omega_2 t^2, \quad
     \phi(t) = \phi_0 + \phi_1 t + \phi_2 t^2, \quad
     \kappa(t) = \kappa_0 + \kappa_1 t + \kappa_2 t^2
     $$

2. **Rotation Matrices:**

   - Rotation about the Z-axis:
     $$
     R_W =
     \begin{bmatrix}
     1 & 0 & 0 \\
     0 & \cos \omega & -\sin \omega \\
     0 & \sin \omega & \cos \omega
     \end{bmatrix}
     $$

   - Rotation about the X-axis :
     $$
     R_P =
     \begin{bmatrix}
     \cos \phi & 0 & \sin \phi \\
     0 & 1 & 0 \\
     -\sin \phi & 0 & \cos \phi
     \end{bmatrix}
     $$

   - Rotation about the Y-axis:
     $$
     R_K =
     \begin{bmatrix}
     \cos \kappa & -\sin \kappa & 0 \\
     \sin \kappa & \cos \kappa & 0 \\
     0 & 0 & 1
     \end{bmatrix}
     $$

   - Total rotation matrix:
     $$
     R = R_K R_P R_W
     $$

3. **Projection Equations:**

   - Ground point in camera coordinates:
     $$
     \delta X = X_g - X_s(t), \quad \delta Y = Y_g - Y_s(t), \quad \delta Z = Z_g - Z_s(t)
     $$

   - Rotated coordinates:
     $$
     X' = R_{11} \delta X + R_{12} \delta Y + R_{13} \delta Z
     $$
     $$
     Y' = R_{21} \delta X + R_{22} \delta Y + R_{23} \delta Z
     $$
     $$
     Z' = R_{31} \delta X + R_{32} \delta Y + R_{33} \delta Z
     $$

   - Projected image coordinates:
     $$
     x = -f \frac{X'}{Z'}, \quad y = -f \frac{Y'}{Z'}
     $$

   Where \( f \) is the focal length.

### RMSE Calculation

The RMSE is calculated as:

$$
RMSE = \sqrt{\frac{1}{N} \sum_{i=1}^{N} \left( (x_{obs,i} - x_{proj,i})^2 + (y_{obs,i} - y_{proj,i})^2 \right)}
$$

Where:
- \( N \) is the number of observations.
- \( x_{obs,i}, y_{obs,i} \) are the observed image coordinates.
- \( x_{proj,i}, y_{proj,i} \) are the projected coordinates based on the optimized parameters.

## Troubleshooting

- **Solver Failure:** Ensure that the initial parameters are reasonable and that the data is correctly loaded.
- **NaN or Inf Values:** Check for division by zero or invalid mathematical operations in the code.
- **Data Integrity:** Verify that the CSV file is correctly formatted and contains the expected data.

## References

- **Ceres Solver Documentation:** [Ceres Solver](http://ceres-solver.org/)
- **Eigen Library:** [Eigen](https://eigen.tuxfamily.org/)
- **GLog Library:** [GLog](https://github.com/google/glog)

---

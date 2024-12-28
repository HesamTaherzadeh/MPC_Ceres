#ifndef MPC_HPP
#define MPC_HPP

#include <vector>
#include <string>
#include <Eigen/Dense>
#include "ceres/ceres.h"
#include "cost.hpp"

class MPC {
public:
    MPC(const std::string& csv_path, double principal_distance, double pixel_size, int num_rows);

    void solveForLeftImage();
    void solveForRightImage();
    void computeRMSEForLeftImage();
    void computeRMSEForRightImage();

private:

    void loadCSVData();
    void initializeParameters();
    void solve(const std::vector<double>& xMeas, const std::vector<double>& yMeas, double* params, const std::string& topic);
    double computeRMSE(const std::vector<size_t>& indices, const std::vector<double>& xMeas,
                       const std::vector<double>& yMeas, const double* params);

    std::string csv_path_;
    double principal_distance_;
    double pixel_size_;
    int num_rows_;

    std::vector<double> Gx_, Gy_, Gz_;
    std::vector<double> Lx_, Ly_, Rx_, Ry_;
    std::vector<double> tVals_;
    std::vector<size_t> gcps_, icps_;

    double leftParams_[18];
    double rightParams_[18];
};

#endif // MPC_HPP
#include "MPC.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include "glog/logging.h"

MPC::MPC(const std::string& csv_path, double principal_distance, double pixel_size, int num_rows)
    : csv_path_(csv_path), principal_distance_(principal_distance), pixel_size_(pixel_size), num_rows_(num_rows) {
    loadCSVData();
    initializeParameters();
}

void MPC::loadCSVData() {
    std::ifstream inFile(csv_path_);
    if (!inFile.is_open()) {
        std::cerr << "Cannot open " << csv_path_ << std::endl;
        exit(1);
    }

    std::string header_line;
    if (!std::getline(inFile, header_line)) {
        std::cerr << "CSV has no data or no header.\n";
        exit(1);
    }

    std::string line;
    int lineCount = 0;
    while (std::getline(inFile, line)) {
        if (line.empty()) break;
        std::stringstream ss(line);

        double gx, gy, gz;
        double lc, lr, rc, rr;
        int tag;
        char comma;
        if (!(ss >> gx >> comma >> gy >> comma >> gz >> comma >> lc >> comma >> lr >> comma >> rc >> comma >> rr >> comma >> tag)) {
            std::cerr << "Error parsing line " << (lineCount + 2) << " in CSV.\n";
            exit(1);
        }

        Gx_.push_back(gx);
        Gy_.push_back(gy);
        Gz_.push_back(gz);

        Lx_.push_back((lr - num_rows_ / 2.0) * pixel_size_);
        Ly_.push_back((lc - num_rows_ / 2.0) * pixel_size_);
        Rx_.push_back((rr - num_rows_ / 2.0) * pixel_size_);
        Ry_.push_back((rc - num_rows_ / 2.0) * pixel_size_);

        if (tag == 1) gcps_.push_back(lineCount);
        else icps_.push_back(lineCount);

        lineCount++;
    }
    inFile.close();

    tVals_.reserve(Gx_.size());
    for (size_t i = 0; i < Gx_.size(); i++) {
        tVals_.push_back(static_cast<double>(i));
    }
}

void MPC::initializeParameters() {
    Eigen::MatrixXd Amat(2 * gcps_.size(), 4);
    Eigen::VectorXd bvec(2 * gcps_.size());

    for (size_t i = 0; i < gcps_.size(); ++i) {
        size_t idx = gcps_[i];
        Amat(2 * i, 0) = Lx_[idx]; Amat(2 * i, 1) = -Ly_[idx]; Amat(2 * i, 2) = 1.0; Amat(2 * i, 3) = 0.0;
        bvec(2 * i) = Gx_[idx];
        Amat(2 * i + 1, 0) = Ly_[idx]; Amat(2 * i + 1, 1) = Lx_[idx]; Amat(2 * i + 1, 2) = 0.0; Amat(2 * i + 1, 3) = 1.0;
        bvec(2 * i + 1) = Gy_[idx];
    }

    Eigen::Vector4d approxVals = (Amat.transpose() * Amat).ldlt().solve(Amat.transpose() * bvec);
    double a = approxVals(0), b = approxVals(1), cc = approxVals(2), d = approxVals(3);
    double abMag = std::sqrt(a * a + b * b);

    for (int i = 0; i < 18; ++i) {
        leftParams_[i] = 0.0;
        rightParams_[i] = 0.0;
    }

    leftParams_[0] = rightParams_[0] = cc;
    leftParams_[3] = rightParams_[3] = d;
    leftParams_[6] = rightParams_[6] = abMag * principal_distance_;
    leftParams_[15] = rightParams_[15] = std::atan2(b, a);
}

void MPC::solve(const std::vector<double>& xMeas, const std::vector<double>& yMeas, double* params, const std::string& topic) {
    std::cout << topic << std::endl;

    ceres::Problem problem;
    for (size_t i = 0; i < gcps_.size(); ++i) {
        size_t idx = gcps_[i];
        ceres::CostFunction* costFun = new ceres::AutoDiffCostFunction<ResectionCost, 2, 18>(
            new ResectionCost(xMeas[idx], yMeas[idx], tVals_[idx], Gx_[idx], Gy_[idx], Gz_[idx], principal_distance_));
        problem.AddResidualBlock(costFun, nullptr, params);
    }

    ceres::Solver::Options opts;
    opts.minimizer_progress_to_stdout = true;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.max_num_iterations = 100;
    opts.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

    ceres::Solver::Summary summary;
    ceres::Solve(opts, &problem, &summary);
    std::cout << summary.FullReport() << "\n\n";
}

void MPC::solveForLeftImage() {
    solve(Lx_, Ly_, leftParams_, "=========================== Solving for Left image ===============================");
}

void MPC::solveForRightImage() {
    solve(Rx_, Ry_, rightParams_, "=========================== Solving for Right image ===============================");
}

double MPC::computeRMSE(const std::vector<size_t>& indices, const std::vector<double>& xMeas,
                        const std::vector<double>& yMeas, const double* params) {
    double sumSquaredResiduals = 0.0;
    for (size_t i : indices) {
        ResectionCost cost(xMeas[i], yMeas[i], tVals_[i], Gx_[i], Gy_[i], Gz_[i], principal_distance_);
        double residuals[2];
        cost(params, residuals);
        sumSquaredResiduals += residuals[0] * residuals[0] + residuals[1] * residuals[1];
    }
    return std::sqrt(sumSquaredResiduals / indices.size());
}

void MPC::computeRMSEForLeftImage() {
    double rmse = computeRMSE(icps_, Lx_, Ly_, leftParams_);
    std::cout << "Left Image RMSE: " << rmse << std::endl;
}

void MPC::computeRMSEForRightImage() {
    double rmse = computeRMSE(icps_, Rx_, Ry_, rightParams_);
    std::cout << "Right Image RMSE: " << rmse << std::endl;
}
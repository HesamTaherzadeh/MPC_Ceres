#include "MPC.hpp"

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    std::string csv_path = (argc < 2) ? "data/tagged.csv" : argv[1];
    double principal_distance = 1.082;
    double pixel_size = 1.3e-05;
    int num_rows = 6000;

    MPC mpc(csv_path, principal_distance, pixel_size, num_rows);

    mpc.solveForLeftImage();
    mpc.solveForRightImage();

    mpc.computeRMSEForLeftImage();
    mpc.computeRMSEForRightImage();

    return 0;
}
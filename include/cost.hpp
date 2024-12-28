struct ResectionCost {
    ResectionCost(double observedX, double observedY,
                  double timeValue,
                  double groundX, double groundY, double groundZ,
                  double focalLength)
      : observedX_(observedX),
        observedY_(observedY),
        time_(timeValue),
        groundX_(groundX),
        groundY_(groundY),
        groundZ_(groundZ),
        focalLength_(focalLength) {}

    template <typename T>
    bool operator()(const T* const parameters, T* residuals) const {
        const T& X0 = parameters[0];  const T& X1 = parameters[1];  const T& X2 = parameters[2];
        const T& Y0 = parameters[3];  const T& Y1 = parameters[4];  const T& Y2 = parameters[5];
        const T& Z0 = parameters[6];  const T& Z1 = parameters[7];  const T& Z2 = parameters[8];
        const T& W0 = parameters[9];  const T& W1 = parameters[10]; const T& W2 = parameters[11];
        const T& P0 = parameters[12]; const T& P1 = parameters[13]; const T& P2 = parameters[14];
        const T& K0 = parameters[15]; const T& K1 = parameters[16]; const T& K2 = parameters[17];

        T timeVariable = T(time_);

        T Xs = X0 + X1 * timeVariable + X2 * timeVariable * timeVariable;
        T Ys = Y0 + Y1 * timeVariable + Y2 * timeVariable * timeVariable;
        T Zs = Z0 + Z1 * timeVariable + Z2 * timeVariable * timeVariable;

        T wAngle = W0 + W1 * timeVariable + W2 * timeVariable * timeVariable;
        T pAngle = P0 + P1 * timeVariable + P2 * timeVariable * timeVariable;
        T kAngle = K0 + K1 * timeVariable + K2 * timeVariable * timeVariable;

        T cosW = ceres::cos(wAngle); T sinW = ceres::sin(wAngle);
        T cosP = ceres::cos(pAngle); T sinP = ceres::sin(pAngle);
        T cosK = ceres::cos(kAngle); T sinK = ceres::sin(kAngle);

        T rotationW[9] = {
            T(1), T(0), T(0),
            T(0), cosW, -sinW,
            T(0), sinW, cosW
        };

        T rotationP[9] = {
            cosP, T(0), sinP,
            T(0), T(1), T(0),
            -sinP, T(0), cosP
        };

        T rotationK[9] = {
            cosK, -sinK, T(0),
            sinK, cosK, T(0),
            T(0), T(0), T(1)
        };

        auto multiply3x3Matrices = [&](const T* matrixA, const T* matrixB, T* resultMatrix) {
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    resultMatrix[3 * row + col] = matrixA[3 * row + 0] * matrixB[3 * 0 + col]
                                                + matrixA[3 * row + 1] * matrixB[3 * 1 + col]
                                                + matrixA[3 * row + 2] * matrixB[3 * 2 + col];
                }
            }
        };

        T rotationKP[9], rotationMatrix[9];
        multiply3x3Matrices(rotationK, rotationP, rotationKP);
        multiply3x3Matrices(rotationKP, rotationW, rotationMatrix);

        T deltaX = T(groundX_) - Xs;
        T deltaY = T(groundY_) - Ys;
        T deltaZ = T(groundZ_) - Zs;

        T transformedX = rotationMatrix[0] * deltaX + rotationMatrix[1] * deltaY + rotationMatrix[2] * deltaZ;
        T transformedY = rotationMatrix[3] * deltaX + rotationMatrix[4] * deltaY + rotationMatrix[5] * deltaZ;
        T transformedZ = rotationMatrix[6] * deltaX + rotationMatrix[7] * deltaY + rotationMatrix[8] * deltaZ;

        T projectedX = -T(focalLength_) * (transformedX / transformedZ);
        T projectedY = -T(focalLength_) * (transformedY / transformedZ);

        residuals[0] = projectedX;
        residuals[1] = projectedY - T(observedY_);

        return true;
    }

private:
    double observedX_;
    double observedY_;
    double time_;
    double groundX_, groundY_, groundZ_;
    double focalLength_;
};
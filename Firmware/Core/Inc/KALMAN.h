#ifndef KALMAN_H
#define KALMAN_H

#include "Eigen/Dense"

class KALMAN
{
private:
    Eigen::MatrixXf A, B, C, Q, R, I, u, y, K, P_minus, P, x_minus, x;

public:
    void Initialise(Eigen::MatrixXf object_A, Eigen::MatrixXf object_B, Eigen::MatrixXf object_C,
                    Eigen::MatrixXf object_Q, Eigen::MatrixXf object_R, Eigen::MatrixXf object_I,
                    Eigen::MatrixXf object_P, Eigen::MatrixXf object_x);
    Eigen::MatrixXf Estimate(Eigen::MatrixXf object_u, Eigen::MatrixXf object_y);
};

#endif

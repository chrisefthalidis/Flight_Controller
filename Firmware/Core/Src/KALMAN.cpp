#include "kalman.h"

void KALMAN::Initialise(Eigen::MatrixXf object_A, Eigen::MatrixXf object_B, Eigen::MatrixXf object_C,
                        Eigen::MatrixXf object_Q, Eigen::MatrixXf object_R, Eigen::MatrixXf object_I,
                        Eigen::MatrixXf object_P, Eigen::MatrixXf object_x)
{
    A.resize(object_A.rows(), object_A.cols());
    B.resize(object_B.rows(), object_B.cols());
    C.resize(object_C.rows(), object_C.cols());
    Q.resize(object_Q.rows(), object_Q.cols());
    R.resize(object_R.rows(), object_R.cols());
    I.resize(object_I.rows(), object_I.cols());
    P.resize(object_P.rows(), object_P.cols());
    x.resize(object_x.rows(), object_x.cols());

    A << object_A;
    B << object_B;
    C << object_C;
    Q << object_Q;
    R << object_R;
    I << object_I;
    P << object_P;
    x << object_x;
}

Eigen::MatrixXf KALMAN::Estimate(Eigen::MatrixXf object_u, Eigen::MatrixXf object_y)
{
    u.resize(object_u.rows(), object_u.cols());
    y.resize(object_y.rows(), object_y.cols());

    u << object_u;
    y << object_y;

    // Prediction
    x_minus = (A * x) + (B * u);
    P_minus = A * P * A.transpose() + Q;

    // Correction
    K = (P_minus * C.transpose()) * (C * P_minus * C.transpose() + R).inverse();
    x = x_minus + K * (y - C * x_minus);
    P = (I - K * C) * P_minus;

    return x;
}
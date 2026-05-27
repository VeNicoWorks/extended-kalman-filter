#pragma once
#include <Eigen/Dense>

class ExtendedKalmanFilter {

public:
//Creates State Vector x and other variables. 

    Eigen::Vector4d x; //The state vector that holds our x/y comps as well as the respective velocities

    Eigen::Matrix4d P; // covariance matrix gives uncertainty

    Eigen::Matrix4d F; //Our transition matrix, kinematics predicting future motion

    Eigen::Matrix4d Q; //Process noise matrix, basically just accounts for like the chaos of the real world

    Eigen::Matrix2d R; //Measurement noise matrix, accounts for imperfections in our radar system/hardware

    Eigen::MatrixXd H; //Actual Measurement matrix

    ExtendedKalmanFilter();

Eigen::MatrixXd CalcJacobian(const Eigen::VectorXd &x_state);

//Declarations for our prediction and correction steps
void prediction_step(double deltat);
void correction_step(const Eigen::Vector2d &sensor_measurement);
};



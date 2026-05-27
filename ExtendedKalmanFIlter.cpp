#include <iostream>
#include "main.hpp"
#include <Eigen/Dense>
#include <cmath>

ExtendedKalmanFilter::ExtendedKalmanFilter(){

    x << 0.0,0.0,0.0,0.;

    P = Eigen::Matrix4d::Identity() * 1000.0;

    F << Eigen::Matrix4d::Identity();

    Q << Eigen::Matrix4d::Zero();

    R << 1.0, 0.0,
         0.0, 0.05;
    H = Eigen::MatrixXd(2,4);
    H << 1.0,0.0,0.0,0.0,
         0.0,1.0,0.0,0.0;
}
//Defines prediction step, allows you to calculate powers of deltat. Define inside class so its in scope
void ExtendedKalmanFilter::prediction_step(double deltat){
    double deltat_squared = deltat * deltat;
    double deltat_cubed = deltat_squared * deltat;
    double deltat_fourthpow = deltat_cubed * deltat;

    F(0,2) = deltat;
    F(1,3) = deltat;

    double Q_accelx = 9.0;
    double Q_accely = 9.0;

    Q << (deltat_fourthpow / 4.0) * Q_accelx, 0.0, (deltat_cubed/2.0) * Q_accelx, 0.0,
        0.0, (deltat_fourthpow / 4.0) * Q_accely, 0.0, (deltat_cubed/2.0) * Q_accely, 
        (deltat_cubed / 2.0) * Q_accelx, 0.0, deltat_squared * Q_accelx, 0.0,
        0.0, (deltat_cubed / 2.0) * Q_accely, 0.0, deltat_squared * Q_accely;

    //Predict future path, take object state vector forwards in time
    x = F * x;
    P = F * P * F.transpose() + Q;
}

void ExtendedKalmanFilter::correction_step(const Eigen::Vector2d &z){
 
  //Setting our state vector up so we can convert it into polar coords
  double position_x = x(0);
  double position_y = x(1);

  double range = std::sqrt(position_x * position_x + position_y * position_y);
  double bearing = std::atan2(position_y, position_x);
    
    Eigen::VectorXd polar_coords = Eigen::VectorXd(2);
    polar_coords << range, bearing;

    //Getting innovative, here we calculate the innovation
    Eigen::VectorXd y = z - polar_coords;

    //Normalize bearing angle of y to avoid massive errors when func goes beyond 180 or -180
    while (y(1) > M_PI) y(1) = y(1) - 2.0 * M_PI;
    while (y(1) < -M_PI) y(1) = y(1) + 2.0 * M_PI;

    H = CalcJacobian(x);
//H transpose and residual covariance (S)
    Eigen::MatrixXd Ht = H.transpose();

    Eigen::MatrixXd S = H * P * Ht + R;

//Kalman Gain defined as K
    Eigen::MatrixXd K = P * Ht * S.inverse();

//Update state vector/position
    x = x + K * y;
//Update state covariance/uncertainty P by utilizing I identity matrix
    long x_size = x.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size,x_size);

    P = (I - K * H) * P;
    
}

Eigen::MatrixXd ExtendedKalmanFilter::CalcJacobian(const Eigen::VectorXd &x_state){

    Eigen::MatrixXd Hj = Eigen::MatrixXd::Zero(2,4);

    double position_x = x_state(0);
    double position_y= x_state(1);

    //Range equation to calculate object distance
    double squared_positions = position_x * position_x + position_y * position_y;
    double range = std::sqrt(squared_positions);

    //Test for edge case where we divide by zero
    if (std::abs(squared_positions) < 0.000001){
        std::cout << "Error, can't divide by zero pal" << std::endl;
        return Hj;
    }
// Row 1: Range ROC. Row 2: Bearing ROC
    Hj << (position_x / range), (position_y / range), 0, 0,
          -(position_y / squared_positions), (position_x / squared_positions), 0, 0;

        return Hj;
}
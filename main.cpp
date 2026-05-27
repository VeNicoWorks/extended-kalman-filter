#include <iostream>
#include "main.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <vector>

int main(){
    //Opens dataset created by python
    std::ifstream infile("sensor_data.txt");
    //Defines output file
    std::ofstream outfile("ekf_estimate.txt");
    if (!infile.is_open()){
        std::cerr << "Error: sensor_data.txt is NOT opening" << std::endl;
        return 1;
    }

    //Here we are gonna have our variables that are being tracked
    double timestamp;
    double real_x;
    double real_y;

    // Our Noisy range and bearing in this case are our initial polar values
    double noisy_range;
    double noisy_bearing;

    //Track timestamps to calculate dt n write filter states

    bool is_initialized = false;
    double last_timestamp = 0.0;

    std::cout << "Reading data rn wait a second for results";

    //Extended Kalman Filter execution
    ExtendedKalmanFilter EKF;

    while(infile >> timestamp >> real_x >> real_y >> noisy_range >> noisy_bearing){
            
            //Create measurement vector Z
            Eigen::VectorXd z(2);
            z << noisy_range,noisy_bearing;


        //Initialize the state vector and convert back to Cartesian
        if (!is_initialized){
            double initial_x  = noisy_range * cos(noisy_bearing);
            double initial_y = noisy_range * sin(noisy_bearing);
            
            //Initialize state vector with the assumption that we have 0 initial velocity
            EKF.x = Eigen::VectorXd(4);
            EKF.x << initial_x, initial_y, 0.0, 0.0;

            last_timestamp = timestamp;
            is_initialized = true;
            continue;
            
            
        }
        //Our delta t, changes in time
        double dt = timestamp - last_timestamp;
        last_timestamp = timestamp;
        EKF.prediction_step(dt);
        EKF.correction_step(z);

        //Export estimates by using an output file
        outfile << timestamp << " " << EKF.x(0) << " " << EKF.x(1) << " " << EKF.x(2) << " " << EKF.x(3) << "\n";

    }

    infile.close();
    outfile.close();
    std::cout << "We have tracked and corrected the noisy mivement. Results can be found is ekf_estimate.txt" << "\n";
    return 0;
}
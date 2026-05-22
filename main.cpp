#include <iostream>
#include <Eigen/Dense>

void predictNextState(Eigen::MatrixXd &state, Eigen::MatrixXd &transition){
    state = transition * state;
}
int main(){
Eigen::MatrixXd x(2,1);
x << 8.0,
     2.0;
Eigen::MatrixXd F(2,2);
F << 1.0, 3.0,
     0.0, 1.0;


std::cout << "We are starting off with " << x << "\n\n";
predictNextState(x,F);



std::cout << "After 1 second the predicted state is " << x << "\n";

return 0;
}
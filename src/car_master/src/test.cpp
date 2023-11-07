#include "traj.hpp"
#include <iostream>
using namespace std;
using namespace Eigen;
int main()
{
    MatrixXd Path(4, 3);
    MatrixXd Vel = MatrixXd::Zero(2, 3);
    MatrixXd Acc = MatrixXd::Zero(2, 3);
    Path << 0, 0, 0,
        1, 1, 0,
        2, 2, 0,
        3, 3, 0;
    Traj::PolyTrajectory poly(4);
    poly.PolyQPGeneration(Path, Vel, Acc);
    cout << poly.PolyCoeff << endl;
    cout << poly.Time << endl;
    poly.TimeScaling(0.5);
    cout << poly.PolyCoeff << endl;
    cout << poly.Time << endl;

    return 0;
}

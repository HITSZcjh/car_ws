#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include <minimumsnap/PolynomialTrajectory.h>
using namespace std;
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}
extern ros::Publisher _polynomial_traj_coef_pub;
// define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for (int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}
/*

    STEP 2: Learn the "Closed-form solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time) // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order = 2 * d_order - 1; // the order of polynomial
    int p_num1d = p_order + 1;     // the number of variables in each segment

    int m = Time.size();                                 // the number of segments
    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d); // position(x,y,z), so we need (3 * p_num1d) coefficients
    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

    /*   Produce Mapping Matrix A to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */


    MatrixXd M = MatrixXd::Zero(p_num1d * m, p_num1d * m);
    for (int i = 0; i < m; i++)
    {
        int mid_num1 = i * p_num1d;
        for (int j = 0; j < d_order; j++)
        {
            M(mid_num1 + j, mid_num1 + j) = TrajectoryGeneratorWaypoint::Factorial(j);
        }
        for (int j = 0; j < d_order; j++)
        {
            for (int k = j; k < p_num1d; k++)
                M(mid_num1 + d_order + j, mid_num1 + k) = TrajectoryGeneratorWaypoint::Factorial(k) /
                                                          TrajectoryGeneratorWaypoint::Factorial(k-j) * pow(Time(i), k - j);
        }
    }


    /*   Produce the dereivatives in X, Y and Z axis directly.  */

    int df_size = 3+(m-1)+3;

    MatrixXd C_T = MatrixXd::Zero(p_num1d * m, d_order * (m + 1));
    C_T(0, 0) = 1;
    C_T(1, 1) = 1;
    C_T(2, 2) = 1;
    C_T(d_order * (2*m-1),df_size-3) = 1;
    C_T(d_order * (2*m-1)+1,df_size-2) = 1;
    C_T(d_order * (2*m-1)+2,df_size-1) = 1;
    for(int i = 3;i<d_order;i++)
    {
        C_T(i, df_size+(i-2)-1) = 1;
        C_T(d_order * (2*m-1)+i,d_order * (m + 1)-(d_order-1-3)+(i-3)-1) = 1;
    }
    for(int i = 0; i<(m-1);i++)
    {
        C_T((2*i+1)*d_order, 3+i) = 1;
        C_T((2*i+2)*d_order, 3+i) = 1;
        for(int j=1; j<d_order; j++)
        {
            C_T((2*i+1)*d_order+j, df_size+(d_order-3)+i*(d_order-1)+j-1) = 1;
            C_T((2*i+2)*d_order+j, df_size+(d_order-3)+i*(d_order-1)+j-1) = 1;
        }
    }

    /*   Produce the Minimum Snap cost function, the Hessian Matrix   */

    VectorXd mid_num1 = VectorXd::Zero(p_order-d_order+1);
    for(int i =0; i<mid_num1.size();i++)
        mid_num1(i) = TrajectoryGeneratorWaypoint::Factorial(d_order+i)/TrajectoryGeneratorWaypoint::Factorial(i);
    MatrixXd Q = MatrixXd::Zero(p_num1d* m, p_num1d* m);
    for(int i = 0; i<m; i++)
    {
        for(int j =0;j<p_order-d_order+1;j++)
            for(int k =0;k<p_order-d_order+1;k++)
            {
                Q(j+d_order+i*p_num1d,k+d_order+i*p_num1d) = mid_num1(j)*mid_num1(k)*pow(Time(i),j+k+1)/(j+k+1);
            }
    }
    MatrixXd M_inverse = M.inverse();
    MatrixXd R = C_T.transpose()*M_inverse.transpose()*Q*M_inverse*C_T;

    
    MatrixXd df(Path.rows()+4,3);
    df.row(0) = Path.row(0);
    df.row(1) = Vel.row(0);
    df.row(2) = Acc.row(0);
    df.block(3,0,Path.rows()-1,3) = Path.block(1,0,Path.rows()-1,3);
    df.row(df.rows()-2) = Vel.row(1);
    df.row(df.rows()-1) = Acc.row(1);

    MatrixXd Rpp = R.block(df_size,df_size,R.rows()-df_size,R.cols()-df_size);
    MatrixXd Rfp = R.block(0,df_size,df_size,R.cols()-df_size);

    MatrixXd dp = -Rpp.inverse()*Rfp.transpose()*df;


    MatrixXd dmid(df.rows()+dp.rows(),df.cols());
    dmid << df, dp;
    MatrixXd temp = M_inverse*C_T*dmid;
    for(int i = 0; i< m;i++)
    {
        PolyCoeff.row(i) << temp.block(i*p_num1d,0,p_num1d,1).transpose(),temp.block(i*p_num1d,1,p_num1d,1).transpose(),temp.block(i*p_num1d,2,p_num1d,1).transpose();
    }

    minimumsnap::PolynomialTrajectory coef;
    coef.header.stamp = ros::Time::now();
    for(int i=0;i<temp.rows();i++)
    {
        coef.coef_x.push_back(temp(i,0));
        coef.coef_y.push_back(temp(i,1));
        coef.coef_z.push_back(temp(i,2));
    }
    coef.order = p_order;
    for(int i=0;i<Time.size();i++)
    {
        coef.time.push_back(Time(i));
    }
    _polynomial_traj_coef_pub.publish(coef);
    return PolyCoeff;
}

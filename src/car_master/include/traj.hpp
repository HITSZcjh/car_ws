#pragma once
#include <math.h>
#include <Eigen/Eigen>

namespace Traj
{
    class CircleTrajectory
    {
    public:
        Eigen::Vector2d pos;
        Eigen::Vector2d vel;
        Eigen::Vector2d origin;
        double radius;
        double omega;
        double theta;
        CircleTrajectory(Eigen::Vector2d origin, double radius, double omega, double theta) : origin(origin), radius(radius), omega(omega), theta(theta)
        {
        }
        void step(double t);
    };

    class PolyTrajectory
    {
    private:
        int d_order; // the order of derivative
        int p_order; // the order of polynomial

        int Factorial(int x);
        Eigen::VectorXd timeAllocation(const Eigen::MatrixXd &Path);

    public:
        int p_num1d; // the number of variables in each segment
        Eigen::VectorXd Time;
        Eigen::MatrixXd PolyCoeff;
        double max_vel;
        double max_acc;
        int n_segment;
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;

        Eigen::MatrixXd Path; // waypoints coordinates (3d)
        Eigen::MatrixXd BoundVel;  // boundary velocity
        Eigen::MatrixXd BoundAcc;  // boundary acceleration

        PolyTrajectory(int d_order, double max_vel = 1.0, double max_acc = 1.0) : d_order(d_order), p_order(2 * d_order - 1), p_num1d(2 * d_order), max_vel(max_vel), max_acc(max_acc)
        {
        }

        void PolyQPGeneration();

        void TimeScaling(double alpha); // alpha < 1, slower; alpha > 1, faster

        void step(double t);

        void sample2D(int num, Eigen::MatrixX2d& point_list);
    };

}
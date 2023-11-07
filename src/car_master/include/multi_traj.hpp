#pragma once

#include "traj.hpp"
#include <memory>
namespace MultiTraj
{
    using namespace Traj;
    using namespace Eigen;

    class MultiTrajectory
    {
    private:
        int traj_num;
        
        std::vector<std::shared_ptr<PolyTrajectory>> poly_traj_list;
        std::vector<std::shared_ptr<CircleTrajectory>> circle_traj_list;
        std::vector<MatrixX2d> sample_point_list;
        double time_now;
        enum traj_state
        {
            INIT = 0,
            PLANNING,
            PERFORM
        } state;
        MatrixX2d init_pos;
        MatrixX2d init_vel;
    public:
        MultiTrajectory(int traj_num, MatrixX2d init_pos, MatrixX2d init_vel, Vector2d circle_origin, double circle_radius);
        void Planning();
        void Sample2D(std::shared_ptr<PolyTrajectory> poly, std::shared_ptr<CircleTrajectory> circle, int num, int T, MatrixX2d& point_list);
        MatrixXd GetPosAndVel(double t);

    };
}
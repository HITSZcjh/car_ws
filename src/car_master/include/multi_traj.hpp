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
        std::vector<std::unique_ptr<PolyTrajectory>> poly_traj_list;
        std::vector<std::unique_ptr<CircleTrajectory>> circle_traj_list;
        std::vector<MatrixX2d> sample_point_list;
        double time_now;
        enum traj_state
        {
            INIT = 0,
            POLY,
            CIRCLE
        } state;
        MatrixX2d init_pos;
        MatrixX2d init_vel;
    public:
        MultiTrajectory(int traj_num, MatrixX2d init_pos, MatrixX2d init_vel);
        void Loop();
        void Sample2D(std::unique_ptr<PolyTrajectory> poly, std::unique_ptr<CircleTrajectory>, int num, MatrixX2d& point_list);
    };
}
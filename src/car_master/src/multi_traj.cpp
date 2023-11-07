#include "multi_traj.hpp"
#include <cmath>
#include <limits>
namespace MultiTraj
{
    using namespace Traj;
    using namespace Eigen;

    MultiTrajectory::Sample2D()

    MultiTrajectory::MultiTrajectory(int traj_num, MatrixX2d init_pos, MatrixX2d init_vel) : traj_num(traj_num), time_now(0.0), state(INIT), init_pos(init_pos), init_vel(init_vel)
    {
        poly_traj_list.resize(traj_num);
        circle_traj_list.resize(traj_num);
        for (int i = 0; i < traj_num; i++)
        {
            poly_traj_list[i] = std::make_unique<PolyTrajectory>(4, 0.25, 0.25);

            poly_traj_list[i]->Path = MatrixXd::Zero(2, 3);
            poly_traj_list[i]->Path.row(0).segment(0, 2) = init_pos.row(i);

            poly_traj_list[i]->BoundVel = MatrixXd::Zero(2, 3);
            poly_traj_list[i]->BoundVel.row(0).segment(0, 2) = init_vel.row(i);

            poly_traj_list[i]->BoundAcc = MatrixXd::Zero(2, 3);
        }
    }

    void MultiTrajectory::Loop()
    {
        switch (state)
        {
        case INIT:
        {
            Vector2d origin(1.0, 0.0);

            for (int i = 0; i < traj_num; i++)
            {
                double theta = atan2(init_pos(i, 1) - origin(1), init_pos(i, 0) - origin(0));
                circle_traj_list[i] = std::make_unique<CircleTrajectory>(origin, 1.0, 0.25, theta);
                circle_traj_list[i]->step(0); // 获取入圆点位置与速度

                poly_traj_list[i]->Path.row(1).segment(0, 2) = circle_traj_list[i]->pos;
                poly_traj_list[i]->BoundVel.row(1).segment(0, 2) = circle_traj_list[i]->vel;

                poly_traj_list[i]->PolyQPGeneration();
            }
            state = POLY;
            break;
        }
        case POLY:
        {
            const double min_disance = 0.2;
            const int sample_num = 100;

            sample_point_list.resize(traj_num);
            poly_traj_list[0]->sample2D(sample_num, sample_point_list[0]);

            for (int scaling_num = 1; scaling_num < traj_num; scaling_num++)
            {
                for (int i = scaling_num - 1; i >= 0; i--)
                {
                    double distance = std::numeric_limits<double>::infinity();

                    while (1)
                    {
                        poly_traj_list[scaling_num]->sample2D(sample_num, sample_point_list[scaling_num]);

                        MatrixX2d distance_matrix = sample_point_list[scaling_num] - sample_point_list[i];
                        VectorXd distance_vector = distance_matrix.rowwise().norm();
                        distance = distance_vector.maxCoeff();

                        if (distance < min_disance)
                            break;
                        else
                            poly_traj_list[scaling_num]->TimeScaling(0.9);
                    }
                }
            }
            break;
        }
        case CIRCLE:
        {
            
            break;
        }
    }

}

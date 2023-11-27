#include "multi_traj.hpp"
#include <cmath>
#include <limits>

#include <ros/ros.h>
namespace MultiTraj
{
    using namespace Traj;
    using namespace Eigen;

    void MultiTrajectory::Sample2D(std::shared_ptr<PolyTrajectory> poly, std::shared_ptr<CircleTrajectory> circle, int num, int T, MatrixX2d &point_list)
    {
        point_list = MatrixXd::Zero(num, 2);
        double delta_t = T / (num - 1);
        for (int i = 0; i < num; i++)
        {
            double t_now = i * delta_t;
            if (t_now > poly->Time.sum())
            {
                t_now -= poly->Time.sum();
                circle->step(t_now);
                point_list.row(i) = circle->pos;
            }
            else
            {
                poly->step(t_now);
                point_list.row(i) = poly->pos.segment(0, 2);
            }
        }
    }

    MultiTrajectory::MultiTrajectory(int traj_num, MatrixX2d init_pos, MatrixX2d init_vel, Vector2d circle_origin, double circle_radius) : 
    traj_num(traj_num), init_pos(init_pos), init_vel(init_vel)
    {
        poly_traj_list.resize(traj_num);
        circle_traj_list.resize(traj_num);
        for (int i = 0; i < traj_num; i++)
        {
            poly_traj_list[i] = std::make_shared<PolyTrajectory>(4, 0.1, 0.1);

            poly_traj_list[i]->Path = MatrixXd::Zero(2, 3);
            poly_traj_list[i]->Path.row(0).segment(0, 2) = init_pos.row(i);

            poly_traj_list[i]->BoundVel = MatrixXd::Zero(2, 3);
            poly_traj_list[i]->BoundVel.row(0).segment(0, 2) = init_vel.row(i);

            poly_traj_list[i]->BoundAcc = MatrixXd::Zero(2, 3);
        }

        for (int i = 0; i < traj_num; i++)
        {
            double theta = atan2(init_pos(i, 1) - circle_origin(1), init_pos(i, 0) - circle_origin(0));
            circle_traj_list[i] = std::make_unique<CircleTrajectory>(circle_origin, circle_radius, 0.2, theta);
            circle_traj_list[i]->step(0); // 获取入圆点位置与速度

            poly_traj_list[i]->Path.row(1).segment(0, 2) = circle_traj_list[i]->pos;
            poly_traj_list[i]->BoundVel.row(1).segment(0, 2) = circle_traj_list[i]->vel;

        }
    }

    void MultiTrajectory::Planning()
    {
        for (int i = 0; i < traj_num; i++)
        {
            poly_traj_list[i]->PolyQPGeneration();
        }

        constexpr double min_disance = 0.4;
        constexpr int sample_num = 100;

        sample_point_list.resize(traj_num);
        Sample2D(poly_traj_list[0], circle_traj_list[0], sample_num, poly_traj_list[0]->Time.sum(), sample_point_list[0]);

        for (int scaling_num = 1; scaling_num < traj_num; scaling_num++)
        {
            for (int i = scaling_num - 1; i >= 0; i--)
            {
                double distance = std::numeric_limits<double>::infinity();

                while (1)
                {
                    Sample2D(poly_traj_list[scaling_num], circle_traj_list[scaling_num], sample_num, poly_traj_list[scaling_num]->Time.sum(), sample_point_list[scaling_num]);
                    Sample2D(poly_traj_list[i], circle_traj_list[i], sample_num, poly_traj_list[scaling_num]->Time.sum(), sample_point_list[i]);

                    MatrixX2d distance_matrix = sample_point_list[scaling_num] - sample_point_list[i];
                    VectorXd distance_vector = distance_matrix.rowwise().norm();
                    distance = distance_vector.minCoeff();

                    if (distance > min_disance)
                    {
                        int check_num = 0;
                        for (int j = scaling_num - 1; j > i; j--)
                        {
                            Sample2D(poly_traj_list[j], circle_traj_list[j], sample_num, poly_traj_list[scaling_num]->Time.sum(), sample_point_list[j]);
                            MatrixX2d distance_matrix = sample_point_list[scaling_num] - sample_point_list[j];
                            VectorXd distance_vector = distance_matrix.rowwise().norm();
                            distance = distance_vector.minCoeff();
                            if (distance > min_disance)
                                check_num++;
                            else
                                break;
                        }
                        if(check_num == scaling_num - i - 1)
                            break;
                        else
                            poly_traj_list[scaling_num]->TimeScaling(0.9);
                    }
                    else
                        poly_traj_list[scaling_num]->TimeScaling(0.9);
                }
            }
        }
    }

    MatrixXd MultiTrajectory::GetPosAndVel(double t)
    {
        MatrixXd output = MatrixXd::Zero(traj_num, 4);
        for (int i = 0; i < traj_num; i++)
        {
            double time = t;
            if (time > poly_traj_list[i]->Time.sum())
            {
                time -= poly_traj_list[i]->Time.sum();
                circle_traj_list[i]->step(time);
                output.row(i).segment(0, 2) = circle_traj_list[i]->pos;
                output.row(i).segment(2, 2) = circle_traj_list[i]->vel;
            }
            else
            {
                poly_traj_list[i]->step(time);
                output.row(i).segment(0, 2) = poly_traj_list[i]->pos.segment(0, 2);
                output.row(i).segment(2, 2) = poly_traj_list[i]->vel.segment(0, 2);
            }
        }
        return output;
    }

}

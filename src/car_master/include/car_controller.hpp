#pragma once

#include <Eigen/Eigen>
#include <math.h>

namespace CarController
{
    using namespace Eigen;
    Vector2d real_pos;
    Vector2d real_vel;
    double real_theta; // -pi~pi
    
    double vel_max;
    double vel_min;

    class PID_t
    {
    public:
        double kp;
        double ki;
        double kd;
        double fdb;
        double ref;
        double error;
        double last_error;
        double total_error;
        double output;
        double output_max;
        double output_min;
        double p_output;
        double d_output;
        double i_output;
        double ts;
        /* data */

        PID_t(double kp, double ki, double kd, double output_max, double output_min, double ts)
            : kp(kp), ki(ki), kd(kd), output_max(output_max), output_min(output_min), ts(ts){};

        void PID_Calculate();
        void PID_Calculate_for_theta();
        ~PID_t(){};
    };

    class VelController
    {
    private:
        PID_t pid_x;
        PID_t pid_y;
    public:
        VelController(double ki, double ts) : 
        pid_x(0.0, ki, 0.0, vel_max, vel_min, ts), pid_y(0.0, ki, 0.0, vel_max, vel_min, ts){};
        Vector2d ControlLoop(Vector2d& ref_vel);
    };

    class ThetaController
    {
    private:
        PID_t pid_theta; // 角度误差pid
    public:
        ThetaController(double ts) : pid_theta(1.0, 0.1, 0.15, 1.0, -1.0, ts){};
        double ControlLoop(Vector2d& ref_vel);

    };

    class PositionController
    {
    private:
        PID_t pid_c; // 径向位置误差pid
        Vector2d kl; // 径向位置速度误差系数
        double ts;
    public:
        PositionController(double ts):
        pid_c(0.5, 0.1, 0.0, vel_max, vel_min, ts),kl(1.0, 1.0):ts(ts){};

        Vector2d ControlLoop(Vector2d& ref_pos, Vector2d& ref_vel);
    };

}


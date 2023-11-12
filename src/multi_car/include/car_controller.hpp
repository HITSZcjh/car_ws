#pragma once

#include <Eigen/Eigen>
#include <math.h>

namespace CarController
{
    using namespace Eigen;

    extern Vector2d real_pos;
    extern Vector2d real_vel;
    extern double real_theta; // -pi~pi
    
    extern double vel_max;
    extern double vel_min;

    extern double omega_max;
    extern double omega_min;
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
        VelController(double kp, double ki, double kd, double ts) : 
        pid_x(kp, ki, kd, vel_max, vel_min, ts), pid_y(kp, ki, kd, vel_max, vel_min, ts){};
        void SetParam(double kp, double ki, double kd); 
        Vector2d ControlLoop(Vector2d& ref_vel);
    };

    class ThetaController
    {
    private:
        PID_t pid_theta; // 角度误差pid
    public:
        ThetaController(double kp, double ki, double kd, double ts) : pid_theta(kp, ki, kd, omega_max, omega_min, ts){};
        void SetParam(double kp, double ki, double kd); 
        double ControlLoop(double theta);
        double ControlLoop(Vector2d& ref_vel);

    };

    class PositionController
    {
    private:
        PID_t pid_c; // 径向位置误差pid
        double ts;
    public:
        PositionController(double kp, double ki, double kd, double ts):
        pid_c(kp, ki, kd, vel_max, vel_min, ts),ts(ts){};
        void SetParam(double kp, double ki, double kd); 
        Vector2d ControlLoop(Vector2d& ref_pos, Vector2d& ref_vel);
    };

}


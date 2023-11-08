#include "car_controller.hpp"

namespace CarController
{
    Vector2d real_pos;
    Vector2d real_vel;
    double real_theta; // -pi~pi
    
    double vel_max = 3.0;
    double vel_min = -3.0;

    void PID_t::PID_Calculate()
    {
        error = ref - fdb;
        p_output = kp * error;
        d_output = kd * (error - last_error) / ts;
        if (!(error > 0 && total_error > (output_max) / ki) && !(error < 0 && total_error < (output_min) / ki))
            total_error += error * ts;
        i_output = ki * total_error;
        output = p_output + i_output + d_output;
        if (output > output_max)
            output = output_max;
        else if (output < output_min)
            output = output_min;
        last_error = error;
    }
    void PID_t::PID_Calculate_for_theta()
    {
        error = ref - fdb;
        if (error > M_PI)
            error -= 2 * M_PI;
        else if (error < -M_PI)
            error += 2 * M_PI;
        p_output = kp * error;
        d_output = kd * (error - last_error) / ts;

        if (!(error > 0 && total_error > (output_max) / ki) && !(error < 0 && total_error < (output_min) / ki))
            total_error += error * ts;
        i_output = ki * total_error;
        output = p_output + i_output + d_output;
        if (output > output_max)
            output = output_max;
        else if (output < output_min)
            output = output_min;
        last_error = error;
    }

    Vector2d VelController::ControlLoop(Vector2d &ref_vel)
    {
        pid_x.fdb = real_vel(0);
        pid_x.ref = ref_vel(0);
        pid_x.PID_Calculate();

        pid_y.fdb = real_vel(1);
        pid_y.ref = ref_vel(1);
        pid_y.PID_Calculate();

        Vector2d output(pid_x.output, pid_y.output);
        return output;
    }

    double ThetaController::ControlLoop(Vector2d &ref_vel)
    {
        pid_theta.fdb = real_theta;
        pid_theta.ref = atan2(ref_vel(1), ref_vel(0));
        pid_theta.PID_Calculate_for_theta();
        return pid_theta.output;
    }

    Vector2d PositionController::ControlLoop(Vector2d &ref_pos, Vector2d &ref_vel)
    {
        // 定义y轴正向为参考速度正方向，计算x轴正方向
        Vector2d y_axis = ref_vel / ref_vel.norm();
        Vector2d x_axis = Vector2d(y_axis(1), -y_axis(0));

        Vector2d pos_error = ref_pos - real_pos;
        double pos_error_l = pos_error.dot(y_axis);
        double pos_error_c = pos_error.dot(x_axis);
        double real_vel_l = real_vel.dot(y_axis);

        pid_c.ref = pos_error_c;
        pid_c.fdb = 0; // 为了设置 pid_c.error = pos_error_c
        pid_c.PID_Calculate();
        Vector2d output_c = pid_c.output * x_axis;

        double acc = (ref_vel.squaredNorm() - real_vel_l * real_vel_l) / (2 * pos_error_l + 1e-8);

        Vector2d output_l = (real_vel_l + acc * ts) * y_axis;

        return (output_c + output_l);
    }
}

#include "car_controller.hpp"

namespace CarController
{
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

    double ThetaController::ControlLoop(Vector2d& ref_vel)
    {
        pid_theta.fdb = real_theta;
        pid_theta.ref = atan2(ref_vel(1), ref_vel(0));
        pid_theta.PID_Calculate_for_theta();
        return pid_theta.output;
    }

    Vector2d PositionController::ControlLoop(Vector2d &ref_pos, Vector2d &ref_vel)
    {
        Vector2d pos_error = ref_pos - real_pos;
        Vector2d pos_error_l = pos_error.dot(ref_vel) / ref_vel.norm() * ref_vel / ref_vel.norm();
        Vector2d pos_error_c = pos_error - pos_error_l;
        Vector2d vel_error = ref_vel - real_vel.dot(ref_vel) / ref_vel.norm() * ref_vel / ref_vel.norm();

        Matrix2d temp; // 用于计算pos_error_c与ref_vel为顺时针还是逆时针用于判断error正负
        temp.row(0) = pos_error_c;
        temp.row(1) = ref_vel;
        double det = temp.determinant();
        if (det > 0)
            pid_c.ref = pos_error_c.norm();
        else
            pid_c.ref = -pos_error_c.norm();
        pid_c.fdb = 0.0; // 为了设置error，设置ref为error， fdb为0
        pid_c.PID_Calculate();
        Vector2d output_c = pid_c.output * pos_error_c / pos_error_c.norm();

        Vector2d output_l = kl.dot(Vector2d(pos_error_l.norm(), vel_error.norm())) * pos_error_l / pos_error_l.norm();
    
        return (output_c + output_l);
    }
}

#include "car_controller.hpp"
#include <iostream>
namespace CarController
{
    using namespace std;

    Vector2d real_pos;
    Vector2d real_vel;
    double real_theta; // -pi~pi

    double vel_max = 3.0;
    double vel_min = -3.0;
    double omega_max = 2.0;
    double omega_min = -2.0;

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

    double LPF_t::LPF_Calculate(double input)
    {
        output = (cutoff_freq * ts * input + last_output) / (1 + cutoff_freq * ts);
        last_output = output;
        return output;
    }

    void VelController::SetParam(double kp, double ki, double kd)
    {
        pid_x.kp = kp;
        pid_x.ki = ki;
        pid_x.kd = kd;
        pid_x.total_error = 0;
        pid_y.kp = kp;
        pid_y.ki = ki;
        pid_y.kd = kd;
        pid_y.total_error = 0;
        cout<<"VelController Update Param to: kp "+std::to_string(kp)+" ki "+std::to_string(ki)+" kd "+std::to_string(kd)<<endl;
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

    void ThetaController::SetParam(double kp, double ki, double kd)
    {
        pid_theta.kp = kp;
        pid_theta.ki = ki;
        pid_theta.kd = kd;
        pid_theta.total_error = 0;
        cout<<"ThetaController Update Param to: kp "+std::to_string(kp)+" ki "+std::to_string(ki)+" kd "+std::to_string(kd)<<endl;
    }

    double ThetaController::ControlLoop(double theta)
    {
        pid_theta.fdb = real_theta;
        pid_theta.ref = theta;
        pid_theta.PID_Calculate_for_theta();
        cout << real_theta << " " << theta << endl;
        return pid_theta.output;
    }

    double ThetaController::ControlLoop(Vector2d &ref_vel)
    {
        pid_theta.fdb = real_theta;
        pid_theta.ref = atan2(ref_vel(1), ref_vel(0));
        pid_theta.PID_Calculate_for_theta();
        return pid_theta.output;
    }

    void PositionController::SetParam(double kp, double ki, double kd, double k1, double k2)
    {
        pid_c.kp = kp;
        pid_c.ki = ki;
        pid_c.kd = kd;
        pid_c.total_error = 0;
        kl[0] = k1;
        kl[1] = k2;
        cout<<"PositionController Update Param to: kp "+std::to_string(kp)+" ki "+std::to_string(ki)+" kd "+std::to_string(kd)<<endl;
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
        double vel_error_l = ref_vel.norm() - real_vel_l;

        pid_c.ref = pos_error_c;
        pid_c.fdb = 0; // 为了设置 pid_c.error = pos_error_c
        pid_c.PID_Calculate();
        Vector2d output_c = pid_c.output * x_axis;

        Vector2d output_l = (kl[0]*pos_error_l+kl[1]*vel_error_l) * y_axis;

        return (output_c + output_l);
    }
}

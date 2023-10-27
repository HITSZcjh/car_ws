#include <ros/ros.h>
#include <string>
#include <cstdlib>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "math.h"
#include <minimumsnap/PolynomialTrajectory.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
using namespace std;

ros::Publisher vel_pub;
#define PI 3.1415926535

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

    PID_t(double kp, double ki, double kd, double output_max, double output_min, double ts) : kp(kp), ki(ki), kd(kd), output_max(output_max), output_min(output_min), ts(ts){

                                                                                                                                                                      };
    void PID_Calculate()
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

    void PID_Calculate_for_theta()
    {
        error = ref - fdb;
        if (error > PI)
            error -= 2 * PI;
        else if (error < -PI)
            error += 2 * PI;
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
    ~PID_t(){

    };
};

PID_t pos_x_pid(8, 0.3, 0.15, 2.0, -2.0, 0.01);
PID_t pos_y_pid(8, 0.3, 0.15, 2.0, -2.0, 0.01);
PID_t theta_pid(1, 0.1, 0.15, 1.0, -1.0, 0.01);
PID_t vel_x_pid(0, 1, 0, 1.0, -1.0, 0.01);
PID_t vel_y_pid(0, 1, 0, 1.0, -1.0, 0.01);

// PID_t pos_x_pid(1, 0.1, 0.15, 1.0, -1.0, 0.01);
// PID_t pos_y_pid(1, 0.1, 0.15, 1.0, -1.0, 0.01);
// PID_t theta_pid(1, 0.1, 0.15, 1.0, -1.0, 0.01);
// PID_t vel_x_pid(0, 1, 0, 1.0, -1.0, 0.01);
// PID_t vel_y_pid(0, 1, 0, 1.0, -1.0, 0.01);

geometry_msgs::PoseStamped pose_msg;
void PoseCallback(const geometry_msgs::PoseStamped &msg)
{
    pose_msg = msg;
}

geometry_msgs::TwistStamped real_vel_msg;
void VelocityCallback(const geometry_msgs::TwistStamped &msg)
{
    real_vel_msg = msg;
}

// geometry_msgs::PoseStamped target_pose_msg;
// void TargetPoseCallback(const geometry_msgs::PoseStamped &msg)
// {
//     target_pose_msg = msg;
// }

void go_origin(double fdb_x, double fdb_y, double fdb_theta, double fdb_vel_x, double fdb_vel_y)
{
    pos_x_pid.fdb = fdb_x;
    pos_x_pid.ref = 0;
    pos_x_pid.PID_Calculate();

    pos_y_pid.fdb = fdb_y;
    pos_y_pid.ref = 0;
    pos_y_pid.PID_Calculate();

    theta_pid.fdb = fdb_theta;
    theta_pid.ref = 0;
    theta_pid.PID_Calculate_for_theta();

    vel_x_pid.fdb = fdb_vel_x;
    vel_x_pid.ref = pos_x_pid.output;
    vel_x_pid.PID_Calculate();

    vel_y_pid.fdb = fdb_vel_y;
    vel_y_pid.ref = pos_y_pid.output;
    vel_y_pid.PID_Calculate();

    geometry_msgs::Twist target_vel_msg;
    target_vel_msg.linear.x = vel_y_pid.output * sin(fdb_theta) + vel_x_pid.output * cos(fdb_theta);
    target_vel_msg.linear.y = vel_y_pid.output * cos(fdb_theta) - vel_x_pid.output * sin(fdb_theta);
    target_vel_msg.angular.z = theta_pid.output;

    vel_pub.publish(target_vel_msg);
}

// 移动小车的函数
void driveCar(double fdb_x, double fdb_y, double fdb_theta, double fdb_vel_x, double fdb_vel_y, double traj_pos_x, double traj_pos_y, double traj_vel_x, double traj_vel_y)
{
    pos_x_pid.fdb = fdb_x;
    pos_x_pid.ref = traj_pos_x;
    pos_x_pid.PID_Calculate();

    pos_y_pid.fdb = fdb_y;
    pos_y_pid.ref = traj_pos_y;
    pos_y_pid.PID_Calculate();

    theta_pid.fdb = fdb_theta;
    theta_pid.ref = atan2(traj_vel_y, traj_vel_x);
    theta_pid.PID_Calculate_for_theta();

    vel_x_pid.fdb = fdb_vel_x;
    vel_x_pid.ref = 0.8 * traj_vel_x + 0.2 * pos_x_pid.output;
    vel_x_pid.PID_Calculate();

    vel_y_pid.fdb = fdb_vel_y;
    vel_y_pid.ref = 0.8 * traj_vel_y + 0.2 * pos_y_pid.output;
    vel_y_pid.PID_Calculate();

    // geometry_msgs::Twist target_vel_msg;
    // target_vel_msg.linear.x = pos_y_pid.output * sin(fdb_theta) + pos_x_pid.output * cos(fdb_theta);
    // target_vel_msg.linear.y = pos_y_pid.output * cos(fdb_theta) - pos_x_pid.output * sin(fdb_theta);
    // target_vel_msg.angular.z = theta_pid.output;

    geometry_msgs::Twist target_vel_msg;
    target_vel_msg.linear.x = vel_y_pid.output * sin(fdb_theta) + vel_x_pid.output * cos(fdb_theta);
    target_vel_msg.linear.y = vel_y_pid.output * cos(fdb_theta) - vel_x_pid.output * sin(fdb_theta);
    target_vel_msg.angular.z = theta_pid.output;

    vel_pub.publish(target_vel_msg);

    cout << "*********" << endl;

    cout << "theta_pid.ref" << theta_pid.ref << "traj_vel_y" << traj_vel_y << "  "
         << "traj_vel_x" << traj_vel_x << "  "
         << "pos_x_pid.output" << pos_x_pid.output << endl;
    // cout << target_vel_msg << endl;
    // cout << real_vel_msg << endl;
    // cout << "xpid:" << pos_x_pid.fdb << " " << pos_x_pid.output << " " << target_vel_msg.linear.x << endl;
    // cout << "ypid:" << pos_y_pid.fdb << " " << pos_y_pid.output << " " << target_vel_msg.linear.y << endl;
}

class CircleTrajectory
{
public:
    double time;
    double pos_x;
    double pos_y;
    double vel_x;
    double vel_y;
    double origin_x;
    double origin_y;
    double ts;
    double radius;
    double omega;
    CircleTrajectory(double origin_x, double origin_y, double radius, double omega, double ts) : time(0), origin_x(origin_x), origin_y(origin_y), ts(ts), radius(radius), omega(omega)
    {
    }
    void step()
    {
        pos_x = origin_x + radius * sin(omega * time);
        pos_y = origin_y + radius - radius * cos(omega * time);
        vel_x = omega * radius * cos(omega * time);
        vel_y = omega * radius * sin(omega * time);
        time += ts;
    }
};

nav_msgs::Path *real_path_ptr = new nav_msgs::Path();
nav_msgs::Path *desire_path_ptr = new nav_msgs::Path();
enum ControlState
{
    INIT = 0,
    STOP,
    TRAJ
};
ControlState state = INIT;
// configuration for trajectory
int _n_segment = 0;
Eigen::VectorXd _time;
Eigen::MatrixXd _coef[3];
ros::Time _start_time, _final_time;
int _order = 0;
void rcvPolynomialTrajCoefCallBack(const minimumsnap::PolynomialTrajectory &coef)
{
    delete real_path_ptr, desire_path_ptr;
    real_path_ptr = new nav_msgs::Path();
    desire_path_ptr = new nav_msgs::Path();
    state = TRAJ;
    _order = coef.order;
    _n_segment = coef.time.size();
    _start_time = _final_time = ros::Time::now();
    _time.resize(_n_segment);

    for (int i = 0; i < _n_segment; i++)
    {
        _final_time += ros::Duration(coef.time[i]);
        _time(i) = coef.time[i];
    }

    _coef[0].resize(coef.order + 1, _n_segment);
    _coef[1].resize(coef.order + 1, _n_segment);
    _coef[2].resize(coef.order + 1, _n_segment);

    int shift = 0;
    for (int idx = 0; idx < _n_segment; ++idx)
    {

        for (int j = 0; j < (coef.order + 1); ++j)
        {
            _coef[0](j, idx) = coef.coef_x[shift + j];
            _coef[1](j, idx) = coef.coef_y[shift + j];
            _coef[2](j, idx) = coef.coef_z[shift + j];
        }

        shift += (coef.order + 1);
    }
}

int main(int argc, char **argv)
{
    cout << "你好" << endl;
    ros::init(argc, argv, "car_master");
    ros::NodeHandle nh;
    ros::Subscriber pose_sub = nh.subscribe("/vrpn_client_node/test/pose", 10, &PoseCallback);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate rate(100);
    ros::Subscriber vel_sub = nh.subscribe("/vrpn_client_node/test/twist", 10, &VelocityCallback);
    // ros::Subscriber target_pose_sub = nh.subscribe("/target_pose", 1, &TargetPoseCallback);
    ros::Subscriber polynomial_traj_coef_sub = nh.subscribe("polynomial_traj_coef", 1, rcvPolynomialTrajCoefCallBack);
    ros::Publisher real_path_pub = nh.advertise<nav_msgs::Path>("real_path",1);
    ros::Publisher desire_path_pub = nh.advertise<nav_msgs::Path>("desire_path",1);

    // while (pose_msg.header.seq == 0 || target_pose_msg.header.seq == 0 || real_vel_msg.header.seq == 0)
    //     ros::spinOnce();

    while (pose_msg.header.seq == 0 || real_vel_msg.header.seq == 0)
        ros::spinOnce();

    while (ros::ok())
    {
        cout << "state" << state << endl;
        Eigen::Quaterniond quaternion(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
                                      pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);

        Eigen::Vector3d euler = quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
        if (fabs(euler[0] - PI) > 0.2)
            euler[2] += PI;
        while (euler[2] > PI)
            euler[2] -= 2 * PI;
        while (euler[2] < -PI)
            euler[2] += 2 * PI;

        if (state == INIT || state == STOP)
        {
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = 0.0;
            vel_msg.linear.y = 0.0;
            vel_msg.angular.z = 0.0;
            vel_pub.publish(vel_msg);
        }
        else
        {
            if (state == TRAJ && ros::Time::now() > _final_time)
            {
                state = STOP;
            }
            if (state == STOP)
            {
                geometry_msgs::Twist vel_msg;
                vel_msg.linear.x = 0.0;
                vel_msg.linear.y = 0.0;
                vel_msg.angular.z = 0.0;
                vel_pub.publish(vel_msg);
            }
            else if (state == TRAJ)
            {
                double t = max(0.0, (ros::Time::now() - _start_time).toSec());

                int idx = 0;
                while (t >= _time(idx))
                {
                    t -= _time(idx);
                    idx++;
                }

                Eigen::Vector2d target_pos = Eigen::Vector2d::Zero();
                Eigen::Vector2d target_vel = Eigen::Vector2d::Zero();
                int cur_poly_num = _order + 1;
                for (int i = 0; i < cur_poly_num; i++)
                {
                    target_pos(0) += _coef[0].col(idx)(i) * pow(t, i);
                    target_pos(1) += _coef[1].col(idx)(i) * pow(t, i);

                    if (i < (cur_poly_num - 1))
                    {
                        target_vel(0) += (i + 1) * _coef[0].col(idx)(i + 1) * pow(t, i);
                        target_vel(1) += (i + 1) * _coef[1].col(idx)(i + 1) * pow(t, i);
                    }
                }
                driveCar(pose_msg.pose.position.x, pose_msg.pose.position.y, euler[2], real_vel_msg.twist.linear.x, real_vel_msg.twist.linear.y, target_pos(0), target_pos(1), target_vel(0), target_vel(1));
                geometry_msgs::PoseStamped point;
                point.header.stamp = ros::Time::now();
                point.header.frame_id = "world";
                desire_path_ptr->header.stamp = ros::Time::now();
                desire_path_ptr->header.frame_id = "world";
                point.pose.position.x = target_pos(0);
                point.pose.position.y = target_pos(1);
                desire_path_ptr->poses.push_back(point);
                desire_path_pub.publish(*desire_path_ptr);
            }
            geometry_msgs::PoseStamped point;
            point.header.stamp = ros::Time::now();
            point.header.frame_id = "world";

            real_path_ptr->header.stamp = ros::Time::now();
            real_path_ptr->header.frame_id = "world";
            point.pose.position.x = pose_msg.pose.position.x;
            point.pose.position.y = pose_msg.pose.position.y;
            real_path_ptr->poses.push_back(point);
            real_path_pub.publish(*real_path_ptr);
        }
        rate.sleep();
        ros::spinOnce();

    }
    return 0;
}
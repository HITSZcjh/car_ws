#pragma once
#include <ros/ros.h>
#include "diff_mpc_controller.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "multi_car/ContorlRef.h"
#include "car_controller.hpp"
namespace DiffMPCNode
{
    using namespace Eigen;
    class DiffMPCROSNode
    {
    private:
        ros::NodeHandle nh;
        ros::Subscriber real_pose_sub, real_vel_sub, control_ref_sub;
        ros::Publisher cmd_vel_pub;
        double ts;
        ros::Rate rate;
        std::array<bool,3> init_flag;
        double theta_bias;
        Vector2d real_pos;
        double real_vel;
        double real_omega;
        double real_theta;
        multi_car::ContorlRef control_ref_msg;
        DiffMPC::DiffMPCController diff_mpc_controller;
        CarController::PID_t vel_pid;
        CarController::LPF_t vel_lpf;
        /* data */
    public:
        DiffMPCROSNode(/* args */);
        void RealPosCallback(const geometry_msgs::PoseStamped &msg);
        void RealVelCallback(const geometry_msgs::TwistStamped &msg);
        void ControlRefCallback(const multi_car::ContorlRef &msg);
        void GetThetaBias();
        void Loop();
        ~DiffMPCROSNode(){}
    };
    
} // namespace DiffMPCNode
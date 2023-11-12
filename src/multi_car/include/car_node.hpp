#pragma once

#include <ros/ros.h>
#include "multi_car/ContorlRef.h"
#include "car_controller.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <dynamic_reconfigure/server.h>
#include <multi_car/car_param_Config.h>
namespace CarNode
{
    using namespace Eigen;
    class CarROSNode
    {
    private:
        ros::NodeHandle nh;
        ros::Subscriber real_pose_sub, real_vel_sub, control_ref_sub;
        ros::Publisher cmd_vel_pub;
        double ts;
        ros::Rate rate;
        CarController::VelController vel_controller;
        CarController::ThetaController theta_controller;
        CarController::PositionController pos_controller;

        Vector2d ref_pos;
        Vector2d ref_vel;

        std::array<bool,3> init_flag;
        double theta_bias;
        multi_car::car_param_Config config_for_debug;
        /* data */
    public:
        CarROSNode();
        void RealPosCallback(const geometry_msgs::PoseStamped &msg);
        void RealVelCallback(const geometry_msgs::TwistStamped &msg);
        void ControlRefCallback(const multi_car::ContorlRef &msg);
        void UpdateParamCallback(multi_car::car_param_Config &config, uint32_t level);
        void InitialParam();
        void GetThetaBias();
        void Loop();
    };
}
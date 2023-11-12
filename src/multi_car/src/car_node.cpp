#include "car_node.hpp"
#include "geometry_msgs/Twist.h"
#include <iostream>
namespace CarNode
{
    using namespace Eigen;
    using namespace std;

    CarROSNode::CarROSNode()
        : ts(0.01), rate(1 / ts), vel_controller(0, 1, 0.01, ts), theta_controller(1.0, 0.1, 0.15, ts), pos_controller(0.5, 0.1, 0.0, ts), init_flag({false,false,false}), theta_bias(0.0)
    {
        ROS_INFO_STREAM("CarROSNode Start Initial");
        real_pose_sub = nh.subscribe("real_pose", 1, &CarROSNode::RealPosCallback, this);
        real_vel_sub = nh.subscribe("real_vel", 1, &CarROSNode::RealVelCallback, this);
        control_ref_sub = nh.subscribe("control_ref", 1, &CarROSNode::ControlRefCallback, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        InitialParam();

        dynamic_reconfigure::Server<multi_car::car_param_Config> server;
        dynamic_reconfigure::Server<multi_car::car_param_Config>::CallbackType f;
        f = boost::bind(&CarROSNode::UpdateParamCallback, this, _1, _2);
        server.setCallback(f);

        while (ros::ok())
        {
            if (init_flag[0] && init_flag[1])
                break;
            ros::spinOnce();
        }

        GetThetaBias();
        ROS_INFO_STREAM("CarROSNode Initialed");

        while (ros::ok())
        {
            Loop();
            ros::spinOnce();
            rate.sleep();
        }
    }

    void CarROSNode::InitialParam()
    {
        double kp,ki,kd;
        nh.param("vel_kp", kp, 0.0);
        nh.param("vel_ki", ki, 1.0);
        nh.param("vel_kd", kd, 0.01);
        vel_controller.SetParam(kp, ki, kd);
        nh.param("theta_kp", kp, 1.0);
        nh.param("theta_ki", ki, 0.1);
        nh.param("theta_kd", kd, 0.15);
        theta_controller.SetParam(kp, ki, kd);
        nh.param("pos_kp", kp, 0.5);
        nh.param("pos_ki", ki, 0.1);
        nh.param("pos_kd", kd, 0.0);
        pos_controller.SetParam(kp, ki, kd);
    }

    void CarROSNode::RealPosCallback(const geometry_msgs::PoseStamped &msg)
    {
        init_flag[0] = true;
        CarController::real_pos(0) = msg.pose.position.x;
        CarController::real_pos(1) = msg.pose.position.y;
        Quaterniond quaternion(msg.pose.orientation.x, msg.pose.orientation.y,
                               msg.pose.orientation.z, msg.pose.orientation.w);
        Vector3d euler = quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
        if (fabs(euler[0] - M_PI) > 0.2)
            euler[2] += M_PI;
        while (euler[2] > M_PI)
            euler[2] -= 2 * M_PI;
        while (euler[2] < -M_PI)
            euler[2] += 2 * M_PI;
        CarController::real_theta = euler[2] + theta_bias;
    }

    void CarROSNode::RealVelCallback(const geometry_msgs::TwistStamped &msg)
    {
        init_flag[1] = true;
        CarController::real_vel(0) = msg.twist.linear.x;
        CarController::real_vel(1) = msg.twist.linear.y;
    }

    void CarROSNode::ControlRefCallback(const multi_car::ContorlRef &msg)
    {
        init_flag[2] = true;
        ref_pos[0] = msg.pos_x;
        ref_pos[1] = msg.pos_y;
        ref_vel[0] = msg.vel_x;
        ref_vel[1] = msg.vel_y;
        // ref_pos(msg.pos_x, msg.pos_y);
        // ref_vel(msg.vel_x, msg.vel_y);
    }

    void CarROSNode::UpdateParamCallback(multi_car::car_param_Config &config, uint32_t level)
    {
        ROS_INFO_STREAM("GetReconfigureRequest");
        config_for_debug = config;

        vel_controller.SetParam(config.vel_kp, config.vel_ki, config.vel_kd);
        theta_controller.SetParam(config.theta_kp, config.theta_ki, config.theta_kd);
        // pos_controller.SetParam(config.pos_kp, config.pos_ki, config.pos_kd);
    }

    void CarROSNode::GetThetaBias()
    {
        ROS_INFO_STREAM("Start Get Theta Bias");
        Vector2d init_pos = CarController::real_pos;
        geometry_msgs::Twist cmd_vel_msg;
        double init_theta = CarController::real_theta;
        cmd_vel_msg.linear.x = 0.2;
        cmd_vel_msg.linear.y = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        for(int i = 0;i<100;i++)
        {
            cmd_vel_pub.publish(cmd_vel_msg);
            ros::spinOnce();
            rate.sleep();
        }
        Vector2d delta = CarController::real_pos - init_pos;
        theta_bias = atan2(delta[1], delta[0]) - init_theta;
        ROS_INFO_STREAM("Get Theta Bias:" + std::to_string(theta_bias));
    }

    void CarROSNode::Loop()
    {
        Vector2d vel = pos_controller.ControlLoop(ref_pos, ref_vel);

        Vector2d debug_vel(config_for_debug.vel_x, config_for_debug.vel_y);
        Vector2d vel_output = vel_controller.ControlLoop(debug_vel);
        double theta_output = theta_controller.ControlLoop(config_for_debug.theta);
        theta_output = 0;

        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = vel_output[1] * sin(CarController::real_theta) + vel_output[0] * cos(CarController::real_theta);
        cmd_vel_msg.linear.y = vel_output[1] * cos(CarController::real_theta) - vel_output[0] * sin(CarController::real_theta);
        cmd_vel_msg.angular.z = theta_output;
        cmd_vel_pub.publish(cmd_vel_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_node");
    CarNode::CarROSNode car_node;
    return 0;
}
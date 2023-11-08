#include "car_node.hpp"
#include "geometry_msgs/Twist.h"
#include <iostream>
namespace CarNode
{
    geometry_msgs::Twist debug_msg;
    void debug_callback(const geometry_msgs::Twist &msg)
    {
        debug_msg = msg;
    }
    ros::Subscriber deug_sub;
    using namespace Eigen;
    using namespace std;

    CarROSNode::CarROSNode()
        : ts(0.01), rate(1 / ts), vel_controller(0, 1, 0.01, ts), theta_controller(ts), pos_controller(ts), init_flag(0)
    {
        deug_sub = nh.subscribe("debug", 1, &debug_callback);
        real_pose_sub = nh.subscribe("/vrpn_client_node/car0/pose", 1, &CarROSNode::RealPosCallback, this);
        real_vel_sub = nh.subscribe("/vrpn_client_node/car0/twist", 1, &CarROSNode::RealVelCallback, this);
        control_ref_sub = nh.subscribe("control_ref", 1, &CarROSNode::ControlRefCallback, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        std::cout << "你好" << std::endl;
        while (init_flag < 3)
        {
            ros::spinOnce();
        }

        while (ros::ok())
        {
            std::cout << "你好" << std::endl;
            Loop();
            ros::spinOnce();
            rate.sleep();
        }
    }

    void CarROSNode::RealPosCallback(const geometry_msgs::PoseStamped &msg)
    {
        if (init_flag < 3)
            init_flag++;
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
        CarController::real_theta = euler[2];
    }

    void CarROSNode::RealVelCallback(const geometry_msgs::TwistStamped &msg)
    {
        if (init_flag < 3)
            init_flag++;
        CarController::real_vel(0) = msg.twist.linear.x;
        CarController::real_vel(1) = msg.twist.linear.y;
    }

    void CarROSNode::ControlRefCallback(const multi_car::ContorlRef &msg)
    {
        if (init_flag < 3)
            init_flag++;
        ref_pos(msg.pos_x, msg.pos_y);
        ref_vel(msg.vel_x, msg.vel_y);
    }

    void CarROSNode::Loop()
    {
        Vector2d vel = pos_controller.ControlLoop(ref_pos, ref_vel);

        vel[0] = debug_msg.linear.x;
        vel[1] = debug_msg.linear.y;
        
        std::cout << ros::Time::now().toSec() << std::endl;
        // std::cout << vel << std::endl;

        Vector2d vel_output = vel_controller.ControlLoop(vel);
        double theta_output = theta_controller.ControlLoop(ref_vel);
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
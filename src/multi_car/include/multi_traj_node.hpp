#pragma once
#include "multi_traj.hpp"
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "serial_send.hpp"
#include <nav_msgs/Path.h>

namespace MultiTrajNode
{
    using namespace MultiTraj;
    using namespace Eigen;
    class MultiTrajROSNode
    {
    private:
        ros::NodeHandle nh;
        ros::Publisher ref_pub;
        double ts;
        ros::Rate rate;
        int traj_num;
        MatrixX2d init_pos;
        MatrixX2d init_vel;
        Vector2d circle_origin;
        double circle_radius;
        double start_time;
        std::shared_ptr<MultiTrajectory> multi_traj;
        std::vector<ros::Subscriber> real_pose_sub_list;
        std::vector<ros::Subscriber> real_vel_sub_list;
        std::vector<Vector2d> real_pos_list;
        std::vector<Vector2d> real_vel_list;
        std::vector<double> real_theta_list;
        std::vector<int> init_flag;
        SerialSend::SerialRosSend serial_send;

        std::vector<ros::Publisher> real_path_pub_list;
        std::vector<nav_msgs::Path> real_path_msg_list;
        std::vector<ros::Publisher> desire_path_pub_list;
        std::vector<nav_msgs::Path> desire_path_msg_list;
        /* data */
    public:
        MultiTrajROSNode();
        void RealPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int index);
        void RealVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg, int index);
        void Loop();
    };
}
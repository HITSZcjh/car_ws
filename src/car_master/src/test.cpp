#include "multi_traj.hpp"
#include <iostream>
#include <random>
#include <ros/ros.h>
#include <nav_msgs/Path.h>

using namespace std;
using namespace Eigen;
using namespace MultiTraj;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");

    ros::NodeHandle nh;
    std::vector<ros::Publisher> desire_path_pub_list;
    std::vector<nav_msgs::Path> desire_path_msg_list;
    for(int i = 0;i<3; i++)
    {
        ros::Publisher desire_path_pub = nh.advertise<nav_msgs::Path>("desire_path"+std::to_string(i),1);
        nav_msgs::Path desire_path_msg;
        desire_path_msg.header.frame_id = "world";
        desire_path_pub_list.push_back(desire_path_pub);
        desire_path_msg_list.push_back(desire_path_msg);
    }

    ros::Rate rate(10);

    std::default_random_engine generator;
    std::uniform_real_distribution<double> pos_distribution(-5.0, 5.0);
    std::uniform_real_distribution<double> vel_distribution(-0.1, 0.1);

    MatrixXd init_pos(3, 2);
    init_pos << pos_distribution(generator), pos_distribution(generator),
        pos_distribution(generator), pos_distribution(generator),
        pos_distribution(generator), pos_distribution(generator);

    MatrixXd init_vel(3, 2);
    init_vel << vel_distribution(generator), vel_distribution(generator),
        vel_distribution(generator), vel_distribution(generator),
        vel_distribution(generator), vel_distribution(generator);
    
    MultiTrajectory multi_traj(3, MatrixXd::Zero(3, 2), MatrixXd::Zero(3, 2), Vector2d(1, 0), 1.0);
    multi_traj.Planning();
    double start_time = ros::Time::now().toSec();
    while (ros::ok())
    {
        double t = ros::Time::now().toSec() - start_time;
        MatrixXd pos_vel = multi_traj.GetPosAndVel(t);
        for(int i = 0; i<3 ;i++)
        {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.pose.position.x = pos_vel(i, 0);
            pose_stamped.pose.position.y = pos_vel(i, 1);
            desire_path_msg_list[i].header.stamp = ros::Time::now();
            desire_path_msg_list[i].poses.push_back(pose_stamped);
            desire_path_pub_list[i].publish(desire_path_msg_list[i]);
        }
        rate.sleep();
    }
    


    return 0;
}

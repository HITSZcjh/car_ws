#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "diff_mpc_node");
    ros::NodeHandle nh;
    int car_num;
    nh.param("car_num", car_num, 3);

    std::vector<ros::Publisher> real_pose_pub_list;
    std::vector<ros::Publisher> real_vel_pub_list;
    std::vector<ros::Subscriber> real_odom_sub_list;
    real_pose_pub_list.resize(car_num);
    real_vel_pub_list.resize(car_num);
    real_odom_sub_list.resize(car_num);
    for (int i = 0; i < car_num; i++)
    {
        real_pose_pub_list[i] = nh.advertise<geometry_msgs::PoseStamped>("/car" + std::to_string(i) + "/real_pose", 1);
        real_vel_pub_list[i] = nh.advertise<geometry_msgs::TwistStamped>("/car" + std::to_string(i) + "/real_vel", 1);
        real_odom_sub_list[i] = nh.subscribe<nav_msgs::Odometry>("/car" + std::to_string(i) + "/odom", 1, [&, i](const nav_msgs::Odometry::ConstPtr &msg) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.pose.position.x = msg->pose.pose.position.x;
            pose_stamped.pose.position.y = msg->pose.pose.position.y;
            pose_stamped.pose.orientation = msg->pose.pose.orientation;
            real_pose_pub_list[i].publish(pose_stamped);

            geometry_msgs::TwistStamped twist_stamped;
            twist_stamped.header.stamp = ros::Time::now();
            twist_stamped.twist.linear.x = msg->twist.twist.linear.x;
            twist_stamped.twist.linear.y = msg->twist.twist.linear.y;
            twist_stamped.twist.angular.z = msg->twist.twist.angular.z;
            real_vel_pub_list[i].publish(twist_stamped);
        });
    }
    ros::spin();
}
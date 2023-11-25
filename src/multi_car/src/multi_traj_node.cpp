#include "multi_traj_node.hpp"
#include "multi_car/ContorlRef.h"
#include <algorithm>

namespace MultiTrajNode
{
    MultiTrajROSNode::MultiTrajROSNode()
        : ts(0.01), rate(1 / ts), circle_origin(0, 2), circle_radius(1)
    {
        nh.param("car_num", traj_num, 3);

        ROS_INFO_STREAM("MultiTrajROSNode Start Initial");
        real_pos_list.resize(traj_num);
        real_vel_list.resize(traj_num);
        real_theta_list.resize(traj_num);
        real_pose_sub_list.resize(traj_num);
        real_vel_sub_list.resize(traj_num);
        control_ref_pub_list.resize(traj_num);
        init_flag.resize(traj_num);
        real_path_pub_list.resize(traj_num);
        real_path_msg_list.resize(traj_num);
        desire_path_pub_list.resize(traj_num);
        desire_path_msg_list.resize(traj_num);
        for (int i = 0; i < traj_num; i++)
        {
            real_pose_sub_list[i] = nh.subscribe<geometry_msgs::PoseStamped>("/car" + std::to_string(i) + "/real_pose", 1, boost::bind(&MultiTrajROSNode::RealPosCallback, this, _1, i));
            real_vel_sub_list[i] = nh.subscribe<geometry_msgs::TwistStamped>("/car" + std::to_string(i) + "/real_vel", 1, boost::bind(&MultiTrajROSNode::RealVelCallback, this, _1, i));

            init_flag[0][0] = false;
            init_flag[0][1] = false;

            control_ref_pub_list[i] = nh.advertise<multi_car::ContorlRef>("control_ref" + std::to_string(i), 1);
            real_path_msg_list[i].header.frame_id = "world";
            real_path_pub_list[i] = nh.advertise<nav_msgs::Path>("real_path" + std::to_string(i), 1);
            desire_path_msg_list[i].header.frame_id = "world";
            desire_path_pub_list[i] = nh.advertise<nav_msgs::Path>("desire_path" + std::to_string(i), 1);
        }

        while (ros::ok())
        {
            int cnt = 0;
            for (int i = 0; i < traj_num; i++)
            {
                if (init_flag[i][0] && init_flag[i][1])
                    cnt++;
            }
            if (cnt == traj_num)
                break;
            else
                ros::spinOnce();
        }
        ROS_INFO_STREAM("MultiTrajROSNode Initialed");

        init_pos.resize(traj_num, 2);
        init_vel.resize(traj_num, 2);
        for (int i = 0; i < traj_num; i++)
        {
            init_pos.row(i) = real_pos_list[i];
            init_vel.row(i) = real_vel_list[i];
        }
        ROS_INFO_STREAM("MultiTrajROSNode Start Planning");
        multi_traj = std::make_shared<MultiTrajectory>(traj_num, init_pos, init_vel, circle_origin, circle_radius);
        multi_traj->Planning();
        ROS_INFO_STREAM("MultiTrajROSNode End Planning");

        start_time = ros::Time::now().toSec();
        while (ros::ok())
        {
            Loop(40+1, 0.4);
            ros::spinOnce();
            rate.sleep();
        }
    }

    void MultiTrajROSNode::RealPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int index)
    {
        init_flag[index][0] = true;
        real_pos_list[index](0) = msg->pose.position.x;
        real_pos_list[index](1) = msg->pose.position.y;
        Quaterniond quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                               msg->pose.orientation.z, msg->pose.orientation.w);
        Vector3d euler = quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
        if (fabs(euler[0] - M_PI) > 0.2)
            euler[2] += M_PI;
        while (euler[2] > M_PI)
            euler[2] -= 2 * M_PI;
        while (euler[2] < -M_PI)
            euler[2] += 2 * M_PI;
        real_theta_list[index] = euler[2];
    }

    void MultiTrajROSNode::RealVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg, int index)
    {
        init_flag[index][1] = true;
        real_vel_list[index](0) = msg->twist.linear.x;
        real_vel_list[index](1) = msg->twist.linear.y;
    }

    void MultiTrajROSNode::Loop(int ref_num, double ts)
    {
        double t = ros::Time::now().toSec() - start_time;

        std::vector<MatrixXd> ref_pos_vel_list;
        ref_pos_vel_list.resize(ref_num);
        for (int i = 0; i < ref_num; i++)
        {
            ref_pos_vel_list[i] = multi_traj->GetPosAndVel(t + i * ts);
        }
        for (int i = 0; i < traj_num; i++)
        {
            multi_car::ContorlRef ref_msg;
            for (int j = 0; j < ref_num; j++)
            {
                ref_msg.pos_x.push_back(ref_pos_vel_list[j](i, 0));
                ref_msg.pos_y.push_back(ref_pos_vel_list[j](i, 1));
                ref_msg.vel_x.push_back(ref_pos_vel_list[j](i, 2));
                ref_msg.vel_y.push_back(ref_pos_vel_list[j](i, 3));
            }
            control_ref_pub_list[i].publish(ref_msg);
        }

        // serial_send.send(real_pos_list, real_vel_list, real_theta_list, ref_pos_vel);

        MatrixXd ref_pos_vel_now = ref_pos_vel_list[0];
        for (int i = 0; i < traj_num; i++)
        {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.pose.position.x = real_pos_list[i](0);
            pose_stamped.pose.position.y = real_pos_list[i](1);
            real_path_msg_list[i].header.stamp = ros::Time::now();
            real_path_msg_list[i].poses.push_back(pose_stamped);
            real_path_pub_list[i].publish(real_path_msg_list[i]);

            pose_stamped.pose.position.x = ref_pos_vel_now(i, 0);
            pose_stamped.pose.position.y = ref_pos_vel_now(i, 1);
            desire_path_msg_list[i].header.stamp = ros::Time::now();
            desire_path_msg_list[i].poses.push_back(pose_stamped);
            desire_path_pub_list[i].publish(desire_path_msg_list[i]);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_traj_node");
    MultiTrajNode::MultiTrajROSNode multi_traj_ros_node;
    return 0;
}
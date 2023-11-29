#include "diff_mpc_node.hpp"

namespace DiffMPCNode
{
    DiffMPCROSNode::DiffMPCROSNode()
        : ts(0.02), rate(1 / ts), init_flag({false, false, false}), theta_bias(0.0),
          vel_pid(1.0, 2.0, 0.2, 0.4, -0.4, ts), vel_lpf(10, ts)
    {
        ROS_INFO_STREAM("DiffMPCROSNode Start Initial");
        real_pose_sub = nh.subscribe("real_pose", 1, &DiffMPCROSNode::RealPosCallback, this);
        real_vel_sub = nh.subscribe("real_vel", 1, &DiffMPCROSNode::RealVelCallback, this);
        control_ref_sub = nh.subscribe("control_ref", 1, &DiffMPCROSNode::ControlRefCallback, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        double vel_max, vel_min, omega_max, omega_min, d_v_max, d_v_min, d_omega_max, d_omega_min;
        nh.getParam("vel_max", vel_max);
        nh.getParam("vel_min", vel_min);
        nh.getParam("omega_max", omega_max);
        nh.getParam("omega_min", omega_min);
        nh.getParam("d_v_max", d_v_max);
        nh.getParam("d_v_min", d_v_min);
        nh.getParam("d_omega_max", d_omega_max);
        nh.getParam("d_omega_min", d_omega_min);
        diff_mpc_controller = std::make_unique<DiffMPC::DiffMPCController>(vel_max, vel_min, omega_max, omega_min,
                                                                           d_v_max, d_v_min, d_omega_max, d_omega_min);

        while (ros::ok())
        {
            if (init_flag[0] && init_flag[1])
                break;
            ros::spinOnce();
        }
        GetThetaBias();
        nh.setParam("theta_bias", theta_bias);
        while (ros::ok())
        {
            if (init_flag[2])
                break;
            ros::spinOnce();
        }
        ROS_INFO_STREAM("DiffMPCROSNode Initialed");

        while (ros::ok())
        {
            Loop();
            ros::spinOnce();
            rate.sleep();
        }
    }

    void DiffMPCROSNode::RealPosCallback(const geometry_msgs::PoseStamped &msg)
    {
        init_flag[0] = true;
        real_pos(0) = msg.pose.position.x;
        real_pos(1) = msg.pose.position.y;
        Quaterniond quaternion(msg.pose.orientation.x, msg.pose.orientation.y,
                               msg.pose.orientation.z, msg.pose.orientation.w);
        Vector3d euler = quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
        if (fabs(euler[0] - M_PI) > 0.2)
            euler[2] += M_PI;
        while (euler[2] > M_PI)
            euler[2] -= 2 * M_PI;
        while (euler[2] < -M_PI)
            euler[2] += 2 * M_PI;
        real_theta = euler[2] + theta_bias;
    }

    void DiffMPCROSNode::RealVelCallback(const geometry_msgs::TwistStamped &msg)
    {
        init_flag[1] = true;
        if (cos(real_theta) * msg.twist.linear.x + sin(real_theta) * msg.twist.linear.y > 0)
        {
            real_vel = Vector2d(msg.twist.linear.x, msg.twist.linear.y).norm();
        }
        else
            real_vel = -Vector2d(msg.twist.linear.x, msg.twist.linear.y).norm();
        real_omega = msg.twist.angular.z * 100;
    }

    void DiffMPCROSNode::ControlRefCallback(const multi_car::ContorlRef &msg)
    {
        init_flag[2] = true;
        control_ref_msg = msg;
    }

    void DiffMPCROSNode::GetThetaBias()
    {
        ROS_INFO_STREAM("Start Get Theta Bias");
        Vector2d init_pos = real_pos;
        geometry_msgs::Twist cmd_vel_msg;
        double init_theta = real_theta;
        cmd_vel_msg.linear.x = 0.2;
        cmd_vel_msg.linear.y = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        for (int i = 0; i < 100; i++)
        {
            cmd_vel_pub.publish(cmd_vel_msg);
            ros::spinOnce();
            rate.sleep();
        }
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_pub.publish(cmd_vel_msg);
        ros::spinOnce();
        rate.sleep();

        Vector2d delta = real_pos - init_pos;
        theta_bias = atan2(delta[1], delta[0]) - init_theta;
        ROS_INFO_STREAM("Get Theta Bias:" + std::to_string(theta_bias));
    }

    void DiffMPCROSNode::Loop()
    {
        double x0[DiffMPC::NX];
        x0[0] = real_pos(0);
        x0[1] = real_pos(1);
        x0[2] = real_theta;
        x0[3] = real_vel;
        x0[4] = real_omega;
        // std::cout << "x0:";
        // for(int i = 0;i < DiffMPC::NX;i++)
        // {
        //     std::cout<<x0[i] << " ";
        // }
        // std::cout<<std::endl;

        double yref[DiffMPC::N][DiffMPC::NX + DiffMPC::NU] = {0};
        int sample_num = control_ref_msg.pos_x.size();
        for (int i = 0; i < DiffMPC::N; i++)
        {
            if (i < sample_num)
            {
                yref[i][0] = control_ref_msg.pos_x[i];
                yref[i][1] = control_ref_msg.pos_y[i];
            }
            else
            {
                yref[i][0] = control_ref_msg.pos_x[sample_num - 1];
                yref[i][1] = control_ref_msg.pos_y[sample_num - 1];
            }
            // yref[i][2] = 0.0;
            // yref[i][3] = Vector2d(control_ref_msg.vel_x[i], control_ref_msg.vel_y[i]).norm();
            // yref[i][4] = 0.0;
            // yref[i][5] = 0.0;
            // yref[i][6] = 0.0;
        }
        // std::cout<<"yref:"<<yref[0][0]<<" "<<yref[0][1]<<std::endl;
        double yref_e[DiffMPC::NX] = {0};
        if (DiffMPC::N < sample_num)
        {
            yref_e[0] = control_ref_msg.pos_x[DiffMPC::N];
            yref_e[1] = control_ref_msg.pos_y[DiffMPC::N];
        }
        else
        {
            yref_e[0] = control_ref_msg.pos_x[sample_num - 1];
            yref_e[1] = control_ref_msg.pos_y[sample_num - 1];
        }

        // yref_e[2] = 0.0;
        // yref_e[3] = Vector2d(control_ref_msg.vel_x[DiffMPC::N], control_ref_msg.vel_y[DiffMPC::N]).norm();
        // yref_e[4] = 0.0;
        // yref_e[5] = 0.0;
        // yref_e[6] = 0.0;

        Vector2d u = diff_mpc_controller->Solve(x0, yref, yref_e);

        geometry_msgs::Twist cmd_vel_msg;
        vel_pid.fdb = vel_lpf.LPF_Calculate(real_vel);
        vel_pid.ref = u[0] * 0.2 + real_vel;
        vel_pid.PID_Calculate();

        cmd_vel_msg.linear.x = vel_pid.output;
        cmd_vel_msg.linear.y = 0;
        cmd_vel_msg.angular.z = u[1] * 0.2 + real_omega;
        cmd_vel_pub.publish(cmd_vel_msg);
        // std::cout << "cmd_vel: " << cmd_vel_msg.linear.x << " cmd_omega: " << cmd_vel_msg.angular.z << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "diff_mpc_node");
    DiffMPCNode::DiffMPCROSNode diff_mpc_node;
    return 0;
}
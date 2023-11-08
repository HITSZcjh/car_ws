#include "serial_send.hpp"
namespace SerialSend
{
    SerialRosSend::SerialRosSend()
    {
        try
        {
            // 设置串口属性，并打开串口
            ser.setPort("/dev/ttyUSB0");
            ser.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to open port ");
        }
        if (ser.isOpen())
        {
            ROS_INFO_STREAM("Serial Port initialized");
        }
        else
            ROS_ERROR_STREAM("Unable to open port ");
    }
    void SerialRosSend::send(std::vector<Eigen::Vector2d> &real_pos_list, std::vector<Eigen::Vector2d> &real_vel_list, std::vector<double> &real_theta_list, Eigen::MatrixXd &ref_pos_vel)
    {
        for (int i = 0; i < 2; i++)
        {
            msg.data[0 + i * 9] = real_pos_list[i + 1][0];
            msg.data[1 + i * 9] = real_pos_list[i + 1][1];
            msg.data[2 + i * 9] = real_theta_list[i + 1];
            msg.data[3 + i * 9] = real_vel_list[i + 1][0];
            msg.data[4 + i * 9] = real_vel_list[i + 1][1];
            msg.data[5 + i * 9] = ref_pos_vel(i + 1, 0);
            msg.data[6 + i * 9] = ref_pos_vel(i + 1, 1);
            msg.data[7 + i * 9] = ref_pos_vel(i + 1, 2);
            msg.data[8 + i * 9] = ref_pos_vel(i + 1, 3);
        }
        ser.write((uint8_t *)&msg, SMSG_BAG_LENGTH);
    }
}
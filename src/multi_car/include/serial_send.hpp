#pragma once
#include <ros/ros.h>
#include <serial/serial.h>
#include <Eigen/Eigen>

namespace SerialSend
{

    constexpr uint8_t HEADER = 0X55;
    constexpr uint8_t TAIL = 0X3F;

    constexpr int SMSG_HEAD_LENGTH = 1;
    constexpr int SMSG_TAIL_LENGTH = 1;
    constexpr int SMSG_DATA_LENGTH = 9 * 2;
    constexpr int SMSG_BAG_LENGTH = SMSG_HEAD_LENGTH + SMSG_DATA_LENGTH * 4 + SMSG_TAIL_LENGTH;

    typedef struct
    {
        uint8_t head;
        float data[SMSG_DATA_LENGTH];
        uint8_t tail;
    } __attribute__((packed)) Smsg_t;

    class SerialRosSend
    {
    private:
        Smsg_t msg = {HEADER, {0}, TAIL};
        serial::Serial ser;

    public:
        SerialRosSend();
        void send(std::vector<Eigen::Vector2d> &real_pos_list, std::vector<Eigen::Vector2d> &real_vel_list, std::vector<double> &real_theta_list, Eigen::MatrixXd &ref_pos_vel);
    };

}
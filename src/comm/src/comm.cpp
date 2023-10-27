#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <string>
#include <cstdlib>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define HEADER 0X55
#define TAIL 0X3F

#define SMSG_HEAD_LENGTH 1
#define SMSG_TAIL_LENGTH 1
#define SMSG_DATA_LENGTH 6
#define SMSG_BAG_LENGTH (SMSG_HEAD_LENGTH + SMSG_DATA_LENGTH*4 + SMSG_TAIL_LENGTH)

typedef struct
{
    uint8_t head;
    float data[SMSG_DATA_LENGTH];
    uint8_t tail;
} __attribute__((packed)) Smsg_t;

Smsg_t msg = {HEADER, {0}, TAIL};
serial::Serial ser; //声明串口对象 

void transformCallback(const geometry_msgs::TransformStamped::ConstPtr& transform_msg)
{
    Eigen::Quaterniond quaternion(transform_msg->transform.rotation.x, transform_msg->transform.rotation.y, 
    transform_msg->transform.rotation.z, transform_msg->transform.rotation.w);
    Eigen::Vector3d euler = quaternion.toRotationMatrix().eulerAngles(2,1,0);

    msg.data[0] = transform_msg->transform.translation.x;
    msg.data[1] = transform_msg->transform.translation.y;
    msg.data[2] = transform_msg->transform.translation.z;
    msg.data[3] = euler[0];
    msg.data[4] = euler[1];
    msg.data[5] = euler[2];
    ser.write((uint8_t*)&msg, SMSG_BAG_LENGTH);
}

int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "serial"); 
    ros::NodeHandle nh; 
    ros::Subscriber transform_sub = nh.subscribe("/vicon/test/test", 1000, &transformCallback);

    try 
    { 
        //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB0"); 
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    }
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 
    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    }
 
    ros::spin();
    return 0;
}
 
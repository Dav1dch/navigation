#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include "yhs_can_msgs/ctrl_cmd.h"
#include <string>
#include <cmath> 
#define PI 3.1415926 
// max car speed
const float max_speed_radio = 10.0;
const float max_rot_radio = 10.0;



// car speed
float speed_radio = 0.1;
float rot_radio = 0.1;
float max_speed = 0.3;
float max_rot_speed = 30;

struct Car {
    unsigned int gear;
    float linear;
    float angular;
    float z;
};

struct Car car = {3,0.0,0.0};

double odomTime = 0;

void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
{
  odomTime = odomIn->header.stamp.toSec();
}

void cmd_callback(const geometry_msgs::Twist &msgs)
{
    // limit speed
    float x = msgs.linear.x;

    float linear = x * speed_radio;
    float angular = msgs.angular.z / PI * 180.0 * rot_radio;
    
    if (abs(linear) > max_speed){
        linear = linear>0? max_speed:-max_speed;
    }

    if ( abs(angular) > max_rot_speed){
        angular = angular>0? max_rot_speed:-max_rot_speed;
    }

    car.gear = 3;
    car.linear = linear;
    // 角度
    car.angular = angular;
    // 弧度
    car.z = msgs.angular.z * rot_radio;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_control_node");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");
    std::string sub_topic = "cmd_vel";
    std::string pub_topic = "ctrl_cmd";

    private_nh.getParam("sub_topic", sub_topic);
    private_nh.getParam("pub_topic", pub_topic);
    private_nh.getParam("speed_radio", speed_radio);
    private_nh.getParam("max_speed", max_speed);
    private_nh.getParam("max_rot_speed", max_rot_speed);
    private_nh.getParam("rot_radio", rot_radio);

    if ( speed_radio > max_speed_radio)
    {
        speed_radio = max_speed_radio;
    }

    if (speed_radio <=-max_speed_radio)
    {
        speed_radio = -max_speed_radio;
    }

    if (rot_radio > max_rot_radio)
    {
        rot_radio = max_rot_radio;
    }

    if (rot_radio <= -max_rot_radio)
    {
        rot_radio = -max_rot_radio;
    }

    ROS_INFO("now car speed_radio is: %.2f  ", speed_radio);
    ROS_INFO("now car rot_radio is: %.2f  ", rot_radio);
    ROS_INFO("now car max_speed is: %.2f  ", max_speed);
    ROS_INFO("now car max_rot_speed is: %.2f  ", max_rot_speed);
    ROS_INFO("now sub_topic is: %s ", sub_topic.c_str());
    ROS_INFO("now pub_topic is: %s ", pub_topic.c_str());


    ros::Publisher cmd_pub = n.advertise<yhs_can_msgs::ctrl_cmd>(pub_topic, 5);
    ros::Subscriber cmd_sub = n.subscribe(sub_topic, 5, cmd_callback);

    ros::Subscriber subOdom = n.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, odomHandler);
    ros::Publisher cmd_pub_stamp = n.advertise<geometry_msgs::TwistStamped>(pub_topic+"_stamp", 5);

    geometry_msgs::TwistStamped msg_stamp;
    msg_stamp.header.frame_id = "/vehicle";

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        yhs_can_msgs::ctrl_cmd msg;
        msg.ctrl_cmd_gear = car.gear;
        msg.ctrl_cmd_linear = car.linear;
        msg.ctrl_cmd_angular = car.angular;
        cmd_pub.publish(msg);

        msg_stamp.header.stamp = ros::Time().fromSec(odomTime);
        msg_stamp.twist.linear.x = car.linear;
        msg_stamp.twist.angular.z = car.z;
        cmd_pub_stamp.publish(msg_stamp);

        ros::spinOnce();

        loop_rate.sleep();

    }

    // stop car
    // yhs_can_msgs::ctrl_cmd msg;
    // msg.ctrl_cmd_gear = 3;
    // msg.ctrl_cmd_linear = 0;
    // msg.ctrl_cmd_angular = 0;
    // cmd_pub.publish(msg);
    // ROS_INFO(" car stop! ");

    return 0;
}

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Twist.h"
#include "yhs_can_msgs/l_wheel_fb.h"
#include "yhs_can_msgs/ctrl_fb.h"
#include "yhs_can_msgs/r_wheel_fb.h"
#include <string>
#include <boost/array.hpp>
#include <cmath>


float r_wheel_fb_velocity = 0.0;
float l_wheel_fb_velocity = 0.0;
float linear_velocity = 0.0;
float angular_velocity = 0.0;
//声明赋值：
const boost::array<double, 36> ODOM_POSE_COVARIANCE = {{1e-3, 0, 0, 0, 0, 0,
                                                        0, 1e-3, 0, 0, 0, 0,
                                                        0, 0, 1e6, 0, 0, 0,
                                                        0, 0, 0, 1e6, 0, 0,
                                                        0, 0, 0, 0, 1e6, 0,
                                                        0, 0, 0, 0, 0, 1e3}};
const boost::array<double, 36> ODOM_POSE_COVARIANCE2 = {{1e-9, 0, 0, 0, 0, 0,
                                                         0, 1e-3, 1e-9, 0, 0, 0,
                                                         0, 0, 1e6, 0, 0, 0,
                                                         0, 0, 0, 1e6, 0, 0,
                                                         0, 0, 0, 0, 1e6, 0,
                                                         0, 0, 0, 0, 0, 1e-9}};
const boost::array<double, 36> ODOM_TWIST_COVARIANCE = {{1e-3, 0, 0, 0, 0, 0,
                                                         0, 1e-3, 0, 0, 0, 0,
                                                         0, 0, 1e6, 0, 0, 0,
                                                         0, 0, 0, 1e6, 0, 0,
                                                         0, 0, 0, 0, 1e6, 0,
                                                         0, 0, 0, 0, 0, 1e3}};
const boost::array<double, 36> ODOM_TWIST_COVARIANCE2 = {{1e-9, 0, 0, 0, 0, 0,
                                                          0, 1e-3, 1e-9, 0, 0, 0,
                                                          0, 0, 1e6, 0, 0, 0,
                                                          0, 0, 0, 1e6, 0, 0,
                                                          0, 0, 0, 0, 1e6, 0,
                                                          0, 0, 0, 0, 0, 1e-9}};

void left_callback(const yhs_can_msgs::l_wheel_fb &msg){
    l_wheel_fb_velocity = msg.l_wheel_fb_velocity;
}

void right_callback(const yhs_can_msgs::r_wheel_fb &msg){
    r_wheel_fb_velocity = msg.r_wheel_fb_velocity;
}

void ctrl_fb_callback(const yhs_can_msgs::ctrl_fb &msg){
    linear_velocity = msg.ctrl_fb_linear;
    angular_velocity = msg.ctrl_fb_angular / 180.0 * 3.14159265;
}

int main(int argc, char **argv)
{

    std::string l_wheel_sub_topic = "l_wheel_fb";
    std::string r_wheel_sub_topic = "r_wheel_fb";
    // std::string ctrl_fb_sub_topic = "ctrl_fb";

    // ROS_INFO("now car speed_radio is: %.2f  ", speed_radio);
    ros::init(argc, argv, "wheel_odom");
    ros::NodeHandle n;
    ros::Duration(1.0).sleep();    //等待话题发布完成
	tf::TransformBroadcaster odom_broadcaster;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("wheel_odom", 100);
    ros::Subscriber l_sub = n.subscribe(l_wheel_sub_topic, 100, left_callback);
    ros::Subscriber r_sub = n.subscribe(r_wheel_sub_topic, 100, right_callback);
    // ros::Subscriber ctrl_sub = n.subscribe(ctrl_fb_sub_topic, 100, ctrl_fb_callback);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        // yhs_can_msgs::ctrl_cmd msg;
        // msg.ctrl_cmd_gear = car.gear;
        // msg.ctrl_cmd_linear = car.linear;
        // msg.ctrl_cmd_angular = car.angular;

        // cmd_pub.publish(msg);

        ros::spinOnce();
        current_time = ros::Time::now();
        double ctime = current_time.toSec();
        double dt = (current_time - last_time).toSec();
        last_time = current_time;

        double vxy = (l_wheel_fb_velocity + r_wheel_fb_velocity) / 2.0;
        double vth = (r_wheel_fb_velocity - l_wheel_fb_velocity) / 0.495;
        double r = vxy / vth;
        // if (vxy != 0.0){
        //     ROS_INFO("vxy: %f .  ctrl_linear: %f", vxy, linear_velocity);
        //     ROS_INFO("vth: %f .  ctrl_angular: %f", vth, angular_velocity);
        // }
        // vxy = linear_velocity;
        // vth = angular_velocity;

        double dxy = vxy * dt;
        double dth = vth * dt;
        double delta_x = 0.0;
        double delta_y = 0.0;

        if (dth != 0){
            delta_x = (sin(dth) / dth) * dxy;
            delta_y = ((1 - cos(dth)) / dth) * dxy;
        }else{
            delta_x = dxy;
            delta_y = 0.0;
        }


        // double delta_x = dxy * cos(dth);
        // double delta_y = dxy * sin(dth);
        // ROS_INFO("dx:%f dy:%f", delta_x, delta_y);

        x += (cos(th) * delta_x - sin(th) * delta_y);
        y += (sin(th) * delta_x + cos(th) * delta_y);
            // x += delta_x;
            // y += delta_y;

        th += dth;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);    //转换成四元数
 
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "wheel_odom";
		odom_trans.child_frame_id = "base_link";
 
		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
 
		// odom_broadcaster.sendTransform(odom_trans);    //发布odom和base_footprint的tf数
 
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "wheel_odom";
 
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
        // if (dxy == 0 && dth == 0){
        //     odom.pose.covariance = ODOM_POSE_COVARIANCE2;
        // }else{
        //     odom.pose.covariance = ODOM_POSE_COVARIANCE;
        // }
 
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vxy;    //两轮差动小车只有线速度x
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.angular.z = vth;
        if (dxy == 0 && dth == 0){
            odom.twist.covariance = ODOM_POSE_COVARIANCE2;
        }else{
            odom.twist.covariance = ODOM_POSE_COVARIANCE;
        }
 
		odom_pub.publish(odom);    //发布odom数据




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

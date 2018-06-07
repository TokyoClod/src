/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>

#include <camera_calibration_parsers/parse.h>
#include "PID_position.h"
#include <tf/transform_broadcaster.h>
using namespace std;


mavros_msgs::State fcu_state;
std_msgs::Int8 track_state;
geometry_msgs::PoseArray object_pose;
geometry_msgs::TwistStamped vel_skew, camera_vel;
// geometry_msgs::PoseStamped camera_pose;
geometry_msgs::Pose uav_pose;
geometry_msgs::PoseStamped pose, lostPose;

std_msgs::Int8 theta,Rad;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    fcu_state = *msg;
}

void track_state_cb(const std_msgs::Int8::ConstPtr& msg_track_state){
    track_state = *msg_track_state;
}

void vel_skew_cb(const geometry_msgs::TwistStamped::ConstPtr& msg_vel_kew){
    vel_skew = *msg_vel_kew;
}

// void camera_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg_camera_pose){
//     camera_pose = *msg_camera_pose;
// }

void camera_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg_camera_vel){
    camera_vel = *msg_camera_vel;
}

void uav_pos_cb(const nav_msgs::Odometry::ConstPtr& msg_uav_odom){
    nav_msgs::Odometry uav_odom = *msg_uav_odom;
    uav_pose = uav_odom.pose.pose;
    // wMc = visp_bridge::toVispHomogeneousMatrix(uav_pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_apriltag_node");
    ros::NodeHandle nh;

    tf::TransformBroadcaster broadcaster;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);   
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //track with april_ros package
    ros::Subscriber track_state_sub = nh.subscribe<std_msgs::Int8>
            ("tag_tracker_status",10,track_state_cb);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel",10);
    ros::Subscriber vel_pub = nh.subscribe<geometry_msgs::TwistStamped>("tag_velocity_skew", 10, vel_skew_cb);

    //subscribe the pose and vel of the px4
    // ros::Subscriber camera_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //         ("mavros/local_position/pose", 10,camera_pose_cb);
    ros::Subscriber uav_pos_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth", 10, uav_pos_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/mavros/local_position/velocity", 10, camera_vel_cb); 

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);/*
    PID_position pid_1(1,0,0.);
    PID_position pid_2(1,0,0.);
    PID_position pid_3(1,0,0.);
    PID_position pid_4(1,0,0.);*/

    // wait for FCU connection
    while(ros::ok() && fcu_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    double yaw_init, z_init;
    //nh.param<double>("yaw_init", yaw_init, 0);
    //nh.param<double>("z_init", z_init, 4.0);

    ros::param::get("~yaw_init", yaw_init);
    ros::param::get("~z_init", z_init);
    
    //double yaw = 30;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = z_init;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_init*3.1416/180);

    // lostPose.pose.position.x = 0;
    // lostPose.pose.position.y = 0;
    // lostPose.pose.position.z = z_init;
    lostPose.pose=pose.pose;

    ROS_INFO("Program Start");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd, disarm_cmd;
    arm_cmd.request.value = true;
    disarm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    //std_msgs::String mode_now;  
    geometry_msgs::TwistStamped vel_to_pub;
    bool land_flag = false;
    while(ros::ok()){
        ros::Time begin =ros::Time::now();
        if( fcu_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            ROS_INFO("current_fcu_state.mode= '%s'",fcu_state.mode.c_str());   
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if(land_flag){
                if( arming_client.call(disarm_cmd) && disarm_cmd.response.success){
                        ROS_INFO("Vehicle disarmed");  
                    }
                last_request = ros::Time::now();  
            }
            else if( !fcu_state.armed && (ros::Time::now() - last_request > ros::Duration(2.0))){
                //ROS_INFO("arming_client:%d",arming_client.call(arm_cmd));
                //ROS_INFO("arm_cmd:%d",arm_cmd.response.success);
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");     
                }
                last_request = ros::Time::now();  
            }
            
            if(fcu_state.armed){
                cout<<"track_state:"<<track_state<<endl;

                if (track_state.data == 0) {
                    //ROS_INFO("track lost"); 
                    // cout <<"z"<<lostPose.pose.position.z<<endl;
                    local_pos_pub.publish(lostPose);
                    ros::spinOnce();   
                    // if(uav_pose.position.z < z_init-0.1) 
                    //     track_state.data = 0;    
                    
                }                       
                else if(track_state.data == 1) {
                    //ROS_INFO("tracking");
                    local_vel_pub.publish(vel_skew);
                    
                    lostPose.pose.position.x = uav_pose.position.x;
                    lostPose.pose.position.y = uav_pose.position.y;
                    lostPose.pose.position.z = z_init;
                    lostPose.pose.orientation.z = uav_pose.orientation.z;  
                    ros::spinOnce();                                             
                }
                else if(track_state.data == 2){
                    land_flag = true; 
                    // if( arming_client.call(disarm_cmd) && disarm_cmd.response.success){
                    //     ROS_INFO("Vehicle disarmed");  
                    //     land_flag = true;   
                    // }
                    last_request = ros::Time::now();  
                    // lostPose.pose.position.z = 0;
                    // local_pos_pub.publish(lostPose);
                    ros::spinOnce();
                }
                else{}
            }
            else{
                ros::spinOnce();
            }
   
        }
        //local_vel_pub.publish(vel_to_pub);
        //local_pos_pub.publish(pose);

        /*
        //define the tf from camera frame to world frame
        tf::Transform transform;
        geometry_msgs::Quaternion vel_quat = tf::createQuaternionMsgFromRollPitchYaw(camera_vel.twist.angular.x,
                                                                            camera_vel.twist.angular.y,
                                                                            camera_vel.twist.angular.z);
        geometry_msgs::TransformStamped vel_trans;
        vel_trans.header.stamp = ros::Time::now();
        vel_trans.header.frame_id = "local_origin";
        vel_trans.child_frame_id = "map";
    
        vel_trans.transform.translation =  camera_vel.twist.linear;
        vel_trans.transform.rotation = vel_quat;
        broadcaster.sendTransform(vel_trans);*/

        ros::Time end = ros::Time::now();
        // ROS_INFO("%f ms",1000*(end-begin).toSec());

        // ros::spinOnce();
        rate.sleep();
    }
    

    return 0;
}

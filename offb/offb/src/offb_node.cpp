/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */
#include <iostream>
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <camera_calibration_parsers/parse.h>

#include <tf/transform_broadcaster.h>

mavros_msgs::State fcu_state;
std_msgs::Int8 track_state;
geometry_msgs::PoseStamped object_pose;
geometry_msgs::PoseStamped camera_pose;
geometry_msgs::Point circle_center;
geometry_msgs::TwistStamped vel_skew;

std_msgs::Int8 theta,Rad;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    fcu_state = *msg;
}

void track_state_cb(const std_msgs::Int8::ConstPtr& msg_track_state){
    track_state = *msg_track_state;
}

void object_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg_object_pose){
    object_pose = *msg_object_pose;
}

void camera_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg_camera_pose){
    camera_pose = *msg_camera_pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel",10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");


    ros::Subscriber track_state_sub = nh.subscribe<std_msgs::Int8>
            ("visp_auto_tracker/status",1,track_state_cb);
    ros::Subscriber object_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("visp_auto_tracker/object_position", 10, object_pose_cb);
    ros::Subscriber camera_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10,camera_pose_cb);

    /* camera calibration information service
    
    ros::ServiceClient client = nh.serviceClient<sensor_msgs::SetCameraInfo>("/iris_down_cam/downward_cam/set_camera_info");
    sensor_msgs::CameraInfo camera_info;
    std::string filename = "/home/abner/cali.ini";
    std::string camera_name = "downward_cam";
    camera_calibration_parsers::readCalibration(filename, camera_name, camera_info);

    sensor_msgs::SetCameraInfo srv;
    srv.request.camera_info = camera_info;
    if (client.call(srv))
    {
      if(srv.response.success)
      {
        std::ostringstream ossInf;
        ossInf << srv.request.camera_info;
        //ROS_INFO("%s", ossInf.str().c_str());
        ROS_INFO("Calibration successfully applied.");
      }
      else
      {
        std::ostringstream ossStm;
        ossStm << srv.response.status_message;
        ROS_ERROR("%s", ossStm.str().c_str());
        return 1;
      }    
    }
    else
    {
      ROS_ERROR("Failed to call service camera/set_camera_info");
      return 1;
    }*/

    
    Rad.data=1;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && fcu_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.5;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(90 * 3.1416 / 180);
    //pose.pose.orientation.x = 3.1415;


    circle_center = pose.pose.position;

    geometry_msgs::TwistStamped vel;
    vel.twist.linear.x=0;
    vel.twist.linear.y=0;
    vel.twist.linear.z=0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        //local_vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Program Start");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    //std_msgs::String mode_now;  
    int iter = 0;
    while(ros::ok()){
        if( fcu_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            //std_msgs::String mode_now=fcu_state.mode;
            //printf("fcu_state.mode= %s",mode_now);
            ROS_INFO("current_fcu_state.mode= '%s'",fcu_state.mode.c_str()); 
            ROS_INFO("set_mode_client:%d",set_mode_client.call(offb_set_mode));
            ROS_INFO("offb_set_mode:%d",offb_set_mode.response.mode_sent);
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !fcu_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");     
                }
                last_request = ros::Time::now();  
            }
            
            if(fcu_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(0.05))){
                   if(track_state.data == 3) {
                        pose.pose.position.x = camera_pose.pose.position.x + object_pose.pose.position.x;
                        pose.pose.position.y = camera_pose.pose.position.y + object_pose.pose.position.y;
                        //pose.pose.position.x += 0.5;
                        //pose.pose.position.y += 0.5;
                        pose.pose.position.z = 1.5;
                        pose.pose.orientation.x = 10;

                        vel.twist.linear.x = 0.2;
                        vel.twist.linear.y = 0.;
                        vel.twist.linear.z = 0.5;
                        vel.twist.angular.z= 0.2;

                   }
                   else if(track_state.data == 4){
                       ROS_INFO("Track Lost!");
                       theta.data=0;
                       circle_center = camera_pose.pose.position;
                   }
                   else{
                       std::cout<<iter<<std::endl;
                       if (iter < 200){   
                            theta.data+=(36*0.05);
                            //pose.pose.position.x = circle_center.x + cos(Rad.data*theta.data*3.14159/180);
                            //pose.pose.position.y = circle_center.y + sin(Rad.data*theta.data*3.14159/180);
                            //pose.pose.position.z = 1.5; 
                            pose.pose.position.x = 0;
                            pose.pose.position.y = 0;
                            pose.pose.position.z = 1.5; 
                        }
                        else{
                            vel_skew.twist.linear.x = 0;
                            vel_skew.twist.linear.y = 0;
                            vel_skew.twist.linear.z = 0;
                            vel_skew.twist.angular.z = 0.5;
                        }
                   }
                   iter ++;
                } 
            
        }
        if (iter < 200){
            local_pos_pub.publish(pose);
            }
        else{
            local_vel_pub.publish(vel_skew);
        }
        //ROS_INFO("Next Loop");
        //local_vel_pub.publish(vel);
        // local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

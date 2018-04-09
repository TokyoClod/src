//=================================================================================================
// Copyright (c) 2012-2016, Institute of Flight Systems and Automatic Control,
// Technische Universität Darmstadt.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of hector_quadrotor nor the names of its contributors
//       may be used to endorse or promote products derived from this software
//       without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ugv_robot
{

class Teleop
{
private:

  ros::NodeHandle node_handle_;
  ros::Subscriber joy_subscriber_;
  ros::Publisher velocity_publisher_;
 

  geometry_msgs::PoseStamped pose_;


  struct Axis
  {
    Axis()
      : axis(0), factor(0.0), offset(0.0)
    {}

    int axis;
    double factor;
    double offset;
  };

  struct Button
  {
    Button()
      : button(0)
    {}

    int button;
  };

  struct
  {
    Axis x;
    Axis y;
    Axis z;
    Axis thrust;
    Axis yaw;
  } axes_;

  struct
  {
    Button slow;
    Button go;
    Button stop;
    Button interrupt;
  } buttons_;

  double slow_factor_;
  std::string base_link_frame_, base_stabilized_frame_, world_frame_;

public:
  Teleop()
  {
    ros::NodeHandle private_nh("~");

    private_nh.param<int>("x_axis", axes_.x.axis, 5);
    private_nh.param<int>("y_axis", axes_.y.axis, 4);
    private_nh.param<int>("z_axis", axes_.z.axis, 2);
    private_nh.param<int>("thrust_axis", axes_.thrust.axis, -3);
    private_nh.param<int>("yaw_axis", axes_.yaw.axis, 1);

    private_nh.param<double>("yaw_velocity_max", axes_.yaw.factor, 90.0);

    private_nh.param<int>("slow_button", buttons_.slow.button, 4);
    private_nh.param<int>("go_button", buttons_.go.button, 1);
    private_nh.param<int>("stop_button", buttons_.stop.button, 2);
    private_nh.param<int>("interrupt_button", buttons_.interrupt.button, 3);
    private_nh.param<double>("slow_factor", slow_factor_, 0.2);



    ros::NodeHandle robot_nh;

    // TODO factor out
    robot_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
    
    robot_nh.param<std::string>("world_frame", world_frame_, "world");
    robot_nh.param<std::string>("base_stabilized_frame", base_stabilized_frame_, "base_stabilized");


    ROS_INFO("goes well");
    private_nh.param<double>("x_velocity_max", axes_.x.factor, 2.0);
    private_nh.param<double>("y_velocity_max", axes_.y.factor, 2.0);
    private_nh.param<double>("z_velocity_max", axes_.z.factor, 2.0);

    joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&Teleop::joyTwistCallback, this, _1));                                    
    velocity_publisher_ = robot_nh.advertise<geometry_msgs::Twist>("ugv/cmd_vel", 10);
  }

  ~Teleop()
  {
    stop();
  }


  void joyTwistCallback(const sensor_msgs::JoyConstPtr &joy)
  {
    geometry_msgs::Twist velocity;

    velocity.linear.x = getAxis(joy, axes_.x);
    velocity.linear.y = getAxis(joy, axes_.y);
    velocity.linear.z = getAxis(joy, axes_.z);
    velocity.angular.z = getAxis(joy, axes_.yaw) * M_PI/180.0;
    if (getButton(joy, buttons_.slow))
    {
      velocity.linear.x *= slow_factor_;
      velocity.linear.y *= slow_factor_;
      velocity.linear.z *= slow_factor_;
      velocity.angular.z *= slow_factor_;
    }
    

    if (getButton(joy, buttons_.stop))
    {
        velocity.linear.x =0;
        velocity.linear.y =0;
        velocity.linear.z =0;
        velocity.angular.x =0;
        velocity.angular.y =0;
        velocity.angular.z =0;
    }

    velocity_publisher_.publish(velocity);
  }


  double getAxis(const sensor_msgs::JoyConstPtr &joy, const Axis &axis)
  {
    if (axis.axis == 0 || std::abs(axis.axis) > joy->axes.size())
    {
      ROS_ERROR_STREAM("Axis " << axis.axis << " out of range, joy has " << joy->axes.size() << " axes");
      return 0;
    }

    double output = std::abs(axis.axis) / axis.axis * joy->axes[std::abs(axis.axis) - 1] * axis.factor + axis.offset;

    // TODO keep or remove deadzone? may not be needed
    // if (std::abs(output) < axis.max_ * 0.2)
    // {
    //   output = 0.0;
    // }

    return output;
  }

  bool getButton(const sensor_msgs::JoyConstPtr &joy, const Button &button)
  {
    if (button.button <= 0 || button.button > joy->buttons.size())
    {
      ROS_ERROR_STREAM("Button " << button.button << " out of range, joy has " << joy->buttons.size() << " buttons");
      return false;
    }

    return joy->buttons[button.button - 1] > 0;
  }



  void stop()
  {
    if (velocity_publisher_.getNumSubscribers() > 0)
    {
      velocity_publisher_.publish(geometry_msgs::Twist());
    }
  }
};

} // namespace ugv_robot

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ugv_teleop");

  ugv_robot::Teleop teleop;
  ros::spin();

  return 0;
}
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_calibration_parsers/parse.h>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apply_camera_info");
  if (argc != 3)
  {
    ROS_INFO("usage: apply_camera_info <camera topic> <calibration file>");
    return 1;
  }

  ros::NodeHandle n;

  std::ostringstream ossSet;
  ossSet << argv[1] << "set_camera_info";
  std::string s = ossSet.str();
  ros::ServiceClient client = n.serviceClient<sensor_msgs::SetCameraInfo>(s);

  std::string camera_name = argv[1];
  std::string filename = argv[2];

  sensor_msgs::CameraInfo camera_info;
  camera_calibration_parsers::readCalibration(filename, camera_name, camera_info);
  ROS_INFO(camera_info.c_str);

  sensor_msgs::SetCameraInfo srv;
  srv.request.camera_info = camera_info;

  if (client.call(srv))
  {
    if(srv.response.success)
    {
      std::ostringstream ossInf;
      ossInf << srv.request.camera_info;
      ROS_INFO("%s", ossInf.str().c_str());
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
  }

  return 0;
}
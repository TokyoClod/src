#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TwistStamped.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/Int8.h"
#include "nav_msgs/Odometry.h"
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h> 

#include <visp3/gui/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpMbEdgeKltTracker.h>
#include <visp/vpTime.h>
#include <visp_bridge/camera.h>
#include <visp_bridge/image.h>
#include <visp_bridge/3dpose.h>

#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/gui/vpPlot.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag36artoolkit.h"
#include <apriltag/common/getopt.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace cv;

vpCameraParameters cam;
vpImage<unsigned char> dst; 
geometry_msgs::TwistStamped vel_skew;
cv_bridge::CvImageConstPtr cv_ptr;
vpHomogeneousMatrix wMc, wMu;
geometry_msgs::Pose ugv_pose, uav_pose;

void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{   
    
    cv_ptr = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8); 
    //dst = cv_bridge::toVispImage(*msg);
    cam = visp_bridge::toVispCameraParameters(*cam_info);
}

void camera_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg_camera_pose){
    geometry_msgs::PoseStamped camera_pose = *msg_camera_pose;
    wMc = visp_bridge::toVispHomogeneousMatrix(camera_pose.pose);
}

void ugv_pos_cb(const nav_msgs::Odometry::ConstPtr& msg_ugv_odom){
    nav_msgs::Odometry ugv_odom = *msg_ugv_odom;
    ugv_pose = ugv_odom.pose.pose;
    wMu = visp_bridge::toVispHomogeneousMatrix(ugv_pose);
}

void uav_pos_cb(const nav_msgs::Odometry::ConstPtr& msg_uav_odom){
    nav_msgs::Odometry uav_odom = *msg_uav_odom;
    uav_pose = uav_odom.pose.pose;
    wMc = visp_bridge::toVispHomogeneousMatrix(uav_pose);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ibvs_node");
    ros::NodeHandle nh;
    ros::Rate rate(10.0);

    double target_pixel_size, target_real_size;

    ros::param::get("~target_pixel_size", target_pixel_size);
    ros::param::get("~target_real_size", target_real_size);

    sensor_msgs::Image img_out;
    image_transport::ImageTransport it(nh);
    image_transport::CameraSubscriber camera_image = it.subscribeCamera("/iris/camera_downward/image_raw", 10, imageCallback);
    //image_transport::Publisher img_pub = it.advertise("Output_camera_image",1);
    image_transport::Publisher outimg_pub = it.advertise("/out_image", 10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("tag_velocity_skew",10);
    ros::Publisher track_state_pub = nh.advertise<std_msgs::Int8>("tag_tracker_status",10);
    // ros::Subscriber camera_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, camera_pose_cb);
    // ros::Subscriber ugv_pos_sub = nh.subscribe<nav_msgs::Odometry>("/ugv_ground_truth/state", 10, ugv_pos_cb);  //????
    ros::Subscriber uav_pos_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth", 10, uav_pos_cb);
    
    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    tf = tag36h11_create();
    tf->black_border = 1;
    cout << "Creating apriltag_detector..." << endl;
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 1.0;
    td->quad_sigma = 0.0;
    td->nthreads = 4;
    td->debug = 0;
    td->refine_edges = 1;
    td->refine_decode = 0;
    td->refine_pose = 0;

    // IBVS Part Init
    std_msgs::Int8 track_state;
    track_state.data = 0;

    vpServo task ;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::MEAN);

    double lambda0,lambdaoo,lambda0_d;
    ros::param::get("~lambda0", lambda0);
    ros::param::get("~lambdaoo", lambdaoo);
    ros::param::get("~lambda0_d", lambda0_d);
    vpAdaptiveGain lambda(lambda0,lambdaoo,lambda0_d);
    task.setLambda(0.5);

    //double Zd = 1.074;  
    ros::spinOnce();
    vpFeaturePoint pp[4], pd[4] ;
    cout<<"px:"<<cam.get_px()<<", py:"<<cam.get_py()<<endl;

    //double target_pixel_size
    //double target_real_size = 0.80;
    double u_1 = (640 + target_pixel_size) / 2.0;
    double v_1 = (480 + target_pixel_size) / 2.0;
    double u_2 = (640 - target_pixel_size) / 2.0;
    double v_2 = (480 - target_pixel_size) / 2.0;

    double u0 = cam.get_u0();
    double v0 = cam.get_v0();
    double Zd = target_real_size / target_pixel_size * cam.get_px();
    cout<<"Zd="<<Zd<<" U0="<<u0<<" V0="<<v0<<endl;   
    
    pd[0].buildFrom((u_1-u0) / cam.get_px(), (v_1-v0) / cam.get_py(), Zd);
    pd[1].buildFrom((u_1-u0) / cam.get_px(), (v_2-v0) / cam.get_py(), Zd);
    pd[2].buildFrom((u_2-u0) / cam.get_px(), (v_2-v0) / cam.get_py(), Zd);
    pd[3].buildFrom((u_2-u0) / cam.get_px(), (v_1-v0) / cam.get_py(), Zd);
    // pd[4].buildFrom(0, 0, Zd);

    pp[0].buildFrom((u_2-u0) / cam.get_px(), (v_2-v0) / cam.get_py(), Zd);
    pp[1].buildFrom((u_1-u0) / cam.get_px(), (v_2-v0) / cam.get_py(), Zd);
    pp[2].buildFrom((u_1-u0) / cam.get_px(), (v_1-v0) / cam.get_py(), Zd);
    pp[3].buildFrom((u_2-u0) / cam.get_px(), (v_1-v0) / cam.get_py(), Zd);
    // pp[4].buildFrom(0, 0, Zd);
    
    #ifdef VISP_HAVE_DISPLAY
        vpPlot plotter(2, 250*2, 500, 100, 200, "Real time curves plotter");
        plotter.setTitle(0, "Visual features error");
        plotter.setTitle(1, "Camera velocities");

        plotter.initGraph(0, 8);
        plotter.initGraph(1, 6);

        plotter.setLegend(0, 0, "x1");
        plotter.setLegend(0, 1, "y1");
        plotter.setLegend(0, 2, "x2");
        plotter.setLegend(0, 3, "y2");
        plotter.setLegend(0, 4, "x3");
        plotter.setLegend(0, 5, "y3");
        plotter.setLegend(0, 6, "x4");
        plotter.setLegend(0, 7, "y4");
        // plotter.setLegend(0, 8, "x5");
        // plotter.setLegend(0, 9, "y5");

        plotter.setLegend(1, 0, "v_x");
        plotter.setLegend(1, 1, "v_y");
        plotter.setLegend(1, 2, "v_z");
        plotter.setLegend(1, 3, "w_x");
        plotter.setLegend(1, 4, "w_y");
        plotter.setLegend(1, 5, "w_z");
    #endif

    
    // vpImagePoint point[4];
    // point[0].set_u(420);point[0].set_v(340);
    // point[1].set_u(420);point[0].set_v(140);
    // point[2].set_u(220);point[0].set_v(140);
    // point[3].set_u(220);point[0].set_v(340);
    for (unsigned int i = 0 ; i < 4 ; i++) {

        // vpFeatureBuilder::create(pd[i], cam, point[i]);
        // vpFeatureBuilder::create(pp[i], cam, point[i]);
        task.addFeature(pp[i], pd[i]);
        cout<<"pd["<<i<<"]=:"<<pd[i].get_x()<<","<<pd[i].get_y()<<","<<pd[i].get_Z()<<endl;
    }

    //ibvs ends

    // tf::TransformListener listener;

    Mat frame, gray;
    unsigned int iter = 0;
    while(ros::ok())
    {
        ros::spinOnce();
        ros::Time begin =ros::Time::now();
        //frame = cv_ptr->image;
        cvtColor(cv_ptr->image, gray, COLOR_BGR2GRAY);
        //imshow("Convert to Grey", gray);
        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);
        //cout << zarray_size(detections) << " tags detected" << endl;

        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            //circle(cv_ptr->image,Point(det->p[0][0], det->p[0][1]),r,Scalar(0,0,0));  
            line(cv_ptr->image, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
            //cout<<"p0-x"<<det->p[0][0]<<"p0-y"<<det->p[0][1]<<endl;
            line(cv_ptr->image, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
            line(cv_ptr->image, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
            line(cv_ptr->image, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);
            //cout<<"p0:"<<det->p[0][0]<<", "<<det->p[0][1]<<" p1:"<<det->p[1][0]<<", "<<det->p[1][1]<<" p2:"<<det->p[2][0]<<", "<<det->p[2][1]<<" p3:"<<det->p[3][0]<<", "<<det->p[3][1]<<endl;
            
            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(cv_ptr->image, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
        
            // update the feature points
            double temp = (abs(det->p[0][1] - det->p[1][1]) + abs(det->p[3][1] - det->p[2][1]) \
                          + abs(det->p[3][0] - det->p[0][0]) + abs(det->p[2][0] - det->p[1][0])) / 4;
            double Z = target_real_size / temp * cam.get_px(); //需要更稳定的估计方法
            // double Z1 = double(uav_pose.position.z) - 0.17;
            // cout<< "Z1:" << Z1 << " Z:" << Z << endl;
            for (unsigned int i = 0 ; i < 4 ; i++) {   //通过将3维点投影到图像平面，来更新特征点
                //point[i].track(cMo);
                stringstream ss;
                ss<<i;
                text = ss.str();
                putText(cv_ptr->image, text, Point(det->p[i][0]-5,
                                       det->p[i][1]-5),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
                double X = (det->p[i][0]-u0) / cam.get_px();
                double Y = (det->p[i][1]-v0) / cam.get_py();          
                pp[i].buildFrom(X, Y, Z);            
                //cout<<"pp["<<i<<"]=:"<<pp[i].get_x()<<","<<pp[i].get_y()<<","<<pp[i].get_Z()<<endl;   
            }
            // pp[4].buildFrom(double((det->c[0]-u0) / cam.get_px()), double((det->c[1]-v0) / cam.get_py()), Z);
            delete det;                     
        }
        
        
        // ibvs part
        if (zarray_size(detections) > 0){ 
            track_state.data = 1;            
            vpColVector v = task.computeControlLaw();
            double e = ( task.getError() ).sumSquare();
            cout<<"e=:"<<e<<endl;
            vpHomogeneousMatrix cMo(vpTranslationVector(0, 0, 0), vpRotationMatrix(vpRzyxVector(-1.5708, 0, 3.1416))); 
            vpVelocityTwistMatrix fVc;
            //fVc.buildFrom((wMc*cMo).inverse() * wMu);
            fVc.buildFrom((wMc*cMo)); //transfer the velocity from camera frame to World Frame
            vpColVector f_v(6);
            //cout<<endl<<"fVc:\n"<<fVc<<endl<<endl;
            //cout<<endl<<"wMc:\n"<<wMc<<endl<<endl;
            f_v = fVc * v;
            
            vel_skew.twist.linear.x = f_v[0];
            vel_skew.twist.linear.y = f_v[1];
            vel_skew.twist.linear.z = f_v[2];
            //vel_skew.twist.angular.x = rzyx[2];
            //vel_skew.twist.angular.y = rzyx[1];
            //vel_skew.twist.angular.z = rzyx[0];
            //vel_skew.twist.angular.x = f_v[3];
            //vel_skew.twist.angular.y = f_v[4];
            vel_skew.twist.angular.z = f_v[5];

            #ifdef VISP_HAVE_DISPLAY
                plotter.plot(0, iter, task.getError());
                plotter.plot(1, iter, v);
            #endif
        }
        else{
            track_state.data = 0;
            vel_skew.twist.linear.x = 0;
            vel_skew.twist.linear.y = 0;
            vel_skew.twist.linear.z = 0;
            vel_skew.twist.angular.x = 0;
            vel_skew.twist.angular.y = 0;
            vel_skew.twist.angular.z = 0;
        }
        //ibvs end
        iter++;
        zarray_destroy(detections);
        // imshow("Tag Detections", cv_ptr->image);
        outimg_pub.publish(cv_ptr->toImageMsg());

        vel_pub.publish(vel_skew);
        // cv_bridge::CvImage::toImageMsg();
        // imshow("Tag Detections", frame);
        // cout<<"track_state:"<<track_state<<endl;
        track_state_pub.publish(track_state);
        // waitKey(30);
        ros::spinOnce();
        rate.sleep();
        ros::Time end = ros::Time::now();
        ROS_INFO("%f ms",1000*(end-begin).toSec());
    }
    plotter.saveData(0, "/home/abner/catkin_ws/src/ibvs/log/error.dat", "matlab");
    plotter.saveData(1, "/home/abner/catkin_ws/src/ibvs/log/vc.dat", "matlab");
    task.kill();
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    return 0;
}



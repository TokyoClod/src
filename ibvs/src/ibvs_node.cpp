#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TwistStamped.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/Int8.h"
#include "nav_msgs/Odometry.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h> 

#include <visp3/gui/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpMbEdgeKltTracker.h>
#include <visp/vpTime.h>
#include <visp_bridge/camera.h>
#include <visp_bridge/image.h>
#include <visp_bridge/3dpose.h>

#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpGenericFeature.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/gui/vpPlot.h>

#include <iostream>

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag9h4.h"
#include "apriltag/tag25h10.h"
#include "apriltag/tag36artoolkit.h"
#include <apriltag/common/getopt.h>
#include <tf/transform_listener.h>

#include <PID_position.h>
using namespace std;
using namespace cv;

vpCameraParameters cam;
vpImage<unsigned char> dst; 
geometry_msgs::TwistStamped vel_skew, vel_skew_a, vel_skew_b;
cv_bridge::CvImageConstPtr cv_ptr;
vpHomogeneousMatrix wMc, wMu;
geometry_msgs::Pose ugv_pose, uav_pose;
double durationT;

void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{   
    
    cv_ptr = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8); 
    // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
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

double get_area(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3 ){
    double s;
    s = 0.5*(x0*y1 - x1*y0+\
             x1*y2 - x2*y1+\
             x2*y3 - x3*y2+\
             x3*y0 - x0*y3);
    return s > 0 ? s : -s;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ibvs_node");
    ros::NodeHandle nh;
    ros::Rate rate(10.0);
    
    /*************************** 初始化存储位置 ****************************/
    ofstream oFile_uav, oFile_ugv; 
    oFile_uav.open("/home/abner/catkin_ws/src/ibvs/log/uav_pose.csv", ios::out | ios::trunc);  
    oFile_ugv.open("/home/abner/catkin_ws/src/ibvs/log/ugv_pose.csv", ios::out | ios::trunc);  
    oFile_uav << "iter" << "," << "x" << "," << "y" << "," << "z" << endl;   
    oFile_ugv << "iter" << "," << "x" << "," << "y" << "," << "z" << endl;  

    /*************************** 从参数服务器读取参数 ****************************/
    double target_pixel_size_a, target_pixel_size_b, target_real_size_a, target_real_size_b;
    ros::param::get("~target_pixel_size_a", target_pixel_size_a);
    ros::param::get("~target_pixel_size_b", target_pixel_size_b);
    cout<<"target_pixel_size_a:"<<target_pixel_size_a<<" target_pixel_size_b:"<<target_pixel_size_b<<endl;
    ros::param::get("~target_real_size_a", target_real_size_a);
    ros::param::get("~target_real_size_b", target_real_size_b);

    double lambda0,lambdaoo,lambda0_d;
    ros::param::get("~lambda0", lambda0);
    ros::param::get("~lambdaoo", lambdaoo);
    ros::param::get("~lambda0_d", lambda0_d);

    double Pa,Ia,Da,Pb,Ib,Db;
    ros::param::get("~Pa", Pa);
    ros::param::get("~Ia", Ia);
    ros::param::get("~Da", Da);
    ros::param::get("~Pb", Pb);
    ros::param::get("~Ib", Ib);
    ros::param::get("~Db", Db);

    int debug_flag, export_flag, pid_flag, taskb_flag;
    ros::param::get("~debug_flag", debug_flag);
    ros::param::get("~export_flag", export_flag);
    ros::param::get("~pid_flag", pid_flag);
    ros::param::get("~taskb_flag", taskb_flag);

    /*************************** 初始化发布器与订阅器 ****************************/
    sensor_msgs::Image img_out;
    image_transport::ImageTransport it(nh);
    image_transport::CameraSubscriber camera_image = it.subscribeCamera("/iris/camera_downward/image_raw", 10, imageCallback);
    //image_transport::Publisher img_pub = it.advertise("Output_camera_image",1);
    image_transport::Publisher outimg_pub = it.advertise("/out_image", 10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("tag_velocity_skew",10);
    ros::Publisher track_state_pub = nh.advertise<std_msgs::Int8>("tag_tracker_status",10);
    // ros::Subscriber camera_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, camera_pose_cb);
    ros::Subscriber ugv_pos_sub = nh.subscribe<nav_msgs::Odometry>("/ugv_ground_truth/state", 10, ugv_pos_cb);  //????
    ros::Subscriber uav_pos_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth", 10, uav_pos_cb);
    
    /*************************** 初始化apriltag检测器 ****************************/
    apriltag_family_t *tf = NULL;
    tf = tag25h10_create();
    // tf = tag9h4_create();
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

    ros::spinOnce();
    rate.sleep();
    // ros::spinOnce();
    // IBVS Part Init
    std_msgs::Int8 track_state;
    track_state.data = 0;

    /*************************** 初始化视觉伺服任务 ****************************/
    vpServo task_a, task_b ;
    task_a.setServo(vpServo::EYEINHAND_CAMERA);
    task_a.setInteractionMatrixType(vpServo::CURRENT);
    task_b.setServo(vpServo::EYEINHAND_CAMERA);
    task_b.setInteractionMatrixType(vpServo::CURRENT);

    // vpAdaptiveGain lambda(lambda0,lambdaoo,lambda0_d);
    task_a.setLambda(0.4);
    task_b.setLambda(0.2);

    /*************************** 初始化PID速度补偿 ****************************/
    PID_position pid_a(Pa, Ia, Da);
    PID_position pid_b(Pb, Ib, Db);

    //double Zd = 1.074;  
    ros::spinOnce();
    vpFeaturePoint ppa[4], ppb[4], pda[4] ,pdb[4];
    cout<<"px:"<<cam.get_px()<<", py:"<<cam.get_py()<<endl;


    /********************** First feature (x,y) 点特征 ***********************/
    #pragma region firstfeature
    double u_1a = (640 + target_pixel_size_a) / 2.0;
    double v_1a = (480 + target_pixel_size_a) / 2.0;
    double u_2a = (640 - target_pixel_size_a) / 2.0;
    double v_2a = (480 - target_pixel_size_a) / 2.0;

    double u_1b = (640 + target_pixel_size_b) / 2.0;
    double v_1b = (480 + target_pixel_size_b) / 2.0;
    double u_2b = (640 - target_pixel_size_b) / 2.0;
    double v_2b = (480 - target_pixel_size_b) / 2.0;

    double u0 = cam.get_u0();
    double v0 = cam.get_v0();
    double Zd_a = target_real_size_a / target_pixel_size_a * cam.get_px() + 1.2;
    double Zd_b = target_real_size_b / target_pixel_size_b * cam.get_px() + 1.2;
    cout<<"Zd_a="<<Zd_a<<" Zd_b="<<Zd_b<<" U0="<<u0<<" V0="<<v0<<endl;   
    
    pda[0].buildFrom((u_1a-u0) / cam.get_px(), (v_1a-v0) / cam.get_py(), Zd_a);
    pda[1].buildFrom((u_1a-u0) / cam.get_px(), (v_2a-v0) / cam.get_py(), Zd_a);
    pda[2].buildFrom((u_2a-u0) / cam.get_px(), (v_2a-v0) / cam.get_py(), Zd_a);
    pda[3].buildFrom((u_2a-u0) / cam.get_px(), (v_1a-v0) / cam.get_py(), Zd_a);
    // pd[4].buildFrom(0, 0, Zd);
    pdb[0].buildFrom((u_1b-u0) / cam.get_px(), (v_1b-v0) / cam.get_py(), Zd_b);
    pdb[1].buildFrom((u_1b-u0) / cam.get_px(), (v_2b-v0) / cam.get_py(), Zd_b);
    pdb[2].buildFrom((u_2b-u0) / cam.get_px(), (v_2b-v0) / cam.get_py(), Zd_b);
    pdb[3].buildFrom((u_2b-u0) / cam.get_px(), (v_1b-v0) / cam.get_py(), Zd_b);

    ppa[0].buildFrom((u_2a-u0) / cam.get_px(), (v_2a-v0) / cam.get_py(), Zd_a);
    ppa[1].buildFrom((u_1a-u0) / cam.get_px(), (v_2a-v0) / cam.get_py(), Zd_a);
    ppa[2].buildFrom((u_1a-u0) / cam.get_px(), (v_1a-v0) / cam.get_py(), Zd_a);
    ppa[3].buildFrom((u_2a-u0) / cam.get_px(), (v_1a-v0) / cam.get_py(), Zd_a);
    // pp[4].buildFrom(0, 0, Zd);
    ppb[0].buildFrom((u_2b-u0) / cam.get_px(), (v_2b-v0) / cam.get_py(), Zd_b);
    ppb[1].buildFrom((u_1b-u0) / cam.get_px(), (v_2b-v0) / cam.get_py(), Zd_b);
    ppb[2].buildFrom((u_1b-u0) / cam.get_px(), (v_1b-v0) / cam.get_py(), Zd_b);
    ppb[3].buildFrom((u_2b-u0) / cam.get_px(), (v_1b-v0) / cam.get_py(), Zd_b);
    #pragma endregion firstfeature

    /********************** Second feature log (Z/Zd) Log高度特征 ***********************/
    #pragma region secondfeature
    vpGenericFeature logZd_a(1); 
    vpGenericFeature logZd_b(1);
    logZd_a.set_s(log(Zd_a));
    logZd_b.set_s(log(Zd_b));
    double x = 0; //The x coordinate of the current point.
    double y = 0; //The y coordinate of the current point.
    double Z = 5; //The depth of the current point.
    vpGenericFeature logZ_a(1); //The dimension of the feature is 1.
    vpGenericFeature logZ_b(1); //The dimension of the feature is 1.
    logZ_a.set_s( log(Z) );
    logZ_b.set_s( log(Z) );
    #pragma endregion secondfeature
    
    #ifdef VISP_HAVE_DISPLAY
        vpPlot plotter(4, 250*2, 500*2, 100, 200, "Real time curves plotter");
        plotter.setTitle(0, "T0 Visual features error");
        plotter.setTitle(1, "T0 Camera velocities");
        plotter.setTitle(2, "T1 Visual features error");
        plotter.setTitle(3, "T1 Camera velocities");

        plotter.initGraph(0, 9);
        plotter.initGraph(1, 6);
        plotter.initGraph(2, 9);
        plotter.initGraph(3, 6);

        plotter.setLegend(0, 0, "x1");
        plotter.setLegend(0, 1, "y1");
        plotter.setLegend(0, 2, "x2");
        plotter.setLegend(0, 3, "y2");
        plotter.setLegend(0, 4, "x3");
        plotter.setLegend(0, 5, "y3");
        plotter.setLegend(0, 6, "x4");
        plotter.setLegend(0, 7, "y4");
        plotter.setLegend(0, 8, "z");

        plotter.setLegend(1, 0, "v_x");
        plotter.setLegend(1, 1, "v_y");
        plotter.setLegend(1, 2, "v_z");
        plotter.setLegend(1, 3, "w_x");
        plotter.setLegend(1, 4, "w_y");
        plotter.setLegend(1, 5, "w_z");

        plotter.setLegend(2, 0, "x1");
        plotter.setLegend(2, 1, "y1");
        plotter.setLegend(2, 2, "x2");
        plotter.setLegend(2, 3, "y2");
        plotter.setLegend(2, 4, "x3");
        plotter.setLegend(2, 5, "y3");
        plotter.setLegend(2, 6, "x4");
        plotter.setLegend(2, 7, "y4");
        plotter.setLegend(2, 8, "z");

        plotter.setLegend(3, 0, "v_x");
        plotter.setLegend(3, 1, "v_y");
        plotter.setLegend(3, 2, "v_z");
        plotter.setLegend(3, 3, "w_x");
        plotter.setLegend(3, 4, "w_y");
        plotter.setLegend(3, 5, "w_z");
    #endif

    /********************** 给伺服任务添加特征 ***********************/
    for (unsigned int i = 0 ; i < 4 ; i++) {
        // vpFeatureBuilder::create(pp[i], cam, point[i]);
        task_a.addFeature(ppa[i], pda[i]);
        task_b.addFeature(ppb[i], pdb[i]);
        cout<<"pda["<<i<<"]=:"<<pda[i].get_x()<<","<<pda[i].get_y()<<","<<pda[i].get_Z()<<endl;
        cout<<"pdb["<<i<<"]=:"<<pdb[i].get_x()<<","<<pdb[i].get_y()<<","<<pdb[i].get_Z()<<endl;
    }
    cout<<endl;
    task_a.addFeature(logZ_a, logZd_a);
    task_a.print();
    cout<<endl;
    task_b.addFeature(logZ_b, logZd_b);
    task_b.print();

    // tf::TransformListener listener;

    Mat frame, gray;
    unsigned int iter = 0;
    int flag_task_a, flag_task_b;
    double e0, e1, s0, s1;
    while(ros::ok())
    {
        flag_task_a = flag_task_b = 0;
        ros::spinOnce();
        ros::Time begin =ros::Time::now();
        if(cv_ptr)
        {
            //frame = cv_ptr->image;
            cvtColor(cv_ptr->image, gray, COLOR_BGR2GRAY);
            //imshow("Convert to Grey", gray);
            // Make an image_u8_t header for the Mat data
            image_u8_t im = { .width = gray.cols,
                .height = gray.rows,
                .stride = gray.cols,
                .buf = gray.data
            };

            // AprilTag检测
            zarray_t *detections = apriltag_detector_detect(td, &im);
            // cout << zarray_size(detections) << " tags detected" << endl;

            // Draw detection outlines and update features
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                //circle(cv_ptr->image,Point(det->p[0][0], det->p[0][1]),r,Scalar(0,0,0));  
                line(cv_ptr->image, Point(det->p[0][0], det->p[0][1]),
                        Point(det->p[1][0], det->p[1][1]),
                        Scalar(0, 0xff, 0), 2);
                line(cv_ptr->image, Point(det->p[0][0], det->p[0][1]),
                        Point(det->p[3][0], det->p[3][1]),
                        Scalar(0, 0, 0xff), 2);
                line(cv_ptr->image, Point(det->p[1][0], det->p[1][1]),
                        Point(det->p[2][0], det->p[2][1]),
                        Scalar(0xff, 0, 0), 2);
                line(cv_ptr->image, Point(det->p[2][0], det->p[2][1]),
                        Point(det->p[3][0], det->p[3][1]),
                        Scalar(0xff, 0, 0), 2);
                // cout<<"p0:"<<det->p[0][0]<<", "<<det->p[0][1]<<" p1:"<<det->p[1][0]<<", "<<det->p[1][1]<<" p2:"<<det->p[2][0]<<", "<<det->p[2][1]<<" p3:"<<det->p[3][0]<<", "<<det->p[3][1]<<endl;
                
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
                // double Za = target_real_size_a / temp * cam.get_px()+1.2; //需要更稳定的估计方法
                // double Z1 = double(uav_pose.position.z) - 0.17;
                // cout<< "Z1:" << Z1 << " Z:" << Z << endl;
                // cout<< " Z:" << Z << endl;

                for (unsigned int i = 0 ; i < 4 ; i++) {   //通过将3维点投影到图像平面，来更新特征点
                    //point[i].track(cMo);
                    stringstream ss;
                    ss<<i;
                    text = ss.str();
                    putText(cv_ptr->image, text, Point(det->p[i][0]-5, det->p[i][1]-5),
                            fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
                    double X = (det->p[i][0]-u0) / cam.get_px();
                    double Y = (det->p[i][1]-v0) / cam.get_py();  
                    if(det->id == 0){
                        double Za = target_real_size_a / temp * cam.get_px()+1.2; //高度估计
                        ppa[i].buildFrom(X, Y, Za); 
                        s0 = get_area(det->p[0][0],det->p[0][1],det->p[1][0],det->p[1][1],det->p[2][0],det->p[2][1],det->p[3][0],det->p[3][1]);
                        // cout<<"s0:"<<s0<<endl;
                        flag_task_a = 1;

                        logZ_a.set_s(log(Za));
                        vpMatrix LlogZa(1, 6);
                        LlogZa[0][0] = LlogZa[0][1] = LlogZa[0][5] = 0;
                        LlogZa[0][2] = -1 / Za;
                        LlogZa[0][3] = -ppa[0].get_y();
                        LlogZa[0][4] = ppa[0].get_x();
                        logZ_a.setInteractionMatrix(LlogZa);
                    }
                    else if(det->id == 1){
                        double Zb = target_real_size_b / temp * cam.get_px()+1.2; //高度估计
                        ppb[i].buildFrom(X, Y, Zb); 
                        s1 = get_area(det->p[0][0],det->p[0][1],det->p[1][0],det->p[1][1],det->p[2][0],det->p[2][1],det->p[3][0],det->p[3][1]);
                        // cout<<"s1:"<<s1<<endl;
                        if (taskb_flag){
                            if(s1 > 2500)  flag_task_b = 1;
                        }    

                        logZ_b.set_s(log(Zb));
                        vpMatrix LlogZb(1, 6);
                        LlogZb[0][0] = LlogZb[0][1] = LlogZb[0][5] = 0;
                        LlogZb[0][2] = -1 / Zb;
                        LlogZb[0][3] = -ppb[0].get_y();
                        LlogZb[0][4] = ppb[0].get_x();
                        logZ_b.setInteractionMatrix(LlogZb);

                    }
                    else{
                        ROS_ERROR("#1: No This ID!!");
                    }        
                    // pp[i].buildFrom(X, Y, Z);            
                    //cout<<"pp["<<i<<"]=:"<<pp[i].get_x()<<","<<pp[i].get_y()<<","<<pp[i].get_Z()<<endl;   
                }

                delete det;                     
            }
            if(debug_flag)
                cout<<"flag_a:"<<flag_task_a<<" flag_b:"<<flag_task_b<<"\t";
            

            
            vpHomogeneousMatrix cMo(vpTranslationVector(0, 0, 0), vpRotationMatrix(vpRzyxVector(-1.5708, 0, 3.1416))); 
            vpVelocityTwistMatrix fVc;
            //fVc.buildFrom((wMc*cMo).inverse() * wMu);
            fVc.buildFrom((wMc*cMo)); //transfer the velocity from camera frame to World Frame
            vpColVector f_v(6), v(6);

            /**************  计算IBVS控制器的输出参考速度：外部图标 ***********/
            if (flag_task_a == 1){ 
                track_state.data = 1;         
                if(pid_flag){
                    /**************  Using IBVS with PID ***********/
                    vpColVector pre_error = task_a.computeError();
                    unsigned int dimError = pre_error.getRows();
                    vpColVector new_error(dimError);
                    for (unsigned int k = 0; k <  dimError; k++) {
                        new_error[k] = pid_a.pid_control(pre_error[k]);
                        // std::cout << "new_error "<<k<<":"<<pre_error[k]<<std::endl;
                    }
                    task_a.setError(new_error);
                    v = task_a.computeControlLaw_pid();
                    e0 = new_error.sumSquare();
                }
                else{
                    /**************  Using IBVS without PID ***********/
                    v = task_a.computeControlLaw();
                    e0 = ( task_a.getError() ).sumSquare();
                }
              
                f_v = fVc * v;
                // cout<<endl<<"f_v:\n"<<f_v<<endl<<endl;
                
                vel_skew_a.twist.linear.x = f_v[0];
                vel_skew_a.twist.linear.y = f_v[1];
                vel_skew_a.twist.linear.z = f_v[2];
                vel_skew_a.twist.angular.z = f_v[5];

                plotter.plot(0, iter, task_a.getError());
                plotter.plot(1, iter, f_v);
                // plotter.plot(2, i/ter, task_b.getError());
                // plotter.plot(3, iter, f_v);
            }

            /**************  计算IBVS控制器的输出参考速度：内部图标优先 ***********/
            if (flag_task_b == 1){ 
                track_state.data = 1;          
                if (pid_flag){
                    /**************  Using IBVS with PID ***********/
                    vpColVector pre_error = task_b.computeError();
                    unsigned int dimError = pre_error.getRows();
                    vpColVector new_error(dimError);
                    for (unsigned int k = 0; k <  dimError; k++) {
                        new_error[k] = pid_b.pid_control(pre_error[k]);
                        // std::cout << "new_error "<<k<<":"<<pre_error[k]<<std::endl;
                    }
                    task_b.setError(new_error);
                    v = task_b.computeControlLaw_pid();
                    e1 = new_error.sumSquare();
                }
                else{
                    /**************  Using IBVS without PID ***********/
                    v = task_b.computeControlLaw();
                    e1 = ( task_b.getError() ).sumSquare();
                }
                // cout<<"e1=:"<<e1<<endl;
                
                f_v = fVc * v;
                // cout<<endl<<"f_v:\n"<<f_v<<endl<<endl;
                
                vel_skew_b.twist.linear.x = f_v[0];
                vel_skew_b.twist.linear.y = f_v[1];
                vel_skew_b.twist.linear.z = f_v[2];
                vel_skew_b.twist.angular.z = f_v[5];

                // plotter.plot(0, iter, task_a.getError());
                // plotter.plot(1, iter, v);
                plotter.plot(2, iter, task_b.getError());
                plotter.plot(3, iter, f_v);

                //judge land situation
                // if(e < 0.001){
                //     durationT += 0.1;
                //     if(durationT > 3.0){
                //         track_state.data = 2;
                //         track_state_pub.publish(track_state);
                //         return 0;
                //     }
                // }
                // else{
                //     durationT = 0;
                // }
                // cout << "durationT:" << durationT << endl;
     
            }
            
            if ((flag_task_a || flag_task_b) == 0){
                track_state.data = 0;
                vel_skew.twist.linear.x = 0;
                vel_skew.twist.linear.y = 0;
                vel_skew.twist.linear.z = 0;
                vel_skew.twist.angular.x = 0;
                vel_skew.twist.angular.y = 0;
                vel_skew.twist.angular.z = 0;
            }

            zarray_destroy(detections);
            outimg_pub.publish(cv_ptr->toImageMsg());

        }
        else{} // 避免cv_bridge没获得图像信息

        oFile_uav << iter << "," << uav_pose.position.x << "," << uav_pose.position.y << "," << uav_pose.position.z << endl;   
        oFile_ugv << iter << "," << ugv_pose.position.x << "," << ugv_pose.position.y << "," << ugv_pose.position.z << endl;
       
        //ibvs end
        iter++;
        
        // imshow("Tag Detections", cv_ptr->image);

        /**************  debug show ***********/
        if (debug_flag){
            if(flag_task_b==1){
                cout<<"e1=:"<< e1 <<" s1:" << s1 << "\t";
            }
            else{
                cout<<"e1=:NULL" <<" s1:" <<" NULL" << "\t";
            }
            if(flag_task_a==1){
                cout<<"e0=:"<< e0 <<" s0:" << s0 <<" ";
            }
            else{
                cout<<"e0=:NULL" <<" s0:" <<" NULL";
            }
            cout<<endl;
        } 

        /**************  发布速度指令 ***********/
        if(flag_task_b==1){
            vel_pub.publish(vel_skew_b);
        }
        else if(flag_task_a==1){
            vel_pub.publish(vel_skew_a);
        }
        else{
            // ROS_ERROR("#2:No This ID!!")
            vel_pub.publish(vel_skew);
        }   
        
        // vel_pub.publish(vel_skew);
        // cv_bridge::CvImage::toImageMsg();
        // imshow("Tag Detections", frame);
        // cout<<"track_state:"<<track_state<<endl;
        track_state_pub.publish(track_state);
        // waitKey(30);
        ros::spinOnce();
        rate.sleep();
        ros::Time end = ros::Time::now();
        // ROS_INFO("%f ms",1000*(end-begin).toSec());
        // cv_ptr.reset();
    }
    if (export_flag){
        plotter.saveData(0, "/home/abner/catkin_ws/src/ibvs/log/T0error.dat", "matlab");
        plotter.saveData(1, "/home/abner/catkin_ws/src/ibvs/log/T0vc.dat", "matlab");
        plotter.saveData(2, "/home/abner/catkin_ws/src/ibvs/log/T1error.dat", "matlab");
        plotter.saveData(3, "/home/abner/catkin_ws/src/ibvs/log/T1vc.dat", "matlab");

        oFile_uav.close();
        oFile_ugv.close();
    }
    

    task_a.kill();
    task_b.kill();
    apriltag_detector_destroy(td);
    tag25h10_destroy(tf);
    // tag9h4_destroy(tf);

    return 0;
}



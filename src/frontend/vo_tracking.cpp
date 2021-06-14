#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <include/tic_toc_ros.h>
#include <include/common.h>
#include <include/f2f_tracking.h>
#include <include/rviz_frame.h>
#include <include/rviz_path.h>

#include <include/yamlRead.h>
#include <include/cv_draw.h>
//#include <sopvo/KeyFrame.h>
#include <include/octomap_feeder.h>
#include <tf/transform_listener.h>
#include <fstream>


namespace sopvo_ns
{
class TrackingNodeletClass : public nodelet::Nodelet
{
public:
    TrackingNodeletClass()  {;}
    ~TrackingNodeletClass() {;}
private:

    enum TYPEOFCAMERA cam_type;
    F2FTracking   *cam_tracker;
    //Subscribers
    message_filters::Subscriber<sensor_msgs::Image> img0_sub;
    message_filters::Subscriber<sensor_msgs::Image> img1_sub;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MyExactSyncPolicy;
    message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MyApproSyncPolicy;
    message_filters::Synchronizer<MyApproSyncPolicy> * approSync_;
    ros::Subscriber senser_fusion_pose;

    //Octomap
    OctomapFeeder* octomap_pub;
    //Visualization
    cv::Mat img0_vis;
    cv::Mat img1_vis;
    image_transport::Publisher img0_pub;
    image_transport::Publisher img1_pub;
    ros::Publisher vision_pose_pub;

    RVIZFrame* frame_pub;
    RVIZPath*  vision_path_pub;
    RVIZPath*  path_lc_pub;
    tf::StampedTransform tranOdomMap;
    tf::TransformListener listenerOdomMap;
    //result output
    int frame_counter;
    bool enable_output_file;
    std::ofstream fd;
    std::string output_file_path;

    virtual void onInit()
    {
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
        //Publisher
        vision_path_pub = new RVIZPath(nh,"/vision_path","map",1,3000);
        path_lc_pub     = new RVIZPath(nh,"/vision_path_lc","map",1,3000);
        frame_pub       = new RVIZFrame(nh,"/vo_camera_pose","map","/vo_curr_frame","map");
        image_transport::ImageTransport it(nh);
        img0_pub = it.advertise("/vo_img0", 1);
        img1_pub = it.advertise("/vo_img1", 1);

        cam_tracker = new F2FTracking();
        //Load Parameter
        string configFilePath, voParamPath;
        nh.getParam("/yamlconfigfile",   configFilePath);
        nh.getParam("/voparamfilepath", voParamPath);
        cout << "camera info path: " << configFilePath << endl;
        cout << "sopvo params path: " << voParamPath << endl;
        int cam_type_from_yaml = getIntVariableFromYaml(configFilePath,"type_of_cam");
        int image_width  = getIntVariableFromYaml(configFilePath,"image_width");
        int image_height = getIntVariableFromYaml(configFilePath,"image_height");
        // not used, reserved for future functions
        Vec4 parameter = Vec4(getDoubleVariableFromYaml(configFilePath,"para_1"),
                              getDoubleVariableFromYaml(configFilePath,"para_2"),
                              getDoubleVariableFromYaml(configFilePath,"para_3"),
                              getDoubleVariableFromYaml(configFilePath,"para_4"));

        cout << "image_width :" << image_width << endl;
        cout << "image_height:" << image_height << endl;

        img0_sub.subscribe(nh, "/vo/image0", 1);
        img1_sub.subscribe(nh, "/vo/image1", 1);
        
        if(cam_type_from_yaml==0) cam_type = STEREO_KITTI;
        if(cam_type_from_yaml==1) cam_type = STEREO_EuRoC_MAV;
        if(cam_type_from_yaml==2) cam_type = Realsense_T265;
        Mat4x4  initPose = Mat44FromYaml(configFilePath,"T_init");
        SE3 T_init = SE3(initPose.topLeftCorner(3,3),initPose.topRightCorner(3,1));

        if(cam_type==STEREO_KITTI)
        {
            cv::Mat cam0_cameraMatrix = cameraMatrixFromYamlIntrinsics(configFilePath,"cam0_intrinsics");
            cv::Mat cam0_distCoeffs   = distCoeffsFromYaml(configFilePath,"cam0_distortion_coeffs");
            Mat4x4  mat_body_cam0  = Mat44FromYaml(configFilePath,"T_body_cam0");
            SE3 T_b_c0 = SE3(mat_body_cam0.topLeftCorner(3,3),
                                mat_body_cam0.topRightCorner(3,1));
            cv::Mat cam1_cameraMatrix = cam0_cameraMatrix;
            cv::Mat cam1_distCoeffs   = cam0_distCoeffs;
            Mat4x4  mat_cam0_cam1 = Mat44FromYaml(configFilePath,"T_cam0_cam1");
            SE3 T_c0_c1 = SE3(mat_cam0_cam1.topLeftCorner(3,3),
                                mat_cam0_cam1.topRightCorner(3,1));
            Mat4x4  mat_w_b  = Mat44FromYaml(configFilePath,"T_world_body");
            SE3 T_w_b = SE3(mat_w_b.topLeftCorner(3,3),mat_w_b.topRightCorner(3,1));
            SE3 T_w_c0 = T_w_b*T_b_c0;
            cam_tracker->init(voParamPath,image_width,image_height,image_width,image_height,
                              cam0_cameraMatrix,cam0_distCoeffs,
                              T_w_c0,
                              parameter,
                              STEREO_KITTI,
                              1.0,
                              cam1_cameraMatrix,cam1_distCoeffs,
                              T_c0_c1,
                              T_init);

            exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(2), img0_sub, img1_sub);
            exactSync_->registerCallback(boost::bind(&TrackingNodeletClass::image_input_callback, this, _1, _2));
        }
        if(cam_type==STEREO_EuRoC_MAV)
        {
            cv::Mat cam0_cameraMatrix = cameraMatrixFromYamlIntrinsics(configFilePath,"cam0_intrinsics");
            cv::Mat cam0_distCoeffs   = distCoeffsFromYaml(configFilePath,"cam0_distortion_coeffs");
            Mat4x4  mat_mavimu_cam0  = Mat44FromYaml(configFilePath,"T_mavimu_cam0");
            SE3 T_mavi_c0 = SE3(mat_mavimu_cam0.topLeftCorner(3,3),
                                mat_mavimu_cam0.topRightCorner(3,1));
            cv::Mat cam1_cameraMatrix = cameraMatrixFromYamlIntrinsics(configFilePath,"cam1_intrinsics");
            cv::Mat cam1_distCoeffs   = distCoeffsFromYaml(configFilePath,"cam1_distortion_coeffs");
            Mat4x4  mat_mavimu_cam1  = Mat44FromYaml(configFilePath,"T_mavimu_cam1");
            SE3 T_mavi_c1 = SE3(mat_mavimu_cam1.topLeftCorner(3,3),
                                mat_mavimu_cam1.topRightCorner(3,1));
            SE3 T_c0_c1 = T_mavi_c0.inverse()*T_mavi_c1;
            Mat4x4  mat_i_mavimu  = Mat44FromYaml(configFilePath,"T_imu_mavimu");
            SE3 T_i_mavi = SE3(mat_i_mavimu.topLeftCorner(3,3),mat_i_mavimu.topRightCorner(3,1));
            SE3 T_i_c0 = T_i_mavi*T_mavi_c0;
            cam_tracker->init(voParamPath,image_width,image_height,image_width,image_height,
                              cam0_cameraMatrix,cam0_distCoeffs,
                              T_i_c0,
                              parameter,
                              STEREO_EuRoC_MAV,
                              1.0,
                              cam1_cameraMatrix,cam1_distCoeffs,
                              T_c0_c1,
                              T_init);
            
            exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(2), img0_sub, img1_sub);
            exactSync_->registerCallback(boost::bind(&TrackingNodeletClass::image_input_callback, this, _1, _2));
        }
        if(cam_type==Realsense_T265)
        {
            int image_width_out  = getIntVariableFromYaml(configFilePath,"image_width_out");
            int image_height_out = getIntVariableFromYaml(configFilePath,"image_height_out");
            cv::Mat cam0_cameraMatrix = cameraMatrixFromYamlIntrinsics(configFilePath,"cam0_intrinsics");
            cv::Mat cam0_distCoeffs   = distCoeffsFromYaml(configFilePath,"cam0_distortion_coeffs");
            cv::Mat cam1_cameraMatrix = cameraMatrixFromYamlIntrinsics(configFilePath,"cam1_intrinsics");
            cv::Mat cam1_distCoeffs   = distCoeffsFromYaml(configFilePath,"cam1_distortion_coeffs");
            
            Mat4x4  mat_body_cam0  = Mat44FromYaml(configFilePath,"T_body_cam0");
            SE3 T_b_c0 = SE3(mat_body_cam0.topLeftCorner(3,3),
                                mat_body_cam0.topRightCorner(3,1));
            
            Mat4x4  mat_cam1_cam0 = Mat44FromYaml(configFilePath,"T_cam1_cam0");

            SE3 T_c1_c0 = SE3(mat_cam1_cam0.topLeftCorner(3,3),mat_cam1_cam0.topRightCorner(3,1));

            SE3 T_c0_c1 = T_c1_c0.inverse();
                                
            Mat4x4  mat_w_b  = Mat44FromYaml(configFilePath,"T_world_body");
            SE3 T_w_b = SE3(mat_w_b.topLeftCorner(3,3),mat_w_b.topRightCorner(3,1));
            SE3 T_w_c0 = T_w_b*T_b_c0;
            cam_tracker->init(voParamPath,image_width,image_height, image_width_out, image_height_out,
                              cam0_cameraMatrix,cam0_distCoeffs,
                              T_w_c0,
                              parameter,
                              Realsense_T265,
                              1.0,
                              cam1_cameraMatrix,cam1_distCoeffs,
                              T_c0_c1,
                              T_init);

            approSync_ = new message_filters::Synchronizer<MyApproSyncPolicy>(MyApproSyncPolicy(10), img0_sub, img1_sub);
            approSync_->registerCallback(boost::bind(&TrackingNodeletClass::image_input_callback, this, _1, _2));
        }

        frame_counter = 0;
        nh.getParam("/output_file_path", output_file_path);
        cout << output_file_path << endl;
        if(output_file_path=="0")
        {
            enable_output_file = false;
        }else
        {
            enable_output_file = true;
            fd.open(output_file_path.c_str());
            fd.close();
        }

        cout << "start tracking thread" << endl;
    }
    
    void image_input_callback(const sensor_msgs::ImageConstPtr & img0_Ptr,
                              const sensor_msgs::ImageConstPtr & img1_Ptr)
    {
        ros::Time tstamp = img0_Ptr->header.stamp;
        cv_bridge::CvImagePtr cvbridge_img0  = cv_bridge::toCvCopy(img0_Ptr, img0_Ptr->encoding);
        cv_bridge::CvImagePtr cvbridge_img1  = cv_bridge::toCvCopy(img1_Ptr, img1_Ptr->encoding);
        bool newkf;//new key frame
        bool reset_cmd;//reset command to localmap node
        this->cam_tracker->image_feed(tstamp.toSec(),
                                      cvbridge_img0->image,
                                      cvbridge_img1->image,
                                      newkf,
                                      reset_cmd);
        frame_pub->pubFramePtsPoseT_c_w(this->cam_tracker->curr_frame->getValid3dPts(),
                                        this->cam_tracker->curr_frame->T_c_w,
                                        tstamp);    

        vision_path_pub->pubPathT_c_w(this->cam_tracker->curr_frame->T_c_w,tstamp);

        cvtColor(cam_tracker->curr_frame->img0,img0_vis,CV_GRAY2BGR);
        // cvtColor(cam_tracker->curr_frame->img1,img1_vis,CV_GRAY2BGR);
        drawFrame(img0_vis,*this->cam_tracker->curr_frame,1,11);

        sensor_msgs::ImagePtr img0_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img0_vis).toImageMsg();
        // sensor_msgs::ImagePtr img1_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img1_vis).toImageMsg();
        img0_pub.publish(img0_msg);
        // img1_pub.publish(img1_msg);
        
        if(enable_output_file)
        {
            frame_counter ++;
            SE3 T_w_c = this->cam_tracker->curr_frame->T_c_w.inverse();
            Vector3d t = T_w_c.translation();
            Quaterniond q = T_w_c.unit_quaternion();
            Mat3x3 R_ = q.toRotationMatrix();
            // if it is kitti test (2D map), remove Z
            fd.open(output_file_path.c_str(),ios::app);
            fd << setprecision(6) << R_(0,0) << " " << R_(0,1) << " " << R_(0,2) << " " <<  t[0] << " ";
            fd << setprecision(6) << R_(1,0) << " " << R_(1,1) << " " << R_(1,2) << " " <<  t[1] << " ";
            fd << setprecision(6) << R_(2,0) << " " << R_(2,1) << " " << R_(2,2) << " " <<  t[2] << std::endl;
            // fd << setprecision(6) << t[0] << " " << t[1] << " " << t[2] << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
            fd.close();
        }
    }//image_input_callback(const sensor_msgs::ImageConstPtr & imgPtr, const sensor_msgs::ImageConstPtr & depthImgPtr)
};//class TrackingNodeletClass
}//namespace sopvo_ns

PLUGINLIB_EXPORT_CLASS(sopvo_ns::TrackingNodeletClass, nodelet::Nodelet)



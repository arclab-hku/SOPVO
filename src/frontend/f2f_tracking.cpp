#include "include/f2f_tracking.h"
#include <chrono>
#include <ros/ros.h>

using namespace std::chrono;
using namespace cv;

void F2FTracking::init(std::string configPath, const int w_in, const int h_in, const int w_out, const int h_out,
                       const Mat c0_cameraMatrix_in, const Mat c0_distCoeffs_in,
                       const SE3 T_i_c0_in,
                       const TYPEOFCAMERA cam_type_in,
                       const double cam_scale_in,
                       const Mat c1_cameraMatrix_in, const Mat c1_distCoeffs_in,
                       const SE3 T_c0_c1,
                       const SE3 T_init)
{
    cv::FileStorage fsSettings(configPath, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        fsSettings.release();
        while(true)
        {
            ros::Duration(1).sleep();
            ROS_ERROR_STREAM("Wrong path for VO parameters input...");
            // std::cout << "ERROR: Wrong path to settings, use default parameters..." << std::endl;
        }
    }
    else
    {
        MINIMUM_KEYPOINTS = fsSettings["sopvo.minimumKeypoint"];
        MAXIMUM_T_ERROR = fsSettings["sopvo.maximumKeyframeShift"];
        sosEnableFlag = true;
        int temp_value = fsSettings["sopvo.sosEnableFlag"];
        if (temp_value == 0)
        {
            sosEnableFlag = false;
        }
        sos_alpha = fsSettings["sopvo.sosAlphaForR"];
        sos_beta = fsSettings["sopvo.sosBetaForT"];
        sop_max_iter = fsSettings["sopvo.maxIter"];
        reprojectionErrorPessimistic = fsSettings["sopvo.reprojectionErrorPessimistic"];
        reprojectionErrorOptimistic = fsSettings["sopvo.reprojectionErrorOptimistic"];
        point_learning_rate = fsSettings["sopvo.pointLearningRate"];
        point_difference_threshold = fsSettings["sopvo.pointDifferenceThreshold"];
        backendEnableFlag = false;
        temp_value = fsSettings["sopvo.backendEnable"];
        if (temp_value != 0)
        {
            backendEnableFlag = true;
        }
        BAenableFlag = false;
        temp_value = fsSettings["sopvo.BAenable"];
        if (temp_value != 0)
        {
            backendEnableFlag = true;
        }
        add_new_keypoints = false;
        temp_value = fsSettings["sopvo.addNewKeypoint"];
        if (temp_value != 0)
        {
            add_new_keypoints = true;
        }
        add_new_keypoints_once = false;
        grid_w = fsSettings["feature.gridW"];
        grid_h = fsSettings["feature.gridH"];
        boundarySize = fsSettings["feature.boundaryBoxSize"];
        nFeatures = fsSettings["feature.nFeatures"];
    }
    fsSettings.release();
    
    this->feature_dem   = new FeatureDEM(w_out,h_out,grid_w,grid_h,nFeatures,boundarySize);
    this->lkorb_tracker = new LKORBTracking(w_out,h_out);

    curr_frame = std::make_shared<CameraFrame>();
    last_frame = std::make_shared<CameraFrame>();
    last_keyframe = std::make_shared<CameraFrame>();
    myOrientationPri = std::make_shared<OrientationPri>();
    curr_frame->height = last_frame->height = last_keyframe->height = h_out;
    curr_frame->width = last_frame->width = last_keyframe->width = w_out;

    SE3 T_c1_c0 = T_c0_c1.inverse();

    this->cam_type = cam_type_in;
    
    K0 = c0_cameraMatrix_in;
    D0 = c0_distCoeffs_in;
    K1 = c1_cameraMatrix_in;
    D1 = c1_distCoeffs_in;
    
    Mat3x3 R_ = T_c1_c0.rotation_matrix();
    Vec3   T_ = T_c1_c0.translation();
    cv::Mat R__ = (cv::Mat1d(3, 3) << R_(0,0), R_(0,1), R_(0,2),
                    R_(1,0), R_(1,1), R_(1,2),
                    R_(2,0), R_(2,1), R_(2,2));
    cv::Mat T__ = (cv::Mat1d(3, 1) << T_(0), T_(1), T_(2));
    cv::Mat R0,R1,P0,P1,Q;
    cv::stereoRectify(K0,D0,K1,D1,cv::Size(w_in,h_in),R__,T__,
                        R0,R1,P0,P1,Q,
                        CALIB_ZERO_DISPARITY,0,cv::Size(w_out,h_out));

    
    switch(this->cam_type)
    {
        case STEREO_KITTI:
            cv::initUndistortRectifyMap(K0,D0,R0,P0,cv::Size(w_out,h_out),CV_32F,c0_RM[0],c0_RM[1]);
            cv::initUndistortRectifyMap(K1,D1,R1,P1,cv::Size(w_out,h_out),CV_32F,c1_RM[0],c1_RM[1]);
            K0_rect = P0.rowRange(0,3).colRange(0,3);
            K1_rect = P1.rowRange(0,3).colRange(0,3);
            D1_rect = D0_rect = (cv::Mat1d(4, 1) << 0,0,0,0);
            break;
        case STEREO_EuRoC_MAV:
            cv::initUndistortRectifyMap(K0,D0,R0,P0,cv::Size(w_out,h_out),CV_32F,c0_RM[0],c0_RM[1]);
            cv::initUndistortRectifyMap(K1,D1,R1,P1,cv::Size(w_out,h_out),CV_32F,c1_RM[0],c1_RM[1]);
            K0_rect = P0.rowRange(0,3).colRange(0,3);
            K1_rect = P1.rowRange(0,3).colRange(0,3);
            D1_rect = D0_rect = (cv::Mat1d(4, 1) << 0,0,0,0);
            break;
        case Realsense_T265:
            cv::fisheye::initUndistortRectifyMap(K0, D0, R0, P0, cv::Size(w_out,h_out), CV_32FC1, c0_RM[0], c0_RM[1]);
            cv::fisheye::initUndistortRectifyMap(K1, D1, R1, P1, cv::Size(w_out,h_out), CV_32FC1, c1_RM[0], c1_RM[1]);
            K0_rect = (cv::Mat_<double>(3,3) << 169.519821, 0.000000, 151.870868, 0.000000, 167.917718, 132.600913, 0.000000, 0.000000, 1.000000);
            D0_rect = (cv::Mat1d(4, 1) << -0.003884, 0.007336, -0.002283, -0.001952);
            K1_rect = (cv::Mat_<double>(3,3) << 169.392025, 0.000000, 150.805310, 0.000000, 167.969904, 137.542866, 0.000000, 0.000000, 1.000000);
            D1_rect = (cv::Mat1d(4, 1) << 0.000446, 0.006281, 0.007351, -0.004575);
            cv::initUndistortRectifyMap(K0_rect, D0_rect, R0, P0, cv::Size(w_out,h_out), CV_32FC1, c0_RM_rect[0], c0_RM_rect[1]);
            cv::initUndistortRectifyMap(K1_rect, D1_rect, R1, P1, cv::Size(w_out,h_out), CV_32FC1, c1_RM_rect[0], c1_RM_rect[1]);
            K0_rect = P0.rowRange(0,3).colRange(0,3);
            K1_rect = P1.rowRange(0,3).colRange(0,3);
            D1_rect = D0_rect = (cv::Mat1d(4, 1) << 0,0,0,0);
            break;
        // default:
        //     cv::initUndistortRectifyMap(K0,D0,R0,P0,cv::Size(w_out,h_out),CV_32F,c0_RM[0],c0_RM[1]);
        //     cv::initUndistortRectifyMap(K1,D1,R1,P1,cv::Size(w_out,h_out),CV_32F,c1_RM[0],c1_RM[1]);
        //     K0_rect = P0.rowRange(0,3).colRange(0,3);
        //     K1_rect = P1.rowRange(0,3).colRange(0,3);
        //     D1_rect = D0_rect = (cv::Mat1d(4, 1) << 0,0,0,0);
        //     break;
    }
    
    StereoCamera dc;
    Mat3x4 P0_,P1_;
    P0_(0,0) = P0.at<double>(0,0);
    P0_(1,1) = P0.at<double>(1,1);
    P0_(0,2) = P0.at<double>(0,2);
    P0_(1,2) = P0.at<double>(1,2);
    P0_(2,2) = P0.at<double>(2,2);
    P0_(0,3) = P0.at<double>(0,3);
    P0_(1,3) = P0.at<double>(1,3);
    P0_(2,3) = P0.at<double>(2,3);

    P1_(0,0) = P1.at<double>(0,0);
    P1_(1,1) = P1.at<double>(1,1);
    P1_(0,2) = P1.at<double>(0,2);
    P1_(1,2) = P1.at<double>(1,2);
    P1_(2,2) = P1.at<double>(2,2);
    P1_(0,3) = P1.at<double>(0,3);
    P1_(1,3) = P1.at<double>(1,3);
    P1_(2,3) = P1.at<double>(2,3);

    dc.setSteroCamInfo(K0_rect, D0_rect, P0_,
                            K1_rect, D1_rect, P1_,
                            T_c0_c1);
    curr_frame->d_camera = last_frame->d_camera = last_keyframe->d_camera = dc;
    this->frameCount = 0;
    this->vo_tracking_state = Init_sopvo;
    this->has_localmap_feedback = false;
    // initial pose
    // Mat3x3 R_w_c;
    // 0  0  1
    //-1  0  0
    // 0 -1  0
    // R_w_c << 0, 0, 1, -1, 0, 0, 0,-1, 0;
    T_c_w_last_keyframe = T_init.inverse();
    T_c_w_last_frame = T_c_w_last_keyframe;
    myOrientationPri->init(T_c1_c0, K1, sos_alpha, sos_beta, sop_max_iter);
}

bool F2FTracking::init_frame()
{
    bool init_succeed=false;
    curr_frame->T_c_w = T_c_w_last_frame;
    vector<Vec2> pts2d_img0, pts2d_img1;
    std::vector<cv::Point2f> pts0, pts1;
    // vector<cv::Mat>  descriptors;
    this->feature_dem->fb_tracking(curr_frame->img0, curr_frame->img1, pts0, pts1);
    // extract orb descriptors
    vector<cv::KeyPoint> tmpKPs;
    // cv::Mat tmpDescriptors;
    cv::KeyPoint::convert(pts0,tmpKPs);
    cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();
    // extractor->compute(curr_frame->img0, tmpKPs, tmpDescriptors); // this function removes some keypoints without telling their index, which is not good... 
    // cout << " forward-backward tracking finished, keep " << tmpKPs.size() << " keypoints for this keyframe" << endl;
    // descriptors.clear();
    // descriptors_to_vMat(tmpDescriptors, descriptors);
    pts2d_img0 = vcvP2f_2_vVec2(pts0);
    pts2d_img1 = vcvP2f_2_vVec2(pts1);

    int p3d_counter = 0;

    for(size_t i=0; i<tmpKPs.size(); i++)
    {
        cv::Point2f point_check = cv::Point2f(tmpKPs.at(i).pt.x, tmpKPs.at(i).pt.y);
        std::vector<cv::Point2f>::iterator it = std::find(pts0.begin(), pts0.end(), point_check);
        size_t idx = std::distance(pts0.begin(),it);
        Vec3 pt3d_c;
        bool valid_3d_point = Triangulation::trignaulationPtFromStereo(pts2d_img0.at(idx), pts2d_img1.at(idx), curr_frame->d_camera.P0_, curr_frame->d_camera.P1_, pt3d_c);
        if(valid_3d_point)
        {
            Vec2 reProj = curr_frame->d_camera.camera2pixel(pt3d_c);
            Vec2 err = pts2d_img0.at(idx)-reProj;
            if (err.norm() < reprojectionErrorOptimistic)
            {
                curr_frame->landmarks.push_back(LandMarkInFrame(pts2d_img0.at(idx), pt3d_c, true, curr_frame->T_c_w));
                p3d_counter ++;
            }
            else
            {
                curr_frame->landmarks.push_back(LandMarkInFrame(pts2d_img0.at(idx), Vec3(0,0,0), false, curr_frame->T_c_w));
            }
        }
        else
        {
            curr_frame->landmarks.push_back(LandMarkInFrame(pts2d_img0.at(idx), Vec3(0,0,0), false, curr_frame->T_c_w));
        }
    }
    cout << "frame " << frameCount;
    cout << ": point could initialized, " << p3d_counter << " points have been generated. " << endl;

    if (p3d_counter < 2*MINIMUM_KEYPOINTS)
    {
        add_new_keypoints_once = true;
    }
    else
    {
        add_new_keypoints_once = false;
    }

    if(curr_frame->validLMCount() > MINIMUM_KEYPOINTS)
    {
        ID_POSE tmp;
        tmp.frame_id = curr_frame->frame_id;
        tmp.T_c_w = curr_frame->T_c_w;
        T_c_w_last_keyframe = curr_frame->T_c_w;
        T_c_w_last_frame = T_c_w_last_keyframe;
        pose_records.push_back(tmp);
        init_succeed = true;
    }
    return init_succeed;
}

bool F2FTracking::reset_keyframe()
{
    bool init_succeed=false;
    last_frame->landmarks.clear();
    vector<Vec2> pts2d_img0, pts2d_img1;
    std::vector<cv::Point2f> pts0, pts1;
    // vector<cv::Mat>  descriptors;
    this->feature_dem->fb_tracking(last_frame->img0, last_frame->img1, pts0, pts1);
    // extract orb descriptors
    vector<cv::KeyPoint> tmpKPs;
    // cv::Mat tmpDescriptors;
    cv::KeyPoint::convert(pts0,tmpKPs);
    cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();
    // extractor->compute(last_frame->img0, tmpKPs, tmpDescriptors); // this function removes some keypoints without telling their index, which is not good... 
    // cout << " forward-backward tracking finished, keep " << tmpKPs.size() << " keypoints for this keyframe" << endl;
    // descriptors.clear();
    // descriptors_to_vMat(tmpDescriptors, descriptors);
    pts2d_img0 = vcvP2f_2_vVec2(pts0);
    pts2d_img1 = vcvP2f_2_vVec2(pts1);

    int p3d_counter = 0;

    for(size_t i=0; i<tmpKPs.size(); i++)
    {
        cv::Point2f point_check = cv::Point2f(tmpKPs.at(i).pt.x, tmpKPs.at(i).pt.y);
        std::vector<cv::Point2f>::iterator it = std::find(pts0.begin(), pts0.end(), point_check);
        size_t idx = std::distance(pts0.begin(),it);
        Vec3 pt3d_c;
        bool valid_3d_point = Triangulation::trignaulationPtFromStereo(pts2d_img0.at(idx), pts2d_img1.at(idx), last_frame->d_camera.P0_, last_frame->d_camera.P1_, pt3d_c);
        if(valid_3d_point)
        {
            Vec2 reProj = curr_frame->d_camera.camera2pixel(pt3d_c);
            Vec2 err = pts2d_img0.at(idx)-reProj;
            if (err.norm() < reprojectionErrorOptimistic)
            {
                last_frame->landmarks.push_back(LandMarkInFrame(pts2d_img0.at(idx), pt3d_c, true, last_frame->T_c_w));
                p3d_counter ++;  
            }
            else
            {
                last_frame->landmarks.push_back(LandMarkInFrame(pts2d_img0.at(idx), Vec3(0,0,0), false, last_frame->T_c_w));
            }                             
        }
        else
        {
            last_frame->landmarks.push_back(LandMarkInFrame(pts2d_img0.at(idx), Vec3(0,0,0), false, last_frame->T_c_w));
        }
    }

    if (p3d_counter < 2*MINIMUM_KEYPOINTS)
    {
        add_new_keypoints_once = true;
    }
    else
    {
        add_new_keypoints_once = false;
    }

    cout << "frame " << frameCount;
    cout << ": point could initialized, " << p3d_counter << " points have been generated. " << endl;

    ID_POSE tmp;
    tmp.frame_id = last_frame->frame_id;
    tmp.T_c_w = last_frame->T_c_w;
    T_c_w_last_frame = last_frame->T_c_w;
    pose_records.push_back(tmp);

    if(last_frame->validLMCount() > MINIMUM_KEYPOINTS)
    {
        T_c_w_last_keyframe = last_frame->T_c_w;
        init_succeed = true;
    }
    return init_succeed;
}

bool F2FTracking::pnp_from_lastframe()
{
    bool need_reset = false;
    vector<cv::Point2f> p2d;
    vector<cv::Point3f> p3d;
    curr_frame->getValid2d3dPair_cvPf(p2d,p3d);
    if(p2d.size() < MINIMUM_KEYPOINTS)
    {
        cout << "insufficient keypoints, reset keyframe..." << endl;
        need_reset = true;
    }
    else
    {
        cv::Mat r_ = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat t_ = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat r_old = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat t_old = cv::Mat::zeros(3, 1, CV_64FC1);
        SE3_to_rvec_tvec(last_frame->T_c_w, r_ , t_ ); 
        SE3_to_rvec_tvec(last_frame->T_c_w, r_old , t_old );
        cv::Mat inliers;
        solvePnPRansac(p3d,p2d,K0_rect,D0_rect,
                    r_,t_,false,100,3.0,0.99,inliers,cv::SOLVEPNP_ITERATIVE);
        curr_frame->T_c_w = SE3_from_rvec_tvec(r_,t_); 
        // abnormal motion detection
        SE3 T_diff_key_curr = last_frame->T_c_w*(curr_frame->T_c_w.inverse());
        Vec3 t=T_diff_key_curr.translation();
        Vec3 r=T_diff_key_curr.so3().log();
        double t_norm = fabs(t[0]) + fabs(t[1]) + fabs(t[2]);
        double r_norm = fabs(r[0]) + fabs(r[1]) + fabs(r[2]);
        if (t_norm > MAXIMUM_T_ERROR || r_norm > MAXIMUM_T_ERROR)
        {
            cout << "abnormal motion, reset keyframe..." << endl;
            need_reset = true;
        }
        else
        {
            std::vector<uchar> status;
            for (int i = 0; i < (int)p2d.size(); i++)
                status.push_back(0);
            for( int i = 0; i < inliers.rows; i++)
            {
                int n = inliers.at<int>(i);
                status[n] = 1;
            }
            curr_frame->updateLMState(status);
        }
    }
    return need_reset;
}

void F2FTracking::image_feed(const double time,
                             const cv::Mat img0_in,
                             const cv::Mat img1_in,
                             bool &new_keyframe,
                             bool &reset_cmd)
{
    auto start = high_resolution_clock::now();
    new_keyframe = false;
    reset_cmd = false;
    frameCount++;
    // cout << "frame : " << frameCount << endl;
    last_frame.swap(curr_frame);
    curr_frame->clear();
    curr_frame->frame_id = frameCount;
    curr_frame->frame_time = time;
    curr_frame->T_c_w = T_c_w_last_frame;
    switch(this->cam_type) 
    {
        case STEREO_KITTI:
            curr_frame->img0 = img0_in.clone();
            curr_frame->img1 = img1_in.clone();
            break;
        case STEREO_EuRoC_MAV:
            cv::remap(img0_in, curr_frame->img0, c0_RM[0], c0_RM[1], cv::INTER_LINEAR);
            cv::remap(img1_in, curr_frame->img1, c1_RM[0], c1_RM[1], cv::INTER_LINEAR);
            break;
        case Realsense_T265:
            cv::remap(img0_in, curr_frame->img0, c0_RM[0], c0_RM[1], cv::INTER_LINEAR);
            cv::remap(img1_in, curr_frame->img1, c1_RM[0], c1_RM[1], cv::INTER_LINEAR);
            cv::remap(curr_frame->img0, curr_frame->img0, c0_RM_rect[0], c0_RM_rect[1], cv::INTER_LINEAR);
            cv::remap(curr_frame->img1, curr_frame->img1, c1_RM_rect[0], c1_RM_rect[1], cv::INTER_LINEAR);
            break;
        default:
            cv::remap(img0_in, curr_frame->img0, c0_RM[0], c0_RM[1], cv::INTER_LINEAR);
            cv::remap(img1_in, curr_frame->img1, c1_RM[0], c1_RM[1], cv::INTER_LINEAR);
            break;
    }

    switch(vo_tracking_state)
    {
        case Init_sopvo:
        {
            if(this->init_frame())
            {
                // new_keyframe = true;
                vo_tracking_state = Run_sopvo;
                cout << "keyframe initialization success..." << endl;
            }
            break;
        }
        case Run_sopvo:
        {
            /* F2F Workflow
                STEP0: for loop closure (not yet available)
                STEP1: Keypoint tracking and PnP
                STEP2: 3D points refinement
                STEP3: Update keyframe and record pose
                        */
            //STEP1:
            vector<Vec2> lm2d_from,lm2d_to,outlier_tracking;
            this->lkorb_tracker->tracking(*last_frame,
                                        *curr_frame,
                                        lm2d_from,
                                        lm2d_to,
                                        outlier_tracking);
            bool need_reset_keyframe = this->pnp_from_lastframe();
            if(need_reset_keyframe)
            {
                // reset_cmd = true;
                if (this->reset_keyframe())
                {
                    lm2d_from.clear();
                    lm2d_to.clear();
                    outlier_tracking.clear();
                    curr_frame->landmarks.clear(); 
                    this->lkorb_tracker->tracking(*last_frame,
                                                *curr_frame,
                                                lm2d_from,
                                                lm2d_to,
                                                outlier_tracking);
                    if(this->pnp_from_lastframe())
                    {
                        cout << "fail to reset keyframe from last frame..." << endl;
                        vo_tracking_state = Reset_sopvo;
                        break;
                    }
                }
                else
                {
                    cout << "fail to reset keyframe from last frame..." << endl;
                    vo_tracking_state = Reset_sopvo;
                    break;
                }
            }
            //STEP2:
            if(BAenableFlag)
            {
                try
                {
                    OptimizeInFrame::optimize(*curr_frame); // BA process, currently not available
                }
                catch(const std::exception& e)
                {
                    cout << "BA failed..." << endl;
                }
            }
            
            try
            {
                if ((add_new_keypoints || add_new_keypoints_once) && curr_frame->validLMCount() < 2*MINIMUM_KEYPOINTS)
                {
                    vector<Vec2> newKeyPts;
                    int newPtsCount;
                    this->feature_dem->redetect(curr_frame->img0,
                                                curr_frame->get2dPtsVec(),
                                                newKeyPts,newPtsCount);
                    for(size_t i=0; i<newKeyPts.size(); i++)
                    {
                        curr_frame->landmarks.push_back(LandMarkInFrame(newKeyPts.at(i),
                                                                        Vec3(0,0,0),
                                                                        false,
                                                                        curr_frame->T_c_w));
                    }
                }
                else
                {
                    add_new_keypoints_once = false;
                }
                

                vector<Vec3> pts3d_c_cam_measure;
                vector<Vec2> pts2d_img0, pts2d_img1;
                vector<bool> cam_measure_mask;
                int op_rejected_counter = 0;
                int op_evaluate_counter = 0;
                curr_frame->recover3DPts_c_FromStereo(pts3d_c_cam_measure, pts2d_img0, pts2d_img1, cam_measure_mask);
                for(size_t i=0; i<curr_frame->landmarks.size(); i++)
                {
                    if (cam_measure_mask.at(i)==false) continue;
                    Vec3 lm_c_measure = pts3d_c_cam_measure.at(i);
                    if(curr_frame->landmarks.at(i).hasDepthInf())
                    {
                        //transfor to Camera frame
                        Vec3 lm_c = StereoCamera::world2cameraT_c_w(curr_frame->landmarks.at(i).lm_3d_w,curr_frame->T_c_w);
                        Vec3 lm_c_update;
                        Vec3 dist_3d = lm_c - lm_c_measure;
                        Vec2 reProj = curr_frame->d_camera.camera2pixel(lm_c);
                        Vec2 reProj_measure = curr_frame->d_camera.camera2pixel(lm_c_measure);
                        Vec2 err = pts2d_img0.at(i)-reProj;
                        Vec2 err_measure = pts2d_img0.at(i)-reProj_measure;
                        if (err.norm() < reprojectionErrorPessimistic && err_measure.norm() < reprojectionErrorPessimistic)
                        {
                            if (dist_3d.norm() < point_difference_threshold)
                            {
                                // lm_c_update = 0.9*lm_c + 0.1*lm_c_measure;
                                lm_c_update = (1 - point_learning_rate)*lm_c + point_learning_rate*lm_c_measure;
                            }
                            else
                            {
                                lm_c_update = lm_c;
                            }
                            curr_frame->landmarks.at(i).lm_3d_c = lm_c_update;
                            curr_frame->landmarks.at(i).lm_3d_w = StereoCamera::camera2worldT_c_w(lm_c_update, curr_frame->T_c_w);
                            curr_frame->landmarks.at(i).lmState = LMSTATE_NORMAL;
                            curr_frame->landmarks.at(i).lm_tracking_state = LM_TRACKING_INLIER;
                        }
                        else if(err.norm() < reprojectionErrorOptimistic && err_measure.norm() < reprojectionErrorOptimistic && sosEnableFlag) // stereo orientation prior check
                        {
                            op_evaluate_counter ++;
                            Vec3 lm_c_update = (1 - point_learning_rate)*lm_c + point_learning_rate*lm_c_measure;
                            myOrientationPri->getPair(pts2d_img1.at(i), lm_c_update);
                            if(myOrientationPri->sosFeasibilityCheck())
                            {
                                curr_frame->landmarks.at(i).lm_3d_c = lm_c_update;
                                curr_frame->landmarks.at(i).lm_3d_w = StereoCamera::camera2worldT_c_w(lm_c_update,curr_frame->T_c_w);
                                curr_frame->landmarks.at(i).lmState = LMSTATE_NORMAL;
                                curr_frame->landmarks.at(i).lm_tracking_state = LM_TRACKING_INLIER;
                            }
                            else
                            {
                                curr_frame->landmarks.at(i).lm_tracking_state = LM_TRACKING_OUTLIER;
                                op_rejected_counter ++;
                            }
                            myOrientationPri->clearPair();
                        }
                        else
                        {
                            curr_frame->landmarks.at(i).lm_tracking_state = LM_TRACKING_OUTLIER;
                        }
                    }
                    else//Do not have position
                    {
                        Vec3 pt3d_w = StereoCamera::camera2worldT_c_w(lm_c_measure,curr_frame->T_c_w);
                        curr_frame->landmarks.at(i).lm_3d_c = lm_c_measure;
                        curr_frame->landmarks.at(i).lm_3d_w = pt3d_w;
                        curr_frame->landmarks.at(i).lmState = LMSTATE_NORMAL;
                        curr_frame->landmarks.at(i).lm_has_3d = true;
                        curr_frame->landmarks.at(i).lm_tracking_state = LM_TRACKING_INLIER;
                    }
                }
                curr_frame->eraseReprjOutlier();
                // cout << "orientation prior:" << op_rejected_counter << " out of " << op_evaluate_counter << " points are rejected..." << endl;
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                // cout << "BA process failed, reset keyframe...";
                // reset_cmd = true;
                if (this->reset_keyframe())
                {
                    lm2d_from.clear();
                    lm2d_to.clear();
                    outlier_tracking.clear();
                    curr_frame->landmarks.clear(); 
                    this->lkorb_tracker->tracking(*last_frame,
                                                *curr_frame,
                                                lm2d_from,
                                                lm2d_to,
                                                outlier_tracking);
                    if(this->pnp_from_lastframe())
                    {
                        cout << "fail to reset keyframe from last frame..." << endl;
                        vo_tracking_state = Reset_sopvo;
                        break;
                    }
                }
                else
                {
                    cout << "fail to reset keyframe from last frame..." << endl;
                    vo_tracking_state = Reset_sopvo;
                    break;
                }
            }
            //STEP3:
            if(curr_frame->validLMCount() < MINIMUM_KEYPOINTS)
            {
                cout << "insufficient landmarks, reset keyframe..." << endl;
                // reset_cmd = true;
                this->reset_keyframe();
                lm2d_from.clear();
                lm2d_to.clear();
                outlier_tracking.clear();
                curr_frame->landmarks.clear(); 
                this->lkorb_tracker->tracking(*last_frame,
                                            *curr_frame,
                                            lm2d_from,
                                            lm2d_to,
                                            outlier_tracking);
                if(this->pnp_from_lastframe())
                {
                    cout << "fail to reset keyframe from last frame..." << endl;
                    vo_tracking_state = Reset_sopvo;
                    break;
                }
            }

            // SE3 T_diff = T_c_w_last_frame.inverse()*curr_frame->T_c_w;
            // Vec3 t = T_diff.translation();
            // Vec3 r = T_diff.so3().log();
            SE3 T_curr_last = curr_frame->T_c_w*(T_c_w_last_frame.inverse());
            Vec3 t = T_curr_last.translation();
            Vec3 r = T_curr_last.so3().log();
            double t_norm = fabs(t[0]) + fabs(t[1]) + fabs(t[2]);
            double r_norm = fabs(r[0]) + fabs(r[1]) + fabs(r[2]);
            // cout << "t_norm = " << t_norm << " r_norm = " << r_norm << endl;
            // if it is kitti test (2D map), remove the roll and pitch motion
            // if (this->cam_type == STEREO_KITTI)
            // {
            //     // Quaterniond q_diff = T_diff.unit_quaternion();
            //     // Mat3x3 R_diff = q_diff.toRotationMatrix();
            //     // Eigen::Vector3d euler = R_diff.eulerAngles(2, 1, 0);//roll pitch yaw
            //     // Mat3x3 R_rp = euler2RotationMatrix(euler[0],euler[1],0);
            //     // // cout << "euler angle " << euler.transpose() << endl;
            // }
            if(t_norm < MAXIMUM_T_ERROR && r_norm < MAXIMUM_T_ERROR)
            {
                // new_keyframe = true;
                T_c_w_last_frame = curr_frame->T_c_w;
            }

            SE3 T_diff_key_curr = T_c_w_last_keyframe*(curr_frame->T_c_w.inverse());
            t=T_diff_key_curr.translation();
            r=T_diff_key_curr.so3().log();
            t_norm = fabs(t[0]) + fabs(t[1]) + fabs(t[2]);
            r_norm = fabs(r[0]) + fabs(r[1]) + fabs(r[2]);
            if(t_norm > 5*MAXIMUM_T_ERROR || r_norm > 2*MAXIMUM_T_ERROR)
            {
                cout << "reset keyframe..." << endl;
                // reset_cmd = true;
                this->reset_keyframe();
                lm2d_from.clear();
                lm2d_to.clear();
                outlier_tracking.clear();
                curr_frame->landmarks.clear(); 
                this->lkorb_tracker->tracking(*last_frame,
                                            *curr_frame,
                                            lm2d_from,
                                            lm2d_to,
                                            outlier_tracking);
                if(this->pnp_from_lastframe())
                {
                    cout << "fail to reset keyframe from last frame..." << endl;
                    vo_tracking_state = Reset_sopvo;
                    break;
                }
            }

            ID_POSE tmp;
            tmp.frame_id = curr_frame->frame_id;
            tmp.T_c_w = T_c_w_last_frame;
            pose_records.push_back(tmp);

            break;
        }//end of state: Tracking
        case Reset_sopvo:
        {
            ID_POSE tmp;
            tmp.frame_id = curr_frame->frame_id;
            tmp.T_c_w = T_c_w_last_frame;
            pose_records.push_back(tmp);
            cout << "please get the current pose from pixhawk EKF local pose and reinit the keyframe..." << endl;
            if(this->init_frame())
            {
                // reset_cmd = true;
                // new_keyframe = true;
                vo_tracking_state = Run_sopvo;
                cout << "keyframe initialization success..." << endl;
            }
            break;
        }//end of state: TrackingFail
    }//end of state machine

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    curr_frame->solving_time = (duration.count()/1000.0);
}

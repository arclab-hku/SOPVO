#ifndef F2F_TRACKING
#define F2F_TRACKING

#include "include/feature_dem.h"
#include "include/lkorb_tracking.h"
#include "include/camera_frame.h"
#include "include/orientation_prior.h"
#include "include/optimize_in_frame.h"

enum TRACKINGSTATE{Init_sopvo,
                   Run_sopvo,
                   Reset_sopvo};

struct ID_POSE {
    int    frame_id;
    SE3    T_c_w;
};

class F2FTracking
{
public:
    enum TYPEOFCAMERA cam_type;
    //Modules
    FeatureDEM         *feature_dem;
    LKORBTracking      *lkorb_tracker;

    //states:
    bool has_localmap_feedback;
    int  frameCount;
    enum TRACKINGSTATE   vo_tracking_state;

    //varialbe
    cv::Mat K0,D0,K1,D1;
    cv::Mat K0_rect;//cam0 rectified cameraMatrix;
    cv::Mat D0_rect;//cam0 rectified distCoeffs;
    cv::Mat K1_rect;//cam1 rectified cameraMatrix;
    cv::Mat D1_rect;//cam1 rectified distCoeffs;
    cv::Mat c0_RM[2];
    cv::Mat c1_RM[2];
    cv::Mat c0_RM_rect[2];
    cv::Mat c1_RM_rect[2];

    SE3 T_c_w_last_keyframe;
    SE3 T_c_w_last_frame;
    SE3 T_c_w_fusion_feedback;
    bool fusion_pose_update;
    deque<ID_POSE> pose_records;
    CameraFrame::Ptr curr_frame,last_frame,last_keyframe;
    OrientationPri::Ptr myOrientationPri;
    int sop_max_iter;

    void image_feed(const double time,
                    const cv::Mat img0_in,
                    const cv::Mat img1_in,
                    bool &new_keyframe,
                    bool &reset_cmd);

    void init(std::string configPath,
              const int w_in, const int h_in, const int w_out, const int h_out,
              const cv::Mat c0_cameraMatrix_in,
              const cv::Mat c0_distCoeffs_in,
              const SE3 T_i_c0_in,
              const TYPEOFCAMERA cam_type_in=STEREO_EuRoC_MAV,
              const double cam_scale_in=1000.0,
              const cv::Mat c1_cameraMatrix_in = cv::Mat1d(3, 3),
              const cv::Mat c1_distCoeffs_in = cv::Mat1d(4, 1),
              const SE3 T_c0_c1=SE3(),
              const SE3 T_init=SE3());

private:
    int MINIMUM_KEYPOINTS;
    int MAXIMUM_T_ERROR;
    int grid_w;
    int grid_h;
    int nFeatures;
    int boundarySize;
    bool backendEnableFlag;
    bool BAenableFlag;
    bool sosEnableFlag;
    bool add_new_keypoints;
    bool add_new_keypoints_once;
    double sos_alpha;
    double sos_beta;
    double reprojectionErrorPessimistic;
    double reprojectionErrorOptimistic;
    double point_learning_rate;
    double point_difference_threshold;
    bool init_frame(void);
    bool reset_keyframe(void);
    void landmark_updating(void);
    bool pnp_from_lastframe(void);
};//class F2FTracking

#endif

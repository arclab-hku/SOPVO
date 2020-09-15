#ifndef STEREOCAMERA_H_
#define STEREOCAMERA_H_

#include "include/common.h"


enum TYPEOFCAMERA{STEREO_KITTI,
                  STEREO_EuRoC_MAV,
                  Realsense_T265};
//Support Stero Camera
//fx,fy,cx,cy are rectified cam para

class StereoCamera
{
public:
    typedef std::shared_ptr<StereoCamera> Ptr;
    //parameter
    enum TYPEOFCAMERA cam_type;
    double cam0_fx, cam0_fy, cam0_cx, cam0_cy;
    double cam_scale_factor;
    Mat3x3 K0_;
    Mat3x4 P0_;
    cv::Mat K0,D0,P0;


    StereoCamera();
    static Vec3 world2cameraT_c_w( const Vec3& p_w, const SE3& T_c_w );
    static Vec3 camera2worldT_c_w( const Vec3& p_c, const SE3& T_c_w );
    static Vec2 camera2pixel( const Vec3& p_c,
                              const double fx, const double fy,
                              const double cx, const double cy);
    static Vec3 pixel2camera( const Vec2& p_p,
                              const double fx, const double fy,
                              const double cx, const double cy,
                              double depth=1 );

    Vec2 camera2pixel( const Vec3& p_c );
    Vec3 pixel2camera( const Vec2& p_p,
                       double depth=1 );

    Vec3 pixel2worldT_c_w( const Vec2& p_p, const SE3& T_c_w, double depth=1 );
    Vec2 world2pixelT_c_w( const Vec3& p_w, const SE3& T_c_w );

    double cam1_fx, cam1_fy, cam1_cx, cam1_cy;
    Mat3x3 K1_;
    Mat3x4 P1_;
    cv::Mat K1,D1,P1;
    SE3    T_cam0_cam1;
    SE3    T_cam1_cam0;
    void setSteroCamInfo(const cv::Mat K0_in, const cv::Mat D0_in, const Mat3x4 P0_in,
                         const cv::Mat K1_in, const cv::Mat D1_in, const Mat3x4 P1_in,
                         const SE3 T_c0_c1_in);

private:

};

#endif // STEREOCAMERA_H_

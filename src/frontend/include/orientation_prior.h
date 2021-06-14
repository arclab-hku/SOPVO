#ifndef ORIENTATIONPRIOR_H
#define ORIENTATIONPRIOR_H

#include <include/common.h>
#include "include/camera_frame.h"
#include <iostream>
#include <utility>
#include <sdpa_call.h>

class OrientationPri
{
  private:
    double Xw, Yw, Zw, Uc, Vc; // 3D point in world coordinate system (Xw, Yw, Zw) and 2D point in camera coordinate system (Uc, Vc)
    cv::Mat K_inv;
    cv::Mat var_minmax;
    std::vector<cv::Mat> Qg;
    void cayley_R2v(cv::Mat &R_mat, cv::Mat &I_mat, cv::Mat &cayley_vec, cv::Mat &skew_vec);
    void getQg(void);
    void printVector(double* ele, int dim, char* printFormat, FILE* fpout);
  public:
    int max_iter;
    typedef std::shared_ptr<OrientationPri> Ptr;
    OrientationPri();
    ~OrientationPri();
    void init(SE3 T_c1c0, cv::Mat K, double r_error, double t_error, int max_iter_number);
    void getPair(Vec2 p2d, Vec3 p3d);
    bool sosFeasibilityCheck(void);
    void clearPair(void);
    void testSDPA(void);
    
};

#endif // ORIENTATIONPRIOR_H

#include "include/orientation_prior.h"

using namespace std;

OrientationPri::OrientationPri()
{}

void OrientationPri::init(SE3 T_c1c0, cv::Mat K, double r_error, double t_error, int max_iter_number = 100)
{
    K_inv = K.inv();
    cv::Mat R_ = cv::Mat::zeros(3, 3, CV_64FC1);
    cv::Mat t_ = cv::Mat::zeros(3, 1, CV_64FC1);
    SE3_to_Rt(T_c1c0, R_, t_);

    cv::Mat cayley_v = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat t_new = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat skew_v = cv::Mat::zeros(3, 3, CV_64FC1);
    cv::Mat I_ = cv::Mat::eye(3,3,CV_64FC1);
    cayley_R2v(R_, I_, cayley_v, skew_v);
    t_new = (I_ - skew_v)*t_;
    var_minmax = cv::Mat::zeros(6, 2, CV_64FC1);
    var_minmax.at<double>(0,0) = cayley_v.at<double>(0,0) - r_error;
    var_minmax.at<double>(1,0) = cayley_v.at<double>(1,0) - r_error;
    var_minmax.at<double>(2,0) = cayley_v.at<double>(2,0) - r_error;
    var_minmax.at<double>(0,1) = cayley_v.at<double>(0,0) + r_error;
    var_minmax.at<double>(1,1) = cayley_v.at<double>(1,0) + r_error;
    var_minmax.at<double>(2,1) = cayley_v.at<double>(2,0) + r_error;
    var_minmax.at<double>(3,0) = t_new.at<double>(0,0) - t_error;
    var_minmax.at<double>(4,0) = t_new.at<double>(1,0) - t_error;
    var_minmax.at<double>(5,0) = t_new.at<double>(2,0) - t_error;
    var_minmax.at<double>(3,1) = t_new.at<double>(0,0) + t_error;
    var_minmax.at<double>(4,1) = t_new.at<double>(1,0) + t_error;
    var_minmax.at<double>(5,1) = t_new.at<double>(2,0) + t_error;

    max_iter = max_iter_number;
}

void OrientationPri::getPair(Vec2 p2d, Vec3 p3d)
{
    Xw = p3d[0];
    Yw = p3d[1];
    Zw = p3d[2];

    cv::Mat point2d_img = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat point2d_cam = cv::Mat::zeros(3, 1, CV_64FC1);
    point2d_img.at<double>(0,0) = p2d(0,0);
    point2d_img.at<double>(1,0) = p2d(1,0);
    point2d_img.at<double>(2,0) = 1;
    point2d_cam = K_inv * point2d_img;
    Uc = point2d_cam.at<double>(0,0)/point2d_cam.at<double>(2,0);
    Vc = point2d_cam.at<double>(1,0)/point2d_cam.at<double>(2,0);
    
    getQg();
}

void OrientationPri::clearPair()
{
    Qg.clear();
    Xw = 0;
    Yw = 0;
    Zw = 0;
    Uc = 0;
    Vc = 0;
}

void OrientationPri::cayley_R2v(cv::Mat &R_mat, cv::Mat &I_mat, cv::Mat &cayley_vec, cv::Mat &skew_vec)
{
    cv::Mat temp = R_mat + I_mat;
    skew_vec = (R_mat - I_mat)*temp.inv();
    cayley_vec.at<double>(0,0) = skew_vec.at<double>(2,1);
    cayley_vec.at<double>(1,0) = skew_vec.at<double>(0,2);
    cayley_vec.at<double>(2,0) = skew_vec.at<double>(1,0);
}

void OrientationPri::getQg(void)
{
    Qg.clear();
    int nVar = 6;
    cv::Mat I_ = cv::Mat::eye(7,7,CV_64FC1);
    cv::Mat var_minmax_square = var_minmax.mul(var_minmax);
    double N_sum = 0;
    
    for (int i = 0; i < 6; i++)
    {
        N_sum = N_sum + max(var_minmax_square.at<double>(i,0),var_minmax_square.at<double>(i,1));
    }
    
    for(int idx = 0; idx < 6; idx ++)
    {
        cv::Mat Qtemp = cv::Mat::zeros(7, 7, CV_64FC1);
        double lower_bound = var_minmax.at<double>(idx,0);
        double upper_bound = var_minmax.at<double>(idx,1);
        Qtemp.at<double>(idx,idx) = -1;
        Qtemp.at<double>(idx,nVar) = (lower_bound + upper_bound)/2;
        Qtemp.at<double>(nVar,idx) = (lower_bound + upper_bound)/2;
        Qtemp.at<double>(nVar,nVar) = -lower_bound*upper_bound;
        Qg.push_back(I_ - Qtemp);
    }

    cv::Mat Qtemp = 2*cv::Mat::ones(7, 7, CV_64FC1);
    Qtemp.at<double>(nVar,nVar) = 1 - N_sum;
    Qg.push_back(Qtemp);
}

bool OrientationPri::sosFeasibilityCheck()
{
    bool    infeasible_flag = false;
    SDPA	myLMIs;
    // myLMIs.setDisplay(stdout);
    myLMIs.setParameterType(SDPA::PARAMETER_DEFAULT);
    // myLMIs.printParameters(stdout);
    myLMIs.setParameterMaxIteration(max_iter);
    int mDIM   = 10;
    int nBlock = 1;
    myLMIs.inputConstraintNumber(mDIM);
    myLMIs.inputBlockNumber(nBlock);
    myLMIs.inputBlockSize(1,7);
    myLMIs.inputBlockType(1,SDPA::SDP);
    myLMIs.initializeUpperTriangleSpace();

    myLMIs.inputCVec(1,0);
    myLMIs.inputCVec(2,0);
    myLMIs.inputCVec(3,0);
    myLMIs.inputCVec(4,1);
    myLMIs.inputCVec(5,1);
    myLMIs.inputCVec(6,1);
    myLMIs.inputCVec(7,1);
    myLMIs.inputCVec(8,1);
    myLMIs.inputCVec(9,1);
    myLMIs.inputCVec(10,1);
    // Q1
    myLMIs.inputElement(1, 1, 1, 1, Yw - Zw*Vc);
    myLMIs.inputElement(1, 1, 1, 2, (Zw*Uc - Xw)/2);
    myLMIs.inputElement(1, 1, 1, 3, (Xw*Vc - Yw*Uc)/2);
    myLMIs.inputElement(1, 1, 1, 5, Vc/2);
    myLMIs.inputElement(1, 1, 1, 7, Zw + Yw*Vc);
    myLMIs.inputElement(1, 1, 2, 5, -Uc/2);
    myLMIs.inputElement(1, 1, 2, 7, (-Xw*Vc - Yw*Uc)/2);
    myLMIs.inputElement(1, 1, 3, 6, -Uc/2);
    myLMIs.inputElement(1, 1, 3, 7, (-Xw-Zw*Uc)/2);
    myLMIs.inputElement(1, 1, 5, 7, -1/2);
    myLMIs.inputElement(1, 1, 6, 7, Vc/2);
    myLMIs.inputElement(1, 1, 7, 7, Zw*Vc - Yw);
    // Q2
    myLMIs.inputElement(2, 1, 1, 2, (Yw - Zw*Vc)/2);
    myLMIs.inputElement(2, 1, 1, 4, -Vc/2);
    myLMIs.inputElement(2, 1, 1, 7, (-Xw*Vc - Yw*Uc)/2);
    myLMIs.inputElement(2, 1, 2, 2, Zw*Uc - Xw);
    myLMIs.inputElement(2, 1, 2, 3, (Xw*Vc - Yw*Uc)/2);
    myLMIs.inputElement(2, 1, 2, 4, Uc/2);
    myLMIs.inputElement(2, 1, 2, 6, 1/2);
    myLMIs.inputElement(2, 1, 2, 7, Zw + Xw*Uc);
    myLMIs.inputElement(2, 1, 3, 6, -Vc/2);
    myLMIs.inputElement(2, 1, 3, 7, (-Yw - Zw*Vc)/2);
    myLMIs.inputElement(2, 1, 4, 7, 1/2);
    myLMIs.inputElement(2, 1, 6, 7, -Uc/2);
    myLMIs.inputElement(2, 1, 7, 7, Xw - Zw*Uc);
    // Q3
    myLMIs.inputElement(3, 1, 1, 3, (Yw - Zw*Vc)/2);
    myLMIs.inputElement(3, 1, 1, 4, -1/2);
    myLMIs.inputElement(3, 1, 1, 7, (-Xw - Zw*Uc)/2);
    myLMIs.inputElement(3, 1, 2, 3, (Zw*Uc - Xw)/2);
    myLMIs.inputElement(3, 1, 2, 5, -1/2);
    myLMIs.inputElement(3, 1, 2, 7, (-Yw - Zw*Vc)/2);
    myLMIs.inputElement(3, 1, 3, 3, Xw*Vc - Yw*Uc);
    myLMIs.inputElement(3, 1, 3, 4, Uc/2);
    myLMIs.inputElement(3, 1, 3, 5, Vc/2);
    myLMIs.inputElement(3, 1, 3, 7, Xw*Uc + Yw*Vc);
    myLMIs.inputElement(3, 1, 4, 7, -Vc/2);
    myLMIs.inputElement(3, 1, 5, 7, Uc/2);
    myLMIs.inputElement(3, 1, 7, 7, Yw*Uc - Xw*Uc);
    // Qg
    for(size_t k = 0; k < Qg.size(); k ++)
    {
        cv::Mat Qtemp = Qg.at(k);
        for (int i = 0; i < 7; i++)
        {
            for (int j = i; j < 7; j++)
            {
                if(Qtemp.at<double>(i,j)!=0)
                {
                    myLMIs.inputElement(k+4, 1, i+1, j+1, Qtemp.at<double>(i,j));
                }
            }
            
        }
    }
    myLMIs.initializeUpperTriangle();
    myLMIs.initializeSolve();
    myLMIs.solve();

    if (myLMIs.getIteration() < max_iter)
    {
        double* ele = myLMIs.getResultXVec();
        for (int k=4; k<myLMIs.getConstraintNumber(); k++) 
        {
            if(ele[k] < 0)
            {
                infeasible_flag = true;
                break;
            }
        }
    }
    else
    {
        infeasible_flag = true;
    }
    
    myLMIs.terminate();
    // myLMIs.Delete();

    return infeasible_flag;
}

void OrientationPri::testSDPA()
{
    SDPA::printSDPAVersion(stdout);
    SDPA	Problem1;
    Problem1.setDisplay(stdout);

    // All parameteres are renewed
    Problem1.setParameterType(SDPA::PARAMETER_DEFAULT);
    Problem1.printParameters(stdout);

    int mDIM   = 3;
    int nBlock = 1;
    Problem1.inputConstraintNumber(mDIM);
    Problem1.inputBlockNumber(nBlock);
    Problem1.inputBlockSize(1,2);
    Problem1.inputBlockType(1,SDPA::SDP);

    Problem1.initializeUpperTriangleSpace();

    Problem1.inputCVec(1,48);
    Problem1.inputCVec(2,-8);
    Problem1.inputCVec(3,20);

    Problem1.inputElement(0, 1, 1, 1, -11);
    Problem1.inputElement(0, 1, 2, 2,  23);
    
    Problem1.inputElement(1, 1, 1, 1,  10);
    Problem1.inputElement(1, 1, 1, 2,   4);
    
    Problem1.inputElement(2, 1, 2, 2,  -8);
    
    Problem1.inputElement(3, 1, 1, 2,  -8);
    Problem1.inputElement(3, 1, 2, 2,  -2);

    Problem1.initializeUpperTriangle();
    Problem1.initializeSolve();

    Problem1.solve();
    fprintf(stdout, "xVec = \n");
    // Problem1.printResultXVec();
    printVector(Problem1.getResultXVec(),
            Problem1.getConstraintNumber(), (char*)"%+8.3e",
            stdout);

    Problem1.terminate();
}

void OrientationPri::printVector(double* ele, int dim, char* printFormat, FILE* fpout)
{
  fprintf(fpout,"[ ");
  for (int k=0; k<dim-1; ++k) {
    fprintf(fpout,printFormat,ele[k]);
    fprintf(fpout," ");
  }
  fprintf(fpout,printFormat,ele[dim-1]);
  fprintf(fpout,"]; \n");
}
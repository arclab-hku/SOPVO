#include "include/lkorb_tracking.h"

LKORBTracking::LKORBTracking(int width_in,int height_in)
{
    this->width=width_in;
    this->height=height_in;
}


bool LKORBTracking::tracking(CameraFrame& from,
                             CameraFrame& to,
                             vector<Vec2>& lm2d_from,
                             vector<Vec2>& lm2d_to,
                             vector<Vec2>& outlier)
{
    //STEP1: Optical Flow
    int outlier_untracked_cnt=0;
    int outlier_orb_unmatch_cnt=0;

    vector<cv::Point2f> from_cvP2f = from.get2dPtsVec_cvP2f();
    vector<cv::Point2f> tracked_cvP2f;
    // vector<cv::Mat>     trackedLMDescriptors;
    vector<float>   err;
    vector<unsigned char> mask_tracked;
    // vector<unsigned char> mask_hasorb;
    // vector<unsigned char> mask_matched;
    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 30, 0.01);

    cv::Mat from_img, to_img;
    cv::equalizeHist(from.img0,from_img);
    cv::equalizeHist(to.img0,to_img);
    cv::calcOpticalFlowPyrLK(from_img, to_img, from_cvP2f, tracked_cvP2f,
                             mask_tracked, err, cv::Size(21,21), 6, criteria);

    // //estimate F matrix check
    // vector<unsigned char> mask_F_consistant;
    // cv::findFundamentalMat(from_cvP2f, tracked_cvP2f,
    //                     cv::FM_RANSAC, 5.0, 0.99, mask_F_consistant);

    // cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();

    vector<cv::KeyPoint>     tracked_lm_cvKP;
    // cv::Mat descriptorsMat;

    cv::KeyPoint::convert(tracked_cvP2f,tracked_lm_cvKP);
    // extractor->compute(to.img0, tracked_lm_cvKP, descriptorsMat);

    //copy to new frame
    bool reuslt = false;
    for(int i=from.landmarks.size()-1; i>=0; i--)
    {
        // if(mask_tracked.at(i) ==1 && mask_F_consistant.at(i) == 1)
        if(mask_tracked.at(i) ==1)
        {//inliers
            lm2d_from.push_back(from.landmarks.at(i).lm_2d);
            lm2d_to.push_back(Vec2(tracked_cvP2f.at(i).x,tracked_cvP2f.at(i).y));
            LandMarkInFrame lm=from.landmarks.at(i);
            lm.lm_2d=Vec2(tracked_cvP2f.at(i).x,tracked_cvP2f.at(i).y);
            to.landmarks.push_back(lm);
        }
        else
        {//outliers
            outlier.push_back(from.landmarks.at(i).lm_2d);
        }
        
    }
    if(lm2d_to.size()>=20 && outlier.size()>lm2d_to.size())
    {
        reuslt=true;
    }

    return reuslt;
}

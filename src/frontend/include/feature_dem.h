#ifndef FEATUREDEM_H
#define FEATUREDEM_H

#include <include/common.h>
#include "include/camera_frame.h"
#include <iostream>
#include <utility>

/* Feature Detector--------Region Based FAST Detector
 *         Extractor-------ORB descriptor
 *         Match Checker---Check by ORB distance
 *  * /



/* Region Based FAST Detector
 * Regions:
 *  | 0 | 1 | 2 | 3 |
 *  | 4 | 5 | 6 | 7 |
 *  | 8 | 9 | 10| 11|
 *  | 12| 13| 14| 15|
 *  //Detect FAST features
 *  //Devided all features into 16 regions
 *  //For every region, select features by Harris index and boundary size
 * */

#define MAX_REGION_FREATURES_NUM (20)
#define MIN_REGION_FREATURES_NUM (2)


#define BOUNDARYBOXSIZE          (5)


using namespace std;


class FeatureDEM
{
private:

  int width;
  int height;
  int vstep;    // number of region vertical
  int vsize;
  int hstep;    // number of region horizontal
  int hsize;    
  int subtotal; // maximum feature in each region
  int regionWidth;
  int regionHeight;
  int boundary_dis;

  std::vector<pair<cv::Point2f,float> > regionKeyPts[16];
  cv::Mat detectorMask[16];

  void calHarrisR(const cv::Mat& img, cv::Point2f& Pt, float &R);

  void filterAndFillIntoRegion(const cv::Mat& img,
                               const vector<cv::Point2f>& pts);

  void fillIntoRegion(const cv::Mat& img,
                      const vector<cv::Point2f>& pts);

public:

  FeatureDEM(const int image_width,
             const int image_height,
             const int gird_width,
             const int grid_height,
             const int max_f,
             int boundaryBoxSize=BOUNDARYBOXSIZE);
  ~FeatureDEM();

  void convert_point2f_vec2(std::vector<cv::Point2f>& pts_point2f, vector<Vec2>& pts_vec2);

  void grided_detect(const cv::Mat& img,
                      std::vector<cv::Point2f>& pts);

  void fb_tracking(const cv::Mat& img0,
                    const cv::Mat& img1,
                    std::vector<cv::Point2f>& pts0,
                    std::vector<cv::Point2f>& pts1);  

  void featureDetectionGoodFeaturesToTrack(cv::Mat image, 
                                            std::vector<cv::Point2f>& points,
                                            int maxCorners);

  void featureTracking(cv::Mat img0, 
                        cv::Mat img1, 
                        std::vector<cv::Point2f>& points0, 
                        std::vector<cv::Point2f>& points1);

  void deleteUnmatchFeatures(std::vector<cv::Point2f>& points0, 
                              std::vector<cv::Point2f>& points1, 
                              std::vector<uchar>& status);

  void grided_fastfeature_detection(cv::Mat image, std::vector<cv::KeyPoint>& keypoints);

  void redetect(const cv::Mat& img,
                          const vector<Vec2>& existedPts,
                          vector<Vec2>& newPts,
                          int &newPtscount);

};//class RegionFeatureDetector

#endif // FEATUREDEM_H

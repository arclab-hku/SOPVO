#include <include/feature_dem.h>
#include <opencv2/features2d/features2d.hpp>
#include <unordered_set>
#include <cmath>

// Driver function to sort the vector elements by
// second element of pair in descending order
static bool sortbysecdesc(const pair<cv::Point2f,float> &a,
                          const pair<cv::Point2f,float> &b)
{
    return a.second > b.second;
}

FeatureDEM::FeatureDEM(const int image_width,
                       const int image_height,
                       const int gird_width,
                       const int grid_height,
                       const int max_f,
                       int boundaryBoxSize)
{
    width  = image_width;
    height = image_height;
    hstep  = gird_width;
    vstep  = grid_height;
    hsize = int(width/hstep);
    vsize = int(height/vstep);
    subtotal = max_f;
    regionWidth  = floor(width/4.0);
    regionHeight = floor(height/4.0);
    boundary_dis = floor(boundaryBoxSize/2.0);
    int gridx[5],gridy[5];
    for(int i=0; i<5; i++)
    {
        gridx[i]=i*regionWidth;
        gridy[i]=i*regionHeight;
    }
    for(int i=0; i<16; i++)
    {
        detectorMask[i] = cv::Mat(image_height, image_width, CV_8S,cv::Scalar(0));
        int x_begin,x_end,y_begin,y_end;
        x_begin = gridx[i%4];
        x_end   = gridx[(i%4)+1];
        y_begin = gridy[i/4];
        y_end   = gridy[(i/4)+1];
        for(int xx=x_begin; xx<x_end; xx++)
        {
            for(int yy=y_begin; yy<y_end; yy++)
            {
                detectorMask[i].at<schar>(yy,xx)=1;
            }
        }
    }
}

FeatureDEM::~FeatureDEM()
{;}

void FeatureDEM::calHarrisR(const cv::Mat& img,
                            cv::Point2f& Pt,
                            float &R)
{
    uchar patch[9];
    int xx = Pt.x;
    int yy = Pt.y;
    patch[0]=img.at<uchar>(cv::Point(xx-1,yy-1));
    patch[1]=img.at<uchar>(cv::Point(xx,yy-1));
    patch[2]=img.at<uchar>(cv::Point(xx+1,yy-1));
    patch[3]=img.at<uchar>(cv::Point(xx-1,yy));
    patch[4]=img.at<uchar>(cv::Point(xx,yy));
    patch[5]=img.at<uchar>(cv::Point(xx+1,yy+1));
    patch[6]=img.at<uchar>(cv::Point(xx-1,yy+1));
    patch[7]=img.at<uchar>(cv::Point(xx,yy+1));
    patch[8]=img.at<uchar>(cv::Point(xx+1,yy+1));
    float IX,IY;
    float X2,Y2;
    float XY;
    IX = (patch[0]+patch[3]+patch[6]-(patch[2]+patch[5]+patch[8]))/3;
    IY = (patch[0]+patch[1]+patch[2]-(patch[6]+patch[7]+patch[8]))/3;
    X2 = IX*IX;
    Y2 = IY*IX;
    XY = IX*IX;
    //M = | X2  XY |
    //    | XY  Y2 |
    //R = det(M)-k(trace^2(M))
    //  = X2*Y2-XY*XY  - 0.05*(X2+Y2)*(X2+Y2)
    R = (X2*Y2)-(XY*XY) - 0.05*(X2+Y2)*(X2+Y2);
}

void FeatureDEM::filterAndFillIntoRegion(const cv::Mat& img,
                                         const vector<cv::Point2f>& pts)
{
    //Devided all features into 16 regions
    for(size_t i=0; i<pts.size(); i++)
    {
        cv::Point2f pt = pts.at(i);
        if (pt.x>=10 && pt.x<(width-10) && pt.y>=10 && pt.y<(height-10))
        {
            float Harris_R;
            calHarrisR(img,pt,Harris_R);
            int regionNum= 4*floor(pt.y/regionHeight) + (pt.x/regionWidth);
            regionKeyPts[regionNum].push_back(make_pair(pt,Harris_R));
        }
    }
}

void FeatureDEM::fillIntoRegion(const cv::Mat& img,
                                const vector<cv::Point2f>& pts)
{
    for(size_t i=0; i<pts.size(); i++)
    {
        cv::Point2f pt = pts.at(i);
        int regionNum= 4*floor(pt.y/regionHeight) + (pt.x/regionWidth);
        regionKeyPts[regionNum].push_back(make_pair(pt,99999.0));
    }
}

void FeatureDEM::grided_detect(const cv::Mat& img, std::vector<cv::Point2f>& tmpPts)
{
    //Clear
    tmpPts.clear();
    for(int i=0; i<16; i++)
    {
        regionKeyPts[i].clear();
    }
    //Detect FAST
    // cv::Ptr<cv::FastFeatureDetector> detector= cv::FastFeatureDetector::create();
    //cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(3000);
    vector<cv::KeyPoint> tmpKPs;
    grided_fastfeature_detection(img, tmpKPs);
    //Fill into region
    cv::KeyPoint::convert(tmpKPs, tmpPts);
    filterAndFillIntoRegion(img, tmpPts);
    //For every region, select features by Harris index and boundary size
    for(int i=0; i<16; i++)
    {
        sort(regionKeyPts[i].begin(), regionKeyPts[i].end(), sortbysecdesc);
        vector<pair<cv::Point2f,float>> tmp = regionKeyPts[i];
        regionKeyPts[i].clear();
        int count = 0;
        for(size_t j=0; j<tmp.size(); j++)
        {
            int outSideConflictBoundary = 1;
            for(size_t k=0; k<regionKeyPts[i].size(); k++)
            {
                float dis_x = fabs(tmp.at(j).first.x-regionKeyPts[i].at(k).first.x);
                float dis_y = fabs(tmp.at(j).first.y-regionKeyPts[i].at(k).first.y);
                if(dis_x<=boundary_dis || dis_y<=boundary_dis)
                {
                    outSideConflictBoundary=0;
                }
            }
            if(outSideConflictBoundary)
            {
                regionKeyPts[i].push_back(tmp.at(j));
                count++;
                if(count>=MAX_REGION_FREATURES_NUM) break;
            }
        }
    }
    //output
    tmpPts.clear();
    for(int i=0; i<16; i++)
    {
        //cout << regionKeyPts[i].size() << "in Region " << i << endl;
        for(size_t j=0; j<regionKeyPts[i].size(); j++)
        {
            tmpPts.push_back(regionKeyPts[i].at(j).first);
        }
    }
}

void FeatureDEM::fb_tracking(const cv::Mat& img0, const cv::Mat& img1, std::vector<cv::Point2f>& pts0, std::vector<cv::Point2f>& pts1)
{
    std::vector<cv::Point2f> pts0_add;
    std::vector<cv::Point2f> pts1_add;

    grided_detect(img0, pts0);
    // featureDetectionGoodFeaturesToTrack(img0, pts0, 200);
    grided_detect(img1, pts1_add);
    // featureDetectionGoodFeaturesToTrack(img1, pts1_add, 200);
    // forward tracking
    featureTracking(img0, img1, pts0, pts1);
    // cout << " forward tracking finished, keep " << pts0.size() << " keypoints for this keyframe" << endl;
    // backward tracking
    featureTracking(img1, img0, pts1_add, pts0_add);
    // cout << " backward tracking finished, keep " << pts0_add.size() << " keypoints for this keyframe" << endl;
    // combine keypoints and remove the duplicates
    for(int i = 0; i < pts0_add.size(); i ++)
    {
        pts0_add[i].x = round(pts0_add[i].x);
        pts0_add[i].y = round(pts0_add[i].y);
        std::vector<cv::Point2f>::iterator it = std::find(pts0.begin(), pts0.end(), pts0_add[i]);
        if(it == pts0.end())
        {
            pts0.push_back(pts0_add[i]);
            pts1.push_back(pts1_add[i]);
        }
    }
}

void FeatureDEM::featureDetectionGoodFeaturesToTrack(cv::Mat image, std::vector<cv::Point2f>& points, int maxCorners)  
{   
//uses GoodFeaturesToTrack for feature dection, modify parameters as necessary
  double qualityLevel = 0.01;
  double minDistance = 5.;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;
  cv::Mat mask;

  cv::goodFeaturesToTrack( image, points, maxCorners, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k );
}

void FeatureDEM::featureTracking(cv::Mat img0, cv::Mat img1, std::vector<cv::Point2f>& points0, std::vector<cv::Point2f>& points1)  
{ 
  std::vector<float> err; 
  std::vector<uchar> status;                   
  cv::Size winSize=cv::Size(21,21);                                                                                             
  cv::calcOpticalFlowPyrLK(img0, img1, points0, points1, status, err, cv::Size(31,31),10, 
                            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01));
  deleteUnmatchFeatures(points0, points1, status);
}

void FeatureDEM::deleteUnmatchFeatures(std::vector<cv::Point2f>& points0, std::vector<cv::Point2f>& points1, std::vector<uchar>& status)
{
  //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
  int indexCorrection = 0;
  for( int i=0; i<status.size(); i++)
     {  
        cv::Point2f pt = points1.at(i- indexCorrection);
        if ( (status.at(i) == 0) || (pt.x < 10) || (pt.y < 10) || (pt.x > width-10) || (pt.y > height-10) )   
        {
              points0.erase (points0.begin() + (i - indexCorrection));
              points1.erase (points1.begin() + (i - indexCorrection));
              indexCorrection++;
        }
     }
}

void FeatureDEM::convert_point2f_vec2(std::vector<cv::Point2f>& pts_point2f, vector<Vec2>& pts_vec2)
{
    pts_vec2.clear();
    for(size_t i=0; i<pts_point2f.size(); i++)
    {
        pts_vec2.push_back(Vec2(pts_point2f.at(i).x,pts_point2f.at(i).y));
    }
}

void FeatureDEM::grided_fastfeature_detection(cv::Mat image, std::vector<cv::KeyPoint>& keypoints)
{
    keypoints.clear();
    std::vector<cv::KeyPoint> gridpoints;
    cv::Ptr<cv::FastFeatureDetector> ptrFAST = cv::FastFeatureDetector::create();
    for (int i = 0; i < vstep; i++)
    {
        for (int j = 0; j < hstep; j++)
        {
            // create ROI over current grid
            cv::Mat imageROI = image(cv::Rect(j*hsize, i*vsize, hsize,vsize));
            // detect the keypoints in grid
            gridpoints.clear(); 
            ptrFAST->detect(imageROI, gridpoints);
            // get the strongest FAST features
            auto itEnd(gridpoints.end());
            if (gridpoints.size() > subtotal)
            {
                // select the strongest features
                std::nth_element(gridpoints.begin(), gridpoints.begin() + subtotal, gridpoints.end(), 
                                [](cv::KeyPoint& a, cv::KeyPoint& b) {return a.response > b.response;});
                itEnd = gridpoints.begin() + subtotal;
            }
            // add them to the global keypoint vector 
            for (auto it = gridpoints.begin(); it != itEnd; ++it) 
            {          
                // convert to image coordinates           
                it->pt += cv::Point2f(j*hsize, i*vsize);            
                keypoints.push_back(*it);
            }
        }
    }
}

void FeatureDEM::redetect(const cv::Mat& img,
                          const vector<Vec2>& existedPts,
                          vector<Vec2>& newPts,
                          int &newPtscount)
{
    //Clear
    newPts.clear();
    newPtscount=0;
    for(int i=0; i<16; i++)
    {
        regionKeyPts[i].clear();
    }
    vector<cv::Point2f> newPts_cvP2f;
    newPts_cvP2f.clear();
    vector<cv::Point2f> existedPts_cvP2f=vVec2_2_vcvP2f(existedPts);
    filterAndFillIntoRegion(img,existedPts_cvP2f);
    vector<cv::KeyPoint> features;
    vector<cv::Point2f>  kps;
    grided_fastfeature_detection(img, features);
    cv::KeyPoint::convert(features,kps);
    vector<pair<cv::Point2f,float>> regionKeyPts_prepare[16];
    //devide into different region
    for(size_t i=0; i<kps.size(); i++)
    {
        cv::Point2f pt = kps.at(i);
        if (pt.x>=10 && pt.x<(width-10) && pt.y>=10 && pt.y<(height-10))
        {
            float Harris_R;
            calHarrisR(img,pt,Harris_R);
            if(1)
            {
                int regionNum= 4*floor(pt.y/regionHeight) + (pt.x/regionWidth);
                regionKeyPts_prepare[regionNum].push_back(make_pair(pt,Harris_R));
            }
        }
    }
    for(size_t i=0; i<16; i++)
    {
        sort(regionKeyPts_prepare[i].begin(), regionKeyPts_prepare[i].end(), sortbysecdesc);
        for(size_t j=0; j<regionKeyPts_prepare[i].size(); j++)
        {
            int noFeatureNearby = 1;
            cv::Point pt=regionKeyPts_prepare[i].at(j).first;
            for(size_t k=0; k<regionKeyPts[i].size(); k++)
            {
                float dis_x = fabs(pt.x-regionKeyPts[i].at(k).first.x);
                float dis_y = fabs(pt.y-regionKeyPts[i].at(k).first.y);
                if(dis_x <= boundary_dis || dis_y <= boundary_dis)
                {
                    noFeatureNearby=0;
                }
            }
            if(noFeatureNearby)
            {
                regionKeyPts[i].push_back(make_pair(pt,999999.0));
                newPts_cvP2f.push_back(pt);
                if(regionKeyPts[i].size() >= MAX_REGION_FREATURES_NUM) break;
            }
        }
    }
    //output
    if(newPts_cvP2f.size()>0)
    {
        // cv::Mat tmpDescriptors;
        vector<cv::KeyPoint> tmpKPs;
        cv::KeyPoint::convert(newPts_cvP2f,tmpKPs);
        // cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();
        // extractor->compute(img, tmpKPs, tmpDescriptors);
        for(size_t i=0; i<tmpKPs.size(); i++)
        {
            Vec2 pt(tmpKPs.at(i).pt.x,tmpKPs.at(i).pt.y);
            newPts.push_back(pt);
        }
        newPtscount=newPts.size();
        // descriptors_to_vMat(tmpDescriptors,newDescriptors);
    }
}
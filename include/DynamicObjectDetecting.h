
#ifndef DYNAMIC_FOBJECT_DETECTING_H
#define DYNAMIC_FOBJECT_DETECTING_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <vector>

#include "yolo_v2_class.hpp" 


namespace ORB_SLAM2
{


class DynamicObjectDetecting
{
public:
    DynamicObjectDetecting();
    void init(std::vector<std::string> specifiedThings, std::string cfgFile, std::string weightFile, std::string labelPath, double DetectTh);
    void run();

    void insertImage(const cv::Mat& im);
    std::vector<bbox_t> getDetectResult();
    
    void setFinish();
    bool isFinished();
    void requestFinish();

    std::vector<bbox_t> getDynamicObjectBbox(const cv::Mat& mImGray);
    std::vector<bbox_t> getSpecifiedObjectBbox(const cv::Mat& mImGray);
    void saveDistance();
    void setImGrayPre(const cv::Mat &imGrayPre);

    std::vector<std::string> id2name;
    std::unique_ptr<Detector> detector;

private:
    void loadLabel();
    void calcDynamicPoints(const cv::Mat &imGray);
    const std::vector<cv::Point2f>& getDynamicPoints();
    const std::vector<cv::Point2f>& getTrackedPoints();

    // config parameters
    std::string mCfgFile, mWeightFile;
    std::string mLabelPath;
    double mDetectTh;
    std::vector<std::string> mSpecifiedThings;

    // obtain the dynamic points
    cv::Mat mImGrayPre;
    std::vector<cv::Point2f> mDynamicPoints;
    std::vector<cv::Point2f> mTrackedPoints;
    const double limit_dis_epi = 1; 
    const double limit_of_check = 2120;
    const int limit_edge_corner = 5; 

};


}  // namespace ORB_SLAM2


#endif



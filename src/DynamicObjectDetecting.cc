#include <fstream>

#include "DynamicObjectDetecting.h"


template<typename T, typename... Ts>
std::unique_ptr<T> make_unique(Ts&&... params)
{
    return std::unique_ptr<T>(new T(std::forward<Ts>(params)...));
}

namespace ORB_SLAM2
{


DynamicObjectDetecting::DynamicObjectDetecting(){}

void DynamicObjectDetecting::init(std::vector<std::string> specifiedThings, std::string cfgFile, std::string weightFile, std::string labelPath, double detectTh)
{

    mCfgFile = cfgFile;
    mSpecifiedThings = specifiedThings;
    mWeightFile = weightFile;
    mLabelPath = labelPath;
    mDetectTh = detectTh;
    detector = ::make_unique<Detector>(mCfgFile, mWeightFile,0); //初始化检测器, 第三个参数是gpu_id
    loadLabel();
}

void DynamicObjectDetecting::loadLabel()
{
    std::ifstream fLabel; 
    id2name.resize(80);

    fLabel.open(mLabelPath.c_str());
    int i = 0;
    while (!fLabel.eof()) {
        std::string name;
        getline(fLabel,name);
        if (!name.empty()) {
            id2name[i++] = name;
        }
    }
}

std::vector<bbox_t> DynamicObjectDetecting::getDynamicObjectBbox(const cv::Mat& mImGray) {
    std::vector<bbox_t> results = detector->detect(mImGray, mDetectTh); 
    calcDynamicPoints(mImGray);
    const std::vector<cv::Point2f>& trackedPoints = getTrackedPoints();
    const std::vector<cv::Point2f>& dynamicPoints = getDynamicPoints();
    std::vector<std::vector<int>> g(mImGray.rows,std::vector<int>(mImGray.cols,0));
    for (auto pt: trackedPoints) {
        g[pt.y][pt.x] = 1;
    }
    for (auto pt: dynamicPoints) {
        g[pt.y][pt.x] = 2;
    }

    std::vector<bbox_t> dynaObjs;  
    for (bbox_t bbox : results) {
        int cntPoint = 0;
        int cntDynaPoint = 0;
        for (int deltaY = 0; deltaY < bbox.h; deltaY++) {
            for (int deltaX = 0; deltaX < bbox.w; deltaX++) {
                if (bbox.y+deltaY>=mImGray.rows || bbox.x+deltaX >= mImGray.cols) { // bbox可能会超出图像边界
                    continue;
                }
                if (g[bbox.y+deltaY][bbox.x+deltaX]==2) {
                    cntDynaPoint++;
                    cntPoint++;
                }else if (g[bbox.y+deltaY][bbox.x+deltaX]==1) {
                    cntPoint++;
                }
            }
        }                
        if (cntDynaPoint > 10 || (cntDynaPoint <= 10 && (double)cntDynaPoint/cntPoint > 0.8)) { // 根据动态特征点判断是否为动态目标
            dynaObjs.push_back(bbox);
        }
    }
    return dynaObjs;
}

std::vector<bbox_t> DynamicObjectDetecting::getSpecifiedObjectBbox(const cv::Mat& mImGray) {
    std::vector<bbox_t> results = detector->detect(mImGray, mDetectTh); 
    std::vector<bbox_t> dynaObjs;  
    for (bbox_t bbox : results) {
        for (std::string  thing :mSpecifiedThings) {
            if (id2name[bbox.obj_id] == thing) {
                dynaObjs.push_back(bbox); 
            }
        }
    }
    return dynaObjs;
}


/* Next function is from DS-SLAM，https://github.com/ivipsourcecode/DS-SLAM */
// Epipolar constraints and output the T matrix.
void DynamicObjectDetecting::calcDynamicPoints(const cv::Mat &imGray)
{
    mDynamicPoints.clear();
    if (mImGrayPre.empty()) { // 第一帧的情况
        return;
    }

    std::vector<uchar> state;
    std::vector<float> err;
    std::vector<cv::Point2f> lastFramePoints; 
    std::vector<cv::Point2f> F_lastFramePoints;  // F表示去除异常值
    std::vector<cv::Point2f> trackedPoints; 
    std::vector<cv::Point2f> F_trackedPoints; 


	// Detect dynamic target and ultimately optput the T matrix
    cv::goodFeaturesToTrack(mImGrayPre, lastFramePoints, 1000, 0.01, 8, cv::Mat(), 3, false, 0.04);  // Harris(true)、ShiTomas(false)角点检测
    cv::cornerSubPix(mImGrayPre, lastFramePoints, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.01)); // 亚像素角点检测
	cv::calcOpticalFlowPyrLK(mImGrayPre, imGray, lastFramePoints, trackedPoints, state, err, cv::Size(21, 21), 3, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.01));

    // remove outliers
	for (int i = 0; i < state.size(); i++) {
        if (state[i] != 0) {
            int dx[10] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
            int dy[10] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };
            int x1 = lastFramePoints[i].x, y1 = lastFramePoints[i].y;
            int x2 = trackedPoints[i].x, y2 = trackedPoints[i].y;
            if ((x1 < limit_edge_corner || x1 >= imGray.cols - limit_edge_corner || x2 < limit_edge_corner || x2 >= imGray.cols - limit_edge_corner
                    || y1 < limit_edge_corner || y1 >= imGray.rows - limit_edge_corner || y2 < limit_edge_corner || y2 >= imGray.rows - limit_edge_corner)) {
                state[i] = 0;
                continue;
            }
            double sum_check = 0;
            for (int j = 0; j < 9; j++) {
                sum_check += abs(mImGrayPre.at<uchar>(y1 + dy[j], x1 + dx[j]) - imGray.at<uchar>(y2 + dy[j], x2 + dx[j]));
            }
                
            if (sum_check > limit_of_check) {
                state[i] = 0;
            }
            
            if (state[i]) {
                F_lastFramePoints.push_back(lastFramePoints[i]);
                F_trackedPoints.push_back(trackedPoints[i]);
            }
        }
    }

    // F-Matrix
    cv::Mat mask = cv::Mat(cv::Size(1, 300), CV_8UC1);
    cv::Mat F = cv::findFundamentalMat(F_lastFramePoints, F_trackedPoints, mask, cv::FM_RANSAC, 0.05, 0.99);
    mTrackedPoints = F_trackedPoints;

    for (int i = 0; i < state.size(); i++) {
        if (state[i] != 0) {
            double A = F.at<double>(0, 0)*lastFramePoints[i].x + F.at<double>(0, 1)*lastFramePoints[i].y + F.at<double>(0, 2);
            double B = F.at<double>(1, 0)*lastFramePoints[i].x + F.at<double>(1, 1)*lastFramePoints[i].y + F.at<double>(1, 2);
            double C = F.at<double>(2, 0)*lastFramePoints[i].x + F.at<double>(2, 1)*lastFramePoints[i].y + F.at<double>(2, 2);
            double dd = fabs(A*trackedPoints[i].x + B*trackedPoints[i].y + C) / sqrt(A*A + B*B);

            // Judge outliers
            if (dd > limit_dis_epi) {
                mDynamicPoints.push_back(trackedPoints[i]); // mDynamicPoints为动态点集合 
            }
        }
    }
}

const std::vector<cv::Point2f>& DynamicObjectDetecting::getTrackedPoints() 
{
    return mTrackedPoints;
}

const std::vector<cv::Point2f>& DynamicObjectDetecting::getDynamicPoints() 
{
    return mDynamicPoints;
}

void DynamicObjectDetecting::setImGrayPre(const cv::Mat &imGrayPre) 
{
    mImGrayPre = imGrayPre;
}

}
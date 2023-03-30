#ifndef STEREO_MATCH_H_
#define STEREO_MATCH_H_
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>
#include <sstream>

namespace ORB_SLAM2
{
class StereoMatch
{

public:
    int Init();

    int ComputeDepthMap(cv::Mat& Left, cv::Mat& Right,  cv::Mat& xyz);
    int SetQ(cv::Mat& Q);
    
private:
    void SavePCLCloud( cv::Mat& img, cv::Mat& xyz);

    void FilterDepth(cv::Mat& xyz);
private:

    cv::Ptr<cv::StereoSGBM> m_pSGBM;
    cv::Mat m_Q;

    float m_fMaxDepth;
    float m_fMinDepth;
    std::string m_strSavePath;
};

}


#endif STEREO_MATCH_H_
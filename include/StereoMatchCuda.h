#ifndef STEREO_MATCH_CUDA_H_
#define STEREO_MATCH_CUDA_H_
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>
#include <sstream>

#include "libsgm.h"

namespace ORB_SLAM2
{
    class StereoMatchCuda
    {

public:
    StereoMatchCuda();
    int Init();

    int ComputeDepthMap(cv::Mat& Left, cv::Mat& Right,  cv::Mat& xyz);
    int SetQ(cv::Mat& Q);
    
private:
    void SavePCLCloud( cv::Mat& img, cv::Mat& xyz);

    void FilterDepth(cv::Mat& xyz);

    void ShowRectiedImg(cv::Mat& Left, cv::Mat& Right);
private:

    //cv::Ptr<cv::StereoSGBM> m_pSGBM;
    sgm::StereoSGM* m_pSGMCuda;
    cv::Mat m_Q;
    int m_nImgW;
    int m_nImgH;
    int m_nSrcC;
    int m_nDstC;
    bool m_bUseSubPixel;

    float m_fMaxDepth;
    float m_fMinDepth;
    std::string m_strSavePath;


    };


}


#endif //STEREO_MATCH_CUDA_H_
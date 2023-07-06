/*** 
 * @Author: DYG
 * @Date: 2023-07-06 17:09:20
 * @LastEditors: DYG
 * @LastEditTime: 2023-07-06 17:12:13
 * @FilePath: /ORB_SLAM2_detailed_comments/include/StereoMatchCuda.h
 * @Description: 
 * @Copyright (c) 2023 by HJ, All Rights Reserved. 
 */
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
    /*** 
     * @description: 初始化
     * @return {int} 成功返回0，其他为失败
     */    
    int Init();

    /*** 
     * @description: 计算深度图
     * @param {Mat&} Left 左目
     * @param {Mat&} Right 右目
     * @param {Mat&} xyz  计算得到的深度图
     * @return {int} 成功返回0，其他为失败
     */    
    int ComputeDepthMap(cv::Mat& Left, cv::Mat& Right,  cv::Mat& xyz);
    int SetQ(cv::Mat& Q);
    
private:
    /*** 
     * @description: 保存深度图为pcl文件
     * @param {Mat&} img  rgb图像
     * @param {Mat&} xyz  深度图
     * @return {*}
     */    
    void SavePCLCloud( cv::Mat& img, cv::Mat& xyz);

    /*** 
     * @description: 过滤深度不在阀值范围内的深度数据
     * @param {Mat&} xyz
     * @return {*}
     */    
    void FilterDepth(cv::Mat& xyz);

    /*** 
     * @description: 显示立体校正后的双目图像
     * @param {Mat&} Left 左目图像
     * @param {Mat&} Right 右目图像
     * @return {*}
     */    
    void ShowRectiedImg(cv::Mat& Left, cv::Mat& Right);
private:

    //cv::Ptr<cv::StereoSGBM> m_pSGBM;
    sgm::StereoSGM* m_pSGMCuda; //sgm对象指针
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
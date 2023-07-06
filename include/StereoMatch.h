/*** 
 * @Author: DYG
 * @Date: 2023-07-06 17:06:32
 * @LastEditors: DYG
 * @LastEditTime: 2023-07-06 17:11:42
 * @FilePath: /ORB_SLAM2_detailed_comments/include/StereoMatch.h
 * @Description: 
 * @Copyright (c) 2023 by HJ, All Rights Reserved. 
 */
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
    /*** 
     * @description: 初始化
     * @return {int} 成功返回0，其他为失败
     */    
    int Init();

    /*** 
     * @description: 计算深度图
     * @param {Mat&} Left 左目相机
     * @param {Mat&} Right 右目相机
     * @param {Mat&} xyz   生成的深度图
     * @return {int} 成功返回0，其他为失败
     */    
    int ComputeDepthMap(cv::Mat& Left, cv::Mat& Right,  cv::Mat& xyz);

    /*** 
     * @description: 设置立体校正得到的相关参数
     * @param {Mat&} Q 立体校正得到的相关参数
     * @return {int} 成功返回0，其他为失败
     */    
    int SetQ(cv::Mat& Q);
    
private:
    /*** 
     * @description: 保存单帧
     * @param {Mat&} img
     * @param {Mat&} xyz
     * @return {*}
     */    
    void SavePCLCloud( cv::Mat& img, cv::Mat& xyz);

    /*** 
     * @description: 过滤掉深度在阀值之外的深度数据
     * @param {Mat&} xyz 深度图
     * @return {*}
     */    
    void FilterDepth(cv::Mat& xyz);

    /*** 
     * @description: 显示立体校正过后的双目图像
     * @param {Mat&} Left 左目图像
     * @param {Mat&} Right
     * @return {*}
     */    
    void ShowRectiedImg(cv::Mat& Left, cv::Mat& Right);
private:

    cv::Ptr<cv::StereoSGBM> m_pSGBM; //sgbm对象指针
    cv::Mat m_Q;

    float m_fMaxDepth;
    float m_fMinDepth;
    std::string m_strSavePath;
};

}


#endif STEREO_MATCH_H_
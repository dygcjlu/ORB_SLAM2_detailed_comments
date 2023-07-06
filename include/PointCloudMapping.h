/*** 
 * @Author: DYG
 * @Date: 2023-05-25 10:38:16
 * @LastEditors: DYG
 * @LastEditTime: 2023-07-06 17:11:24
 * @FilePath: /ORB_SLAM2_detailed_comments/include/PointCloudMapping.h
 * @Description: 
 * @Copyright (c) 2023 by HJ, All Rights Reserved. 
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include <atomic>

#include "System.h"
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "StereoMatch.h"
#include "StereoMatchCuda.h"
// using namespace ORB_SLAM2;

namespace ORB_SLAM2
{

struct CustomPoint3d
{
    float point3d[3];
    int rgbcolor[3];
    int nPointIndex;
    int nPointViewNum;
    std::vector<int> vecViewId; //kf id

    CustomPoint3d()
    {
        nPointIndex = 0;
        nPointViewNum = 0;
        point3d[0] = 0.;
        point3d[1] = 0.;
        point3d[2] = 0.;
    }

};

class PointCloudMapping
{
public:
    // typedef pcl::PointXYZRGBA PointT;
    // typedef pcl::PointCloud<PointT> PointCloud;

    PointCloudMapping(double resolution_, double meank_, double thresh_);
    ~PointCloudMapping();
    //void save();

    /*** 
     * @description: 获取关键帧
     * @param {KeyFrame} *kf 关键帧对象指针
     * @return {*}
     */    
    void insertKeyFrame(KeyFrame *kf);

    /*** 
     * @description: 释放资源
     * @return {*}
     */    
    void shutdown();

    /*** 
     * @description: 
     * @return {*}
     */    

    /*** 
     * @description: 点云生成和可视化线程
     * @return {*}
     */    
    void viewer();


    //void inserttu(cv::Mat &color, cv::Mat &depth, int idk);

public:
    /*** 
     * @description: 设置立体校正的Q矩阵
     * @param {Mat} Q 立体校正矩阵
     * @return {int} 成功返回0，其他为失败
     */    
    int SetRectifiedQ(cv::Mat Q);

    /*** 
     * @description: 设置相机内参矩阵
     * @param {Mat} P
     * @return {int} 成功返回0，其他为失败
     */    
    int SetRectifiedP(cv::Mat P);


    /*** 
     * @description: 保存相机轨迹位置数据
     * @param {KeyFrame} * 关键帧列表
     * @return {*}
     */        
    void SaveCameraPosition( std::list<KeyFrame *>& lNewKeyFrames);

    //thread, generate depth map from stereo match
    /*** 
     * @description: 点云生成线程，先获取全部关键帧，生成深度图，然后融合深度图生成最终的点云数据
     * @return {*}
     */    
    void GenerateDepthMapThread();

    /*** 
     * @description: 点云生成线程，扫描过程中实时获取关键帧，生成深度图，扫描完成后再融合所有深度图生成最终的点云数据
     * @return {*}
     */    
    void RealTimeGeneratePointCloudThread();

    /*** 
     * @description: 将领域帧投影到当前帧
     * @param {KeyFrame*} kf 当前关键帧指针
     * @return {*}
     */    
    void ProjectFromNeighbor(KeyFrame* kf);

    /*** 
     * @description: 保存单帧图像的pcl点云数据
     * @param {Mat&} img rgb图像
     * @param {Mat&} xyz 深度图
     * @param {string} strFileName 保存文件名
     * @return {*}
     */    
    void SavePCLCloud(cv::Mat& img, cv::Mat& xyz, std::string strFileName);


    /*** 
     * @description: 判断两个深度是否足够接近到可以被认为是同一个点
     * @param {float} d1  深度1
     * @param {float} d2  深度2
     * @param {float} fThreshold  阀值
     * @return {bool}
     */    
    bool IsDepthSimilar(float d1, float d2, float fThreshold);

    /*** 
     * @description: 判断两个关键帧的相对位姿是否发生了改变
     * @param {KeyFrame} *kf 当前帧指针
     * @param {KeyFrame} *NeighborKf 领域帧指针
     * @return {bool}
     */    
    bool IsPoseChange(KeyFrame *kf, KeyFrame *NeighborKf);

    /*** 
     * @description: 单帧深度图优化
     * @param {KeyFrame*} kf 当前关键帧指针
     * @return {*}
     */    
    void FilterDepthMap(KeyFrame* kf);

    /*** 
     * @description: 删除非法的关键帧
     * @return {*}
     */    
    void DeleteBadKF();

    
    /*** 
     * @description: 判断当前线程是否空闲
     * @return {bool}
     */    
    bool IsThreadBusy();

    /*** 
     * @description: 将所有深度图直接融合成一个三维点云
     * @return {*}
     */    
    void FusePCLCloud();

    /*** 
     * @description: 将所有深度图融合成一个三维点云，同时过滤掉部分质量差的点云
     * @return {*}
     */    
    void FusePCLCloud2();

public:
    int mnloopcount = 0;
    vector<KeyFrame *> currentvpKFs;
    bool cloudbusy = false;
    bool loopbusy = false;
    void updatecloud(Map &curMap);
    void Clear();
    bool bStop = false;

    // 关于更新时的变量
    std::atomic<bool> mabIsUpdating;


protected:
    void generatePointCloud(KeyFrame *kf);
    void generatePointCloudStereo(KeyFrame *kf);

    std::list<KeyFrame *> mlNewKeyFrames;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalMap;
    shared_ptr<thread> viewerThread;

    bool shutDownFlag = false;
    std::mutex shutDownMutex;

    condition_variable keyFrameUpdated;
    std::mutex mMutexGlobalMap;
   
    // vector<PointCloude>     pointcloud;
    // data to generate point clouds
    vector<KeyFrame *> keyframes;
    vector<cv::Mat> colorImgs;
    vector<cv::Mat> depthImgs;
    vector<cv::Mat> colorImgks;
    vector<cv::Mat> depthImgks;
    vector<int> ids;
    std::mutex keyframeMutex;
    std::mutex updateMutex;
    uint16_t lastKeyframeSize = 0;

    double resolution = 0.04;
    double meank = 50;
    double thresh = 1;
    pcl::VoxelGrid<pcl::PointXYZRGBA> *voxel;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> *statistical_filter;

    StereoMatch m_stereoMatch;
    StereoMatchCuda m_stereoMatchCuda;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalCameraMap;

    shared_ptr<thread> m_depthMapThread;
    shared_ptr<thread> m_realTimeDepthMapThread;
    

    std::map<int, KeyFrame *> m_mapKeyFrame;
    std::map<int, std::map<int, cv::Mat>> m_mapNeighborDepth;
    std::map<int, cv::Mat> m_mapKF2NeighborPose;
    cv::Mat m_K; //Intrinsics after rectified
    float m_fMaxDepth; //max depth of point 
    float m_fMinDepth; //min depth of point

    std::string m_strSavePath; //save depth map to this directory
    float m_fFloatZero;
    bool m_bIsBusy;

    bool m_bFirstReceived;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_newGlobalMap;
    std::mutex m_MutexNewGlobalMap;

    //some threshold param
    float m_fDepthDiffStrict;// = 0.02;
    int m_nMinViewNum;
    //int nMinViews = 2; //not include current depth map
    float m_fGoodViewRatio;// = 0.75;
    bool m_bUseCuda;
    bool m_bIsSaveData;

};
}
#endif // POINTCLOUDMAPPING_H

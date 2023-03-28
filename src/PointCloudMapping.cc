/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "PointCloudMapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h> 
#include <pcl/point_types.h>
#include "Converter.h"
#include "System.h"

#include<sys/time.h>
namespace ORB_SLAM2
{
// int currentloopcount = 0;
PointCloudMapping::PointCloudMapping(double resolution_, double meank_, double thresh_)
    : mabIsUpdating(false)
{
    shutDownFlag = false;
    this->resolution = resolution_;
    this->meank = meank_;
    this->thresh = thresh_;
    std::cout<<resolution<<" "<<meank<<" "<<thresh<<std::endl;
    statistical_filter = new pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA>(true);
    voxel = new pcl::VoxelGrid<pcl::PointXYZRGBA>();
    statistical_filter->setMeanK(meank);
    statistical_filter->setStddevMulThresh(thresh);
    voxel->setLeafSize(resolution, resolution, resolution);
    globalMap = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    globalCameraMap = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

    viewerThread = make_shared<thread>(bind(&PointCloudMapping::viewer, this));

    //m_depthMapThread = make_shared<thread>(bind(&PointCloudMapping::GenerateDepthMapThread, this));


    m_stereoMatch.Init();
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
    m_depthMapThread->join();
}

void PointCloudMapping::Clear()
{
    std::cout << "清除稠密地图" << std::endl;
    std::unique_lock<std::mutex> lck(mMutexGlobalMap);
    globalMap.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
}
void PointCloudMapping::insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth, int idk, vector<KeyFrame *> vpKFs)
{
    // cout << "receive a keyframe, 第" << kf->mnId << "个" << endl;
    // if (color.empty())
    //     return;
    // unique_lock<mutex> lck(keyframeMutex);
    // keyframes.push_back(kf);
    // currentvpKFs = vpKFs;
    // PointCloude pointcloude;
    // pointcloude.pcID = idk;
    // pointcloude.T = ORB_SLAM3::Converter::toSE3Quat(kf->GetPose());
    // pointcloude.pcE = generatePointCloud(kf, color, depth);
    // kf->mptrPointCloud = pointcloude.pcE;
    // pointcloud.push_back(pointcloude);
    // keyFrameUpdated.notify_one();
}

void PointCloudMapping::insertKeyFrame(KeyFrame *kf)
{
    cout << "receive a keyframe, 第" << kf->mnId << "个" << endl;
    if (kf->imLeftRgb.empty())
    {
        cout<<"kf->imLeftRgb.empty()"<< endl;
        return;
    }
    if(kf->isBad())
    {
        cout<<"this frame is bad!"<<endl;
        return;
    }
     cout << "insertKeyFrame 0"  << endl;
        
    unique_lock<mutex> lck(keyframeMutex);
    mlNewKeyFrames.emplace_back(kf);
    cout << "insertKeyFrame 1"  << endl;
    /*
    if(mlNewKeyFrames.size() > 200)
        mlNewKeyFrames.pop_front();
    */

}

void PointCloudMapping::generatePointCloud(KeyFrame *kf) //,Eigen::Isometry3d T
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    // point cloud is null ptr
    for (int m = 0; m < kf->imDepth.rows; m += 4)
    {
        for (int n = 0; n < kf->imDepth.cols; n += 4)
        {
            float d = kf->imDepth.ptr<float>(m)[n];
            if (d < 0.05 || d > 6)
            {
                //cout<<"depth is not valid:"<<d << endl;
                continue;
            }
                
            pcl::PointXYZRGBA p;
            p.z = d;
            p.x = (n - kf->cx) * p.z / kf->fx;
            p.y = (m - kf->cy) * p.z / kf->fy;

            p.b = kf->imLeftRgb.ptr<uchar>(m)[n * 3];
            p.g = kf->imLeftRgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = kf->imLeftRgb.ptr<uchar>(m)[n * 3 + 2];

            pPointCloud->points.push_back(p);
        }
    }
    pPointCloud->height = 1;
    pPointCloud->width = pPointCloud->points.size();
    pPointCloud->is_dense = true;
    kf->mptrPointCloud = pPointCloud;
}


void PointCloudMapping::generatePointCloudStereo(KeyFrame *kf) //,Eigen::Isometry3d T
{
    if(kf->imLeftRgb.channels() == 1)
    {
        cv::cvtColor(kf->imLeftRgb, kf->imLeftRgb, cv::COLOR_GRAY2BGR);
        cv::cvtColor(kf->imRightRgb, kf->imRightRgb, cv::COLOR_GRAY2BGR);
    }

    m_stereoMatch.ComputeDepthMap(kf->imLeftRgb, kf->imRightRgb, kf->imDepth);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // point cloud is null ptr
    for (int m = 0; m < kf->imDepth.rows; m += 4)
    {
        for (int n = 0; n < kf->imDepth.cols; n += 4)
        {
            /*
            if((m<20) || (m>kf->imDepth.rows-20))
            {
                //skip black border
                continue;
            }
            */

            pcl::PointXYZRGBA p;
            
            cv::Vec3f xyzPixel = kf->imDepth.at<cv::Vec3f>(m, n);
            cv::Vec3b bgrPixel = kf->imLeftRgb.at<cv::Vec3b>(m, n);
            p.x = xyzPixel.val[0];
            p.y = xyzPixel.val[1];
            p.z = xyzPixel.val[2];
            if(p.z > 0.8 )
            {
                //std::cout<<p.z<<std::endl;
                continue;
            }
            //std::cout<<"  "<<p.z;//<<std::endl;
                
            if(p.z < 0.005)
            {
                //std::cout<<"p.z < 1 "<<std::endl;
                continue;
            }   
            p.b = bgrPixel.val[0];
            p.g = bgrPixel.val[1];
            p.r = bgrPixel.val[2];
        
        
            pPointCloud->points.push_back(p);
        }
    }
    pPointCloud->height = 1;
    pPointCloud->width = pPointCloud->points.size();
    pPointCloud->is_dense = true;
    kf->mptrPointCloud = pPointCloud;

    std::cout<<"point size :"<<pPointCloud->points.size()<<std::endl;
}

void PointCloudMapping::SaveCameraPosition( std::list<KeyFrame *>& lNewKeyFrames)
{
     for (auto pKF : lNewKeyFrames)
    {
        //Eigen::Vector3f Ow2 = pKF->GetCameraCenter();
        cv::Mat Ow2 = pKF->GetCameraCenter();

        pcl::PointXYZRGBA p;
            
        p.x = Ow2.at<float>(0,0);
        p.y = Ow2.at<float>(0,1);
        p.z = Ow2.at<float>(0,2);
            
        p.b = 127;
        p.g = 127;
        p.r = 127;

        globalCameraMap->points.push_back(p);
    }

    globalCameraMap->height = 1;
    globalCameraMap->width = globalCameraMap->points.size();
    globalCameraMap->is_dense = false;
    std::string strSavePath = "/media/xxd/Data2/datasets/3d/za/";

    std::string strSaveName = strSavePath + "camera_position.ply";
    pcl::PLYWriter writer;
	writer.write(strSaveName, *globalCameraMap);
    std::cout<<"save camera position"<<std::endl;


}

Eigen::Matrix<float,4,4> toMatrix4f(const cv::Mat &cvMat4)
{
    Eigen::Matrix<float,4,4> M;

    M << cvMat4.at<float>(0,0), cvMat4.at<float>(0,1), cvMat4.at<float>(0,2), cvMat4.at<float>(0,3),
            cvMat4.at<float>(1,0), cvMat4.at<float>(1,1), cvMat4.at<float>(1,2), cvMat4.at<float>(1,3),
            cvMat4.at<float>(2,0), cvMat4.at<float>(2,1), cvMat4.at<float>(2,2), cvMat4.at<float>(2,3),
            cvMat4.at<float>(3,0), cvMat4.at<float>(3,1), cvMat4.at<float>(3,2), cvMat4.at<float>(3,3);

    return M;
}

void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    int nKFCount = 0;
    cout<<"thread viewer start"<<endl;
    // KeyFrame * pCurKF;
    while (1)
    {
        {
            unique_lock<mutex> lck_shutdown(shutDownMutex);
            if (shutDownFlag)
            {
                break;
            }
        }
        // std::cout<<"sssss"<<std::endl;
        if (bStop || mabIsUpdating)
        {
            //cout<<"loopbusy || bStop"<<endl;
            continue;
        }
        // std::cout<<"1111111"<<std::endl;
        int N;
        std::list<KeyFrame *> lNewKeyFrames;
        {
            unique_lock<mutex> lck(keyframeMutex);
            N = mlNewKeyFrames.size();
            lNewKeyFrames = mlNewKeyFrames;
            if(N == 0)
                continue;
            else
            {
                mlNewKeyFrames.clear();
            }

        }

        //SaveCameraPosition(lNewKeyFrames);
        
        // timeval start, finish; //定义开始，结束变量
        //初始化
        std::cout<<"待处理点云个数 = "<<N<<std::endl;
        double generatePointCloudTime = 0, transformPointCloudTime = 0; 
        for (auto pKF : lNewKeyFrames)
        {
            nKFCount++;
            if (pKF->isBad())
                continue;
            // gettimeofday(&start,NULL);
            if(pKF->imRightRgb.empty())
            {
                generatePointCloud(pKF);
            }else
            {
                generatePointCloudStereo(pKF);
            }
            
            if( kf->mptrPointCloud->points.size()<1)
            {
                continue;
            }
            // gettimeofday(&finish,NULL);//初始化结束时间
            // generatePointCloudTime += finish.tv_sec - start.tv_sec + (finish.tv_usec - start.tv_usec)/1000000.0;

            // gettimeofday(&start,NULL);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::transformPointCloud(*(pKF->mptrPointCloud), *(p), toMatrix4f(pKF->GetPoseInverse()));

            {
                std::unique_lock<std::mutex> lck(mMutexGlobalMap);
                *globalMap += *p;
            }
            // gettimeofday(&finish,NULL);//初始化结束时间
            // transformPointCloudTime += finish.tv_sec - start.tv_sec + (finish.tv_usec - start.tv_usec)/1000000.0;

            if(nKFCount%10 == 0)
            {
                std::string strSavePath = "/media/xxd/Data2/datasets/3d/za/";

                std::string strSaveName = strSavePath + std::to_string(nKFCount) + "_global.ply";
                pcl::PLYWriter writer;
	            writer.write(strSaveName, *globalMap);
                std::cout<<"save global pcl cloud, nKFCount:"<<nKFCount<<std::endl;
  
            }
        }
        // gettimeofday(&start,NULL);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);

        // 去除孤立点这个比较耗时，用处也不是很大，可以去掉
        // statistical_filter->setInputCloud(globalMap);  
        // statistical_filter->filter(*tmp);

        voxel->setInputCloud(globalMap);
        voxel->filter(*globalMap);
        // gettimeofday(&finish,NULL);//初始化结束时间
        // double filter = finish.tv_sec - start.tv_sec + (finish.tv_usec - start.tv_usec)/1000000.0;//转换浮点型
        // std::cout<<"filter: "<<filter<<std::endl;

        // std::cout<<"generatePointCloudTime: "<<generatePointCloudTime<<std::endl;
        // std::cout<<"transformPointCloudTime: "<<transformPointCloudTime<<std::endl;
        // gettimeofday(&start,NULL);
        viewer.showCloud(globalMap);  // 这个比较费时，建议不需要实时显示的可以屏蔽或改成几次显示一次
        // gettimeofday(&finish,NULL);//初始化结束时间
        // double duration = finish.tv_sec - start.tv_sec + (finish.tv_usec - start.tv_usec)/1000000.0;//转换浮点型
        // std::cout<<"showCloud: "<<duration<<std::endl;
       
        
        
    }

    cout<<"thread viewer end"<<endl;
}

// 保存地图的函数，需要的自行调用~
void PointCloudMapping::save()
{
    std::unique_lock<std::mutex> lck(mMutexGlobalMap);
    pcl::io::savePCDFile("result.pcd", *globalMap);
    cout << "globalMap save finished" << endl;
}
void PointCloudMapping::updatecloud(Map &curMap)
{
    std::unique_lock<std::mutex> lck(updateMutex);
    
    mabIsUpdating = true;
    currentvpKFs = curMap.GetAllKeyFrames();
    // loopbusy = true;
    cout << "开始点云更新" << endl;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmpGlobalMap(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr curPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmpGlobalMapFilter(new pcl::PointCloud<pcl::PointXYZRGBA>());
    for (int i = 0; i < currentvpKFs.size(); i++)
    {
        if (!mabIsUpdating)
        {
            std::cout << "中断点云更新" <<std::endl;
            return;
        }
        if (!currentvpKFs[i]->isBad() && currentvpKFs[i]->mptrPointCloud)
        {
            
            pcl::transformPointCloud(
                *(currentvpKFs[i]->mptrPointCloud), *(curPointCloud),
                toMatrix4f(currentvpKFs[i]->GetPoseInverse()));
            *tmpGlobalMap += *curPointCloud;

            voxel->setInputCloud(tmpGlobalMap);
            voxel->filter(*tmpGlobalMapFilter);
            tmpGlobalMap->swap(*tmpGlobalMapFilter);
        }
    }
    cout << "点云更新完成" << endl;
    {
        std::unique_lock<std::mutex> lck(mMutexGlobalMap);
        globalMap = tmpGlobalMap;
    }
    mabIsUpdating = false;
}

int PointCloudMapping::SetRectifiedQ(cv::Mat Q)
{
    m_stereoMatch.SetQ(Q);
    return 0;
}

////////////////////////////////////////////////////////

void PointCloudMapping::GenerateDepthMapThread()
{
    /*
    while(!shutDownFlag)
    {
        int N = 0;
        std::list<KeyFrame *> lNewKeyFrames;
        {
            unique_lock<mutex> lck(keyframeMutex);
            N = mlNewKeyFrames.size();
            lNewKeyFrames = mlNewKeyFrames;
            if(N == 0)
            {
                usleep(10*1000*1000); //10ms
                continue;
            }
                
            else
            {
                mlNewKeyFrames.clear();
            }

        }


    }*/

}





}
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

#include <chrono>
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
    : mabIsUpdating(false),m_fMaxDepth(1.0),m_fMinDepth(0.001), m_fFloatZero(0.000001)
    ,m_bIsBusy(false), m_bFirstReceived(false)
{
    m_stereoMatch.Init();
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
    m_newGlobalMap = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    globalCameraMap = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

    //viewerThread = make_shared<thread>(bind(&PointCloudMapping::viewer, this));

    m_depthMapThread = make_shared<thread>(bind(&PointCloudMapping::GenerateDepthMapThread, this));

    m_strSavePath = "/media/xxd/Data2/datasets/3d/za/";


   
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
    cout << "receive a keyframe, id" << kf->mnId << endl;
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

    if(!m_bFirstReceived)
    {
        m_bFirstReceived = true;
    }


    {
        kf->imDepth.release();
        unique_lock<mutex> lck(keyframeMutex);
        mlNewKeyFrames.emplace_back(kf);

    }

   
    
 
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
        p.y = Ow2.at<float>(1,0);
        p.z = Ow2.at<float>(2,0);
        //p.x = Ow2.at<float>(0,0);
        //p.y = Ow2.at<float>(0,1);
        //p.z = Ow2.at<float>(0,2);
            
            
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
            
            if( pKF->mptrPointCloud->points.size()<1)
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


int PointCloudMapping::SetRectifiedP(cv::Mat P)
{
    
    m_K = P;
    m_K.convertTo(m_K, CV_32F);
    std::cout<<"k:"<<m_K<<std::endl;
    return 0;
}

////////////////////////////////////////////////////////

bool PointCloudMapping::IsDepthSimilar(float d1, float d2, float fThreshold)
{
    float fDiff = fabs(d1-d2)/d1;

    return fDiff < fThreshold;
}

void PointCloudMapping::FilterDepthMap(KeyFrame* kf)
{
    // cv::Mat newDepthMap = cv::Mat::zeros(kf->imDepth.rows, kf->imDepth.cols, CV_32FC3);
    cv::Mat newDepthMap = kf->imDepth.clone();

    if(!m_mapNeighborDepth.count(kf->mnId))
    {
        std::cout<<"this key frame do not have neighbor depth map, id:"<< kf->mnId <<std::endl;
        return;
    }
    std::map<int, cv::Mat>& tmpNeighborDepthMap = m_mapNeighborDepth[kf->mnId];

    float fDepthDiffStrict = 0.02;
    int nMinViews = 2; //not include current depth map
    float fGoodViewRatio = 0.75;

     
            
    for(int i = 0; i < kf->imDepth.rows; i++)
    {
        for(int j = 0; j < kf->imDepth.cols; j++)
        {
            float depth = kf->imDepth.at<cv::Vec3f>(i,j)[2];
            if(depth <= m_fFloatZero)
            {
                continue;
            }

            //filter with neighbor depth map
            {
                int nGoodViews =0;
		        int nViews = 0;
            
                for(auto& it : tmpNeighborDepthMap)
                {
                    float z = it.second.at<cv::Vec3f>(i,j)[2];
                    if(z > 0)
                    {
                        ++nViews;
			    		if (IsDepthSimilar(depth, z, fDepthDiffStrict)) 
                        {
				    		// agrees with this neighbor
				    		++nGoodViews;
				    	}
                    }
                }

                if ((nGoodViews < nMinViews) || ((nGoodViews * 1.0) <= nViews * fGoodViewRatio)) 
                {
                    newDepthMap.at<cv::Vec3f>(i,j)[0] = 0.;
                    newDepthMap.at<cv::Vec3f>(i,j)[1] = 0.;
			    	newDepthMap.at<cv::Vec3f>(i,j)[2] = 0.;
			    }
            }

            {
                //filter with current depth map
                //todo
                int nGoodViews =0;
		        int nViews = 0;
            }
        }
    }
 
    newDepthMap.copyTo(kf->imDepth);

    std::string strSaveName = m_strSavePath + std::to_string(kf->mnId) + "_" + "filtered.ply";
    SavePCLCloud(kf->imLeftRgb, kf->imDepth, strSaveName);

}


void PointCloudMapping::ProjectFromNeighbor(KeyFrame* kf)
{
    int nTopN = 15;
    std::vector<KeyFrame*> vecNeighborKF = kf->GetBestCovisibilityKeyFrames(nTopN);
    cout<<"neighbor kf num:"<<vecNeighborKF.size()<<endl;
    //current kf
    cv::Mat curKFTcw = kf->GetPose();
    cv::Mat KT = m_K * curKFTcw;
    float pfc00 = KT.at<float>(0, 0); float pfc01 = KT.at<float>(0, 1); float pfc02 = KT.at<float>(0, 2); float pfc03 = KT.at<float>(0, 3);
    float pfc10 = KT.at<float>(1, 0); float pfc11 = KT.at<float>(1, 1); float pfc12 = KT.at<float>(1, 2); float pfc13 = KT.at<float>(1, 3);
    float pfc20 = KT.at<float>(2, 0); float pfc21 = KT.at<float>(2, 1); float pfc22 = KT.at<float>(2, 2); float pfc23 = KT.at<float>(2, 3);

    float hfc00 = curKFTcw.at<float>(0, 0); float hfc01 = curKFTcw.at<float>(0, 1); float hfc02 = curKFTcw.at<float>(0, 2); float hfc03 = curKFTcw.at<float>(0, 3);
    float hfc10 = curKFTcw.at<float>(1, 0); float hfc11 = curKFTcw.at<float>(1, 1); float hfc12 = curKFTcw.at<float>(1, 2); float hfc13 = curKFTcw.at<float>(1, 3);
    float hfc20 = curKFTcw.at<float>(2, 0); float hfc21 = curKFTcw.at<float>(2, 1); float hfc22 = curKFTcw.at<float>(2, 2); float hfc23 = curKFTcw.at<float>(2, 3);
                 

    for(auto& neighborKF : vecNeighborKF)
    {
        if(neighborKF->imDepth.empty())
        {
            cout<<"this neighbor does not have depth map, id: "<<neighborKF->mnId<<endl;
            continue;
        }

        if(m_mapNeighborDepth.count(kf->mnId))
        {
            std::map<int, cv::Mat>& tmpNeighborDepth = m_mapNeighborDepth[kf->mnId];
            if(tmpNeighborDepth.count(neighborKF->mnId))
            {
                 //alreay exist
                continue;
            }
        }
            

        cv::Mat neighbor2CurDepth = cv::Mat::zeros(kf->imDepth.rows, kf->imDepth.cols, CV_32FC3);
        cv::Mat Twc = neighborKF->GetPoseInverse();
        float Twc00 = Twc.at<float>(0, 0); float Twc01 = Twc.at<float>(0, 1); float Twc02 = Twc.at<float>(0, 2); float Twc03 = Twc.at<float>(0, 3); 
        float Twc10 = Twc.at<float>(1, 0); float Twc11 = Twc.at<float>(1, 1); float Twc12 = Twc.at<float>(1, 2); float Twc13 = Twc.at<float>(1, 3); 
        float Twc20 = Twc.at<float>(2, 0); float Twc21 = Twc.at<float>(2, 1); float Twc22 = Twc.at<float>(2, 2); float Twc23 = Twc.at<float>(2, 3); 
        
      
        for(int i = 0; i < neighborKF->imDepth.rows; i++)
        {
            for(int j = 0; j < neighborKF->imDepth.cols; j++)
            {
                //neighbor kf
                //cv::Vec3f xyzPixel = kf->imDepth.at<cv::Vec3f>(i, j);
                //cv::Mat x3Dc = (cv::Mat_<float>(3,1) << xyzPixel.val[0], xyzPixel.val[1], xyzPixel.val[2]);
                
                float neighborKFPosZ = neighborKF->imDepth.at<cv::Vec3f>(i, j)[2];
                if(neighborKFPosZ <= m_fMinDepth)
                {
                    continue;
                }
                float neighborKFPosX = neighborKF->imDepth.at<cv::Vec3f>(i, j)[0];
                float neighborKFPosY = neighborKF->imDepth.at<cv::Vec3f>(i, j)[1];

                //cv::Mat neighborWPos = Twc.rowRange(0,3).colRange(0,3) * x3Dc + Twc.rowRange(0,3).col(3);
                //cv::Mat neighborCurPos = curKFTcw.rowRange(0,3).colRange(0,3) * neighborWPos + curKFTcw.rowRange(0,3).col(3);
                
                //float x = neighborWPos.at<float>(0,0);
                //float y = neighborWPos.at<float>(1,0);
                //float z = neighborWPos.at<float>(2,0);
                float neighborWPosX =  Twc00 * neighborKFPosX + Twc01 * neighborKFPosY + Twc02 * neighborKFPosZ + Twc03;
                float neighborWPosY =  Twc10 * neighborKFPosX + Twc11 * neighborKFPosY + Twc12 * neighborKFPosZ + Twc13;
                float neighborWPosZ =  Twc20 * neighborKFPosX + Twc21 * neighborKFPosY + Twc22 * neighborKFPosZ + Twc23;

                
                float neighborCurPosZ = hfc20 * neighborWPosX + hfc21 * neighborWPosY + hfc22 * neighborWPosZ + hfc23;

                //if(fabs(z2 - neighborCurPos.at<float>(2,0)) > 0.000001)
                //{
               //     std::cout<<"this is not right"<<std::endl;
                //}

                if((neighborCurPosZ > m_fMinDepth) && (neighborCurPosZ < m_fMaxDepth))
                {
                    float w2 = pfc20 * neighborWPosX + pfc21 * neighborWPosY + pfc22 * neighborWPosZ + pfc23;
                    int u2 = (int)((pfc00 * neighborWPosX + pfc01 * neighborWPosY + pfc02 * neighborWPosZ + pfc03) / w2);
                    int v2 = (int)((pfc10 * neighborWPosX + pfc11 * neighborWPosY + pfc12 * neighborWPosZ + pfc13) / w2);

                    if (u2 > 0 && u2<kf->imDepth.cols && v2 > 0 && v2 < kf->imDepth.rows)
                    {
                        float neighborCurPosX = hfc00 * neighborWPosX + hfc01 * neighborWPosY + hfc02 * neighborWPosZ + hfc03;
                        float neighborCurPosY = hfc10 * neighborWPosX + hfc11 * neighborWPosY + hfc12 * neighborWPosZ + hfc13;

                        neighbor2CurDepth.at<cv::Vec3f>(v2,u2)[0] = neighborCurPosX;
                        neighbor2CurDepth.at<cv::Vec3f>(v2,u2)[1] = neighborCurPosY;
                        neighbor2CurDepth.at<cv::Vec3f>(v2,u2)[2] = neighborCurPosZ;
                    }
                }     
            }
        }

        m_mapNeighborDepth[kf->mnId][neighborKF->mnId] = neighbor2CurDepth;
        std::string strSaveName = m_strSavePath + std::to_string(kf->mnId) + "_" + std::to_string(neighborKF->mnId) + ".ply";

        //SavePCLCloud(kf->imLeftRgb, neighbor2CurDepth, strSaveName);
    }


    return;
}

bool PointCloudMapping::IsThreadBusy()
{
    return m_bIsBusy;
}

void PointCloudMapping::GenerateDepthMapThread()
{
    bool bFirstReceived = false;
    while(!shutDownFlag)
    {
        m_bIsBusy = false;
        usleep(1*1000*1000); //1ms
        int nKFNum = 0;
        std::list<KeyFrame *> lNewKeyFrames;
        {
            unique_lock<mutex> lck(keyframeMutex);
            nKFNum = mlNewKeyFrames.size();
            lNewKeyFrames = mlNewKeyFrames;
            if(nKFNum != 0)
            {
                mlNewKeyFrames.clear();
            }
        }

        if(0 == nKFNum)
        {
            usleep(10*1000*1000); //10ms
            continue;
        }

        //wait until all kf are received
        if(m_bFirstReceived && !bFirstReceived)
        {
            bFirstReceived = true;
            sleep(1); //1s
        }

        m_bIsBusy = true;

        //insert key frame to new map
        for(auto& kf : lNewKeyFrames)
        {
            if(m_mapKeyFrame.count(kf->mnId))
            {
                continue;
            }
            
            //it is a new key frame
            m_mapKeyFrame[kf->mnId] = kf;
            cout<<"get new key frame, id:"<<kf->mnId<<endl;
        }

        //compute depth map
        for(auto it = m_mapKeyFrame.begin(); it != m_mapKeyFrame.end(); ++it)
        {
            if(!it->second->imDepth.empty())
            {
                //already have depth map
                continue;
            }

            std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
            m_stereoMatch.ComputeDepthMap(it->second->imLeftRgb, it->second->imRightRgb, it->second->imDepth);
            std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
            auto time_d = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_end - time_start).count();
            cout<<"key frame compute depth map, id:"<<it->first<<"time elapse:"<< time_d<<endl;

            std::string strSaveName = m_strSavePath + std::to_string(it->second->mnId) + "_" + ".ply";
            SavePCLCloud(it->second->imLeftRgb, it->second->imDepth, strSaveName);
        }

       
        for(auto it = m_mapKeyFrame.begin(); it != m_mapKeyFrame.end(); ++it)
        {
            
            cout<<"project neighbor depth map to current, id:"<<it->first<<endl;
            //project neighbor depth map to current 
            std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
            ProjectFromNeighbor(it->second);
            std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
            auto time_d = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_end - time_start).count();
            cout<<"ProjectFromNeighbor time elapse:"<< time_d<<endl;

            //filter depth map
            FilterDepthMap(it->second);

            std::chrono::steady_clock::time_point time_2 = std::chrono::steady_clock::now();
            auto time_d2 = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_2 - time_end).count();
            cout<<"FilterDepthMap time elapse:"<< time_d2<<endl;
        }

        FusePCLCloud();

    }
}



void PointCloudMapping::SavePCLCloud(cv::Mat& img, cv::Mat& xyz, std::string strFileName)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // point cloud is null ptr
   for (int m = 0; m < xyz.rows; m++)
    {
        for (int n = 0; n < xyz.cols; n++)
        {
            pcl::PointXYZRGBA p;
            //pcl::PointXYZRGB p;
            
            cv::Vec3f xyzPixel = xyz.at<cv::Vec3f>(m, n);
            p.z = xyzPixel.val[2];
            /*
            if(p.z > m_fMaxDepth || p.z <  m_fMinDepth)
            {
                //std::cout<<p.z<<std::endl;
                continue;
            }*/
            p.x = xyzPixel.val[0];
            p.y = xyzPixel.val[1];

            cv::Vec3b bgrPixel = img.at<cv::Vec3b>(m, n);
            p.b = bgrPixel.val[0];
            p.g = bgrPixel.val[1];
            p.r = bgrPixel.val[2];
          

            pPointCloud->points.push_back(p);
        }
    }
    pPointCloud->height = 1;
    pPointCloud->width = pPointCloud->points.size();
    pPointCloud->is_dense = true;
    //kf->mptrPointCloud = pPointCloud;

/*
    std::string strSavePath = "/media/xxd/Data2/datasets/3d/za/";
    //strFilePath = strSavePath + "pclPointCloud.ply";
    static int i = 0;
    i++;
    std::string strSaveName = strSavePath + std::to_string(i) + ".ply";
    */

    pcl::PLYWriter writer;
	writer.write(strFileName, *pPointCloud);
   
   
     return;
}

void PointCloudMapping::FusePCLCloud()
{
    for(auto it = m_mapKeyFrame.begin(); it != m_mapKeyFrame.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        cv::Mat& depthMap = it->second->imDepth;
        cv::Mat& rgb = it->second->imLeftRgb;
          
        for (int m = 0; m < depthMap.rows; m++)
        {
        for (int n = 0; n < depthMap.cols; n++)
        {
            pcl::PointXYZRGBA p;
            //pcl::PointXYZRGB p;
            
            cv::Vec3f xyzPixel = depthMap.at<cv::Vec3f>(m, n);
            p.z = xyzPixel.val[2];
            if(p.z > m_fMaxDepth || p.z <  m_fMinDepth)
            {
                //std::cout<<p.z<<std::endl;
                continue;
            }
            p.x = xyzPixel.val[0];
            p.y = xyzPixel.val[1];

            cv::Vec3b bgrPixel = rgb.at<cv::Vec3b>(m, n);
           
            p.b = bgrPixel.val[0];
            p.g = bgrPixel.val[1];
            p.r = bgrPixel.val[2];
          

            pPointCloud->points.push_back(p);
        }
        }
        pPointCloud->height = 1;
        pPointCloud->width = pPointCloud->points.size();
        pPointCloud->is_dense = true;
        pcl::transformPointCloud(*(pPointCloud), *(pCloud), toMatrix4f(it->second->GetPoseInverse()));
        {
            std::unique_lock<std::mutex> lck(m_MutexNewGlobalMap);
            *m_newGlobalMap += *pCloud;
        }
        
           
    }

    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);

    // 去除孤立点这个比较耗时，用处也不是很大，可以去掉
    // statistical_filter->setInputCloud(globalMap);  
    // statistical_filter->filter(*tmp);

    voxel->setInputCloud(m_newGlobalMap);
    voxel->filter(*m_newGlobalMap);

    std::string strSaveName = m_strSavePath + "_global.ply";
    pcl::PLYWriter writer;
	writer.write(strSaveName, *m_newGlobalMap);
    std::cout<<"save global pcl cloud"<<std::endl;
  
}


}
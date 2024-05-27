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

#include <sys/time.h>
namespace ORB_SLAM2
{
    // int currentloopcount = 0;
    PointCloudMapping::PointCloudMapping(double resolution_, double meank_, double thresh_)
        : mabIsUpdating(false), m_fMaxDepth(1.0), m_fMinDepth(0.001), m_fFloatZero(0.000001), m_bIsBusy(false), m_bFirstReceived(false)
    {
        m_bIsSaveData = true;
        m_bUseCuda = false;
        m_fDepthDiffStrict = 0.01;
        m_nMinViewNum = 4;
        m_fGoodViewRatio = 0.75;
        if (m_bUseCuda)
        {
            m_stereoMatchCuda.Init();
        }
        else
        {
            m_stereoMatch.Init();
        }

        shutDownFlag = false;
        this->resolution = resolution_;
        this->meank = meank_;
        this->thresh = thresh_;
        std::cout << resolution << " " << meank << " " << thresh << std::endl;
        statistical_filter = new pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA>(true);
        voxel = new pcl::VoxelGrid<pcl::PointXYZRGBA>();
        statistical_filter->setMeanK(meank);
        statistical_filter->setStddevMulThresh(thresh);
        voxel->setLeafSize(resolution, resolution, resolution);
        globalMap = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
        m_newGlobalMap = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
        globalCameraMap = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

        // viewerThread = make_shared<thread>(bind(&PointCloudMapping::viewer, this));

#ifdef REAL_TIME_GENERATE
        m_realTimeDepthMapThread = make_shared<thread>(bind(&PointCloudMapping::RealTimeGeneratePointCloudThread, this));
#else
        m_depthMapThread = make_shared<thread>(bind(&PointCloudMapping::GenerateDepthMapThread, this));
#endif

        //

        

        m_strSavePath = "/media/xxd/Data2/datasets/3d/za/";
    }

    PointCloudMapping::~PointCloudMapping()
    {
        shutdown();
    }

    void PointCloudMapping::shutdown()
    {
        {
            unique_lock<mutex> lck(shutDownMutex);
            shutDownFlag = true;
            //keyFrameUpdated.notify_one();
        }
        //viewerThread->join();
#ifdef REAL_TIME_GENERATE
        m_realTimeDepthMapThread->join();
#else
        m_depthMapThread->join();
#endif
 
    }

    void PointCloudMapping::Clear()
    {
        std::cout << "清除稠密地图" << std::endl;
        std::unique_lock<std::mutex> lck(mMutexGlobalMap);
        globalMap.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    }
    

    void PointCloudMapping::insertKeyFrame(KeyFrame *kf)
    {
       
        if (kf->imLeftRgb.empty())
        {
            cout << "kf->imLeftRgb.empty()" << endl;
            return;
        }
        if (kf->isBad())
        {
            //cout << "this frame is bad!" << endl;
            return;
        }

        if (!m_bFirstReceived)
        {
            m_bFirstReceived = true;
            sleep(1);
        }
        //cout << "receive a keyframe, id" << kf->mnId << endl;

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
                    // cout<<"depth is not valid:"<<d << endl;
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
        if (kf->imLeftRgb.channels() == 1)
        {
            cv::cvtColor(kf->imLeftRgb, kf->imLeftRgb, cv::COLOR_GRAY2BGR);
            cv::cvtColor(kf->imRightRgb, kf->imRightRgb, cv::COLOR_GRAY2BGR);
        }

        //
        if (m_bUseCuda)
        {

            m_stereoMatchCuda.ComputeDepthMap(kf->imLeftRgb, kf->imRightRgb, kf->imDepth);
        }
        else
        {

            m_stereoMatch.ComputeDepthMap(kf->imLeftRgb, kf->imRightRgb, kf->imDepth);
        }

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        //  point cloud is null ptr
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
                if (p.z > 0.8)
                {
                    // std::cout<<p.z<<std::endl;
                    continue;
                }
                // std::cout<<"  "<<p.z;//<<std::endl;

                if (p.z < 0.005)
                {
                    // std::cout<<"p.z < 1 "<<std::endl;
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

        std::cout << "point size :" << pPointCloud->points.size() << std::endl;
    }

    void PointCloudMapping::SaveCameraPosition(std::list<KeyFrame *> &lNewKeyFrames)
    {
        for (auto pKF : lNewKeyFrames)
        {
            // Eigen::Vector3f Ow2 = pKF->GetCameraCenter();
            cv::Mat Ow2 = pKF->GetCameraCenter();

            pcl::PointXYZRGBA p;

            p.x = Ow2.at<float>(0, 0);
            p.y = Ow2.at<float>(1, 0);
            p.z = Ow2.at<float>(2, 0);
            // p.x = Ow2.at<float>(0,0);
            // p.y = Ow2.at<float>(0,1);
            // p.z = Ow2.at<float>(0,2);

            p.b = 127;
            p.g = 127;
            p.r = 127;

            globalCameraMap->points.push_back(p);
        }

        globalCameraMap->height = 1;
        globalCameraMap->width = globalCameraMap->points.size();
        globalCameraMap->is_dense = false;

        if (m_bIsSaveData)
        {
            // std::string strSavePath = "/media/xxd/Data2/datasets/3d/za/";

            std::string strSaveName = m_strSavePath + "camera_position.ply";
            pcl::PLYWriter writer;
            writer.write(strSaveName, *globalCameraMap);
        }
    }

    Eigen::Matrix<float, 4, 4> toMatrix4f(const cv::Mat &cvMat4)
    {
        Eigen::Matrix<float, 4, 4> M;

        M << cvMat4.at<float>(0, 0), cvMat4.at<float>(0, 1), cvMat4.at<float>(0, 2), cvMat4.at<float>(0, 3),
            cvMat4.at<float>(1, 0), cvMat4.at<float>(1, 1), cvMat4.at<float>(1, 2), cvMat4.at<float>(1, 3),
            cvMat4.at<float>(2, 0), cvMat4.at<float>(2, 1), cvMat4.at<float>(2, 2), cvMat4.at<float>(2, 3),
            cvMat4.at<float>(3, 0), cvMat4.at<float>(3, 1), cvMat4.at<float>(3, 2), cvMat4.at<float>(3, 3);

        return M;
    }

    void PointCloudMapping::viewer()
    {
        pcl::visualization::CloudViewer viewer("viewer");
        int nKFCount = 0;
        cout << "thread viewer start" << endl;
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
                // cout<<"loopbusy || bStop"<<endl;
                continue;
            }
            // std::cout<<"1111111"<<std::endl;
            int N;
            std::list<KeyFrame *> lNewKeyFrames;
            {
                unique_lock<mutex> lck(keyframeMutex);
                N = mlNewKeyFrames.size();
                lNewKeyFrames = mlNewKeyFrames;
                if (N == 0)
                    continue;
                else
                {
                    mlNewKeyFrames.clear();
                }
            }

            // SaveCameraPosition(lNewKeyFrames);

            // timeval start, finish; //定义开始，结束变量
            // 初始化
            std::cout << "待处理点云个数 = " << N << std::endl;
            double generatePointCloudTime = 0, transformPointCloudTime = 0;
            for (auto pKF : lNewKeyFrames)
            {
                nKFCount++;
                if (pKF->isBad())
                    continue;
                // gettimeofday(&start,NULL);
                if (pKF->imRightRgb.empty())
                {
                    generatePointCloud(pKF);
                }
                else
                {
                    generatePointCloudStereo(pKF);
                }

                if (pKF->mptrPointCloud->points.size() < 1)
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

                if (nKFCount % 10 == 0)
                {

                    if (m_bIsSaveData)
                    {
                        // std::string strSavePath = "/media/xxd/Data2/datasets/3d/za/";

                        std::string strSaveName = m_strSavePath + std::to_string(nKFCount) + "_global.ply";
                        pcl::PLYWriter writer;
                        writer.write(strSaveName, *globalMap);
                        std::cout << "save global pcl cloud, nKFCount:" << nKFCount << std::endl;
                    }
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
            viewer.showCloud(globalMap); // 这个比较费时，建议不需要实时显示的可以屏蔽或改成几次显示一次
            // gettimeofday(&finish,NULL);//初始化结束时间
            // double duration = finish.tv_sec - start.tv_sec + (finish.tv_usec - start.tv_usec)/1000000.0;//转换浮点型
            // std::cout<<"showCloud: "<<duration<<std::endl;
        }

        cout << "thread viewer end" << endl;
    }

    // 保存地图的函数，需要的自行调用~
   // void PointCloudMapping::save()
   ////////////////////////////////////////////////// {
    //    std::unique_lock<std::mutex> lck(mMutexGlobalMap);
   //     pcl::io::savePCDFile("result.pcd", *globalMap);
   //     cout << "globalMap save finished" << endl;
   // }
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
                std::cout << "中断点云更新" << std::endl;
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
        //

        if (m_bUseCuda)
        {

            m_stereoMatchCuda.SetQ(Q);
        }
        else
        {

            m_stereoMatch.SetQ(Q);
        }
        return 0;
    }

    int PointCloudMapping::SetRectifiedP(cv::Mat P)
    {

        m_K = P;
        m_K.convertTo(m_K, CV_32F);
        std::cout << "k:" << m_K << std::endl;
        return 0;
    }

    ////////////////////////////////////////////////////////

    bool PointCloudMapping::IsDepthSimilar(float d1, float d2, float fThreshold)
    {
        // depth should be positive and greater than zero
        if (d1 <= m_fFloatZero || d2 <= m_fFloatZero)
        {
            return false;
        }

        float fDiff = fabs(d1 - d2) / d1;

        return fDiff < fThreshold;
    }

    void PointCloudMapping::DeleteBadKF()
    {
        std::map<int, KeyFrame *>::iterator it;
        
        for(it = m_mapKeyFrame.begin(); it != m_mapKeyFrame.end();)
        {
            

            if(it->second->isBad())
            {
                //erase bad key frame and its neighbors in m_mapNeighborDepth
                if (m_mapNeighborDepth.count(it->second->mnId))
                {
                    m_mapNeighborDepth[it->second->mnId].clear();
                    m_mapNeighborDepth.erase(it->second->mnId);
                }

                //if this kf is neighbor of other kf, erase it
                std::map<int, std::map<int, cv::Mat>>::iterator iter;
                for(iter = m_mapNeighborDepth.begin(); iter != m_mapNeighborDepth.end();iter++)
                {
                    if(iter->second.count(it->second->mnId))
                    {
                        iter->second[it->second->mnId].release();

                        iter->second.erase(it->second->mnId);
                        
                    }          
                }  

                it->second->imLeftRgb.release();
                it->second->imRightRgb.release();
                it->second->imDepth.release();

                //erase bad key frame in m_mapKeyFrame
                cout<<"delete this bad kf from m_mapKeyFrame, id:"<<it->second->mnId<<endl;
                it = m_mapKeyFrame.erase(it);
            }else{

                ++it;
            }
        }

        /*
        cout<<"m_mapKeyFrame size:"<<m_mapKeyFrame.size()<<endl;
        std::map<int, std::map<int, cv::Mat>>::iterator itor;
        for(itor = m_mapNeighborDepth.begin(); itor != m_mapNeighborDepth.end();itor++)
        {
            cout<<"kf id:"<<itor->first<<", map size:"<< itor->second.size()<<endl;
                        
        }  
        */

         

        

    }

    void PointCloudMapping::FilterDepthMap(KeyFrame *kf)
    {
        if (kf->isBad())
        {
            cout << "this is bad kf, id:"<<kf->mnId << endl;
            return;
        }
        // cv::Mat newDepthMap = cv::Mat::zeros(kf->imDepth.rows, kf->imDepth.cols, CV_32FC3);
        cv::Mat newDepthMap = kf->imDepth.clone();

        if (!m_mapNeighborDepth.count(kf->mnId))
        {
            std::cout << "this key frame do not have neighbor depth map, id:" << kf->mnId << std::endl;
            return;
        }
        std::map<int, cv::Mat> &tmpNeighborDepthMap = m_mapNeighborDepth[kf->mnId];

        // float fDepthDiffStrict = 0.02;
        // int nMinViews = 2; //not include current depth map
        // float fGoodViewRatio = 0.75;
        bool bAdjust = true;

        for (int i = 0; i < kf->imDepth.rows; i++)
        {
            for (int j = 0; j < kf->imDepth.cols; j++)
            {
                float depth = kf->imDepth.at<cv::Vec3f>(i, j)[2];
                if (depth <= m_fFloatZero)
                {
                    continue;
                }

                // filter with neighbor depth map
                {
                    int nGoodViews = 0;
                    int nViews = 0;
                    std::vector<float> tmpVec;
                    tmpVec.push_back(depth);

                    for (auto &it : tmpNeighborDepthMap)
                    {
                        
                        float z = it.second.at<cv::Vec3f>(i, j)[2];
                        if (z > 0)
                        {
                            ++nViews;
                            if (IsDepthSimilar(depth, z, m_fDepthDiffStrict))
                            {
                                // agrees with this neighbor
                                ++nGoodViews;
                                tmpVec.push_back(z);
                            }
                        }
                    }

                    if ((nGoodViews < m_nMinViewNum) || ((nGoodViews * 1.0) <= nViews * m_fGoodViewRatio))
                    {
                        newDepthMap.at<cv::Vec3f>(i, j)[0] = 0.;
                        newDepthMap.at<cv::Vec3f>(i, j)[1] = 0.;
                        newDepthMap.at<cv::Vec3f>(i, j)[2] = 0.;
                    }
                    else
                    {
                        if (bAdjust)
                        {
                            float fMean = std::accumulate(tmpVec.begin(), tmpVec.end(), 0.0) / tmpVec.size();
                            newDepthMap.at<cv::Vec3f>(i, j)[2] = fMean;
                        }
                    }
                }

                {
                    // filter with current depth map
                    // todo
                    int nGoodViews = 0;
                    int nViews = 0;
                }
            }
        }

        newDepthMap.copyTo(kf->imDepth);
        if (m_bIsSaveData)
        {
            std::string strSaveName = m_strSavePath + std::to_string(kf->mnId) + "_" + "filtered.ply";
            SavePCLCloud(kf->imLeftRgb, kf->imDepth, strSaveName);
        }
        /*

        {
            // free map memory
            std::map<int, std::map<int, cv::Mat>>::iterator it;
            for (it = m_mapNeighborDepth.begin(); it != m_mapNeighborDepth.end(); it++)
            {
                std::map<int, cv::Mat>::iterator iter;
                for (iter = it->second.begin(); iter != it->second.end(); iter++)
                {
                    iter->second.release();
                }
            }

            m_mapNeighborDepth.clear();
        }
        */
    }

    bool PointCloudMapping::IsPoseChange(KeyFrame *kf, KeyFrame *NeighborKf)
    {
        int nId = kf->mnId * 100000 + NeighborKf->mnId;
        cv::Mat Toldc1c2 = kf->GetPose() * NeighborKf->GetPoseInverse();  //Tc1w * Twc2 
        cv::Mat Tnewc1c2 = m_mapKF2NeighborPose[nId];
        bool bIsChanged = false;
        float fEqualThreshold = 0.000001;
        for(int i = 0; i < Tnewc1c2.rows; i++)
        {
            for(int j = 0; j < Tnewc1c2.cols; j++)
            {
                if(fabs(Toldc1c2.at<float>(i, j) - Tnewc1c2.at<float>(i, j)) > fEqualThreshold)
                {
                    bIsChanged = true;
                    break;
                }
            }

            if(bIsChanged)
            {
                break;
            }
        }


        return bIsChanged;
    }

    void PointCloudMapping::ProjectFromNeighbor(KeyFrame *kf)
    {
        if (kf->isBad())
        {
            cout << "this is bad kf, id:"<<kf->mnId << endl;
            return;
        }

        int nTopN = 15;
        std::vector<KeyFrame *> vecNeighborKF = kf->GetBestCovisibilityKeyFrames(nTopN);
        //cout << "neighbor kf num:" << vecNeighborKF.size() << endl;
        // current kf
        cv::Mat curKFTcw = kf->GetPose();
        cv::Mat KT = m_K * curKFTcw;
        float pfc00 = KT.at<float>(0, 0);
        float pfc01 = KT.at<float>(0, 1);
        float pfc02 = KT.at<float>(0, 2);
        float pfc03 = KT.at<float>(0, 3);
        float pfc10 = KT.at<float>(1, 0);
        float pfc11 = KT.at<float>(1, 1);
        float pfc12 = KT.at<float>(1, 2);
        float pfc13 = KT.at<float>(1, 3);
        float pfc20 = KT.at<float>(2, 0);
        float pfc21 = KT.at<float>(2, 1);
        float pfc22 = KT.at<float>(2, 2);
        float pfc23 = KT.at<float>(2, 3);

        float hfc00 = curKFTcw.at<float>(0, 0);
        float hfc01 = curKFTcw.at<float>(0, 1);
        float hfc02 = curKFTcw.at<float>(0, 2);
        float hfc03 = curKFTcw.at<float>(0, 3);
        float hfc10 = curKFTcw.at<float>(1, 0);
        float hfc11 = curKFTcw.at<float>(1, 1);
        float hfc12 = curKFTcw.at<float>(1, 2);
        float hfc13 = curKFTcw.at<float>(1, 3);
        float hfc20 = curKFTcw.at<float>(2, 0);
        float hfc21 = curKFTcw.at<float>(2, 1);
        float hfc22 = curKFTcw.at<float>(2, 2);
        float hfc23 = curKFTcw.at<float>(2, 3);

        for (auto &neighborKF : vecNeighborKF)
        {
            if (neighborKF->isBad())
            {
                cout << "this neighbor is a bad key frame, id: " << neighborKF->mnId << endl;
                continue;
            }

            if (neighborKF->imDepth.empty())
            {
                //cout << "this neighbor does not have depth map, id: " << neighborKF->mnId << endl;
                continue;
            }

            if (m_mapNeighborDepth.count(kf->mnId))
            {
                std::map<int, cv::Mat> &tmpNeighborDepth = m_mapNeighborDepth[kf->mnId];
                if (tmpNeighborDepth.count(neighborKF->mnId)  )
                {
                    // alreay exist
                    if(!IsPoseChange(kf, neighborKF))
                    {
                        //pose is not changed since last time update this depth
                        continue;

                    }
                    
                }
            }

            cv::Mat neighbor2CurDepth = cv::Mat::zeros(kf->imDepth.rows, kf->imDepth.cols, CV_32FC3);
            cv::Mat Twc = neighborKF->GetPoseInverse();
            float Twc00 = Twc.at<float>(0, 0);
            float Twc01 = Twc.at<float>(0, 1);
            float Twc02 = Twc.at<float>(0, 2);
            float Twc03 = Twc.at<float>(0, 3);
            float Twc10 = Twc.at<float>(1, 0);
            float Twc11 = Twc.at<float>(1, 1);
            float Twc12 = Twc.at<float>(1, 2);
            float Twc13 = Twc.at<float>(1, 3);
            float Twc20 = Twc.at<float>(2, 0);
            float Twc21 = Twc.at<float>(2, 1);
            float Twc22 = Twc.at<float>(2, 2);
            float Twc23 = Twc.at<float>(2, 3);

            //cout << "neighbor id: " << neighborKF->mnId << " w:"<<neighborKF->imDepth.cols<< "h:"<<neighborKF->imDepth.rows << endl;

            for (int i = 0; i < neighborKF->imDepth.rows; i++)
            {
                for (int j = 0; j < neighborKF->imDepth.cols; j++)
                {
                    // neighbor kf
                    // cv::Vec3f xyzPixel = kf->imDepth.at<cv::Vec3f>(i, j);
                    // cv::Mat x3Dc = (cv::Mat_<float>(3,1) << xyzPixel.val[0], xyzPixel.val[1], xyzPixel.val[2]);

                    float neighborKFPosZ = neighborKF->imDepth.at<cv::Vec3f>(i, j)[2];
                    if (neighborKFPosZ <= m_fMinDepth)
                    {
                        continue;
                    }
                    float neighborKFPosX = neighborKF->imDepth.at<cv::Vec3f>(i, j)[0];
                    float neighborKFPosY = neighborKF->imDepth.at<cv::Vec3f>(i, j)[1];

                    // cv::Mat neighborWPos = Twc.rowRange(0,3).colRange(0,3) * x3Dc + Twc.rowRange(0,3).col(3);
                    // cv::Mat neighborCurPos = curKFTcw.rowRange(0,3).colRange(0,3) * neighborWPos + curKFTcw.rowRange(0,3).col(3);

                    // float x = neighborWPos.at<float>(0,0);
                    // float y = neighborWPos.at<float>(1,0);
                    // float z = neighborWPos.at<float>(2,0);
                    float neighborWPosX = Twc00 * neighborKFPosX + Twc01 * neighborKFPosY + Twc02 * neighborKFPosZ + Twc03;
                    float neighborWPosY = Twc10 * neighborKFPosX + Twc11 * neighborKFPosY + Twc12 * neighborKFPosZ + Twc13;
                    float neighborWPosZ = Twc20 * neighborKFPosX + Twc21 * neighborKFPosY + Twc22 * neighborKFPosZ + Twc23;

                    float neighborCurPosZ = hfc20 * neighborWPosX + hfc21 * neighborWPosY + hfc22 * neighborWPosZ + hfc23;

                    // if(fabs(z2 - neighborCurPos.at<float>(2,0)) > 0.000001)
                    //{
                    //     std::cout<<"this is not right"<<std::endl;
                    //}

                    if ((neighborCurPosZ > m_fMinDepth) && (neighborCurPosZ < m_fMaxDepth))
                    {
                        float w2 = pfc20 * neighborWPosX + pfc21 * neighborWPosY + pfc22 * neighborWPosZ + pfc23;
                        int u2 = (int)((pfc00 * neighborWPosX + pfc01 * neighborWPosY + pfc02 * neighborWPosZ + pfc03) / w2);
                        int v2 = (int)((pfc10 * neighborWPosX + pfc11 * neighborWPosY + pfc12 * neighborWPosZ + pfc13) / w2);

                        if (u2 > 0 && u2 < kf->imDepth.cols && v2 > 0 && v2 < kf->imDepth.rows)
                        {
                            float neighborCurPosX = hfc00 * neighborWPosX + hfc01 * neighborWPosY + hfc02 * neighborWPosZ + hfc03;
                            float neighborCurPosY = hfc10 * neighborWPosX + hfc11 * neighborWPosY + hfc12 * neighborWPosZ + hfc13;

                            neighbor2CurDepth.at<cv::Vec3f>(v2, u2)[0] = neighborCurPosX;
                            neighbor2CurDepth.at<cv::Vec3f>(v2, u2)[1] = neighborCurPosY;
                            neighbor2CurDepth.at<cv::Vec3f>(v2, u2)[2] = neighborCurPosZ;
                        }
                    }
                }
            }

            m_mapNeighborDepth[kf->mnId][neighborKF->mnId] = neighbor2CurDepth;
            {
                int nId = kf->mnId * 100000 + neighborKF->mnId;
                cv::Mat Tc1c2 = kf->GetPose() * neighborKF->GetPoseInverse();  //Tc1w * Twc2 
                m_mapKF2NeighborPose[nId] = Tc1c2;
            }
            //std::string strSaveName = m_strSavePath + std::to_string(kf->mnId) + "_" + std::to_string(neighborKF->mnId) + ".ply";

            // SavePCLCloud(kf->imLeftRgb, neighbor2CurDepth, strSaveName);
        }

        return;
    }

    bool PointCloudMapping::IsThreadBusy()
    {
        return m_bIsBusy;
    }

    void PointCloudMapping::GenerateDepthMapThread()
    {
        pcl::visualization::CloudViewer viewer("viewer");
        bool bFirstReceived = false;
        while (!shutDownFlag)
        {
            m_bIsBusy = false;
            usleep(1 * 1000 * 1000); // 1ms
            int nKFNum = 0;

            // wait until all kf are received
            if (m_bFirstReceived && !bFirstReceived)
            {
                bFirstReceived = true;
                sleep(1); // 1s
            }

            std::list<KeyFrame *> lNewKeyFrames;
            {
                unique_lock<mutex> lck(keyframeMutex);
                nKFNum = mlNewKeyFrames.size();
                lNewKeyFrames = mlNewKeyFrames;
                if (nKFNum != 0)
                {
                    mlNewKeyFrames.clear();
                }
            }

            if (0 == nKFNum)
            {
                usleep(10 * 1000 * 1000); // 10ms
                continue;
            }

            

            m_bIsBusy = true;

            // insert key frame to new map
            for (auto &kf : lNewKeyFrames)
            {
                if (m_mapKeyFrame.count(kf->mnId))
                {
                    continue;
                }

                // it is a new key frame
                m_mapKeyFrame[kf->mnId] = kf;
                cout << "get new key frame, id:" << kf->mnId << endl;
            }

            // compute depth map
            std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
            for (auto it = m_mapKeyFrame.begin(); it != m_mapKeyFrame.end(); ++it)
            {
                if (!it->second->imDepth.empty())
                {
                    // already have depth map
                    continue;
                }

                
                //
                it->second->imDepth.create(it->second->imLeftRgb.size(), it->second->imLeftRgb.type());
                if (m_bUseCuda)
                {
                    m_stereoMatchCuda.ComputeDepthMap(it->second->imLeftRgb, it->second->imRightRgb, it->second->imDepth);
                }
                else
                {
                    m_stereoMatch.ComputeDepthMap(it->second->imLeftRgb, it->second->imRightRgb, it->second->imDepth);
                }
               
                if (m_bIsSaveData)
                {
                    std::string strSaveName = m_strSavePath + std::to_string(it->second->mnId) + "_" + ".ply";
                    SavePCLCloud(it->second->imLeftRgb, it->second->imDepth, strSaveName);
                }
            }
            std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
            auto time_d = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_end - time_start).count();
            cout << "compute depth map,time elapse:" << time_d << endl;

            for (auto it = m_mapKeyFrame.begin(); it != m_mapKeyFrame.end(); ++it)
            {

                //cout << "project neighbor depth map to current, id:" << it->first << endl;
                // project neighbor depth map to current
                //std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
                ProjectFromNeighbor(it->second);
                //std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
                //auto time_d = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_end - time_start).count();
                //cout << "ProjectFromNeighbor time elapse:" << time_d << endl;

                // filter depth map
                FilterDepthMap(it->second);

                //std::chrono::steady_clock::time_point time_2 = std::chrono::steady_clock::now();
                //auto time_d2 = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_2 - time_end).count();
                //cout << "FilterDepthMap time elapse:" << time_d2 << endl;
            }
            std::chrono::steady_clock::time_point project_time = std::chrono::steady_clock::now();
            auto project_time_d2 = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(project_time - time_end).count();
            cout << "project and filter time elapse:" << project_time_d2 << endl;

            DeleteBadKF();
            /*

            FusePCLCloud();

            viewer.showCloud(m_newGlobalMap);
            sleep(5);*/
            FusePCLCloud2();
        }
    }

    void PointCloudMapping::SavePCLCloud(cv::Mat &img, cv::Mat &xyz, std::string strFileName)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        //  point cloud is null ptr
        for (int m = 0; m < xyz.rows; m++)
        {
            for (int n = 0; n < xyz.cols; n++)
            {
                pcl::PointXYZRGBA p;
                // pcl::PointXYZRGB p;

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
        // kf->mptrPointCloud = pPointCloud;

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
        for (auto it = m_mapKeyFrame.begin(); it != m_mapKeyFrame.end(); ++it)
        {
            if(it->second->isBad())
            {
                cout << "this is bad kf, id:"<<it->second->mnId << endl;
                continue;
            }

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            cv::Mat &depthMap = it->second->imDepth;
            cv::Mat &rgb = it->second->imLeftRgb;

            for (int m = 0; m < depthMap.rows; m++)
            {
                for (int n = 0; n < depthMap.cols; n++)
                {
                    pcl::PointXYZRGBA p;
                    // pcl::PointXYZRGB p;

                    cv::Vec3f xyzPixel = depthMap.at<cv::Vec3f>(m, n);
                    p.z = xyzPixel.val[2];
                    if (p.z > m_fMaxDepth || p.z < m_fMinDepth)
                    {
                        // std::cout<<p.z<<std::endl;
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
            if(pPointCloud->points.size() > 1)
            {
                pcl::transformPointCloud(*(pPointCloud), *(pCloud), toMatrix4f(it->second->GetPoseInverse()));
                std::unique_lock<std::mutex> lck(m_MutexNewGlobalMap);
                *m_newGlobalMap += *pCloud;
            }
            
        }

        // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);

        // 去除孤立点这个比较耗时，用处也不是很大，可以去掉
        // statistical_filter->setInputCloud(globalMap);
        // statistical_filter->filter(*tmp);
        if(m_newGlobalMap->points.size())
        {
            voxel->setInputCloud(m_newGlobalMap);
            voxel->filter(*m_newGlobalMap);
        }

        

        //if (m_bIsSaveData && m_newGlobalMap->points.size())
        {
            std::string strSaveName = m_strSavePath + "_global.ply";
            pcl::PLYWriter writer;
            writer.write(strSaveName, *m_newGlobalMap);
            std::cout << "save global pcl cloud" << std::endl;
        }
    }

    void PointCloudMapping::FusePCLCloud2()
    {
        int n = m_mapKeyFrame.size();
        if (0 == n)
        {
            std::cout << "m_mapKeyFrame.size() == 0" << std::endl;
            return;
        }
        int w = 0;
        int h = 0;
        int nTopNum = 20;

        std::vector<std::pair<KeyFrame *, int>> vecMap;
        // std::vector<PAIR> vecMap;
        for (auto &it : m_mapKeyFrame)
        {
            if (0 == w || 0 == h)
            {
                w = it.second->imDepth.cols;
                h = it.second->imDepth.rows;
            }

            vecMap.push_back(std::pair<KeyFrame *, int>(it.second, it.second->GetConnectionsKFNum()));
            std::cout << "id:" << it.second->mnId << " kf num:" << it.second->GetConnectionsKFNum() << std::endl;
        }

        std::sort(vecMap.begin(), vecMap.end(), [](std::pair<KeyFrame *, int> a, std::pair<KeyFrame *, int> b)
                  { return a.second > b.second; });

        std::cout << "afer sort" << std::endl;

        std::map<int, int> mapKFId2VecIndex;

        for (int i = 0; i < vecMap.size(); i++)
        {
            std::cout << "id:" << vecMap[i].first->mnId << " kf num:" << vecMap[i].second << std::endl;
            mapKFId2VecIndex[vecMap[i].first->mnId] = i;
        }

        // int *arrIsDepthUsed = new int[n * h * w]; //hundreds m Byte
        // memset(arrIsDepthUsed, 0, n * h * w);
        std::vector<int> arrIsDepthUsed(n * h * w, 0);
        // const int USED = 1;
        //
        std::map<int, CustomPoint3d> mapPointCloud;
        int nPointCount = 1;

        std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

        for (int i = 0; i < vecMap.size(); i++)
        {
            KeyFrame *kf = vecMap[i].first;
            int nNeighborNum = vecMap[i].second;
            int nKfId = kf->mnId;
            std::cout << "fuse kf id:" << nKfId << std::endl;

            cv::Mat Twc = kf->GetPoseInverse(); // camera coord to world coord
            float Twc00 = Twc.at<float>(0, 0);
            float Twc01 = Twc.at<float>(0, 1);
            float Twc02 = Twc.at<float>(0, 2);
            float Twc03 = Twc.at<float>(0, 3);
            float Twc10 = Twc.at<float>(1, 0);
            float Twc11 = Twc.at<float>(1, 1);
            float Twc12 = Twc.at<float>(1, 2);
            float Twc13 = Twc.at<float>(1, 3);
            float Twc20 = Twc.at<float>(2, 0);
            float Twc21 = Twc.at<float>(2, 1);
            float Twc22 = Twc.at<float>(2, 2);
            float Twc23 = Twc.at<float>(2, 3);

            for (int m = 0; m < h; ++m)
            {
                for (int n = 0; n < w; ++n)
                {
                    float curKFZ = kf->imDepth.at<cv::Vec3f>(m, n)[2];
                    if (curKFZ <= m_fMinDepth)
                    {
                        continue;
                    }

                    int nIdexInMap = mapKFId2VecIndex[nKfId];
                    int nIdexCurUsed = nIdexInMap * h * w + m * w + n;
                    int nPointIndex = arrIsDepthUsed[nIdexCurUsed];

                    if (nPointIndex != 0)
                    {
                        // warn: will ignore some depth
                        continue;
                    }
                    //arrIsDepthUsed[nIdexCurUsed] = nPointCount;

                    float curKFX = kf->imDepth.at<cv::Vec3f>(m, n)[0];
                    float curKFY = kf->imDepth.at<cv::Vec3f>(m, n)[1];

                    float curKFWPosX = Twc00 * curKFX + Twc01 * curKFY + Twc02 * curKFZ + Twc03;
                    float curKFWPosY = Twc10 * curKFX + Twc11 * curKFY + Twc12 * curKFZ + Twc13;
                    float curKFWPosZ = Twc20 * curKFX + Twc21 * curKFY + Twc22 * curKFZ + Twc23;

                    // std::vector<KeyFrame*> vecNeighborKF = kf->GetBestCovisibilityKeyFrames(nNeighborNum);
                    std::vector<KeyFrame *> vecNeighborKF = kf->GetBestCovisibilityKeyFrames(nTopNum);
                    int nGoodView = 0;
                    int nView = 0;
                    CustomPoint3d tmpPoint3d;
                    std::vector<int> vecTmpUsedIndex;
                    for (auto &neighborKF : vecNeighborKF)
                    {

                        cv::Mat neighborKFTcw = neighborKF->GetPose(); // world coord to camera coord
                        float hfc00 = neighborKFTcw.at<float>(0, 0);
                        float hfc01 = neighborKFTcw.at<float>(0, 1);
                        float hfc02 = neighborKFTcw.at<float>(0, 2);
                        float hfc03 = neighborKFTcw.at<float>(0, 3);
                        float hfc10 = neighborKFTcw.at<float>(1, 0);
                        float hfc11 = neighborKFTcw.at<float>(1, 1);
                        float hfc12 = neighborKFTcw.at<float>(1, 2);
                        float hfc13 = neighborKFTcw.at<float>(1, 3);
                        float hfc20 = neighborKFTcw.at<float>(2, 0);
                        float hfc21 = neighborKFTcw.at<float>(2, 1);
                        float hfc22 = neighborKFTcw.at<float>(2, 2);
                        float hfc23 = neighborKFTcw.at<float>(2, 3);

                        float curKFInNeighborX = hfc00 * curKFWPosX + hfc01 * curKFWPosY + hfc02 * curKFWPosZ + hfc03;
                        float curKFInNeighborY = hfc10 * curKFWPosX + hfc11 * curKFWPosY + hfc12 * curKFWPosZ + hfc13;
                        float curKFInNeighborZ = hfc20 * curKFWPosX + hfc21 * curKFWPosY + hfc22 * curKFWPosZ + hfc23;

                        if (curKFInNeighborZ < m_fMinDepth)
                        {
                            continue;
                        }

                        // camera coord to pixel coord
                        float fx = m_K.at<float>(0, 0); /* 0 */
                        float cx = m_K.at<float>(0, 2);
                        /* 0 */ float fy = m_K.at<float>(1, 1);
                        float cy = m_K.at<float>(1, 2);
                        /* 0 */ /* 0 */ float k31 = 1.0;

                        int u2 = (int)((fx * curKFInNeighborX + cx * curKFInNeighborZ) / curKFInNeighborZ);
                        int v2 = (int)((fy * curKFInNeighborY + cy * curKFInNeighborZ) / curKFInNeighborZ);
                        // if (u2 > 0 && u2< w && v2 > 0 && v2 < h)
                        if (u2 <= 0 || u2 >= w || v2 <= 0 || v2 >= h)
                        {
                            continue;
                        }

                        int nNeighborIdexInMap = mapKFId2VecIndex[neighborKF->mnId];
                        int nNeighborIdexInUsedArr = nNeighborIdexInMap * h * w + v2 * w + u2;
                        nPointIndex = arrIsDepthUsed[nNeighborIdexInUsedArr];
                        /*
                        if(nPointIndex != 0)
                        {
                            continue;
                        }*/

                        float neighborDepth = neighborKF->imDepth.at<cv::Vec3f>(v2, u2)[2];
                        if (neighborDepth > 0)
                        {
                            nView++;
                            if (IsDepthSimilar(curKFInNeighborZ, neighborDepth, m_fDepthDiffStrict))
                            {
                                nGoodView++;
                                tmpPoint3d.vecViewId.push_back(neighborKF->mnId);
                            }
                        }

                        if (curKFInNeighborZ < neighborDepth)
                        {
                            // this neighbor deth is blocked by the projection from current depth map
                            neighborKF->imDepth.at<cv::Vec3f>(v2, u2)[0] = 0.;
                            neighborKF->imDepth.at<cv::Vec3f>(v2, u2)[1] = 0.;
                            neighborKF->imDepth.at<cv::Vec3f>(v2, u2)[2] = 0.;
                        }

                        //arrIsDepthUsed[nNeighborIdexInUsedArr] = nPointCount;
                        vecTmpUsedIndex.push_back(nNeighborIdexInUsedArr);
                    }

                    if (nGoodView >= m_nMinViewNum)
                    {
                        //CustomPoint3d tmpPoint3d;
                        tmpPoint3d.nPointIndex = nPointCount;
                        tmpPoint3d.nPointViewNum = nGoodView;
                        tmpPoint3d.point3d[0] = curKFWPosX;
                        tmpPoint3d.point3d[1] = curKFWPosY;
                        tmpPoint3d.point3d[2] = curKFWPosZ;
                        tmpPoint3d.rgbcolor[0] = kf->imLeftRgb.at<cv::Vec3b>(m, n)[2];
                        tmpPoint3d.rgbcolor[1] = kf->imLeftRgb.at<cv::Vec3b>(m, n)[1];
                        tmpPoint3d.rgbcolor[2] = kf->imLeftRgb.at<cv::Vec3b>(m, n)[0];

                        mapPointCloud[nPointCount] = tmpPoint3d;

                        arrIsDepthUsed[nIdexCurUsed] = nPointCount; //set used for current frame 
                        for(auto& index : vecTmpUsedIndex) //set used for neighbor
                        {
                            arrIsDepthUsed[index] = nPointCount;
                        }


                        nPointCount++;
                    }
                }
            }
        }
        std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
        auto time_d = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_end - time_start).count();
        cout << "fuse point cloud time elapse:" << time_d << endl;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        // for(auto& it : mapPointCloud)
        std::map<int, CustomPoint3d>::iterator it;
        for (it = mapPointCloud.begin(); it != mapPointCloud.end(); it++)
        {
            float z = it->second.point3d[2];
            if (z > m_fMaxDepth || z < m_fMinDepth)
            {
                // std::cout<<p.z<<std::endl;
                continue;
            }
            pcl::PointXYZRGBA p;
            p.x = it->second.point3d[0];
            p.y = it->second.point3d[1];
            p.z = z;

            p.b = it->second.rgbcolor[2];
            p.g = it->second.rgbcolor[1];
            p.r = it->second.rgbcolor[0];

            pPointCloud->points.push_back(p);
        }
        pPointCloud->height = 1;
        pPointCloud->width = pPointCloud->points.size();
        pPointCloud->is_dense = true;
        std::cout << "cloud size:"<< pPointCloud->points.size() << std::endl;

        voxel->setInputCloud(pPointCloud);
        voxel->filter(*pPointCloud);
        std::cout << "cloud size, after filter:"<< pPointCloud->points.size() << std::endl;

        if (m_bIsSaveData)
        {
            std::string strSaveName = m_strSavePath + "_global2.ply";
            pcl::PLYWriter writer;
            writer.write(strSaveName, *pPointCloud);
            std::cout << "save global2 pcl cloud" << std::endl;
        }
    }

    ////////////////////////////////////////////////////////////////
    void PointCloudMapping::RealTimeGeneratePointCloudThread()
    {
        pcl::visualization::CloudViewer viewer("viewer");
       
        while (!shutDownFlag)
        {
            int nKFNum = 0;
            std::list<KeyFrame *> lNewKeyFrames;
            {
                unique_lock<mutex> lck(keyframeMutex);
                nKFNum = mlNewKeyFrames.size();
                lNewKeyFrames = mlNewKeyFrames;
                if (nKFNum != 0)
                {
                    mlNewKeyFrames.clear();
                }
            }

            if (0 == nKFNum)
            {
                usleep(10 * 1000 * 1000); // 10ms
                continue;
            }

            //cout << "get " << lNewKeyFrames.size() << " new key frame"<< endl;
            // insert key frame to new map
            int nNum = 0;
            for (auto &kf : lNewKeyFrames)
            {
                if (m_mapKeyFrame.count(kf->mnId))
                {
                    continue;
                }

                if(kf->isBad())
                {
                    continue;
                }

                // it is a new key frame
                m_mapKeyFrame[kf->mnId] = kf;
                nNum++;
                cout << "get new key frame, id:" << kf->mnId << endl;
            }
            cout << "get " << nNum << " new key frame"<< endl;

            if(0 == nNum)
            {
                continue;
            }

            // compute depth map
            std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
            for (auto it = m_mapKeyFrame.begin(); it != m_mapKeyFrame.end(); ++it)
            {
                if(it->second->isBad())
                {
                    cout << "bad frame not going to compute depth map, id:" << it->second->mnId << endl;
                    continue;
                }
              
                if (!it->second->imDepth.empty())
                {
                    // already have depth map
                    continue;
                }

                if (m_bUseCuda)
                {
                    m_stereoMatchCuda.ComputeDepthMap(it->second->imLeftRgb, it->second->imRightRgb, it->second->imDepth);
                }
                else
                {
                    m_stereoMatch.ComputeDepthMap(it->second->imLeftRgb, it->second->imRightRgb, it->second->imDepth);
                }
               
                if (m_bIsSaveData)
                {
                    std::string strSaveName = m_strSavePath + std::to_string(it->second->mnId) + "_" + ".ply";
                    SavePCLCloud(it->second->imLeftRgb, it->second->imDepth, strSaveName);
                }
            }
             std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
            auto time_d = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_end - time_start).count();
            cout << "compute depth time elapse:" << time_d << endl;
            

            for (auto it = m_mapKeyFrame.begin(); it != m_mapKeyFrame.end(); ++it)
            {

                //cout << "project neighbor depth map to current, id:" << it->first << endl;
                // project neighbor depth map to current
                //std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
                ProjectFromNeighbor(it->second);
                //std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
                //auto time_d = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_end - time_start).count();
                //cout << "ProjectFromNeighbor time elapse:" << time_d << endl;
                //cout << "ProjectFromNeighbor done" << endl;

                // filter depth map
                FilterDepthMap(it->second);
                //cout << "FilterDepthMap done" << endl;

                //std::chrono::steady_clock::time_point time_2 = std::chrono::steady_clock::now();
                //auto time_d2 = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_2 - time_end).count();
                //cout << "FilterDepthMap time elapse:" << time_d2 << endl;
            }
            std::chrono::steady_clock::time_point time_end1 = std::chrono::steady_clock::now();
            auto time_d1 = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_end1 - time_end).count();
            cout << "filter time elapse:" << time_d1 << endl;
           

            DeleteBadKF();

            FusePCLCloud();
            
            if( m_newGlobalMap->points.size() >0)
            {
                cout << "global cloud size:" << m_newGlobalMap->points.size() << endl;
                viewer.showCloud(m_newGlobalMap);
            }
            

            std::chrono::steady_clock::time_point time_end2 = std::chrono::steady_clock::now();
            auto time_d2 = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_end2 - time_end).count();
            cout << "FusePCLCloud time elapse:" << time_d2 << endl;
            
        }
         std::cout << "thread RealTimeGeneratePointCloudThread quit!" << std::endl;
    }


}
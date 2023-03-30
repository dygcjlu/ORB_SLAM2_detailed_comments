
#include <iostream>
#include <pcl/io/ply_io.h> 
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

#include "StereoMatch.h"


using namespace cv;

namespace ORB_SLAM2
{
int StereoMatch::Init()
{
    m_fMaxDepth = 1.0;
    m_fMinDepth = 0.001;
    m_strSavePath = "/media/xxd/Data2/datasets/3d/za/";

    int SADWindowSize = 7;
    m_pSGBM = cv::StereoSGBM::create(0,16,3);

    m_pSGBM->setPreFilterCap(63);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    m_pSGBM->setBlockSize(sgbmWinSize);

    int cn = 3;// img1.channels();

    m_pSGBM->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    m_pSGBM->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    m_pSGBM->setMinDisparity(0);
    m_pSGBM->setNumDisparities(256);
    m_pSGBM->setUniquenessRatio(10);
    m_pSGBM->setSpeckleWindowSize(300);
    m_pSGBM->setSpeckleRange(32);
    m_pSGBM->setDisp12MaxDiff(1);
    m_pSGBM->setMode(cv::StereoSGBM::MODE_SGBM);
    

    return 0;
}

void ShowRectiedImg(cv::Mat& Left, cv::Mat& Right)
{
    Mat canvas;
    double sf;
    int w, h;
    sf = 600./MAX(Left.cols, Left.rows);
    w = cvRound(Left.cols*sf);
    h = cvRound(Left.rows*sf);
    canvas.create(h, w*2, CV_8UC3);

   
    Mat leftPart = canvas(Rect(0, 0, w, h));
    resize(Left, leftPart, leftPart.size(), 0, 0, INTER_AREA);

   
    Mat rightPart = canvas(Rect(w, 0, w, h));
    resize(Right, rightPart, rightPart.size(), 0, 0, INTER_AREA);

    for(int j = 0; j < canvas.rows; j += 16 )
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
    std::cout<<"show rectified image"<<std::endl;

    imshow("rectified", canvas);
    waitKey(1000);

}


int StereoMatch::ComputeDepthMap(cv::Mat& Left, cv::Mat& Right,  cv::Mat& xyz)
{
    static bool showRectiedImg=false;
    if(showRectiedImg)
    {
        showRectiedImg = false;
        ShowRectiedImg(Left, Right);
    }

    Mat disp, disp8;
    float disparity_multiplier = 1.0f;

    m_pSGBM->compute(Left, Right, disp);
    if (disp.type() == CV_16S)
        disparity_multiplier = 16.0f;

    disp.convertTo(disp8, CV_8U, 255/(256*16.));

    /*
    std::string strSavePath = "/media/xxd/Data2/datasets/3d/za/";
    static int i = 0;
    i++;
    std::string strSaveName = strSavePath + std::to_string(i) + ".jpg";
    imwrite(strSaveName, disp8);
    */

    //Mat xyz;
    Mat floatDisp;
    disp.convertTo(floatDisp, CV_32F, 1.0f / disparity_multiplier);
    reprojectImageTo3D(floatDisp, xyz, m_Q, true);
    FilterDepth(xyz);

    //SavePCLCloud(Left, xyz);

    return 0;
}

void StereoMatch::FilterDepth(cv::Mat& xyz)
{
     for (int m = 0; m < xyz.rows; m++)
    {
        for (int n = 0; n < xyz.cols; n++)
        {

            float z = xyz.at<cv::Vec3f>(m,n)[2];
            if(z > m_fMaxDepth || z < m_fMinDepth)
            {
                xyz.at<cv::Vec3f>(m,n)[0] = 0;
                xyz.at<cv::Vec3f>(m,n)[1] = 0;
                xyz.at<cv::Vec3f>(m,n)[2] = 0;
            }
        }
    }

}


void StereoMatch::SavePCLCloud( cv::Mat& img, cv::Mat& xyz)
{
   
    double min, max;
    cv::minMaxLoc(xyz, &min, &max);
    //std::cout<<"min:"<<min<<" max:"<<max<<std::endl;
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // point cloud is null ptr
    for (int m = 0; m < xyz.rows; m++)
    {
        for (int n = 0; n < xyz.cols; n++)
        {
            //pcl::PointXYZRGBA p;
            pcl::PointXYZRGB p;
            
            cv::Vec3f xyzPixel = xyz.at<cv::Vec3f>(m, n);
            cv::Vec3b bgrPixel = img.at<cv::Vec3b>(m, n);
            p.x = xyzPixel.val[0];
            p.y = xyzPixel.val[1];
            p.z = xyzPixel.val[2];
            /*
            if(p.z > 0.8 )
            {
                //std::cout<<p.z<<std::endl;
                continue;
            }
            //std::cout<<"  "<<p.z;//<<std::endl;
                
            if(p.z < 0.05)
            {
                //std::cout<<"p.z < 1 "<<std::endl;
                continue;
            }   */
            p.b = bgrPixel.val[0];
            p.g = bgrPixel.val[1];
            p.r = bgrPixel.val[2];
          

            pPointCloud->points.push_back(p);
        }
    }
    //std::cout<<"point size:"<<pPointCloud->points.size()<<std::endl;
    pPointCloud->height = 1;
    pPointCloud->width = pPointCloud->points.size();
    pPointCloud->is_dense = true;

    //std::string strSavePath = "/media/xxd/Data2/datasets/3d/za/";
    //strFilePath = strSavePath + "pclPointCloud.ply";

    static int i = 0;
    i++;
    std::string strSaveName = m_strSavePath + std::to_string(i) + ".ply";

    pcl::PLYWriter writer;
	writer.write(strSaveName, *pPointCloud);
    //std::cout<<"save pcl cloud"<<std::endl;

   
     return;
  
}

int StereoMatch::SetQ(cv::Mat& Q)
{
    m_Q = Q;
    std::cout<<"set Q: "<<m_Q<<std::endl;
    return 0;
}

}
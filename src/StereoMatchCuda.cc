#include <iostream>
#include <pcl/io/ply_io.h> 
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

#include "StereoMatchCuda.h"


using namespace cv;

namespace ORB_SLAM2
{
    StereoMatchCuda::StereoMatchCuda()
    {
        m_pSGMCuda = nullptr;
        m_nImgW = 0;
        m_nImgH = 0;

        m_fMaxDepth = 1.0;
        m_fMinDepth = 0.001;

    }

int StereoMatchCuda::Init()
{

    m_pSGMCuda = nullptr;
    m_nImgW = 0;
    m_nImgH = 0;

    m_fMaxDepth = 1.0;
    m_fMinDepth = 0.01;
    m_strSavePath = "/media/xxd/Data2/datasets/3d/za/";

    

    //pSGMCuda = new sgm::StereoSGM(width, height, disp_size, src_depth, dst_depth, sgm::EXECUTE_INOUT_CUDA2CUDA, param);


    
    

    return 0;
}

void StereoMatchCuda::ShowRectiedImg(cv::Mat& Left, cv::Mat& Right)
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


int StereoMatchCuda::ComputeDepthMap(cv::Mat& Left, cv::Mat& Right,  cv::Mat& xyz)
{
    if(nullptr == m_pSGMCuda)
    {
        m_nImgW =  Left.cols;
        m_nImgH = Left.rows;
        int disp_size = 128;
        m_nSrcC = 8;
        m_nDstC = 16;
        m_bUseSubPixel = true;
        const sgm::StereoSGM::Parameters param(10, 120, 0.85, m_bUseSubPixel, sgm::PathType::SCAN_8PATH, 90, 1,  sgm::CensusType::CENSUS_9x7);
        //m_pSGMCuda = new sgm::StereoSGM(m_nImgW, m_nImgH, disp_size, m_nSrcC, m_nDstC, sgm::EXECUTE_INOUT_CUDA2CUDA, param);
        m_pSGMCuda = new sgm::StereoSGM(m_nImgW, m_nImgH, disp_size, m_nSrcC, m_nDstC, sgm::EXECUTE_INOUT_HOST2HOST, param);
    }
    cv::Mat leftGray;
    cv::Mat rightGray;

    if(3 == Left.channels())
    {
        cv::cvtColor(Left, leftGray, cv::COLOR_RGB2GRAY);
    }else{
        leftGray = Left.clone();
    }

    if(3 == Right.channels())
    {
        cv::cvtColor(Right, rightGray, cv::COLOR_RGB2GRAY);
    }else{
        rightGray = Right.clone();
    }

    static bool showRectiedImg=false;
    if(showRectiedImg)
    {
        showRectiedImg = false;
        ShowRectiedImg(Left, Right);
    }

   

    cv::Mat disparity_32f;

    /*
    const int src_bytes = m_nSrcC * m_nImgW * m_nImgH / 8;
	const int dst_bytes = m_nDstC * m_nImgW * m_nImgH / 8;

    device_buffer d_I1(src_bytes), d_I2(src_bytes), d_disparity(dst_bytes);
    d_I1.upload(leftGray.data);
	d_I2.upload(rightGray.data);
    cv::Mat disparity(leftGray.size(), CV_16S), disparity_32f;
	m_pSGMCuda.execute(d_I1.data, d_I2.data, d_disparity.data);
    d_disparity.download(disparity.data);
    */
    cv::Mat disparity(leftGray.size(), CV_16S);
    std::cout<<"img size:"<<leftGray.size()<<std::endl;

	m_pSGMCuda->execute(leftGray.data, rightGray.data, disparity.data);


    const int disp_scale = m_bUseSubPixel ? sgm::StereoSGM::SUBPIXEL_SCALE : 1;
    disparity.convertTo(disparity_32f, CV_32F, 1. / disp_scale);
    /*

    double min, max;
    cv::minMaxLoc(disparity_32f, &min, &max);
    cv::Scalar tempVal = cv::mean( disparity_32f );
    std::cout<<"min:"<<min<<" max:"<<max<<std::endl;
    std::cout<<"mean:"<<tempVal<<std::endl;
    */

    reprojectImageTo3D(disparity_32f, xyz, m_Q, true);
    FilterDepth(xyz);


    return 0;
}

void StereoMatchCuda::FilterDepth(cv::Mat& xyz)
{
     
     int nBorderLen = 20;
     for (int m = 0; m < xyz.rows; m++)
    {
        for (int n = 0; n < xyz.cols; n++)
        {
            if((m<nBorderLen) || (m>(xyz.rows-nBorderLen)) || (n < nBorderLen) || (n > (xyz.cols - nBorderLen)))
            {
                xyz.at<cv::Vec3f>(m,n)[0] = 0;
                xyz.at<cv::Vec3f>(m,n)[1] = 0;
                xyz.at<cv::Vec3f>(m,n)[2] = 0;
                continue;
            }

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


void StereoMatchCuda::SavePCLCloud( cv::Mat& img, cv::Mat& xyz)
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

int StereoMatchCuda::SetQ(cv::Mat& Q)
{
    m_Q = Q;
    std::cout<<"set Q: "<<m_Q<<std::endl;
    return 0;
}

}
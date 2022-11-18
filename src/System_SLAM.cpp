#include "System_SLAM.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Tracking.h"

using namespace std;
using namespace cv;


System::System()
{
    
}

System::System(int type, 
                Mat intrinsic_param, 
                Mat distortion_param,
                vector<double> other_param)
{
    m_intrinsic_param = intrinsic_param; // 3x3
    m_distortion_param = distortion_param; //8x1
    
    mpTracker = new tracker(type,intrinsic_param,distortion_param,other_param);
    mpManager = new Manager(type);
    mpTracker->mpManager = this->mpManager;//데이터 매니저를 등록합니다.
    if(type == TYPE_RGBD)
    {
        cout<<"RGBD SLAM start"<<endl;
    }
    else
    {
        cout<<"나머지 모드는 정의가 안되어있습니다.."<<endl;
    }
}

void System::registerImageMono(Mat Image)
{
    
}

void System::registerImageStereo(Mat leftImage, Mat rightImage)
{
    
}

void System::registerImageRGBD(Mat Image, Mat DepthMap)
{
    mpTracker->trackImageRGBD(Image,DepthMap);
}

Mat System::getCurrentPose()
{
    
}

vector<Mat> System::getPoseSequence()
{
    
}
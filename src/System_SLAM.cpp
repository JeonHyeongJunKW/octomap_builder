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
    m_intrinsic_param = intrinsic_param;
    m_distortion_param = distortion_param;
    
    mpTracker = new tracker(type,intrinsic_param,distortion_param,other_param);
    if(type == TYPE_RGBD)
    {
        cout<<"참멀리왔네"<<endl;
    }
    else
    {
        cout<<"아직 정의가 안되어있습니다."<<endl;
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
    
}

Mat System::getCurrentPose()
{
    
}

vector<Mat> System::getPoseSequence()
{
    
}
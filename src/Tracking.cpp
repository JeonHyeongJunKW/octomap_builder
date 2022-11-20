#include "Tracking.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
tracker::tracker()
{
    
}

tracker::tracker(int type, 
                Mat intrinsic_param, 
                Mat distortion_param,
                vector<double> other_param)
{
    this->intrinsic_param = intrinsic_param;
    this->distortin_param = distortin_param;
    this->other_param = other_param;
    this->depthfactor = this->other_param[0];
}

void tracker::trackImageMono(Mat Image)
{
    
}

void tracker::trackImageStereo(Mat leftImage, Mat rightImage)
{
    
}

void tracker::trackImageRGBD(Mat Image, Mat depthImage)
{
    //새로운 프레임을 등록합니다.
    Frame newFrame(Image, depthImage,this->depthfactor,this->intrinsic_param,this->distortin_param);
    newFrame.id = this->mpManager->frame_size;
    cout<<"Add New Image"<<endl;
    mpManager->AddNewFrame(newFrame);
    if(TRACK_MODE == TRACK_INITIALIZE)
    {
        //초기화단계, 현재프레임의 자세를 초기화하고 맵포인트를 등록합니다.
        TRACK_MODE = TRACK_OK;
    }
    //새로운 프레임으로부터 초기자세를 구합니다.
    //최적화를 합니다.
    //맵포인트를 등록 및 아웃라이어를 제거합니다.
}

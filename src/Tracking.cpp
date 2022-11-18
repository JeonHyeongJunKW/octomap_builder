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
    Frame newFrame(Image,depthImage);
    newFrame.id = this->mpManager->frame_size;
    mpManager->AddNewFrame(newFrame);

    //새로운 프레임으로부터 초기자세를 구합니다.
    //최적화를 합니다.
    //맵포인트를 등록 및 아웃라이어를 제거합니다.
}

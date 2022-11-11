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
    
}

void tracker::trackImageMono(Mat Image)
{
    
}

void tracker::trackImageStereo(Mat leftImage, Mat rightImage)
{
    
}

void tracker::trackImageRGBD(Mat Image, Mat deptImage)
{
    
}

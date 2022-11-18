#ifndef TRACKING_H
#define TRACKING_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "DataManager.h"
using namespace std;
using namespace cv;

class tracker
{
    public:
    tracker();
    tracker(int type, 
                Mat intrinsic_param, 
                Mat distortion_param,
                vector<double> other_param);
    void trackImageMono(Mat Image);
    void trackImageStereo(Mat leftImage, Mat rightImage);
    void trackImageRGBD(Mat Image, Mat depthImage);
    Manager* mpManager;
    Mat distortin_param; //8x1
    Mat intrinsic_param; //3x3
    vector<double> other_param;
};

#endif
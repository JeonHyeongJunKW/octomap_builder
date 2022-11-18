#ifndef FRAME_H
#define FRAME_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "MapPoint.h"
using namespace std;
using namespace cv;

class Frame
{
    public:

    Frame();
    Frame(Mat Image);
    Frame(Mat leftImage, Mat rightImage);
    Frame(Mat Image, Mat deptImage, double DepthFactor);

    int id;
    
};

#endif
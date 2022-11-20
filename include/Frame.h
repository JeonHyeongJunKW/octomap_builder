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
    Frame(Mat Image, Mat intrinsicParam, Mat distortionParam);
    Frame(Mat leftImage, Mat rightImage, Mat intrinsicParam, Mat distortionParam);
    Frame(Mat Image, Mat deptImage, double DepthFactor, Mat intrinsicParam, Mat distortionParam);

    int id;
    Mat depthImage;
    Mat RGBImage;
    double depthfactor;

    Mat intrinsicParam;
    Mat distortionParam;
    vector<KeyPoint> mKps;
    Mat mkp_decs;
    vector<int> MapPointIdx;
};

#endif
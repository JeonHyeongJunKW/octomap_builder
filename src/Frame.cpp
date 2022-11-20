#include "Frame.h"
#include "MapPoint.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


Frame::Frame()
{
    
}

Frame::Frame(Mat Image, Mat intrinsicParam, Mat distortionParam)
{
    
}

Frame::Frame(Mat leftImage, Mat rightImage, Mat intrinsicParam, Mat distortionParam)
{
    
}

Frame::Frame(Mat Image, Mat deptImage, double DepthFactor, Mat intrinsicParam, Mat distortionParam)
{
    this->RGBImage = Image;
    this->depthImage = depthImage;
    this->depthfactor = DepthFactor;
    this->intrinsicParam = intrinsicParam;
    this->distortionParam = distortionParam;

    double scalefactor = 1.2;
    double pyramid_size = 8;
    const static auto& _orb_OrbHandle = ORB::create(2000,scalefactor,pyramid_size,31,0,2);

    _orb_OrbHandle->detectAndCompute(Image, noArray(), this->mKps, this->mkp_decs);
}


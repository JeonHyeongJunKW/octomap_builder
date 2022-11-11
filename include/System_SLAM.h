#ifndef SYSTEM_H
#define SYSTEM_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "Tracking.h"
#include "DataManager.h"

using namespace std;
using namespace cv;

class System
{

    public:
        System();
        System(int type, 
                Mat intrinsic_param, 
                Mat distortion_param,
                vector<double> other_param);
        void registerImageMono(Mat Image);
        void registerImageStereo(Mat leftImage, Mat rightImage);
        void registerImageRGBD(Mat Image, Mat DepthMap);
        Mat getCurrentPose();
        vector<Mat> getPoseSequence();

        tracker* mpTracker;// 사진 받아서 미리정해진 카메라파라미터 기준으로, 프레임등록, 맵포인트등록, 초기자세 구하기(stereo 부터 안함), 최적화

    public:
        Mat m_intrinsic_param;
        Mat m_distortion_param;
};

#endif
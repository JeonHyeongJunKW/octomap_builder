#ifndef DMANAGER_H
#define DMANAGER_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "Frame.h"
#include "MapPoint.h"

using namespace std;
using namespace cv;
enum CameraType
{
    TYPE_MONO,
    TYPE_STEREO,
    TYPE_RGBD,
};
class Manager
{
    
    public: 
    Manager();
    Manager(bool type);
    void AddNewFrame(Frame f);
    void AddNewMP(MapPoint MPs);
    void DeleteMP(int idx);

    vector<Frame> frames;
    Frame currentFrame;
    vector<MapPoint> MPs;
    vector<int> deletedMPs;
    int frame_size =0;
    int mp_size =0;
    int deleted_mp_size =0;
    int type;
};

#endif
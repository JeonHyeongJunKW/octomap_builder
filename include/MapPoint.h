#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

class MapPoint
{
    public:

    MapPoint(Point3d pose,int idx);
    vector<int> FrameIdx;
    vector<int> lMappointIdx;
    int g_Idx;
    Point3d coords;
    bool is_used = true;
};

#endif
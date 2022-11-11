#include "MapPoint.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
MapPoint::MapPoint(Point3d pose,int idx)
{
    this->coords = pose;
    this->g_Idx = idx;
}

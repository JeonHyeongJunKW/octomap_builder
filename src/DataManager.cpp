#include "DataManager.h"
#include "Frame.h"
Manager::Manager()
{

}

Manager::Manager(bool type)
{
    this->type = type;
}

void Manager::AddNewFrame(Frame f)
{
    frames.push_back(f);
    //맵포인트를 만들어서 추가합니다.
    vector<KeyPoint> mp_kps = f.mKps;
    Mat des = f.mkp_decs;
    Mat pixelPoint(3,1,CV_32F);
    
    Mat inv_inst = f.intrinsicParam.inv();
    for(int idx=0; idx<mp_kps.size(); idx++)
    {
        int px =(int)(mp_kps[idx].pt.x);
        int py =(int)(mp_kps[idx].pt.y);
        // cout<<py<<", "<<px<<endl;

        //TODO 이미지 가로세로 확인해서 넣을것
        // double depth = (double)(f.depthImage.at<uint16_t>(py,px))/f.depthfactor;
        // pixelPoint.at<float>(0,0) = Undist_pixel.at<Vec2f>(i*camera_width+j,0)[0];
        // pixelPoint.at<float>(1,0) = Undist_pixel.at<Vec2f>(i*camera_width+j,0)[1];
        // pixelPoint.at<float>(2,0) = 1.0;
        //             Point_Width_depth =(inv_inst*pixelPoint)*depth_meter;
        

    }
    this->frame_size++;
}

void Manager::AddNewMP(MapPoint MPs)
{
    this->MPs.push_back(MPs);
    this->mp_size++;
}

void Manager::DeleteMP(int idx)
{
    this->deleted_mp_size++;
    deletedMPs.push_back(idx);
}

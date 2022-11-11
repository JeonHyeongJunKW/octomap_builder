#include "DataManager.h"
#include "Frame.h"
Manager::Manager()
{

}

Manager::Manager(bool type)
{
    
}

void Manager::AddNewFrame(Frame f)
{
    frames.push_back(f);
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

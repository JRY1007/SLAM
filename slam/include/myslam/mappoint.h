/*
 *MapPoint类；(表示路标点)
 *将估计路标点的世界坐标；
 *拿当前帧提取到的与地图中的路标点匹配，来估计相机的运动；
 *要存储它对应的描述子；
 *记录一个点被观测到的次数和被匹配到的次数，作为评价它好坏程度的指标。
 */

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam
{
    
class Frame;
class MapPoint
{
public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long      id_;        // ID
    static unsigned long factory_id_;    
    bool    good_;   
    Vector3d    pos_;       // 世界位姿
    Vector3d    norm_;      // 方向
    Mat    descriptor_; // 匹配的描述子
    
    list<Frame*>    observed_frames_;   // 观察点的关键帧 
    int     matched_times_;     // 匹配次数
    int     visible_times_;     // 现有的关键帧可见
    
    MapPoint();
    MapPoint( 
        unsigned long id, 
        const Vector3d& position, 
        const Vector3d& norm, 
        Frame* frame=nullptr, 
        const Mat& descriptor=Mat() 
    );
    
    inline cv::Point3f getPositionCV() const {
        return cv::Point3f( pos_(0,0), pos_(1,0), pos_(2,0) );
    }
    
        static MapPoint::Ptr createMapPoint();
        static MapPoint::Ptr createMapPoint( 
        const Vector3d& pos_world, 
        const Vector3d& norm_,
        const Mat& descriptor,
        Frame* frame );
};
}

#endif // MAPPOINT_H

/*
 * Map类；（管理着所有的路标点，并负责添加新路标、删除不好的路标等工作）
 * Map类中实际存储了各个关键帧和路标点，既需要随机访问，又需要随时插入和删除
 */

#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam
{
class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    unordered_map<unsigned long, MapPoint::Ptr >  map_points_;        // 路标点
    unordered_map<unsigned long, Frame::Ptr >     keyframes_;         //关键帧
  
    Map() {}
    
    void insertKeyFrame( Frame::Ptr frame ); //插入关键帧
    void insertMapPoint( MapPoint::Ptr map_point );  //插入地图点
};
}

#endif // MAP_H

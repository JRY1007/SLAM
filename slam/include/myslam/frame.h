/*
 * Frame类（提供数据存储和接口）
 *定义数据: ID、时间戳、位姿、相机、图像等；
 *提取方法：创建Frame、寻找给定点对应的深度、获取相机光心、判断某个点是否在视野内等。
 */

#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam 
{
    

class MapPoint;
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long                  id_;         // id 
    double                         time_stamp_; //时间
    SE3                            T_c_w_;      // 位姿
    Camera::Ptr                    camera_;     //针孔相机模型
    Mat                            color_, depth_; // 图像的颜色和深度
    // std::vector<cv::KeyPoint>      keypoints_;  // 特征点
    // std::vector<MapPoint*>         map_points_; // 地图点
    bool                           is_key_frame_;  // 关键帧的判断
    
public:
    Frame();
    Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat() );
    ~Frame();
    
    //创建frame
    static Frame::Ptr createFrame(); 
    
    //读取图像的深度
    double findDepth( const cv::KeyPoint& kp );

    // 读取相机光心
    Vector3d getCamCenter() const;
    
    void setPose( const SE3& T_c_w );
    
    //判断点是否在视野内
    bool isInFrame( const Vector3d& pt_world );
};

}

#endif // FRAME_H

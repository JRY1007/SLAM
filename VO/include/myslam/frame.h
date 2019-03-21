/*
 * 存储关键帧
 * 读取图像数据
 *
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
    SE3                            T_c_w_;      // 世界到相机的位姿变换
    Camera::Ptr                    camera_;     //针孔相机模型
    Mat                            color_, depth_; // 图像的颜色和深度
    // std::vector<cv::KeyPoint>      keypoints_;  // 特征点
    // std::vector<MapPoint*>         map_points_; // 地图点
    bool                           is_key_frame_;  // 关键帧的判断
    
public:
    Frame();
    Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat() );
    ~Frame();
    
    static Frame::Ptr createFrame(); 
    
    //读取图像的深度
    double findDepth( const cv::KeyPoint& kp );
    
    // 读取相机光心
    Vector3d getCamCenter() const;
    
    void setPose( const SE3& T_c_w );
    
    bool isInFrame( const Vector3d& pt_world );
};

}

#endif // FRAME_H

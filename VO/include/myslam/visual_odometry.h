/*
 *
 */

#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"

#include <opencv2/features2d/features2d.hpp>

namespace myslam 
{
class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOState {
        INITIALIZING=-1,
        OK=0,
        LOST
    };
    
    VOState     state_;     // VO状态
    Map::Ptr    map_;       //地图
    
    Frame::Ptr  ref_;       // 参考帧
    Frame::Ptr  curr_;      // 匹配帧 
    
    cv::Ptr<cv::ORB> orb_;  // ORB检测和计算
    vector<cv::KeyPoint>    keypoints_curr_;    // 匹配帧的关键点
    Mat     descriptors_curr_;  //匹配帧的描述子
    
    cv::FlannBasedMatcher   matcher_flann_;     // FLANN匹配器
    vector<MapPoint::Ptr>   match_3dpts_;       // 匹配3D点
    vector<int>     match_2dkp_index_;  //匹配2D像素
   
    SE3 T_c_w_estimated_;    // 匹配帧 位姿估计
    int num_inliers_;        // ICP
    int num_lost_;           // 无法匹配的次数
    
    // 参数
    int num_of_features_;   // 特征数量
    double scale_factor_;   // 图像金字塔的大小
    int level_pyramid_;     // 金字塔的级数
    float match_ratio_;     // 选择匹配点的概率
    int max_num_lost_;      //连续不匹配的次数
    int min_inliers_;       // minimum inliers
    double key_frame_min_rot;   // 两帧的最小旋转
    double key_frame_min_trans; // 两帧的最小变换
    double  map_point_erase_ratio_; // 取消匹配点的概率
    
public: // functions 
    VisualOdometry();
    ~VisualOdometry();
    
    bool addFrame( Frame::Ptr frame );      // 添加关键帧
    
protected:  
    // inner operation 
    void extractKeyPoints();
    void computeDescriptors(); 
    void featureMatching();
    void poseEstimationPnP(); 
    void optimizeMap();
    
    void addKeyFrame();
    void addMapPoints();
    bool checkEstimatedPose(); 
    bool checkKeyFrame();
    
    double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );
    
};
}

#endif // VISUALODOMETRY_H

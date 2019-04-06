/*
 *VO类定义了视觉里程计
 * 特征匹配和pnp优化以及局部地图
 */

#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"
#include <opencv2/features2d/features2d.hpp>
#include "myslam/g2o_types.h"
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
namespace myslam 
{  

    
class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr;  //智能指针
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
    int num_inliers_;        // 内点
    int num_lost_;           // 无法匹配的次数
    
    // 参数
    int num_of_features_;   // 特征数量
    double scale_factor_;   // 图像金字塔的大小
    int level_pyramid_;     // 金字塔的级数
    
    float match_ratio_;     // 选择匹配点的概率
    int max_num_lost_;      //连续不匹配的次数
    int min_inliers_;       // 最小化内点（正确的点）
    double key_frame_min_rot;   // 两帧的最小旋转
    double key_frame_min_trans; // 两帧的最小变换
    double  map_point_erase_ratio_; // 取消匹配点的概率
    
public: // functions 
    VisualOdometry();
    ~VisualOdometry();
    
    bool addFrame( Frame::Ptr frame );      // 添加关键帧
    
protected:  
  
    void extractKeyPoints();  //提取关键点
    void computeDescriptors();   //计算描述子
    void featureMatching();  //特征匹配
    void poseEstimationPnP(); //PnP进行位姿估计
    void optimizeMap();  //优化地图
    
    void addKeyFrame();  //添加关键帧
    void addMapPoints();  //添加地图点
    bool checkEstimatedPose();   //校对位姿估计
    bool checkKeyFrame();  //校对关键帧
    double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );
    
};



}

#endif // VISUALODOMETRY_H

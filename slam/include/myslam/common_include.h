/*
 * 将所有包含的头文件列出
 *
 */


#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

// for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

//Sophus
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SO3;
using Sophus::SE3;

//cv
#include <opencv2/core/core.hpp>
using cv::Mat;

//pcl
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
 #include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>

// std 
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
using namespace std; 
#endif
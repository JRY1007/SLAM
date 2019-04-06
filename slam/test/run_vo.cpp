//-------------- test the visual odometry -------------
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include <pcl/io/ply_io.h>



// 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT; 
    typedef pcl::PointCloud<PointT> PointCloud;

PointCloud::Ptr filterPointCloud(PointCloud::Ptr original ) 
{
    // Voxel grid 滤波降采样
    static pcl::VoxelGrid<PointT> voxel;
    double gridsize = 0.01f;
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud(original );
    PointCloud::Ptr tmp( new PointCloud() );
    voxel.filter(* tmp ); 
    return tmp;
}

int main ( int argc, char** argv )
{
    if ( argc != 2 )  //保证有两个输入
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }
    PointCloud::Ptr cloud( new PointCloud ); 
    myslam::Config::setParameterFile ( argv[1] );
    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );

    string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin ( dataset_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );

        if ( fin.good() == false )
            break;
    }

    myslam::Camera::Ptr camera ( new myslam::Camera );

    // 可视化
    cv::viz::Viz3d vis ( "Visual Odometry" );
    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose ( cam_pose );
   
    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
    vis.showWidget ( "World", world_coor );
    vis.showWidget ( "Camera", camera_coor );

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,6>> Block;  // 6x6 BlockSolver
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
    // 梯度下降方法，从GN, LM, DogLeg 中选
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
    g2o::SparseOptimizer globalOptimizer;     // 图模型
    globalOptimizer.setAlgorithm( solver );   // 设置求解器

        // 不要输出调试信息
    globalOptimizer.setVerbose( false );
    // 向globalOptimizer增加第一个顶点
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( 1);
    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
    v->setFixed( true ); //第一个顶点固定，不用优化
    globalOptimizer.addVertex( v );
    
    cout<<"read total "<<rgb_files.size() <<" entries"<<endl;
    for ( int i=0; i<rgb_files.size()-1; i++ )
    {
        cout<<"****** loop "<<i+1<<" ******"<<endl;
        Mat color = cv::imread ( rgb_files[i] );
        Mat depth = cv::imread ( depth_files[i], -1 );
        if ( color.data==nullptr || depth.data==nullptr )
            break;
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];

        boost::timer timer;
        vo->addFrame ( pFrame );
        cout<<"VO costs time: "<<timer.elapsed() <<endl;

        if ( vo->state_ == myslam::VisualOdometry::LOST )
            break;
        SE3 Twc = pFrame->T_c_w_.inverse();

        // 显示地图和相机位姿
        cv::Affine3d M (
            cv::Affine3d::Mat3 (
                Twc.rotation_matrix() ( 0,0 ), Twc.rotation_matrix() ( 0,1 ), Twc.rotation_matrix() ( 0,2 ),
                Twc.rotation_matrix() ( 1,0 ), Twc.rotation_matrix() ( 1,1 ), Twc.rotation_matrix() ( 1,2 ),
                Twc.rotation_matrix() ( 2,0 ), Twc.rotation_matrix() ( 2,1 ), Twc.rotation_matrix() ( 2,2 )
            ),
            cv::Affine3d::Vec3 (
                Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
            )
        );
        Mat img_show = color.clone();
        for ( auto& pt:vo->map_->map_points_ )
        {
            myslam::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
            cv::circle ( img_show, cv::Point2d ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,255 ), 2);
	    
        }
        cv::imshow ( "image", img_show );
        cv::waitKey ( 1);
        vis.setWidgetPose ( "Camera", M );
        vis.spinOnce ( 1,false );
        cout<<endl; 
	
	Eigen::Isometry3d T =Eigen::Isometry3d::Identity();
	T.prerotate(Twc.rotation_matrix());
	T.pretranslate(Twc.translation());
	
	 // 向g2o中增加这个顶点与上一帧联系的边
        // 顶点部分
        // 顶点只需设定id即可
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( vo->curr_->id_ );
        v->setEstimate(  Eigen::Isometry3d::Identity()  );
        globalOptimizer.addVertex(v);
        // 边部分
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        // 连接此边的两个顶点id
        edge->vertices() [0] = globalOptimizer.vertex( vo->ref_->id_);
        edge->vertices() [1] = globalOptimizer.vertex( vo->curr_->id_);
        // 信息矩阵
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
        // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
        // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
        // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
        information(0,0) = information(1,1) = information(2,2) = 10000;
        information(3,3) = information(4,4) = information(5,5) = 40000;
        // 也可以将角度设大一些，表示对角度的估计更加准确
        edge->setInformation( information );
        edge->setMeasurement( T );
        // 将此边加入图中
        globalOptimizer.addEdge(edge);
	/*
        Eigen::Isometry3d T =Eigen::Isometry3d::Identity();
	T.prerotate(Twc.rotation_matrix());
	T.pretranslate(Twc.translation());
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
                if ( d==0 ) continue; // 为0表示没有测量到
                Eigen::Vector3d point; 
                point[2] = double(d)/camera->depth_scale_; 
                point[0] = (u-camera->cx_)*point[2]/camera->fx_;
                point[1] = (v-camera->cy_)*point[2]/camera->fy_; 
                Eigen::Vector3d pointWorld = T*point;
                
                PointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels()+1 ];
                p.r = color.data[ v*color.step+u*color.channels()+2 ];
                cloud->points.push_back( p );
            }
    
       cloud->is_dense = false;
       cloud = filterPointCloud(cloud);
       cout<<"点云共有"<<cloud->size()<<"个点."<<endl;
        pcl::io::savePCDFileBinary("map.pcd", *cloud );
       pcl::io::savePLYFileBinary("map.ply", *cloud );
	 viewer.showCloud(cloud);*/
    } 
    
    globalOptimizer.save("./result/result_before.g2o");
    cout<<"prepare optimizing ..."<<endl;
    globalOptimizer.setVerbose(true);
    globalOptimizer.initializeOptimization();
    cout<<"calling optimizing ..."<<endl;
    globalOptimizer.optimize(30);  
    cout<<"saving optimization results ..."<<endl;
    globalOptimizer.save("./result/result_after.g2o");
    return 0;
}

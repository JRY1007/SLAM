/*
 * 读取参数文件并调用
 *
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "myslam/common_include.h" 

namespace myslam 
{
class Config
{
private:
    static std::shared_ptr<Config> config_; 
    cv::FileStorage file_;
    
    Config () {} 
public:
    ~Config();  //析构函数，调用结束后破坏
    
    // 创建参数文件
    static void setParameterFile( const std::string& filename ); 
    
    // 调取参数值
    template< typename T >
    static T get( const std::string& key )
    {
        return T( Config::config_->file_[key] );
    }
};
}

#endif // CONFIG_H

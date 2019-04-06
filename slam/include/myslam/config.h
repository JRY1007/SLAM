/*
 * Config类负责参数文件的读取，并在程序任意地方都可随时提供参数的值
 *把构造函数声明为私有，防止这个类的对象在别处建立，它只能在 setParameterFile时构造。实际构造的对象是Config的智能指针：static shared_ptr<Config>config_ 。
 *用 OpenCV提供的 FileStorage类。它可以读取一个 YAML文件，且可以访问其中任意一个字段。
 *通过一个模版函数 get，来获得任意类型的参数值。
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
    ~Config();  
    
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

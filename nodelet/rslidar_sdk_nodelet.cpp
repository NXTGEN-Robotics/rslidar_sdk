#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "manager/node_manager.hpp"

#include <rs_driver/macro/version.hpp>
#include <signal.h>

#include <ros/ros.h>
#include <ros/package.h>


namespace robosense
{
namespace lidar
{
    class RSLidarNodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
        
        private:
            std::shared_ptr<robosense::lidar::NodeManager> demo_ptr;
    };
}
}

namespace robosense
{
namespace lidar
{
void RSLidarNodelet::onInit()
{
    RS_TITLE << "********************************************************" << RS_REND;
    RS_TITLE << "**********                                    **********" << RS_REND;
    RS_TITLE << "**********    RSLidar_SDK Version: v" << RSLIDAR_VERSION_MAJOR 
    << "." << RSLIDAR_VERSION_MINOR 
    << "." << RSLIDAR_VERSION_PATCH << "     **********" << RS_REND;
    RS_TITLE << "**********                                    **********" << RS_REND;
    RS_TITLE << "********************************************************" << RS_REND;

    std::string config_path;
    config_path = (std::string)PROJECT_PATH;
    config_path += "/config/config.yaml";

    ros::NodeHandle priv_nh = getPrivateNodeHandle();
    std::string path;
    priv_nh.param("config_path", path, std::string(""));
    if (!path.empty())
    {
        config_path = path;
    }
    else
    {
        ROS_WARN("No config file was provided! Using default ...");
    }

    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path);
    }
    catch (...)
    {
        RS_ERROR << "The format of config file " << config_path 
        << " is wrong. Please check (e.g. indentation)." << RS_REND;
        return ;
    }

    demo_ptr = std::make_shared<robosense::lidar::NodeManager>();
    demo_ptr->init(config);
    demo_ptr->start();

    RS_MSG << "RoboSense-LiDAR-Driver is running....." << RS_REND;
}
}

}

PLUGINLIB_EXPORT_CLASS(robosense::lidar::RSLidarNodelet, nodelet::Nodelet)

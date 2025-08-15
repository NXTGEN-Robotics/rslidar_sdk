/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

Simple composable wrapper for RSLidar SDK to enable intra-process communication

Modified by: Taha Elmokadem (NXTGEN Industries)
Date: 2025
*********************************************************************************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <yaml-cpp/yaml.h>

#include "manager/node_manager.hpp"

namespace robosense
{
namespace lidar
{

class RSLidarComposableNode : public rclcpp::Node
{
public:
    explicit RSLidarComposableNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("rslidar_sdk_node", options)
    {
        RCLCPP_INFO(get_logger(), "RSLidar Composable Node starting...");
        
        // Get config path parameter (same as original implementation)
        std::string config_path = declare_parameter<std::string>("config_path", "");
        
        if (config_path.empty()) {
            RCLCPP_ERROR(get_logger(), "config_path parameter is required!");
            return;
        }
        
        // Load YAML config (same as original implementation)
        YAML::Node config;
        try {
            config = YAML::LoadFile(config_path);
            RCLCPP_INFO(get_logger(), "Config loaded from: %s", config_path.c_str());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to load config file %s: %s", config_path.c_str(), e.what());
            return;
        }
        
        // Create and initialize NodeManager (same as original implementation)
        node_manager_ = std::make_shared<NodeManager>();
        
        try {
            node_manager_->init(config);
            node_manager_->start();
            RCLCPP_INFO(get_logger(), "RSLidar NodeManager started successfully");
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to start NodeManager: %s", e.what());
            node_manager_.reset();
            return;
        }
    }
    
    ~RSLidarComposableNode()
    {
        if (node_manager_) {
            RCLCPP_INFO(get_logger(), "Stopping RSLidar NodeManager...");
            node_manager_->stop();
        }
    }

private:
    std::shared_ptr<NodeManager> node_manager_;
};

} // namespace lidar
} // namespace robosense

// Register the component with the ROS 2 component system
RCLCPP_COMPONENTS_REGISTER_NODE(robosense::lidar::RSLidarComposableNode)

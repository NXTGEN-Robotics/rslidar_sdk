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
        
        // Update topic names to include namespace (fix for composable nodes)
        // The RSLidar driver creates internal nodes that don't inherit our namespace,
        // so we need to prepend the namespace to topic names in the config
        std::string node_namespace = get_namespace();
        if (!node_namespace.empty() && node_namespace != "/" && config["lidar"]) {
            for (size_t i = 0; i < config["lidar"].size(); ++i) {
                if (config["lidar"][i]["ros"]) {
                    auto ros_config = config["lidar"][i]["ros"];
                    
                    // Update topic names to include namespace
                    if (ros_config["ros_recv_packet_topic"]) {
                        std::string topic = ros_config["ros_recv_packet_topic"].as<std::string>();
                        if (topic[0] != '/') { // Only modify relative topic names
                            ros_config["ros_recv_packet_topic"] = node_namespace + "/" + topic;
                        }
                    }
                    
                    if (ros_config["ros_send_packet_topic"]) {
                        std::string topic = ros_config["ros_send_packet_topic"].as<std::string>();
                        if (topic[0] != '/') { // Only modify relative topic names
                            ros_config["ros_send_packet_topic"] = node_namespace + "/" + topic;
                        }
                    }
                    
                    if (ros_config["ros_send_imu_data_topic"]) {
                        std::string topic = ros_config["ros_send_imu_data_topic"].as<std::string>();
                        if (topic[0] != '/') { // Only modify relative topic names
                            ros_config["ros_send_imu_data_topic"] = node_namespace + "/" + topic;
                        }
                    }
                    
                    if (ros_config["ros_send_point_cloud_topic"]) {
                        std::string topic = ros_config["ros_send_point_cloud_topic"].as<std::string>();
                        if (topic[0] != '/') { // Only modify relative topic names
                            ros_config["ros_send_point_cloud_topic"] = node_namespace + "/" + topic;
                            RCLCPP_INFO(get_logger(), "Updated point cloud topic to: %s", 
                                       ros_config["ros_send_point_cloud_topic"].as<std::string>().c_str());
                        }
                    }
                }
            }
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



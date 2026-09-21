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
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

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
        config_path_ = declare_parameter<std::string>("config_path", "");

        // Warm-idle initial state. Fail-safe default = streaming on. The runtime
        // toggle is served via the ~/set_streaming service and this param's on-set
        // callback; both funnel through apply_streaming().
        streaming_enabled_ = declare_parameter<bool>("streaming_enabled", true);

        if (config_path_.empty()) {
            RCLCPP_ERROR(get_logger(), "config_path parameter is required!");
            return;
        }
        
        // Defer NodeManager initialization to avoid shared_from_this() in constructor
        // Use a timer to initialize after construction is complete
        init_timer_ = create_wall_timer(
            std::chrono::milliseconds(1), 
            std::bind(&RSLidarComposableNode::initialize_node_manager, this)
        );
    }
    
    ~RSLidarComposableNode()
    {
        if (init_timer_) {
            init_timer_->cancel();
        }
        if (node_manager_) {
            RCLCPP_INFO(get_logger(), "Stopping RSLidar NodeManager...");
            node_manager_->stop();
        }
    }

private:
    void initialize_node_manager()
    {
        // Cancel the timer since we only need to run once
        init_timer_->cancel();
        init_timer_.reset();
        
        // Load YAML config (same as original implementation)
        YAML::Node config;
        try {
            config = YAML::LoadFile(config_path_);
            RCLCPP_INFO(get_logger(), "Config loaded from: %s", config_path_.c_str());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to load config file %s: %s", config_path_.c_str(), e.what());
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
        
        // Create and initialize NodeManager (with composable node support)
        node_manager_ = std::make_shared<NodeManager>();
        node_manager_->setParentNode(shared_from_this());
        
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

        setup_streaming_control();
    }

    // Warm-idle control surface: a SetBool service, a latched Bool status, and a
    // dedicated callback group. node_manager_->start() leaves the driver
    // streaming, so we reconcile once here to honour the startup param.
    void setup_streaming_control()
    {
        // Latched status so late subscribers always see the current state. IPC
        // disabled: transient_local is incompatible with intra-process, and
        // consumers live in other processes anyway.
        rclcpp::PublisherOptions status_pub_options;
        status_pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
        streaming_active_pub_ = create_publisher<std_msgs::msg::Bool>(
            "~/streaming_active", rclcpp::QoS(1).transient_local(), status_pub_options);

        // Dedicated callback group so the toggle never starves behind driver work
        // in the multi-threaded container.
        streaming_control_cbg_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        set_streaming_srv_ = create_service<std_srvs::srv::SetBool>(
            "~/set_streaming",
            std::bind(&RSLidarComposableNode::handle_set_streaming, this,
                      std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS().get_rmw_qos_profile(), streaming_control_cbg_);

        // Runtime toggle via `ros2 param set ... streaming_enabled`.
        param_cb_handle_ = add_on_set_parameters_callback(
            std::bind(&RSLidarComposableNode::on_set_parameters, this,
                      std::placeholders::_1));

        // Reconcile initial state from the startup param and latch status.
        apply_streaming(streaming_enabled_);

        RCLCPP_INFO(get_logger(), "Streaming control ready (driver %s)",
                    streaming_enabled_ ? "STREAMING" : "IDLE");
    }

    // Single funnel for every streaming state change (startup param, service,
    // param-set). Fans out to the driver via the NodeManager, mirrors the state
    // into streaming_enabled_, and latches the status topic.
    void apply_streaming(bool enable)
    {
        if (node_manager_) {
            node_manager_->setStreaming(enable);
        }
        streaming_enabled_ = enable;
        publish_streaming_active();
    }

    void handle_set_streaming(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        const bool desired = request->data;
        const bool changed = (desired != streaming_enabled_);
        apply_streaming(desired);
        response->success = true;
        response->message = std::string("lidar streaming ") + (desired ? "ENABLED" : "IDLE");
        if (changed) {
            RCLCPP_INFO(get_logger(), "LiDAR streaming -> %s (warm-idle)",
                        desired ? "ENABLED" : "IDLE");
        }
    }

    rcl_interfaces::msg::SetParametersResult on_set_parameters(
        const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& p : params) {
            if (p.get_name() == "streaming_enabled" &&
                p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                apply_streaming(p.as_bool());
            }
        }
        return result;
    }

    void publish_streaming_active()
    {
        if (streaming_active_pub_) {
            std_msgs::msg::Bool msg;
            msg.data = streaming_enabled_;
            streaming_active_pub_->publish(msg);
        }
    }

    std::shared_ptr<NodeManager> node_manager_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    std::string config_path_;

    // Warm-idle control.
    bool streaming_enabled_{true};
    rclcpp::CallbackGroup::SharedPtr streaming_control_cbg_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_streaming_srv_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr streaming_active_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

} // namespace lidar
} // namespace robosense



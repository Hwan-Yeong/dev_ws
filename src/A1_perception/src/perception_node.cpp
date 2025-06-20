﻿#include <chrono>
#include <functional>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "tf2/LinearMath/Quaternion.h"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "yaml-cpp/yaml.h"

#include "filter/filter_factory.hpp"
#include "layer.hpp"
#include "perception_node.hpp"

namespace A1::perception
{

void pointCloud2Callback(
    const std::string& name,
    std::shared_ptr<PerceptionNode> node,
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
{
    if (!node->perception_use)
    {
        return;
    }

    // AMCL Pose값 확인 후 필터링 진행
    Position position = node->getPosition();
    auto now = node->now();
    rclcpp::Time position_time(position.timestamp, node->get_clock()->get_clock_type());
    if (now - position_time > rclcpp::Duration(std::chrono::milliseconds(5000)))
    {
        return;
    }

    auto layer = Layer();
    layer.sensor_timestamp = msg->header.stamp;
    layer.timestamp = node->now();
    layer.cloud.reserve(msg->width);

    size_t num_points = msg->width;
    size_t point_step = msg->point_step;
    const uint8_t* data_ptr = msg->data.data();

    for (size_t i = 0; i < num_points; i++)
    {
        float x, y, z;
        std::memcpy(&x, data_ptr + i * point_step + 0, sizeof(float));
        std::memcpy(&y, data_ptr + i * point_step + 4, sizeof(float));
        std::memcpy(&z, data_ptr + i * point_step + 8, sizeof(float));
        layer.cloud.emplace_back(x, y, z);
    }

    node->setSensorLayerMap(name, layer);
}

void laserScanCallback(
    const std::string& name,
    std::shared_ptr<PerceptionNode> node,
    const sensor_msgs::msg::LaserScan::SharedPtr& msg)
{
    // 적용은 됐으나, rviz 상에서 /scan 보다 변환이 느리고,
    // 회전 변환하는데 리소스를 많이 사용함. 그래서 주석처리 후, 같이 보고 판단 진행
    auto layer = Layer();
    layer.sensor_timestamp = msg->header.stamp;
    layer.timestamp = node->now();
    layer.cloud.clear();
    // LaserScan 데이터를 PointCloud로 변환
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        float range = msg->ranges[i];

        if (std::isfinite(range))
        {  // 유효한 거리 값만 변환
            float angle = msg->angle_min + i * msg->angle_increment;
            pcl::PointXYZ point;
            point.x = range * std::cos(angle);
            point.y = range * std::sin(angle);
            point.z = 0.0f;  // 2D LiDAR이므로 Z값은 0

            layer.cloud.push_back(point);  // 변환된 점 추가
        }
    }

    node->setSensorLayerMap(name, layer);
}

// void boundingBox2DArrayCallback(
//     const std::string& name,
//     std::shared_ptr<PerceptionNode> node,
//     const vision_msgs::msg::BoundingBox2DArray::SharedPtr& msg)
// {
//     // TODO (sw): Implement this function (layer is not good choice)
//     auto layer = Layer();

//     for (const auto& box : msg->boxes)
//     {
//         // object.x = box.center.position.x;
//         // object.y = box.center.position.y;
//         // object.width = box.size_x;
//         // object.height = box.size_y;
//     }
// }

PerceptionNode::PerceptionNode() : Node("A1_perception")
{
    // 파라미터 선언: params.yaml의 "ros__parameters" 아래에 node_params가 있으므로 기본값을 설정
    std::string node_params{};
    this->declare_parameter("node_params", "node.yaml");
    this->get_parameter("node_params", node_params);

    int log_level{};
    this->declare_parameter("log_level", 10);
    this->get_parameter("log_level", log_level);

    // 로그 레벨 설정
    rcutils_logging_set_logger_level(this->get_logger().get_name(), static_cast<RCUTILS_LOG_SEVERITY>(log_level));

    RCLCPP_INFO(this->get_logger(), "A1_perception has been started.");
    try
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("A1_perception");
        std::string full_path = package_share_directory + "/" + "params" + "/" + node_params;
        this->config = YAML::LoadFile(full_path)["A1_perception"]["node"];
    }
    catch (const std::exception& e)
    {
        // fallback (ament_index_cpp::get_package_share_directory()가 제대로 작동하지 않을 경우)
        RCLCPP_ERROR(this->get_logger(), "Failed to load config file: %s", e.what());
        std::string fallback_path = "install/A1_perception/share/A1_perception/params/" + node_params;
        this->config = YAML::LoadFile(fallback_path)["A1_perception"]["node"];
    }
}

// 수정된 initFilters(): layers는 시퀀스 내부에 mapping 형태로 구성되어 있으며, 각 mapping의 값에 inputs와 filter 항목이
// 있음
void PerceptionNode::initFilters(const YAML::Node& config)
{
    auto pnode = std::dynamic_pointer_cast<PerceptionNode>(this->shared_from_this());
    // config: 시퀀스([ {multi_tof: {...}}, {cliff: {...}} ])
    auto s0 = YAML::Dump(config);
    for (const auto& layer : config)
    {
        std::string name = layer.first.as<std::string>();
        const YAML::Node& layer_config = layer.second;
        auto s = YAML::Dump(layer_config);
        this->filters[name] = FilterFactory::create(pnode, layer_config["filter"]);
    }
}

void PerceptionNode::initController()
{
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.best_effort();
    this->controller_subscribers["clear_request_from_udp"] = this->create_subscription<std_msgs::msg::Empty>(
        "/localization/clear/costmap",
        qos_profile,
        [this](const std_msgs::msg::Empty::SharedPtr msg)
        {
            (void)msg;
            this->resetLayers();
            RCLCPP_INFO(this->get_logger(), "Perception's all layers cleared by UDP command.");
        });

    this->controller_subscribers["clear_request_from_rviz_init_pose"] =
        this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            qos_profile,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
            {
                (void)msg;
                this->resetLayers();
                RCLCPP_INFO(this->get_logger(), "Perception's all layers cleared by initial pose estimate.");
            });
    this->controller_subscribers["clear_request_from_rviz_goal_pose"] =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            qos_profile,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                (void)msg;
                this->resetLayers();
                RCLCPP_INFO(this->get_logger(), "Perception's all layers cleared by goal pose.");
            });
    this->controller_subscribers["motor_status"] = this->create_subscription<robot_custom_msgs::msg::MotorStatus>(
        "/motor_status",
        10,
        [this](const robot_custom_msgs::msg::MotorStatus::SharedPtr msg)
        {
            this->motor_status.setLeftRpm(msg->left_motor_rpm);
            this->motor_status.setRightRpm(msg->right_motor_rpm);
        });

    // [EB]turn on/off maneuver action
    this->controller_subscribers["maneuver_use"] = this->create_subscription<std_msgs::msg::Bool>(
        "/maneuver/use",
        qos_profile,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            this->perception_use = msg->data;
            if (!this->perception_use)
            {
                RCLCPP_INFO(this->get_logger(), "[Perception] OFF");
                this->resetLayers();
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "[Perception] ON");
            }
        });
}

// 수정된 initSubscribers(): inputs는 시퀀스 내부에 mapping 형태로 정의되어 있음
void PerceptionNode::initSubscribers(const YAML::Node& config)
{
    auto pnode = std::static_pointer_cast<PerceptionNode>(this->shared_from_this());
    if (!pnode)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get shared pointer from this");
        return;
    }
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.best_effort();

    // config: 시퀀스([ {robot_position: {...}}, {multi_tof: {...}} ]) 형식
    for (const auto& input_item : config)
    {
        std::string input_name = input_item.first.as<std::string>();
        YAML::Node input_data = input_item.second;
        std::string input_type = input_data["type"].as<std::string>();
        std::string input_topic = input_data["topic"].as<std::string>();

        if (input_type == "PointCloud2")
        {
            auto callback = [this, pnode, input_name](sensor_msgs::msg::PointCloud2::SharedPtr msg)
            { pointCloud2Callback(input_name, pnode, msg); };
            auto subs = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic, qos_profile, callback);
            this->subscribers[input_name] = subs;
        }
        else if (input_type == "LaserScan")
        {
            auto callback = [this, pnode, input_name](sensor_msgs::msg::LaserScan::SharedPtr msg)
            { laserScanCallback(input_name, pnode, msg); };
            auto subs = this->create_subscription<sensor_msgs::msg::LaserScan>(input_topic, qos_profile, callback);
            this->subscribers[input_name] = subs;
        }
        // else if (input_type == "BoundingBox2DArray")
        // {
        //     auto callback = [this, pnode, input_name](vision_msgs::msg::BoundingBox2DArray::SharedPtr msg)
        //     { boundingBox2DArrayCallback(input_name, pnode, msg); };
        //     auto subs = this->create_subscription<vision_msgs::msg::BoundingBox2DArray>(input_topic, 10, callback);
        //     this->subscribers[input_name] = subs;
        // }
        else if (input_type == "PoseWithCovarianceStamped")
        {
            auto callback = [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
            {
                this->robot_position.x = msg->pose.pose.position.x;
                this->robot_position.y = msg->pose.pose.position.y;
                this->robot_position.z = msg->pose.pose.position.z;
                tf2::Quaternion q;
                q.setX(msg->pose.pose.orientation.x);
                q.setY(msg->pose.pose.orientation.y);
                q.setZ(msg->pose.pose.orientation.z);
                q.setW(msg->pose.pose.orientation.w);
                this->robot_position.timestamp = this->now();
                this->robot_position.setFromQuaternion(q);
            };
            auto subs = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                input_topic, qos_profile, callback);
            this->subscribers[input_name] = subs;
        }
        else if (input_type == "sensor_msgs/msg/Imu")
        {
            auto callback = [this, input_name](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                tf2::Quaternion q;
                q.setX(msg->orientation.x);
                q.setY(msg->orientation.y);
                q.setZ(msg->orientation.z);
                q.setW(msg->orientation.w);
                this->imu_data.setFromQuaternion(q);
            };
            auto subs = this->create_subscription<sensor_msgs::msg::Imu>(input_topic, qos_profile, callback);
            this->subscribers[input_name] = subs;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unsupported input type: %s", input_type.c_str());
        }
    }
}

// 수정된 initPublishers(): layers의 각 항목에 대해 debug publisher 생성
void PerceptionNode::initPublishers(const YAML::Node& config)
{
    // 기본 output publisher
    auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("perception/output", 10);
    this->publishers["output"] = pub;
    this->publishers["stop"] = this->create_publisher<std_msgs::msg::Int8>("perception/action/stop", 10);
    this->publishers["climb"] = this->create_publisher<std_msgs::msg::Bool>("perception/climb", 10);

    for (const auto& layer : config)
    {
        std::string name = layer.first.as<std::string>();
        auto debug_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("perception/debug/" + name, 10);
        this->publishers[name] = debug_pub;
    }
}

void PerceptionNode::init()
{
    this->initSubscribers(this->config["inputs"]);
    this->initMultiToFSubscribers(this->config["multi_tof_inputs"]);
    this->initFilters(this->config["layers"]);
    this->initPublishers(this->config["layers"]);
    this->initController();

    auto climb_condition = this->config["climb_condition"];
    auto pnode = std::static_pointer_cast<PerceptionNode>(this->shared_from_this());
    this->climb_checker = ClimbChecker(pnode, climb_condition);

    auto timer_callback = std::bind(&PerceptionNode::timerCallback, this);
    auto period = std::chrono::milliseconds(this->config["period_ms"].as<int>());
    this->timer = this->create_wall_timer(period, timer_callback);
}

void PerceptionNode::initMultiToFSubscribers(const YAML::Node& config)
{
    auto pnode = std::static_pointer_cast<PerceptionNode>(this->shared_from_this());
    if (!pnode)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get shared pointer from this");
        return;
    }

    for (const auto& input_item : config)
    {
        std::string direction = input_item.first.as<std::string>();
        auto array = input_item.second.as<std::vector<int>>();
        for (const auto& tof_ij : array)
        {
            std::stringstream string_stream;
            std::string input_name = "multi_tof_" + direction + "_idx_" + std::to_string(tof_ij);
            std::string input_topic = "/sensor_to_pointcloud/tof/multi/" + direction + "/idx_" + std::to_string(tof_ij);

            auto callback = [this, pnode, input_name](sensor_msgs::msg::PointCloud2::SharedPtr msg)
            { pointCloud2Callback(input_name, pnode, msg); };
            auto subs = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic, 10, callback);
            this->subscribers[input_name] = subs;
        }
    }
}

void PerceptionNode::setSensorLayerMap(const std::string& name, const Layer& layer)
{
    this->sensor_layer_map[name] = layer;
}

Layer PerceptionNode::getSensorLayer(const std::string& name) const
{
    auto it = this->sensor_layer_map.find(name);
    if (it != this->sensor_layer_map.end())
    {
        return it->second;
    }
    else
    {
        return Layer();
    }
}

MotorStatus PerceptionNode::getMotorStatus()
{
    return this->motor_status;
}

Position PerceptionNode::getPosition()
{
    return this->robot_position;
}

Position PerceptionNode::getIMUdata()
{
    return this->imu_data;
}

std::unordered_map<std::string, Layer>& PerceptionNode::getDropOffLayerMap()
{
    return this->drop_off_layer_map;
}

std::unordered_map<std::string, std::any>& PerceptionNode::getPublishers()
{
    return this->publishers;
}

void PerceptionNode::clearDropOffLayerMap()
{
    this->drop_off_layer_map.clear();
}

void PerceptionNode::sendActionStop(int data)
{
    auto action_stop_pub =
        std::any_cast<std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int8>>>(this->publishers["stop"]);
    std_msgs::msg::Int8 msg;
    // 0 : NO_STOP
    // 1 : DROP_OFF STOP
    // 2 : 1D TOF STOP
    msg.data = data;
    action_stop_pub->publish(msg);
}

void PerceptionNode::resetLayers()
{
    this->layers.clear();
    this->drop_off_layer_map.clear();
    this->climb_checker.reset();
}

sensor_msgs::msg::PointCloud2 PerceptionNode::convertToPointCloud2(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.height = 1;
    msg.width = cloud.size();
    msg.is_dense = true;
    msg.is_bigendian = false;
    msg.point_step = 12;
    msg.row_step = 12 * cloud.size();
    msg.data.resize(msg.row_step);

    sensor_msgs::msg::PointField x_field;
    x_field.name = "x";
    x_field.offset = 0;
    x_field.datatype = 7;
    x_field.count = 1;

    sensor_msgs::msg::PointField y_field;
    y_field.name = "y";
    y_field.offset = 4;
    y_field.datatype = 7;
    y_field.count = 1;

    sensor_msgs::msg::PointField z_field;
    z_field.name = "z";
    z_field.offset = 8;
    z_field.datatype = 7;
    z_field.count = 1;
    msg.fields.push_back(x_field);
    msg.fields.push_back(y_field);
    msg.fields.push_back(z_field);
    for (size_t i = 0; i < cloud.size(); i++)
    {
        std::memcpy(&msg.data[i * 12 + 0], &cloud[i].x, sizeof(float));
        std::memcpy(&msg.data[i * 12 + 4], &cloud[i].y, sizeof(float));
        std::memcpy(&msg.data[i * 12 + 8], &cloud[i].z, sizeof(float));
    }
    return msg;
}

void PerceptionNode::timerCallback()
{
    if (!this->perception_use)
    {
        return;
    }

    this->climb_checker.climbCheck();
    // Accumulate data from sensors
    auto layers_node = this->config["layers"];
    for (auto it = layers_node.begin(); it != layers_node.end(); ++it)
    {
        std::string name = it->first.as<std::string>();
        YAML::Node layer_config = it->second;
        auto s = YAML::Dump(layer_config);

        for (const auto& input : layer_config["inputs"])
        {
            auto input_name = input.as<std::string>();
            auto layer = this->getSensorLayer(input_name);
            if (!layer.cloud.empty())
            {
                this->layers[name].push_back(layer);
            }
        }
    }

    // Update filters
    for (const auto& [name, filter] : this->filters)
    {
        if (this->layers[name].empty())
        {
            continue;
        }
        this->layers[name] = filter->update(this->layers[name]);
    }

    // Publish results
    std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>> output_cloud_map;
    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    for (const auto& [name, layers] : this->layers)
    {
        output_cloud_map[name] = pcl::PointCloud<pcl::PointXYZ>();
        for (const auto& layer : layers)
        {
            if (layer.cloud.empty())
            {
                continue;
            }
            output_cloud_map[name].insert(output_cloud_map[name].end(), layer.cloud.begin(), layer.cloud.end());
            output_cloud.insert(output_cloud.end(), layer.cloud.begin(), layer.cloud.end());
        }
    }

    for (const auto& [key, layer] : this->drop_off_layer_map)
    {
        if (layer.cloud.empty())
        {
            continue;
        }
        output_cloud.insert(output_cloud.end(), layer.cloud.begin(), layer.cloud.end());
    }

    auto output_msg = this->convertToPointCloud2(output_cloud);
    auto pub_ptr =
        std::any_cast<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>>(this->publishers["output"]);
    if (pub_ptr)
    {
        pub_ptr->publish(output_msg);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "output publisher not found");
    }

    for (const auto& [name, output_layer_cloud] : output_cloud_map)
    {
        auto debug_msg = this->convertToPointCloud2(output_layer_cloud);
        auto pub_ptr =
            std::any_cast<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>>(this->publishers[name]);
        if (pub_ptr)
        {
            pub_ptr->publish(debug_msg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "publisher not found for key: %s", name.c_str());
        }
    }
}

bool PerceptionNode::isClimb()
{
    return this->climb_checker.isClimb();
}

}  // namespace A1::perception
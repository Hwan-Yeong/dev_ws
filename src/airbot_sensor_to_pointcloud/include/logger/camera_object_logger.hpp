#ifndef __CAMERA_OBJECT_LOGGER__
#define __CAMERA_OBJECT_LOGGER__

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "robot_custom_msgs/msg/ai_data.hpp"
#include "robot_custom_msgs/msg/ai_data_array.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"

class CameraObjectLogger
{
public:
    CameraObjectLogger();
    ~CameraObjectLogger();

    void updateParams(double dist_margin, double width_margin, double height_margin);
    void log(std::pair<robot_custom_msgs::msg::AIDataArray, vision_msgs::msg::BoundingBox2DArray> object_info);
    void logInfoClear();

private:
    double dist_margin_;
    double width_margin_;
    double height_margin_;
    std::map<int, std::vector<vision_msgs::msg::BoundingBox2D>> objects_;

    std::map<int, std::vector<vision_msgs::msg::BoundingBox2D>> updateObjects(
        robot_custom_msgs::msg::AIDataArray object_array,
        vision_msgs::msg::BoundingBox2DArray object_bbox_array);
};





#endif // CAMERA_OBJECT_LOGGER
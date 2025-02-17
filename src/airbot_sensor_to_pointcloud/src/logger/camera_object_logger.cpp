#include "logger/camera_object_logger.hpp"

CameraObjectLogger::CameraObjectLogger(double dist_margin = 0.1,
                                       double size_margin = 0.1)
    : dist_margin_(dist_margin),
      size_margin_(size_margin)
{
}

CameraObjectLogger::~CameraObjectLogger()
{
}

void CameraObjectLogger::log(std::pair<robot_custom_msgs::msg::AIDataArray, vision_msgs::msg::BoundingBox2DArray> object_info)
{
    std::map<int, vision_msgs::msg::BoundingBox2D> new_objects = updateObjects(object_info.first, object_info.second);

    if (new_objects != objects_) {
        objects_ = new_objects;
        RCLCPP_INFO(rclcpp::get_logger("||"),"==============================================================");
        for (const auto& object : objects_) {
            RCLCPP_INFO(rclcpp::get_logger("||"), "[ID]: %u, [Position (X, Y): (%f, %f)], [Size (W, H): (%f, %f)]",
                        object.first, object.second.center.position.x, object.second.center.position.y, object.second.size_x, object.second.size_y);
        }
    } else {
        return;
    }

}

void CameraObjectLogger::logInfoClear()
{
    objects_.clear();
    RCLCPP_INFO(rclcpp::get_logger("CameraObjectLogger"), "Camera Objects Log Clear");
}

std::map<int, vision_msgs::msg::BoundingBox2D> CameraObjectLogger::updateObjects(
    robot_custom_msgs::msg::AIDataArray object_array,
    vision_msgs::msg::BoundingBox2DArray object_bbox_array)
{
    std::map<int, vision_msgs::msg::BoundingBox2D> ret;
    ret = objects_;

    for (size_t i=0; i<object_bbox_array.boxes.size(); ++i) {
        const auto& object = object_bbox_array.boxes[i];
        int id = object_array.data_array[i].id;

        if (objects_.find(id) != objects_.end()) {
            vision_msgs::msg::BoundingBox2D old_object = objects_[id];

            double distance = std::sqrt(std::pow(object.center.position.x - old_object.center.position.x, 2) +
                                        std::pow(object.center.position.y - old_object.center.position.y, 2));
            if (distance <= dist_margin_) {
                continue;
            }
            double width_diff = std::abs(object.size_x - old_object.size_x);
            double height_diff = std::abs(object.size_y - old_object.size_y);
            if (width_diff <= size_margin_ && height_diff <= size_margin_) {
                continue;
            }
        }

        ret[id] = object;
    }
    return ret;
}
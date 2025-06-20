
#include "perception_node.hpp"

#include "climb_checker.hpp"
#include "yaml-cpp/yaml.h"

namespace A1::perception
{

ClimbChecker::ClimbChecker(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
{
    this->node_ptr = node_ptr_;
    this->climbing_timeout = config["timeout_ms"].as<int>();
    this->climbing_pitch_alpha = config["pitch_alpha"].as<float>();
    this->enable_climbing_threshold = config["enable_climbing_threshold"].as<float>() * M_PI / 180;
    this->disable_climbing_threshold = config["disable_climbing_threshold"].as<float>() * M_PI / 180;
    this->estimated_bias = 0;
}

void ClimbChecker::reset()
{
    this->climb_flag = false;
    this->estimated_bias = 0;
}

void ClimbChecker::setClimb(bool is_climb)
{
    this->climb_flag = is_climb;
}

bool ClimbChecker::isClimb()
{
    return this->climb_flag;
}

void ClimbChecker::climbCheck()
{
    auto node = this->node_ptr;
    float pitch = node->getIMUdata().pitch;

    if ((!this->isClimb()) && ((pitch - this->estimated_bias) < this->enable_climbing_threshold))
    {
        RCLCPP_INFO(
            node->get_logger(), "Remove All Drop off data, pause low obstacle detect by climb. Pitch : %f", pitch);
        this->setClimb(true);
        this->climbing_time = node->now();
    }
    else if (this->isClimb() && (pitch - this->estimated_bias) >= this->disable_climbing_threshold)
    {
        if ((node->now() - this->climbing_time) > rclcpp::Duration::from_seconds(this->climbing_timeout / 1000.0))
        {
            this->setClimb(false);
            this->estimated_bias = 0;
        }
    }

    if (this->isClimb())
    {
        node->clearDropOffLayerMap();
    }
    else
    {
        this->estimated_bias = this->computeExponentialWeightMovingAverage(this->estimated_bias, pitch);
    }

    this->publishClimbState();
}

void ClimbChecker::publishClimbState(void)
{
    auto node = this->node_ptr;
    std_msgs::msg::Bool climb_msg{};
    climb_msg.data = this->isClimb();
    auto pub_ptr =
        std::any_cast<std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>>(node->getPublishers()["climb"]);
    pub_ptr->publish(climb_msg);
}

double ClimbChecker::computeExponentialWeightMovingAverage(double prev, double curr)
{
    return (1.0 - this->climbing_pitch_alpha) * prev + this->climbing_pitch_alpha * curr;
}
}  // namespace A1::perception
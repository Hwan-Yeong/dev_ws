#include "error_monitor/error_monitor_node.hpp"

ErrorMonitorNode::ErrorMonitorNode()
    : Node("airbot_error_monitor")
{
    rclcpp::QoS qos_state_profile = rclcpp::QoS(5).reliable().durability_volatile();

    initVariables();
    setParams();

    // Subscriber
    bottom_ir_data_sub_ = this->create_subscription<robot_custom_msgs::msg::BottomIrData>(
        "bottom_ir_data", 10, std::bind(&ErrorMonitorNode::bottomIrDataCallback, this, std::placeholders::_1)
    );
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_data", 10, std::bind(&ErrorMonitorNode::imuCallback, this, std::placeholders::_1)
    );
    battery_status_sub_ = this->create_subscription<robot_custom_msgs::msg::BatteryStatus>(
        "/battery_status", 10, std::bind(&ErrorMonitorNode::batteryCallback, this, std::placeholders::_1)
    );
    station_data_sub_ = this->create_subscription<robot_custom_msgs::msg::StationData>(
        "/station_data", 10, std::bind(&ErrorMonitorNode::stationDataCallback, this, std::placeholders::_1)
    );
    robot_state_sub_ = this->create_subscription<robot_custom_msgs::msg::RobotState>(
        "/state_datas", qos_state_profile, std::bind(&ErrorMonitorNode::robotStateCallback, this, std::placeholders::_1)
    );
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&ErrorMonitorNode::odomCallback, this, std::placeholders::_1)
    );
    tof_sub_ = this->create_subscription<robot_custom_msgs::msg::TofData>(
        "/tof_data", 10, std::bind(&ErrorMonitorNode::tofCallback, this, std::placeholders::_1)
    );

    // Publisher
    fall_down_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/s_code/fall_down", 20);
    low_battery_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/s_code/low_battery", 10);
    board_overheat_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/s_code/board_overheat", 10);
    battery_discharge_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/s_code/discharging_battery", 10);
    charging_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/e_code/charging", 10);
    lift_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/s_code/lifted", 10);
    cliff_detection_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/s_code/cliff_detected", 10);
    one_d_tof_detection_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/s_code/top_tof_obstacle_error", 10);

    // Timer
    timer_ = this->create_wall_timer(
        10ms,
        std::bind(&ErrorMonitorNode::errorMonitor, this));
    RCLCPP_INFO(this->get_logger(), "node initialized");
}

ErrorMonitorNode::~ErrorMonitorNode()
{
    RCLCPP_INFO(this->get_logger(), "node terminated");
}

void ErrorMonitorNode::init()
{
    addMonitor<LowBatteryErrorMonitor>(std::make_shared<LowBatteryErrorMonitor>());
    addMonitor<FallDownErrorMonitor>(std::make_shared<FallDownErrorMonitor>());
    addMonitor<BoardOverheatErrorMonitor>(std::make_shared<BoardOverheatErrorMonitor>());
    addMonitor<BatteryDischargingErrorMonitor>(std::make_shared<BatteryDischargingErrorMonitor>());
    addMonitor<ChargingErrorMonitor>(std::make_shared<ChargingErrorMonitor>());
    addMonitor<LiftErrorMonitor>(std::make_shared<LiftErrorMonitor>());
    addMonitor<CliffDetectionErrorMonitor>(std::make_shared<CliffDetectionErrorMonitor>());
    addMonitor<TofErrorMonitor>(std::make_shared<TofErrorMonitor>());
}

void ErrorMonitorNode::initVariables()
{
    update_battery_status_low_battery = false;
    update_battery_status_battery_discharging = false;
    update_battery_status_charging = false;
    update_bottom_ir_data_fall_down = false;
    update_bottom_ir_data_lift = false;
    update_bottom_ir_data_cliff_detection = false;
    update_imu_fall_down = false;
    update_imu_lift = false;
    update_station_data_charging = false;
    update_robot_state_low_battery = false;
    update_robot_state_battery_discharging = false;
    update_robot_state_cliff_detection = false;
    update_odom_data_cliff_detection = false;
    update_tof_one_d_detection = false;

    publish_cnt_low_battery_error_ = 0;
    publish_cnt_fall_down_error_ = 0;
    publish_cnt_board_overheat_error_ = 0;
    publish_cnt_battery_discharge_error_ = 0;
    publish_cnt_charging_error_ = 0;
    publish_cnt_lift_error_ = 0;
    publish_cnt_cliff_detection_error_ = 0;
    publish_cnt_tof_detection_error_ = 0;

    bottom_ir_data = robot_custom_msgs::msg::BottomIrData();
    imu_data = sensor_msgs::msg::Imu();
    battery_data = robot_custom_msgs::msg::BatteryStatus();
    station_data = robot_custom_msgs::msg::StationData();
    odom_data = nav_msgs::msg::Odometry();
    tof_data = robot_custom_msgs::msg::TofData();
}

void ErrorMonitorNode::setParams()
{
    this->declare_parameter<int>("low_battery_error.monitoring_rate_ms", 1000);
    this->declare_parameter<int>("discharging_error.monitoring_rate_ms", 1000);
    this->declare_parameter<int>("board_overheat_error.monitoring_rate_ms", 1000);
    this->declare_parameter<int>("charging_error.monitoring_rate_ms", 1000);
    this->declare_parameter<int>("fall_down_error.monitoring_rate_ms", 1000);
    this->declare_parameter<int>("lift_error.monitoring_rate_ms", 10);
    this->declare_parameter<int>("cliff_error.monitoring_rate_ms", 10);
    this->declare_parameter<int>("tof_sensor.monitoring_rate_ms", 50);

    this->get_parameter("low_battery_error.monitoring_rate_ms", publish_cnt_low_battery_error_rate_);
    this->get_parameter("discharging_error.monitoring_rate_ms", publish_cnt_battery_discharge_error_rate_);
    this->get_parameter("board_overheat_error.monitoring_rate_ms", publish_cnt_board_overheat_error_rate_);
    this->get_parameter("charging_error.monitoring_rate_ms", publish_cnt_charging_error_rate_);
    this->get_parameter("fall_down_error.monitoring_rate_ms", publish_cnt_fall_down_error_rate_);
    this->get_parameter("lift_error.monitoring_rate_ms", publish_cnt_lift_error_rate_);
    this->get_parameter("cliff_error.monitoring_rate_ms", publish_cnt_cliff_detection_error_rate_);
    this->get_parameter("tof_sensor.monitoring_rate_ms", publish_cnt_tof_detection_error_rate_);

    RCLCPP_INFO(this->get_logger(), "=================== ERROR MONITOR PARAMETER ===================");
    RCLCPP_INFO(this->get_logger(), "Low Battery Rate: %d ms", publish_cnt_low_battery_error_rate_);
    RCLCPP_INFO(this->get_logger(), "Fall Down Rate: %d ms", publish_cnt_fall_down_error_rate_);
    RCLCPP_INFO(this->get_logger(), "Board Overheat Rate: %d ms", publish_cnt_board_overheat_error_rate_);
    RCLCPP_INFO(this->get_logger(), "Battery Discharge Rate: %d ms", publish_cnt_battery_discharge_error_rate_);
    RCLCPP_INFO(this->get_logger(), "Charging Rate: %d ms", publish_cnt_charging_error_rate_);
    RCLCPP_INFO(this->get_logger(), "Lift Error Rate: %d ms", publish_cnt_lift_error_rate_);
    RCLCPP_INFO(this->get_logger(), "Cliff Detection Error Rate: %d ms", publish_cnt_cliff_detection_error_rate_);
    RCLCPP_INFO(this->get_logger(), "===============================================================");
}

void ErrorMonitorNode::errorMonitor()
{
    std_msgs::msg::Bool error_msg;

    publish_cnt_low_battery_error_ +=10;
    publish_cnt_fall_down_error_ +=10;
    publish_cnt_board_overheat_error_ +=10;
    publish_cnt_battery_discharge_error_ += 10;
    publish_cnt_charging_error_ += 10;
    publish_cnt_lift_error_ += 10;
    publish_cnt_cliff_detection_error_ += 10;
    publish_cnt_tof_detection_error_ += 10;

    // fall down monitor
    if (update_bottom_ir_data_fall_down && update_imu_fall_down
        && (publish_cnt_fall_down_error_ >= publish_cnt_fall_down_error_rate_)) {
        bool fall_down_error = this->runMonitor<FallDownErrorMonitor>(std::make_pair(bottom_ir_data, imu_data));
        if (fall_down_error) {
            //RCLCPP_INFO(this->get_logger(), "fall_down_error : %s", fall_down_error ? "true" : "false");
            error_msg.data = true;
            fall_down_error_pub_->publish(error_msg);
        } else {
            error_msg.data = false;
            fall_down_error_pub_->publish(error_msg);
        }
        publish_cnt_fall_down_error_ = 0;
        update_bottom_ir_data_fall_down = false;
        update_imu_fall_down = false;
    }

    // low battery monitor
    if (update_battery_status_low_battery && update_robot_state_low_battery
        && (publish_cnt_low_battery_error_ >= publish_cnt_low_battery_error_rate_)) {
        bool low_battery_error = this->runMonitor<LowBatteryErrorMonitor>(std::make_pair(battery_data, station_data));
        if (low_battery_error) {
            // RCLCPP_INFO(this->get_logger(), "low_battery_error : %s", low_battery_error ? "true" : "false");
            error_msg.data = true;
            low_battery_error_pub_->publish(error_msg);
        } else {
            error_msg.data = false;
            low_battery_error_pub_->publish(error_msg);
        }
        publish_cnt_low_battery_error_ = 0;
        update_battery_status_low_battery = false;
    }

    // board overheat monitor
    if (publish_cnt_board_overheat_error_ >= publish_cnt_board_overheat_error_rate_) {
        bool board_overheat_error = this->runMonitor<BoardOverheatErrorMonitor>(std::nullptr_t());
        if (board_overheat_error) {
            // RCLCPP_INFO(this->get_logger(), "board_overheat_error : %s", board_overheat_error ? "true" : "false");
            // error_msg.data = true;
            // board_overheat_error_pub_->publish(error_msg);
        } else {
            // icbaek, 2025.03.19 : false 여도 publish하지 않게 하였음.
            // error_msg.data = false;
            // board_overheat_error_pub_->publish(error_msg);
        }
        publish_cnt_board_overheat_error_ = 0;
    }

    // battery discharging monitor
    if (update_battery_status_battery_discharging && update_robot_state_battery_discharging
        && (publish_cnt_battery_discharge_error_ >= publish_cnt_battery_discharge_error_rate_)) {
        bool battery_discharge_error = this->runMonitor<BatteryDischargingErrorMonitor>(std::make_pair(battery_data, station_data));
        if (battery_discharge_error) {
            // RCLCPP_INFO(this->get_logger(), "battery_discharge_error : %s", battery_discharge_error ? "true" : "false");
            error_msg.data = true;
            battery_discharge_error_pub_->publish(error_msg);
        } else {
            error_msg.data = false;
            battery_discharge_error_pub_->publish(error_msg);
        }
        publish_cnt_battery_discharge_error_ = 0;
        update_battery_status_battery_discharging = false;
    }

    // charging monitor
    if (update_station_data_charging && update_battery_status_charging
        && (publish_cnt_charging_error_ >= publish_cnt_charging_error_rate_)) {
        bool charging_error = this->runMonitor<ChargingErrorMonitor>(std::make_tuple(battery_data, station_data, robot_state));
        if (charging_error) {
            //RCLCPP_INFO(this->get_logger(), "charging_error : %s", charging_error ? "true" : "false");
            error_msg.data = true;
            charging_error_pub_->publish(error_msg);
        } else {
            error_msg.data = false;
            charging_error_pub_->publish(error_msg);
        }
        publish_cnt_charging_error_ = 0;
        update_station_data_charging = false;
        update_battery_status_charging = false;
    }

    // lift monitor
    if (update_bottom_ir_data_lift && update_imu_lift
        && (publish_cnt_lift_error_ >= publish_cnt_lift_error_rate_)) {
        bool lift_error = this->runMonitor<LiftErrorMonitor>(std::make_pair(bottom_ir_data, imu_data));
        if (lift_error) {
            // RCLCPP_INFO(this->get_logger(), "lift_error : %s", lift_error ? "true" : "false");
            error_msg.data = true;
            lift_error_pub_->publish(error_msg);
        } else {
            error_msg.data = false;
            lift_error_pub_->publish(error_msg);
        }
        publish_cnt_lift_error_ = 0;
        update_bottom_ir_data_lift = false;
        update_imu_lift = false;
    }

    // cliff detectoin monitor
    if (update_bottom_ir_data_cliff_detection && update_odom_data_cliff_detection && update_robot_state_cliff_detection
        && (publish_cnt_cliff_detection_error_ >= publish_cnt_cliff_detection_error_rate_)) {
        bool cliff_detection_error = this->runMonitor<CliffDetectionErrorMonitor>(std::make_tuple(bottom_ir_data, odom_data, robot_state));
        if (cliff_detection_error) {
            // RCLCPP_INFO(this->get_logger(), "cliff_detection_error : %s", cliff_detection_error ? "true" : "false");
            error_msg.data = true;
            cliff_detection_error_pub_->publish(error_msg);
        } else {
            error_msg.data = false;
            cliff_detection_error_pub_->publish(error_msg);
        }
        publish_cnt_cliff_detection_error_ = 0;
        update_bottom_ir_data_cliff_detection = false;
        update_odom_data_cliff_detection = false;
        update_robot_state_cliff_detection = false;
    }

    // 1D ToF Error Monitor
    if (update_tof_one_d_detection && (publish_cnt_tof_detection_error_ >= publish_cnt_tof_detection_error_rate_))
    {
        bool tof_sensor_error = this->runMonitor<TofErrorMonitor>(tof_data);
        if (tof_sensor_error) {            
            error_msg.data = true;
            one_d_tof_detection_error_pub_->publish(error_msg);
        } else {
            error_msg.data = false;
            one_d_tof_detection_error_pub_->publish(error_msg);
        }
        publish_cnt_tof_detection_error_ = 0;
        update_tof_one_d_detection = false;
    }

    // publish_cnt_* 변수 오버플로우 방지
    if (publish_cnt_low_battery_error_ >= 100000) publish_cnt_low_battery_error_ = 0;
    if (publish_cnt_fall_down_error_ >= 100000) publish_cnt_fall_down_error_ = 0;
    if (publish_cnt_board_overheat_error_ >= 100000) publish_cnt_board_overheat_error_ = 0;
    if (publish_cnt_battery_discharge_error_ >= 100000) publish_cnt_battery_discharge_error_ = 0;
    if (publish_cnt_charging_error_ >= 100000) publish_cnt_charging_error_ = 0;
    if (publish_cnt_lift_error_ >= 100000) publish_cnt_lift_error_ = 0;
    if (publish_cnt_cliff_detection_error_ >= 100000) publish_cnt_cliff_detection_error_ = 0;
    if (publish_cnt_tof_detection_error_ >= 100000) publish_cnt_tof_detection_error_ = 0;
}

void ErrorMonitorNode::batteryCallback(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg)
{
    battery_data = *msg;
    update_battery_status_low_battery = true;
    update_battery_status_battery_discharging = true;
    update_battery_status_charging = true;
}

void ErrorMonitorNode::bottomIrDataCallback(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg)
{
    bottom_ir_data = *msg;
    update_bottom_ir_data_fall_down = true;
    update_bottom_ir_data_lift = true;
    update_bottom_ir_data_cliff_detection = true;
}

void ErrorMonitorNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_data = *msg;
    update_imu_fall_down = true;
    update_imu_lift = true;
}

void ErrorMonitorNode::stationDataCallback(const robot_custom_msgs::msg::StationData::SharedPtr msg)
{
    station_data = *msg;
    update_station_data_charging = true;
}

void ErrorMonitorNode::robotStateCallback(const robot_custom_msgs::msg::RobotState::SharedPtr msg)
{
    robot_state = *msg;
    update_robot_state_low_battery = true;
    update_robot_state_battery_discharging = true;
    update_robot_state_cliff_detection = true;
}

void ErrorMonitorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_data = *msg;
    update_odom_data_cliff_detection = true;
}

void ErrorMonitorNode::tofCallback(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    tof_data = *msg;
    update_tof_one_d_detection = true;
}
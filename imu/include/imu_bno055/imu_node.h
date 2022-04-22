#pragma once
/*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Monash Nova Rover Team

This class monitors for data from the IMU, and
publishes raw data to a ROS topic to be interpreted
in python.
This code is adapted for ROS2 from code in
https://github.com/leighleighleigh/ros-imu-bno055
by Leigh Oliver, modified from original code in
https://github.com/dheera/ros-imu-bno055
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
NODE: IMU_pub
TOPICS:
  - /elec/imu_pub
SERVICES: None
ACTIONS:  None
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PACKAGE:    ros2-imu-bno055
AUTHOR(S):  Max Tory
CREATION:   22/04/2022
EDITED:     22/04/2022
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#include "rclcpp/rclcpp.hpp"
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <linux/i2c-dev.h>
#include <smbus_functions.h>
#include "watchdog/watchdog.h"
#include <imu_bno055/imu_activity.h>

using namespace std::chrono_literals;
using namespace imu_bno055;
using std::placeholders::_1;
using std::placeholders::_2;

class IMUNode : public rclcpp::Node {

    public:
    
    // default constructor
    IMUNode();
    ~IMUNode();

    // --------------------------------------------------------------//
    private:

    // Clock for accessing IMU data
    rclcpp::TimerBase::SharedPtr timer;

    // Publisher for IMU data
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;

    // Publisher for Euler angles
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr eulerPublisher;

    // Services to reset and calibrate the device
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetService;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrateService;

    imu_bno055::IMUActivity imu;
    watchdog::Watchdog watchdog;

    // --------------------------------------------------------------//
    private:
    
    /// @brief Gets data from the IMU and publishes it
    void publishData();

    /// @brief Fills IMU message with relevant data
    void fillIMUData(sensor_msgs::msg::Imu& msgIMU, imu_bno055::IMURecord& imuData);

    /// @brief Fills Euler angle message with relevant data
    void fillEulerData(geometry_msgs::msg::Vector3Stamped& msgEuler, imu_bno055::IMURecord& imuData);

    /// @brief Resets the IMU activity object
    void onServiceReset(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);

    /// @brief Calibrates the IMU activity object
    void onServiceCalibrate(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
};

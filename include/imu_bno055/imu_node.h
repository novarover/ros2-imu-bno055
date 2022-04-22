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

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/KeyValue.h>

#include <linux/i2c-dev.h>
#include <smbus_functions.h>
#include "watchdog/watchdog.h"
#include <imu_bno055/imu_activity.h>

using namespace std::chrono_literals;
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
    rclcpp::Publisher<sensor_msgs::Imu> imuPublisher;

    // Publisher for Euler angles
    rclcpp::Publisher<geometry_msgs::Vector3Stamped eulerPublisher;

    // Services to reset and calibrate the device
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetService;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrateService;

    IMUActivity imu;
    watchdog::Watchdog watchdog;
    // --------------------------------------------------------------//
    private:
    
    /// @brief Gets data from the IMU and publishes it
    void publishData();

}

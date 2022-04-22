/*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Monash Nova Rover Team

PACKAGE:    imu
AUTHOR(S):  Max Tory
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#include <imu_bno055/imu_node.h>

IMUNode::IMUNode() : Node("IMU_Node"){
    // Set up the object that interacts with the raw IMU data
    imu.start(this->get_logger());
    
    // Initialise ros timer
    timer = this->create_wall_timer(100ms, std::bind(&IMUNode::publishData, this));
    
    // Initialise publisher for all IMU data
    imuPublisher = this->create_publisher<sensor_msgs::Imu>("/imu/data", 10);

    // Initialise publisher for Euler angles
    eulerPublisher = this->create_publisher<sensor_msgs::Vector3Stamped("/imu/euler", 10);
    
    // Initialise reset service
    resetService = this->create_service<std_srvs::srv::Trigger>(
        "/imu/reset",
        std::bind(&IMUActivity::onServiceReset, &imu, _1, _2)
    );

    // Initialise calibration service
    calibrateService = this->create_service<std_srvs::srv::Trigger>(
        "/imu/calibrate",
        std::bind(&ImuActivity::onServiceCalibrate, &imu, _1, _2)
    );

    seq = 0;

    // Set up the watchdog to monitor the IMU
    this->watchdog.start(5000ms);
}

IMUNode::~IMUNode(){
    // Destructor stops the imu and the watchdog
    imu.stop(this->get_logger());
    this->watchdog.stop();
}

void IMUNode::publishData(){
    // Get data from the IMU and publish it
    sensor_msgs::Imu msgIMU;
    geometry_msgs::Vector3Stamped msgEuler;

    msgIMU.header.stamp = this->now();
    msgIMU.header.frame_id = "imu";
    msgIMU.header.seq = seq;

    msgEuler.header.stamp = this->now();
    msgEuler.header.frame_id = "euler";
    msgEuler.header.seq = seq;

    IMURecord imuData = imu.getData(this->get_logger());

    fillIMUData(msgIMU, imuData);
    fillEulerData(msgEuler, imuData);

    if (imuData.system_status == 5){
        // Only publish if status is good
        imuPublisher.publish(msgIMU);
        eulerPublisher.publish(msgEuler);

        seq++;    
    }
}

void IMUNode::fillIMUData(sensor_msgs::Imu& msgIMU, IMURecord& imuData){
    /*
    Fills a ROS message with data from the IMU
    :param msgIMU: message to be sent over ROS
    :param imuData: raw data struct filled by the IMU
    */
    double quat_norm = std::pow(
        std::pow(imuData.fused_orientation_w, 2) +
        std::pow(imuData.fused_orientation_x, 2) +
        std::pow(imuData.fused_orientation_y, 2) +
        std::pow(imuData.fused_orientation_z, 2), 0.5
    );

    // orientation quaternion
    msgIMU.orientation.x = (double) imuData.fused_orientation_x / quat_norm;
    msgIMU.orientation.y = (double) imuData.fused_orientation_x / quat_norm;
    msgIMU.orientation.z = (double) imuData.fused_orientation_x / quat_norm;
    msgIMU.orientation.w = (double) imuData.fused_orientation_x / quat_norm;

    // Linear acceleration
    msgIMU.linear_acceleration.x = (double)imuData.fused_linear_acceleration_x / 100.0;
    msgIMU.linear_acceleration.y = (double)imuData.fused_linear_acceleration_y / 100.0;
    msgIMU.linear_acceleration.z = (double)imuData.fused_linear_acceleration_z / 100.0;

    // Angular velocity
    msgIMU.angular_velocity.x = (double)imuData.raw_angular_velocity_x / 900.0;
    msgIMU.angular_velocity.y = (double)imuData.raw_angular_velocity_y / 900.0;
    msgIMU.angular_velocity.z = (double)imuData.raw_angular_velocity_z / 900.0;
}

void IMUNode::fillEulerData(geometry_msgs::Vector3Stamped& msgEuler, IMURecord& imuData){
    /*
    Fills a ROS message with orientation data in the form of Euler angles
    :param msgEuler: message to be sent over ROS
    :param imuData: raw data struct filled by the IMU
    */
    msg_euler.vector.x = (double)record.fused_roll / (double)16;
    msg_euler.vector.y = (double)record.fused_pitch / (double)16;
    msg_euler.vector.z = (double)record.fused_heading / (double)16;
}

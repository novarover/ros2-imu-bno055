/*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Monash Nova Rover Team

PACKAGE:    imu
AUTHOR(S):  Max Tory
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#include <imu_bno055/imu_activity.h>

IMUActivity::IMUActivity(){
    // Initialising diagnostic things
    current_status.level = 0;
    current_status.name = "BNO055 IMU";
    current_status.hardware_id = "bno055_i2c";

    diagnostic_msgs::KeyValue calib_stat;
    calib_stat.key = "Calibration status";
    calib_stat.value = "";
    current_status.values.push_back(calib_stat);

    diagnostic_msgs::KeyValue selftest_result;
    selftest_result.key = "Self-test result";
    selftest_result.value = "";
    current_status.values.push_back(selftest_result);

    diagnostic_msgs::KeyValue intr_stat;
    intr_stat.key = "Interrupt status";
    intr_stat.value = "";
    current_status.values.push_back(intr_stat);

    diagnostic_msgs::KeyValue sys_clk_stat;
    sys_clk_stat.key = "System clock status";
    sys_clk_stat.value = "";
    current_status.values.push_back(sys_clk_stat);

    diagnostic_msgs::KeyValue sys_stat;
    sys_stat.key = "System status";
    sys_stat.value = "";
    current_status.values.push_back(sys_stat);

    diagnostic_msgs::KeyValue sys_err;
    sys_err.key = "System error";
    sys_err.value = "";
    current_status.values.push_back(sys_err);
}

// ******** private methods ******** //

bool BNO055I2CActivity::reset(rclcpp::logger& logger) {
    // TODO: use non-blocking sleeps
    int i = 0;

    _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
    std::sleep(0.025);

    // reset
    _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0x20);
    std::sleep(0.025);

    // wait for chip to come back online
    while(_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        sleep(0.010);
        if(i++ > 500) {
            ROS_ERROR_STREAM(logger, "chip did not come back online in 5 seconds after reset");
            return false;
        }
    }
    std::sleep(0.100);

    // normal power mode
    _i2c_smbus_write_byte_data(file, BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
    std::sleep(0.010);

    _i2c_smbus_write_byte_data(file, BNO055_PAGE_ID_ADDR, 0);
    _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0);
    std::sleep(0.025);

    _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF); // NDOF is absolute orientation
    std::sleep(0.025);

    return true;
}

// ******** public methods ******** //

bool BNO055I2CActivity::start(rclcpp::logger& logger) {
    ROS_INFO(logger, "starting");

    // Handle to the I2C bus
    file = open(param_device.c_str(), O_RDWR);

    // Check that we have access to this bus
    if(ioctl(file, I2C_SLAVE, param_address) < 0) {
        ROS_ERROR(logger, "i2c open failed, check permissions");
        return false;
    }

    int checkAddressCounter = 0;
    bool foundDevice = false;

    while(!foundDevice)
    {
        // Update the address we are accessing
        if(ioctl(file, I2C_SLAVE, param_address) < 0) {
            ROS_ERROR(logger, "i2c open failed, check permissions");
            return false;
        }
        
        // Get chip ID from chip @ address
        uint8_t foundChipID = _i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR);
        
        if(foundChipID != BNO055_ID) {
            ROS_WARN(logger, "incorrect chip ID. want=0x%x, got=0x%x",BNO055_ID,foundChipID);
            ROS_WARN(logger, "No BNO055 found on bus %s, at address 0x%02x",param_device.c_str(),param_address);
        }else{
            ROS_INFO(logger, "Found BNO055 on bus %s, at address 0x%02x",param_device.c_str(),param_address);
            // We have found the bno!
            foundDevice = true;
        }

        // If we still havn't found the device
        if(!foundDevice)
        {
            // Change the param_address and scan A/B addresses
            if(param_check_all_addresses)
            {
                // Change address
                if(checkAddressCounter == 0)
                {
                    param_address = (int)BNO055_ADDRESS_A;

                }
                if(checkAddressCounter == 1)
                {
                    param_address = (int)BNO055_ADDRESS_B;
                }

                // Out of addresses to check
                if(checkAddressCounter == 2)
                {
                    // Failed to find device
                    // return false.
                    if(!foundDevice)
                    {
                        ROS_ERROR(logger, "no BNO055 devices found on any address");
                        return false;
                    }
                }
                 
                // Increment counter for the loop
                checkAddressCounter++;
            }
        }
    }

    ROS_INFO_STREAM(logger, "rev ids:"
      << " accel:" << _i2c_smbus_read_byte_data(file, BNO055_ACCEL_REV_ID_ADDR)
      << " mag:" << _i2c_smbus_read_byte_data(file, BNO055_MAG_REV_ID_ADDR)
      << " gyro:" << _i2c_smbus_read_byte_data(file, BNO055_GYRO_REV_ID_ADDR)
      << " sw:" << _i2c_smbus_read_word_data(file, BNO055_SW_REV_ID_LSB_ADDR)
      << " bl:" << _i2c_smbus_read_byte_data(file, BNO055_BL_REV_ID_ADDR));

    if(!reset()) {
        ROS_ERROR(logger, "chip reset and setup failed");
        return false;
    }

    return true;
}

IMURecord BNO055I2CActivity::getData() {
    IMURecord record;

    unsigned char c = 0;

    // can only read a length of 0x20 at a time, so do it in 2 reads
    // BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR is the start of the data block that aligns with the IMURecord struct
    if(_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR, 0x20, (uint8_t*)&record) != 0x20) {
        return NULL;
    }
    if(_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR + 0x20, 0x13, (uint8_t*)&record + 0x20) != 0x13) {
        return NULL;
    }

    /*if(seq % abs(param_diag_pub_interval) == 0) {
        current_status.values[DIAG_CALIB_STAT].value = std::to_string(record.calibration_status);
        current_status.values[DIAG_SELFTEST_RESULT].value = std::to_string(record.self_test_result);
        current_status.values[DIAG_INTR_STAT].value = std::to_string(record.interrupt_status);
        current_status.values[DIAG_SYS_CLK_STAT].value = std::to_string(record.system_clock_status);
        current_status.values[DIAG_SYS_STAT].value = std::to_string(record.system_status);
        current_status.values[DIAG_SYS_ERR].value = std::to_string(record.system_error_code);
        pub_status.publish(current_status);
    }*/

    return record;
}

bool BNO055I2CActivity::onServiceReset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    if(!reset()) {
        return false;
    }
    return true;
}

bool BNO055I2CActivity::onServiceCalibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    // TODO implement this
    return true;
}
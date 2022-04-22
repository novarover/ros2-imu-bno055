/*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Monash Nova Rover Team

PACKAGE:    imu
AUTHOR(S):  Max Tory
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#include <imu_bno055/imu_activity.h>
using namespace imu_bno055;
using namespace std::literals::chrono_literals;

IMUActivity::IMUActivity(){
    // Initialising diagnostic things
    current_status.level = 0;
    current_status.name = "BNO055 IMU";
    current_status.hardware_id = "bno055_i2c";

    diagnostic_msgs::msg::KeyValue calib_stat;
    calib_stat.key = "Calibration status";
    calib_stat.value = "";
    current_status.values.push_back(calib_stat);

    diagnostic_msgs::msg::KeyValue selftest_result;
    selftest_result.key = "Self-test result";
    selftest_result.value = "";
    current_status.values.push_back(selftest_result);

    diagnostic_msgs::msg::KeyValue intr_stat;
    intr_stat.key = "Interrupt status";
    intr_stat.value = "";
    current_status.values.push_back(intr_stat);

    diagnostic_msgs::msg::KeyValue sys_clk_stat;
    sys_clk_stat.key = "System clock status";
    sys_clk_stat.value = "";
    current_status.values.push_back(sys_clk_stat);

    diagnostic_msgs::msg::KeyValue sys_stat;
    sys_stat.key = "System status";
    sys_stat.value = "";
    current_status.values.push_back(sys_stat);

    diagnostic_msgs::msg::KeyValue sys_err;
    sys_err.key = "System error";
    sys_err.value = "";
    current_status.values.push_back(sys_err);
}

// ******** private methods ******** //

bool IMUActivity::reset(const rclcpp::Logger& logger) {
    int i = 0;

    _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
    std::this_thread::sleep_for(0.025s);

    // reset
    _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0x20);
    std::this_thread::sleep_for(0.025s);

    // wait for chip to come back online
    while(_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        std::this_thread::sleep_for(0.010s);
        if(i++ > 500) {
            RCLCPP_ERROR_STREAM(logger, "chip did not come back online in 5 seconds after reset");
            return false;
        }
    }
    std::this_thread::sleep_for(0.100s);

    // normal power mode
    _i2c_smbus_write_byte_data(file, BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
    std::this_thread::sleep_for(0.010s);

    _i2c_smbus_write_byte_data(file, BNO055_PAGE_ID_ADDR, 0);
    _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0);
    std::this_thread::sleep_for(0.025s);

    _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF); // NDOF is absolute orientation
    std::this_thread::sleep_for(0.025s);

    return true;
}

// ******** public methods ******** //

bool IMUActivity::start(const rclcpp::Logger& logger) {
    RCLCPP_INFO(logger, "starting");

    // Handle to the I2C bus
    file = open(param_device.c_str(), O_RDWR);

    // Check that we have access to this bus
    if(ioctl(file, I2C_SLAVE, param_address) < 0) {
        RCLCPP_ERROR(logger, "i2c open failed, check permissions");
        return false;
    }

    int checkAddressCounter = 0;
    bool foundDevice = false;

    while(!foundDevice)
    {
        // Update the address we are accessing
        if(ioctl(file, I2C_SLAVE, param_address) < 0) {
            RCLCPP_ERROR(logger, "i2c open failed, check permissions");
            return false;
        }
        
        // Get chip ID from chip @ address
        uint8_t foundChipID = _i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR);
        
        if(foundChipID != BNO055_ID) {
            RCLCPP_WARN(logger, "incorrect chip ID. want=0x%x, got=0x%x",BNO055_ID,foundChipID);
            RCLCPP_WARN(logger, "No BNO055 found on bus %s, at address 0x%02x",param_device.c_str(),param_address);
        }else{
            RCLCPP_INFO(logger, "Found BNO055 on bus %s, at address 0x%02x",param_device.c_str(),param_address);
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
                        RCLCPP_ERROR(logger, "no BNO055 devices found on any address");
                        return false;
                    }
                }
                 
                // Increment counter for the loop
                checkAddressCounter++;
            }
        }
    }

    RCLCPP_INFO_STREAM(logger, "rev ids:"
      << " accel:" << _i2c_smbus_read_byte_data(file, BNO055_ACCEL_REV_ID_ADDR)
      << " mag:" << _i2c_smbus_read_byte_data(file, BNO055_MAG_REV_ID_ADDR)
      << " gyro:" << _i2c_smbus_read_byte_data(file, BNO055_GYRO_REV_ID_ADDR)
      << " sw:" << _i2c_smbus_read_word_data(file, BNO055_SW_REV_ID_LSB_ADDR)
      << " bl:" << _i2c_smbus_read_byte_data(file, BNO055_BL_REV_ID_ADDR));

    if(!reset(logger)) {
        RCLCPP_ERROR(logger, "chip reset and setup failed");
        return false;
    }

    return true;
}

IMURecord IMUActivity::getData() {
    IMURecord record;

    // can only read a length of 0x20 at a time, so do it in 2 reads
    // BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR is the start of the data block that aligns with the IMURecord struct
    if(_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR, 0x20, (uint8_t*)&record) != 0x20) {
        record.system_status = 0;
    }
    if(_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR + 0x20, 0x13, (uint8_t*)&record + 0x20) != 0x13) {
        record.system_status = 0;
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

void IMUActivity::onServiceReset(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res, const rclcpp::Logger& logger) {
    if(!reset(logger)) {
        res->success = false;
    } else {
        res->success = true;
    }
}

void IMUActivity::onServiceCalibrate(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res, const rclcpp::Logger& logger) {
    // TODO implement this
    res->success = true;
}

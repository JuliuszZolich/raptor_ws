#include "can_wrapper/VescStatusHandler.hpp"

/**
 * @brief Constructor for VescStatusHandler.
 * @param nh ROS NodeHandle for communication with ROS.
 */
VescStatusHandler::VescStatusHandler(ros::NodeHandle& nh)
{
    // Subscribe to the raw CAN frame topic
    mStatusGrabber = nh.subscribe<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_RX, 1024, &VescStatusHandler::statusGrabber, this);
    // Advertise the VESC status topic
    mStatusPublisher = nh.advertise<can_wrapper::VescStatus>(RosCanConstants::RosTopics::can_vesc_status, 256);
}

/**
 * @brief Callback function to handle incoming CAN frames.
 * @param frame The received CAN frame.
 */
void VescStatusHandler::statusGrabber(const can_msgs::Frame::ConstPtr &frame)
{
    // Convert the ROS CAN frame to a VESC raw frame
    auto vescFrame = VescInterop::rosToVesc(*frame);
    // Create a key-value pair for motor status
    auto key = MotorStatusKey(vescFrame.vescID, (VESC_Command)vescFrame.command);
    auto value = MotorStatusValue(vescFrame, frame->header.stamp);

    // Find the motor status in the map
    auto findResult = mMotorStatus.find(key);

    if (findResult == mMotorStatus.cend())
    {
        // Insert new motor status if not found
        mMotorStatus.insert(std::pair<MotorStatusKey, MotorStatusValue>(key, value));
    }
    else
    {
        // Update the motor status if found
        if (key.commandId == VESC_COMMAND_STATUS_1)
            sendUpdate(key.vescId);
        mMotorStatus[key] = value;
    }
}

/**
 * @brief Sends an update for the specified VESC ID.
 * @param vescId The VESC ID to send the update for.
 */
void VescStatusHandler::sendUpdate(uint8_t vescId)
{
    MotorStatusKey key;
    can_wrapper::VescStatus status;

    // Process status 1
    key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_1);
    if (mMotorStatus.find(key) != mMotorStatus.cend())
    {
        VESC_Status_1 statusData;
        VESC_ZeroMemory(&statusData, sizeof(statusData));
        VESC_convertRawToStatus1(&statusData, &mMotorStatus[key].vescFrame);

        status.ERPM = statusData.erpm;
        status.Current = statusData.current;
        status.DutyCycle = statusData.dutyCycle;
    }

    // Process status 2
    key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_2);
    if (mMotorStatus.find(key) != mMotorStatus.cend())
    {
        VESC_Status_2 statusData;
        VESC_ZeroMemory(&statusData, sizeof(statusData));
        VESC_convertRawToStatus2(&statusData, &mMotorStatus[key].vescFrame);

        status.AhUsed = statusData.apmHours;
        status.AhCharged = statusData.apmHoursChg;
    }

    // Process status 3
    key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_3);
    if (mMotorStatus.find(key) != mMotorStatus.cend())
    {
        VESC_Status_3 statusData;
        VESC_ZeroMemory(&statusData, sizeof(statusData));
        VESC_convertRawToStatus3(&statusData, &mMotorStatus[key].vescFrame);

        status.WhUsed = statusData.wattHours;
        status.WhCharged = statusData.wattHoursChg;
    }

    // Process status 4
    key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_4);
    if (mMotorStatus.find(key) != mMotorStatus.cend())
    {
        VESC_Status_4 statusData;
        VESC_ZeroMemory(&statusData, sizeof(statusData));
        VESC_convertRawToStatus4(&statusData, &mMotorStatus[key].vescFrame);

        status.TempFet = statusData.tempFet;
        status.TempMotor = statusData.tempMotor;
        status.CurrentIn = statusData.currentIn;
        status.PidPos = statusData.pidPos;
    }

    // Process status 5
    key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_5);
    if (mMotorStatus.find(key) != mMotorStatus.cend())
    {
        VESC_Status_5 statusData;
        VESC_ZeroMemory(&statusData, sizeof(statusData));
        VESC_convertRawToStatus5(&statusData, &mMotorStatus[key].vescFrame);

        status.Tachometer = statusData.tachometer;
        status.VoltsIn = statusData.voltsIn;
    }

    // Process status 6
    key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_6);
    if (mMotorStatus.find(key) != mMotorStatus.cend())
    {
        VESC_Status_6 statusData;
        VESC_ZeroMemory(&statusData, sizeof(statusData));
        VESC_convertRawToStatus6(&statusData, &mMotorStatus[key].vescFrame);

        status.ADC1 = statusData.adc1;
        status.ADC2 = statusData.adc2;
        status.ADC3 = statusData.adc3;
        status.PPM = statusData.ppm;
    }

    // Process status 7
    key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_7);
    if (mMotorStatus.find(key) != mMotorStatus.cend())
    {
        VESC_Status_7 statusData;
        VESC_ZeroMemory(&statusData, sizeof(statusData));
        VESC_convertRawToStatus7(&statusData, &mMotorStatus[key].vescFrame);

        status.PrecisePos = statusData.precisePos;
    }

    // Set the VESC ID and timestamp
    status.VescId = key.vescId;
    status.header.stamp = ros::Time::now();

    ROS_INFO("Published status!");
    lastSendTime = ros::Time::now();
    // Publish the status message
    mStatusPublisher.publish(status);
}

/**
 * @brief Clears the motor status map.
 */
void VescStatusHandler::clear()
{
    mMotorStatus.clear();
}
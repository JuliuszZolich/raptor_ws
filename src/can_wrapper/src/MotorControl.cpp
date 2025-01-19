#include "can_wrapper/MotorControl.hpp"

/**
 * @brief Constructor for MotorControl.
 * @param nh ROS NodeHandle for communication with ROS.
 */
MotorControl::MotorControl(const ros::NodeHandle &nh) : mNh(nh)
{
    // Advertise the raw CAN frame publisher
    mRawCanPub = mNh.advertise<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_TX, 256);
    // Subscribe to the motor velocity topic
    mSetMotorVelSub = mNh.subscribe(RosCanConstants::RosTopics::can_set_motor_vel, 256, &MotorControl::handleSetMotorVel, this);
}

/**
 * @brief Handles setting motor velocity.
 * @param msg Message containing wheel velocities.
 */
void MotorControl::handleSetMotorVel(const can_wrapper::Wheels &msg)
{
    // Send the motor velocity command
    sendMotorVel(msg);
}

/**
 * @brief Sends motor velocity command.
 * @param msg Message containing wheel velocities.
 */
void MotorControl::sendMotorVel(const can_wrapper::Wheels msg)
{
    // 8 since there are 4 wheels, each being VESC + stepper combo
    std::array<can_msgs::Frame, 8> sendQueue;

    auto sendQueueIter = sendQueue.begin();

    // Stepper motor commands
    *sendQueueIter++ = encodeMotorVel
    (
        msg.frontLeft.setAngle,
        static_cast<VESC_Command>(msg.frontLeft.commandIdAngle),
        RosCanConstants::VescIds::front_left_stepper
    );

    *sendQueueIter++ = encodeMotorVel
    (
        msg.frontRight.setAngle,
        static_cast<VESC_Command>(msg.frontRight.commandIdAngle),
        RosCanConstants::VescIds::front_right_stepper
    );

    *sendQueueIter++ = encodeMotorVel
    (
        msg.rearLeft.setAngle,
        static_cast<VESC_Command>(msg.rearLeft.commandIdAngle),
        RosCanConstants::VescIds::rear_left_stepper
    );

    *sendQueueIter++ = encodeMotorVel
    (
        msg.rearRight.setAngle,
        static_cast<VESC_Command>(msg.rearRight.commandIdAngle),
        RosCanConstants::VescIds::rear_right_stepper
    );

    // VESC motor commands
    *sendQueueIter++ = encodeMotorVel
    (
        msg.frontLeft.setValue,
        static_cast<VESC_Command>(msg.frontLeft.commandId),
        RosCanConstants::VescIds::front_left_vesc
    );

    *sendQueueIter++ = encodeMotorVel
    (
        msg.frontRight.setValue,
        static_cast<VESC_Command>(msg.frontRight.commandId),
        RosCanConstants::VescIds::front_right_vesc
    );

    *sendQueueIter++ = encodeMotorVel
    (
        msg.rearLeft.setValue,
        static_cast<VESC_Command>(msg.rearLeft.commandId),
        RosCanConstants::VescIds::rear_left_vesc
    );

    *sendQueueIter++ = encodeMotorVel
    (
        msg.rearRight.setValue,
        static_cast<VESC_Command>(msg.rearRight.commandId),
        RosCanConstants::VescIds::rear_right_vesc
    );

    // Publish all commands
    for(auto iter = sendQueue.begin(); iter < sendQueue.end(); iter++)
        mRawCanPub.publish(*iter);
}

/**
 * @brief Encodes motor velocity message into CAN frame.
 * @param msg Motor velocity.
 * @param command VESC command type.
 * @param vescId VESC identifier.
 * @return Encoded CAN frame.
 */
can_msgs::Frame MotorControl::encodeMotorVel(const float msg, const VESC_Command command, const VESC_Id_t vescId)
{
    VESC_CommandFrame cmdf;
    VESC_ZeroMemory(&cmdf, sizeof(cmdf));

    // Set the command frame data
    cmdf.commandData = msg;
    cmdf.command = command;
    cmdf.vescID = vescId;

    VESC_RawFrame rf;
    VESC_ZeroMemory(&rf, sizeof(rf));
    // Convert the command frame to raw frame
    VESC_convertCmdToRaw(&rf, &cmdf);

    // Convert the raw frame to ROS CAN frame
    can_msgs::Frame fr = VescInterop::vescToRos(rf);
    fr.header.seq = mSetMotorVelSeq++; // Increment the sequence number
    return fr;
}
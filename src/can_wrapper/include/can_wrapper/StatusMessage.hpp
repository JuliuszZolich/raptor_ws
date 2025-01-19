#ifndef STATUSMESSAGE_HPP_
#define STATUSMESSAGE_HPP_

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <can_wrapper/RoverStatus.h>
#include <can_wrapper/VescInterop.hpp>
#include "can_wrapper/RosCanConstants.hpp"
extern "C"
{
#include <libVescCan/VESC.h>
}

/**
 * @brief Class for handling status messages over CAN bus.
 */
class StatusMessage
{
public:
    /**
     * @brief Constructor for StatusMessage.
     * @param nh ROS NodeHandle for communication with ROS.
     * @param sendOnUpdate Flag to send messages on update.
     */
    StatusMessage(const ros::NodeHandle &nh, bool sendOnUpdate = true);

    /**
     * @brief Sends a status message.
     * @param msg Message containing rover status.
     */
    void sendStatusMessage(const can_wrapper::RoverStatus &msg);

    /**
     * @brief Sends the last status message.
     */
    void sendStatusMessage();

private:
    /**
     * @brief Encodes a status message into a CAN frame.
     * @param msg Rover status message.
     * @return Encoded CAN frame.
     */
    can_msgs::Frame encodeStatusMessage(const can_wrapper::RoverStatus &msg);

    /**
     * @brief Handles the reception of a status message.
     * @param msg Message containing rover status.
     */
    void handleStatusMessage(const can_wrapper::RoverStatus &msg);

    ros::NodeHandle mNh; /**< ROS NodeHandle for communication with ROS. */
    uint32_t mStatusMessageSeq; /**< Sequence number for communication status messages. */
    bool mSendOnUpdate; /**< Flag to send messages on update. */
    can_wrapper::RoverStatus mLastStatus; /**< Last status message received. */

    ros::Publisher mRawCanPub; /**< ROS publisher for raw CAN messages. */
    ros::Subscriber mStatusMessageSub; /**< ROS subscriber for communication status messages. */
};

#endif // STATUSMESSAGE_HPP_
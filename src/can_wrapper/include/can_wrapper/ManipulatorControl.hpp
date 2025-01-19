#ifndef ManipulatorControl_h_
#define ManipulatorControl_h_

#include <ros/ros.h>
#include <string>
#include <can_msgs/Frame.h>
#include <array>
#include <can_wrapper/VescInterop.hpp>
#include "can_wrapper/RosCanConstants.hpp"
#include "can_wrapper/ManipulatorControl.h"
extern "C"
{
#include <libVescCan/VESC.h>
}

/**
 * @brief Class for controlling the manipulator using CAN messages.
 */
class ManipulatorControl
{
public:
    /**
     * @brief Constructor for ManipulatorControl.
     * @param nh The ROS NodeHandle.
     */
    ManipulatorControl(const ros::NodeHandle &nh);

private:
    /**
     * @brief Callback to handle manipulator control messages.
     * @param manipulatroCtlMsg The received manipulator control message.
     */
    void handleManipulatorCtl(const can_wrapper::ManipulatorControl::ConstPtr& manipulatroCtlMsg);

    /**
     * @brief Encodes a stepper message for the CAN bus.
     * @param stepper The stepper data to encode.
     * @param vescId The VESC ID.
     * @return The encoded CAN frame.
     */
    can_msgs::Frame encodeStepper(const can_wrapper::Stepper& stepper, const VESC_Id_t vescId);

    ros::NodeHandle mNh; ///< ROS NodeHandle.
    ros::Publisher mRawCanPub; ///< ROS publisher for raw CAN messages.
    ros::Subscriber mManipulatorCtlSub; ///< ROS subscriber for motor velocity messages.
};

#endif //ManipulatorControl_h_
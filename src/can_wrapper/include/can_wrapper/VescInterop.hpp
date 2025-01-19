#include <linux/can.h>
#include <can_msgs/Frame.h>
#include <memory>
extern "C"
{
#include <libVescCan/VESC_Structs.h>
}

namespace VescInterop
{
    /**
     * @brief Converts a VESC raw CAN frame to a ROS CAN frame.
     * @param rawFrame The raw CAN frame from VESC.
     * @return The corresponding ROS CAN frame.
     */
    can_msgs::Frame vescToRos(const VESC_RawFrame& rawFrame);

    /**
     * @brief Converts a ROS CAN frame to a VESC raw CAN frame.
     * @param canFrame The ROS CAN frame.
     * @return The corresponding VESC raw CAN frame.
     */
    VESC_RawFrame rosToVesc(const can_msgs::Frame& canFrame);
}
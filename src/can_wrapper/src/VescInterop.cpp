#include <can_wrapper/VescInterop.hpp>

/**
 * @brief Converts a VESC raw frame to a ROS CAN frame.
 * @param vescf The VESC raw frame to convert.
 * @return The converted ROS CAN frame.
 */
can_msgs::Frame VescInterop::vescToRos(const VESC_RawFrame& vescf)
{
    // Cast the VESC raw frame to a CAN frame
    auto canf = reinterpret_cast<const can_frame*>(&vescf);

    can_msgs::Frame rosf;

    // Set the ROS CAN frame fields based on the CAN frame
    rosf.id = canf->can_id;
    rosf.is_extended = (canf->can_id & CAN_EFF_FLAG) == CAN_EFF_FLAG;
    rosf.is_rtr = (canf->can_id & CAN_RTR_FLAG) == CAN_RTR_FLAG;
    rosf.is_error = (canf->can_id & CAN_ERR_FLAG) == CAN_ERR_FLAG;

    rosf.dlc = canf->can_dlc;
    memcpy(rosf.data.begin(), canf->data, canf->can_dlc);

    return rosf;
}

/**
 * @brief Converts a ROS CAN frame to a VESC raw frame.
 * @param rosf The ROS CAN frame to convert.
 * @return The converted VESC raw frame.
 */
VESC_RawFrame VescInterop::rosToVesc(const can_msgs::Frame& rosf)
{
    VESC_RawFrame vescf;
    VESC_ZeroMemory(&vescf, sizeof(vescf));

    // Cast the VESC raw frame to a CAN frame
    auto canf = reinterpret_cast<can_frame*>(&vescf);

    // Set the CAN frame fields based on the ROS CAN frame
    canf->can_id = rosf.id;
    canf->can_id |= rosf.is_extended ? CAN_EFF_FLAG : 0;
    canf->can_id |= rosf.is_rtr ? CAN_RTR_FLAG : 0;
    canf->can_id |= rosf.is_error ? CAN_ERR_FLAG : 0;

    // Return early if the frame is an error frame
    if(rosf.is_error)
        return vescf;
    // Return early if the data length code is greater than 8
    if (rosf.dlc > 8)
        return vescf;

    canf->can_dlc = rosf.dlc;
    memcpy(canf->data, rosf.data.begin(), rosf.dlc);

    return vescf;
}
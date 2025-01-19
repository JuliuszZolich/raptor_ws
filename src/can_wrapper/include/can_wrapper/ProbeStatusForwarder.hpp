#ifndef ProbeStatusForwarder_h_
#define ProbeStatusForwarder_h_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <can_wrapper/ProbeStatus.h>
#include <can_wrapper/RosCanConstants.hpp>
#include <can_wrapper/VescInterop.hpp>
#include <can_msgs/Frame.h>
extern "C"
{
#include <libVescCan/VESC.h>
}

/**
 * @brief Class for forwarding probe status using CAN messages.
 */
class ProbeStatusForwarder
{
public:
    /**
     * @brief Constructor for ProbeStatusForwarder.
     * @param nh The ROS NodeHandle.
     */
    ProbeStatusForwarder(ros::NodeHandle& nh);

private:
    /**
     * @brief Callback to grab probe status from CAN messages.
     * @param frame The received CAN frame.
     */
    void probeStatusGrabber(const can_msgs::Frame::ConstPtr &frame);

    /**
     * @brief Publishes the status of the probe.
     */
    void statusPublisher();

    /**
     * @brief Updates the status with new VESC status 8 data.
     * @param status8 The new VESC status 8 data.
     */
    void newStatus8(VESC_Status_8& status8);

    /**
     * @brief Updates the status with new VESC status 9 data.
     * @param status9 The new VESC status 9 data.
     */
    void newStatus9(VESC_Status_9& status9);

    /**
     * @brief Logs a warning if the status is considered rotten.
     * @param statusNum The status number to check.
     */
    void warnRotten(int statusNum);

    /**
     * @brief Logs an info message if the status is not rotten.
     */
    void infoNotRotten();

    const uint8_t ROTTEN_THRESHOLD = 10; ///< Threshold for considering the status as rotten.

    ros::NodeHandle mNh; ///< ROS NodeHandle.

    bool mRottenNoted = false; ///< Flag to indicate if rotten status has been noted.

    VESC_Status_8 mStatus8; ///< VESC status 8 data.
    bool mStatus8Fresh = false; ///< Flag to indicate if status 8 data is fresh.
    uint8_t mStatus8Staleness = 0; ///< Staleness counter for status 8 data.

    VESC_Status_9 mStatus9; ///< VESC status 9 data.
    bool mStatus9Fresh = false; ///< Flag to indicate if status 9 data is fresh.
    uint8_t mStatus9Staleness = 0; ///< Staleness counter for status 9 data.

    ros::Subscriber mProbeStatusGrabber; ///< ROS subscriber for probe status messages.
    ros::Publisher mProbeStatusPublisher; ///< ROS publisher for probe status messages.
};

#endif //ProbeStatusForwarder_h_
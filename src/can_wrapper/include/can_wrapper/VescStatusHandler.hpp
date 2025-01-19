#ifndef VescStatusHandler_h_
#define VescStatusHandler_h_

#include <ros/ros.h>
#include <unordered_map>
#include <boost/algorithm/algorithm.hpp>
#include <boost/shared_ptr.hpp>
#include <can_wrapper/VescStatus.h>
#include <can_wrapper/RosCanConstants.hpp>
#include <can_wrapper/VescInterop.hpp>
#include <can_msgs/Frame.h>
extern "C"
{
#include <libVescCan/VESC.h>
}

/**
 * @brief Structure representing a key for motor status.
 */
struct MotorStatusKey
{
    VESC_Id_t vescId; /**< VESC identifier. */
    VESC_Command commandId; /**< VESC command identifier. */

    MotorStatusKey() = default;

    /**
     * @brief Constructor for MotorStatusKey.
     * @param vescId VESC identifier.
     * @param commandId VESC command identifier.
     */
    inline MotorStatusKey(VESC_Id_t vescId, VESC_Command commandId) :
        vescId(vescId), commandId(commandId)
    {}

    /**
     * @brief Equality operator for MotorStatusKey.
     * @param rhs Right-hand side MotorStatusKey.
     * @return True if equal, false otherwise.
     */
    inline bool operator==(const MotorStatusKey &rhs) const
    {
        return vescId == rhs.vescId && commandId == rhs.commandId;
    }
};

/**
 * @brief Structure representing a value for motor status.
 */
struct MotorStatusValue
{
    VESC_RawFrame vescFrame; /**< VESC raw frame. */
    ros::Time recivedTime; /**< Time when the frame was received. */

    MotorStatusValue() = default;

    /**
     * @brief Constructor for MotorStatusValue.
     * @param vescFrame VESC raw frame.
     * @param recivedTime Time when the frame was received.
     */
    inline MotorStatusValue(VESC_RawFrame vescFrame, ros::Time recivedTime) :
        vescFrame(vescFrame), recivedTime(recivedTime)
    {}
};

/**
 * @brief Hash function for MotorStatusKey.
 */
template<class T>
struct MyHash;

/**
 * @brief Specialization of hash function for MotorStatusKey.
 */
template<>
struct MyHash<MotorStatusKey>
{
public:
    /**
     * @brief Hash function for MotorStatusKey.
     * @param key MotorStatusKey to hash.
     * @return Hash value.
     */
    inline std::size_t operator()(MotorStatusKey const& key) const
    {
        std::size_t h1 = std::hash<uint8_t>()(key.vescId);
        std::size_t h2 = std::hash<uint8_t>()(uint8_t(key.commandId));
        return h1 ^ (h2 << 1);
    }
};

/**
 * @brief Class for handling VESC status messages.
 */
class VescStatusHandler
{
public:
    /**
     * @brief Constructor for VescStatusHandler.
     * @param nh ROS NodeHandle for communication with ROS.
     */
    VescStatusHandler(ros::NodeHandle& nh);

    /**
     * @brief Callback function for grabbing status from CAN frame.
     * @param frame Pointer to the received CAN frame.
     */
    void statusGrabber(const can_msgs::Frame::ConstPtr &frame);

    /**
     * @brief Sends an update for a specific VESC ID.
     * @param vescId VESC identifier.
     */
    void sendUpdate(uint8_t vescId);

    /**
     * @brief Clears the stored motor status.
     */
    void clear();

private:
    ros::Time lastSendTime; /**< Time of the last sent message. */

    std::unordered_map<MotorStatusKey, MotorStatusValue, MyHash<MotorStatusKey>> mMotorStatus; /**< Map storing motor status. */

    ros::Subscriber mStatusGrabber; /**< ROS subscriber for status messages. */
    ros::Publisher mStatusPublisher; /**< ROS publisher for status messages. */

    ros::Timer mMotorCommandTimer; /**< Timer for motor commands. */
};

#endif //VescMotorController_h_
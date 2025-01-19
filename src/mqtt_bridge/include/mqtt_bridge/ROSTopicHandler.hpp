#ifndef ROS_TOPIC_HANDLER_H
#define ROS_TOPIC_HANDLER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "can_wrapper/Wheels.h"
#include "can_wrapper/VescStatus.h"
#include "can_wrapper/RoverControl.h"
#include "can_wrapper/RoverStatus.h"
#include "mqtt_bridge/ManipulatorMessage.h"
#define RAPIDJSON_HAS_STDSTRING 1
#include "rapidjson/document.h"

// forward declarations
namespace mqtt
{
  class async_client;
}

/**
 * @brief Class to handle ROS topics and MQTT messages.
 */
class ROSTopicHandler
{
public:
  /**
   * @brief Constructor for ROSTopicHandler.
   * @param mqttClient Shared pointer to the MQTT async client.
   * @param mqttQOS Quality of Service level for MQTT.
   */
  ROSTopicHandler(std::shared_ptr<mqtt::async_client> mqttClient, int mqttQOS);

  /**
   * @brief Publishes a Wheels message to the ROS topic.
   * @param message The Wheels message to publish.
   */
  void publishMessage_Wheels(can_wrapper::Wheels message);

  /**
   * @brief Publishes a RoverControl message to the ROS topic.
   * @param message The RoverControl message to publish.
   */
  void publishMessage_RoverControl(can_wrapper::RoverControl message);

  /**
   * @brief Publishes a ManipulatorControl message to the ROS topic.
   * @param message The ManipulatorControl message to publish.
   */
  void publishMessage_ManipulatorControl(mqtt_bridge::ManipulatorMessage message);

  /**
   * @brief Publishes a RoverStatus message to the ROS topic.
   * @param message The RoverStatus message to publish.
   */
  void publishMessage_RoverStatus(can_wrapper::RoverStatus message);

private:
  /**
   * @brief Publishes an MQTT message.
   * @param topicName The name of the MQTT topic.
   * @param message The message to publish.
   */
  void publishMqttMessage(const std::string topicName, const char *message);

  /**
   * @brief Adds a timestamp to a JSON document.
   * @param doc The JSON document.
   * @param nsec The timestamp in nanoseconds.
   */
  void addTimestampToJSON(rapidjson::Document &doc, long int nsec);

  /**
   * @brief Adds a member to a JSON document.
   * @tparam T The type of the value.
   * @param doc The JSON document.
   * @param name The name of the member.
   * @param value The value of the member.
   */
  template<typename T>
  void addMemberToJSON(rapidjson::Document &doc, std::string name, T value);

  /**
   * @brief Adds members from a map to a JSON document.
   * @tparam T The type of the values in the map.
   * @param doc The JSON document.
   * @param m The map containing the members to add.
   */
  template<typename T>
  void addMembersFromMapToJSON(rapidjson::Document &doc, const std::map<std::string, T>& m);

  /**
   * @brief Callback function for receiving VESC status messages.
   * @param receivedMsg The received VESC status message.
   */
  void callback_VescStatus(const can_wrapper::VescStatus::ConstPtr &receivedMsg);

  /**
   * @brief Timer callback to publish VESC status messages.
   * @param event The timer event.
   */
  void fire_VescStatus(const ros::TimerEvent& event);

  /**
   * @brief Publishes VESC status as an MQTT message.
   * @param msg The VESC status message to publish.
   */
  void publishMqttMessage_VescStatus(std::shared_ptr<can_wrapper::VescStatus> msg);

  /**
   * @brief Callback function for receiving ZED IMU data messages.
   * @param receivedMsg The received ZED IMU data message.
   */
  void callback_ZedImuData(const sensor_msgs::Imu::ConstPtr &receivedMsg);

  /**
   * @brief Timer callback to publish ZED IMU data messages.
   * @param event The timer event.
   */
  void fire_ZedImuData(const ros::TimerEvent& event);

  /**
   * @brief Publishes ZED IMU data as an MQTT message.
   * @param msg The ZED IMU data message to publish.
   */
  void publishMqttMessage_ZedImuData(std::shared_ptr<sensor_msgs::Imu> msg);

  std::shared_ptr<mqtt::async_client> mCli; ///< Shared pointer to the MQTT async client.
  int mQOS; ///< Quality of Service level for MQTT.

  const double mInterval_VescStatus = 0.05; ///< Interval for VESC status timer.
  ros::Subscriber mSub_VescStatus; ///< ROS subscriber for VESC status.
  ros::Timer mTimer_VescStatus; ///< ROS timer for VESC status.
  std::map<int, std::shared_ptr<can_wrapper::VescStatus>> mMsgMap_VescStatus; ///< Map of VESC status messages.

  const double mInterval_ZedImuData = 0.05; ///< Interval for ZED IMU data timer.
  ros::Subscriber mSub_ZedImuData; ///< ROS subscriber for ZED IMU data.
  ros::Timer mTimer_ZedImuData; ///< ROS timer for ZED IMU data.
  std::shared_ptr<sensor_msgs::Imu> mMsg_ZedImuData; ///< Shared pointer to the ZED IMU data message.
  bool mFirst_ZedImuData = true; ///< Flag to indicate if the first ZED IMU data message has been received.

  ros::Publisher mPub_Wheels; ///< ROS publisher for Wheels messages.
  ros::Publisher mPub_RoverControl; ///< ROS publisher for RoverControl messages.
  ros::Publisher mPub_ManipulatorControl; ///< ROS publisher for ManipulatorControl messages.
  ros::Publisher mPub_RoverStatus; ///< ROS publisher for RoverStatus messages.
};

#endif // ROS_TOPIC_HANDLER_H
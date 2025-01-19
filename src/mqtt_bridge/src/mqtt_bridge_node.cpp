#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include <sstream>
#include <mqtt/async_client.h>
#include <ros/console.h>

#define RAPIDJSON_HAS_STDSTRING 1

/**
 * @brief Exception class for JSON assertion errors.
 */
class JsonAssertException : public std::exception
{
public:
    /**
     * @brief Returns the exception message.
     * @return The exception message.
     */
    char const *what()
    {
        return "JSON assert exception";
    }
};

// Custom assert(x) for rapidjson library so that it doesn't abort the program on errors
#define RAPIDJSON_ASSERT(x) (static_cast<bool>(x) ? void(0) : throw JsonAssertException())

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/error/en.h"
#include "mqtt_bridge/ROSTopicHandler.hpp"

// TODO:
// - rosdep (for paho)?

/**
 * @brief Converts Unix timestamp in milliseconds to ROS timestamp.
 * @param msec Unix timestamp in milliseconds.
 * @return ROS timestamp.
 */
ros::Time unixMillisecondsToROSTimestamp(unsigned long int msec)
{
    ros::Time timestamp;
    timestamp.fromSec(msec / (double)1000.0);
    return timestamp;
}

/**
 * @brief Processes incoming MQTT message for wheels and publishes it to ROS.
 * @param payloadMsg The MQTT message payload.
 * @param rth Shared pointer to the ROS topic handler.
 */
void processMqttWheelsMessage(const char *payloadMsg, std::shared_ptr<ROSTopicHandler> rth)
{
    rapidjson::Document d;
    rapidjson::ParseResult ok = d.Parse(payloadMsg);

    if (!ok)
    {
        ROS_WARN_STREAM("JSON parse error: " << rapidjson::GetParseError_En(ok.Code()) << " (" << ok.Offset() << "), discarding MQTT message.");
    }
    else
    {
        try
        {
            can_wrapper::Wheels msg;

            // msg.commandId = d["commandId"].GetUint();
            // msg.frontLeft = d["frontLeft"].GetDouble();
            // msg.frontRight = d["frontRight"].GetDouble();
            // msg.rearLeft = d["rearLeft"].GetDouble();
            // msg.rearRight = d["rearRight"].GetDouble();

            // msg.header.stamp = unixMillisecondsToROSTimestamp(d["Timestamp"].GetUint64());

            rth->publishMessage_Wheels(msg);
        }
        catch (JsonAssertException e)
        {
            ROS_WARN("JSON assert exception, discarding MQTT message.");
        }
    }
}

/**
 * @brief Processes incoming MQTT message for rover control and publishes it to ROS.
 * @param payloadMsg The MQTT message payload.
 * @param rth Shared pointer to the ROS topic handler.
 */
void processMqttRoverControlMessage(const char *payloadMsg, std::shared_ptr<ROSTopicHandler> rth)
{
    rapidjson::Document d;
    rapidjson::ParseResult ok = d.Parse(payloadMsg);

    if (!ok)
    {
        ROS_WARN_STREAM("JSON parse error: " << rapidjson::GetParseError_En(ok.Code()) << " (" << ok.Offset() << "), discarding MQTT message.");
    }
    else
    {
        try
        {
            can_wrapper::RoverControl msg;

            msg.Vel = d["Vel"].GetDouble();
            msg.XAxis = d["XAxis"].GetDouble();
            msg.YAxis = d["YAxis"].GetDouble();
            msg.Mode = d["Mode"].GetUint();

            msg.header.stamp = unixMillisecondsToROSTimestamp(d["Timestamp"].GetUint64());

            rth->publishMessage_RoverControl(msg);
        }
        catch (JsonAssertException e)
        {
            ROS_WARN("JSON assert exception, discarding MQTT message.");
        }
    }
}

/**
 * @brief Processes incoming MQTT message for manipulator control and publishes it to ROS.
 * @param payloadMsg The MQTT message payload.
 * @param rth Shared pointer to the ROS topic handler.
 */
void processMqttManipulatorControlMessage(const char *payloadMsg, std::shared_ptr<ROSTopicHandler> rth)
{
    rapidjson::Document d;
    rapidjson::ParseResult ok = d.Parse(payloadMsg);

    if (!ok)
    {
        ROS_WARN_STREAM("JSON parse error: " << rapidjson::GetParseError_En(ok.Code()) << " (" << ok.Offset() << "), discarding MQTT message.");
    }
    else
    {
        try
        {
            mqtt_bridge::ManipulatorMessage msg;

            msg.Axis1 = d["Axis1"].GetDouble();
            msg.Axis2 = d["Axis2"].GetDouble();
            msg.Axis3 = d["Axis3"].GetDouble();
            msg.Axis4 = d["Axis4"].GetDouble();
            msg.Axis5 = d["Axis5"].GetDouble();
            msg.Axis6 = d["Axis6"].GetDouble();
            msg.Gripper = d["Gripper"].GetDouble();

            msg.header.stamp = unixMillisecondsToROSTimestamp(d["Timestamp"].GetUint64());

            rth->publishMessage_ManipulatorControl(msg);
        }
        catch (JsonAssertException e)
        {
            ROS_WARN("JSON assert exception, discarding MQTT message.");
        }
    }
}

/**
 * @brief Processes incoming MQTT message for rover status and publishes it to ROS.
 * @param payloadMsg The MQTT message payload.
 * @param rth Shared pointer to the ROS topic handler.
 */
void processMqttRoverStatusMessage(const char *payloadMsg, std::shared_ptr<ROSTopicHandler> rth)
{
    rapidjson::Document d;
    rapidjson::ParseResult ok = d.Parse(payloadMsg);

    if (!ok)
    {
        ROS_WARN_STREAM("JSON parse error: " << rapidjson::GetParseError_En(ok.Code()) << " (" << ok.Offset() << "), discarding MQTT message.");
    }
    else
    {
        try
        {
            can_wrapper::RoverStatus msg;

            msg.CommunicationState = d["CommunicationState"].GetInt();
            msg.PadConnected = d["PadConnected"].GetBool();
            msg.ControlMode = d["ControlMode"].GetInt();

            msg.header.stamp = unixMillisecondsToROSTimestamp(d["Timestamp"].GetUint64());

            rth->publishMessage_RoverStatus(msg);
        }
        catch (JsonAssertException e)
        {
            ROS_WARN("JSON assert exception, discarding MQTT message.");
        }
    }
}
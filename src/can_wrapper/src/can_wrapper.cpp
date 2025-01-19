#include <ros/ros.h>
#include <memory>
#include <linux/can.h>
#include <can_msgs/Frame.h>
#include <std_msgs/String.h>

#include "can_wrapper/MotorControl.hpp"
#include "can_wrapper/RosCanConstants.hpp"
#include "can_wrapper/VescStatusHandler.hpp"
#include "can_wrapper/RoverControl.h"
#include "can_wrapper/StatusMessage.hpp"
#include "can_wrapper/ManipulatorControl.hpp"
#include "can_wrapper/ProbeStatusForwarder.hpp"

#include <ros/service.h>
#include <std_srvs/SetBool.h>

/**
 * @brief Enum representing the different modes of the CAN node.
 */
enum class CanNodeMode
{
 Created, /**< Node is created. */
 Opening, /**< Node is opening. */
 Opened, /**< Node is opened. */
 Closing, /**< Node is closing. */
 Closed, /**< Node is closed. */
 Faulted /**< Node is in faulted state. */
};

/**
 * @brief Function to perform driving-related operations.
 * @param mtrCtl Reference to the MotorControl object.
 */
void doDrivingStuff(MotorControl &mtrCtl);

static std::chrono::system_clock::time_point lastSendWheels; /**< Last time the wheels command was sent. */
static std::chrono::nanoseconds diff; /**< Time difference for wheel commands. */

/**
 * @brief Main function for the CAN wrapper node.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Exit status.
 */
int main(int argc, char *argv[])
{
 ros::init(argc, argv, "can_wrapper");

 // Set logger level to WARN
 if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn))
 {
  ros::console::notifyLoggerLevelsChanged();
 }

 ros::NodeHandle n;

 // Initialize control and status handler objects
 MotorControl motorControl(n);
 VescStatusHandler mVescStatusHandler(n);
 StatusMessage mStatusMessage(n, true);
 ManipulatorControl mManipulatorCtl(n);
 ProbeStatusForwarder mProbeStatusForwarder(n);

 CanNodeMode canNodeMode = CanNodeMode::Created; /**< Initial CAN node mode. */
 ros::Rate rate(100); /**< Loop rate in Hz. */

 std_srvs::SetBool::Request req; /**< Service request object. */
 std_srvs::SetBool::Response res; /**< Service response object. */

 int iter = 0; /**< Iteration counter. */

 // Main loop
 while (ros::ok())
 {
  can_wrapper::Wheels vel; /**< Wheel velocity message. */
  switch (canNodeMode)
  {
  case CanNodeMode::Created:
   // Wait for the service to be available
   if (!ros::service::waitForService("/CAN/ros_can_integration/check_status", 15))
   {
    ROS_WARN("Service /CAN/ros_can_integration/check_status is not yet available. Retrying...");
    canNodeMode = CanNodeMode::Faulted;
    break;
   }

   // Call the service to check status
   if (!ros::service::call("/CAN/ros_can_integration/check_status", req, res))
   {
    ROS_WARN("Service /CAN/ros_can_integration/check_status call failed. Retrying...");
    canNodeMode = CanNodeMode::Faulted;
    break;
   }

   // Check the response from the service
   if (res.success != 0)
   {
    ROS_WARN("Package ros_can_integration is not yet ready... Retrying...");
    canNodeMode = CanNodeMode::Faulted;
    break;
   }
   canNodeMode = CanNodeMode::Opening;
   break;

  case CanNodeMode::Opening:
   ros::Duration(0.1).sleep();
   canNodeMode = CanNodeMode::Opened;
   break;

  case CanNodeMode::Opened:
   // Send status message periodically
   if ((iter++%100) == 9)
   {
    mStatusMessage.sendStatusMessage();
   }
   break;

  case CanNodeMode::Closing:
   ROS_INFO("CanNodeMode::Closing");
   canNodeMode = CanNodeMode::Closed;
   break;

  case CanNodeMode::Closed:
   canNodeMode = CanNodeMode::Opening;
   break;

  case CanNodeMode::Faulted:
   ros::Duration(5).sleep();
   canNodeMode = CanNodeMode::Created;
   break;

  default:
   ROS_INFO("CanNodeMode::Unknown");
   canNodeMode = CanNodeMode::Faulted;
   break;
  }
  rate.sleep();
  ros::spinOnce();
 }

 std::cout << "Can Wrapper shutting down" << std::endl;
}
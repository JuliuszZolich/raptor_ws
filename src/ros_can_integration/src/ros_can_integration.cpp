#include <ros/ros.h>
#include <memory>
#include <linux/can.h>
#include <can_msgs/Frame.h>
#include <std_msgs/Int32.h>

#include "ros_can_integration/CanSocket.hpp"

/**
 * @brief Main function for the ROS CAN integration node.
 * Initializes the ROS node, sets up publishers, subscribers, and services, and enters the main loop.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Exit status.
 */
int main(int argc, char *argv[])
{
    // Initialize the ROS node with the name "ros_can_integration"
    ros::init(argc, argv, "ros_can_integration");

    // Set the logger level to Info if possible
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    // Create a NodeHandle to interact with the ROS system
    ros::NodeHandle n;

    // Create a CanSocket object for the "can0" interface with a timeout of 1000 microseconds
    CanSocket cSocket("can0", 0, 1000);

    // Advertise a publisher for CAN raw messages on the "/CAN/RX/raw" topic
    ros::Publisher canRawPub = n.advertise<can_msgs::Frame>("/CAN/RX/raw", 1024);

    // Subscribe to the "/CAN/TX/raw" topic to handle outgoing CAN messages
    ros::Subscriber canRawSub = n.subscribe("/CAN/TX/raw", 1024, &CanSocket::handleRosCallback, &cSocket);

    // Advertise a service to check the status of the CAN socket
    ros::ServiceServer service = n.advertiseService("/CAN/ros_can_integration/check_status", &CanSocket::getErrorCodeCallback, &cSocket);

    // Advertise a publisher for the CAN socket status on the "/CAN/ros_can_integration/status" topic
    ros::Publisher canStatusPub = n.advertise<std_msgs::Int32>("/CAN/ros_can_integration/status", 16, true);

    // Variable to store the last error code
    int lastErrCode = 2;

    // Set the loop rate to 1000 Hz
    ros::Rate rate(1000);

    // Main loop
    while (ros::ok())
    {
        // Check if the error code has changed
        if (lastErrCode != cSocket.getErrorCode())
        {
            lastErrCode = cSocket.getErrorCode();
            std_msgs::Int32 msg;
            msg.data = lastErrCode;
            canStatusPub.publish(msg);
        }

        // Await and publish CAN messages
        cSocket.awaitAndPublishCanMessage(canRawPub);

        // Sleep for the remaining time to maintain the loop rate
        rate.sleep();

        // Process any incoming messages or service requests
        ros::spinOnce();
    }

    // Print a shutdown message
    std::cout << "Ros-Can integration shutting down" << std::endl;
}
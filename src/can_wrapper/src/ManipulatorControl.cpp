#include "can_wrapper/ManipulatorControl.hpp"

/**
 * @brief Constructor for ManipulatorControl.
 * Initializes the ROS publisher and subscriber for CAN messages.
 * @param nh The ROS NodeHandle.
 */
ManipulatorControl::ManipulatorControl(const ros::NodeHandle &nh):
	mNh(nh)
{
    // Advertise a publisher for raw CAN messages
	mRawCanPub = mNh.advertise<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_TX, 256);
    // Subscribe to the manipulator control topic
	mManipulatorCtlSub = mNh.subscribe(RosCanConstants::RosTopics::can_manipulator_ctl, 256, &ManipulatorControl::handleManipulatorCtl, this);
}

/**
 * @brief Callback to handle manipulator control messages.
 * Encodes the received manipulator control message into CAN frames and publishes them.
 * @param manipulatroCtlMsg The received manipulator control message.
 */
void ManipulatorControl::handleManipulatorCtl(const can_wrapper::ManipulatorControl::ConstPtr &manipulatroCtlMsg)
{
    // 7 since there are 6 axes + 1 gripper
	std::array<can_msgs::Frame, 7> sendQueue;
	auto sendQueueIter = sendQueue.begin();

    // Encode each axis and the gripper into CAN frames
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[0], RosCanConstants::VescIds::manipulator_axis_1);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[1], RosCanConstants::VescIds::manipulator_axis_2);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[2], RosCanConstants::VescIds::manipulator_axis_3);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[3], RosCanConstants::VescIds::manipulator_axis_4);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[4], RosCanConstants::VescIds::manipulator_axis_5);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[5], RosCanConstants::VescIds::manipulator_axis_6);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).gripper, RosCanConstants::VescIds::manipulator_gripper);

    // Publish each CAN frame
	for(auto iter = sendQueue.begin(); iter < sendQueue.end(); iter++)
		mRawCanPub.publish(*iter);
}

/**
 * @brief Encodes a stepper message for the CAN bus.
 * Converts the stepper data into a CAN frame using the VESC library.
 * @param stepper The stepper data to encode.
 * @param vescId The VESC ID.
 * @return The encoded CAN frame.
 */
can_msgs::Frame ManipulatorControl::encodeStepper(const can_wrapper::Stepper &stepper, const VESC_Id_t vescId)
{
	VESC_CommandFrame vesc_cf;
	VESC_ZeroMemory(&vesc_cf, sizeof(vesc_cf));
	VESC_RawFrame vesc_rf;
	VESC_ZeroMemory(&vesc_rf, sizeof(vesc_rf));

	vesc_cf.vescID = vescId;
	vesc_cf.command = stepper.commandId;
	vesc_cf.commandData = stepper.setValue;

	VESC_convertCmdToRaw(&vesc_rf, &vesc_cf);
	return VescInterop::vescToRos(vesc_rf);
}
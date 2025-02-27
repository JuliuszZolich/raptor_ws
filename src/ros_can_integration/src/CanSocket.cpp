#include "ros_can_integration/CanSocket.hpp"

/**
 * @brief Constructor for CanSocket.
 * @param interfaceName The name of the CAN interface.
 * @param awaitMessageTimeoutSec Timeout in seconds for awaiting a message.
 * @param awaitMessageTimeoutUsec Timeout in microseconds for awaiting a message.
 */
CanSocket::CanSocket(std::string interfaceName, uint32_t awaitMessageTimeoutSec, uint32_t awaitMessageTimeoutUsec)
{
    mInterfaceName = interfaceName;
    mAwaitMessageTimeout.tv_sec = awaitMessageTimeoutSec;
    mAwaitMessageTimeout.tv_usec = awaitMessageTimeoutUsec;
    mMaxIterCount = 10;
    mSocketMinimumLifetime = ros::Duration(5.0);
    mSocketCreatedTimestamp = ros::Time(0);
    createSocket();
}

/**
 * @brief Destructor for CanSocket.
 */
CanSocket::~CanSocket()
{
    if (mInitErrCode != 0)
        return;
    close(mSocket);
}

/**
 * @brief Tries to create a CAN socket.
 * @return 0 on success, negative error code on failure.
 */
int CanSocket::tryCreateSocket()
{
    if (mSocketCreatedTimestamp + mSocketMinimumLifetime > ros::Time::now())
        ((mSocketCreatedTimestamp + mSocketMinimumLifetime) - ros::Time::now()).sleep();

    mSocketCreatedTimestamp = ros::Time::now();
    mSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (mSocket < 0)
        return -1;

    ifreq ifr;
    strcpy(ifr.ifr_name, mInterfaceName.c_str());
    if (ioctl(mSocket, SIOCGIFINDEX, &ifr) < 0)
        return -2;
    setsockopt(mSocket, SOL_SOCKET, SO_RCVTIMEO, (const char *)&mAwaitMessageTimeout, sizeof mAwaitMessageTimeout);

    sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    can_err_mask_t err_mask = (CAN_ERR_MASK);

    if (setsockopt(mSocket, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0)
        return -4;

    if (bind(mSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        return -5;

    return 0;
}

/**
 * @brief Tries to handle a socket error.
 * @return 0 on success, negative error code on failure.
 */
int CanSocket::tryHandleError()
{
    if (mInitErrCode != 0)
        return createSocket();

    int error_code;
    socklen_t error_code_size = sizeof(error_code);
    int getSockErr = getsockopt(mSocket, SOL_CAN_RAW, SO_ERROR, &error_code, &error_code_size);

    if (getSockErr < 0)
    {
        ROS_ERROR("CAN: Raw socket getsockopt failed. Not sure this should ever happen. Recreating socket...");
        return createSocket();
    }

    switch (error_code)
    {
    case 0:
        if (sendErrStreak > 50)
            return createSocket();
        return 0;
    case EAGAIN:
    case ETIMEDOUT:
        return -1;
    case EPIPE:
        ROS_ERROR("CAN: Socket pipe broken. Recreating socket...");
        return createSocket();
    default:
        ROS_ERROR_STREAM("CAN: Socket error: " << error_code << ". Recreating socket...");
        return createSocket();
    }
}

/**
 * @brief Sets a filter for the CAN socket.
 * @param id The CAN ID to filter.
 * @param mask The mask to apply to the CAN ID.
 */
void CanSocket::setFilter(canid_t id, canid_t mask)
{
    if (mInitErrCode != 0)
        return;
    can_filter rfilter;
    rfilter.can_id = id;
    rfilter.can_mask = CAN_EFF_MASK & mask;
    if (setsockopt(mSocket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0)
    {
        mInitErrCode = -3;
        return;
    }
}

/**
 * @brief Sends a CAN message.
 * @param frame_id The CAN frame ID.
 * @param frame_len The length of the CAN frame.
 * @param raw The raw data of the CAN frame.
 * @return Number of bytes sent on success, negative error code on failure.
 */
int CanSocket::sendMessage(canid_t frame_id, uint8_t frame_len, uint8_t raw[CAN_MAX_DLEN])
{
    if (mInitErrCode != 0)
        return -2;

    can_frame frame;
    frame.can_id = frame_id;
    frame.can_dlc = frame_len;
    memcpy(frame.data, raw, frame_len);

    return sendMessage(frame);
}

/**
 * @brief Sends a CAN message.
 * @param frame The CAN frame to send.
 * @return Number of bytes sent on success, negative error code on failure.
 */
int CanSocket::sendMessage(const can_frame &frame)
{
    if (mInitErrCode != 0)
    {
        ROS_INFO("Can raw socket not initialized");
        sendErrStreak++;
        return -2;
    }
    ssize_t nbytes = write(mSocket, &frame, sizeof(struct can_frame));
    if (nbytes < 0)
    {
        sendErrStreak++;
        return -1;
    }
    sendErrStreak = 0;
    return nbytes;
}

/**
 * @brief Awaits a CAN message.
 * @param frame The CAN frame to receive.
 * @return Number of bytes received on success, negative error code on failure.
 */
ssize_t CanSocket::awaitMessage(can_frame &frame)
{
    if (mInitErrCode != 0)
        return -2;
    ssize_t nbytes = read(mSocket, &frame, sizeof(struct can_frame));

    if (nbytes < 0)
    {
        return tryHandleError();
    }

    /* paranoid check ... */
    if (nbytes < sizeof(struct can_frame))
    {
        ROS_WARN("CAN: Incomplete frame");
        return -1;
    }
    return nbytes;
}

/**
 * @brief Awaits and publishes a CAN message.
 * @param canRawPub The ROS publisher for CAN messages.
 * @return 0 on success, negative error code on failure.
 */
ssize_t CanSocket::awaitAndPublishCanMessage(ros::Publisher &canRawPub)
{
    can_frame frame;
    if (awaitMessage(frame) < 0)
    {
        ros::Duration(0.0005).sleep();
        return tryHandleError();
    }

    can_msgs::Frame fr;
    fr.is_rtr = (frame.can_id & CAN_RTR_FLAG) != 0;
    fr.is_error = (frame.can_id & CAN_ERR_FLAG) != 0;
    fr.is_extended = (frame.can_id & CAN_EFF_FLAG) != 0;
    fr.id = frame.can_id & (fr.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    fr.dlc = frame.can_dlc;
    memcpy(fr.data.data(), frame.data, CAN_MAX_DLEN);
    fr.header.stamp = ros::Time::now();
    fr.header.seq = mSeqCnt++;
    canRawPub.publish(fr);
    return 0;
}

/**
 * @brief Handles a ROS callback for CAN messages.
 * @param msg The received CAN message.
 */
void CanSocket::handleRosCallback(const can_msgs::Frame::ConstPtr &msg)
{
    if (mInitErrCode != 0)
        return;

    can_frame cMsg;
    cMsg.can_id = msg->id;
    cMsg.can_dlc = msg->dlc;
    memcpy(&cMsg.data, msg->data.data(), CAN_MAX_DLEN);
    for (size_t i = 0; i < mMaxIterCount; i++)
    {
        if (sendMessage(cMsg) >= 0)
            return;
        ros::Duration(0.0005).sleep();
        if (tryHandleError() < 0)
            break;
    }
    ROS_ERROR("CAN: Failed to send message, MaxIterCount exceeded. Aborting...");
}

/**
 * @brief Creates a CAN socket.
 * @return 0 on success, negative error code on failure.
 */
int CanSocket::createSocket()
{
    if (mSocket >= 0)
        close(mSocket);

    int errCode = tryCreateSocket();
    mInitErrCode = errCode;
    ROS_INFO_STREAM_COND(mInitErrCode == 0, "CAN: " << translateInitError());
    ROS_ERROR_STREAM_COND(mInitErrCode != 0, "CAN: " << translateInitError());
    return mInitErrCode;
}

/**
 * @brief Gets the error code of the CAN socket.
 * @return The error code.
 */
int CanSocket::getErrorCode()
{
    return mInitErrCode;
}

/**
 * @brief Callback to get the error code of the CAN socket.
 * @param req The service request.
 * @param res The service response.
 * @return True if the service call was successful, false otherwise.
 */
bool CanSocket::getErrorCodeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.success = getErrorCode() == 0 ? 0 : 1;
    if (res.success != 0)
        res.message = translateInitError();
    return true;
}

/**
 * @brief Translates the initialization error code to a string message.
 * @return The error message.
 */
std::string CanSocket::translateInitError()
{
    switch (mInitErrCode)
    {
    case 0:
        return "Everything's fineee. Can was properly initialized";
    case -1:
        return "Socket creation failed";
    case -2:
        return "Interface index request failed";
    case -3:
        return "Setting socket's mask failed";
    case -4:
        return "Setting socket's error mask failed";
    case -5:
        return "Binding socket failed";
    case -69:
        return "Not initialized";
    default:
        return "I dunno smth's wrong";
    }
}
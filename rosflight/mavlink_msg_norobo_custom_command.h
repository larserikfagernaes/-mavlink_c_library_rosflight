#pragma once
// MESSAGE NOROBO_CUSTOM_COMMAND PACKING

#define MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND 200

MAVPACKED(
typedef struct __mavlink_norobo_custom_command_t {
 float arm; /*<  */
}) mavlink_norobo_custom_command_t;

#define MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN 4
#define MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_MIN_LEN 4
#define MAVLINK_MSG_ID_200_LEN 4
#define MAVLINK_MSG_ID_200_MIN_LEN 4

#define MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_CRC 188
#define MAVLINK_MSG_ID_200_CRC 188



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NOROBO_CUSTOM_COMMAND { \
    200, \
    "NOROBO_CUSTOM_COMMAND", \
    1, \
    {  { "arm", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_norobo_custom_command_t, arm) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NOROBO_CUSTOM_COMMAND { \
    "NOROBO_CUSTOM_COMMAND", \
    1, \
    {  { "arm", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_norobo_custom_command_t, arm) }, \
         } \
}
#endif

/**
 * @brief Pack a norobo_custom_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param arm  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_norobo_custom_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float arm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN];
    _mav_put_float(buf, 0, arm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN);
#else
    mavlink_norobo_custom_command_t packet;
    packet.arm = arm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_MIN_LEN, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_CRC);
}

/**
 * @brief Pack a norobo_custom_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arm  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_norobo_custom_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float arm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN];
    _mav_put_float(buf, 0, arm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN);
#else
    mavlink_norobo_custom_command_t packet;
    packet.arm = arm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_MIN_LEN, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_CRC);
}

/**
 * @brief Encode a norobo_custom_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param norobo_custom_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_norobo_custom_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_norobo_custom_command_t* norobo_custom_command)
{
    return mavlink_msg_norobo_custom_command_pack(system_id, component_id, msg, norobo_custom_command->arm);
}

/**
 * @brief Encode a norobo_custom_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param norobo_custom_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_norobo_custom_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_norobo_custom_command_t* norobo_custom_command)
{
    return mavlink_msg_norobo_custom_command_pack_chan(system_id, component_id, chan, msg, norobo_custom_command->arm);
}

/**
 * @brief Send a norobo_custom_command message
 * @param chan MAVLink channel to send the message
 *
 * @param arm  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_norobo_custom_command_send(mavlink_channel_t chan, float arm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN];
    _mav_put_float(buf, 0, arm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND, buf, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_MIN_LEN, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_CRC);
#else
    mavlink_norobo_custom_command_t packet;
    packet.arm = arm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_MIN_LEN, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_CRC);
#endif
}

/**
 * @brief Send a norobo_custom_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_norobo_custom_command_send_struct(mavlink_channel_t chan, const mavlink_norobo_custom_command_t* norobo_custom_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_norobo_custom_command_send(chan, norobo_custom_command->arm);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND, (const char *)norobo_custom_command, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_MIN_LEN, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_norobo_custom_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float arm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, arm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND, buf, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_MIN_LEN, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_CRC);
#else
    mavlink_norobo_custom_command_t *packet = (mavlink_norobo_custom_command_t *)msgbuf;
    packet->arm = arm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND, (const char *)packet, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_MIN_LEN, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE NOROBO_CUSTOM_COMMAND UNPACKING


/**
 * @brief Get field arm from norobo_custom_command message
 *
 * @return  
 */
static inline float mavlink_msg_norobo_custom_command_get_arm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a norobo_custom_command message into a struct
 *
 * @param msg The message to decode
 * @param norobo_custom_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_norobo_custom_command_decode(const mavlink_message_t* msg, mavlink_norobo_custom_command_t* norobo_custom_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    norobo_custom_command->arm = mavlink_msg_norobo_custom_command_get_arm(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN;
        memset(norobo_custom_command, 0, MAVLINK_MSG_ID_NOROBO_CUSTOM_COMMAND_LEN);
    memcpy(norobo_custom_command, _MAV_PAYLOAD(msg), len);
#endif
}

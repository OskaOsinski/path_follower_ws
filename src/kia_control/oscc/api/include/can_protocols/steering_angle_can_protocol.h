/**
 * @file steering_can_protocol.h
 * @brief Steering CAN Protocol.
 *
 */


#ifndef _OSCC_STEERING_ANGLE_CAN_PROTOCOL_H_
#define _OSCC_STEERING_ANGLE_CAN_PROTOCOL_H_


#include <stdint.h>
#include "magic.h"

#define OSCC_STEERING_ANGLE_CAN_ID (0x0B8)

#define OSCC_STEERING_ANGLE_CAN_REPORT (0xB9)


/*
 * @brief Steering report message publishing frequency. [Hz]
 *
 */
#define OSCC_REPORT_STEERING_PUBLISH_FREQ_IN_HZ (50)

#pragma pack(push)
#pragma pack(1)

typedef struct
{
    uint8_t enable;
    int16_t angle;	
    int16_t max_velocity; /* Steering torque request [-1.0, 1.0] where -1.0 is
                           * max torque in the counterclockwise direction and 1
                           * is max torque in the clockwise direction.
                           * (Note: this is inverse to standard torque direction)
                           */
  
} oscc_steering_angle_command_s;

typedef struct
{
    uint8_t magic[2]; /*!< Magic number identifying CAN frame as from OSCC.
                       *   Byte 0 should be \ref OSCC_MAGIC_BYTE_0.
                       *   Byte 1 should be \ref OSCC_MAGIC_BYTE_1. */

    uint8_t enabled; /*!< Steering controls enabled state.
                      * Zero value means disabled (commands are ignored).
                      * Non-zero value means enabled (no timeouts or overrides have occured). */

    uint8_t operator_override; /*!< Driver override state.
                                * Zero value means there has been no operator override.
                                * Non-zero value means an operator has physically overridden
                                * the system. */

    uint8_t dtcs; /*!< Bitfield of DTCs present in the module. */

    uint8_t reserved[3]; /*!< Reserved. */
} oscc_steering_angle_report_s;

#pragma pack(pop)

#endif /* _OSCC_STEERING_ANGLE_CAN_PROTOCOL_H_ */

#ifndef MUXPROTOCOL_H
#define MUXPROTOCOL_H

#ifdef __KERNEL__
#include <linux/byteorder/little_endian.h>
#include <linux/string.h>
#else
#include "common_defs.h"
#endif

/**	@brief Defines the commands supported by the protocol.
 *
 * 	Adding new commands introduces a new major version
*/
typedef enum MpuMsgType
{
	msg_version=0,
	msg_nack=1,
	msg_reset=2,
	msg_rtc=3,
	msg_ina=4,			/* Sensor	*/
	msg_poweroff=5,		/* Switch off power */
	msg_valid=6,		/* Get the status of valid lines */
	msg_charge=7,
	msg_init=8,			/* Reason for start */

} MpuMsgType_t;

/**	@brief  Expand this level table for new majors.
 *
 * 	The index is the major version and the ordinate is the last
 * 	command supported as given in then protocol enumeration
 */
static const MpuMsgType_t version_levels[] = {
	msg_charge,		/* Major level 0 */
};
static const int version_majors = sizeof(version_levels);


/**	@brief 	Main message header
 *
 */
typedef struct MpuMsgHeader
{
	u16 type;
	u16 payloadLen;
} MpuMsgHeader_t;

int mpu_create_message(MpuMsgType_t type, u8 *buffer, const u8 *payload, u16 len);
MpuMsgHeader_t *mpu_message_header(u8 *buffer);
u8* mpu_get_payload(u8 *buffer);


typedef struct MpuVersionHeader
{
	u16 ver_major;
	u16 ver_minor;
	char git_info[80];
} MpuVersionHeader_t;

MpuVersionHeader_t* mpu_get_version_header(const u8 *buffer);


u32 valid_get_data(u8 *buffer);


/**	@brief Charge control message
 *
 */
typedef enum ChargeMessageType
{
	msg_chargeEnable = 0,
	msg_chargeDisable
} ChargeMessageType_t;

typedef struct ChargeMessage
{
	u16 type;
	u16 chargeMask;		/* Bit field */
} ChargeMessage_t;

ChargeMessage_t charge_get_message(u8 *buffer);
int charge_create_message(ChargeMessageType_t type, u8 *buffer, u16 mask);

/** @brief	Init message gives the reason cpu was booted
 */

typedef enum InitEvent
{
	msg_initNone=0,
	msg_initKeyPressed=1,
	msg_initPowerInserted=2,
	msg_initBatteryInserted=4,
} InitEventType_t;

typedef struct InitMessage
{
	u32 event_mask;	/* Bit vector of InitEventType_t */
} InitMessage_t;

InitMessage_t init_get_message(const u8 *buffer);
int init_create_message(u8 *buffer, u32 emask);


#endif // MUXPROTOCOL_H

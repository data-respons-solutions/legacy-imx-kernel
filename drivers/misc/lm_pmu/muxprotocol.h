#ifndef MUXPROTOCOL_H
#define MUXPROTOCOL_H

#ifdef __KERNEL__
#include <linux/byteorder/little_endian.h>
#include <linux/string.h>
#else
#include "common_defs.h"
#endif

typedef enum MpuMsgType
{
	msg_oneWire=0,
	msg_rtc,
	msg_version,
	msg_ina,
	msg_poweroff,	/* Switch off power */
	msg_valid,		/* Get the status of valid lines */
	msg_charge,
	msg_nack,
} MpuMsgType_t;

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
} MpuVersionHeader_t;

MpuVersionHeader_t* mpu_get_version_header(const u8 *buffer);


u32 valid_get_data(u8 *buffer);


/**********************************
 *
 * Charge messages
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

#endif // MUXPROTOCOL_H

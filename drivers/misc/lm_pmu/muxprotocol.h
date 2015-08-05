#ifndef MUXPROTOCOL_H
#define MUXPROTOCOL_H

#ifndef __KERNEL__
#include <stddef.h>
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
#include <string.h>
#else
#include <linux/byteorder/little_endian.h>
#include <linux/string.h>
#endif

#ifdef STM32L051C6
#define cpu_to_le16(x) (x)
#define cpu_to_le32(x) (x)
#define le32_to_cpu(x) (x)
#define le16_to_cpu(x) (x)
#endif

typedef enum MpuMsgType
{
	msg_oneWire=0,
	msg_rtc,
	msg_version,
	msg_ina,
	msg_poweroff,
} MpuMsgType_t;

/* Set this bit in the type field of the return message on error */
#define MPU_MSG_ERROR_SET 0x8000

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

#endif // MUXPROTOCOL_H

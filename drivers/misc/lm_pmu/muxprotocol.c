#include "muxprotocol.h"

int mpu_create_message(MpuMsgType_t type, u8 *buffer, const u8 *payload, u16 len)
{
	MpuMsgHeader_t *msg = (MpuMsgHeader_t*)buffer;

#ifdef STM32L051C6
    msg->type = type;
    msg->payloadLen = len;
#else
    msg->type = cpu_to_le16(type);
	msg->payloadLen = cpu_to_le16(len);
#endif

	if(len > 0 && payload != 0)
	{
		memcpy(buffer+sizeof(MpuMsgHeader_t), payload, len);
	}
	return sizeof(MpuMsgHeader_t) + len;
}

MpuMsgHeader_t *mpu_message_header(u8 *buffer)
{
	MpuMsgHeader_t *msg = (MpuMsgHeader_t*)buffer;

#ifdef STM32L051C6
    msg->type = msg->type;
    msg->payloadLen = msg->payloadLen;
#else
	msg->type = le16_to_cpu(msg->type);
	msg->payloadLen = le16_to_cpu(msg->payloadLen);
#endif
	return msg;
}

u8* mpu_get_payload(u8 *buffer)
{
	return buffer + sizeof(MpuMsgHeader_t);
}

MpuVersionHeader_t* mpu_get_version_header(const u8 *buffer)
{
	MpuVersionHeader_t *hdr = (MpuVersionHeader_t*)buffer;

#ifdef STM32L051C6
    hdr->ver_major = hdr->ver_major;
    hdr->ver_minor = hdr->ver_minor;
#else
    hdr->ver_major = le16_to_cpu(hdr->ver_major);
	hdr->ver_minor = le16_to_cpu(hdr->ver_minor);
#endif
    return hdr;
}

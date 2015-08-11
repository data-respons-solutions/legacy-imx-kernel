#include "muxprotocol.h"

int mpu_create_message(MpuMsgType_t type, u8 *buffer, const u8 *payload, u16 len)
{
	MpuMsgHeader_t *msg = (MpuMsgHeader_t*)buffer;

    msg->type = cpu_to_le16(type);
	msg->payloadLen = cpu_to_le16(len);

	if(len > 0 && payload != 0)
	{
		memcpy(buffer+sizeof(MpuMsgHeader_t), payload, len);
	}
	return sizeof(MpuMsgHeader_t) + len;
}

MpuMsgHeader_t *mpu_message_header(u8 *buffer)
{
	MpuMsgHeader_t *msg = (MpuMsgHeader_t*)buffer;

	msg->type = le16_to_cpu(msg->type);
	msg->payloadLen = le16_to_cpu(msg->payloadLen);
	return msg;
}

u8* mpu_get_payload(u8 *buffer)
{
	return buffer + sizeof(MpuMsgHeader_t);
}

MpuVersionHeader_t* mpu_get_version_header(const u8 *buffer)
{
	MpuVersionHeader_t *hdr = (MpuVersionHeader_t*)buffer;

    hdr->ver_major = le16_to_cpu(hdr->ver_major);
	hdr->ver_minor = le16_to_cpu(hdr->ver_minor);
    return hdr;
}


u32 valid_get_data(u8 *buffer)
{
	u32 *ptr = (u32*)buffer;
	return le32_to_cpu(ptr[0]);
}

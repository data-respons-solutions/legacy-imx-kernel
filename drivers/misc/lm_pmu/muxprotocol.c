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
	int n = strnlen(hdr->git_info, sizeof(hdr->git_info));
	if ( n == sizeof(hdr->git_info))
		hdr->git_info[sizeof(hdr->git_info)-1] = '\0';
    hdr->ver_major = le16_to_cpu(hdr->ver_major);
	hdr->ver_minor = le16_to_cpu(hdr->ver_minor);

    return hdr;
}


u32 valid_get_data(u8 *buffer)
{
	u32 *ptr = (u32*)buffer;
	return le32_to_cpu(ptr[0]);
}

ChargeMessage_t charge_get_message(u8 *buffer)
{
	ChargeMessage_t msg;
	const u16 *p  = (u16*)buffer;
	msg.type = le16_to_cpu(p[0]);
	msg.chargeMask = le16_to_cpu(p[1]);
	return msg;
}

int charge_create_message(ChargeMessageType_t type, u8 *buffer, u16 mask)
{
	u16 *p = (u16*)buffer;
	p[0] = cpu_to_le16((u16)type);
	p[1] = cpu_to_le16(mask);
	return 2*sizeof(u16);
}

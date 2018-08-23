#include <mcu_proto.h>

/*! \ingroup Global
 * \{
 */
u8 mpu_compute_checksum(const u8 *msg, int len)
{
	int i=0;
	u8 	cks = 0;
	for(i=0; i < len; i++)
		cks = cks + msg[i];

	/*	XOR	*/
	cks = ~cks;
	return cks;
}

int mpu_copy_message(u8 *to, const u8 *from)
{
	MpuMsgHeader_t hdr = mpu_message_header(from);
	int len = hdr.payloadLen + sizeof(MpuMsgHeader_t);
	memcpy(to, from, len);
	return len;
}

int mpu_create_message(MpuMsgType_t type, MpuErrorCode_t status, u8 *buffer,  const u8 *payload, u16 len)
{
	MpuMsgHeader_t *msg = (MpuMsgHeader_t*)buffer;
	int total_len;
    msg->type = type;
    msg->replyStatus = status;
	msg->payloadLen = cpu_to_le16(len);
	total_len = sizeof(MpuMsgHeader_t) + len;

	if(len > 0 && payload != 0)
	{
		memcpy(buffer+sizeof(MpuMsgHeader_t), payload, len);
	}
	return total_len;
}

MpuMsgHeader_t mpu_message_header(const u8 *buffer)
{
	const MpuMsgHeader_t *msg_p = (MpuMsgHeader_t*)buffer;
	MpuMsgHeader_t msg;
	msg.type = msg_p->type;
	msg.replyStatus = msg_p->replyStatus;
	msg.payloadLen = le16_to_cpu(msg_p->payloadLen);
	return msg;
}

const u8* mpu_get_payload(const u8 *buffer)
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

int mpu_create_version_message(u8 *buffer, const MpuVersionHeader_t* hdr)
{
    u16 *p = (u16*)buffer;
    p[0] = cpu_to_le16(hdr->ver_major);
    p[1] = cpu_to_le16(hdr->ver_minor);
    return 4;
}

u32 valid_get_data(u8 *buffer)
{
	u32 *ptr = (u32*)buffer;
	return le32_to_cpu(ptr[0]);
}

InitMessage_t init_get_message(const u8 *buffer)
{
	InitMessage_t im;
	u32 *ptr = (u32*)buffer;
	im.event_mask = le32_to_cpu(ptr[0]);
	return im;
}

int init_create_message(u8 *buffer, u32 emask)
{
	((u32*)buffer)[0] = cpu_to_le32(emask);
	return sizeof(u32);
}

int notify_create_message(u8 *buffer, enum NotifyId id, u32 val)
{
	u32 *p = (u32*)buffer;
	p[0] = cpu_to_le32(id);
	p[1] = cpu_to_le32(val);
	return sizeof(NotificationMsg_t);
}

NotificationMsg_t notify_get_message(const u8 *buffer)
{
	NotificationMsg_t msg;
	u32 *p = (u32*)buffer;
	msg.id = le32_to_cpu(p[0]);
	msg.value = le32_to_cpu(p[1]);
	return msg;
}

int mpu_msg_len(const MpuMsgHeader_t *hdr)
{
	unsigned len = hdr->payloadLen + sizeof(MpuMsgHeader_t);
	if (hdr->type < msg_max &&  len <= mpu_max_message_size)
		return len;
	else
		return 0;
}
/*! \brief RTC stuff
 *
 */
int rtc_create_message(RtcMsgType_t type, u8 *buffer, RtcMsg_t *msg)
{
	RtcMsg_t* msg_p = (RtcMsg_t*)(buffer + sizeof(RtcMsgHeader_t));
	RtcMsgHeader_t *msghdr = (RtcMsgHeader_t*)buffer;

	msghdr->type = cpu_to_le16(type);
	if (msg) {
		msghdr->payloadLen = cpu_to_le16(sizeof(*msg));

		msg_p->tm_sec = (msg->tm_sec);
		msg_p->tm_min = (msg->tm_min);
		msg_p->tm_hour = (msg->tm_hour);
		msg_p->tm_mday = (msg->tm_mday);
		msg_p->tm_mon = (msg->tm_mon);
		msg_p->tm_year = (msg->tm_year);
		msg_p->tm_wday = (msg->tm_wday);

		msg_p->sub_second = cpu_to_le16(msg->sub_second);
		return sizeof(RtcMsgHeader_t) + sizeof(*msg);
	}
	else {
		msghdr->payloadLen = 0;
		return sizeof(RtcMsgHeader_t);
	}
}

int rtc_create_alarm_message(RtcMsgType_t type, u8 *buffer, RtcAlarm_t *alarm)
{
	RtcAlarm_t *alarm_p = (RtcAlarm_t*)(buffer + sizeof(RtcMsgHeader_t));
	RtcMsgHeader_t *msghdr = (RtcMsgHeader_t*)buffer;

	msghdr->type = cpu_to_le16(type);
	if (alarm) {
		msghdr->payloadLen = cpu_to_le16(sizeof(RtcAlarm_t));
		alarm_p->enable = cpu_to_le16(alarm->enable);
		alarm_p->pending = cpu_to_le16(alarm->pending);
		alarm_p->tm_sec = (alarm->tm_sec);
		alarm_p->tm_min = (alarm->tm_min);
		alarm_p->tm_hour = (alarm->tm_hour);
		alarm_p->tm_mday = (alarm->tm_mday);
		return sizeof(RtcMsgHeader_t) + sizeof(RtcAlarm_t);
	}
	else {
		msghdr->payloadLen = 0;
		return sizeof(RtcMsgHeader_t);
	}
}

RtcMsgHeader_t rtc_message_header(u8 *buffer)
{
	RtcMsgHeader_t msg;
	RtcMsgHeader_t  *msg_p = (RtcMsgHeader_t*)buffer;

	msg.type = le16_to_cpu(msg_p->type);
	msg.payloadLen = le16_to_cpu(msg_p->payloadLen);
	return msg;
}

int rtc_get_payload(const u8 *buffer, RtcMsg_t *msg)
{
	RtcMsgHeader_t *hdr = (RtcMsgHeader_t*)buffer;
	RtcMsg_t* msg_p = (RtcMsg_t*)(buffer + sizeof(RtcMsgHeader_t));
	if (le16_to_cpu(hdr->payloadLen) > 0) {
		msg->tm_sec = (msg_p->tm_sec);
		msg->tm_min = (msg_p->tm_min);
		msg->tm_hour = (msg_p->tm_hour);
		msg->tm_mday = (msg_p->tm_mday);
		msg->tm_mon = (msg_p->tm_mon);
		msg->tm_year = (msg_p->tm_year);
		msg->tm_wday = (msg_p->tm_wday);

		msg->sub_second = le16_to_cpu(msg_p->sub_second);
		return 0;
	}
	return -1;
}

int rtc_alarm_get_payload(const u8 *buffer, RtcAlarm_t *msg)
{
	RtcMsgHeader_t *hdr = (RtcMsgHeader_t*)buffer;
	RtcAlarm_t* msg_p = (RtcAlarm_t*)(buffer + sizeof(RtcMsgHeader_t));
	if (le16_to_cpu(hdr->payloadLen) > 0) {
		msg->enable = (le16_to_cpu(msg_p->enable));
		msg->pending = (le16_to_cpu(msg_p->pending));
		msg->tm_sec = (msg_p->tm_sec);
		msg->tm_min = (msg_p->tm_min);
		msg->tm_hour = (msg_p->tm_hour);
		msg->tm_mday = (msg_p->tm_mday);
		return 0;
	}
	return -1;
}
/* \} */

/*
 * rtc_proto.c
 *
 *  Created on: May 19, 2015
 *  Author: scc
 */

#include "rtc_proto.h"

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

RtcMsgHeader_t rtc_message_header(u8 *buffer)
{
	RtcMsgHeader_t msg;
	RtcMsgHeader_t  *msg_p = (RtcMsgHeader_t*)buffer;
	
	msg.type = le16_to_cpu(msg_p->type);
	msg.payloadLen = le16_to_cpu(msg_p->payloadLen);
	return msg;
}

int rtc_get_payload(u8 *buffer, RtcMsg_t *msg)
{
	RtcMsgHeader_t *hdr = (RtcMsgHeader_t*)buffer;
	RtcMsg_t* msg_p = (RtcMsg_t*)(buffer + sizeof(RtcMsgHeader_t));
	if (hdr->payloadLen > 0) {
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

/*
 * ina219_proto.c
 *
 *  Created on: July 03, 2015
 *  Author: scc
 */

#include "ina219_proto.h"

int ina219_create_message(Ina219MsgType_t type, u8 *buffer, Ina219Msg_t *msg)
{
	Ina219Msg_t* msg_p = (Ina219Msg_t*)(buffer + sizeof(Ina219MsgHeader_t));
	Ina219MsgHeader_t *msghdr = (Ina219MsgHeader_t*)buffer;
	
	msghdr->type = cpu_to_le16(type);
	if (msg) {
		msghdr->payloadLen = cpu_to_le16(sizeof(*msg));

		msg_p->ina219_shunt_vol = (msg->ina219_shunt_vol);
		msg_p->ina219_bus_vol = (msg->ina219_bus_vol);
		msg_p->ina219_power = (msg->ina219_power);
		msg_p->ina219_current = (msg->ina219_current);
		
		return sizeof(Ina219MsgHeader_t) + sizeof(*msg);
	}
	else {
		msghdr->payloadLen = 0;
		return sizeof(Ina219MsgHeader_t);
	}
}

Ina219MsgHeader_t ina219_message_header(u8 *buffer)
{
	Ina219MsgHeader_t msg;
	Ina219MsgHeader_t  *msg_p = (Ina219MsgHeader_t*)buffer;
	
	msg.type = le16_to_cpu(msg_p->type);
	msg.payloadLen = le16_to_cpu(msg_p->payloadLen);
	return msg;
}

u8* ina219_get_payload(u8 *buffer)
{
	Ina219MsgHeader_t *hdr = (Ina219MsgHeader_t*)buffer;
	
	if (hdr->payloadLen > 0) {
		return buffer + sizeof(Ina219MsgHeader_t);
	}
	
	return NULL;
}

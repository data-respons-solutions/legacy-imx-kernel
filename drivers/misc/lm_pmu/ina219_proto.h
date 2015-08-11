#ifndef INA219_PROTO_H
#define INA219_PROTO_H

#include "muxprotocol.h"


typedef enum Ina219MsgType
{
	msg_ina219_show_value = 0,
} Ina219MsgType_t;

typedef struct Ina219Msg
{
	s32 bus_voltage_uV[2];
	s32 current_uA[2];
	u32 power_uW[2];
} Ina219Msg_t;

typedef struct Ina219MsgHeader
{
	u16 type;
	u16 payloadLen;
} Ina219MsgHeader_t;

int ina219_create_message(Ina219MsgType_t type, u8 *buffer, Ina219Msg_t *msg);
Ina219MsgHeader_t ina219_message_header(u8 *buffer);

u8* ina219_get_payload(u8 *buffer);

#endif // INA219_PROTO_H


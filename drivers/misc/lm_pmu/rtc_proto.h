#ifndef RTC_PROTO_H
#define RTC_PROTO_H

#include "muxprotocol.h"


typedef enum RtcMsgType
{
	msg_rtc_set_time = 0,
	msg_rtc_get_time,
} RtcMsgType_t;

typedef struct RtcMsg
{
	u8 tm_sec;	/* 0-59 */
	u8 tm_min;	/* 0-59 */
	u8 tm_hour;	/* 0-23 */
	u8 tm_mday;	/* 1-31 */
	u8 tm_mon;	/* 1-12 */
	u8 tm_year;	/* 0-99 */
	u8 tm_wday;	/* 1-7  */
	u8 padding;
	u16 sub_second;
} RtcMsg_t;

typedef struct RtcMsgHeader
{
	u16 type;
	u16 payloadLen;
} RtcMsgHeader_t;

int rtc_create_message(RtcMsgType_t type, u8 *buffer, RtcMsg_t *msg);
RtcMsgHeader_t rtc_message_header(u8 *buffer);

int rtc_get_payload(u8 *buffer, RtcMsg_t *msg);
#endif // RTC_PROTO_H


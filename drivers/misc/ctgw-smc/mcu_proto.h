/*
 * pmu_proto.h
 *
 *  Created on: Oct 22, 2017
 *      Author: hcl
 */

#ifndef MCU_PROTO_H_
#define MCU_PROTO_H_

#ifdef __KERNEL__
#include <linux/byteorder/little_endian.h>
#include <linux/string.h>
#else
#include <string.h>
#include <stdint.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef int16_t s16;
typedef uint32_t u32;
typedef int32_t s32;
#define cpu_to_le16(t) (t)
#define cpu_to_le32(t) (t)
#define le16_to_cpu(t) (t)
#define le32_to_cpu(t) (t)
void microDelay(int us);
#endif

/*! \ingroup SystemConstants
 * \{
 *
 */
#define fwVersionMajor 3  //!< Firmware Version major
#define fwVersionMinor 3  //!< Firmware Version minor
/* \} */

#ifdef __cplusplus
extern "C" {
#endif
u8 mpu_compute_checksum(const u8 *msg, int len);
int mpu_copy_message(u8 *to, const u8 *from);

static const u32 mpu_max_message_size = 32;

/*!	\brief Defines the commands supported by the protocol.
 *
 * 	Adding new commands introduces a new major version
 */
typedef enum MpuMsgType {
	msg_version = 0,
	msg_reset = 1,
	msg_gpo = 2,
	msg_defaults = 3,
	msg_grace = 4, /* NA */
	msg_poweroff = 5, /* Switch off power */
	msg_rtc = 6,
	msg_led = 7,
	msg_ignition = 8,
	msg_init = 9, /* Reason for start */
	msg_async = 10,
	msg_reboot = 11,
	msg_notify = 12, /* From version 2.1 */
	msg_debug = 13,
	msg_sensors = 14, /* From version 2.2 */
	msg_power_supply = 15, /* From version 3.0 */
	msg_set_start_options = 16, /* From version 3.1 */
	msg_max
} MpuMsgType_t;

/*! \brief Error and status codes */
typedef enum MpuErrorCode {
	mpu_status_ok = 0,
	mpu_status_crc = 1,
	mpu_status_err = 2,
	mpu_status_impl = 3
} MpuErrorCode_t;

/*! /brief 	Main message header */
typedef struct MpuMsgHeader {
	u8 type; /*!< MpuMsgType */
	u8 replyStatus; /*!< MpuErrorCode */
	u16 payloadLen; /*!< Length of payload (ex CRC) */
} MpuMsgHeader_t;

int mpu_create_message(MpuMsgType_t type, MpuErrorCode_t status, u8 *buffer,
		const u8 *payload, u16 len);
MpuMsgHeader_t mpu_message_header(const u8 *buffer);
const u8* mpu_get_payload(const u8 *buffer);

typedef struct MpuVersionHeader {
	u16 ver_major;
	u16 ver_minor;
} MpuVersionHeader_t;

MpuVersionHeader_t* mpu_get_version_header(const u8 *buffer);
int mpu_create_version_message(u8 *buffer, const MpuVersionHeader_t* hdr);

u32 valid_get_data(u8 *buffer);

/** @brief	Init message gives the reason cpu was booted
 */

typedef enum InitEvent {
	msg_init_none = 0x00,
	msg_init_ignition = 0x01,
	msg_init_gpi = 0x02,
	msg_init_acc1 = 0x04,
	msg_init_acc2 = 0x08,
	msg_init_rtc = 0x10,
	msg_init_cold = 0x20,
	msg_init_reset = 0x40,
	init_select_mask = (msg_init_gpi | msg_init_acc1 | msg_init_acc2)
} InitEventType_t;

typedef struct InitMessage {
	u16 set_mask_cmd; /* 1=set, 0=get */
	u16 event_mask; /* Bit vector of InitEventType_t */
} InitMessage_t;

InitMessage_t init_get_message(const u8 *buffer);
int init_create_message(u8 *buffer, u32 emask);
typedef enum RtcMsgType {
	msg_rtc_set_time = 0,
	msg_rtc_get_time,
	msg_rtc_set_alarm,
	msg_rtc_get_alarm,
	msg_rtc_cancel_alarm,
} RtcMsgType_t;

/* All values in BCD format */
typedef struct RtcMsg {
	u8 tm_sec; /* 0-59 */
	u8 tm_min; /* 0-59 */
	u8 tm_hour; /* 0-23 */
	u8 tm_mday; /* 1-31 */
	u8 tm_mon; /* 1-12 */
	u8 tm_year; /* 0-99 */
	u8 tm_wday; /* 1-7  */
	u8 padding;
	u16 sub_second;
} RtcMsg_t;

static const uint32_t wakeup_max_seconds = 60 * 60 * 24 * 28;
typedef struct RtcAlarm {
	u16 enable;
	u16 pending;
	u8 tm_sec; /* 0-59 */
	u8 tm_min; /* 0-59 */
	u8 tm_hour; /* 0-23 */
	u8 tm_mday; /* 1-31 */
} RtcAlarm_t;

typedef struct RtcMsgHeader {
	u16 type;
	u16 payloadLen;
} RtcMsgHeader_t;

int rtc_create_message(RtcMsgType_t type, u8 *buffer, RtcMsg_t *msg);
int rtc_create_alarm_message(RtcMsgType_t type, u8 *buffer, RtcAlarm_t *alarm);
RtcMsgHeader_t rtc_message_header(u8 *buffer);

int rtc_get_payload(const u8 *buffer, RtcMsg_t *msg);
int rtc_alarm_get_payload(const u8 *buffer, RtcAlarm_t *msg);
int mpu_msg_len(const MpuMsgHeader_t *hdr);

enum NotifyId {
	GpiIgnition = 0, GpiWakeup = 1, PowerFail = 2
};

typedef struct NotificationMsg {
	u32 id;
	u32 value;
} NotificationMsg_t;

int notify_create_message(u8 *buffer, enum NotifyId id, u32 val);
NotificationMsg_t notify_get_message(const u8 *buffer);

enum LedId {
	LedStatusGreen = 0, LedStatusRed = 1
};

typedef struct LedMsg {
	u8 led_id;
	u8 led_on;
} LedMsg_t;

enum IgnitionMessageType {
	IgnitionDelayGet = 0, IgnitionDelaySet = 1
};

enum GpoMessageType {
	GpoGet = 0, GpoSet = 1,
};

enum GraceMessageType {
	GraceGet = 0, GraceSet = 1
};

typedef struct SensorMsg {
	s32 voltage_in; /* Input main power in mV */
	s32 current_sys; /* SYS current in mA */
	s32 voltage_scap; /* Supercap voltage in mV */
	u16 power_good; /* Boolean */
	u16 spare1;
} SensorMsg_t;

typedef struct PowerSupplyMsg {
	s16 gpo1_sense1;
	s16 gpo1_sense2;
	s16 dcout_sense1;
	s16 dcout_sense2; /* CAN bus */
	u16 dcout_online1;
	u16 dcout_online2;
	s32 voltage_in; /* All runs on VSYS in */
} PowerSupplyMsg_t;
#ifdef __cplusplus
}
#endif
#endif /* MCU_PROTO_H_ */

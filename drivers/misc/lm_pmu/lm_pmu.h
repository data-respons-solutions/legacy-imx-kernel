/*
 * lm_pmu.h
 *
 *  Created on: Aug 11, 2015
 *      Author: hcl
 */

#ifndef DRIVERS_MISC_LM_PMU_LM_PMU_H_
#define DRIVERS_MISC_LM_PMU_LM_PMU_H_

#define PROTO_BUF_SIZE 1024

struct lm_pmu_private {
	const struct lm_pmu_config *config;
	u8 outgoing_buffer[PROTO_BUF_SIZE];
	u8 incoming_buffer[PROTO_BUF_SIZE];
	struct mutex	serial_lock;

	struct work_struct response_work;
	wait_queue_head_t wait;
	volatile bool acked;
	struct rtc_device *rtc;
	struct i2c_client *i2c_dev;
	InitMessage_t init_status;	/* Bit vector of InitEventType_t */
	int gpio_msg_complete;
	int gpio_irq;
	int gpio_irq_nr;
	int gpio_alert;
	int gpio_alert_irq_nr;
	MpuVersionHeader_t version;
	void *subclass_data;
	bool pmu_ready;
	irqreturn_t (*alert_cb)(void*);
};

void lm_pmu_set_subclass_data(struct lm_pmu_private* priv, void* _data);

void *lm_pmu_get_subclass_data(struct lm_pmu_private *priv);

struct lm_pmu_private *lm_pmu_init(struct i2c_client *client);

int lm_pmu_exchange(struct lm_pmu_private *priv,
		MpuMsgType_t proto,
		u8 *tx_buffer,
		int tx_len,
		u8 *rx_buffer,
		int rx_len);

void lm_pmu_poweroff(void);
int lm_pmu_deinit(struct lm_pmu_private *priv);
int lm_pmu_set_charge(struct lm_pmu_private *priv, ChargeMessageType_t t, u16 mask);
#endif /* DRIVERS_MISC_LM_PMU_LM_PMU_H_ */

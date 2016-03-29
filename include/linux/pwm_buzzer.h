/*
 * pwm_buzzer.h
 *
 *  Created on: 19. des. 2013
 *      Author: hcl
 */

#ifndef PWM_BUZZER_H_
#define PWM_BUZZER_H_

struct platform_pwm_buzzer_data {
	int pwm_id;
	unsigned int pwm_period_ns;
};


#endif /* PWM_BUZZER_H_ */

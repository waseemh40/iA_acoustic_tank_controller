/*
 * delay.h
 *
 *  Created on: Apr 7, 2017
 *      Author: waseemh
 */

#ifndef DRIVERS_DELAY_H_
#define DRIVERS_DELAY_H_

#include "pinmap.h"
#include "em_timer.h"

	//PWM timer
#define PWM_TIMER 			TIMER0
#define PWM_CLK				cmuClock_TIMER0
#define	PWM_TOP				1024
	//speed timer Timer clock=1.5 MHz Hz, Pre_scale=1, CPU clock=1.5 MHz
#define SPEED_TIMER			TIMER1
#define SPEED_TMR_CLK		cmuClock_TIMER1
#define SPEED_TIMER_IRQ		TIMER1_IRQn
#define	SPEED_TIMER_TOP		150			//for 0.1 msec....

	//control loop timer
#define SAMPLING_TIMER		TIMER2
#define SAMPLING_TMR_CLK	cmuClock_TIMER2
#define SAMPLING_TIMER_IRQ	TIMER2_IRQn
#define	SAMPLING_TIMER_TOP	60000		//25 Hz or 40msec sample time
	//delay loop timer
#define DELAY_TIMER			TIMER3
#define DELAY_TMR_CLK		cmuClock_TIMER3
#define DELAY_TIMER_IRQ		TIMER3_IRQn

#define DUTY_0				5
#define DUTY_25				256
#define DUTY_50				512
#define DUTY_75				768
#define DUTY_100			1024
#define DUTY_STEP			51		//5% step
#define DUTY_MIN			408		//40%
#define DUTY_MAX			1024	//100%
/*
 * public variables
 */
/*
 * private functions
 */
/*
 * public functions
 */
void 		pwm_init(void);
void 		pwm_generate(uint16_t duty_cycle);
void 		pwm_stop(void);

void 		timer_init(void);
uint32_t	timer_value(void);

void 		sampling_timer_init(void);

void		delay_init(void);
void 		delay_ms(uint8_t milli_sec);


#endif /* SRC_DELAY_H_ */

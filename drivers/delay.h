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
#include "em_burtc.h"
#include "em_letimer.h"

	//PWM timer
#define PWM_TIMER 			TIMER0
#define PWM_CLK				cmuClock_TIMER0
#define	PWM_TOP				1024
	//speed timer Timer clock=1.5 MHz Hz, Pre_scale=1, CPU clock=1.5 MHz (BURTC)
/*
#define SPEED_TIMER			LETIMER0
#define SPEED_TMR_CLK		cmuClock_LETIMER0
#define SPEED_TIMER_IRQ		LETIMER0_IRQn
#define	SPEED_TIMER_TOP		150			//for 0.1 msec....
*/
#define ms_sec_top_ref		299	//0.1msec ->  @ 3MHz clock
	//acoustic pulse
#define ACOUSTIC_TIMER		TIMER1
#define ACOUSTIC_TIMER_CLK	cmuClock_TIMER1
#define	ACOUSTIC_TIMER_TOP	86		//(68965.5 Hz freq)
	//Encoder loop timer
#define ENCODER_TIMER		TIMER2
#define ENCODER_TMR_CLK		cmuClock_TIMER2
#define ENCODER_TIMER_IRQ	TIMER2_IRQn
#define	ENCODER_TIMER_TOP	60000		//25 Hz or 40msec sample time
#define ENCODER_IRQ			TIMER2_IRQn
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
#define DUTY_MIN			103		//10%
#define DUTY_MAX			615		//60%
	//values for 8V
#define PID_SP_STEP			51		//5% step
#define PID_SP_MIN			102		//10%
#define PID_SP_MAX			614		//60%
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
uint32_t	timer_value(uint32_t *half_sec_value);

void 		encoder_timer_init(void);

void		delay_init(void);
void 		delay_ms(uint8_t milli_sec);

void 		acoustic_init(void);
void 		pulses_stop(void);
void 		pulses_generate(uint16_t duty_cycle);

#endif /* SRC_DELAY_H_ */

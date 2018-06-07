/*
 * gpio.c
 *
 *  Created on: Jun 1, 2018
 *      Author: waseemh
 */


#include "gpio.h"

/*
 * extern declarations
 */

extern void 	timer_irq_handler(void);
extern void 	inc_dec_irq_handler(uint8_t irq_number);
extern bool		motor_direction_flag;

#ifdef			DEBUG_MODE
extern uint16_t	uart_msg_freq;
#endif

/*
 * private shared variables
 */
static uint8_t	debounce_op_sw_count=0;

void gpio_init(void){
	CMU_ClockEnable(cmuClock_GPIO, true);

			//outputs
	GPIO_PinModeSet(PWM_PORT, PWM_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(OUT_PORT, LED, gpioModePushPull, 0);
	GPIO_PinModeSet(OUT_PORT, MOTOR_DIR, gpioModePushPull, 0);
			//inputs
	GPIO_PinModeSet(SWITCH_PORT, OP_SW, gpioModeInput, 1);
	GPIO_PinModeSet(SWITCH_PORT, DEC_SW, gpioModeInput, 1);
	GPIO_PinModeSet(SWITCH_PORT, INC_SW, gpioModeInput, 1);
			//INT setups
	GPIO_IntConfig(SWITCH_PORT,OP_SW,false,true,true);	//falling edge and enabled
	GPIO_IntConfig(SWITCH_PORT,DEC_SW,false,true,true);
	GPIO_IntConfig(SWITCH_PORT,INC_SW,false,true,true);
	GPIO_IntClear(_GPIO_IF_MASK);
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
			//clear outputs
	GPIO_PinOutClear(PWM_PORT, PWM_PIN);
	GPIO_PinOutClear(OUT_PORT, LED);
	GPIO_PinOutClear(OUT_PORT, MOTOR_DIR);
	pwm_init();
	return;
}

void GPIO_ODD_IRQHandler()
 {
	uint32_t int_mask = GPIO_IntGetEnabled();
	GPIO_IntClear(int_mask);
	if (int_mask & 1<<OP_SW){
		//debounce_op_sw_count++;

		#ifdef			 	DEBUG_MODE
		uart_msg_freq++;
		#endif
		//if(debounce_op_sw_count>=1){
			timer_irq_handler();
			motor_direction_flag=!motor_direction_flag;
			debounce_op_sw_count=0;
		//}
	}
	else if (int_mask & 1<<DEC_SW){
		inc_dec_irq_handler(DEC_SW);
		//timer_irq_handler();
	}
	else{
		;
	}
	return;
 }

void GPIO_EVEN_IRQHandler()
 {
	uint32_t int_mask = GPIO_IntGetEnabled();
	GPIO_IntClear(int_mask);
	if (int_mask & 1<<INC_SW){
		inc_dec_irq_handler(INC_SW);
	}
	return;
 }

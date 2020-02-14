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

extern void 	optical_sw_irq_handler();
extern void 	inc_dec_irq_handler(uint8_t irq_number);
extern bool		motor_direction_flag;

#ifdef			DEBUG_MODE
extern uint16_t	uart_msg_freq;
#endif

/*
 * private shared variables
 */

void gpio_init(void){
	CMU_ClockEnable(cmuClock_GPIO, true);

	GPIO_DriveModeSet(PWM_PORT,gpioDriveModeHigh );
			//outputs
	GPIO_PinModeSet(PWM_PORT, PWM_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(OUT_PORT, LED, gpioModePushPull, 0);
	GPIO_PinModeSet(OUT_PORT, LED_2, gpioModePushPull, 0);
	GPIO_PinModeSet(OUT_PORT, MOTOR_DIR, gpioModePushPull, 0);
	GPIO_PinModeSet(ACOUSTIC_PORT, ACOUSTIC_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(ACOUSTIC_PORT, ACOUSTIC_PIN2, gpioModePushPull, 0);
			//inputs
	GPIO_PinModeSet(SWITCH_PORT, OP_SW, gpioModeInput, 1);
	GPIO_PinModeSet(SWITCH_PORT, DEC_SW, gpioModeInput, 1);
	GPIO_PinModeSet(SWITCH_PORT, INC_SW, gpioModeInput, 1);
	GPIO_PinModeSet(PWM_PORT, DEC_SW_2, gpioModeInputPull, 1);
	GPIO_PinModeSet(SWITCH_PORT, INC_SW_2, gpioModeInputPull, 1);
	GPIO_PinModeSet(ENCODER_PORT, ENCODER_PIN, gpioModeInput, 1);
			//INT setups
	//GPIO_IntConfig(SWITCH_PORT,OP_SW,false,true,true);	//falling edge and enabled
	GPIO_IntConfig(SWITCH_PORT,DEC_SW,false,true,true);
	GPIO_IntConfig(SWITCH_PORT,INC_SW,false,true,true);
	GPIO_IntConfig(PWM_PORT,DEC_SW_2,false,true,true);
	GPIO_IntConfig(SWITCH_PORT,INC_SW_2,false,true,true);
	GPIO_IntClear(_GPIO_IF_MASK);
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
			//clear outputs
	GPIO_PinOutClear(PWM_PORT, PWM_PIN);
	GPIO_PinOutClear(OUT_PORT, LED);
	GPIO_PinOutClear(OUT_PORT, MOTOR_DIR);
	GPIO_PinOutClear(ACOUSTIC_PORT, ACOUSTIC_PIN);
	GPIO_PinOutClear(ACOUSTIC_PORT, ACOUSTIC_PIN2);
	//pwm_init();
	return;
}

void GPIO_ODD_IRQHandler()
 {
	uint32_t int_mask = GPIO_IntGetEnabled();
	GPIO_IntClear(int_mask);
	if (int_mask & 1<<OP_SW){

		#ifdef			 	DEBUG_MODE
		uart_msg_freq++;
		#endif
			//optical_sw_irq_handler();
	}
	else if (int_mask & 1<<DEC_SW){
		inc_dec_irq_handler(DEC_SW);
		//timer_irq_handler();
	}
	else if (int_mask & 1<<DEC_SW_2){
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
	else if (int_mask & 1<<INC_SW_2){
		inc_dec_irq_handler(INC_SW);
	}
	else{
		;
	}
	return;
 }

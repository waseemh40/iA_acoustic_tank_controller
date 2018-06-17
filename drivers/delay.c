/*
 * delay.c
 *
 *  Created on: Apr 7, 2017
 *      Author: waseemh
 */


#include "delay.h"


/*
 * private variables
 */
static 	uint32_t 	tenth_msec_counter=0;
static 	uint32_t	last_timer_value=0;
static 	uint8_t		count=0;
static 	uint32_t	sum_frequency_value=0;
/*
 * public variables
 */
uint32_t	frequency_value=0;

/*
 * IRQs
 */
void TIMER1_IRQHandler(void)
{
	uint32_t	int_mask=TIMER_IntGet(SPEED_TIMER);
	if(int_mask & TIMER_IF_OF){
		TIMER_IntClear(SPEED_TIMER, TIMER_IF_OF);      // Clear overflow flag
		tenth_msec_counter++;                     // Increment counter
	}
}
void TIMER2_IRQHandler(void)
{

	uint32_t	int_mask=TIMER_IntGet(SAMPLING_TIMER);
	if(int_mask & TIMER_IF_ICBOF0){
		TIMER_IntClear(SAMPLING_TIMER, TIMER_IEN_ICBOF0);      // Clear CBOF flag
		sum_frequency_value+=TIMER_CaptureGet(SAMPLING_TIMER,0);
		count++;
		if(count>=32){
			frequency_value=sum_frequency_value>>5;
			count=0;
			sum_frequency_value=0;
		}

	}
}
/*
 * private functions
 */
void 		pwm_enable(){
	TIMER_Enable(PWM_TIMER,true);
	return;
}

void 		pwm_disable(){
	TIMER_Enable(PWM_TIMER,false);
	return;
}
void 		timer_stop(void){
	TIMER_Enable(SPEED_TIMER,false);
	return;
}
void 		timer_start(void){

	TIMER_Enable(SPEED_TIMER,true);
	return;
}

/*
 * public functions
 */
void 		pwm_init(void){

	CMU_ClockEnable(PWM_CLK, true);

  			//init timer
	TIMER_Init_TypeDef PWMTimerInit = TIMER_INIT_DEFAULT;
  	PWMTimerInit.enable=false;
  	PWMTimerInit.prescale=timerPrescale16;
  			//set cc mode to PWM
  	TIMER_InitCC_TypeDef PWMTimerCCInit = TIMER_INITCC_DEFAULT;
  	PWMTimerCCInit.cmoa=timerOutputActionToggle ;	//should be ignored
  	PWMTimerCCInit.mode=timerCCModePWM;

	TIMER_InitCC(PWM_TIMER, 0, &PWMTimerCCInit);
	PWM_TIMER->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC3); 					//LOC 3, CC0 (PD1)
	TIMER_TopSet(PWM_TIMER, PWM_TOP);			//366.21 Hz 48000000/(2^7*1024)
	TIMER_CounterSet(PWM_TIMER, 0);
	TIMER_Init(PWM_TIMER, &PWMTimerInit);

	return;

}

void 		pwm_generate(uint16_t duty_cycle){

	if(duty_cycle>=DUTY_100){
		duty_cycle=DUTY_100;
	}
	if (duty_cycle<=DUTY_0){
		duty_cycle=DUTY_0;
	}
	TIMER_CounterSet(PWM_TIMER, 0);
	TIMER_CompareSet(PWM_TIMER, 0, (uint32_t)duty_cycle);
	TIMER_CompareBufSet(PWM_TIMER, 0, (uint32_t)duty_cycle);
	pwm_enable();
	return;
}

void 		pwm_stop(void){
	pwm_disable();
	return;
}
		/*
		 * Counter for speed measurement
		 */
void timer_irq_handler(void){
	timer_stop();
	last_timer_value=tenth_msec_counter;
	tenth_msec_counter=0;
	timer_start();

	return;
}
void 		timer_init(void){
  			//init timer
	TIMER_Init_TypeDef SPEEDTimerInit = TIMER_INIT_DEFAULT;
  	SPEEDTimerInit.enable=true;
  	SPEEDTimerInit.prescale=timerPrescale1;

	CMU_ClockEnable(SPEED_TMR_CLK, true);
	TIMER_TopSet(SPEED_TIMER, SPEED_TIMER_TOP);
	TIMER_CounterSet(SPEED_TIMER, 0);
	TIMER_IntEnable(SPEED_TIMER, TIMER_IF_OF);     // Enable Timer1 overflow interrupt
	NVIC_ClearPendingIRQ(SPEED_TIMER_IRQ);
	NVIC_EnableIRQ(SPEED_TIMER_IRQ);				// Enable TIMER1 interrupt vector in NVIC
	TIMER_Init(SPEED_TIMER, &SPEEDTimerInit);

	return;
}



uint32_t timer_value(void){

	return (uint32_t) last_timer_value;
}
	//control loop sample time timer
void 		sampling_timer_init(void){

	CMU_ClockEnable(SAMPLING_TMR_CLK, true);

  			//init timer
	TIMER_Init_TypeDef ENCODERTimerInit = TIMER_INIT_DEFAULT;
	ENCODERTimerInit.enable=true;
	ENCODERTimerInit.riseAction=timerInputActionReloadStart;
	ENCODERTimerInit.prescale=timerPrescale1;
  			//set cc mode to PWM
  	TIMER_InitCC_TypeDef ENCODERTimerCCInit = TIMER_INITCC_DEFAULT;
  	ENCODERTimerCCInit.edge=timerEdgeRising;
  	ENCODERTimerCCInit.mode=timerCCModeCapture;
  	ENCODERTimerCCInit.filter= true;

	TIMER_InitCC(SAMPLING_TIMER, 0, &ENCODERTimerCCInit);
	SAMPLING_TIMER->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC1); 					//LOC 1, CC0 (PA12)
	TIMER_CounterSet(SAMPLING_TIMER, 0);
	TIMER_IntEnable(SAMPLING_TIMER, TIMER_IEN_ICBOF0);     // Enable Timer2 ICBOF
	NVIC_ClearPendingIRQ(SAMPLING_IRQ);
	NVIC_EnableIRQ(SAMPLING_IRQ);				// Enable TIMER2 interrupt vector in NVIC
	TIMER_Init(SAMPLING_TIMER, &ENCODERTimerInit);

	return;
}
		//delay timer
void delay_init(void){
  	TIMER_Init_TypeDef timerDELAYInit = TIMER_INIT_DEFAULT;
  	timerDELAYInit.enable=false;
  	timerDELAYInit.prescale=timerPrescale4;		//37.5 cc= 0.1msec and 375 cc= 1msec
	CMU_ClockEnable(DELAY_TMR_CLK, true);
	TIMER_TopSet(DELAY_TIMER, 0);
	TIMER_CounterSet(DELAY_TIMER, 0);
	TIMER_IntEnable(DELAY_TIMER, TIMER_IF_OF);
	TIMER_Init(DELAY_TIMER, &timerDELAYInit);
	return;

}
void delay_enable(){
	TIMER_Enable(DELAY_TIMER,true);
	return;
}

void delay_disable(){
	TIMER_Enable(DELAY_TIMER,false);
	return;
}

void delay_ms(uint8_t milli_sec){
	uint32_t 	converted_top	=0;
	if(milli_sec>=100){milli_sec=100;}
	if(milli_sec<=1){milli_sec=1;}
	converted_top=milli_sec*375;
	TIMER_TopSet(DELAY_TIMER, converted_top);
	TIMER_TopBufSet(DELAY_TIMER, converted_top);
	TIMER_CounterSet(DELAY_TIMER, 0);
	delay_enable();
	while(!TIMER_IntGetEnabled(DELAY_TIMER));
	TIMER_IntClear(DELAY_TIMER,TIMER_IF_OF);
	delay_disable();
	return;
}

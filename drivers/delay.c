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
static 	uint8_t		count=0;
static 	uint32_t	sum_rpm_scaled=0;
static 	uint32_t 	half_sec_counter=0;

/*
 * public variables
 */
uint32_t	rpm_scaled=0;
char 		rs232_buf[64];
bool		timer_flag=false;

/*
 * IRQs
 */
void LETIMER0_IRQHandler(void)
{
	uint32_t	int_mask=LETIMER_IntGet(LETIMER_IF_UF);
	if(int_mask & LETIMER_IF_UF){
		LETIMER_IntClear(LETIMER0,int_mask);      // Clear overflow flag
		tenth_msec_counter++;                     // Increment counter
		if (tenth_msec_counter>=1000){
			timer_flag=true;
			tenth_msec_counter=0;
		}
	}
}
void TIMER2_IRQHandler(void)
{

	uint32_t	int_mask=TIMER_IntGet(ENCODER_TIMER);
	if(int_mask & TIMER_IF_ICBOF0){
		sum_rpm_scaled+=TIMER_CaptureGet(ENCODER_TIMER,0);
		count++;
		if(count>=64){
			rpm_scaled=sum_rpm_scaled>>6;
			count=0;
			sum_rpm_scaled=0;
			encoder_stop();
		}
		TIMER_IntClear(ENCODER_TIMER, TIMER_IEN_ICBOF0);      // Clear CBOF flag
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
void 		acoustic_enable(){
	TIMER_Enable(ACOUSTIC_TIMER,true);
	return;
}

void 		acoustic_disable(){
	TIMER_Enable(ACOUSTIC_TIMER,false);
	return;
}
void 		timer_stop(void){
	LETIMER_Enable(LETIMER0,false);
	//tenth_msec_counter=0;
	half_sec_counter=0;
	return;
}
void 		timer_start(void){
	LETIMER_Enable(LETIMER0,true);
	return;
}

void 		encoder_stop(void){
  	TIMER_InitCC_TypeDef ENCODERTimerCCInit = TIMER_INITCC_DEFAULT;
  	ENCODERTimerCCInit.edge=timerEdgeRising;
  	ENCODERTimerCCInit.mode=timerCCModeOff;//timerCCModeCapture
  	ENCODERTimerCCInit.filter= true;
	TIMER_InitCC(ENCODER_TIMER, 0, &ENCODERTimerCCInit);
	TIMER_CounterSet(ENCODER_TIMER, 0);
	TIMER_Enable(ENCODER_TIMER,false);
	return;
}
void 		encoder_start(void){
	sum_rpm_scaled=0;
	count=0;
  	TIMER_InitCC_TypeDef ENCODERTimerCCInit = TIMER_INITCC_DEFAULT;
  	ENCODERTimerCCInit.edge=timerEdgeRising;
  	ENCODERTimerCCInit.mode=timerCCModeCapture;//timerCCModeCapture
  	ENCODERTimerCCInit.filter= true;
	TIMER_InitCC(ENCODER_TIMER, 0, &ENCODERTimerCCInit);
	TIMER_CounterSet(ENCODER_TIMER, 0);
	TIMER_Enable(ENCODER_TIMER,true);
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
  	PWMTimerInit.prescale=timerPrescale1;
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
	TIMER_CompareSet(PWM_TIMER, 0, (uint32_t)43);
	TIMER_CompareBufSet(PWM_TIMER, 0, (uint32_t)43);
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
//void timer_irq_handler(void){
//	//timer_stop();
//	last_timer_value=tenth_msec_counter;
//	//tenth_msec_counter=0;
//	//timer_start();
//
//	return;
//}
void 		timer_init(void){
  			//init timer
		//Setup and initialize BURTC
	//Setup and initialize LETIMER
	LETIMER_Init_TypeDef	letimer_init=LETIMER_INIT_DEFAULT;
	letimer_init.enable=false;
	letimer_init.comp0Top=true;
	CMU_ClockEnable(cmuClock_LETIMER0, true);
	LETIMER_Reset(LETIMER0);
	LETIMER_CompareSet(LETIMER0,0,ms_sec_top_ref);
	LETIMER_Init(LETIMER0,&letimer_init);
	LETIMER_IntEnable(LETIMER0,LETIMER_IF_UF);
	NVIC_ClearPendingIRQ(LETIMER0_IRQn);
	NVIC_EnableIRQ(LETIMER0_IRQn);
	delay_ms(100);
	timer_start();
	return;
}



uint32_t timer_value(uint32_t *half_sec_value){

	*half_sec_value=half_sec_counter;
	return (uint32_t) tenth_msec_counter;

}
	//control loop sample time timer
void 		encoder_timer_init(void){

	CMU_ClockEnable(ENCODER_TMR_CLK, true);

  			//init timer
	TIMER_Init_TypeDef ENCODERTimerInit = TIMER_INIT_DEFAULT;
	ENCODERTimerInit.enable=false;
	ENCODERTimerInit.riseAction=timerInputActionReloadStart;
	ENCODERTimerInit.prescale=timerPrescale4;
  			//set cc mode to PWM
  	TIMER_InitCC_TypeDef ENCODERTimerCCInit = TIMER_INITCC_DEFAULT;
  	ENCODERTimerCCInit.edge=timerEdgeRising;
  	ENCODERTimerCCInit.mode=timerCCModeOff;//timerCCModeCapture
  	ENCODERTimerCCInit.filter= true;

	TIMER_InitCC(ENCODER_TIMER, 0, &ENCODERTimerCCInit);
	ENCODER_TIMER->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC1); 					//LOC 1, CC0 (PA12)
	TIMER_CounterSet(ENCODER_TIMER, 0);
	TIMER_IntEnable(ENCODER_TIMER, TIMER_IEN_ICBOF0);     // Enable Timer2 ICBOF
	NVIC_ClearPendingIRQ(ENCODER_IRQ);
	NVIC_EnableIRQ(ENCODER_IRQ);				// Enable TIMER2 interrupt vector in NVIC
	TIMER_Init(ENCODER_TIMER, &ENCODERTimerInit);

	return;
}

uint32_t 	encoder_value(void){
	return (rpm_scaled);
}
		//delay timer
void delay_init(void){
  	TIMER_Init_TypeDef timerDELAYInit = TIMER_INIT_DEFAULT;
  	timerDELAYInit.enable=false;
  	timerDELAYInit.prescale=timerPrescale16;		//37.5 cc= 0.1msec and 375 cc= 1msec
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
void 		acoustic_init(void){

	CMU_ClockEnable(ACOUSTIC_TIMER_CLK, true);

  			//init timer
	TIMER_Init_TypeDef PWMTimerInit = TIMER_INIT_DEFAULT;
  	PWMTimerInit.enable=false;
  	PWMTimerInit.prescale=timerPrescale1;
  			//set cc mode to PWM
  	TIMER_InitCC_TypeDef PWMTimerCCInit = TIMER_INITCC_DEFAULT;
  	PWMTimerCCInit.cmoa=timerOutputActionToggle ;	//should be ignored
  	PWMTimerCCInit.mode=timerCCModePWM;

	TIMER_InitCC(ACOUSTIC_TIMER, 0, &PWMTimerCCInit);
	ACOUSTIC_TIMER->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC4); 					//LOC 4, CC0 (PD6)
	TIMER_TopSet(ACOUSTIC_TIMER, ACOUSTIC_TIMER_TOP);			//366.21 Hz 48000000/(2^7*1024)
	TIMER_CounterSet(ACOUSTIC_TIMER, 0);
	TIMER_Init(ACOUSTIC_TIMER, &PWMTimerInit);

	return;

}

void 		pulses_generate(uint16_t duty_cycle){

	if(duty_cycle>=DUTY_100){
		duty_cycle=DUTY_100;
	}
	if (duty_cycle<=DUTY_0){
		duty_cycle=DUTY_0;
	}
	TIMER_CounterSet(ACOUSTIC_TIMER, 0);
	TIMER_CompareSet(ACOUSTIC_TIMER, 0, (uint32_t)43);
	TIMER_CompareBufSet(ACOUSTIC_TIMER, 0, (uint32_t)43);
	acoustic_enable();
	return;
}

void 		pulses_stop(void){
	acoustic_disable();
	return;
}

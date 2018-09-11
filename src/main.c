#include "../drivers/delay.h"
#include "../drivers/gpio.h"
#include "../drivers/rs232.h"
#include "../drivers/pid_controller.h"
#include <string.h>
#include "em_rmu.h"

//extern void 	timer_irq_handler(void);
//extern uint32_t	rpm_scaled;		//rpm=90000/rpm_sclaed (does not include gear ratio)

/*
 * private variables
 */
static uint16_t 	pid_sp=PID_SP_MIN;
static int			control_out=0;
static char 		rs232_buf[512];
static uint32_t 	converted_rpm_to_fb=0;
static uint16_t 	mod_factor=2;


#define N_PULSES		1

void delay_multiple(int factor,int delay);
/*
 * public variables
 */
bool 				motor_direction_flag=true;
bool 				start_flag=false;

#ifdef			 	DEBUG_MODE
uint16_t			uart_msg_freq=0;
#endif

/*
 * shared extern functions
 */
 void inc_dec_irq_handler(uint8_t irq_number){
	 start_flag=true;
	if (irq_number==DEC_SW){
		pid_sp-=PID_SP_STEP;
		GPIO_PinOutClear(OUT_PORT, LED);
	}
	else if(irq_number==INC_SW) {
		pid_sp+=PID_SP_STEP;
		GPIO_PinOutSet(OUT_PORT, LED);
	}
	else {
		;
	}
	if(pid_sp>=PID_SP_MAX){
		pid_sp=PID_SP_MAX;		//80% is max, 5% step size
	}
	if(pid_sp<=PID_SP_MIN){
		pid_sp=PID_SP_MIN;		//10% is min, 5% step size
	}
	pwm_generate((uint16_t)pid_sp);
	if(pid_sp<410)
	{
		if (N_PULSES==1)
		{
			mod_factor=1100;
		}
		else{
			mod_factor=550;
		}
 	 }
 	 else
		if (N_PULSES==1)
		{
			mod_factor=550;
		}
		else{
			mod_factor=200;
		}

	return;
}
 void 	optical_sw_irq_handler(){
		motor_direction_flag=!motor_direction_flag;

 }


 void sampler_function_hanlder(void){
	 static uint32_t current_timer_value=0;
	 static uint32_t last_timer_value=0;
	 static	uint32_t half_sec_value=0;
	 static	uint32_t pulses_generated=0;
	 static bool	 flag_start_pulses=false;
	 	 	 uint32_t read_switch_input=0;
	 	 	 int 	 loop_var=0;


	 /*
	  * simulate speed checks
	  */
	 #ifdef			 	DEBUG_MODE
	 if(uart_msg_freq>=15){
		 duty_cycle_sp+=(2*DUTY_STEP);
		if(duty_cycle_sp>=DUTY_MAX){
			duty_cycle_sp=DUTY_MAX;		//80% is max, 5% step size
		}
		if(duty_cycle_sp<=DUTY_MIN){
			duty_cycle_sp=DUTY_MIN;		//10% is min, 5% step size
		}
		pwm_generate(duty_cycle_sp);
		uart_msg_freq=0;
	 }
	 #endif
	 /*
	  *
	  */
//	 	 	 //feedback signal and rmp related
//	 converted_rpm_to_fb=(uint16_t)((165-rpm_scaled)*1.1);
//	 if(converted_rpm_to_fb>100){
//		 converted_rpm_to_fb=95;
//	 }
//	 if(converted_rpm_to_fb<0){
//		 converted_rpm_to_fb=0;
//	 }
	 	 	 //timer related
	 current_timer_value=timer_value(&half_sec_value);
		 	 //acoustic pulses
	 if(half_sec_value%mod_factor<=3 && flag_start_pulses==true) //%100=0 -> 1 sec and %200=0 -> 2 secs
	 {
		 for (loop_var=0;loop_var<N_PULSES;loop_var++){
			 pulses_generated++;

			 encoder_start();
			 pulses_generate(10);
			 delay_ms(100);
			 delay_ms(50);
			 pulses_stop();
			 encoder_stop();
			 read_switch_input=0;
			 read_switch_input=encoder_value();

			 sprintf(rs232_buf,"%d .RPM=%d\tHalf_sec_count=%ld\tSet_Point=%d\tPulse_number=%d\t\n",loop_var,(180000/read_switch_input),(half_sec_value/50),pid_sp,pulses_generated);
			 rs232_transmit_string(rs232_buf,strlen(rs232_buf));

			 delay_multiple(7,100);

		 }


		 flag_start_pulses=false;

	 }
	 	 	 //OP switch input
	 read_switch_input=1;
	 read_switch_input=GPIO_PinInGet(SWITCH_PORT,OP_SW);
	 if(read_switch_input==0){
		 	 //motor direction and de-bounce
		 motor_direction_flag=!motor_direction_flag;
		 if(motor_direction_flag){
			 GPIO_PinOutSet(OUT_PORT, MOTOR_DIR);
		 }
		 else {
			 GPIO_PinOutClear(OUT_PORT, MOTOR_DIR);
		 }
		#ifdef			 	DEBUG_MODE
		uart_msg_freq++;
		#endif
		current_timer_value=timer_value(&half_sec_value);
		timer_stop();
		timer_start();
		delay_multiple(10,100);
			//timer related
		sprintf(rs232_buf,"\t\t\t\tGlobal_Timer=%d\tTrip_Time=%d\tDirection=%d\n",current_timer_value,(current_timer_value-last_timer_value),motor_direction_flag);
		rs232_transmit_string(rs232_buf,strlen(rs232_buf));
		last_timer_value=current_timer_value;
		flag_start_pulses=true;
	 }
//	 control_out=pid_control(pid_sp,converted_rpm_to_fb);
	 delay_ms(40);
 }
int main(void)
{
	 /*
	  ********************* Chip initialization*************
	  */
			CHIP_Init();
			CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
			CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
			CMU_ClockSelectSet (cmuClock_HF, cmuSelect_HFXO);
			CMU_ClockDivSet (cmuClock_CORE,cmuClkDiv_8);
			CMU_ClockDivSet (cmuClock_HFPER ,cmuClkDiv_8);
			CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_HFCLKLE);

	 /*
	  *******************************************************
	  */
			gpio_init();
			pwm_init();
			rs232_init();
			rs232_enable();
			delay_init();
			RMU_ResetControl(rmuResetBU, rmuResetModeClear);
			timer_init();
			encoder_timer_init();
			encoder_stop();
			pid_change_gains(2,9);
			acoustic_init();
			//delay_ms(10);
			 sprintf(rs232_buf,"\t\t***Debug mode: Init set point=\t%d***\n",pid_sp);
			 rs232_transmit_string(rs232_buf,strlen(rs232_buf));

			GPIO_PinOutSet(OUT_PORT, 2);
			GPIO_PinOutSet(OUT_PORT, 3);
			while(!start_flag);
			//timer_start();

  while (1) {
	  sampler_function_hanlder();
	  }

}
void delay_multiple(int factor,int delay){
	int loop_var=0;

	for (loop_var=0;loop_var<factor;loop_var++){
		delay_ms(delay);
	}
}

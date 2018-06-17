#include "../drivers/delay.h"
#include "../drivers/gpio.h"
#include "../drivers/rs232.h"
#include "../drivers/pid_controller.h"
#include <string.h>

extern void 	timer_irq_handler(void);
extern uint32_t	frequency_value;

/*
 * private variables
 */
static uint16_t 	pid_sp=50;
static int			control_out=0;
static char 		rs232_buf[128];
static uint32_t 	converted_frequency_value=0;

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
	//pwm_generate(pid_sp);
	return;
}

 void sampler_function_hanlder(void){
	 static uint32_t last_timer_value=0;
	 static uint32_t current_timer_value=0;
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
	 converted_frequency_value=(uint16_t)((370-frequency_value)/2.5);
	 if(converted_frequency_value>100){
		 converted_frequency_value=95;
	 }
	 if(converted_frequency_value<0){
		 converted_frequency_value=0;
	 }
	 current_timer_value=timer_value();
	 if(last_timer_value!=current_timer_value){
		 sprintf(rs232_buf,"freq=%d\tfb_value=%ld\tset_point=\t%d\tcontrol_val=\t%d\t\tmilli_second_time=\t%ld\tdirection=%d\n",frequency_value,converted_frequency_value,pid_sp,control_out,current_timer_value,(uint8_t)motor_direction_flag);
		 rs232_transmit_string(rs232_buf,strlen(rs232_buf));
	 }
	 last_timer_value=current_timer_value;
	 current_timer_value=1;
	 current_timer_value=GPIO_PinInGet(SWITCH_PORT,OP_SW);
	 if(current_timer_value==0){
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
		//if(debounce_op_sw_count>=1){
		timer_irq_handler();
		delay_ms(100);
		delay_ms(100);
		delay_ms(100);
		delay_ms(100);
		delay_ms(100);
		delay_ms(100);
		delay_ms(100);
		delay_ms(100);
		delay_ms(100);
		delay_ms(100);
	 }
	 control_out=pid_control(pid_sp,converted_frequency_value);
//	 sprintf(rs232_buf,"\t\t\tfreq=%d\tfb_value=%ld\tset_point=\t%d\tcontrol_val=\t%d\n",frequency_value,converted_frequency_value,pid_sp,control_out);
//	 rs232_transmit_string(rs232_buf,strlen(rs232_buf));

	// if(control_out>0){
		 pwm_generate((uint16_t)control_out);
	 //}
	 //else{
	//	 control_out=-1*control_out;
	//	 pwm_generate((uint16_t)control_out);
	 //}
	 //pwm_generate(162);

	 delay_ms(50);

 }
int main(void)
{
	 /*
	  ********************* Chip initialization*************
	  */
			CHIP_Init();
			CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
			CMU_ClockSelectSet (cmuClock_HF, cmuSelect_HFXO);
			CMU_ClockDivSet (cmuClock_CORE,cmuClkDiv_32);
			CMU_ClockDivSet (cmuClock_HFPER ,cmuClkDiv_32);
	 /*
	  *******************************************************
	  */
			gpio_init();
			pwm_init();
			rs232_init();
			rs232_enable();
			timer_init();
			sampling_timer_init();
			delay_init();
			pid_change_gains(3,20);
			//delay_ms(10);
			 sprintf(rs232_buf,"\t\t***Debug mode: Init set point=\t%d***\n",pid_sp);
			 rs232_transmit_string(rs232_buf,strlen(rs232_buf));

			GPIO_PinOutSet(OUT_PORT, 2);
			GPIO_PinOutSet(OUT_PORT, 3);
			while(!start_flag);
  while (1) {
	  sampler_function_hanlder();
	  }

}

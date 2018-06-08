#include "../drivers/delay.h"
#include "../drivers/gpio.h"
#include "../drivers/rs232.h"
//#include "../drivers/segmentlcd.h"
#include <string.h>

extern void 	timer_irq_handler(void);
/*
 * private variables
 */
static uint16_t 	duty_cycle_sp=DUTY_MIN;
static char 		rs232_buf[128];

	//avergae timer calcs
/*static uint32_t		timer_avg_value=0;
static uint32_t		timer_value_sum=0;
static uint8_t		timer_avg_samples=32;
*/
/*
 * public variables
 */
bool 				motor_direction_flag=false;

#ifdef			 	DEBUG_MODE
uint16_t			uart_msg_freq=0;
#endif

/*
 * shared extern functions
 */
 void inc_dec_irq_handler(uint8_t irq_number){
	if (irq_number==DEC_SW){
		duty_cycle_sp-=DUTY_STEP;
		GPIO_PinOutClear(OUT_PORT, LED);
	}
	else if(irq_number==INC_SW) {
		duty_cycle_sp+=DUTY_STEP;
		GPIO_PinOutSet(OUT_PORT, LED);
	}
	else {
		;
	}
	if(duty_cycle_sp>=DUTY_MAX){
		duty_cycle_sp=DUTY_MAX;		//80% is max, 5% step size
	}
	if(duty_cycle_sp<=DUTY_MIN){
		duty_cycle_sp=DUTY_MIN;		//10% is min, 5% step size
	}
	pwm_generate(duty_cycle_sp);
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
	 current_timer_value=timer_value();
	 if(last_timer_value!=current_timer_value){
		 sprintf(rs232_buf,"Set point=\t%d\tmilli_second time=\t%ld\tDirection=%d\n",duty_cycle_sp,current_timer_value,(uint8_t)motor_direction_flag);
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
		//sprintf(rs232_buf,"OP SW is low\n");
		//rs232_transmit_string(rs232_buf,strlen(rs232_buf));
	 }

	 delay_ms(25);

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
			//sampling_timer_init();
			delay_init();
			//delay_ms(10);
			 sprintf(rs232_buf,"\t\t***Debug mode: Init set point=\t%d***\n",duty_cycle_sp);
			 rs232_transmit_string(rs232_buf,strlen(rs232_buf));

			GPIO_PinOutSet(OUT_PORT, 2);
			GPIO_PinOutSet(OUT_PORT, 3);

				//GP variables
			int 		outer_loop_var=0;
			int 		inner_loop_var=0;

  while (1) {
	  sampler_function_hanlder();
	  }

}

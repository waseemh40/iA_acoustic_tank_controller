/*
 * pid_controller.c
 *
 *  Created on: Jun 17, 2018
 *      Author: waseemh
 */


#include "pid_controller.h"

static uint8_t 	gain_kp=1;
static uint8_t	gain_ki=1;		//one-tenth of actual value

int 	pid_control(uint16_t set_point,uint16_t feedback){
		static		int			p_error=0;
		static 		int			i_error=0;
		static 		int			control_out=0;
		static 		int			shifted_out=0;
		static 		int			dt=1/50;


			//update errors
		p_error=set_point-feedback;
		i_error+=p_error;

			//calculate control out
		control_out+= (gain_kp*p_error) + (gain_ki*dt*i_error);

			//anti-wind-up control

		if(control_out>=1024){
			control_out=1024;
		}
		if(control_out<=-1024){
			control_out=-1024;
		}
		shifted_out=512+(control_out/2);
		if(shifted_out<100){
			shifted_out=102;
		}
		return shifted_out;

}
void 	pid_change_gains(uint8_t kp,uint8_t ki){
	gain_kp=kp;
	gain_ki=ki;
	return;
}

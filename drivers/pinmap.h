/*
 * pinMap.h
 *
 *  Created on: May 31, 2018
 *      Author: waseemh
 */

#ifndef DRIVERS_PINMAP_H_
#define DRIVERS_PINMAP_H_

#include "em_device.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_system.h"
#include "em_chip.h"
#include "em_core.h"
#include "em_rtc.h"
#include "em_lcd.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

		/*
		 * mapping rule:
		 * gpioPort_n.pinNumber_x
		 * e.g. SPI_port.MOSI
		 */
//#define  	DEBUG_MODE			true
		/*
		 * Oscillators and Clocks
		 */
#define 	LFXCO_FREQ			32768
		/*
		 * Pins Definition
		 */
	//ports
#define 	PWM_PORT			gpioPortD
#define 	SWITCH_PORT			gpioPortB
#define 	OUT_PORT			gpioPortE
#define 	RS232_PORT			gpioPortC
#define 	ENCODER_PORT		gpioPortA
#define 	ACOUSTIC_PORT		gpioPortD

	//pins
#define 	PWM_PIN				1			//PD1 <->	To breadboard
//#define 	LT_SW_FRONT			11			//PB11 <->	Not used
//#define 	LT_SW_REAR			12			//PB12 <->	Not used
#define 	OP_SW				11			//PB11 <->	To breadboard
//#define		SATRT_SW			12			//PB12 <-> 	To breadboard (Maybe Not used)
#define		DEC_SW				9			//PB9 <-> 	On board
#define		INC_SW				10			//PB10 <-> 	On board

#define		LED					3			//PE3 <-> 	On  board
#define		MOTOR_DIR			2			//PE2 <-> 	To breadboard

#define 	RS232_TX			0			//PC0 <-> To breadboard
#define 	RS232_RX			1			//PC1 <-> To breadborad

#define 	ENCODER_PIN			12			//PA12 <-> To Encoder

#define 	ACOUSTIC_PIN		6			//PD6 (CC0) <->	To breadboard

#endif /* DRIVERS_PINMAP_H_ */

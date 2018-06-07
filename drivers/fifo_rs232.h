/*
 * fifo_rs232.h
 *
 *  Created on: Jun 4, 2018
 *      Author: waseemh
 */

#ifndef DRIVERS_FIFO_RS232_H_
#define DRIVERS_FIFO_RS232_H_


#include <stdlib.h>
#include "pinmap.h"


#define FIFO_TX_RS232_SIZE		512
#define FIFO_RX_RS232_SIZE		16

typedef enum{
	fifo_rx_data=0,
	fifo_tx_data
}fifo_rs232_type_t;

void 		fifo_rs232_init(void);
bool 		fifo_rs232_is_empty(fifo_rs232_type_t fifo_type);
bool 		fifo_rs232_is_full(fifo_rs232_type_t fifo_type);
char 		fifo_rs232_remove(fifo_rs232_type_t fifo_type);
void 		fifo_rs232_add(fifo_rs232_type_t fifo_type,char data);



#endif /* DRIVERS_FIFO_RS232_H_ */

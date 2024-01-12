/*!
    \file  I2C_IE.h
    \brief The header file of I2C0 and I2C1 interrupt
*/

/*
    Copyright (C) 2017 GigaDevice

    2014-12-26, V1.0.0, platform GD32F1x0(x=3,5)
    2016-01-15, V2.0.0, platform GD32F1x0(x=3,5,7,9)
    2016-04-30, V3.0.0, firmware update for GD32F1x0(x=3,5,7,9)
    2017-06-19, V3.1.0, firmware update for GD32F1x0(x=3,5,7,9)
*/

#ifndef I2C_IE_H
#define I2C_IE_H

#include "gd32f3x0.h"
//IIC从机地址
#define I2C0_SLAVE_ADDRESS7 0x30 //从机地址  0x18向左移动一位 0x30 地址只取高7位，第8位是读写位
#define I2C1_SLAVE_ADDRESS7 0x11

extern  volatile uint8_t*       i2c_txbuffer;
extern  volatile uint8_t*       i2c_rxbuffer;
extern  volatile uint16_t       I2C_nBytes;

/* function declarations */
/* handle I2C0 event interrupt request */
void I2C0_EventIRQ_Handler(void);
/* handle I2C0 error interrupt request */
void I2C0_ErrorIRQ_Handler(void);
/* handle I2C1 event interrupt request */
void I2C1_EventIRQ_Handler(void);
/* handle I2C1 error interrupt request */
void I2C1_ErrorIRQ_Handler(void);

#endif /* I2C_IE_H */


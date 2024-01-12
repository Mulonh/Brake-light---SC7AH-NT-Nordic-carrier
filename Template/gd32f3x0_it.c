/*!
    \file    gd32f3x0_it.c
    \brief   interrupt service routines

    \version 2017-06-06, V1.0.0, firmware for GD32F3x0
    \version 2019-06-01, V2.0.0, firmware for GD32F3x0
    \version 2020-09-30, V2.1.0, firmware for GD32F3x0
    \version 2022-01-06, V2.2.0, firmware for GD32F3x0
*/

/*
    Copyright (c) 2022, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f3x0_it.h"
#include "main.h"
#include "systick.h"
#include "gd32f350r_eval.h"
#include "I2C_IE.h"

int timer=0;
extern uint32_t R_W;
extern uint8_t i2c_buffer_receiver[7];
/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/ 
void SysTick_Handler(void)
{
    delay_decrement();
}

void EXTI0_1_IRQHandler(void)
{

}

void EXTI2_3_IRQHandler(void)
{

}


/*!
    \brief      this function handles external lines 4 to 15 interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void EXTI4_15_IRQHandler(void)
{
    if (RESET != exti_interrupt_flag_get(INT1_EXTI_LINE))
    {
        exti_interrupt_flag_clear(INT1_EXTI_LINE);
        /* 使能 I2C0 */
        i2c_enable(I2C0);
        /* 使能I2C发送应答 */
        i2c_ack_config(I2C0, I2C_ACK_ENABLE);
        /* 启用12C0中断 */
//        i2c_interrupt_enable(I2C0,I2C_INT_ERR | I2C_INT_BUF | I2C_INT_EV);
        i2c_interrupt_enable(I2C0, I2C_INT_ERR);
        i2c_interrupt_enable(I2C0, I2C_INT_BUF);
        i2c_interrupt_enable(I2C0, I2C_INT_EV);
        /* 等待直到I2C总线空闲 */
        while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
        /* 主机发送一个启动条件到12C总线*/
        i2c_start_on_bus(I2C0);
        i2c_rxbuffer = i2c_buffer_receiver;
        R_W=I2C_TRANSMITTER;
    }
}

/*!              
    \brief      处理I2C0事件中断请求
    \param[in]  none
    \param[out] none
    \retval     none
*/
void I2C0_EV_IRQHandler(void)
{
    I2C0_EventIRQ_Handler();
}

/*!
    \brief      函数处理I2C0错误中断请求
    \param[in]  none
    \param[out] none
    \retval     none
*/
void I2C0_ER_IRQHandler(void)
{
    I2C0_ErrorIRQ_Handler();
}


void TIMER1_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_UP))
    {
        //50ms进来一次
        timer++;
        if(timer>=160)  //8s以上
        {
            timer_disable(TIMER1);
        }
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_UP);
    }	
}


#include "gd32f3x0_i2c.h"
#include "I2C_IE.h"

uint32_t event1;
uint8_t i2c_receiver[16];
uint32_t R_W;
/*!
    \brief      处理12c0事件中断请求
    \param[in]  none
    \param[out] none
    \retval     none
*/
void I2C0_EventIRQ_Handler(void)
{ 
	  
	  //检查I2C中断标志  I2C_INT_FLAG_SBSEND 以主模式中断标志发出的启动条件
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_SBSEND)) /* 等待起始位发送完成 一旦 SBSEND 位被清 0，I2C 就开始发送地址或者地址头到 I2C 总线。 */
	{
        /* 主机发送从机地址 */
        i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, R_W); /* 发送器件地址,接收 I2C_RECEIVER 发送 I2C_TRANSMITTER */ 
    }
		//检查I2C中断标志 地址以主模式发送或以从模式接收并匹配中断标志
	else if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_ADDSEND)) /* 等待从机地址发送完成  地址位发送出去之后，I2C 硬件将 ADDSEND 位 置1 */
	{
        if(R_W == I2C_RECEIVER)
		{
            if((1 == I2C_nBytes)||(2 == I2C_nBytes))
            {
                /* 清除ADDSEND之前清除ACKEN */
                i2c_ack_config(I2C0, I2C_ACK_DISABLE);
                i2c_interrupt_flag_clear(I2C0,I2C_INT_FLAG_ADDSEND);//通过读 I2C_STAT0 寄存器然后读 I2C_STAT1 寄存器
            }
            else
            {
                /* 清除ADDSEND位 */
                i2c_interrupt_flag_clear(I2C0,I2C_INT_FLAG_ADDSEND);//通过读 I2C_STAT0 寄存器然后读 I2C_STAT1 寄存器
            }
        }
        else if(R_W == I2C_TRANSMITTER)
        {
            i2c_interrupt_flag_clear(I2C0,I2C_INT_FLAG_ADDSEND);//通过读 I2C_STAT0 寄存器然后读 I2C_STAT1 寄存器
            //寄存器地址
            i2c_data_transmit(I2C0,0xA7);//控制寄存器2  子地址最高位为 1，表示地址自增的批量读写模式。
            while(!i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_BTC));
            R_W=I2C_RECEIVER;
            i2c_start_on_bus(I2C0);					
        }
    }
    //检查I2C中断标志 12C DATA在接收中断标志期间不为空
    else if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_RBNE))//一旦接收到第一个字节，硬件会将 RBNE 位置 1。此时软件可以从 I2C_DATA 寄存器读取第一个字节，之后 RBNE 位被清 0。
    {//此后任何时候 RBNE 被置 1，软件就可以从 I2C_DATA 寄存器读取一个字节。
        /* 从I2C data读取数据字节*/
        if(I2C_nBytes>0)
        {
            if(3 == I2C_nBytes)
			{
                /* 等待，直到第二个最后的数据字节被接收到移位寄存器 */
                while(!i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_BTC));
                /* 为最后一个数据字节发送一个NACK */
                i2c_ack_config(I2C0, I2C_ACK_DISABLE);
            }
            /* 从I2C data读取数据字节*/
            *i2c_rxbuffer++ = i2c_data_receive(I2C0);
            I2C_nBytes--;
            if(0 == I2C_nBytes)
            {
                /* 发送停止条件 */
                R_W=I2C_TRANSMITTER;
                i2c_stop_on_bus(I2C0);
                i2c_ack_config(I2C0, I2C_ACK_ENABLE);
                i2c_ackpos_config(I2C0, I2C_ACKPOS_CURRENT);
//                i2c_interrupt_disable(I2C0, I2C_INT_ERR | I2C_INT_BUF | I2C_INT_EV);
                i2c_interrupt_enable(I2C0, I2C_INT_ERR);
                i2c_interrupt_enable(I2C0, I2C_INT_BUF);
                i2c_interrupt_enable(I2C0, I2C_INT_EV);
                I2C_nBytes=7;
            }
        }
    }
		
}

/*!
    \brief      handle I2C0 error interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void I2C0_ErrorIRQ_Handler(void)
{
    /* no acknowledge received */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_AERR)){
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_AERR);
			
    }

    /* SMBus alert */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_SMBALT)){
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_SMBALT);
    }

    /* bus timeout in SMBus mode */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_SMBTO)){
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_SMBTO);
			
    }

    /* over-run or under-run when SCL stretch is disabled */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_OUERR)){
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_OUERR);
    }

    /* arbitration lost */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_LOSTARB)){
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_LOSTARB);
    }

    /* bus error */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_BERR)){
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_BERR);
    }

    /* CRC value doesn't match */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_PECERR)){
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_PECERR);
    }

    /* disable the error interrupt */
//    i2c_interrupt_disable(I2C_BASE,I2C_INT_ERR | I2C_INT_BUF | I2C_INT_EV);
    
    /* disable the I2C0 interrupt */
    i2c_interrupt_disable(I2C0, I2C_INT_ERR);
    i2c_interrupt_disable(I2C0, I2C_INT_BUF);
    i2c_interrupt_disable(I2C0, I2C_INT_EV);
    
}


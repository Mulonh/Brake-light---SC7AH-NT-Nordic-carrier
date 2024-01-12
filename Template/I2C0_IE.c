
#include "gd32f3x0_i2c.h"
#include "I2C_IE.h"

uint32_t event1;
uint8_t i2c_receiver[16];
uint32_t R_W;
/*!
    \brief      ����12c0�¼��ж�����
    \param[in]  none
    \param[out] none
    \retval     none
*/
void I2C0_EventIRQ_Handler(void)
{ 
	  
	  //���I2C�жϱ�־  I2C_INT_FLAG_SBSEND ����ģʽ�жϱ�־��������������
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_SBSEND)) /* �ȴ���ʼλ������� һ�� SBSEND λ���� 0��I2C �Ϳ�ʼ���͵�ַ���ߵ�ַͷ�� I2C ���ߡ� */
	{
        /* �������ʹӻ���ַ */
        i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, R_W); /* ����������ַ,���� I2C_RECEIVER ���� I2C_TRANSMITTER */ 
    }
		//���I2C�жϱ�־ ��ַ����ģʽ���ͻ��Դ�ģʽ���ղ�ƥ���жϱ�־
	else if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_ADDSEND)) /* �ȴ��ӻ���ַ�������  ��ַλ���ͳ�ȥ֮��I2C Ӳ���� ADDSEND λ ��1 */
	{
        if(R_W == I2C_RECEIVER)
		{
            if((1 == I2C_nBytes)||(2 == I2C_nBytes))
            {
                /* ���ADDSEND֮ǰ���ACKEN */
                i2c_ack_config(I2C0, I2C_ACK_DISABLE);
                i2c_interrupt_flag_clear(I2C0,I2C_INT_FLAG_ADDSEND);//ͨ���� I2C_STAT0 �Ĵ���Ȼ��� I2C_STAT1 �Ĵ���
            }
            else
            {
                /* ���ADDSENDλ */
                i2c_interrupt_flag_clear(I2C0,I2C_INT_FLAG_ADDSEND);//ͨ���� I2C_STAT0 �Ĵ���Ȼ��� I2C_STAT1 �Ĵ���
            }
        }
        else if(R_W == I2C_TRANSMITTER)
        {
            i2c_interrupt_flag_clear(I2C0,I2C_INT_FLAG_ADDSEND);//ͨ���� I2C_STAT0 �Ĵ���Ȼ��� I2C_STAT1 �Ĵ���
            //�Ĵ�����ַ
            i2c_data_transmit(I2C0,0xA7);//���ƼĴ���2  �ӵ�ַ���λΪ 1����ʾ��ַ������������дģʽ��
            while(!i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_BTC));
            R_W=I2C_RECEIVER;
            i2c_start_on_bus(I2C0);					
        }
    }
    //���I2C�жϱ�־ 12C DATA�ڽ����жϱ�־�ڼ䲻Ϊ��
    else if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_RBNE))//һ�����յ���һ���ֽڣ�Ӳ���Ὣ RBNE λ�� 1����ʱ������Դ� I2C_DATA �Ĵ�����ȡ��һ���ֽڣ�֮�� RBNE λ���� 0��
    {//�˺��κ�ʱ�� RBNE ���� 1������Ϳ��Դ� I2C_DATA �Ĵ�����ȡһ���ֽڡ�
        /* ��I2C data��ȡ�����ֽ�*/
        if(I2C_nBytes>0)
        {
            if(3 == I2C_nBytes)
			{
                /* �ȴ���ֱ���ڶ������������ֽڱ����յ���λ�Ĵ��� */
                while(!i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_BTC));
                /* Ϊ���һ�������ֽڷ���һ��NACK */
                i2c_ack_config(I2C0, I2C_ACK_DISABLE);
            }
            /* ��I2C data��ȡ�����ֽ�*/
            *i2c_rxbuffer++ = i2c_data_receive(I2C0);
            I2C_nBytes--;
            if(0 == I2C_nBytes)
            {
                /* ����ֹͣ���� */
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


#include "gd32f3x0.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "gd32f350r_eval.h"
#include "gd32f3x0_it.h"
#include "gd32f3x0_libopt.h"
#include "I2C_IE.h"
#include <stdlib.h>
#include <float.h>

void rcu_config(void);
void gpio_configuration(void);
void timer_configuration(void);
void i2c_config(void);
void i2c_nvic_config(void);
void sc7a20_Init(void);
void sc7a20_manage(void);
void i2c_data_write(uint32_t i2c_periph, uint8_t data, uint8_t ADDR);
void irc40k_config(void);

//              寄存器地址 20   23   21   22   25   24   30   32   33   34   36   37
uint8_t SC7A20_REG[12] = {0x77,0x88,0x00,0x10,0x02,0x00,0x08,0x00,0x00,0x22,0x10,0x00};//400Hz 正常模式

uint16_t OUT_X,OUT_Y,OUT_Z;
int_least16_t SC7A20_OUT_X,SC7A20_OUT_Y,SC7A20_OUT_Z;
extern uint32_t R_W;
uint8_t i2c_buffer_receiver[7];

int normal_light=150;
int brake_light=1000;
int led_off=0;
int ix;
int delay_value=0;
float timer_delay;

int light_state = 0;
int timer_state = 0;

volatile uint8_t*       i2c_txbuffer;
volatile uint8_t*       i2c_rxbuffer;
volatile uint16_t       I2C_nBytes;
extern int timer;

#define Y_AXIS 0
#define Z_AXIS 1

int axis;
float accellOutput[] = { 0, 0};
float gravityOutput[] = { 0, 0 };


#define accellFilterTapsCount 3
float accellFilterForward_Taps[accellFilterTapsCount] = {0.001002806981976860902991410817719497572 , 0.002005613963953721805982821635438995145 , 0.001002806981976860902991410817719497572};
float accellFilterFeedback_Taps[accellFilterTapsCount] = {1, -1.908448680564245769630815630080178380013,  0.912459908492153082271158837102120742202};
float accellInputSamples[3][accellFilterTapsCount];
float accellOutputSamples[3][accellFilterTapsCount];
int accellSamplesBufferIndex = 0;


#define gravityFilterTapsCount 3
float gravityFilterForward_Taps[gravityFilterTapsCount] = { 0.000000617629752038056825743804392064851 , 0.000001235259504076113651487608784129701 , 0.000000617629752038056825743804392064851};
float gravityFilterFeedback_Taps[gravityFilterTapsCount] = { 1.00000, -1.997775920506088542794032036908902227879, 0.997778391025096689226359103486174717546};
float gravityInputSamples[3][gravityFilterTapsCount];
float gravityOutputSamples[3][gravityFilterTapsCount];
int gravitySamplesBufferIndex = 0; 

float forwardAccelleration;
int sampleIndex;

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

int main(void)
{
    rcu_config();
    gpio_configuration();
    systick_config();
    i2c_nvic_config();
    i2c_config();
    irc40k_config();

    sc7a20_Init();
    timer_configuration();
    I2C_nBytes = 7;
    i2c_rxbuffer = i2c_buffer_receiver;   
    
    gd_eval_INT_init(INT1,INT_MODE_EXTI,EXTI_TRIG_FALLING);  //INT1 下降沿触发
//    gd_eval_INT_init(INT2,INT_MODE_EXTI,EXTI_TRIG_FALLING);//INT2 下降沿触发
	
		for (axis = 0; axis < 2; axis++){
    for (sampleIndex = 0; sampleIndex < gravityFilterTapsCount; sampleIndex++)
    {
      gravityInputSamples[axis][sampleIndex] = 0;
      gravityOutputSamples[axis][sampleIndex] = 0;
    }

    for (sampleIndex = 0; sampleIndex < accellFilterTapsCount; sampleIndex++)
    {
      accellInputSamples[axis][sampleIndex] = 0;
      accellOutputSamples[axis][sampleIndex] = 0;
    }
		}
	
    if(RESET != rcu_flag_get(RCU_FLAG_FWDGTRST))
    {
        rcu_all_reset_flag_clear();
    }
    else
    {
        timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,normal_light);
        delay_1ms(500);
        timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,led_off);
        timer_disable(TIMER1);
    }
     
    /* USART configuration */
   gd_eval_com_init(EVAL_COM1);//串口调试使用
    
    timer_channel_output_pulse_value_config(TIMER13,TIMER_CH_0,normal_light);//LED ON
    
    while(1)
    {
        sc7a20_manage();
        /*if(timer>=10)
        {
            timer=0;
            timer_disable(TIMER1);
            timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0,led_off);//LED1 OFF
            timer_channel_output_pulse_value_config(TIMER13,TIMER_CH_0,normal_light);//LED ON
        }	*/
    }
}


/*
    \brief      enable the peripheral clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void)
{
    /* enable GPIOA clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* enable I2C0 clock */
    rcu_periph_clock_enable(RCU_I2C0);
    
}


/**
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
  */

void gpio_configuration(void)
{
    /*PWM configure PA7(TIMER0 CH0) as alternate function*/
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_7);
    
    /*PWM configure PA4(TIMER13 CH0) as alternate function*/
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    gpio_af_set(GPIOA, GPIO_AF_4, GPIO_PIN_4);

    /* I2C0  GPIO ports */
    
    /* connect PA9 to I2C0_SCL */
    gpio_af_set(GPIOA, GPIO_AF_4, GPIO_PIN_9);
    /* connect PA10 to I2C0_SDA */
    gpio_af_set(GPIOA, GPIO_AF_4, GPIO_PIN_10);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_9);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_10); 
    
    
}

/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void timer_configuration(void)
{
    /* -----------------------------------------------------------------------
    TIMER1 configuration: generate 3 PWM signals with 3 different duty cycles:
    TIMER1CLK = SystemCoreClock / 72 = 1MHz
    TIMER1 channel 1 duty cycle = (4000/ 16000)* 100  = 25%
    TIMER1 channel 2 duty cycle = (8000/ 16000)* 100  = 50%
    TIMER1 channel 3 duty cycle = (12000/ 16000)* 100 = 75%
    ----------------------------------------------------------------------- */
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    /* enable TIMER clock */
    rcu_periph_clock_enable(RCU_TIMER0); //系统时钟为72MHz
    rcu_periph_clock_enable(RCU_TIMER1); 
    rcu_periph_clock_enable(RCU_TIMER13); 
    
    timer_deinit(TIMER0);
    timer_deinit(TIMER1);
    timer_deinit(TIMER13);


    /* TIMER0 配置 */
    timer_initpara.prescaler         = 2; //预分频  
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;//边沿对齐
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;//向上计数
    timer_initpara.period            = 1000; //周期 频率=72MHz/(预分频+1)/(周期+1)
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER0,&timer_initpara);

    /* CH0 configuration in PWM0 mode */ 
    timer_ocintpara.outputstate  = TIMER_CCX_DISABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;     //使能互补通道输出
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    
    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocintpara);
    
    /* CH1 configuration in PWM mode1,duty cycle 50% 占空比设置*/
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 0); //占空比
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    timer_primary_output_config(TIMER0, ENABLE);
    
    /* 自动重新加载预加载启用 */
    timer_auto_reload_shadow_enable(TIMER0);
    /* 自动重载预加载启用 */
    timer_enable(TIMER0);
    
     /* TIMER13 配置 */
    timer_initpara.prescaler         = 2; //预分频  
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;//边沿对齐
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;//向上计数
    timer_initpara.period            = 1000; //周期 频率=72MHz/(预分频+1)/(周期+1)
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER13,&timer_initpara);

    /* CH1 configuration in PWM0 mode */
    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;

    timer_channel_output_config(TIMER13, TIMER_CH_0, &timer_ocintpara);

    /* CH1 configuration in PWM mode1,duty cycle 50% 占空比设置*/
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 0); //占空比
    timer_channel_output_mode_config(TIMER13, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER13, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
    /* 自动重新加载预加载启用 */
    timer_auto_reload_shadow_enable(TIMER13);
    /* 自动重载预加载启用 */
    timer_enable(TIMER13);

    /* TIMER2 配置 */
    timer_initpara.prescaler         = 5999; //预分频
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;//边沿对齐
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;//向上计数
    timer_initpara.period            = 151; //12.5 ms
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  

    timer_interrupt_enable(TIMER1, TIMER_INT_UP);
    nvic_irq_enable(TIMER1_IRQn, 1, 1);

    /* 自动重新加载预加载启用 */
    timer_auto_reload_shadow_enable(TIMER1);
    //使能定时器14
    timer_enable(TIMER1);
}

/*!
    \brief      cofigure the I2C0 and I2C1 interfaces..
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_config(void)
{
    /* I2C clock configure */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
    /* I2C address configure */
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_SLAVE_ADDRESS7);
    /* 使能 I2C0 */
    i2c_enable(I2C0);
    /* 使能I2C发送应答 */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}

/*!
    \brief      i2c_data_write
    \param[in]  i2c_periph,data,ADDR
    \param[out] none
    \retval     none
*/
void i2c_data_write(uint32_t i2c_periph, uint8_t data, uint8_t ADDR)
{
     /* 等待直到I2C总线空闲 */
    while(i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY));
    /* 主机发送一个启动条件到12C总线*/
    i2c_start_on_bus(i2c_periph);
    
    /* 等待，直到设置SBSEND位 */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND));
    /* 发送从地址到I2C总线 */
    i2c_master_addressing(i2c_periph, I2C0_SLAVE_ADDRESS7, I2C_TRANSMITTER);/* 发送器件地址,接收 I2C_RECEIVER 发送 I2C_TRANSMITTER */ 
    /*等待，直到ADDSEND位设置 */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND));
    /* 清除 ADDSEND 标志位 */
    i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);
    /* 等待直到传输数据缓冲区为空 */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_TBE));
    /* 发送数据字节 */
    i2c_data_transmit(i2c_periph,ADDR);
    
	while(!i2c_flag_get(i2c_periph, I2C_FLAG_TBE));
	i2c_data_transmit(i2c_periph,data);
    
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_TBE));
    /* send a stop condition to I2C bus*/
    i2c_stop_on_bus(i2c_periph);
//    while(I2C_CTL0(i2c_periph)&0x0200);
    while(I2C_CTL0(I2C0) & I2C_CTL0_STOP);
//    fwdgt_counter_reload();
}


/*!
    \brief      memory compare function
    \param[in]  src : source data
    \param[in]  dst : destination data
    \param[in]  length : the compare data length
    \param[out] none
    \retval     ErrStatus : ERROR or SUCCESS
*/
ErrStatus memory_compare(uint8_t *src, uint8_t *dst, uint16_t length)
{
    while(length--) {
        if(*src++ != *dst++) {
            return ERROR;
        }
    }
    return SUCCESS;
}


void sc7a20_Init(void)
{
    i2c_data_write(I2C0,SC7A20_REG[0],0x20);//低功耗模式 ODR 400Hz
    i2c_data_write(I2C0,SC7A20_REG[1],0x23); //+-2g
    i2c_data_write(I2C0,SC7A20_REG[2],0x21);//range bdu  0x20--0xA8
    i2c_data_write(I2C0,SC7A20_REG[3],0x22);//AOI中断on int1
    i2c_data_write(I2C0,SC7A20_REG[4],0x25);//控制寄存器6
    i2c_data_write(I2C0,SC7A20_REG[5],0x24);
    i2c_data_write(I2C0,SC7A20_REG[6],0x30);//中断1配置 x,y,z高事件“或”检测
    i2c_data_write(I2C0,SC7A20_REG[7],0x32);//中断1阀值寄存器  检测门限: 1-127, 值越小, 灵敏度越高
    i2c_data_write(I2C0,SC7A20_REG[8],0x33);//中断1持续时间
    fwdgt_counter_reload();
}

void sc7a20_manage(void)
{
    if(i2c_buffer_receiver[0] == 0x0F)
    {

					OUT_Y=(i2c_buffer_receiver[4]<<4)+(i2c_buffer_receiver[3]>>4);
					if((OUT_Y&0x800) == 0x800)SC7A20_OUT_Y=0xF000+OUT_Y;else  SC7A20_OUT_Y=OUT_Y;

					OUT_Z=(i2c_buffer_receiver[6]<<4)+(i2c_buffer_receiver[5]>>4);
					if((OUT_Z&0x800) == 0x800)SC7A20_OUT_Z=0xF000+OUT_Z;else  SC7A20_OUT_Z=OUT_Z;
					
					accellInputSamples[0][accellSamplesBufferIndex] = SC7A20_OUT_Y/(float)1023;
					gravityInputSamples[0][gravitySamplesBufferIndex] = SC7A20_OUT_Y/(float)1023;
					accellInputSamples[1][accellSamplesBufferIndex] = -SC7A20_OUT_Z/(float)1023;
					gravityInputSamples[1][gravitySamplesBufferIndex] = -SC7A20_OUT_Z/(float)1023;

  //Gravity is isolated with the low pass filter
			  accellOutput[0] = 0;
				gravityOutput[0] = 0;
				accellOutput[1] = 0;
				gravityOutput[1] = 0;
			
			//FILTER START
						
for (axis = 0; axis < 2; axis++) {
					sampleIndex = accellSamplesBufferIndex;
					for (ix = 0; ix < accellFilterTapsCount; ix++) {
						accellOutput[axis] += accellInputSamples[axis][sampleIndex] * accellFilterForward_Taps[ix];
						if (ix > 0) accellOutput[axis] -= accellOutputSamples[axis][sampleIndex] * accellFilterFeedback_Taps[ix];
								sampleIndex = (sampleIndex > 0) ? sampleIndex - 1 : accellFilterTapsCount - 1;
    }
					accellOutputSamples[axis][accellSamplesBufferIndex] = accellOutput[axis];
					sampleIndex = gravitySamplesBufferIndex;
						for (ix = 0; ix < gravityFilterTapsCount; ix++) {
						gravityOutput[axis] += gravityInputSamples[axis][sampleIndex] * gravityFilterForward_Taps[ix];
						if (ix > 0)gravityOutput[axis] -= gravityOutputSamples[axis][sampleIndex] * gravityFilterFeedback_Taps[ix];
						sampleIndex = (sampleIndex > 0) ? sampleIndex - 1 : gravityFilterTapsCount - 1;
    }
	// Move to the next  position of the circular buffer and wrap around if we are
  // at the end of the array.
    gravityOutputSamples[axis][gravitySamplesBufferIndex] = gravityOutput[axis];
  }
				//FILTER END

				// IF CHOCK IN Y DIRECTION -> Forward acceleration 0 for 600 samples
				if(SC7A20_OUT_Y/(float)1023 < -1.3 )
				{
					delay_value = 200;
				}

				if(delay_value > 0)
				{
					forwardAccelleration = 0;
					delay_value--;
				}
				else
				{
					forwardAccelleration =  accellOutput[Y_AXIS] * gravityOutput[Z_AXIS]-accellOutput[Z_AXIS]*gravityOutput[Y_AXIS];
				}
				
				accellSamplesBufferIndex = (accellSamplesBufferIndex + 1) % accellFilterTapsCount;
				gravitySamplesBufferIndex = (gravitySamplesBufferIndex + 1) % gravityFilterTapsCount;

			
				i2c_buffer_receiver[0]=0;
				
        if(forwardAccelleration<-0.08 && -SC7A20_OUT_Z/(float)1023 < 0)
        {
            timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,brake_light);//LED ON
            timer_channel_output_pulse_value_config(TIMER13,TIMER_CH_0,brake_light);//LED ON 100%
						light_state = 1;
        }
				
				
				
				// Turn off light if under a certain treshold
        else if(forwardAccelleration>= -0.04 && light_state == 1)
        {
					if (timer_state == 0 && timer != 0)
					{
						timer=0;
					}
					
					timer_enable(TIMER1);
					timer_state = 1;
					//Original 80
					//3Hz 80
					//2Hz 65
            if(timer>=72) //
            {
                timer=0;//定时计数清零
                timer_disable(TIMER1);//失能定时器
								timer_state = 0;
								light_state = 0;
							
                timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,led_off);//LED OFF
                timer_channel_output_pulse_value_config(TIMER13,TIMER_CH_0,normal_light);//LED 60%
            }
        }
    }
	fwdgt_counter_reload();
}

/*!
    \brief      cofigure the NVIC peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_nvic_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(I2C0_EV_IRQn, 0, 2);
    nvic_irq_enable(I2C0_ER_IRQn, 0, 1);
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM1, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM1, USART_FLAG_TBE));
    return ch;
}

/*!
    \brief      IRC40K configuration function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void irc40k_config(void)
{
    /* enable IRC40K */
    rcu_osci_on(RCU_IRC40K);
    /* wait till IRC40K is ready */
    rcu_osci_stab_wait(RCU_IRC40K);
}



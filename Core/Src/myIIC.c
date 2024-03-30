/**
  ******************************************************************************
  * @file    myIIC.c
  * @author  carrotgrx
  * @version V0.0
  * @date    2024
  * @brief   软件IIC 驱动
  ******************************************************************************
  */

#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_crs.h"
#include "stm32h7xx_ll_gpio.h"
#include "myIIC.h"

#define CPU_FREQUENCY_MHZ 480

/* 定义IIC总线连接的GPIO端口, 用户只需要修改下面4行代码即可任意改变SCL和SDA的引脚 */
#define GPIO_PORT_IIC     GPIOB                         /* GPIO端口 */
#define RCC_IIC_ENABLE    do { \
                          __IO uint32_t tmpreg; \
                          SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);\
                          /* Delay after an RCC peripheral clock enabling */ \
                          tmpreg = READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);\
                          UNUSED(tmpreg); \
                          } while(0)                  /* GPIO端口时钟 */
#define IIC_SCL_PIN       LL_GPIO_PIN_8                  /* 连接到SCL时钟线的GPIO */
#define IIC_SDA_PIN       LL_GPIO_PIN_9                  /* 连接到SDA数据线的GPIO */

#define IIC_SCL_1()  LL_GPIO_SetOutputPin(GPIO_PORT_IIC, IIC_SCL_PIN)		/* SCL = 1 */
#define IIC_SCL_0()  LL_GPIO_ResetOutputPin(GPIO_PORT_IIC, IIC_SCL_PIN)		/* SCL = 0 */

#define IIC_SDA_1()  LL_GPIO_SetOutputPin(GPIO_PORT_IIC, IIC_SDA_PIN)		/* SDA = 1 */
#define IIC_SDA_0()  LL_GPIO_ResetOutputPin(GPIO_PORT_IIC, IIC_SDA_PIN)		/* SDA = 0 */

#define IIC_SDA_READ()  LL_GPIO_IsInputPinSet(GPIO_PORT_IIC, IIC_SDA_PIN)	/* 读SDA口线状态 */


/*
*********************************************************************************************************
*	函 数 名: IIC_Delay
*	功能说明: IIC总线位延迟，最快400KHz
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void IIC_Delay(uint8_t delay)
{
    int last, curr, val;
    int temp;

    while (delay != 0)
    {
        temp = delay > 900 ? 900 : delay;
        last = SysTick->VAL;
        curr = last - CPU_FREQUENCY_MHZ * temp;
        if (curr >= 0)
        {
            do
            {
                val = SysTick->VAL;
            }
            while ((val < last) && (val >= curr));
        }
        else
        {
            curr += CPU_FREQUENCY_MHZ * 1000;
            do
            {
                val = SysTick->VAL;
            }
            while ((val <= last) || (val > curr));
        }
        delay -= temp;
    }
}

/*
*********************************************************************************************************
*	函 数 名: IIC_Start
*	功能说明: CPU发起IIC总线启动信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void IIC_Start(void)
{
    /* 当SCL高电平时，SDA出现一个下跳沿表示IIC总线启动信号 */
    IIC_SDA_1();
    IIC_SCL_1();
    IIC_Delay(4);
    IIC_SDA_0();
    IIC_Delay(4);
    IIC_SCL_0();
    IIC_Delay(4);
}

/*
*********************************************************************************************************
*	函 数 名: IIC_Start
*	功能说明: CPU发起IIC总线停止信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void IIC_Stop(void)
{
    /* 当SCL高电平时，SDA出现一个上跳沿表示IIC总线停止信号 */
    IIC_SDA_0();
    IIC_SCL_1();
    IIC_Delay(4);
    IIC_SDA_1();
}

/*
*********************************************************************************************************
*	函 数 名: IIC_SendByte
*	功能说明: CPU向IIC总线设备发送8bit数据
*	形    参：_ucByte ： 等待发送的字节
*	返 回 值: 无
*********************************************************************************************************
*/
void IIC_Send_Byte(uint8_t _ucByte)
{
    uint8_t i;

    /* 先发送字节的高位bit7 */
    for (i = 0; i < 8; i++)
    {
        if (_ucByte & 0x80)
        {
            IIC_SDA_1();
        }
        else
        {
            IIC_SDA_0();
        }
        IIC_Delay(2);
        IIC_SCL_1();
        IIC_Delay(2);
        IIC_SCL_0();
        if (i == 7)
        {
            IIC_SDA_1(); // 释放总线
        }
        _ucByte <<= 1;	/* 左移一个bit */
        IIC_Delay(2);
    }
}

/*
*********************************************************************************************************
*	函 数 名: IIC_ReadByte
*	功能说明: CPU从IIC总线设备读取8bit数据
*	形    参：无
*	返 回 值: 读到的数据
*********************************************************************************************************
*/
uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t i;
    uint8_t value;

    /* 读到第1个bit为数据的bit7 */
    value = 0;
    for (i = 0; i < 8; i++)
    {
        value <<= 1;
        IIC_SCL_1();
        IIC_Delay(2);
        if (IIC_SDA_READ())
        {
            value++;
        }
        IIC_SCL_0();
        IIC_Delay(2);
    }
    if(ack == 0)
        IIC_NAck();
    else
        IIC_Ack();
    return value;
}

/*
*********************************************************************************************************
*	函 数 名: IIC_WaitAck
*	功能说明: CPU产生一个时钟，并读取器件的ACK应答信号
*	形    参：无
*	返 回 值: 返回0表示正确应答，1表示无器件响应
*********************************************************************************************************
*/
uint8_t IIC_Wait_Ack(void)
{
    uint8_t re;

    IIC_SDA_1();	/* CPU释放SDA总线 */
    IIC_Delay(1);
    IIC_SCL_1();	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
    IIC_Delay(1);
    if (IIC_SDA_READ())	/* CPU读取SDA口线状态 */
    {
        re = 1;
    }
    else
    {
        re = 0;
    }
    IIC_SCL_0();
    IIC_Delay(2);
    return re;
}

/*
*********************************************************************************************************
*	函 数 名: IIC_Ack
*	功能说明: CPU产生一个ACK信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void IIC_Ack(void)
{
    IIC_SDA_0();	/* CPU驱动SDA = 0 */
    IIC_Delay(2);
    IIC_SCL_1();	/* CPU产生1个时钟 */
    IIC_Delay(2);
    IIC_SCL_0();
    IIC_Delay(2);
    IIC_SDA_1();	/* CPU释放SDA总线 */
}

/*
*********************************************************************************************************
*	函 数 名: IIC_NAck
*	功能说明: CPU产生1个NACK信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void IIC_NAck(void)
{
    IIC_SDA_1();	/* CPU驱动SDA = 1 */
    IIC_Delay(2);
    IIC_SCL_1();	/* CPU产生1个时钟 */
    IIC_Delay(2);
    IIC_SCL_0();
    IIC_Delay(2);
}

/*
*********************************************************************************************************
*	函 数 名: IIC_GPIO_Config
*	功能说明: 配置IIC总线的GPIO，采用模拟IO的方式实现
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void IIC_GPIO_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStructure;

    RCC_IIC_ENABLE;	/* 打开GPIO时钟 */

    GPIO_InitStructure.Pin = IIC_SCL_PIN | IIC_SDA_PIN;
    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = LL_GPIO_MODE_OUTPUT;  	/* 开漏输出 */
    LL_GPIO_Init(GPIO_PORT_IIC, &GPIO_InitStructure);

    /* 给一个停止信号, 复位IIC总线上的所有设备到待机模式 */
    IIC_Stop();
}

/*
*********************************************************************************************************
*	函 数 名: IIC_CheckDevice
*	功能说明: 检测IIC总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
*	形    参：_Address：设备的IIC总线地址
*	返 回 值: 返回值 0 表示正确， 返回1表示未探测到
*********************************************************************************************************
*/
uint8_t IIC_CheckDevice(uint8_t _Address)
{
    uint8_t ucAck;

    IIC_GPIO_Init();		/* 配置GPIO */

    IIC_Start();		/* 发送启动信号 */

    /* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
    IIC_Send_Byte(_Address|IIC_WR);
    ucAck = IIC_Wait_Ack();	/* 检测设备的ACK应答 */

    IIC_Stop();			/* 发送停止信号 */

    return ucAck;
}

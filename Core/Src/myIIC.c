/**
  ******************************************************************************
  * @file    myIIC.c
  * @author  carrotgrx
  * @version V0.0
  * @date    2024
  * @brief   ���IIC ����
  ******************************************************************************
  */

#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_crs.h"
#include "stm32h7xx_ll_gpio.h"
#include "myIIC.h"

#define CPU_FREQUENCY_MHZ 480

/* ����IIC�������ӵ�GPIO�˿�, �û�ֻ��Ҫ�޸�����4�д��뼴������ı�SCL��SDA������ */
#define GPIO_PORT_IIC     GPIOB                         /* GPIO�˿� */
#define RCC_IIC_ENABLE    do { \
                          __IO uint32_t tmpreg; \
                          SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);\
                          /* Delay after an RCC peripheral clock enabling */ \
                          tmpreg = READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);\
                          UNUSED(tmpreg); \
                          } while(0)                  /* GPIO�˿�ʱ�� */
#define IIC_SCL_PIN       LL_GPIO_PIN_8                  /* ���ӵ�SCLʱ���ߵ�GPIO */
#define IIC_SDA_PIN       LL_GPIO_PIN_9                  /* ���ӵ�SDA�����ߵ�GPIO */

#define IIC_SCL_1()  LL_GPIO_SetOutputPin(GPIO_PORT_IIC, IIC_SCL_PIN)		/* SCL = 1 */
#define IIC_SCL_0()  LL_GPIO_ResetOutputPin(GPIO_PORT_IIC, IIC_SCL_PIN)		/* SCL = 0 */

#define IIC_SDA_1()  LL_GPIO_SetOutputPin(GPIO_PORT_IIC, IIC_SDA_PIN)		/* SDA = 1 */
#define IIC_SDA_0()  LL_GPIO_ResetOutputPin(GPIO_PORT_IIC, IIC_SDA_PIN)		/* SDA = 0 */

#define IIC_SDA_READ()  LL_GPIO_IsInputPinSet(GPIO_PORT_IIC, IIC_SDA_PIN)	/* ��SDA����״̬ */


/*
*********************************************************************************************************
*	�� �� ��: IIC_Delay
*	����˵��: IIC����λ�ӳ٣����400KHz
*	��    �Σ���
*	�� �� ֵ: ��
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
*	�� �� ��: IIC_Start
*	����˵��: CPU����IIC���������ź�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void IIC_Start(void)
{
    /* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾIIC���������ź� */
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
*	�� �� ��: IIC_Start
*	����˵��: CPU����IIC����ֹͣ�ź�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void IIC_Stop(void)
{
    /* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾIIC����ֹͣ�ź� */
    IIC_SDA_0();
    IIC_SCL_1();
    IIC_Delay(4);
    IIC_SDA_1();
}

/*
*********************************************************************************************************
*	�� �� ��: IIC_SendByte
*	����˵��: CPU��IIC�����豸����8bit����
*	��    �Σ�_ucByte �� �ȴ����͵��ֽ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void IIC_Send_Byte(uint8_t _ucByte)
{
    uint8_t i;

    /* �ȷ����ֽڵĸ�λbit7 */
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
            IIC_SDA_1(); // �ͷ�����
        }
        _ucByte <<= 1;	/* ����һ��bit */
        IIC_Delay(2);
    }
}

/*
*********************************************************************************************************
*	�� �� ��: IIC_ReadByte
*	����˵��: CPU��IIC�����豸��ȡ8bit����
*	��    �Σ���
*	�� �� ֵ: ����������
*********************************************************************************************************
*/
uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t i;
    uint8_t value;

    /* ������1��bitΪ���ݵ�bit7 */
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
*	�� �� ��: IIC_WaitAck
*	����˵��: CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
*	��    �Σ���
*	�� �� ֵ: ����0��ʾ��ȷӦ��1��ʾ��������Ӧ
*********************************************************************************************************
*/
uint8_t IIC_Wait_Ack(void)
{
    uint8_t re;

    IIC_SDA_1();	/* CPU�ͷ�SDA���� */
    IIC_Delay(1);
    IIC_SCL_1();	/* CPU����SCL = 1, ��ʱ�����᷵��ACKӦ�� */
    IIC_Delay(1);
    if (IIC_SDA_READ())	/* CPU��ȡSDA����״̬ */
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
*	�� �� ��: IIC_Ack
*	����˵��: CPU����һ��ACK�ź�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void IIC_Ack(void)
{
    IIC_SDA_0();	/* CPU����SDA = 0 */
    IIC_Delay(2);
    IIC_SCL_1();	/* CPU����1��ʱ�� */
    IIC_Delay(2);
    IIC_SCL_0();
    IIC_Delay(2);
    IIC_SDA_1();	/* CPU�ͷ�SDA���� */
}

/*
*********************************************************************************************************
*	�� �� ��: IIC_NAck
*	����˵��: CPU����1��NACK�ź�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void IIC_NAck(void)
{
    IIC_SDA_1();	/* CPU����SDA = 1 */
    IIC_Delay(2);
    IIC_SCL_1();	/* CPU����1��ʱ�� */
    IIC_Delay(2);
    IIC_SCL_0();
    IIC_Delay(2);
}

/*
*********************************************************************************************************
*	�� �� ��: IIC_GPIO_Config
*	����˵��: ����IIC���ߵ�GPIO������ģ��IO�ķ�ʽʵ��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void IIC_GPIO_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStructure;

    RCC_IIC_ENABLE;	/* ��GPIOʱ�� */

    GPIO_InitStructure.Pin = IIC_SCL_PIN | IIC_SDA_PIN;
    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = LL_GPIO_MODE_OUTPUT;  	/* ��©��� */
    LL_GPIO_Init(GPIO_PORT_IIC, &GPIO_InitStructure);

    /* ��һ��ֹͣ�ź�, ��λIIC�����ϵ������豸������ģʽ */
    IIC_Stop();
}

/*
*********************************************************************************************************
*	�� �� ��: IIC_CheckDevice
*	����˵��: ���IIC�����豸��CPU�����豸��ַ��Ȼ���ȡ�豸Ӧ�����жϸ��豸�Ƿ����
*	��    �Σ�_Address���豸��IIC���ߵ�ַ
*	�� �� ֵ: ����ֵ 0 ��ʾ��ȷ�� ����1��ʾδ̽�⵽
*********************************************************************************************************
*/
uint8_t IIC_CheckDevice(uint8_t _Address)
{
    uint8_t ucAck;

    IIC_GPIO_Init();		/* ����GPIO */

    IIC_Start();		/* ���������ź� */

    /* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
    IIC_Send_Byte(_Address|IIC_WR);
    ucAck = IIC_Wait_Ack();	/* ����豸��ACKӦ�� */

    IIC_Stop();			/* ����ֹͣ�ź� */

    return ucAck;
}

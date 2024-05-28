#include "delay.h"

static uint16_t  g_fac_us = 0;      /* us��ʱ������ */
/**
 * @brief       ��ʼ���ӳٺ���
 * @param       sysclk: ϵͳʱ��Ƶ��, ��CPUƵ��(HCLK)
 * @retval      ��
 */
void delay_init(uint16_t sysclk)
{

    SysTick->CTRL = 0;                                          /* ��Systick״̬���Ա���һ�����裬������￪���жϻ�ر����ж� */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);   /* SYSTICKʹ���ں�ʱ��Դ8��Ƶ,��systick�ļ��������ֵֻ��2^24 */
    g_fac_us = sysclk / 8;                                      /* �����Ƿ�ʹ��OS,g_fac_us����Ҫʹ��,��Ϊ1us�Ļ���ʱ�� */

}
/**
 * @brief       ��ʱnus
 * @param       nus: Ҫ��ʱ��us��.
 * @note        ע��: nus��ֵ,��Ҫ����1864135us(���ֵ��2^24 / g_fac_us @g_fac_us = 9)
 * @retval      ��
 */
void delay_us(uint32_t nus)
{
    uint32_t temp;
    SysTick->LOAD = nus * g_fac_us; /* ʱ����� */
    SysTick->VAL = 0x00;            /* ��ռ����� */
    SysTick->CTRL |= 1 << 0 ;       /* ��ʼ���� */

    do
    {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16))); /* CTRL.ENABLEλ����Ϊ1, ���ȴ�ʱ�䵽�� */

    SysTick->CTRL &= ~(1 << 0) ;    /* �ر�SYSTICK */
    SysTick->VAL = 0X00;            /* ��ռ����� */
}

/**
 * @brief       ��ʱnms
 * @param       nms: Ҫ��ʱ��ms�� (0< nms <= 65535)
 * @retval      ��
 */
void delay_ms(uint16_t nms)
{
    uint32_t repeat = nms / 1000;   /*  ������1000,�ǿ��ǵ������г�ƵӦ��,
                                     *  ����128Mhz��ʱ��, delay_us���ֻ����ʱ1048576us������
                                     */
    uint32_t remain = nms % 1000;

    while (repeat)
    {
        delay_us(1000 * 1000);      /* ����delay_us ʵ�� 1000ms ��ʱ */
        repeat--;
    }

    if (remain)
    {
        delay_us(remain * 1000);    /* ����delay_us, ��β����ʱ(remain ms)������ */
    }
}

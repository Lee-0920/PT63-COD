/*
 * HardwareFlag.c
 *
 *  Created on: 2020年4月10日
 *      Author: Administrator
 */

#include "stm32f4xx.h"
#include <HardwareType.h>

#define HARDWARETYPE_BIT0_RCC       RCC_AHB1Periph_GPIOE
#define HARDWARETYPE_BIT0_PORT     GPIOE
#define HARDWARETYPE_BIT0_PIN        GPIO_Pin_5

#define HARDWARETYPE_BIT1_RCC       RCC_AHB1Periph_GPIOE
#define HARDWARETYPE_BIT1_PORT    GPIOE
#define HARDWARETYPE_BIT1_PIN       GPIO_Pin_6

/**
 * @brief 硬件板卡类型标记初始化
 */
void HardwareType_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(HARDWARETYPE_BIT0_RCC | HARDWARETYPE_BIT1_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_InitStructure.GPIO_Pin = HARDWARETYPE_BIT0_PIN;
    GPIO_Init(HARDWARETYPE_BIT0_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = HARDWARETYPE_BIT1_PIN;
    GPIO_Init(HARDWARETYPE_BIT1_PORT, &GPIO_InitStructure);
}

/**
 * @brief 读取硬件版本标记
 * @return 标记值
 */
Uint8 HardwareType_GetValue(void)
{
    Uint8 value = 0;

    if (GPIO_ReadInputDataBit(HARDWARETYPE_BIT0_PORT, HARDWARETYPE_BIT0_PIN))
    {
        value += 1;
    }
    if (GPIO_ReadInputDataBit(HARDWARETYPE_BIT1_PORT, HARDWARETYPE_BIT1_PIN))
    {
        value += 2;
    }

    return value;
}

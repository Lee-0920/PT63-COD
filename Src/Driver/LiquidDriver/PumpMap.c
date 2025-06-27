/*
 * PumpMap.c
 *
 *  Created on: 2016年5月30日
 *      Author: Administrator
 */
#include <LiquidDriver/PumpDriver.h>
#include <LiquidDriver/PumpMap.h>
#include "stm32f4xx.h"

void PumpMap_Init(Pump *pump)
{

    pump[0].driver.pinClock = GPIO_Pin_11;
    pump[0].driver.portClock = GPIOA;
    pump[0].driver.rccClock = RCC_AHB1Periph_GPIOA;

    pump[0].driver.pinDir = GPIO_Pin_12;
    pump[0].driver.portDir = GPIOA;
    pump[0].driver.rccDir = RCC_AHB1Periph_GPIOA;

    pump[0].driver.pinEnable = GPIO_Pin_10;
    pump[0].driver.portEnable = GPIOC;
    pump[0].driver.rccEnable = RCC_AHB1Periph_GPIOC;

    PumpDriver_Init(&pump[0].driver);
    PumpDriver_PullLow(&pump[0].driver);
    PumpDriver_Disable(&pump[0].driver);


    pump[1].driver.pinClock = GPIO_Pin_8;
    pump[1].driver.portClock = GPIOC;
    pump[1].driver.rccClock = RCC_AHB1Periph_GPIOC;

    pump[1].driver.pinDir = GPIO_Pin_7;
    pump[1].driver.portDir = GPIOC;
    pump[1].driver.rccDir = RCC_AHB1Periph_GPIOC;

    pump[1].driver.pinEnable = GPIO_Pin_9;
    pump[1].driver.portEnable = GPIOC;
    pump[1].driver.rccEnable = RCC_AHB1Periph_GPIOC;

    PumpDriver_Init(&pump[1].driver);
    PumpDriver_PullLow(&pump[1].driver);
    PumpDriver_Disable(&pump[1].driver);

    pump[2].driver.pinClock = GPIO_Pin_0;
    pump[2].driver.portClock = GPIOB;
    pump[2].driver.rccClock = RCC_AHB1Periph_GPIOB;

    pump[2].driver.pinDir = GPIO_Pin_1;
    pump[2].driver.portDir = GPIOB;
    pump[2].driver.rccDir = RCC_AHB1Periph_GPIOB;

    pump[2].driver.pinEnable = GPIO_Pin_5;
    pump[2].driver.portEnable = GPIOC;
    pump[2].driver.rccEnable = RCC_AHB1Periph_GPIOC;

    PumpDriver_Init(&pump[2].driver);
    PumpDriver_PullLow(&pump[2].driver);
    PumpDriver_Disable(&pump[2].driver);
}


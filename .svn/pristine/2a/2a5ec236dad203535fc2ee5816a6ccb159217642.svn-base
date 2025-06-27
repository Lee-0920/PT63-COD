/*
 * ThermostatDeviceMap.c
 *
 *  Created on: 2019年8月16日
 *      Author: Administrator
 */

/*
 * ThermostatDeviceMap.c
 *
 *  Created on: 2017年11月16日
 *      Author: LIANG
 */

#include "ThermostatDeviceMap.h"
#include "Driver/System.h"
#include <string.h>
#include "Tracer/Trace.h"

//加热丝输出
static Bool ThermostatDeviceMap_SetOutputWay1(ThermostatDeviceDriver *deviceDriver, float level);

//风扇输出
static Bool ThermostatDeviceMap_SetOutputWay2(ThermostatDeviceDriver *deviceDriver, float level);

void ThermostatDeviceMap_Init(ThermostatDevice* device)
{
    //加热设备
    //消解加热丝1 //TEMP_CTRL
    device[0].maxDutyCycle = 0.25;
    device[0].setOutputWayFunc = ThermostatDeviceMap_SetOutputWay1;
    device[0].deviceDriver.mode = THERMOSTATDEVICEDRIVER_PWM;
    device[0].deviceDriver.port = GPIOE;
    device[0].deviceDriver.pin = GPIO_Pin_9;
    device[0].deviceDriver.gpioRcc = RCC_AHB1Periph_GPIOE;
    device[0].deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource9;
    device[0].deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM1;
    device[0].deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB2PeriphClockCmd;
    device[0].deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB2Periph_TIM1;
    device[0].deviceDriver.modeConfig.PWMConfig.timerPrescaler = 179;
    device[0].deviceDriver.modeConfig.PWMConfig.timerPeriod = 49999;
    device[0].deviceDriver.modeConfig.PWMConfig.timerChannel = 1;
    device[0].deviceDriver.modeConfig.PWMConfig.timer = TIM1;
    device[0].deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
    device[0].deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM2;//在向上计数模式下，TIMx_CNT < TIMx_CCR1时，通道1为无效电平
    ThermostatDevice_Init(&device[0]);

    //消解加热丝2 //TEMP_CTRL2
    device[1].maxDutyCycle = 0.25;
    device[1].setOutputWayFunc = ThermostatDeviceMap_SetOutputWay1;
    device[1].deviceDriver.mode = THERMOSTATDEVICEDRIVER_VIRTUAL_PWM;
    device[1].deviceDriver.port = GPIOE;
    device[1].deviceDriver.pin = GPIO_Pin_8;
    device[1].deviceDriver.gpioRcc = RCC_AHB1Periph_GPIOE;
    device[1].deviceDriver.modeConfig.IOConfig.open = Bit_SET;
    device[1].deviceDriver.modeConfig.IOConfig.close = Bit_RESET;
    ThermostatDevice_Init(&device[1]);

    //制冷设备
    //消解风扇1 //FAN_CTRL
    device[2].maxDutyCycle = 1;
    device[2].setOutputWayFunc = ThermostatDeviceMap_SetOutputWay2;
    device[2].deviceDriver.mode = THERMOSTATDEVICEDRIVER_PWM;
    device[2].deviceDriver.port = GPIOD;
    device[2].deviceDriver.pin = GPIO_Pin_14;
    device[2].deviceDriver.gpioRcc = RCC_AHB1Periph_GPIOD;
    device[2].deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource14;
    device[2].deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM4;
    device[2].deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB1PeriphClockCmd;
    device[2].deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB1Periph_TIM4;
    device[2].deviceDriver.modeConfig.PWMConfig.timerPrescaler = 89;
    device[2].deviceDriver.modeConfig.PWMConfig.timerPeriod = 49999;
    device[2].deviceDriver.modeConfig.PWMConfig.timerChannel = 3;
    device[2].deviceDriver.modeConfig.PWMConfig.timer = TIM4;
    device[2].deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
    device[2].deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM2;
    ThermostatDevice_Init(&device[2]);

    //消解风扇2 //VALVE18
    device[3].maxDutyCycle = 1;
    device[3].setOutputWayFunc = ThermostatDeviceMap_SetOutputWay2;
    device[3].deviceDriver.mode = THERMOSTATDEVICEDRIVER_PWM;
    device[3].deviceDriver.port = GPIOD;
    device[3].deviceDriver.pin = GPIO_Pin_12;
    device[3].deviceDriver.gpioRcc = RCC_AHB1Periph_GPIOD;
    device[3].deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource12;
    device[3].deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM4;
    device[3].deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB1PeriphClockCmd;
    device[3].deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB1Periph_TIM4;
    device[3].deviceDriver.modeConfig.PWMConfig.timerPrescaler = 89;
    device[3].deviceDriver.modeConfig.PWMConfig.timerPeriod = 49999;
    device[3].deviceDriver.modeConfig.PWMConfig.timerChannel = 1;
    device[3].deviceDriver.modeConfig.PWMConfig.timer = TIM4;
    device[3].deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
    device[3].deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM2;
    ThermostatDevice_Init(&device[3]);
}

static Bool ThermostatDeviceMap_SetOutputWay1(ThermostatDeviceDriver *deviceDriver, float level)
{
    TRACE_CODE("\n Output way 1");
    return ThermostatDeviceDriver_SetOutput(deviceDriver, level);
}

static Bool ThermostatDeviceMap_SetOutputWay2(ThermostatDeviceDriver *deviceDriver, float level)
{
    TRACE_CODE("\n Output way 2");
    if (0 != level)
    {
        level = 0.5 * level + 0.5;
        if (level < 0.75)
        {
            ThermostatDeviceDriver_SetOutput(deviceDriver, 1);
            System_Delay(200);
        }
    }
    return ThermostatDeviceDriver_SetOutput(deviceDriver, level);
}

char* ThermostatDeviceMap_GetName(Uint8 index)
{
    static char name[35] = "";
    switch(index)
    {
    case MEASUREMODULE_HEATER1:
        strcpy(name, "MeasureModuleHeater1");
        break;
    case MEASUREMODULE_HEATER2:
        strcpy(name, "MeasureModuleHeater2");
        break;
    case MEASUREMODULE_FAN1:
        strcpy(name, "MeasureModuleFan1");
        break;
    case MEASUREMODULE_FAN2:
        strcpy(name, "MeasureModuleFan2");
        break;
    default:
        strcpy(name, "NULL");
        break;
    }
    return name;
}


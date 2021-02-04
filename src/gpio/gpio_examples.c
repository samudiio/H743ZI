/*
 * gpio_examples.c
 *
 *  Created on: Feb 2, 2021
 *      Author: Samuel
 */

#include "gpio_examples.h"
#include "stm32h743zi.h"

/* Macro definitions */

#define HIGH 1
#define BTN_PRESSED HIGH

#define LED1_GREEN  GPIO_PIN_NO_0
#define LED2_BLUE   GPIO_PIN_NO_7
#define LED3_RED    GPIO_PIN_NO_14

/* Function definitions */

void delay(void)
{
    for(uint32_t i = 0 ; i < 2000000 ; i ++);
}

void LED_Toggling_With_PushPull_Cfg(void)
{
    GPIO_Handle_t GpioLed;

    GpioLed.pGPIOx = GPIOB;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = LED2_BLUE;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOB, ENABLE);

    GPIO_Init(&GpioLed);

    while(1)
    {
        GPIO_ToggleOutputPin(GPIOB, LED2_BLUE);
        delay();
    }
}

void LED_Toggling_With_OpenDrain_Cfg(void)
{
    GPIO_Handle_t GpioLed;

    GpioLed.pGPIOx = GPIOB;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = LED1_GREEN;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOB, ENABLE);

    GPIO_Init(&GpioLed);

    while(1)
    {
        GPIO_ToggleOutputPin(GPIOB, LED1_GREEN);
        delay();
    }
}

void Led_Button(void)
{
    GPIO_Handle_t GpioLed;
    GPIO_Handle_t GpioBtn;

    /* LED GPIO configuration */
    GpioLed.pGPIOx = GPIOB;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = LED3_RED;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOB, ENABLE);

    GPIO_Init(&GpioLed);

    /* Button GPIO configuration */
    GpioBtn.pGPIOx = GPIOC;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOC, ENABLE);

    GPIO_Init(&GpioBtn);

    while(1)
    {
        if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED)
        {
            delay();
            GPIO_ToggleOutputPin(GPIOB, LED3_RED);
        }
    }
}


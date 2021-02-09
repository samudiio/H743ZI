/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Samuel Martinez
 * @brief          : Main program body
 * *****************************************************************************
 */
#include "stm32h743zi.h"
#include "gpio_examples.h"


int main(void)
{

    //LED_Toggling_With_PushPull_Cfg();
    //LED_Toggling_With_OpenDrain_Cfg();
    //Led_Button();
    Led_Button_Ext();


	return 0;
}

void EXTI0_IRQHandler(void)
{
    /* Handle the interrupt */
    GPIO_IRQHandling(0);
}

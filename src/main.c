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
    //IntButton_Led();
    //ExtButton_Led();
    //ExtButton_Led_Interrupt();
    IntButton_Led_Interrupt();

	return 0;
}


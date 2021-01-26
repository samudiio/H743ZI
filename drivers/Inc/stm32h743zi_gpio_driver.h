/*
 * stm32h743zi_gpio_driver.h
 *
 *  Created on: Jan 25, 2021
 *      Author: Samuel
 */

#ifndef INC_STM32H743ZI_GPIO_DRIVER_H_
#define INC_STM32H743ZI_GPIO_DRIVER_H_

#include "stm32h743zi.h"


/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_AltFuncMode;
}GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIOpin
 */
typedef struct
{
    GPIO_RegDef_t *pGPIOx;              /* This holds the base address of the GPIO port to which the pin belongs */
    GPIO_PinConfig_t GPIO_PinConfig;    /* This holds GPIO pin configuration settings */
}GPIO_Handle_t;

/*****************************************************************************************
 *                                  APIs supported by this driver
 *****************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(void);

/*
 * Init and De-init
 */
void GPIO_Init(void);
void GPIO_Deinit(void);

/*
 * Data read and write
 */
void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_WriteToOutputPin(void);
void GPIO_WriteToOutputPort(void);
void GPIO_ToggleOutputPin(void);

/*
 * IRW Configuration adn ISR handling
 */
void GPIO_IRQConfig(void);
void GPIO_IRQHandling(void);

#endif /* INC_STM32H743ZI_GPIO_DRIVER_H_ */

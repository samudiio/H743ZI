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
 * GPIO pin numbers
 */
typedef enum{
    GPIO_PIN_NO_0,
    GPIO_PIN_NO_1,
    GPIO_PIN_NO_2,
    GPIO_PIN_NO_3,
    GPIO_PIN_NO_4,
    GPIO_PIN_NO_5,
    GPIO_PIN_NO_6,
    GPIO_PIN_NO_7,
    GPIO_PIN_NO_8,
    GPIO_PIN_NO_9,
    GPIO_PIN_NO_10,
    GPIO_PIN_NO_11,
    GPIO_PIN_NO_12,
    GPIO_PIN_NO_13,
    GPIO_PIN_NO_14,
    GPIO_PIN_NO_15
}GPIO_Pin_Numbers_t;

/*
 * GPIO pin possible modes.
 */
typedef enum{
    GPIO_MODE_IN     = 0,       /* Input mode */
    GPIO_MODE_OUT    = 1,       /* General purpose output mode */
    GPIO_MODE_ALTFN  = 2,       /* Alternate function mode */
    GPIO_MODE_ANALOG = 3,       /* Analog mode (reset state) */
    GPIO_MODE_IT_FT  = 4,       /* Irq Falling Edge */
    GPIO_MODE_IT_RT  = 5,       /* Irq Rising Edge */
    GPIO_MODE_IT_RFT = 6        /* Irq Rising Edge-Falling Edge Trigger */
}GPIO_Pin_Modes_t;

/*
 * GPIO pin possible output types.
 */
typedef enum{
    GPIO_OP_TYPE_PP = 0,        /* Output push-pull (reset state) */
    GPIO_OP_TYPE_OD = 1         /* Output open-drain */
}GPIO_OutputType_t;

/*
 * GPIO pin possible output speeds.
 */
typedef enum{
    GPIO_SPEED_LOW    = 0,      /* Low speed */
    GPIO_SPEED_MEDIUM = 1,      /* Medium speed */
    GPIO_SPEED_FAST   = 2,      /* High speed */
    GPOI_SPEED_HIGH   = 3       /* Very high speed*/
}GPIO_Pin_Speed_t;

/*
 * GPIO pin pull up AND pull down configuration macros.
 */
typedef enum{
    GPIO_NO_PUPD = 0,           /* No pull-up, pull-down */
    GPIO_PIN_PU  = 1,           /* Pull-up */
    GPIO_PIN_PD  = 2            /* Pull-down */
}GPIO_Pin_PuPd_t;

/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct
{
    GPIO_Pin_Numbers_t GPIO_PinNumber;
    GPIO_Pin_Modes_t GPIO_PinMode;
    GPIO_Pin_Speed_t GPIO_PinSpeed;
    GPIO_Pin_PuPd_t GPIO_PinPuPdControl;
    GPIO_OutputType_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;
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
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32H743ZI_GPIO_DRIVER_H_ */

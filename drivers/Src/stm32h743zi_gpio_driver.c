/*
 * stm32h743zi_gpio_driver.c
 *
 *  Created on: Jan 25, 2021
 *      Author: Samuel
 */


#include "stm32h743zi_gpio_driver.h"


/*********************************************************************
 * @fn                - GPIO_PeriClockControl
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @return            - none
 * @Note              - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }else if (pGPIOx == GPIOJ)
        {
            GPIOJ_PCLK_EN();
        }else if (pGPIOx == GPIOK)
        {
            GPIOK_PCLK_EN();
        }
    }
    else
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DI();
        }else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DI();
        }else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_DI();
        }else if (pGPIOx == GPIOJ)
        {
            GPIOJ_PCLK_DI();
        }else if (pGPIOx == GPIOK)
        {
            GPIOK_PCLK_DI();
        }
    }
}

/*********************************************************************
 * @fn                    - GPIO_Init
 * @brief                 - Initialize the GPIO port and Pin.
 * @param[in] pGPIOHandle - Pointer to the handle structure.
 * @return                - none
 * @Note                  - none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

}

/*********************************************************************
 * @fn                - GPIO_DeInit
 * @brief             - Deinitialize the registers of the given GPIO peripheral.
 * @param[in] pGPIOx  - Base address of the GPIO peripheral.
 * @return            - none
 * @Note              - Sending the register back to its reset state, or reset value.
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}

/*********************************************************************
 * @fn                  - GPIO_ReadFromInputPin
 * @brief               - Read from input pin.
 * @param[in]           - Base address of the GPIO peripheral.
 * @param[in] PinNumber - Pin number.
 * @return              - The redden value must be either 0 or 1.
 * @Note                -
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    return 0;
}

/*********************************************************************
 * @fn                - GPIO_ReadFromInputPort
 * @brief             - Read from input port
 * @param[in]         - Base address of the GPIO port.
 * @return            - Content of the input data register.
 * @Note              - Port is 16 pins wide.
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    return 0;
}

/*********************************************************************
 * @fn                  - GPIO_WriteToOutputPin
 * @brief               - Write to output pin
 * @param[in]           - Base address of the GPIO peripheral.
 * @param[in] PinNumber - Pin number.
 * @param[in] Value     - Either 0 or 1, pin set or pin reset.
 * @return              - none
 * @Note                - none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{


}

/*********************************************************************
 * @fn              - GPIO_WriteToOutputPort
 * @brief           - Write to output port.
 * @param[in]       - Base address of the GPIO peripheral.
 * @param[in] Value - 16 bits value.
 * @return          - none
 * @Note            - There are 16 pins in a port
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

}

/*********************************************************************
 * @fn                  - GPIO_ToggleOutputPin
 * @brief               - Toggle pin
 * @param[in] pGPIOx    - Base address of the GPIO peripheral.
 * @param[in] PinNumber - Pin number.
 * @return              - none
 * @Note                - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*********************************************************************
 * @fn                - GPIO_IRQConfig
 * @brief             -
 * @param[in]         - IRQ number
 * @param[in]         - IRQ priority
 * @param[in]         - Enable or disable
 * @return            -
 * @Note              -
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

/*********************************************************************
 * @fn                - SPI_IRQPriorityConfig
 * @brief             -
 * @param[in]         -
 * @param[in]         -
 * @return            -
 * @Note              -
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{

}

/*********************************************************************
 * @fn                  - GPIO_IRQHandling
 * @brief               - IRQ Handler
 * @param[in] PinNumber - Pin number.
 * @return              -
 * @Note                - Function should know from which pin the interrupt is trigger
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{

}

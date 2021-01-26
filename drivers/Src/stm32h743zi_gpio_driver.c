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
 * @return            -  none
 * @Note              -  none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {

    }
    else
    {

    }
}

/*********************************************************************
 * @fn                - GPIO_Init
 * @brief             -
 * @param[in]         -
 * @return            -
 * @Note              -
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

}

/*********************************************************************
 * @fn                - GPIO_DeInit
 * @brief             -
 * @param[in]         -
 * @return            -
 * @Note              -
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}

/*********************************************************************
 * @fn                - GPIO_ReadFromInputPin
 * @brief             -
 * @param[in]         -
 * @return            -   0 or 1
 * @Note              -
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    return 0;
}

/*********************************************************************
 * @fn                - GPIO_ReadFromInputPort
 * @brief             -
 * @param[in]         -
 * @return            -
 * @Note              -
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    return 0;
}

/*********************************************************************
 * @fn                - GPIO_WriteToOutputPin
 * @brief             -
 * @param[in]         -
 * @param[in]         -
 * @return            -
 * @Note              -
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{


}

/*********************************************************************
 * @fn                - GPIO_WriteToOutputPort
 * @brief             -
 * @param[in]         -
 * @param[in]         -
 * @return            -
 * @Note              -
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

}

/*********************************************************************
 * @fn                - GPIO_ToggleOutputPin
 * @brief             -
 * @param[in]         -
 * @param[in]         -
 * @return            -
 * @Note              -
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*********************************************************************
 * @fn                - GPIO_IRQConfig
 * @brief             -
 * @param[in]         -
 * @param[in]         -
 * @return            -
 * @Note              -
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn                - GPIO_IRQHandling
 * @brief             -
 * @param[in]         -
 * @return            -
 * @Note              -
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{

}

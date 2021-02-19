/*
 * stm32h743zi_gpio_driver.c
 *
 *  Created on: Jan 25, 2021
 *      Author: Samuel
 */


#include "stm32h743zi_gpio_driver.h"

#define MODERX_MAX_BIT_FIELDS   2
#define MODERX_MAX_CFG_VALUE    0x03
#define GPIO_MODE_INPUT         0x00

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
 * @Note                  - Configure mode, speed, output type, pull-up,
 *                          pull-down resistor configuration, alternate
 *                          functionality and various other things.
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0; /*Temp register */
    uint8_t temp1, temp2;
    uint8_t cr_num;
    uint8_t cr_startbit;
    uint8_t portcode;

    /* 1. Configure the mode of gpio pin */

    /* is pin mode cfg part of normal pin configurations  */
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        /* --- Non interrupt mode ---*/

        /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
        /*GPIO_PinMode (defined by the user) will be left shifted to the proper MODERx where x is the pin number*/
        /*Note: Each pin takes 2 bit fields, that is why multiplied by 2*/
        temp = pGPIOHandle->pGPIOx->MODER;
        temp &= ~( MODERX_MAX_CFG_VALUE << (MODERX_MAX_BIT_FIELDS * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); /* clearing */
        temp |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (MODERX_MAX_BIT_FIELDS * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
        pGPIOHandle->pGPIOx->MODER = temp; /* setting */
    }
    else
    {
        /* --- Interrupt mode ---*/

        /* Configure IO Direction mode as input */
        temp = pGPIOHandle->pGPIOx->MODER;
        temp &= ~( MODERX_MAX_CFG_VALUE << (MODERX_MAX_BIT_FIELDS * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); /* clearing */
        temp |= ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode & GPIO_MODE_INPUT) << (MODERX_MAX_BIT_FIELDS * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER = temp; /* setting */

        /* Is pin mode configured as falling edge? */
        if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_FT )
        {
            /* I1. Configure the FTSR1 */
            EXTI->FTSR1 |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

            /* Clear the corresponding RTSR1 bit */
            EXTI->RTSR1 &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

        }else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RT )
        {
            /* Pin mode configured as rising edge */

            /* I1. Configure the RTSR1 */
            EXTI->RTSR1 |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

            /* Clear the corresponding FTSR1 bit */
            EXTI->FTSR1 &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

        }else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT )
        {
            /* Pin mode configured as rising edge/falling edge */

            /* I1. Configure both FTSR and RTSR */
            EXTI->FTSR1 |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->RTSR1 |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        /* I2. configure the GPIO port selection in SYSCFG_EXTICR */
        cr_num = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        cr_startbit = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[cr_num] = portcode << (cr_startbit * 4);

        /* I3 . enable the exti interrupt delivery using CPUIMR1 */
        EXTI->CPUIMR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

    /* 2. Configure the speed */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
    pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); /* clearing */
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    /* 3. Configure the pupd settings */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
    pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); /* clearing */
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    /* 4. Configure the optype */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
    pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); /* clearing */
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    /* 5. Configure the alt functionality */
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        /* Configure the alt function registers. */
        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); /* clearing */
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
    }
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
    if(pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }else if (pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }else if (pGPIOx == GPIOI)
    {
        GPIOI_REG_RESET();
    }else if (pGPIOx == GPIOJ)
    {
        GPIOJ_REG_RESET();
    }else if (pGPIOx == GPIOK)
    {
        GPIOK_REG_RESET();
    }
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
    uint8_t value;

    value = (uint8_t )((pGPIOx->IDR  >> PinNumber) & 0x00000001 );

    return value;
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
    uint16_t value;

    value = (uint16_t)pGPIOx->IDR;

    return value;
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
    if(Value == GPIO_PIN_SET)
    {
        //write 1 to the output data register at the bit field corresponding to the pin number
        pGPIOx->ODR |= ( 1 << PinNumber);
    }else
    {
        //write 0
        pGPIOx->ODR &= ~( 1 << PinNumber);
    }
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
    pGPIOx->ODR  = Value;
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
    pGPIOx->ODR  ^= ( 1 << PinNumber);
}

/*********************************************************************
 * @fn                - GPIO_IRQConfig
 * @brief             - Configuration of the NVIC registers of the ARM Cortex-M processor
 * @param[in]         - IRQ number
 * @param[in]         - IRQ priority
 * @param[in]         - Enable or disable
 * @return            -
 * @Note              - Processor specific configuration
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(IRQNumber <= 31) /* 0 to 31 */
        {
            /* Program ISER0 register */
            *NVIC_ISER0 |= ( 1 << IRQNumber );

        }else if(IRQNumber > 31 && IRQNumber < 64 ) /* 32 to 63 */
        {
            /* Program ISER1 register */
            *NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
        }else if(IRQNumber >= 64 && IRQNumber < 96 )  /* 64 to 95 */
        {
            /* Program ISER2 register */
            *NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
        }else if(IRQNumber >= 96 && IRQNumber < 128 )  /* 96 to 127 */
        {
            /* Program ISER3 register */
            *NVIC_ISER3 |= ( 1 << (IRQNumber % 96) );
        }
        else if(IRQNumber >= 128 && IRQNumber < 160 )  /* 128 to 159 */
        {
            /* Program ISER4 register */
            *NVIC_ISER4 |= ( 1 << (IRQNumber % 128) );
        }

    }else
    {
        if(IRQNumber <= 31) /* 0 to 31 */
        {
            /* Program ICER0 register */
            *NVIC_ICER0 |= ( 1 << IRQNumber );
        }else if(IRQNumber > 31 && IRQNumber < 64 ) /* 32 to 63 */
        {
            /* Program ICER1 register */
            *NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
        }else if(IRQNumber >= 64 && IRQNumber < 96 ) /* 64 to 95 */
        {
            /* Program ICER2 register */
            *NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
        }else if(IRQNumber >= 96 && IRQNumber < 128 )  /* 96 to 127 */
        {
            /* Program ICER3 register */
            *NVIC_ICER3 |= ( 1 << (IRQNumber % 96) );
        }
        else if(IRQNumber >= 128 && IRQNumber < 160 )  /* 128 to 159 */
        {
            /* Program ICER4 register */
            *NVIC_ICER4 |= ( 1 << (IRQNumber % 128) );
        }
    }
}

/*********************************************************************
 * @fn                - SPI_IRQPriorityConfig
 * @brief             - Configuration of the NVIC registers of the ARM Cortex-M processor
 * @param[in]         -
 * @param[in]         -
 * @return            -
 * @Note              -
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
    /* 1. Find out the IPR register */
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section  = IRQNumber %4 ;

    uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NVIC_PRIO_BITS_IMPLEMENTED) ;

    /* Jump by 4; 32bits register*/
    *( NVIC_IPR_BASEADDR + iprx ) |= ( IRQPriority << shift_amount );
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
    /* Clear the exti pr register corresponding to the pin number */
    if(EXTI->CPUPR1 & ( 1 << PinNumber))
    {
        /* Clear pending register bit */
        EXTI->CPUPR1 |= ( 1 << PinNumber);
    }
}

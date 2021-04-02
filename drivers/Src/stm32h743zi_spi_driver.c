/*
 * stm32h743zi_spi_driver.c
 *
 *  Created on: Mar 15, 2021
 *      Author: Samuel
 */

#include "stm32h743zi_spi_driver.h"

/*********************************************************************
 * @fn                - SPI_PeriClockControl
 * @brief             - This function enables or disables peripheral clock for the given SPI peripheral
 * @param[in] pSPIx   - Base address of the SPI peripheral
 * @param[in] EnorDi  - ENABLE or DISABLE macros
 * @return            - none
 * @Note              - none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

}

/*********************************************************************
 * @fn                   - SPI_Init
 * @brief                - Initialize the SPIx peripheral.
 * @param[in] pSPIHandle - Pointer to the handle structure.
 * @return               - none
 * @Note                 -
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

}


/*********************************************************************
 * @fn                - SPI_DeInit
 * @brief             - Deinitialize the registers of the given SPI peripheral.
 * @param[in] pSPIx   - Base address of the SPI peripheral.
 * @return            - none
 * @Note              -
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}

/*********************************************************************
 * @fn                  - SPI_SendData
 * @brief               -
 * @param[in] pSPIx     - Base address of the SPI peripheral.
 * @param[in] pTxBuffer - Tx Buffer
 * @param[in] Len       - Length
 * @return              -
 * @Note                - This is blocking call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{

}

/*********************************************************************
 * @fn                  - SPI_ReceiveData
 * @brief               -
 * @param[in] pSPIx     - Base address of the SPI peripheral.
 * @param[in] pRxBuffer - Rx Buffer
 * @param[in] Len       - Length
 * @return              -
 * @Note                -
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}

/*********************************************************************
 * @fn                  - SPI_IRQInterruptConfig
 * @brief               -
 * @param[in] IRQNumber - IRQ number
 * @param[in] EnorDi    - Enable or disable
 * @return              -
 * @Note                -
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

/*********************************************************************
 * @fn                    - SPI_IRQPriorityConfig
 * @brief                 -
 * @param[in] IRQNumber   -
 * @param[in] IRQPriority -
 * @return                -
 * @Note                  -
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{

}

/*********************************************************************
 * @fn                  - SPI_IRQHandling
 * @brief               -
 * @param[in] pHandle   - Pointer to the handle structure.
 * @return              -
 * @Note                -
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}




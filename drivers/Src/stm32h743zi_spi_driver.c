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
    if(EnorDi == ENABLE)
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }else if (pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }else if (pSPIx == SPI4)
        {
            SPI4_PCLK_EN();
        }else if (pSPIx == SPI5)
        {
            SPI5_PCLK_EN();
        }else if (pSPIx == SPI6)
        {
            SPI6_PCLK_EN();
        }
    }
    else
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }else if (pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }else if (pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }else if (pSPIx == SPI4)
        {
            SPI4_PCLK_DI();
        }else if (pSPIx == SPI5)
        {
            SPI5_PCLK_DI();
        }else if (pSPIx == SPI6)
        {
            SPI6_PCLK_DI();
        }
    }
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
    uint32_t spi_cfg1 = 0x00070007;     /* SPI configuration register 1 */
    uint32_t spi_cfg2 = 0x00;           /* SPI configuration register 2 */

    /* Peripheral clock enable */
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    /**** Configure the SPI_CFG1 register ****/

    /* Configure the number of bits in at single SPI data frame */
    spi_cfg1 |= pSPIHandle->SPIConfig.SPI_DSize << SPI_CFG1_DSIZE_B0;

    // Configure the spi serial clock speed (baud rate)
    spi_cfg1 |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CFG1_MBR_B0;

    pSPIHandle->pSPIx->SPI_CFG1 = spi_cfg1;

    /**** Configure the SPI_CFG2 register ****/

    /* Configure the device mode */
    spi_cfg2 |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CFG2_MASTER;

    /* Configure the serial protocol */
    spi_cfg2 |= pSPIHandle->SPIConfig.SPI_SerProtocol << SPI_CFG2_SP_B0;

    /* Configure the communication mode */
    spi_cfg2 |= pSPIHandle->SPIConfig.SPI_ComMode << SPI_CFG2_COMM_B0;

    /* Configure the DFF */
    spi_cfg2 |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CFG2_LSBFRST;

    /* Configure the CPHA */
    spi_cfg2 |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CFG2_CPHA;

    /* Configure the CPOL */
    spi_cfg2 |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CFG2_CPOL;

    /* Configure the SSM */
    spi_cfg2 |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CFG2_SSM;

    pSPIHandle->pSPIx->SPI_CFG2 = spi_cfg2;
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
    /* TODO */
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




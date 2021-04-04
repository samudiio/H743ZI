/*
 * stm32h743zi_spi_driver.h
 *
 *  Created on: Mar 15, 2021
 *      Author: Samuel
 */

#ifndef INC_STM32H743ZI_SPI_DRIVER_H_
#define INC_STM32H743ZI_SPI_DRIVER_H_

#include "stm32h743zi.h"

/**********************************************
 * SPI_CFG1
 ***********************************************/
/*
 * @SPI_DSize: Number of bits in at single SPI data frame
 */
typedef enum _SPI_DSIZE_ten
{
    SPI_DSIZE_RESERVED0 = 0u,
    SPI_DSIZE_RESERVED1 = 1u,
    SPI_DSIZE_RESERVED2 = 2u,
    SPI_DSIZE_4BITS     = 3u,
    SPI_DSIZE_5BITS     = 4u,
    SPI_DSIZE_6BITS     = 5u,
    SPI_DSIZE_7BITS     = 6u,
    SPI_DSIZE_8BITS     = 7u,
    SPI_DSIZE_9BITS     = 8u,
    SPI_DSIZE_10BITS    = 9u,
    SPI_DSIZE_11BITS    = 10u,
    SPI_DSIZE_12BITS    = 11u,
    SPI_DSIZE_13BITS    = 12u,
    SPI_DSIZE_14BITS    = 13u,
    SPI_DSIZE_15BITS    = 14u,
    SPI_DSIZE_16BITS    = 15u,
    SPI_DSIZE_17BITS    = 16u,
    SPI_DSIZE_18BITS    = 17u,
    SPI_DSIZE_19BITS    = 18u,
    SPI_DSIZE_20BITS    = 19u,
    SPI_DSIZE_21BITS    = 20u,
    SPI_DSIZE_22BITS    = 21u,
    SPI_DSIZE_23BITS    = 22u,
    SPI_DSIZE_24BITS    = 23u,
    SPI_DSIZE_25BITS    = 24u,
    SPI_DSIZE_26BITS    = 25u,
    SPI_DSIZE_27BITS    = 26u,
    SPI_DSIZE_28BITS    = 27u,
    SPI_DSIZE_29BITS    = 28u,
    SPI_DSIZE_30BITS    = 29u,
    SPI_DSIZE_31BITS    = 30u,
    SPI_DSIZE_32BITS    = 31u
}SPI_DSIZE_t;

/*
 * @SPI_MBR: Master baud rate
 */
typedef enum _SPI_MBR_ten
{
    SPI_MBR_DIV2   = 0u,
    SPI_MBR_DIV4   = 1u,
    SPI_MBR_DIV8   = 2u,
    SPI_MBR_DIV16  = 3u,
    SPI_MBR_DIV32  = 4u,
    SPI_MBR_DIV64  = 5u,
    SPI_MBR_DIV128 = 6u,
    SPI_MBR_DIV256 = 7u
}SPI_MBR_t;

/**********************************************
 * SPI_CFG2
 ***********************************************/
/*
 * @SPI_COMM: Communication mode
 */
typedef enum _SPI_COMM_ten
{
    SPI_COMM_FD         = 0u,
    SPI_COMM_SIMPLEX_TX = 1u,
    SPI_COMM_SIMPLEX_RX = 2u,
    SPI_COMM_HD         = 3u
}SPI_COMM_t;

/*
 * @SPI_SP: Serial protocol
 */
typedef enum _SPI_SP_ten
{
    SPI_SP_MOTOROLA        = 0u,
    SPI_SP_TI              = 1u,
    SPI_SP_RESERVED0       = 2u,
    SPI_SP_RESERVED1       = 3u,
    SPI_SP_RESERVED2       = 4u,
    SPI_SP_RESERVED3       = 5u,
    SPI_SP_RESERVED4       = 6u,
    SPI_SP_RESERVED5       = 7u
}SPI_SP_t;

/*
 * @SPI_Master: SPI master
 */
typedef enum _SPI_Master_ten
{
    SPI_SLAVE  = 0u,
    SPI_MASTER = 1u
}SPI_Master_t;

/*
 * @SPI_DFF: Data Frame Format
 */
typedef enum _SPI_DFF_ten
{
    SPI_DFF_MSB_TX_FRST = 0u,
    SPI_DFF_LSB_TX_FRST = 1u
}SPI_DFF_t;

/*
 * @SPI_CPHA: Clock phase
 */
typedef enum _SPI_CPHA_ten
{
    SPI_CPHA_LOW  = 0u,
    SPI_CPHA_HIGH = 1u
}SPI_CPHA_t;

/*
 * @SPI_CPOL: Clock polarity
 */
typedef enum _SPI_CPOL_ten
{
    SPI_CPOL_LOW  = 0u,
    SPI_CPOL_HIGH = 1u
}SPI_CPOL_t;

/*
 * @SPI_SSM: Software management of SS signal input
 */
typedef enum _SPI_SSM_ten
{
    SPI_SSM_DI = 0u,
    SPI_SSM_EN = 1u
}SPI_SSM_t;

/******************************************************************************************
 * Handle and Configuration structures
 ******************************************************************************************/

/*
 * This is a Configuration structure for SPIx peripheral
 */
typedef struct _SPI_Config_tst
{
    SPI_DSIZE_t     SPI_DSize;
    SPI_MBR_t       SPI_SclkSpeed;
    SPI_Master_t    SPI_DeviceMode;
    SPI_SP_t        SPI_SerProtocol;
    SPI_COMM_t      SPI_ComMode;
    SPI_DFF_t       SPI_DFF;
    SPI_CPHA_t      SPI_CPHA ;
    SPI_CPOL_t      SPI_CPOL;
    SPI_SSM_t       SPI_SSM;
}SPI_Config_t;

/*
 * This is a Handle structure for SPIx peripheral
 */
typedef struct _SPI_Handle_tst
{
    SPI_RegDef_t *pSPIx;    /* This holds the base address of the SPIx(x:0...6) peripheral */
    SPI_Config_t SPIConfig; /* This holds SPIx configuration settings */
}SPI_Handle_t;

/******************************************************************************************
 *                              APIs supported by this driver
 *       For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */

#endif /* INC_STM32H743ZI_SPI_DRIVER_H_ */

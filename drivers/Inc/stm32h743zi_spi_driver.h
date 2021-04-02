/*
 * stm32h743zi_spi_driver.h
 *
 *  Created on: Mar 15, 2021
 *      Author: Samuel
 */

#ifndef INC_STM32H743ZI_SPI_DRIVER_H_
#define INC_STM32H743ZI_SPI_DRIVER_H_

#include "stm32h743zi.h"

/*
 * This is a Configuration structure for SPIx peripheral
 */
typedef struct
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPHA ;
    uint8_t SPI_CPOL;
    uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * This is a Handle structure for SPIx peripheral
 */
typedef struct
{
    SPI_RegDef_t *pSPIx;    /* This holds the base address of the SPIx(x:0...6) peripheral */
    SPI_Config_t SPIConfig; /* This holds SPIx configuration settings */
}SPI_Handle_t;

#endif /* INC_STM32H743ZI_SPI_DRIVER_H_ */

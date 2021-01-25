/*
 * stm32h743zi.h
 *
 *  Created on: Jan 19, 2021
 *      Author: Samuel
 *      This header file describes the MCU STM32H743ZI
 */

#ifndef INC_STM32H743ZI_H_
#define INC_STM32H743ZI_H_


/**********************************************
 * Base addresses of Flash and SRAM memories
 ***********************************************/
#define FLASH_BASEADDR          0x08000000U

/* D1 domain, AXI SRAM */
#define SRAM_BASEADDR           0x24000000U

/* D2 domain, AHB SRAM */
#define SRAM1_BASEADDR          0x30000000U
#define SRAM2_BASEADDR          0x30020000U
#define SRAM3_BASEADDR          0x30040000U

/* D3 domain, AHB SRAM */
#define SRAM4_BASEADDR          0x30080000U
#define BACKUP_SRAM_BASEADDR    0x38800000U

/* The TCM SRAMs are dedicated to the Cortex®-M7 */
#define DTCM_RAM_BASEADDR       0x20000000U
#define ITCM_RAM_BASEADDR       0x00000000U

/* System memory*/
#define ROM_BASEADDR            0x1FF00000U

/**********************************************
 * AHBx and APBx Bus Peripheral base addresses
 ***********************************************/

#define PERIPH_BASE             0x40000000U

#define APB1PERIPH_BASE         PERIPH_BASE
#define APB2PERIPH_BASE         0x40010000U
#define AHB1PERIPH_BASE         0x40020000U
#define AHB2PERIPH_BASE         0x48020000U
#define APB3PERIPH_BASE         0x48023000U
#define AHB3PERIPH_BASE         0x51000000U
#define APB4PERIPH_BASE         0x58000000U
#define AHB4PERIPH_BASE         0x58020000U

#endif /* INC_STM32H743ZI_H_ */

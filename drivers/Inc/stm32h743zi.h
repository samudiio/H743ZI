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

/**********************************************
 * Base addresses of peripherals which are hanging on AHB4 bus
 * TODO : Complete for all other peripherals
 ***********************************************/

#define GPIOA_BASEADDR          (AHB4PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR          (AHB4PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR          (AHB4PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR          (AHB4PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR          (AHB4PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR          (AHB4PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR          (AHB4PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR          (AHB4PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR          (AHB4PERIPH_BASE + 0x2000)
#define GPIOJ_BASEADDR          (AHB4PERIPH_BASE + 0x2400)
#define GPIOK_BASEADDR          (AHB4PERIPH_BASE + 0x2800)

#define RCC_BASEADDR            (AHB4PERIPH_BASE + 0x4400)

/**********************************************
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 ***********************************************/

#define I2C1_BASEADDR           (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR           (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR           (APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASEADDR           (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR           (APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR         (APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR         (APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR          (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR          (APB1PERIPH_BASE + 0x5000)

/**********************************************
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 ***********************************************/

#define SPI1_BASEADDR           (APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR         (APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR         (APB2PERIPH_BASE + 0x1400)

/**********************************************
 * Base addresses of peripherals which are hanging on APB4 bus
 * TODO : Complete for all other peripherals
 ***********************************************/

#define EXTI_BASEADDR           (APB4PERIPH_BASE + 0x0000)
#define SYSCFG_BASEADDR         (APB4PERIPH_BASE + 0x0400)

/**********************************************
 * Peripheral register definition structures
 ***********************************************/

/*
 * Peripheral register definition structure for GPIOs
 */
typedef struct
{
     __v uint32_t MODER;                /*GPIO port mode register,                              Address offset: 0x00*/
     __v uint32_t OTYPER;               /*GPIO port output type register,                       Address offset: 0x04*/
     __v uint32_t OSPEEDR;              /*GPIO port output speed register,                      Address offset: 0x08*/
     __v uint32_t PUPDR;                /*GPIO port pull-up/pull-down register,                 Address offset: 0x0C*/
     __v uint32_t IDR;                  /*GPIO port input data register,                        Address offset: 0x10*/
     __v uint32_t ODR;                  /*GPIO port output data register,                       Address offset: 0x14*/
     __v uint32_t BSRR;                 /*GPIO port bit set/reset register,                     Address offset: 0x18*/
     __v uint32_t LCKR;                 /*GPIO port configuration lock register,                Address offset: 0x1C*/
     __v uint32_t AFRL;                 /*GPIO alternate function low register,                 Address offset: 0x20*/
     __v uint32_t AFRH;                 /*GPIO alternate function high register,                Address offset: 0x24*/
}GPIO_RegDef_t;

/**********************************************
 * Peripheral definitions (Peripheral base address typecasted to xxx_RegDef_t)
 ***********************************************/

#define GPIOA                   ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB                   ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC                   ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD                   ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE                   ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF                   ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG                   ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH                   ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI                   ((GPIO_RegDef_t*) GPIOI_BASEADDR)
#define GPIOJ                   ((GPIO_RegDef_t*) GPIOJ_BASEADDR)
#define GPIOK                   ((GPIO_RegDef_t*) GPIOK_BASEADDR)

#endif /* INC_STM32H743ZI_H_ */

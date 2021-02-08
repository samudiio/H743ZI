/*
 * stm32h743zi.h
 *
 *  Created on: Jan 19, 2021
 *      Author: Samuel
 *      This header file describes the MCU STM32H743ZI
 */

#ifndef INC_STM32H743ZI_H_
#define INC_STM32H743ZI_H_

#include <stdint.h>

#define __vo volatile

/********************************** START:Processor Specific Details **********************************/

/**********************************************
 * ARM Cortex®-M7 Processor NVIC ISERx register Addresses
 ***********************************************/

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10C )
#define NVIC_ISER4          ( (__vo uint32_t*)0xE000E110 )
#define NVIC_ISER5          ( (__vo uint32_t*)0xE000E114 )
#define NVIC_ISER6          ( (__vo uint32_t*)0xE000E118 )
#define NVIC_ISER7          ( (__vo uint32_t*)0xE000E11C )

/**********************************************
 * ARM Cortex®-M7 Processor NVIC ICERx register Addresses
 ***********************************************/

#define NVIC_ICER0          ( (__vo uint32_t*)0xE000E180 )
#define NVIC_ICER1          ( (__vo uint32_t*)0xE000E184 )
#define NVIC_ICER2          ( (__vo uint32_t*)0xE000E188 )
#define NVIC_ICER3          ( (__vo uint32_t*)0xE000E18C )
#define NVIC_ICER4          ( (__vo uint32_t*)0xE000E190 )
#define NVIC_ICER5          ( (__vo uint32_t*)0xE000E194 )
#define NVIC_ICER6          ( (__vo uint32_t*)0xE000E198 )
#define NVIC_ICER7          ( (__vo uint32_t*)0xE000E19C )

/**********************************************
 * ARM Cortex®-M7 Processor Priority Register Base Address
 ***********************************************/
#define NVIC_IPR_BASEADDR   ( (__vo uint32_t*)0xE000E400 )

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NVIC_PRIO_BITS_IMPLEMENTED  4       /*!< CM7 uses 4 Bits for the Priority Levels       */

/********************************** START: MCU Specific Details **********************************/

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
     __vo uint32_t MODER;                /* GPIO port mode register,                                 Address offset: 0x00 */
     __vo uint32_t OTYPER;               /* GPIO port output type register,                          Address offset: 0x04 */
     __vo uint32_t OSPEEDR;              /* GPIO port output speed register,                         Address offset: 0x08 */
     __vo uint32_t PUPDR;                /* GPIO port pull-up/pull-down register,                    Address offset: 0x0C */
     __vo uint32_t IDR;                  /* GPIO port input data register,                           Address offset: 0x10 */
     __vo uint32_t ODR;                  /* GPIO port output data register,                          Address offset: 0x14 */
     __vo uint32_t BSRR;                 /* GPIO port bit set/reset register,                        Address offset: 0x18 */
     __vo uint32_t LCKR;                 /* GPIO port configuration lock register,                   Address offset: 0x1C */
     __vo uint32_t AFR[2];               /* AFR[0]: GPIO alternate function low register,            Address offset: 0x20 */
                                         /* AFR[1]: GPIO alternate function high register,           Address offset: 0x24 */
}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC. (Rev Y)
 */
typedef struct
{
    __vo uint32_t CR;                    /* RCC Source Control Register                              Address offset: 0x00 */
    __vo uint32_t ICSCR;                 /* RCC Internal Clock Source Calibration Register           Address offset: 0x04 */
    __vo uint32_t CRRCR;                 /* RCC Clock Recovery RC Register                           Address offset: 0x08 */
        uint32_t RESERVED0;              /* Reserved                                                 Address offset: 0x0C */
    __vo uint32_t CFGR;                  /* RCC Clock Configuration Register                         Address offset: 0x10 */
    __vo uint32_t RESERVED1;             /* Reserved                                                 Address offset: 0x14 */
    __vo uint32_t D1CFGR;                /* RCC Domain 1 Clock Configuration Register                Address offset: 0x18 */
    __vo uint32_t D2CFGR;                /* RCC Domain 2 Clock Configuration Register                Address offset: 0x1C */
    __vo uint32_t D3CFGR;                /* RCC Domain 3 Clock Configuration Register                Address offset: 0x20 */
        uint32_t RESERVED2;              /* Reserved                                                 Address offset: 0x24 */
    __vo uint32_t PLLCKSELR;             /* RCC PLLs Clock Source Selection Register                 Address offset: 0x28 */
    __vo uint32_t PLLCFGR;               /* RCC PLLs Configuration Register                          Address offset: 0x2C */
    __vo uint32_t PLL1DIVR;              /* RCC PLL1 Dividers Configuration Register                 Address offset: 0x30 */
    __vo uint32_t PLL1FRACR;             /* RCC PLL1 Fractional Divider Register                     Address offset: 0x34 */
    __vo uint32_t PLL2DIVR;              /* RCC PLL2 Dividers Configuration Register                 Address offset: 0x38 */
    __vo uint32_t PLL2FRACR;             /* RCC PLL2 Fractional Divider Register                     Address offset: 0x3C */
    __vo uint32_t PLL3DIVR;              /* RCC PLL3 Dividers Configuration Register                 Address offset: 0x40 */
    __vo uint32_t PLL3FRACR;             /* RCC PLL3 Fractional Divider Register                     Address offset: 0x44 */
        uint32_t RESERVED3;              /* Reserved                                                 Address offset: 0x48 */
    __vo uint32_t D1CCIPR;               /* RCC Domain 1 Kernel Clock Configuration Register         Address offset: 0x4C */
    __vo uint32_t D2CCIP1R;              /* RCC Domain 2 Kernel Clock Configuration Register         Address offset: 0x50 */
    __vo uint32_t D2CCIP2R;              /* RCC Domain 2 Kernel Clock Configuration Register         Address offset: 0x54 */
    __vo uint32_t D3CCIPR;               /* RCC Domain 3 Kernel Clock Configuration Register         Address offset: 0x58 */
        uint32_t RESERVED4;              /* Reserved                                                 Address offset: 0x5C */
    __vo uint32_t CIER;                  /* RCC Clock Source Interrupt Enable Register               Address offset: 0x60 */
    __vo uint32_t CIFR;                  /* RCC Clock Source Interrupt Flag Register                 Address offset: 0x64 */
    __vo uint32_t CICR;                  /* RCC Clock Source Interrupt Clear Register                Address offset: 0x68 */
        uint32_t RESERVED5;              /* Reserved                                                 Address offset: 0x6C */
    __vo uint32_t BDCR;                  /* RCC Backup Domain Control Register                       Address offset: 0x70 */
    __vo uint32_t CSR;                   /* RCC Clock Control and Status Register                    Address offset: 0x74 */
        uint32_t RESERVED6;              /* Reserved                                                 Address offset: 0x78 */
    __vo uint32_t AHB3RSTR;              /* RCC AHB3 Reset Register                                  Address offset: 0x7C */
    __vo uint32_t AHB1RSTR;              /* RCC AHB1 Peripheral Reset Register                       Address offset: 0x80 */
    __vo uint32_t AHB2RSTR;              /* RCC AHB2 Peripheral Reset Register                       Address offset: 0x84 */
    __vo uint32_t AHB4RSTR;              /* RCC AHB4 Peripheral Reset Register                       Address offset: 0x88 */
    __vo uint32_t APB3RSTR;              /* RCC APB3 Peripheral Reset Register                       Address offset: 0x8C */
    __vo uint32_t APB1LRSTR;             /* RCC APB1 Peripheral Reset Register                       Address offset: 0x90 */
    __vo uint32_t APB1HRSTR;             /* RCC APB1 Peripheral Reset Register                       Address offset: 0x94 */
    __vo uint32_t APB2RSTR;              /* RCC APB2 Peripheral Reset Register                       Address offset: 0x98 */
    __vo uint32_t APB4RSTR;              /* RCC APB4 Peripheral Reset Register                       Address offset: 0x9C */
    __vo uint32_t GCR;                   /* RCC Global Control Register                              Address offset: 0xA0 */
        uint32_t RESERVED7;              /* Reserved                                                 Address offset: 0xA4 */
    __vo uint32_t D3AMR;                 /* RCC D3 Autonomous mode Register                          Address offset: 0xA8 */
    __vo uint32_t RESERVED8[9];          /* Reserved, 0xAC-0xCC                                      Address offset: 0xAC */
    __vo uint32_t RSR;                   /* RCC Reset Status Register                                Address offset: 0xD0 */
    __vo uint32_t AHB3ENR;               /* RCC AHB3 Clock Register                                  Address offset: 0xD4 */
    __vo uint32_t AHB1ENR;               /* RCC AHB1 Clock Register                                  Address offset: 0xD8 */
    __vo uint32_t AHB2ENR;               /* RCC AHB2 Clock Register                                  Address offset: 0xDC */
    __vo uint32_t AHB4ENR;               /* RCC AHB4 Clock Register                                  Address offset: 0xE0 */
    __vo uint32_t APB3ENR;               /* RCC APB3 Clock Register                                  Address offset: 0xE4 */
    __vo uint32_t APB1LENR;              /* RCC APB1 Clock Register                                  Address offset: 0xE8 */
    __vo uint32_t APB1HENR;              /* RCC APB1 Clock Register                                  Address offset: 0xEC */
    __vo uint32_t APB2ENR;               /* RCC APB2 Clock Register                                  Address offset: 0xF0 */
    __vo uint32_t APB4ENR;               /* RCC APB4 Clock Register                                  Address offset: 0xF4 */
        uint32_t RESERVED9;              /* Reserved                                                 Address offset: 0xF8 */
    __vo uint32_t AHB3LPENR;             /* RCC AHB3 Sleep Clock Register                            Address offset: 0xFC */
    __vo uint32_t AHB1LPENR;             /* RCC AHB1 Sleep Clock Register                            Address offset: 0x100 */
    __vo uint32_t AHB2LPENR;             /* RCC AHB2 Sleep Clock Register                            Address offset: 0x104 */
    __vo uint32_t AHB4LPENR;             /* RCC AHB4 Sleep Clock Register                            Address offset: 0x108 */
    __vo uint32_t APB3LPENR;             /* RCC APB3 Sleep Clock Register                            Address offset: 0x10C */
    __vo uint32_t APB1LLPENR;            /* RCC APB1 Low Sleep Clock Register                        Address offset: 0x110 */
    __vo uint32_t APB1HLPENR;            /* RCC APB1 High Sleep Clock Register                       Address offset: 0x114 */
    __vo uint32_t APB2LPENR;             /* RCC APB2 Sleep Clock Register                            Address offset: 0x118 */
    __vo uint32_t APB4LPENR;             /* RCC APB4 Sleep Clock Register                            Address offset: 0x11C */
        uint32_t RESERVED10[5];          /* Reserved, 0x120-0x130                                    Address offset: 0x120 */
    __vo uint32_t C1_AHB3ENR;            /* TODO                                                     Address offset: 0x134 */
    __vo uint32_t C1_AHB1ENR;            /* TODO                                                     Address offset: 0x138 */
    __vo uint32_t C1_AHB2ENR;            /* TODO                                                     Address offset: 0x13C */
    __vo uint32_t C1_AHB4ENR;            /* TODO                                                     Address offset: 0x140 */
    __vo uint32_t C1_APB3ENR;            /* TODO                                                     Address offset: 0x144 */
    __vo uint32_t C1_APB1LENR;           /* TODO                                                     Address offset: 0x148 */
    __vo uint32_t C1_APB1HENR;           /* TODO                                                     Address offset: 0x14C */
    __vo uint32_t C1_APB2ENR;            /* TODO                                                     Address offset: 0x150 */
    __vo uint32_t C1_APB4ENR;            /* TODO                                                     Address offset: 0x154 */
        uint32_t RESERVED11;             /* Reserved                                                 Address offset: 0x158 */
    __vo uint32_t C1_AHB3LPENR;          /* TODO                                                     Address offset: 0x15C */
    __vo uint32_t C1_AHB1LPENR;          /* TODO                                                     Address offset: 0x160 */
    __vo uint32_t C1_AHB2LPENR;          /* TODO                                                     Address offset: 0x164 */
    __vo uint32_t C1_AHB4LPENR;          /* TODO                                                     Address offset: 0x168 */
    __vo uint32_t C1_APB3LPENR;          /* TODO                                                     Address offset: 0x16C */
    __vo uint32_t C1_APB1LLPENR;         /* TODO                                                     Address offset: 0x170 */
    __vo uint32_t C1_APB1HLPENR;         /* TODO                                                     Address offset: 0x174 */
    __vo uint32_t C1_APB2LPENR;          /* TODO                                                     Address offset: 0x178 */
    __vo uint32_t C1_APB4LPENR;          /* TODO                                                     Address offset: 0x17C */
        uint32_t RESERVED12[32];         /* Reserved, 0x180-0x1FC                                    Address offset: 0x180 */
}RCC_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG.
 */
typedef struct
{
         uint32_t RESERVED0;             /* Reserved                                                  Address offset: 0x00 */
     __vo uint32_t PMCR;                 /* SYSCFG peripheral mode configuration register             Address offset: 0x04 */
     __vo uint32_t EXTICR[4];            /* EXTICR[0]: SYSCFG ext interrupt configuration register 1  Address offset: 0x08 */
                                         /* EXTICR[1]: SYSCFG ext interrupt configuration register 2  Address offset: 0x0C */
                                         /* EXTICR[2]: SYSCFG ext interrupt configuration register 3  Address offset: 0x10 */
                                         /* EXTICR[3]: SYSCFG ext interrupt configuration register 4  Address offset: 0x14 */
     __vo uint32_t CFGR;                 /* SYSCFG configuration register                             Address offset: 0x18 */
         uint32_t RESERVED1;             /* Reserved                                                  Address offset: 0x1C */
     __vo uint32_t CCSR;                 /* SYSCFG compensation cell control/status register          Address offset: 0x20 */
     __vo uint32_t CCVR;                 /* SYSCFG compensation cell value register                   Address offset: 0x24 */
     __vo uint32_t CCCR;                 /* SYSCFG compensation cell code register                    Address offset: 0x28 */
     __vo uint32_t PWRCR;                /* SYSCFG power control register                             Address offset: 0x2C */
         uint32_t RESERVED2[61];         /* Reserved, 0x30-0x120                                      Address offset: 0x30 */
     __vo uint32_t PKGR;                 /* SYSCFG SYSCFG package register                            Address offset: 0x124 */
         uint32_t RESERVED3[118];        /* Reserved, 0x128-0x2FC                                     Address offset: 0x128 */
     __vo uint32_t UR0;                  /* SYSCFG user register 0                                    Address offset: 0x300 */
         uint32_t RESERVED4;             /* Reserved                                                  Address offset: 0x304 */
     __vo uint32_t UR2;                  /* SYSCFG user register 2                                    Address offset: 0x308 */
     __vo uint32_t UR3;                  /* SYSCFG user register 3                                    Address offset: 0x30C */
     __vo uint32_t UR4;                  /* SYSCFG user register 4                                    Address offset: 0x310 */
     __vo uint32_t UR5;                  /* SYSCFG user register 5                                    Address offset: 0x314 */
     __vo uint32_t UR6;                  /* SYSCFG user register 6                                    Address offset: 0x318 */
     __vo uint32_t UR7;                  /* SYSCFG user register 7                                    Address offset: 0x31C */
     __vo uint32_t UR8;                  /* SYSCFG user register 8                                    Address offset: 0x320 */
     __vo uint32_t UR9;                  /* SYSCFG user register 9                                    Address offset: 0x324 */
     __vo uint32_t UR10;                 /* SYSCFG user register 10                                   Address offset: 0x328 */
     __vo uint32_t UR11;                 /* SYSCFG user register 11                                   Address offset: 0x32C */
     __vo uint32_t UR12;                 /* SYSCFG user register 12                                   Address offset: 0x330 */
     __vo uint32_t UR13;                 /* SYSCFG user register 13                                   Address offset: 0x334 */
     __vo uint32_t UR14;                 /* SYSCFG user register 14                                   Address offset: 0x338 */
     __vo uint32_t UR15;                 /* SYSCFG user register 15                                   Address offset: 0x33C */
     __vo uint32_t UR16;                 /* SYSCFG user register 16                                   Address offset: 0x340 */
     __vo uint32_t UR17;                 /* SYSCFG user register 17                                   Address offset: 0x344 */
}SYSCFG_RegDef_t;

/*
 * Peripheral register definition structure for EXTI.
 */
typedef struct
{
    __vo uint32_t RTSR1;                 /* EXTI rising trigger selection register                   Address offset: 0x00 */
    __vo uint32_t FTSR1;                 /* EXTI falling trigger selection register                  Address offset: 0x04 */
    __vo uint32_t SWIER1;                /* EXTI software interrupt event register                   Address offset: 0x08 */
    __vo uint32_t D3PMR1;                /* EXTI D3 pending mask register                            Address offset: 0x0C */
    __vo uint32_t D3PCR1L;               /* EXTI D3 pending clear selection register low             Address offset: 0x10 */
    __vo uint32_t D3PCR1H;               /* EXTI D3 pending clear selection register high            Address offset: 0x14 */
    __vo uint32_t RTSR2;                 /* EXTI rising trigger selection register                   Address offset: 0x20 */
    __vo uint32_t FTSR2;                 /* EXTI falling trigger selection register                  Address offset: 0x24 */
    __vo uint32_t SWIER2;                /* EXTI software interrupt event register                   Address offset: 0x28 */
    __vo uint32_t D3PMR2;                /* EXTI D3 pending mask register                            Address offset: 0x2C */
    __vo uint32_t D3PCR2L;               /* EXTI D3 pending clear selection register low             Address offset: 0x30 */
    __vo uint32_t D3PCR2H;               /* EXTI D3 pending clear selection register high            Address offset: 0x34 */
    __vo uint32_t RTSR3;                 /* EXTI rising trigger selection register                   Address offset: 0x40 */
    __vo uint32_t FTSR3;                 /* EXTI falling trigger selection register                  Address offset: 0x44 */
    __vo uint32_t SWIER3;                /* EXTI software interrupt event register                   Address offset: 0x48 */
    __vo uint32_t D3PMR3;                /* EXTI D3 pending mask register                            Address offset: 0x4C */
    __vo uint32_t D3PCR3L;               /* EXTI D3 pending clear selection register low             Address offset: 0x50 */
    __vo uint32_t D3PCR3H;               /* EXTI D3 pending clear selection register high            Address offset: 0x54 */
        uint32_t RESERVED0[10];          /* Reserved, 0x58-0x7C                                      Address offset: 0x58 */
    __vo uint32_t CPUIMR1;               /* EXTI interrupt mask register                             Address offset: 0x80 */
    __vo uint32_t CPUEMR1;               /* EXTI event mask register                                 Address offset: 0x84 */
    __vo uint32_t CPUPR1;                /* EXTI pending register                                    Address offset: 0x88 */
    __vo uint32_t CPUIMR2;               /* EXTI interrupt mask register                             Address offset: 0x90 */
    __vo uint32_t CPUEMR2;               /* EXTI event mask register                                 Address offset: 0x94 */
    __vo uint32_t CPUPR2;                /* EXTI pending register                                    Address offset: 0x98 */
    __vo uint32_t CPUIMR3;               /* EXTI interrupt mask register                             Address offset: 0xA0 */
    __vo uint32_t CPUEMR3;               /* EXTI event mask register                                 Address offset: 0xA4 */
    __vo uint32_t CPUPR3;                /* EXTI pending register                                    Address offset: 0xA8 */
        uint32_t RESERVED1[5];           /* Reserved, 0xAC-0xBC                                      Address offset: 0xAC */
}EXTI_RegDef_t;

/**********************************************
 * Peripheral definitions (Peripheral base address typecasted to xxx_RegDef_t)
 ***********************************************/

#define GPIOA                           ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB                           ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC                           ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD                           ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE                           ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF                           ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG                           ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH                           ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI                           ((GPIO_RegDef_t*) GPIOI_BASEADDR)
#define GPIOJ                           ((GPIO_RegDef_t*) GPIOJ_BASEADDR)
#define GPIOK                           ((GPIO_RegDef_t*) GPIOK_BASEADDR)

#define RCC                             ((RCC_RegDef_t*) RCC_BASEADDR)

#define SYSCFG                          ((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)        /* SYSCFG Base address type casted */

#define EXTI                            ((EXTI_RegDef_t*) EXTI_BASEADDR)

/**********************************************
 * Clock Enable Macros for GPIOx peripherals
 ***********************************************/

#define GPIOA_PCLK_EN()                 ( RCC->AHB4ENR |= (1 << 0) )                /* 1: GPIOA peripheral clock enabled */
#define GPIOB_PCLK_EN()                 ( RCC->AHB4ENR |= (1 << 1) )                /* 1: GPIOB peripheral clock enabled */
#define GPIOC_PCLK_EN()                 ( RCC->AHB4ENR |= (1 << 2) )                /* 1: GPIOC peripheral clock enabled */
#define GPIOD_PCLK_EN()                 ( RCC->AHB4ENR |= (1 << 3) )                /* 1: GPIOD peripheral clock enabled */
#define GPIOE_PCLK_EN()                 ( RCC->AHB4ENR |= (1 << 4) )                /* 1: GPIOE peripheral clock enabled */
#define GPIOF_PCLK_EN()                 ( RCC->AHB4ENR |= (1 << 5) )                /* 1: GPIOF peripheral clock enabled */
#define GPIOG_PCLK_EN()                 ( RCC->AHB4ENR |= (1 << 6) )                /* 1: GPIOG peripheral clock enabled */
#define GPIOH_PCLK_EN()                 ( RCC->AHB4ENR |= (1 << 7) )                /* 1: GPIOH peripheral clock enabled */
#define GPIOI_PCLK_EN()                 ( RCC->AHB4ENR |= (1 << 8) )                /* 1: GPIOI peripheral clock enabled */
#define GPIOJ_PCLK_EN()                 ( RCC->AHB4ENR |= (1 << 9) )                /* 1: GPIOJ peripheral clock enabled */
#define GPIOK_PCLK_EN()                 ( RCC->AHB4ENR |= (1 << 10) )               /* 1: GPIOK peripheral clock enabled */

/**********************************************
 * Clock Enable Macros for I2Cx peripherals
 ***********************************************/

#define I2C1_PCLK_EN()                  ( RCC->APB1LENR |= (1 << 21) )              /* 1: I2C1 peripheral clocks enabled */
#define I2C2_PCLK_EN()                  ( RCC->APB1LENR |= (1 << 22) )              /* 1: I2C2 peripheral clocks enabled */
#define I2C3_PCLK_EN()                  ( RCC->APB1LENR |= (1 << 23) )              /* 1: I2C2 peripheral clocks enabled */

/**********************************************
 * Clock Enable Macros for SPIx peripherals
 ***********************************************/

#define SPI2_PCLK_EN()                  ( RCC->APB1LENR |= (1 << 14) )              /* 1: SPI2 peripheral clocks enabled */
#define SPI3_PCLK_EN()                  ( RCC->APB1LENR |= (1 << 15) )              /* 1: SPI3 peripheral clocks enabled */

/**********************************************
 * Clock Enable Macros for USARTx peripherals
 ***********************************************/

#define USART2_PCLK_EN()                ( RCC->APB1LENR |= (1 << 17) )              /* 1: USART2 peripheral clocks enabled */
#define USART3_PCLK_EN()                ( RCC->APB1LENR |= (1 << 18) )              /* 1: USART3 peripheral clocks enabled */
#define UART4_PCLK_EN()                 ( RCC->APB1LENR |= (1 << 19) )              /* 1: UART4 peripheral clocks enabled */
#define UART5_PCLK_EN()                 ( RCC->APB1LENR |= (1 << 20) )              /* 1: UART5 peripheral clocks enabled */

/**********************************************
 * Clock Enable Macros for SYSCFG peripheral
 ***********************************************/

#define SYSCFG_PCLK_EN()                ( RCC->APB4ENR |= (1 << 1) )                /* 1: SYSCFG peripheral clock enabled */

/**********************************************
 * Clock Disable Macros for GPIOx peripherals
 ***********************************************/

#define GPIOA_PCLK_DI()                 ( RCC->AHB4ENR &= ~(1 << 0) )               /* 0: GPIOA peripheral clock disabled */
#define GPIOB_PCLK_DI()                 ( RCC->AHB4ENR &= ~(1 << 1) )               /* 0: GPIOB peripheral clock disabled */
#define GPIOC_PCLK_DI()                 ( RCC->AHB4ENR &= ~(1 << 2) )               /* 0: GPIOC peripheral clock disabled */
#define GPIOD_PCLK_DI()                 ( RCC->AHB4ENR &= ~(1 << 3) )               /* 0: GPIOD peripheral clock disabled */
#define GPIOE_PCLK_DI()                 ( RCC->AHB4ENR &= ~(1 << 4) )               /* 0: GPIOE peripheral clock disabled */
#define GPIOF_PCLK_DI()                 ( RCC->AHB4ENR &= ~(1 << 5) )               /* 0: GPIOF peripheral clock disabled */
#define GPIOG_PCLK_DI()                 ( RCC->AHB4ENR &= ~(1 << 6) )               /* 0: GPIOG peripheral clock disabled */
#define GPIOH_PCLK_DI()                 ( RCC->AHB4ENR &= ~(1 << 7) )               /* 0: GPIOH peripheral clock disabled */
#define GPIOI_PCLK_DI()                 ( RCC->AHB4ENR &= ~(1 << 8) )               /* 0: GPIOI peripheral clock disabled */
#define GPIOJ_PCLK_DI()                 ( RCC->AHB4ENR &= ~(1 << 9) )               /* 0: GPIOJ peripheral clock disabled */
#define GPIOK_PCLK_DI()                 ( RCC->AHB4ENR &= ~(1 << 10) )              /* 0: GPIOK peripheral clock disabled */

/**********************************************
 * Clock Disable Macros for I2Cx peripherals
 ***********************************************/

#define I2C1_PCLK_DI()                  ( RCC->APB1LENR &= ~(1 << 21) )             /* 0: I2C1 peripheral clocks disabled */
#define I2C2_PCLK_DI()                  ( RCC->APB1LENR &= ~(1 << 22) )             /* 0: I2C2 peripheral clocks disabled */
#define I2C3_PCLK_DI()                  ( RCC->APB1LENR &= ~(1 << 23) )             /* 0: I2C2 peripheral clocks disabled */

/**********************************************
 * Clock Disable Macros for SPIx peripherals
 ***********************************************/

#define SPI2_PCLK_DI()                  ( RCC->APB1LENR &= ~(1 << 14) )             /* 0: SPI2 peripheral clocks disabled */
#define SPI3_PCLK_DI()                  ( RCC->APB1LENR &= ~(1 << 15) )             /* 0: SPI3 peripheral clocks disabled */

/**********************************************
 * Clock Disable Macros for USARTx peripherals
 ***********************************************/

#define USART2_PCLK_DI()                ( RCC->APB1LENR &= ~(1 << 17) )             /* 0: USART2 peripheral clocks disabled */
#define USART3_PCLK_DI()                ( RCC->APB1LENR &= ~(1 << 18) )             /* 0: USART3 peripheral clocks disabled */
#define UART4_PCLK_DI()                 ( RCC->APB1LENR &= ~(1 << 19) )             /* 0: UART4 peripheral clocks disabled */
#define UART5_PCLK_DI()                 ( RCC->APB1LENR &= ~(1 << 20) )             /* 0: UART5 peripheral clocks disabled */

/**********************************************
 * Clock Disable Macros for SYSCFG peripheral
 ***********************************************/

#define SYSCFG_PCLK_DI()                ( RCC->APB4ENR &= ~(1 << 1) )               /* 0: SYSCFG peripheral clock disabled */

/**********************************************
 * Macros to reset GPIOx peripherals
 ***********************************************/

#define GPIOA_REG_RESET()               do{ (RCC->AHB4RSTR |= (1 << 0));  (RCC->AHB4RSTR &= ~(1 << 0)); }while(0)       /* GPIOA block reset */
#define GPIOB_REG_RESET()               do{ (RCC->AHB4RSTR |= (1 << 1));  (RCC->AHB4RSTR &= ~(1 << 1)); }while(0)       /* GPIOB block reset */
#define GPIOC_REG_RESET()               do{ (RCC->AHB4RSTR |= (1 << 2));  (RCC->AHB4RSTR &= ~(1 << 2)); }while(0)       /* GPIOC block reset */
#define GPIOD_REG_RESET()               do{ (RCC->AHB4RSTR |= (1 << 3));  (RCC->AHB4RSTR &= ~(1 << 3)); }while(0)       /* GPIOD block reset */
#define GPIOE_REG_RESET()               do{ (RCC->AHB4RSTR |= (1 << 4));  (RCC->AHB4RSTR &= ~(1 << 4)); }while(0)       /* GPIOE block reset */
#define GPIOF_REG_RESET()               do{ (RCC->AHB4RSTR |= (1 << 5));  (RCC->AHB4RSTR &= ~(1 << 5)); }while(0)       /* GPIOF block reset */
#define GPIOG_REG_RESET()               do{ (RCC->AHB4RSTR |= (1 << 6));  (RCC->AHB4RSTR &= ~(1 << 6)); }while(0)       /* GPIOG block reset */
#define GPIOH_REG_RESET()               do{ (RCC->AHB4RSTR |= (1 << 7));  (RCC->AHB4RSTR &= ~(1 << 7)); }while(0)       /* GPIOH block reset */
#define GPIOI_REG_RESET()               do{ (RCC->AHB4RSTR |= (1 << 8));  (RCC->AHB4RSTR &= ~(1 << 8)); }while(0)       /* GPIOI block reset */
#define GPIOJ_REG_RESET()               do{ (RCC->AHB4RSTR |= (1 << 9));  (RCC->AHB4RSTR &= ~(1 << 9)); }while(0)       /* GPIOJ block reset */
#define GPIOK_REG_RESET()               do{ (RCC->AHB4RSTR |= (1 << 10)); (RCC->AHB4RSTR &= ~(1 << 10)); }while(0)      /* GPIOK block reset */

/**********************************************
 * Macro to return port code for given GPIOx base address
 * Returns a code (between 0 to 7) for a given GPIO base address(x)
 ***********************************************/

#define GPIO_BASEADDR_TO_CODE(x)        ( (x == GPIOA)?0:\
                                          (x == GPIOB)?1:\
                                          (x == GPIOC)?2:\
                                          (x == GPIOD)?3:\
                                          (x == GPIOE)?4:\
                                          (x == GPIOF)?5:\
                                          (x == GPIOG)?6:\
                                          (x == GPIOH)?7:\
                                          (x == GPIOI)?8:\
                                          (x == GPIOJ)?9:\
                                          (x == GPIOK)?10:0 )

/**********************************************
 * IRQ(Interrupt Request) Numbers
 * TODO: Complete this list for other peripherals
 ***********************************************/

#define IRQ_NO_EXTI0        6
#define IRQ_NO_EXTI1        7
#define IRQ_NO_EXTI2        8
#define IRQ_NO_EXTI3        9
#define IRQ_NO_EXTI4        10
#define IRQ_NO_EXTI9_5      23
#define IRQ_NO_EXTI15_10    40

/**********************************************
 * Generic Macros
 ***********************************************/

#define ENABLE                  1
#define DISABLE                 0
#define SET                     ENABLE
#define RESET                   DISABLE
#define GPIO_PIN_SET            SET
#define GPIO_PIN_RESET          RESET

/**********************************************
 * Include headers
 ***********************************************/
#include "stm32h743zi_gpio_driver.h"

#endif /* INC_STM32H743ZI_H_ */

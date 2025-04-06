/*
 * stm32f407xx.h
 *
 *  Created on: Nov 24, 2024
 *      Author: Dev Tandon
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vo   volatile					// Macro for volatile keyword
#define __weak __attribute__((weak))	// Macro for weak keyword

/**************************** START:Processor Specific Details *************************************/
/*
 * ARM Cortex Mx Processor NVIC (Nested Vectored Interrupt Controller) ISERx (Interrupt Set-Enable Register x) Register Addresses
 */
#define NVIC_ISER0			((__vo uint32_t*)0xE000E100U)
#define NVIC_ISER1			((__vo uint32_t*)0xE000E104U)
#define NVIC_ISER2			((__vo uint32_t*)0xE000E108U)
#define NVIC_ISER3			((__vo uint32_t*)0xE000E10CU)

/*
 * ARM Cortex Mx Processor NVIC (Nested Vectored Interrupt Controller) ICERx (Interrupt Clear-Enable Register x) Register Addresses
 */
#define NVIC_ICER0			((__vo uint32_t*)0xE000E180U)
#define NVIC_ICER1			((__vo uint32_t*)0xE000E184U)
#define NVIC_ICER2			((__vo uint32_t*)0xE000E188U)
#define NVIC_ICER3			((__vo uint32_t*)0xE000E18CU)

/*
 * ARM Cortex Mx Processor NVIC (Nested Vectored Interrupt Controller) Priority Register Addresses
 */
#define NVIC_PR_BASE_ADDR	((__vo uint32_t*)0xE000E400U)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_OF_PR_BITS_IMPLEMENTED		(4U)
/****************************************************************************************************/

/*
 * Base Address of Flash and SRAM memories
 */
#define FLASH_BASEADDR				(0x80000000U)  		// Starting Address of External Flash
#define SRAM1_BASEADDR				(0x20000000U)  		// Starting Address of SRAM1 and Size = 112Kb
#define SRAM2_BASEADDR				(0x20001C00U)  		// Starting Address of SRAM2 and Size = 16Kb  Address Offset = 0x00001C00U
#define ROM_BASEADDR				(0x1FFF0000U)  		// System Memory and Size = 30Kb
#define SRAM						(SRAM1_BASEADDR) 	// Starting Address of SRAM

/*
 * AHBx (Advanced High-Performance Bus x) and APBx (Advanced Peripherals Bus x) Bus Peripheral Base Addresses
 */
#define PERIPH_BASEADDR					(0x40000000U)		// Starting Address of Peripherals											Address Offset = 0x00000000U
#define APB1PERIPH_BASEADDR				(PERIPH_BASEADDR)	// Starting Address of APB1 (Advanced Peripherals Bus 1) Peripherals		Address Offset = 0x00000000U
#define APB2PERIPH_BASEADDR				(0x40010000U)		// Starting Address of APB2 (Advanced Peripherals Bus 2) Peripherals		Address Offset = 0x00001000U
#define AHB1PERIPH_BASEADDR				(0x40020000U)		// Starting Address of AHB1 (Advanced High-Performance Bus 1) Peripherals	Address Offset = 0x00002000U
#define AHB2PERIPH_BASEADDR				(0x50000000U)		// Starting Address of AHB2 (Advanced High-Performance Bus 2) Peripherals	Address Offset = 0x10000000U

/*
 * Base Addresses of peripherals which are hanging on AHB1 (Advanced High-Performance Bus 1) Bus
 * TODO : Complete for all other peripherals
 */
#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000U)		// Starting Address of GPIOA Port	Address Offset = 0x0000U
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400U)		// Starting Address of GPIOB Port	Address Offset = 0x0400U
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800U)		// Starting Address of GPIOC Port	Address Offset = 0x0800U
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00U)		// Starting Address of GPIOD Port	Address Offset = 0x0C00U
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000U)		// Starting Address of GPIOE Port	Address Offset = 0x1000U
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400U)		// Starting Address of GPIOF Port	Address Offset = 0x1400U
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800U)		// Starting Address of GPIOG Port	Address Offset = 0x1800U
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00U)		// Starting Address of GPIOH Port	Address Offset = 0x1C00U
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000U)		// Starting Address of GPIOI Port	Address Offset = 0x2000U

#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800U)		// Starting Address of RCC (Reset and Clock Control)	Address Offset = 0x3800U

/*
 * Base Addresses of peripherals which are hanging on APB1 (Advanced Peripherals Bus 1) Bus
 * TODO : Complete for all other peripherals
 */
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400U)		// Starting Address of I2C1 (Inter-Integrated Circuit 1)	Address Offset = 0x5400U
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800U)		// Starting Address of I2C2 (Inter-Integrated Circuit 2)	Address Offset = 0x5800U
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00U)		// Starting Address of I2C3 (Inter-Integrated Circuit 3)	Address Offset = 0x5C00U

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800U)		// Starting Address of SPI2 (Serial Peripheral Interface Circuit 2)	Address Offset = 0x3800U
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00U)		// Starting Address of SPI3 (Serial Peripheral Interface Circuit 3)	Address Offset = 0x3C00U

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400U)		// Starting Address of USART2 (Universal Synchronous and Asynchronous Receiver-Transmitter 2)	Address Offset = 0x4400U
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800U)		// Starting Address of USART3 (Universal Synchronous and Asynchronous Receiver-Transmitter 3)	Address Offset = 0x4800U
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00U)		// Starting Address of UART4 (Universal Asynchronous Receiver-Transmitter 4)					Address Offset = 0x4C00U
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000U)		// Starting Address of UART5 (Universal Asynchronous Receiver-Transmitter 5)					Address Offset = 0x5000U

/*
 * Base Addresses of peripherals which are hanging on APB2 (Advanced Peripherals Bus 2) Bus
 * TODO : Complete for all other peripherals
 */
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00U)		// Starting Address of EXTI (External Interrupt)												Address Offset = 0x3C00U
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000U)		// Starting Address of SPI1 (Serial Peripheral Interface Circuit 1)								Address Offset = 0x3000U
#define SPI4_BASEADDR				(APB2PERIPH_BASEADDR + 0x3400U)		// Starting Address of SPI4 (Serial Peripheral Interface Circuit 4)								Address Offset = 0x3400U
#define SPI5_BASEADDR				(APB2PERIPH_BASEADDR + 0x5000U)		// Starting Address of SPI5 (Serial Peripheral Interface Circuit 5)								Address Offset = 0x5000U
#define SPI6_BASEADDR				(APB2PERIPH_BASEADDR + 0x5400U)		// Starting Address of SPI6 (Serial Peripheral Interface Circuit 6)								Address Offset = 0x5400U
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800U)		// Starting Address of SYSCFG (System Configuration Controller)									Address Offset = 0x3800U
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000U)		// Starting Address of USART1 (Universal Synchronous and Asynchronous Receiver-Transmitter 1)	Address Offset = 0x1000U
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400U)		// Starting Address of USART6 (Universal Synchronous and Asynchronous Receiver-Transmitter 6)	Address Offset = 0x1400U

/**************************Peripheral Register Definition Structures**************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different (more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM (Reference Manual)
 */

/*
 * Peripheral register definition structure for GPIO (General Purpose Input Output)
 */
typedef struct
{
	__vo uint32_t MODER;		/* GPIO port mode register  	   		  Address Offset: 0x00U */
	__vo uint32_t OTYPER;		/* GPIO port output type register  		  Address Offset: 0x04U */
	__vo uint32_t OSPEEDR;		/* GPIO port output speed register  	  Address Offset: 0x08U */
	__vo uint32_t PUPDR;		/* GPIO port pull-up/pull-down register   Address Offset: 0x0CU */
	__vo uint32_t IDR;			/* GPIO port input data register 		  Address Offset: 0x10U */
	__vo uint32_t ODR;			/* GPIO port output data register 		  Address Offset: 0x14U */
	__vo uint32_t BSRR;			/* GPIO port bit set/reset register 	  Address Offset: 0x18U */
	__vo uint32_t LCKR;			/* GPIO port configuration lock register  Address Offset: 0x1CU */
	__vo uint32_t AFR[2U];		/* AF[0U] : GPIO alternate function low register, AF[1U] : GPIO alternate function high register  Address Offset: 0x20U */
}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC (Reset and Clock Control)
 */
typedef struct
{
	__vo uint32_t CR;			/* RCC clock control register										Address Offset: 0x00U */
	__vo uint32_t PLLCFGR;		/* RCC PLL configuration register									Address Offset: 0x04U */
	__vo uint32_t CFGR;			/* RCC clock configuration register									Address Offset: 0x08U */
	__vo uint32_t CIR;			/* RCC clock interrupt register										Address Offset: 0x0CU */
	__vo uint32_t AHB1RSTR;		/* RCC AHB1 peripheral reset register								Address Offset: 0x10U */
	__vo uint32_t AHB2RSTR;		/* RCC AHB2 peripheral reset register								Address Offset: 0x14U */
	__vo uint32_t AHB3RSTR;		/* RCC AHB3 peripheral reset register								Address Offset: 0x18U */
	uint32_t RESERVED0;			/* Reserved															Address Offset: 0x1CU */
	__vo uint32_t APB1RSTR;		/* RCC APB1 peripheral reset register								Address Offset: 0x20U */
	__vo uint32_t APB2RSTR;		/* RCC APB2 peripheral reset register								Address Offset: 0x24U */
	uint32_t RESERVED1[2U];		/* Reserved															Address Offset: 0x28U - 0x2CU */
	__vo uint32_t AHB1ENR;		/* RCC AHB1 peripheral clock enable register						Address Offset: 0x30U */
	__vo uint32_t AHB2ENR;		/* RCC AHB2 peripheral clock enable register						Address Offset: 0x34U */
	__vo uint32_t AHB3ENR;		/* RCC AHB3 peripheral clock enable register						Address Offset: 0x38U */
	uint32_t RESERVED2;			/* Reserved															Address Offset: 0x3CU */
	__vo uint32_t APB1ENR;		/* RCC APB1 peripheral clock enable register						Address Offset: 0x40U */
	__vo uint32_t APB2ENR;		/* RCC APB2 peripheral clock enable register						Address Offset: 0x44U */
	uint32_t RESERVED3[2U];		/* Reserved															Address Offset: 0x48U - 0x4CU */
	__vo uint32_t AHB1LPENR;	/* RCC AHB1 peripheral clock enable in low power mode register		Address Offset: 0x50U */
	__vo uint32_t AHB2LPENR;	/* RCC AHB2 peripheral clock enable in low power mode register		Address Offset: 0x54U */
	__vo uint32_t AHB3LPENR;	/* RCC AHB3 peripheral clock enable in low power mode register		Address Offset: 0x58U */
	__vo uint32_t RESERVED4;	/* Reserved															Address Offset: 0x5CU */
	__vo uint32_t APB1LPENR;	/* RCC APB1 peripheral clock enable in low power mode register		Address Offset: 0x60U */
	__vo uint32_t APB2LPENR;	/* RCC APB2 peripheral clock enable in low power mode register		Address Offset: 0x64U */
	uint32_t RESERVED5[2U];		/* Reserved															Address Offset: 0x68U - 0x6CU */
	__vo uint32_t BDCR;			/* RCC Backup domain control register								Address Offset: 0x70U */
	__vo uint32_t CSR;			/* RCC clock control & status register								Address Offset: 0x74U */
	uint32_t RESERVED6[2U];		/* Reserved															Address Offset: 0x78U - 0x7CU */
	__vo uint32_t SSCGR;		/* RCC spread spectrum clock generation register					Address Offset: 0x80U */
	__vo uint32_t PLLI2SCFGR;	/* RCC PLLI2S configuration register								Address Offset: 0x84U */
	__vo uint32_t PLLSAICFGR;	/* 																	Address Offset: 0x88U */
	__vo uint32_t DCKCFGR;		/*																	Address Offset: 0x8CU */
	__vo uint32_t CKGATENR;		/*																	Address Offset: 0x90U */
	__vo uint32_t DCKCFGR2;		/*																	Address Offset: 0x94U */
}RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI (External Interrupt)
 */
typedef struct
{
	__vo uint32_t IMR;		/* EXTI Interrupt Mask Register 	   		  Address Offset: 0x00U */
	__vo uint32_t EMR;		/* EXTI Event Mask Register 		  		  Address Offset: 0x04U */
	__vo uint32_t RTSR;		/* EXTI Rising Trigger Selection Register 	  Address Offset: 0x08U */
	__vo uint32_t FTSR;		/* EXTI Falling Trigger Selection Register    Address Offset: 0x0CU */
	__vo uint32_t SWIER;	/* EXTI Software Interrupt Event Register 	  Address Offset: 0x10U */
	__vo uint32_t PR;		/* EXTI Pending Register 		  			  Address Offset: 0x14U */
}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG (System Configuration Controller)
 */
typedef struct
{
	__vo uint32_t MEMRMP;			/* SYSCFG Memory Remap Register 	   		  					  Address Offset: 0x00U */
	__vo uint32_t PMC;				/* SYSCFG Peripheral Mode Configuration Register 		  		  Address Offset: 0x04U */
	__vo uint32_t EXTICR[4U];		/* SYSCFG External Interrupt Configuration Register 1,2,3,4 	  Address Offset: 0x08U - 0x14U */
	__vo uint32_t RESERVED1[2U];	/* Reserved   													  Address Offset: 0x18U - 0x1CU */
	__vo uint32_t CMPCR;			/* Compensation Cell Control Register 	  						  Address Offset: 0x20U */
	__vo uint32_t RESERVED2[2U];	/* Reserved 		  			  								  Address Offset: 0x24U - 0x28U */
	__vo uint32_t CFGR;				/*  		  			  										  Address Offset: 0x2CU */
}SYSCFG_RegDef_t;

/*
 * Peripheral register definition structure for SPI (Serial Peripheral Interface)
 */
typedef struct
{
	__vo uint32_t CR1;		/* SPI Control Register 1 	   		  		Address Offset: 0x00U */
	__vo uint32_t CR2;		/* SPI Control Register 2		  		  	Address Offset: 0x04U */
	__vo uint32_t SR;		/* SPI Status Register 	  				    Address Offset: 0x08U */
	__vo uint32_t DR;		/* SPI Data Register   						Address Offset: 0x0CU */
	__vo uint32_t CRCPR;	/* SPI CRC Polynomial Register 	  			Address Offset: 0x10U */
	__vo uint32_t RXCRCR;	/* SPI RX CRC Register 		  			  	Address Offset: 0x14U */
	__vo uint32_t TXCRCR;	/* SPI TX CRC Register		  			  	Address Offset: 0x18U */
	__vo uint32_t I2SCFGR;	/* SPI_I2S Configuration Register			Address Offset: 0x1CU */
	__vo uint32_t I2SPR;	/* SPI_I2S Prescaler Register			    Address Offset: 0x20U */
}SPI_RegDef_t;

/*
 * Peripheral register definition structure for I2C (Inter Integrated Circuit)
 */
typedef struct
{
	__vo uint32_t CR1;        /* I2C Control register 1    			    Address offset: 0x00 */
	__vo uint32_t CR2;        /* I2C Control register 2   				Address offset: 0x04 */
    __vo uint32_t OAR1;       /* I2C Own address register 1   			Address offset: 0x08 */
	__vo uint32_t OAR2;       /* I2C Own address register 2   			Address offset: 0x0C */
	__vo uint32_t DR;         /* I2C Data register   					Address offset: 0x10 */
	__vo uint32_t SR1;        /* I2C Status register 1 					Address offset: 0x14 */
	__vo uint32_t SR2;        /* I2C Status register 2   				Address offset: 0x18 */
    __vo uint32_t CCR;        /* I2C Clock control register    			Address offset: 0x1C */
    __vo uint32_t TRISE;      /* I2C TRISE register    					Address offset: 0x20 */
	__vo uint32_t FLTR;       /* I2C FLTR register   					Address offset: 0x24 */
}I2C_RegDef_t;

/*
 * peripheral register definition structure for USART (Universal Synchronous and Asynchronous Receiver - Transmitter)
 */
typedef struct
{
	__vo uint32_t SR;         /* USART Status register     				 Address offset: 0x00 */
	__vo uint32_t DR;         /* USART Data register     				 Address offset: 0x04 */
	__vo uint32_t BRR;        /* USART Baud rate register      			 Address offset: 0x08 */
	__vo uint32_t CR1;        /* USART Control register 1     			 Address offset: 0x0C */
	__vo uint32_t CR2;        /* USART Control register 2     		     Address offset: 0x10 */
	__vo uint32_t CR3;        /* USART Control register 3     			 Address offset: 0x14 */
	__vo uint32_t GTPR;       /* USART Guard time and prescaler register Address offset: 0x18 */
} USART_RegDef_t;

/*
 * Peripheral Definitions (Peripheral Base Addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI			((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4			((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5			((SPI_RegDef_t*)SPI5_BASEADDR)
#define SPI6			((SPI_RegDef_t*)SPI6_BASEADDR)

#define I2C1  			((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  			((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  			((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1  		((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  		((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  		((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  			((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  			((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  		((USART_RegDef_t*)USART6_BASEADDR)

/*
 * Clock Enable Macros for GPIOx (General Purpose Input Output x) peripherals
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))

/*
 * Clock Enable Macros for I2Cx (Inter-Integrated Circuit x) peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPIx (Serial Peripheral Interface Circuit x) peripherals
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()		(RCC->APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN()		(RCC->APB2ENR |= (1 << 21))

/*
 * Clock Enable Macros for USARTx (Universal Synchronous and Asynchronous Receiver-Transmitter x) peripherals
 */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()  	(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()  	(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG (System Configuration Controller) peripherals
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

/*
 * Clock Disable Macros for GPIOx (General Purpose Input Output x) peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock Disable Macros for I2Cx (Inter-Integrated Circuit x) peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Disable Macros for SPIx (Serial Peripheral Interface Circuit x) peripherals
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 21))

/*
 * Clock Disable Macros for USARTx (Universal Synchronous and Asynchronous Receiver-Transmitter x) peripherals
 */
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()  	(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()  	(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock Disable Macros for SYSCFG (System Configuration Controller) peripherals
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIOx (General Purpose Input Output x) peripherals
 */
#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)	// Set as 1 then Reset as 0
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)	// Set as 1 then Reset as 0
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)	// Set as 1 then Reset as 0
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)	// Set as 1 then Reset as 0
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)	// Set as 1 then Reset as 0
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)	// Set as 1 then Reset as 0
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)	// Set as 1 then Reset as 0
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)	// Set as 1 then Reset as 0
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));}while(0)	// Set as 1 then Reset as 0

/*
 * Macros to reset SPIx (Serial Peripheral Interface x) peripherals
 */
#define SPI1_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));}while(0)	// Set as 1 then Reset as 0
#define SPI2_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));}while(0)	// Set as 1 then Reset as 0
#define SPI3_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));}while(0)	// Set as 1 then Reset as 0
#define SPI4_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13));}while(0)	// Set as 1 then Reset as 0
#define SPI5_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 20)); (RCC->APB2RSTR &= ~(1 << 20));}while(0)	// Set as 1 then Reset as 0
#define SPI6_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 21)); (RCC->APB2RSTR &= ~(1 << 21));}while(0)	// Set as 1 then Reset as 0

/*
 * Macros to reset I2Cx (Inter Integrated Circuit x) peripherals
 */
#define I2C1_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21));}while(0)	// Set as 1 then Reset as 0
#define I2C2_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22));}while(0)	// Set as 1 then Reset as 0
#define I2C3_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23));}while(0)	// Set as 1 then Reset as 0

/*
 * Macros to reset USARTx (Universal Synchronous and Asynchronous Receiver-Transmitter x) peripherals
 */
#define USART1_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 4));  (RCC->APB2RSTR &= ~(1 << 4));}while(0)	    // Set as 1 then Reset as 0
#define USART2_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17));}while(0)	// Set as 1 then Reset as 0
#define USART3_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18));}while(0)	// Set as 1 then Reset as 0
#define UART4_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19));}while(0)	// Set as 1 then Reset as 0
#define UART5_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20));}while(0)	// Set as 1 then Reset as 0
#define USART6_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 5));  (RCC->APB2RSTR &= ~(1 << 5));}while(0)	    // Set as 1 then Reset as 0

/*
 * Macro to Code from Base Address of GPIOx (General Purpose Input Output x) peripherals
 */
#define GPIO_BASEADDR_TO_CODE(x)	(x == GPIOA) ? 0U : \
									(x == GPIOB) ? 1U : \
									(x == GPIOC) ? 2U : \
									(x == GPIOD) ? 3U : \
									(x == GPIOE) ? 4U : \
									(x == GPIOF) ? 5U : \
									(x == GPIOG) ? 6U : \
									(x == GPIOH) ? 7U : \
									(x == GPIOI) ? 8U : 0U

/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: Update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */
#define IRQ_NO_EXTI0		(6U)
#define IRQ_NO_EXTI1		(7U)
#define IRQ_NO_EXTI2		(8U)
#define IRQ_NO_EXTI3		(9U)
#define IRQ_NO_EXTI4		(10U)
#define IRQ_NO_EXTI9_5		(23U)
#define IRQ_NO_EXTI15_10	(40U)

#define IRQ_NO_SPI1			(35U)
#define IRQ_NO_SPI2			(36U)
#define IRQ_NO_SPI3			(51U)

#define IRQ_NO_I2C1_EV		(31U)
#define IRQ_NO_I2C1_ER		(32U)

#define IRQ_NO_USART1	    (37U)
#define IRQ_NO_USART2		(38U)
#define IRQ_NO_USART3		(39U)
#define IRQ_NO_UART4		(52U)
#define IRQ_NO_UART5		(53U)
#define IRQ_NO_USART6		(71U)

/*
 * Macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0		(0U)
#define NVIC_IRQ_PRI15		(15U)

// Some Generic Macros
#define ENABLE  			(1U)
#define DISABLE 			(0U)
#define SET					(ENABLE)
#define RESET   			(DISABLE)
#define GPIO_PIN_SET		(SET)
#define GPIO_PIN_RESET  	(RESET)
#define HIGH				(ENABLE)
#define LOW					(DISABLE)
#define BTN_PRESSED_HIGH	(HIGH)
#define BTN_PRESSED_LOW		(LOW)
#define FLAG_SET			(SET)
#define FLAG_RESET			(RESET)

/*****************************************************************************
 * Bit Position Definitions of SPI (Serial Peripheral Interface) Peripheral
******************************************************************************/
/*
 * Bit Position definitions of SPI_CR1 (Serial Peripheral Interface Control Register 1)
 */
#define SPI_CR1_CPHA		(0U)
#define SPI_CR1_CPOL		(1U)
#define SPI_CR1_MSTR		(2U)
#define SPI_CR1_BR			(3U)
#define SPI_CR1_SPE			(6U)
#define SPI_CR1_LSBFIRST	(7U)
#define SPI_CR1_SSI			(8U)
#define SPI_CR1_SSM			(9U)
#define SPI_CR1_RXONLY		(10U)
#define SPI_CR1_DFF			(11U)
#define SPI_CR1_CRCNEXT		(12U)
#define SPI_CR1_CRCEN		(13U)
#define SPI_CR1_BIDIOE		(14U)
#define SPI_CR1_BIDIMODE	(15U)

/*
 * Bit Position definitions of SPI_CR2 (Serial Peripheral Interface Control Register 2)
 */
#define SPI_CR2_RXDMAEN		(0U)
#define SPI_CR2_TXDMAEN		(1U)
#define SPI_CR2_SSOE		(2U)
#define SPI_CR2_FRF			(4U)
#define SPI_CR2_ERRIE		(5U)
#define SPI_CR2_RXNEIE		(6U)
#define SPI_CR2_TXEIE		(7U)

/*
 * Bit Position definitions of SPI_SR (Serial Peripheral Interface Status Register)
 */
#define SPI_SR_RXNE			(0U)
#define SPI_SR_TXE			(1U)
#define SPI_SR_CHSIDE		(2U)
#define SPI_SR_UDR			(3U)
#define SPI_SR_CRCERR		(4U)
#define SPI_SR_MODF			(5U)
#define SPI_SR_OVR			(6U)
#define SPI_SR_BSY			(7U)
#define SPI_SR_FRE			(8U)

/******************************************************************************************
 *Bit position definitions of I2C (Inter-Integrated Circuit) peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1 (Inter-Integrated Circuit Control Register 1)
 */
#define I2C_CR1_PE						(0U)
#define I2C_CR1_NOSTRETCH  				(7U)
#define I2C_CR1_START 					(8U)
#define I2C_CR1_STOP  				 	(9U)
#define I2C_CR1_ACK 				 	(10U)
#define I2C_CR1_SWRST  				 	(15U)

/*
 * Bit position definitions I2C_CR2 (Inter-Integrated Circuit Control Register 2)
 */
#define I2C_CR2_FREQ				 	(0U)
#define I2C_CR2_ITERREN				 	(8U)
#define I2C_CR2_ITEVTEN				 	(9U)
#define I2C_CR2_ITBUFEN 			    (10U)

/*
 * Bit position definitions I2C_OAR1 (Inter-Integrated Circuit Own Address Register 1)
 */
#define I2C_OAR1_ADD0    				 (0U)
#define I2C_OAR1_ADD71 				 	 (1U)
#define I2C_OAR1_ADD98  			 	 (8U)
#define I2C_OAR1_ADDMODE   			 	 (15U)

/*
 * Bit position definitions I2C_SR1 (Inter-Integrated Circuit Status Register 1)
 */
#define I2C_SR1_SB 					 	(0U)
#define I2C_SR1_ADDR 				 	(1U)
#define I2C_SR1_BTF 					(2U)
#define I2C_SR1_ADD10 					(3U)
#define I2C_SR1_STOPF 					(4U)
#define I2C_SR1_RXNE 					(6U)
#define I2C_SR1_TXE 					(7U)
#define I2C_SR1_BERR 					(8U)
#define I2C_SR1_ARLO 					(9U)
#define I2C_SR1_AF 					 	(10U)
#define I2C_SR1_OVR 					(11U)
#define I2C_SR1_TIMEOUT 				(14U)

/*
 * Bit position definitions I2C_SR2 (Inter-Integrated Circuit Status Register 2)
 */
#define I2C_SR2_MSL						(0U)
#define I2C_SR2_BUSY 					(1U)
#define I2C_SR2_TRA 					(2U)
#define I2C_SR2_GENCALL 				(4U)
#define I2C_SR2_DUALF 					(7U)

/*
 * Bit position definitions I2C_CCR (Inter-Integrated Clock Control Register)
 */
#define I2C_CCR_CCR 					 (0U)
#define I2C_CCR_DUTY 					 (14U)
#define I2C_CCR_FS  				 	 (15U)

/******************************************************************************************
 *Bit position definitions of USART (Universal Synchronous and Asynchronous Receiver-Transmitter) peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1 (Universal Synchronous and Asynchronous Receiver-Transmitter Control Register 1)
 */
#define USART_CR1_SBK					(0U)
#define USART_CR1_RWU 					(1U)
#define USART_CR1_RE  					(2U)
#define USART_CR1_TE 					(3U)
#define USART_CR1_IDLEIE 				(4U)
#define USART_CR1_RXNEIE  				(5U)
#define USART_CR1_TCIE					(6U)
#define USART_CR1_TXEIE					(7U)
#define USART_CR1_PEIE 					(8U)
#define USART_CR1_PS 					(9U)
#define USART_CR1_PCE 					(10U)
#define USART_CR1_WAKE  				(11U)
#define USART_CR1_M 					(12U)
#define USART_CR1_UE 					(13U)
#define USART_CR1_OVER8  				(15U)

/*
 * Bit position definitions USART_CR2 (Universal Synchronous and Asynchronous Receiver-Transmitter Control Register 2)
 */
#define USART_CR2_ADD   				(0U)
#define USART_CR2_LBDL   				(5U)
#define USART_CR2_LBDIE  				(6U)
#define USART_CR2_LBCL   				(8U)
#define USART_CR2_CPHA   				(9U)
#define USART_CR2_CPOL   				(10U)
#define USART_CR2_STOP   				(12U)
#define USART_CR2_LINEN   				(14U)

/*
 * Bit position definitions USART_CR3 (Universal Synchronous and Asynchronous Receiver-Transmitter Control Register 3)
 */
#define USART_CR3_EIE   				(0U)
#define USART_CR3_IREN   				(1U)
#define USART_CR3_IRLP  				(2U)
#define USART_CR3_HDSEL   				(3U)
#define USART_CR3_NACK   				(4U)
#define USART_CR3_SCEN   				(5U)
#define USART_CR3_DMAR  				(6U)
#define USART_CR3_DMAT   				(7U)
#define USART_CR3_RTSE   				(8U)
#define USART_CR3_CTSE   				(9U)
#define USART_CR3_CTSIE   				(10U)
#define USART_CR3_ONEBIT   				(11U)

/*
 * Bit position definitions USART_SR (Universal Synchronous and Asynchronous Receiver-Transmitter Status Register)
 */
#define USART_SR_PE        				(0U)
#define USART_SR_FE        				(1U)
#define USART_SR_NE        				(2U)
#define USART_SR_ORE       				(3U)
#define USART_SR_IDLE       			(4U)
#define USART_SR_RXNE        			(5U)
#define USART_SR_TC        				(6U)
#define USART_SR_TXE        			(7U)
#define USART_SR_LBD        			(8U)
#define USART_SR_CTS        			(9U)

#endif /* INC_STM32F407XX_H_ */

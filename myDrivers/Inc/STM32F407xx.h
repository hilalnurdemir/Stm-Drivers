/*
 * STM32F407xx.h
 *
 *  Created on: Aug 6, 2021
 *      Author: Hilal_Embedded
 */

#ifndef INC_STM32F407XX_H_

#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stdio.h>


#define _IO          volatile


#define SET_BIT(REG, BIT)    ( (REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)  ( (REG) &= ~(BIT))
#define READ_BIT(REG, BIT)   ( (REG) & (BIT))
#define UNUSED(x)            (void)x

typedef enum
{
	DISABLE = 0x0U,
	ENABLE = !DISABLE

}FunctionalState_t ;

/*
  * IRQ Numbers of MCU = Vector Table
  *
  */

typedef enum{
	EXTI0_IRQNumber = 6,
	EXTI1_IRQNumber = 7,
	EXTI2_IRQNumber = 8,
	EXTI3_IRQNumber = 9


}IRQ_NumberTypeDef_t ;


/*
  * MICROPROCESSOR DEFINES
  *
  */

#define NVIC_ISER0            ((uint32_t*)(0xE000E100))         //pointer ayarladim 4 artarak degisir


 /*
  * MEMORY BASE ADDRESS
  *
  */

#define FLASH_BASE_ADR         (0x08000000UL)    // up to 1mb
#define SRAM1_BASE_ADR         (0x20000000UL)    // up to 112kb
#define SRAM2_BASE_ADR         (0X2001C000UL)    // up to 16kb

/*
 * PERIPHEARAL BASE ADDRESSES
 */

#define PERIPH_BASE_ADR        (0X40000000UL)
#define APB1_BASE_ADR           PERIPH_BASE_ADR
#define APB2_BASE_ADR          (PERIPH_BASE_ADR + 0x00010000UL)
#define AHB1_BASE_ADR          (PERIPH_BASE_ADR + 0x00020000UL)
#define AHB2_BASE_ADR          (PERIPH_BASE_ADR + 0x10000000UL)

/*
 * APB1 PERIPHERAL BASE ADDRESSES
 */

#define TIM2_BASE_ADR           (APB1_BASE_ADR + 0x00000000UL)
#define TIM3_BASE_ADR           (APB1_BASE_ADR + 0x0400UL)       //timer address
#define TIM4_BASE_ADR           (APB1_BASE_ADR + 0x0800UL)
#define TIM5_BASE_ADR           (APB1_BASE_ADR + 0x0C00UL)
#define TIM6_BASE_ADR           (APB1_BASE_ADR + 0x1000UL)


#define SPI2_BASE_ADR           (APB1_BASE_ADR + 0x3800UL)       //SPI base address
#define SPI3_BASE_ADR           (APB1_BASE_ADR + 0x3C00UL)

#define USART2_BASE_ADR         (APB1_BASE_ADR + 0x4400UL)       //USART base address
#define USART3_BASE_ADR         (APB1_BASE_ADR + 0x4800UL)
#define USART4_BASE_ADR         (APB1_BASE_ADR + 0x4C00UL)
#define USART5_BASE_ADR         (APB1_BASE_ADR + 0x5000UL)

#define I2C1_BASE_ADR           (APB1_BASE_ADR + 0x5400UL)      //I2C base
#define I2C2_BASE_ADR           (APB1_BASE_ADR + 0x5800UL)
#define I2C3_BASE_ADR           (APB1_BASE_ADR + 0x5C00UL)

#define UART7_BASE_ADR          (APB1_BASE_ADR + 0x7800UL)     //UART base
#define UART8_BASE_ADR          (APB1_BASE_ADR + 0x7C00UL)




/*
 * APB2 PERIPHEARAL BASE ADDRESSES
 */


#define TIM1_BASE_ADR            APB2_BASE_ADR                   //TIMER base address
#define TIM8_BASE_ADR           (APB2_BASE_ADR + 0x0400UL)

#define SPI1_BASE_ADR           (APB2_BASE_ADR + 0x3000UL)
#define SPI4_BASE_ADR           (APB2_BASE_ADR + 0x3400UL)

#define SYSCFG_BASE_ADR         (APB2_BASE_ADR + 0x3800UL)      //SYSCFG base address
#define EXTI_BASE_ADR           (APB2_BASE_ADR + 0x3C00UL)      //EXTI base address

#define TIM9_BASE_ADR           (APB2_BASE_ADR + 0x4000UL)      //TIMER base address
#define TIM10_BASE_ADR          (APB2_BASE_ADR + 0x4400UL)
#define TIM11_BASE_ADR          (APB2_BASE_ADR + 0x4800UL)

#define SPI5_BASE_ADR           (APB2_BASE_ADR + 0x5000UL)      //SPI base address
#define SPI6_BASE_ADR           (APB2_BASE_ADR + 0x5400UL)

/*
 * AHB1 PERIPHEARAL BASE ADDRESSES
 */

#define GPIOA_BASE_ADR         (AHB1_BASE_ADR)                  //GPIO base address
#define GPIOB_BASE_ADR         (AHB1_BASE_ADR + 0x0400UL)
#define GPIOC_BASE_ADR         (AHB1_BASE_ADR + 0x0800UL)
#define GPIOD_BASE_ADR         (AHB1_BASE_ADR + 0x0C00UL)
#define GPIOE_BASE_ADR         (AHB1_BASE_ADR + 0x1000UL)
#define GPIOF_BASE_ADR         (AHB1_BASE_ADR + 0x1400UL)

#define RCC_BASE_ADR           (AHB1_BASE_ADR + 0x3800UL)       //RCC base address


/*
 * PERIPHERAL STRUCTURE DEFINITIONS
 */

typedef struct{

	_IO uint32_t MODER ;   // GPIO port mode register address offset 0x00
	_IO uint32_t OTYPER ;  // GPIO port output type register offset 0X04
	_IO uint32_t OSPEEDR ; // GPIO port output speed register offset 0x08
	_IO uint32_t PUPDR ;   // GPIO port pull-up/pull-down register offset 0x0C
	_IO uint32_t IDR ;     // GPIO port input data register offset 0x10;
	_IO uint32_t ODR ;     // GPIO port output data register offset 0x14
	_IO uint32_t BSRR ;    // GPIO port bit set/reset register offset 0x18
	_IO uint32_t LCKR ;    // GPIO port configuration lock register offset 0x1C
	_IO uint32_t AFR[2] ;  // GPIO alternate function register offset 0x20

}GPIO_TypeDef_t ;

typedef struct{

	_IO uint32_t CR ;
	_IO uint32_t PLLCFGR ;
	_IO uint32_t CCFGR ;
	_IO uint32_t CIR ;
	_IO uint32_t AHB1RSTR ;
	_IO uint32_t AHB2RSTR ;
	_IO uint32_t AHB3RSTR ;
	_IO uint32_t RESERVED0 ;
	_IO uint32_t APB1RSTR ;
	_IO uint32_t APB2RSTR ;
	_IO uint32_t RESERVED1[2] ;
	_IO uint32_t AHB1ENR ;
	_IO uint32_t AHB2ENR ;
	_IO uint32_t AHB3ENR ;
	_IO uint32_t REVERSED2 ;
	_IO uint32_t APB1ENR ;
	_IO uint32_t APB2ENR ;
	_IO uint32_t RESERVED3[2] ;
	_IO uint32_t AHB1LPENR ;
	_IO uint32_t AHB2LPENR ;
	_IO uint32_t AHB3LPENR ;
	_IO uint32_t RESERVED4 ;
	_IO uint32_t APB1LPENR ;
	_IO uint32_t APB2LPENR ;
	_IO uint32_t RESERVED5[2] ;
	_IO uint32_t BDCR ;
	_IO uint32_t CSR ;
	_IO uint32_t RESERVED6[2] ;
	_IO uint32_t SSCGR ;
	_IO uint32_t PLLI2SCFGR ;



}RCC_TypeDef_t ;

typedef struct{
	_IO uint32_t CR1;            // SPI control register 1 0x00
	_IO uint32_t CR2 ;           // SPI control register 2 0x04
	_IO uint32_t SR ;            // SPI status register 0x08
	_IO uint32_t DR ;            // SPI data register   0x0C
	_IO uint32_t CRCPR ;         // SPI CRC polynomial register 0x10
	_IO uint32_t RXCRCR ;        // SPI RX CRC register 0x14
	_IO uint32_t TXCRCR ;        // SPI TX CRC register 0x18
	_IO uint32_t I2SCFGR ;       // SPI_I2S configuration register
	_IO uint32_t I2SPR ;         // SPI_I2S prescaler register


}SPI_TypeDef_t ;

typedef struct{

	_IO uint32_t MEMRMP ;       // SYSCFG memory remap register 0X00
	_IO uint32_t PMC ;          // SYSCFG peripheral mode configuration register 0X04
	_IO uint32_t EXTI_CR[4] ;      // SYSCFG external interrupt configuration register 0X08 - 0X14
	_IO uint32_t REVERSED0[2] ; //0X18-0X1C
	_IO uint32_t CMPCR ;        // Compensation cell control register 0X20


}SYSCFG_TypeDef_t ;


typedef struct{
	_IO uint32_t IMR ;          // Interrupt mask register 0X00
	_IO uint32_t EMR ;          // Event mask register 0X04
	_IO uint32_t RTSR ;         // Rising trigger selection register 0X08
	_IO uint32_t FTSR ;         // Falling trigger selection register 0X0C
	_IO uint32_t SWIER ;        // Software interrupt event register 0X10
	_IO uint32_t PR  ;          // Pending register 0X14


}EXTI_TypeDef_t ;



#define GPIOA                  ((GPIO_TypeDef_t *)(GPIOA_BASE_ADR)  )
#define GPIOB                  ((GPIO_TypeDef_t *)(GPIOB_BASE_ADR)  )
#define GPIOC                  ((GPIO_TypeDef_t *)(GPIOC_BASE_ADR)  )
#define GPIOD                  ((GPIO_TypeDef_t *)(GPIOD_BASE_ADR)  )

#define RCC                    ((RCC_TypeDef_t *)(RCC_BASE_ADR)     )

#define SYSCFG                 ((SYSCFG_TypeDef_t*)(SYSCFG_BASE_ADR))

#define EXTI                   ((EXTI_TypeDef_t*)(EXTI_BASE_ADR)    )

#define SPI1                   ((SPI_TypeDef_t*)(SPI1_BASE_ADR)     )
#define SPI2                   ((SPI_TypeDef_t*)(SPI2_BASE_ADR)     )
#define SPI3                   ((SPI_TypeDef_t*)(SPI3_BASE_ADR)     )
#define SPI4                   ((SPI_TypeDef_t*)(SPI4_BASE_ADR)     )


/*
 * Bit Definition
 */

#define RCC_AHB1ENR_GPIOAEN_Pos  0U  // RCC AHB1ENR register GPIOAEN bit position
#define RCC_AHB1ENR_GPIOAEN_Mask (0x1 << RCC_AHB1ENR_GPIOAEN_Pos)
#define RCC_AHB1ENR_GPIOAEN       RCC_AHB1ENR_GPIOAEN_Mask

#define RCC_AHB1ENR_GPIOBEN_Pos  1U  // RCC AHB1ENR register GPIOBEN bit position
#define RCC_AHB1ENR_GPIOBEN_Mask (0x1 << RCC_AHB1ENR_GPIOBEN_Pos)
#define RCC_AHB1ENR_GPIOBEN      RCC_AHB1ENR_GPIOBEN_Mask

#define RCC_AHB1ENR_GPIOCEN_Pos  2U  // RCC AHB1ENR register GPIOCEN bit position
#define RCC_AHB1ENR_GPIOCEN_Mask (0x1 << RCC_AHB1ENR_GPIOCEN_Pos)
#define RCC_AHB1ENR_GPIOCEN      RCC_AHB1ENR_GPIOCEN_Mask

#define RCC_AHB1ENR_GPIODEN_Pos  3U  // RCC AHB1ENR register GPIODEN bit position
#define RCC_AHB1ENR_GPIODEN_Mask (0x1 << RCC_AHB1ENR_GPIODEN_Pos)
#define RCC_AHB1ENR_GPIODEN      RCC_AHB1ENR_GPIODEN_Mask

#define RCC_APB2ENR_SYSCFGEN_Pos     14U  //  RCC APB2 register SYSCFGEN bit position
#define RCC_APB2ENR_SYSCFGEN_Mask    (0x1 << RCC_APB2ENR_SYSCFGEN_Pos)
#define RCC_APB2ENR_SYSCFGEN        RCC_APB2ENR_SYSCFGEN_Mask

#define RCC_APB2ENR_SPI1EN_Pos     12U  //  RCC APB2 register SPI1EN bit position
#define RCC_APB2ENR_SPI1EN_Mask    (0x1 << RCC_APB2ENR_SPI1EN_Pos )
#define RCC_APB2ENR_SPI1EN         RCC_APB2ENR_SPI1EN_Mask

#define RCC_APB1ENR_SPI2EN_Pos     14U  //  RCC APB1 register SPI2EN bit position
#define RCC_APB1ENR_SPI2EN_Mask    (0x1 << RCC_APB1ENR_SPI2EN_Pos )
#define RCC_APB1ENR_SPI2EN         RCC_APB1ENR_SPI2EN_Mask

#define RCC_APB1ENR_SPI3EN_Pos     15U  //  RCC APB1 register SPI3EN bit position
#define RCC_APB1ENR_SPI3EN_Mask    (0x1 << RCC_APB1ENR_SPI3EN_Pos )
#define RCC_APB1ENR_SPI3EN         RCC_APB1ENR_SPI3EN_Mask


#define RCC_APB2ENR_SPI4EN_Pos     13U  //  RCC APB2 register SPI4EN bit position
#define RCC_APB2ENR_SPI4EN_Mask    (0x1 << RCC_APB2ENR_SPI4EN_Pos )
#define RCC_APB2ENR_SPI4EN         RCC_APB2ENR_SPI4EN_Mask

#define SPI_CR1_SPEN              6U   // SPI CR1 Enable bit position

#define SPI_SR_TxE                (1U)
#define SPI_CR1_SPE               (6U)
#define SPI_SR_Busy               (7U)

/*
 *
 * Flag Definitons
 */

#define SPI_TxE_FLAG             (0x1U<<SPI_SR_TxE)
#define SPI_Busy_FLAG            (0x1U <<SPI_SR_Busy)


#include "GPIO.h"
#include "RCC.h"
#include "EXTI.h"
#include "SPI.h"



#endif /* INC_STM32F407XX_H_ */

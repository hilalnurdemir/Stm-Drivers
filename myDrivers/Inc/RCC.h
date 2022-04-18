/*
 * RCC.h
 *
 *  Created on: 30  2021
 *      Author: Hilal_Embedded
 */

#ifndef INC_RCC_H_
#define INC_RCC_H_

#include "STM32F407xx.h"

#define RCC_GPIOA_CLK_ENABLE()      do{  \
	                                   uint32_t tempValue = 0 ; \
                                      SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN) ; \
									  tempValue = READ_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOAEN )    ;\
									  UNUSED(tempValue ) ; \
                                       }while(0U)

#define RCC_GPIOB_CLK_ENABLE()     do{   \
	                                  uint32_t tempValue = 0 ; \
                                      SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN) ; \
									  tempValue = READ_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOBEN )    ;\
									  UNUSED(tempValue) ; \
                                       }while(0U)

#define RCC_GPIOC_CLK_ENABLE()      do{   \
	                                  uint32_t tempValue = 0 ; \
                                      SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN) ; \
									  tempValue = READ_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOCEN )    ;\
									  UNUSED(tempValue) ; \
                                       }while(0U)

#define RCC_GPIOD_CLK_ENABLE()      do{  \
                                      uint32_t tempValue = 0 ;   \
                                      SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN) ; \
									  tempValue = READ_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN );  \
									  UNUSED(tempValue) ; \
                                       }while(0U)



#define RCC_SYSCFG_CLK_ENABLE()      do{ \
	                                  uint32_t tempValue = 0; \
                                      SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN) ; \
                                      tempValue = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN); \
                                      UNUSED(tempValue) ; \
	                                  } while(0U)

/*
 *
 * RCC APB1 PERIPH CLOCK CONTROL MAKRO DEFINITIONS
 *
 *
 */


#define RCC_SPI2_CLK_ENABLE()          do{ \
                                        uint32_t tempValue = 0; \
                                        SET_BIT(RCC->APB1_BASE_ADR, RCC_APB1ENR_SPI1EN) ;\
										tempValue = READ_BIT(RCC->APB1_BASE_ADR, RCC_APB1ENR_SPI1EN) ;\
										UNUSED(tempValue) ; \
                                        }while(0U)

#define RCC_SPI3_CLK_ENABLE()          do{ \
                                        uint32_t tempValue = 0; \
                                        SET_BIT(RCC->APB1_BASE_ADR, RCC_APB1ENR_SPI3EN) ;\
										tempValue = READ_BIT(RCC->APB1_BASE_ADR, RCC_APB1ENR_SPI3EN) ;\
										UNUSED(tempValue) ; \
                                        }while(0U)


#define RCC_SPI2_CLK_DISABLE()          CLEAR_BIT(RCC->APB1_BASE_ADR, RCC_APB1ENR_SPI2EN)

/*
 *
 * RCC APB2 PERIPH CLOCK CONTROL MAKRO DEFINITIONS
 *
 *
 */

#define RCC_SPI1_CLK_ENABLE()          do{ \
                                        uint32_t tempValue = 0; \
                                        SET_BIT(RCC->APB2_BASE_ADR, RCC_APB2ENR_SPI1EN) ;\
										tempValue = READ_BIT(RCC->APB2_BASE_ADR, RCC_APB2ENR_SPI1EN) ;\
										UNUSED(tempValue) ; \
                                        }while(0U)

#define RCC_SPI4_CLK_ENABLE()          do{ \
                                        uint32_t tempValue = 0; \
                                        SET_BIT(RCC->APB2_BASE_ADR, RCC_APB2ENR_SPI4EN); \
										tempValue = READ_BIT(RCC->APB2_BASE_ADR, RCC_APB2ENR_SPI4EN) ; \
										UNUSED(tempValue) ; \
                                        }while(0U)


#define RCC_SPI1_CLK_DISABLE()          CLEAR_BIT(RCC->APB2_BASE_ADR, RCC_APB2ENR_SPI1EN)


#define RCC_SYSCFG_CLK_DISABLE()         CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN)
#define RCC_GPOIA_CLK_DISABLE()          CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOAEN)
#define RCC_GPOIB_CLK_DISABLE()          CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOBEN)
#define RCC_GPOIC_CLK_DISABLE()          CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOCEN)
#define RCC_GPOID_CLK_DISABLE()          CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN)






#endif /* INC_RCC_H_ */

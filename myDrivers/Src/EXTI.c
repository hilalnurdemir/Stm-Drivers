/*
 * EXTI.c
 *
 *  Created on: 17 agu 2021
 *      Author: Hilal_Embedded
 */



#include "EXTI.h"

/*
 * @brief EXTI_Init for valid GPIO port and line number
 * @param EXTI_InitStruct = User configuration structure
 * @retval void
 *
 */

void EXTI_Init(EXTI_InitTypeDef_t* EXTI_InitStruct) {

	uint32_t tempValue = 0;

	tempValue = (uint32_t)EXTI_BASE_ADR ;


	EXTI->IMR &= ~(0X1U << EXTI_InitStruct->EXTI_LineNumber) ;
    EXTI->EMR &= ~(0X1U << EXTI_InitStruct->EXTI_LineNumber) ;


   if(EXTI_InitStruct->EXTI_LineCmd != DISABLE){

	   tempValue += EXTI_InitStruct->EXTI_Mode ;

	   *((_IO uint32_t*)tempValue) |= (0X1U << EXTI_InitStruct->EXTI_LineNumber) ;

	   EXTI->FTSR &= ~(0x01 << EXTI_InitStruct->EXTI_LineNumber) ;
	   EXTI->RTSR &= ~(0x01 << EXTI_InitStruct->EXTI_LineNumber) ;



   if(EXTI_InitStruct->TriggerSelection == EXTI_Trigger_RF) {
	   EXTI->FTSR |=(0x01 << EXTI_InitStruct->EXTI_LineNumber) ;
	   EXTI->RTSR |=(0x01 << EXTI_InitStruct->EXTI_LineNumber) ;


   }

   else {
    tempValue += EXTI_InitStruct->TriggerSelection ;
    *((_IO uint32_t*)tempValue) |= (0x01 << EXTI_InitStruct->EXTI_LineNumber) ;

   }
}


   else {

		tempValue = (uint32_t)EXTI_BASE_ADR ;
        }

   }



/*
 * @brief GPIO_LineConfig configures the port and pin SYSCFG
 * @param Port Source = PORT1-G
 * @param EXTI_LineSources = pin Numbers & Line Numbers @defGrroup EXTI_LineValues
 * @param pinState GPIO pin state or reset
 */


void EXTI_LineConfig(uint8_t portSource, uint8_t EXTI_LineSource) {

	uint32_t tempValue ;
	tempValue = SYSCFG->EXTI_CR[EXTI_LineSource >> 2U] ;
	tempValue &= ~(portSource << (EXTI_LineSource & 0x3U)*4);
	tempValue = (portSource << (EXTI_LineSource & 0x3U)*4) ;
	SYSCFG->EXTI_CR[EXTI_LineSource >> 2U] = tempValue ;

}


void NVIC_EnableInterrupt(IRQ_NumberTypeDef_t IRQNumber){

	uint32_t tempValue = 0;

	tempValue = *((IRQNumber >> 5U) + NVIC_ISER0) ;
	tempValue &= ~(0x1U <<(IRQNumber & 0x1FU)) ; // 11111
	tempValue |= (0x1U <<(IRQNumber & 0x1FU)) ;
	*((IRQNumber >> 5U) + NVIC_ISER0)  = tempValue ;


}





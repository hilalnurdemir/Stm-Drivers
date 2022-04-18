/*
 * EXTI.h
 *
 *  Created on: 17 Eyl 2021
 *      Author: Hilal_Embedded
 */

#ifndef INC_EXTI_H_
#define INC_EXTI_H_

#include "STM32F407xx.h"
#include <stdio.h>


/*
 * @def_group PORT_Values
 */

#define EXTI_PortSource_GPIOA         ( (uint8_t)(0x0) )
#define EXTI_PortSource_GPIOB         ( (uint8_t)(0x1) )
#define EXTI_PortSource_GPIOC         ( (uint8_t)(0x2) )
#define EXTI_PortSource_GPIOD         ( (uint8_t)(0x3) )
#define EXTI_PortSource_GPIOE         ( (uint8_t)(0x4) )
#define EXTI_PortSource_GPIOF         ( (uint8_t)(0x5) )
#define EXTI_PortSource_GPIOG         ( (uint8_t)(0x6) )


/*
 * @def_group Line_Values
 */

#define EXTI_LineSource_0               ( (uint8_t)(0x0) )
#define EXTI_LineSource_1               ( (uint8_t)(0x1) )
#define EXTI_LineSource_2               ( (uint8_t)(0x2) )
#define EXTI_LineSource_3               ( (uint8_t)(0x3) )
#define EXTI_LineSource_4               ( (uint8_t)(0x4) )
#define EXTI_LineSource_5               ( (uint8_t)(0x5) )
#define EXTI_LineSource_6               ( (uint8_t)(0x6) )
#define EXTI_LineSource_7               ( (uint8_t)(0x7) )
#define EXTI_LineSource_8               ( (uint8_t)(0x8) )
#define EXTI_LineSource_9               ( (uint8_t)(0x9) )
#define EXTI_LineSource_10              ( (uint8_t)(0x10) )
#define EXTI_LineSource_11              ( (uint8_t)(0x11) )
#define EXTI_LineSource_12              ( (uint8_t)(0x12) )
#define EXTI_LineSource_13              ( (uint8_t)(0x13) )
#define EXTI_LineSource_14              ( (uint8_t)(0x14) )
#define EXTI_LineSource_15              ( (uint8_t)(0x15) )

/*
 * @def_group EXTI_Modes
 */

#define EXTI_MODE_Interrupt           (0x00U)
#define EXTI_MODE_Event               (0x04U)

/*
 * @def_group EXTI_Trigger modes
 */


#define EXTI_TRIGGER_Rising           (0x08U)
#define EXTI_TRIGGER_Falling          (0x0CU)
#define EXTI_Trigger_RF               (0x10U)


typedef struct {

	uint8_t EXTI_LineNumber ;
	uint8_t TriggerSelection ;     // falling or rising edge
	uint8_t EXTI_Mode ;            // event or interrupt @defgup EXTI_Mode values
	FunctionalState_t EXTI_LineCmd ;  // enable or disable


}EXTI_InitTypeDef_t ;



void EXTI_Init(EXTI_InitTypeDef_t *EXTI_InitStruct) ;

void EXTI_LineConfig(uint8_t portSource, uint8_t EXTI_LineSource) ;

void NVIC_EnableInterrupt(uint8_t IRQNumber);

#endif /* INC_EXTI_H_ */

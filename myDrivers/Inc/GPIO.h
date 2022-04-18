/*
 * GPIO.h
 *
 *  Created on: 30 Au 2021
 *      Author: Hilal_Embedded
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "STM32F407xx.h"


/*
 *
 *
 * @def_group GPIO_Pins
 */



#define GPIO_PIN_0    (uint16_t)(0x0001) // 0000 0000 0000 0001
#define GPIO_PIN_1    (uint16_t)(0x0002)
#define GPIO_PIN_2    (uint16_t)(0x0004)
#define GPIO_PIN_3    (uint16_t)(0x0008)
#define GPIO_PIN_4    (uint16_t)(0x0010)
#define GPIO_PIN_5    (uint16_t)(0x0020)
#define GPIO_PIN_6    (uint16_t)(0x0040)
#define GPIO_PIN_7    (uint16_t)(0x0080)
#define GPIO_PIN_8    (uint16_t)(0x0100)
#define GPIO_PIN_9    (uint16_t)(0x0200)
#define GPIO_PIN_10    (uint16_t)(0x0400)
#define GPIO_PIN_11    (uint16_t)(0x0800)
#define GPIO_PIN_12    (uint16_t)(0x1000)
#define GPIO_PIN_13    (uint16_t)(0x2000)
#define GPIO_PIN_14    (uint16_t)(0x4000)
#define GPIO_PIN_15    (uint16_t)(0x8000)
#define GPIO_PIN_ALL    (uint16_t)(0xFFFF)

/*
 *
 *
 * @def_group GPIO_Pin_Modes
 */


#define GPIO_MODE_INPUT   (0x0U)
#define GPIO_MODE_OUTPUT  (0X1U)
#define GPIO_MODE_AF      (0x2U)
#define GPIO_MODE_ANALOG  (0x3U)


/*
 *
 *
 * @def_group GPIO_OTYPE_Modes
 *
 */

#define GPIO_OTYPE_PP      (0x0U)
#define GPIO_OTYPE_OD      (0x1U)

/*
 *
 *
 * @def_group GPIO_PuPd_Modes
 *
 *
 */

#define GPIO_PUPD_NOPULL   (0x0U)
#define GPIO_PUPD_PULLUP   (0x1U)
#define GPIO_PUPD_PULLDOWN (0x2U)


/*
 *
 *
 * @def_group GPIO_Speed_Modes
 *
 *
 */



#define GPIO_SPEED_LOW    (0x0U)
#define GPIO_SPEED_MEDIUM (0x1U)
#define GPIO_SPEED_HIGH   (0x2U)
#define GPIO_SPEED_VERY   (0x3U)


typedef enum{
	GPIO_PIN_RESET = 0x0U ,
	GPIO_PIN_SET = !GPIO_PIN_RESET

}GPIO_PinState_t ;


typedef struct{
	uint32_t pinNumber; // Pin number @def_group GPIO_Pins
	uint32_t Mode ;     // Pin number @def_group GPIO_Pin_Modes
	uint32_t Otype ;    // Pin number @def_group GPIO_OTYPE_Modes
	uint32_t PuPd ;     // Pin number @def_group GPIO_PuPd_Modes
	uint32_t Speed ;    // Pin number @def_group GPIO_Speed_Modes
	uint32_t Altarnate ;

}GPIO_Init_TypeDef_t ;


void GPIO_WritePin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber, GPIO_PinState_t pinState) ; //write pin

GPIO_PinState_t GPIO_ReadPin(GPIO_TypeDef_t *GPIOx , uint16_t pinNumber) ; //Read pin

void GPIO_LockPin(GPIO_TypeDef_t *GPIOx , uint16_t pinNumber) ;

void GPIO_Init(GPIO_TypeDef_t *GPIOx, GPIO_Init_TypeDef_t *GPIO_Config_Struct) ;

void GPIO_Togle_Pin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber);






#endif /* INC_GPIO_H_ */

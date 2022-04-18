/*
 * GPIO.c
 *
 *  Created on: 30 Ağu 2021
 *      Author: Hilal_Embedded
 */
#include "GPIO.h"
#include "STM32F407xx.h"




void GPIO_Init(GPIO_TypeDef_t *GPIOx, GPIO_Init_TypeDef_t *GPIO_Config_Struct) {

	 uint32_t position ;
	 uint32_t fakePosition = 0 ;
	 uint32_t LastPosition = 0 ;

	 for(position=0 ; position <16 ; position++){

		 fakePosition = (0x1 << position) ;
		 LastPosition = (GPIO_Config_Struct->pinNumber) & fakePosition ;
		 if (fakePosition == LastPosition) {

			 /*MODE CONFIG*/

			 uint32_t tempValue = GPIOx->MODER ;
			 tempValue &= ~(0x3U << (position*2)) ; // clear işlemi yaptım
			 tempValue |= (GPIO_Config_Struct->Mode << (position*2)) ;

			 GPIOx->MODER = tempValue ;


			 if(GPIO_Config_Struct->Mode == GPIO_MODE_INPUT || GPIO_Config_Struct->Mode == GPIO_MODE_ANALOG ){

				 /*OUTPUT TYPE CONFIG*/


				 tempValue = GPIOx->OTYPER ;
				 tempValue &= ~(0x1U << position);
				 tempValue |= (GPIO_Config_Struct->Otype << position);
				 GPIOx->OTYPER = tempValue ;

				 /*OUTPUT SPEED CONFIG*/
				 tempValue = GPIOx->OSPEEDR ;
				 tempValue &= ~(0x3U << (position*2));
				 tempValue |= (GPIO_Config_Struct->Speed << (position*2)); // kullanicinin verdigi
				 GPIOx->OSPEEDR= tempValue ;



			 }
			 /*PUSH PULL CONFIG*/
			 tempValue = GPIOx->PUPDR ;
			 tempValue &= ~(0x3U << (position*2));
			 tempValue |= (GPIO_Config_Struct->PuPd << (position*2));
			 GPIOx->PUPDR = tempValue ;



		 }

	 }

}

/*
 * @brief GPIO_WritePin
 * @param GPIOx = GPIO board base address
 * @param GPIO pin numbers 0-15
 * @param pinState GPIO pin state or reset
 */

void GPIO_WritePin(GPIO_TypeDef_t * GPIOx, uint16_t pinNumber, GPIO_PinState_t pinState) {

	if (pinState == GPIO_PIN_SET){

		GPIOx->BSRR = pinNumber ;
	}
	else {
		GPIOx->BSRR = (pinNumber << 16U) ;
	}
}



/*
 * @brief GPIO_ReadPin
 * @param GPIOx = GPIO board base address
 * @param GPIO pin numbers 0-15
 */

GPIO_PinState_t GPIO_ReadPin(GPIO_TypeDef_t* GPIOx , uint16_t pinNumber)
{
	GPIO_PinState_t bitStatus = GPIO_PIN_RESET ;
	if((GPIOx->IDR & pinNumber) != GPIO_PIN_RESET ){
		bitStatus = GPIO_PIN_SET ;

	}
	return bitStatus ;


}

/*
 * @brief GPIO_ReadPin
 * @param GPIOx = GPIO board base address
 * @param GPIO pin numbers 0-15
 * @retVal GPIO_PinState_t
 */

void GPIO_LockPin(GPIO_TypeDef_t* GPIOx , uint16_t pinNumber) {
	uint32_t tempValue = (0x1U << 16) | pinNumber ;

	GPIOx->LCKR = tempValue ; //LCKR[16] = '1' LCKR[0-15] = DATA
	GPIOx->LCKR = pinNumber ; //LCKR[16] = '0' LCKR[0-15] = DATA
	GPIOx->LCKR = tempValue ; //LCKR[16] = '1' LCKR[0-15] = DATA

	tempValue = GPIOx->LCKR ; //read lock register
}


void GPIO_Togle_Pin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber){

	uint32_t tempODRreg = GPIOx->ODR ;
	GPIOx->ODR = ((tempODRreg & pinNumber)<<16U) | ((~tempODRreg) & pinNumber) ;


}


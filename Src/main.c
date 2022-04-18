#include "STM32F407xx.h"

static void GPIO_led();
static void GPIO_InterruptConfig();

int main(void)
{
	GPIO_led();
	//GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |GPIO_PIN_15, GPIO_PIN_RESET);
	GPIO_InterruptConfig();


	for(;;);

}

static void GPIO_led(){

	GPIO_Init_TypeDef_t GPIO_led_S = {0};

	RCC_GPIOD_CLK_ENABLE() ;
	RCC_GPIOA_CLK_ENABLE() ;
	GPIO_InterruptConfig() ;

	GPIO_led_S.pinNumber = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |GPIO_PIN_15 ;
	GPIO_led_S.Mode = GPIO_MODE_OUTPUT ;
	GPIO_led_S.Speed = GPIO_SPEED_LOW  ;
	GPIO_led_S.Otype = GPIO_OTYPE_PP   ;
	GPIO_led_S.PuPd = GPIO_PUPD_NOPULL ;

	GPIO_Init(GPIOD, &GPIO_led_S) ;

}

static void GPIO_InterruptConfig(){

	EXTI_InitTypeDef_t EXTI_InitStruct = { 0} ;
	RCC_SYSCFG_CLK_ENABLE() ;


	EXTI_LineConfig(EXTI_PortSource_GPIOC, EXTI_LineSource_10) ;

	EXTI_InitStruct.EXTI_LineCmd = ENABLE ;
	EXTI_InitStruct.EXTI_LineNumber = EXTI_LineSource_10 ;
	EXTI_InitStruct.EXTI_Mode = EXTI_MODE_Interrupt ;
	EXTI_InitStruct.EXTI_LineCmd = EXTI_TRIGGER_Rising ;

    EXTI_Init(&EXTI_InitStruct) ;


}

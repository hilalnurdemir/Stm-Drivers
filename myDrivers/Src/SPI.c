/*
 * SPI.c
 *
 *  Created on: 21 Eyl 2021
 *      Author: Hilal_Embedded
 */


#include "SPI.h"

/*
 * @brief SPI_Init configures SPI periph
 * @param SPI_Handle = User congig structure
 * @retVal 0
 */


void SPI_Init(SPI_HandleTypeDef_t *SPI_Handle){

	uint32_t tempValue = 0 ;


	tempValue = SPI_Handle->Instance->CR1 ;
	tempValue |= (SPI_Handle->Init.BoundRate) | (SPI_Handle->Init.CPHA ) | (SPI_Handle->Init.CPOL) |(SPI_Handle->Init.DFFFormat)   \
			  |(SPI_Handle->Init.Mode) |(SPI_Handle->Init.FrameFormat) |(SPI_Handle->Init.BusConfig)  | (SPI_Handle->Init.SSM_Cmd);

	SPI_Handle->Instance->CR1 = tempValue;


}

void SPI_PeriphCmd(SPI_HandleTypeDef_t *SPI_Handle, FunctionalState_t stateof_SPI) {

	if (stateof_SPI == ENABLE) {

		SPI_Handle->Instance->CR1 |= (0x1U << SPI_CR1_SPEN) ;

	}

	else {

		SPI_Handle->Instance->CR1 &= ~(0x1U << SPI_CR1_SPEN) ;

	}


}
void SPI_TransmitData(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pData, uint16_t sizeofData){

	if(SPI_Handle->Init.FrameFormat == SPI_DFF_16BITS){

		while(sizeofData > 0) {

			if((SPI_Handle->Instance->SR >> 1U ) & 0x1U) //TXE flag control
				{
				SPI_Handle->Instance->DR = *((uint16_t*)pData) ;//16 bit type cast ;
					pData+= sizeof(uint16_t) ;
					sizeofData-= 2 ;

			}

		}


	}

	else{
		while(sizeofData > 0) {
	       if(SPI_GetFlagStatus(SPI_Handle, SPI_TxE_FLAG)) //TXE flag control
			   {
	    	   SPI_Handle->Instance->DR = *pData ;
	    	   pData++ ;
	    	   sizeofData-- ;
					}
				}

	}
	while(SPI_GetFlagStatus(SPI_Handle, SPI_Busy_FLAG)) ; //wait for BUSY flag SPI_Handle->Instance->SR >> 7U) & 0x1U




}
SPI_FlagStatus_t SPI_GetFlagStatus(SPI_HandleTypeDef_t *SPI_Handle, uint16_t SPI_Flag){



	return (SPI_Handle->Instance->SR & SPI_Flag) ? SPI_FLAG_SET : SPI_FLAG_RESET ;


}



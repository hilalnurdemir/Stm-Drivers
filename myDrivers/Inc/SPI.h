/*
 * SPI.h
 *
 *  Created on: 21 Eyl 2021
 *      Author: Hilal_Embedded
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#include "STM32F407xx.h"

/*
 *
 * @def_group Boundrate Values
 */

#define SPI_BAUNDRATE_DIV2          ((uint32_t)(0x00000000)   )
#define SPI_BAUNDRATE_DIV4          ((uint32_t)(0x00000008)   )
#define SPI_BAUNDRATE_DIV8          ((uint32_t)(0x00000010)   )
#define SPI_BAUNDRATE_DIV16         ((uint32_t)(0x00000018)   )
#define SPI_BAUNDRATE_DIV32         ((uint32_t)(0x00000020)   )
#define SPI_BAUNDRATE_DIV64         ((uint32_t)(0x00000028)   )
#define SPI_BAUNDRATE_DIV128        ((uint32_t)(0x00000030)   )
#define SPI_BAUNDRATE_DIV256        ((uint32_t)(0x00000038)   )


/*
 *
 * @def_group CPHA Values
 */

#define SPI_CHPA_FIRST              ((uint32_t)(0x00000000)   )
#define SPI_CHPA_SECOND             ((uint32_t)(0x00000001)   )

/*
 *
 * @def_group CPHA Values
 */

#define SPI_CPOL_LOW               ((uint32_t)(0x00000000)   )
#define SPI_CPOL_HIGH              ((uint32_t)(0x00000002)   )     //1.Biti 1

/*
 *
 * @def_group DFF Values
 */

#define SPI_DFF_8BITS              ((uint32_t)(0x00000000)   )
#define SPI_DFF_16BITS             ((uint32_t)(0x00000800)   )     //11.Biti 1

/*
 *
 * @def_group Mode Values
 */

#define SPI_Mode_Slave              ((uint32_t)(0x00000000)   )
#define SPI_Mode_Master             ((uint32_t)(0x00000004)   )     //2.Biti 1

/*
 *
 * @def_group FF_Values
 */

#define SPI_FrameFormat_MSB          ((uint32_t)(0x00000000)   )
#define SPI_FrameFormat_LSB          ((uint32_t)(0x00000080)   )     //7.Biti 1

/*
 * @BusConfig_Value
 */
#define SPI_Bus_FullDuplex           ((uint32_t)(0x00000000)   )
#define SPI_ReceiveOnly              ((uint32_t)(0x00000400)   )      // 10. bit 1
#define SPI_Bus_HalfDuple_T          ((uint32_t)(0x0000C000)   )      // 15 ve 14. bit 1
#define SPI_Bus_HalfDuple_R          ((uint32_t)(0x00008000)   )      // 15. bit 1

/*
 *
 *
 *@def_group SSM_Values
 */

#define SPI_SSM_DISABLE             ((uint32_t)(0x00000000)   )
#define SPI_SSM_ENABLE              ((uint32_t)(0x00000300)   )       // 8 ve 9. bit 1


typedef enum{
	SPI_FLAG_RESET = 0x0U ,
	SPI_FLAG_SET = !SPI_FLAG_RESET

}SPI_FlagStatus_t ;

typedef struct{

	uint32_t Mode ;             // Mode values @deg_group Mode_values
	uint32_t CPHA ;             // CPHA values @deg_group CHPA_values
	uint32_t CPOL ;             // CPOL values @deg_group CPOL_values
	uint32_t BoundRate ;        // SPI boundrate value @deg_group SPI_BoundRate
	uint32_t SSM_Cmd ;          // SSM values for SPI @def_group SSM_Values
	uint32_t DFFFormat ;        // DFF value @deg_group DFF_Values
	uint32_t FrameFormat ;      // Frame format values @deg_group FF_Values
    uint32_t BusConfig ;        // BusConfig_value @BusConfig_Value

}SPI_InitTypeDef_t ;

typedef struct{

	SPI_TypeDef_t *Instance ;
	SPI_InitTypeDef_t Init ;

}SPI_HandleTypeDef_t ;

void SPI_Init(SPI_HandleTypeDef_t *SPI_Handle);
void SPI_PeriphCmd(SPI_HandleTypeDef_t *SPI_Handle, FunctionalState_t stateof_SPI) ;
void SPI_TransmitData(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pData, uint16_t sizeofData);
SPI_FlagStatus_t SPI_GetFlagStatus(SPI_HandleTypeDef_t *SPI_Handle, uint16_t SPI_Flag);


#endif /* INC_SPI_H_ */

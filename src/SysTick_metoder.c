/**
  ******************************************************************************
  * @file    SysTick_metoder.c
  * @author  Sivert Sliper and Stian Sørensen
  * @version V1.0
  * @date    08-February-2016
  * @brief   This file contains all the functions prototypes for the SysTick
  *          timer.
  *
  ******************************************************************************
  */

/* Include---- ------------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "stm32f30x_gpio.h"
#include "core_cm4.h"
#include "stm32f30x_dma.h"
/* Global variables -------------------------------------------------------------------*/
#include "extern_decl_global_vars.h"


/* Function declarations ---------------------------------------------------------------*/
void SysTick_init(void);
void SysTick_Handler(void);

uint8_t USART_getRxMessage(void);
uint8_t USART_getNewBytes(void);
void USART_transmit(uint8_t data);
uint8_t CAN_getRxMessages(void);
uint8_t CAN_getByteFromMessage(uint8_t filter_number, uint8_t byte_number);
void CAN_transmitByte(uint16_t StdId, uint8_t data);
void accelerometer_readValue(void);
int16_t accelerometer_getData(uint8_t axis);
void CAN_transmit_AN_RAW(void);

void USART_timestamp_transmit(uint8_t timestamp);
void USART_datalog_transmit(uint8_t header, uint16_t data);
uint8_t ADC_getValues(void);
uint16_t ADC_getChannel(uint8_t channel);
/* Function definitions ----------------------------------------------------------------*/

/**
 * @brief  Configures the SysTick timer for 100 Hz interrupt frequency.
 * @param  None
 * @retval None
 */
void SysTick_init(void) {
	NVIC_SetPriority(SysTick_IRQn, 1);
	SysTick->CTRL = 0; /* Disable SysTick */
	SysTick->LOAD = 72000000/1000;  // 10 msek avbruddsintervall.
	SysTick->VAL = 0;
	SysTick->CTRL = (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk
			| SysTick_CTRL_CLKSOURCE_Msk);
} // end Systick_init()


/**
 * @brief  Configures the CAN-Controller peripheral for 500 kbps communication.
 * 		   Also configures Rx filters according to ID's specified in "can_metoder.h"
 * @param  None
 * @retval None
 */

uint8_t teller = 0;
uint16_t val = 0;
uint32_t valVoltage = 0;
uint8_t timeStamp = 0;

void SysTick_Handler(void){
	teller++;
	/* Measure acceleration and send the data via CAN */

	/* Check for new message on CAN and update LEDs */
	if(CAN_getRxMessages()>0){
		GPIOE->ODR ^= (1u << CAN_getByteFromMessage(2,0)) << 8;
	} // end if



	if(teller>100){
		if ( DMA_GetFlagStatus(DMA1_FLAG_TE1)){
			GPIOE->ODR ^= 1u << 10 ;
			DMA_ClearFlag(DMA1_FLAG_TE1);
		}

		//ADC_StartConversion(ADC1);

		if ((ADC1->ISR & ADC_ISR_OVR) > 0){
			GPIOE->ODR^= 1u << 11 ;
			ADC1->ISR &= ADC_ISR_OVR;
		}

		GPIOE->ODR ^= SYSTICK_LED << 8;

//		accelerometer_readValue();
		if(ADC_getValues() & 0x3F){
			/* 'G' - AN_IN_1, 'H' - AN_IN_2, 'I' - CUR_IN_1
			 * 'J' - CUR_IN_2, 'K' - Int_temp, 'L' - leak_det
			 */
			USART_datalog_transmit('G', ADC_getChannel(5));
			USART_datalog_transmit('H', ADC_getChannel(2));
			USART_datalog_transmit('I', ADC_getChannel(3));
			USART_datalog_transmit('J', ADC_getChannel(4));
			USART_timestamp_transmit(timeStamp++);
		} // end if

		teller = 0;
//		CAN_transmitAcceleration(&accelerometer_data);
	} // end if

} // end Systick_Handler()

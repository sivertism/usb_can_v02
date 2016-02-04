/**
 **************************************************************************************
 * @file    ADC_metoder.c
 * @author  Sivert Sliper, Stian Soerensen
 * @version V1.0
 * @date    3-February-2016
 * @brief   This file contains all the functions for the ADC peripheral.
 **************************************************************************************
 */

/* Include------------------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_adc.h"
#include "stm32f30x_misc.h"
#include "math.h"
#include "ADC_metoder.h"

/* Global variables --------------------------------------------------------------------*/
#include "extern_decl_global_vars.h"


/* Function declarations ---------------------------------------------------------------*/
void ADC_init(void);
void ADC1_2_IRQHandler(void);
void ADC4_IRQHandler(void);
uint8_t ADC_getValues(void);
uint16_t ADC1_getChannel(uint8_t channel);
uint16_t ADC4_getChannel(uint8_t channel);

/* Function definitions ----------------------------------------------------------------*/

/**
 * @brief  	A interrupt request is generated when ADC1 is done converting a sample.
 * 			The ADC is triggered TIM2. Saves the voltage in mV in a buffer indexed by
 * 			the channels position in the sequencer.
 * @param  	None
 * @retval 	None
 */
void ADC1_2_IRQHandler(void){
	if(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)){
		GPIOF->ODR = GPIO_Pin_2;
		ADC1_buffer[channel_counter] = (30000*ADC_GetConversionValue(ADC1))/4096;
		/* Indicate to the main-loop that there is a new measurement available.*/
		new_values |= (1u << channel_counter);
		GPIOE->ODR ^= (STATUS_LED6 << 8);
		/* Increment the channel counter */
		channel_counter++;
		if (channel_counter>3) channel_counter = 0;
		ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
		GPIOF->ODR = 0;
	}
	if(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOS)){
		GPIOE->ODR ^= (UART_TX_LED << 8);
		ADC_ClearFlag(ADC1, ADC_FLAG_EOS);
	}
	/* Reset ADC_FLAG_EOC to clear the interrupt request.*/
} // end ADC3_IRQHandler()

/**
 * @brief  	A interrupt request is generated when ADC4 is done converting a sample.
 * 			The ADC is triggered by TIM2. Saves the voltage in mV in ADC4_conv_val.
 * @param  	None
 * @retval 	None
 */
void ADC4_IRQHandler(void){
	ADC4_conv_val = (30000*ADC_GetConversionValue(ADC4))/4096;
	/* Indicate to the main-loop that there is a new measurement available.*/
	new_values |= (1u << 4);
	GPIOE->ODR ^= (STATUS_LED7 << 8);
	/* Reset ADC_FLAG_EOC to clear the interrupt request.*/
	ADC_ClearFlag(ADC4, ADC_FLAG_EOC);
} // end ADC3_IRQHandler()

/**
 * @brief  	Initializes the following ADC-modules. They are triggered by a timer module
 *			and an interrupt is generated when the ADC's are finished sampling and
 *			converting.
 *				ADC channels:
 *				PIN:		CHANNEL:		DESCRIPTION			TRIGGER:
 *				--------------------------------------------------------
 *				PB12		ADC4_IN3		AN_IN1				TIM2
 *				PC0			ADC1_IN6		LEAKAGE DETECTION
 *				PC1			ADC1_IN7		AN_IN2
 *				PC2			ADC1_IN8		CUR_IN1
 *				PC3			ADC1_IN9		CUR_IN2
 *				--------------------------------------------------------
 * @param  None
 * @retval None
 */
void ADC_init(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	/* Clock setup **********************************************************************/
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div1); // Set clock divider
	RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div1); // Set clock divider
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC34, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	/* GPIO setup ***********************************************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; // Analog mode
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // No internal pull up/down resistor.
	GPIO_Init(GPIOB, &GPIO_InitStructure); // Download settings to GPIOB registers.

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure); // Download settings to GPIOC registers

	/* Calibration **********************************************************************/
	ADC_VoltageRegulatorCmd(ADC1, ENABLE); // Activate voltage regulator
	ADC_VoltageRegulatorCmd(ADC4, ENABLE); // Activate voltage regulator

	/* Wait 10 microseconds for the voltage regulator to finish starting up */
	volatile uint16_t i = 358; // 10us/(2*14ns) = 358 iterations
	while(i-->0);

	/* Using single-mode calibration on ADC1.*/
	ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1); // Start calibration
	while(ADC_GetCalibrationStatus(ADC1) != RESET); // wait

	/* Using single-mode calibration on ADC4.*/
	ADC_SelectCalibrationMode(ADC4, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC4); // Start calibration
	while(ADC_GetCalibrationStatus(ADC4) != RESET); // wait


	/* Common structure *****************************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;
	// ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 100;
	ADC_CommonInit(ADC4, &ADC_CommonInitStructure);
	ADC_CommonInit(ADC1, &ADC_CommonInitStructure);

	/* ADC setup ************************************************************************/

	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
	/* 12 bit resolution => (3.0-0)/2^12 = 732 uV per LSb  */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;

	/* TIM2 will trigger sampling and converting for all ADC channels. */
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_RisingEdge;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
	ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;

	/* Only one channel will be used in ADC4, while 4 will be used in ADC1.
	 * TIM2_TRGO is mapped to external trigger event 10 for ADC1 and 7 for ADC4.*/
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_11;
	ADC_InitStructure.ADC_NbrOfRegChannel = 4;
	ADC_Init(ADC1, &ADC_InitStructure); /* Download settings to ADC1 registers */
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_7;
	ADC_InitStructure.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADC4, &ADC_InitStructure); /* Download settings to ADC4 registers*/

	/* Interrupt settings ***************************************************************/

	/* Interrupt handler settings */
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0; // Will not preempt SysTick
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0; // Lower prio than SysTick
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_Init(&NVIC_InitStruct);
	NVIC_InitStruct.NVIC_IRQChannel = ADC4_IRQn;
	NVIC_Init(&NVIC_InitStruct);

	/* Interrupt request settings */
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	ADC_ITConfig(ADC1, ADC_IT_EOS, ENABLE);
	ADC_ITConfig(ADC4, ADC_IT_EOC, ENABLE);

	/* ADC Channal sequencing ***********************************************************/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_601Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_601Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 2, ADC_SampleTime_601Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 4, ADC_SampleTime_601Cycles5);

	ADC_RegularChannelConfig(ADC4, ADC_Channel_3, 1, ADC_SampleTime_19Cycles5);

	/* Activaton ************************************************************************/
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC4, ENABLE);

	/* Wait for ready flags */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
	while(!ADC_GetFlagStatus(ADC4, ADC_FLAG_RDY));

	/* Start first conversion */
	ADC_StartConversion(ADC1);
	ADC_StartConversion(ADC4);
} // end ADC_oppstart()

/**
 * @brief  	Returns a value that indicates which ADC-channels has new data.
 * @param  	None
 * @retval 	uint8_t newValues, a byte where each bit represents new data from its
 * 			respective ADC channel according to:
 *
 * 				BIT:	PIN:		CHANNEL:		DESCRIPTION			TRIGGER:
 *				----------------------------------------------------------------
 *				0		PB12		ADC4_IN3		AN_IN1				TIM2
 *				1		PC0			ADC12_IN6		LEAKAGE DETECTION
 *				2		PC1			ADC12_IN7		AN_IN2
 *				3		PC2			ADC12_IN8		CUR_IN1
 *				4		PC3			ADC12_IN9		CUR_IN2
 *				----------------------------------------------------------------
 */
uint8_t getValues(void){
	return new_values;
}

/**
 * @brief  	Returns the voltage level of the selected ADC1 channel.
 * @param  	None
 * @retval 	The voltage of the selected channel in 100 uV.
 */
uint16_t ADC1_getChannel(uint8_t channel){
	if (channel > 3) return 0;
	new_values &= ~(1u << channel);
	return ADC1_buffer[channel];
}

/**
 * @brief  	Returns the voltage level of the selected ADC4 channel.
 * @param  	None
 * @retval 	The voltage of the selected channel in 100 uV.
 */
uint16_t ADC4_getChannel(uint8_t channel){
	if (channel >= 1) return 0;
	new_values &= ~(1u << 4);
	return ADC4_conv_val;
}

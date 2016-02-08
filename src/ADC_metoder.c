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
#include "stm32f30x_dma.h"


/* Global variables --------------------------------------------------------------------*/
#include "extern_decl_global_vars.h"


/* Private function prototypes ---------------------------------------------------------*/
void ADC1_2_IRQHandler(void);
void ADC4_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);
uint8_t ADC_getValues(void);
uint16_t ADC1_getChannel(uint8_t channel);
uint16_t ADC4_getChannel(uint8_t channel);

/* Private variables -------------------------------------------------------------------*/
static uint8_t new_values = 0;
static uint8_t channel_counter = 0;
static uint16_t ADC_buffer[6] = {0};


/* Function definitions ----------------------------------------------------------------*/

/**
 * @brief  	A interrupt request is generated when DMA1_Channel1 is finished saving 5
 * 			new ADC-values to memory.
 * @param  	None
 * @retval 	None
 */
void DMA1_Channel1_IRQHandler(){
	/* Indicate to SysTick that a new set of data is ready */
	new_values |= 0x1F;
	DMA_ClearITPendingBit(DMA1_IT_TC1);
	DMA_ClearFlag(DMA1_FLAG_TC1);
}

/**
 * @brief  	A interrupt request is generated when DMA2_Channel1 is finished saving 1
 * 			new ADC-values to memory.
 * @param  	None
 * @retval 	None
 */
void DMA2_Channel2_IRQHandler(){
	/* Indicate to SysTick that a new set of data is ready */
	new_values |= (1u << 5);
	DMA_ClearITPendingBit(DMA2_IT_TC2);
	DMA_ClearFlag(DMA2_FLAG_TC2);
}

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
		ADC_buffer[channel_counter] = (30000*ADC_GetConversionValue(ADC1))/4096;
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
	ADC_buffer[6] = (30000*ADC_GetConversionValue(ADC4))/4096;
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
 *				PF4			ADC1_IN5		INT. TEMPERATURE	TIM2
 *				PC0			ADC1_IN6		LEAKAGE DETECTION	TIM2
 *				PC1			ADC1_IN7		AN_IN2				TIM2
 *				PC2			ADC1_IN8		CUR_IN1				TIM2
 *				PC3			ADC1_IN9		CUR_IN2				TIM2
 *				--------------------------------------------------------
 * @param  None
 * @retval None
 */
void ADC_init(void){

	/* TypeDef declarations *************************************************************/
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	NVIC_InitTypeDef NVIC_InitStruct;

	/* Clock setup **********************************************************************/
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div4); // Set clock divider
	RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div4); // Set clock divider
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC34, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	
	/* GPIO setup ***********************************************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; // Analog mode
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // No internal pull up/down resistor.
	GPIO_Init(GPIOB, &GPIO_InitStructure); // Download settings to GPIOB registers.

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure); // Download settings to GPIOC registers

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	/* Calibration **********************************************************************/
	/* Reset ADC registers to their default values. */
	ADC_DeInit(ADC1);
	ADC_DeInit(ADC4);

	/* Activate voltage regulators */
	ADC_VoltageRegulatorCmd(ADC1, ENABLE);
	ADC_VoltageRegulatorCmd(ADC4, ENABLE);

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
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 10;
	ADC_CommonInit(ADC4, &ADC_CommonInitStructure);
	ADC_CommonInit(ADC1, &ADC_CommonInitStructure);

	/* ADC setup ************************************************************************/
	/*
	 *	Resolution:			12 bit -> 732 uV per LSb.
	 *	Conversion mode:	Triggered from TIM2_TRGO.
	 *	DMA:				DMA request enabled for circular DMA mode.
	 *
	 */

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_RisingEdge;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Enable;
	ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;

	/* Only one channel will be used in ADC4, while 4 will be used in ADC1.
	 * TIM2_TRGO is mapped to external trigger event 11 for ADC1 and 7 for ADC4.*/
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_11;
	ADC_InitStructure.ADC_NbrOfRegChannel = 5;
	ADC_Init(ADC1, &ADC_InitStructure); /* Download settings to ADC1 registers */
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_7;
	ADC_InitStructure.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADC4, &ADC_InitStructure); /* Download settings to ADC4 registers*/
	ADC1->CFGR |= 0b11; // Making sure DMA access is enabled for ADC1

	/* Interrupt settings ***************************************************************/
	/*
	 * No interrupts are used for the ADC modules, the only interrupt will come from
	 * the DMA module(when all DMA-transfers are complete).
	 */

	/* Interrupt handler settings */
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannel = ADC1_2_IRQn;
//	NVIC_Init(&NVIC_InitStruct);
//	NVIC_InitStruct.NVIC_IRQChannel = ADC4_IRQn;
//	NVIC_Init(&NVIC_InitStruct);
//
//	/* Interrupt request settings */
//	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
//	ADC_ITConfig(ADC4, ADC_IT_EOC, ENABLE);

	/* ADC Channal sequencing ***********************************************************/
	/*
	 * ADC1 will sample channels 5-9, ADC4 will sample channel 3.
	 * See the table over ADC_init() for pin mapping and functions.
	 * The sampling time is set to 61.5*4(prescaler) = 246 processor cycles to allow
	 * the DMA to finish data transfers before the next channel is sampled.
	 */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_61Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 2, ADC_SampleTime_61Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 3, ADC_SampleTime_61Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 4, ADC_SampleTime_61Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 5, ADC_SampleTime_61Cycles5);
	ADC_RegularChannelConfig(ADC4, ADC_Channel_3, 1, ADC_SampleTime_61Cycles5);


	/* Activaton ************************************************************************/
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC4, ENABLE);

	/* Wait for ready flags */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
	while(!ADC_GetFlagStatus(ADC4, ADC_FLAG_RDY));



	/* DMA Controller setup *************************************************************/

	/* DMA1 Channel 1, connected to ADC1. */
	DMA_DeInit(DMA1_Channel1);
	DMA1_Channel1->CPAR = ((uint32_t)&(ADC1->DR)); 	// Source register (periph)
	DMA1_Channel1->CMAR = (uint32_t)&ADC_buffer[0]; // Destination register (memory)
	DMA1_Channel1->CNDTR = 5; 						// The number of data to be transfered

	DMA1_Channel1->CCR = (DMA_CCR_PL_1) 	// Medium priority.
						|(DMA_CCR_CIRC) 	// DMA Circular mode enabled
						|(DMA_CCR_MINC) 	// Memory pointer automatic increment enabled.
						|(DMA_CCR_PSIZE_0) 	// Periph. data size = 16 bit.
						|(DMA_CCR_MSIZE_0) 	// Memory data size = 16 bit.
						|(DMA_CCR_TCIE);	// Enable interrupt on Transfer Complete
	//DMA1_Channel1->CCR |= 0b010010110100010;

	/* DMA2 Channel 2, connected to ADC1. */
	DMA_DeInit(DMA2_Channel2);
	DMA2_Channel2->CPAR = ((uint32_t)&(ADC4->DR)); 	// Source register (periph)
	DMA2_Channel2->CMAR = (uint32_t)&ADC_buffer[5]; // Destination register (memory)
	DMA2_Channel2->CNDTR = 1; 						// The number of data to be transfered

	DMA2_Channel2->CCR = (DMA_CCR_PL_1) 	// Medium priority.
						|(DMA_CCR_CIRC) 	// DMA Circular mode enabled
						|(DMA_CCR_MINC) 	// Memory pointer automatic increment enabled.
						|(DMA_CCR_PSIZE_0) 	// Periph. data size = 16 bit.
						|(DMA_CCR_MSIZE_0) 	// Memory data size = 16 bit.
						|(DMA_CCR_TCIE);	// Enable interrupt on Transfer Complete

	/* Activating the DMA channels */
	DMA1_Channel1->CCR |= DMA_CCR_EN;
	DMA2_Channel2->CCR |= DMA_CCR_EN;

	/* Enable DMA interrup handler */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_Init(&NVIC_InitStruct);
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	NVIC_Init(&NVIC_InitStruct);

	/* Enable DMA request from ADC1 and ADC4 */
	ADC_DMACmd(ADC1, ENABLE);
	ADC_DMACmd(ADC4, ENABLE);

	/* Start first conversion ***********************************************************/
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
 *				1		PF4			ADC1_IN5		INT. TEMPERATURE	TIM2
 *				2		PC0			ADC1_IN6		LEAKAGE DETECTION	TIM2
 *				3		PC1			ADC1_IN7		AN_IN2				TIM2
 *				4		PC2			ADC1_IN8		CUR_IN1				TIM2
 *				5		PC3			ADC1_IN9		CUR_IN2				TIM2
 *				----------------------------------------------------------------
 */
uint8_t ADC_getValues(void){
	return new_values;
}

/**
 * @brief  	Returns the voltage level of the selected ADC1 channel.
 * @param  	None
 * @retval 	The voltage of the selected channel in 100 uV.
 */
uint16_t ADC_getChannel(uint8_t channel){
	if (channel > 5) return 0;
	new_values &= ~(1u << channel);
	return ADC_buffer[channel];
}

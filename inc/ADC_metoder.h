/**
  **************************************************************************************
  * @file    ADC_metoder.h
  * @author  Sivert Sliper, Stian Soerensen
  * @version V01
  * @date    03-February-2016
  * @brief   This file contains local variables and macros for ADC_metoder.c
  **************************************************************************************
  */

/* Macro -----------------------------------------------------------------------*/


/* Extern function prototypes ----------------------------------------------------------*/
void ADC_init(void);
uint8_t ADC_getValues(void);
uint16_t ADC_getChannel(uint8_t channel);

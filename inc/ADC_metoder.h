/**
  **************************************************************************************
  * @file    ADC_metoder.h
  * @author  Sivert Sliper, Stian Soerensen
  * @version V01
  * @date    03-February-2016
  * @brief   This file contains local variables and macros for ADC_metoder.c
  **************************************************************************************
  */

/* Private macro -----------------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------***********/
uint8_t new_values = 0;
uint8_t channel_counter = 0;
uint16_t ADC1_buffer[4] = {0};
uint16_t ADC4_conv_val;

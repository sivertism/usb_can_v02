/**
  ******************************************************************************
  * @file    GPIO_metoder.c
  * @author
  * @version V1.0
  * @date    3-February-2016
  * @brief   This file contains all the functions for the GPIO peripheral.
  ******************************************************************************
  */

/* Inklusjoner */
#include "stm32f30x.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"

/* Funksjonsprototyper */
void GPIO_init(void);

////////////////////////////////////////////////////////////////////////////////////////
/* Funksjonsdeklarasjoner */
////////////////////////////////////////////////////////////////////////////////////////

/* GPIO_init()
 @Beskrivelse
 	Initialiserer GPIO-modul for utgang til kompassdioder, samt en utgang (PC6) for
 	avlusingsformål.

 @Inngangsvariabler:
 	-
*/
void GPIO_init(void){
	/* Gir klokketilgang til GPIOC og GPIOE */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);

	/* Innstillinger */
	GPIO_InitTypeDef GPIO_init;
	GPIO_init.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_init.GPIO_OType = GPIO_OType_PP;
	GPIO_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_init.GPIO_Speed = GPIO_Speed_Level_1;

	/* kompassdioder for visuell synkronisering */
	GPIO_init.GPIO_Pin = 0xFF00; // Pin 8-15
	GPIO_Init(GPIOE, &GPIO_init);

	 /*  Testpinne  */
	 GPIO_init.GPIO_Pin = GPIO_Pin_2;
	 GPIO_Init(GPIOF, &GPIO_init);
} // end GPIO_init()

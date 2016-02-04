/*
 * File: init.c
 *
 * Brief: Contains function(s) for calling initialization functions
 */

/* Funtion Prototypes --------------------------------------------------------*/
void init(void);
void CAN_Config(void);
void GPIO_init(void);
void SysTick_init(void);
void USART_init();
void accelerometer_init(void);
void TIM4_init(void);
void TIM2_init(void);
void ADC_init();


/* Funtion Definitions -------------------------------------------------------*/

void init(void){
	GPIO_init();
	USART_init();
	CAN_Config();
	SysTick_init();
	accelerometer_init();
	ADC_init();
	TIM4_init();
	TIM2_init();
}

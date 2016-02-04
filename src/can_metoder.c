/**
  **************************************************************************************
  * @file    CAN_metoder.c
  * @author  Sivert Sliper, Stian Soerensen
  * @version V1.0
  * @date    3-February-2016
  * @brief   This file contains all the functions for the CAN peripheral.
  **************************************************************************************
  */

/* Include------------------------------------------------------------------------------*/
#include "can_metoder.h"

/* Global variables --------------------------------------------------------------------*/
#include "extern_decl_global_vars.h"


/* Function declarations ---------------------------------------------------------------*/
void CAN_Config(void);
void Init_RxMes(CanRxMsg *RxMessage);
void confirm_message(void);
void CAN_IT_Config(void);
uint8_t CAN_getByteFromMessage(uint8_t filter_number, uint8_t byte_number);
uint8_t CAN_getRxMessages(void);
uint16_t ADC1_getChannel(uint8_t channel);
uint16_t ADC4_getChannel(uint8_t channel);
void CAN_transmit_AN_RAW(void);

/* Function definitions ----------------------------------------------------------------*/

/**
 * @brief  Configures the CAN-Controller peripheral for 500 kbps communication.
 * 		   Also configures Rx filters according to ID's specified in "can_metoder.h"
 * @param  None
 * @retval None
 */
void CAN_Config(void){
	GPIO_InitTypeDef  		GPIO_InitStructure;
	CAN_InitTypeDef       	CAN_InitStructure;
	CAN_FilterInitTypeDef 	CAN_FilterInitStructure;
	NVIC_InitTypeDef		NVIC_InitStructure;


	/* Enable clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
	/* CAN GPIOs configuration *********************************************************/

	/* Connect CAN pins to AF7, ref. User manual UM1581 p. 111*/
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_7); //D , PINSOURCE 0, CAN1
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_7); //D , PINSOURCE 1, CAN1

	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //set to alternate function
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* CAN configuration ***************************************************************/

	/* CAN register init */
	CAN_DeInit(CAN1);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_4tq;

	/* CAN Baudrate = 500 kbps (CAN clocked at 9 MHz)
	 * se kapittel 4.1.3 i oppgaven og ref.manual RM0316 seksjon 31.7 */
	CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_6tq;
	CAN_InitStructure.CAN_Prescaler = 4; // 36MHz/4 = 9 MHz

	/* Status LED's */
	GPIOE->ODR = 1u << 8;

	/* Initalize the CAN controller and flash LED's according to success/fail */
	uint8_t can_success = CAN_Init(CAN1, &CAN_InitStructure);
	if (can_success == CAN_InitStatus_Success){
		GPIOE->ODR = 0xFF << 8; // All LED's on if success.
	} else {
		GPIOE->ODR = 0x44 << 8; // 2 LED's on if fail.
	}
	uint32_t pausetimer = 0xFFFFF;
	while(pausetimer-->0); //small pause to check LED's


	/* CAN filter init *****************************************************************/
	CAN_FilterInitStructure.CAN_FilterNumber = 0; // [0...13]
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;

	/* Setting hardware filtering of incomming messages. Using 16-bit list mode
	 * allows 4 11-bit ID's to be compared. The mask-filters are also used as
	 *  ID's.
	 *
	 * => 4 ID's per filter => maximum 4x14 = 56 message ID's total.
	 *
	 * Mapping: StdID[10:0]-RTR-IDE-EXID[17-15] ref. figure 391 RM0316.
	 * */

	/*Filter Match Index 0*/
	CAN_FilterInitStructure.CAN_FilterIdLow = 		  (CAN_RX_FILTER_0 << 5) \
			| (CAN_RTR_DATA << 4)	 \
			| (CAN_ID_STD << 3);
	/*Filter Match Index 1*/
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 	  (CAN_RX_FILTER_1 << 5) \
			| (CAN_RTR_DATA << 4)	 \
			| (CAN_ID_STD << 3);

	/*Filter Match Index 2*/
	CAN_FilterInitStructure.CAN_FilterIdHigh = 		  (CAN_RX_FILTER_2 << 5) \
			| (CAN_RTR_DATA << 4)	 \
			| (CAN_ID_STD << 3); // Filt. no. 2

	/*Filter Match Index 3*/
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 	  (CAN_RX_FILTER_3 << 5) \
			| (CAN_RTR_DATA << 4)	 \
			| (CAN_ID_STD << 3);


	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0; // Rx-buffer
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;

	CAN_FilterInit(&CAN_FilterInitStructure);

	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

	/* NVIC configuration **************************************************************/
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Initialize a standard transmit message */

	TxMsg.IDE = CAN_ID_STD;
	TxMsg.StdId = 0x301;
	TxMsg.ExtId = 0x00;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.DLC = 1;
	TxMsg.Data[0] = 0x0;
}

/**
 * @brief  	This function handles CAN1 RX0 request by storing the received message
 * 			in an array according to the filter match index and message size.
 *
 * 			Received messages are indicated by flipping LED 3.
 *
 * 			The interrupt pending bit is automatically cleared by hardware.
 * @param  	None
 * @retval 	None
 */
void USB_LP_CAN1_RX0_IRQHandler(void){
	CAN_Receive(CAN1, CAN_FIFO0, &RxMsg);
	uint8_t bytes = RxMsg.DLC;

	if (bytes == 0) return; // Return if message is empty.

	while(bytes-->0){
		Rx_Array[RxMsg.FMI][bytes-1] = RxMsg.Data[bytes-1];
	}

	/* Increment message received counter */
	rx_messages++;

	/* Indicating message received.*/
	GPIOE->ODR ^= CAN_RX_LED << 8; // Flip receive-LED.
}

/**
 * @brief  Returns the number of unprocessed messages.
 * @param  None
 * @retval The number of unprocessed messages (uint8_t).
 */
uint8_t CAN_getRxMessages(void){
	return rx_messages;
}

/**
 * @brief  	Returns the specified byte from the Rx array.
 * @param  	uint8_t filter_number: 	Specify which filter index the wanted
 * 									message belongs to.
 * @param	uint8_t byte_number:	Specify where the wanted byte is in the
 * 									data field of the received message.
 * @retval 	The specified byte.
 */
uint8_t CAN_getByteFromMessage(uint8_t filter_number, uint8_t byte_number){
	return Rx_Array[filter_number][byte_number];
}

/**
 * @brief  Transmit byte
 * @param  None
 * @retval The number of unprocessed messages (uint8_t).
 */
void CAN_transmitByte(uint16_t StdId, uint8_t data){
	/* Toggle status LED */
	GPIOE->ODR ^= CAN_TX_LED << 8;

	/* Configure the message to be transmitted. */
	TxMsg.StdId = StdId;
	TxMsg.DLC = 1;
	TxMsg.Data[0] = data;

	/* Put message in Tx Mailbox and store the mailbox number. */
	TransmitMailbox = CAN_Transmit(CAN1, &TxMsg);

	/* Wait on Transmit */
	while((CAN_TransmitStatus(CAN1, TransmitMailbox) != CAN_TxStatus_Ok));
}

/**
 * @brief  Transmit SENSOR_AN_RAW package.
 * @param  None
 * @retval The number of unprocessed messages (uint8_t).
 */
void CAN_transmit_AN_RAW(void){
	/* Toggle status LED */
	GPIOE->ODR ^= CAN_TX_LED << 8;

	/* Configure the message to be transmitted. */
	TxMsg.StdId = SENSOR_AN_RAW;
	TxMsg.DLC = 8;
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.RTR = CAN_RTR_DATA;

	TxMsg.Data[0] = (uint8_t) (ADC4_getChannel(0) & 0xFF);
	TxMsg.Data[1] = (uint8_t) (ADC4_getChannel(0) >> 8);
	TxMsg.Data[2] = (uint8_t) (ADC1_getChannel(1) & 0xFF);
	TxMsg.Data[3] = (uint8_t) (ADC1_getChannel(1) >> 8);
	TxMsg.Data[4] = (uint8_t) (ADC1_getChannel(2) & 0xFF);
	TxMsg.Data[5] = (uint8_t) (ADC1_getChannel(2) >> 8);
	TxMsg.Data[6] = (uint8_t) (ADC1_getChannel(3) & 0xFF);
	TxMsg.Data[7] = (uint8_t) (ADC1_getChannel(3) >> 8);

	/* Put message in Tx Mailbox and store the mailbox number. */
	TransmitMailbox = CAN_Transmit(CAN1, &TxMsg);

	/* Wait for Transmit */
	while(CAN_TransmitStatus(CAN1, TransmitMailbox) != CAN_TxStatus_Ok);
}

/**
 * @brief  Transmit 6 byte data package.
 * @param  None
 * @retval The number of unprocessed messages (uint8_t).
 */
void CAN_transmitAcceleration(int8_t *acc_array){
	/* Toggle status LED */
	GPIOE->ODR ^= CAN_TX_LED << 8;

	/* Configure the message to be transmitted. */
	TxMsg.StdId = SENSOR_ACCELERATION;
	TxMsg.DLC = 6;

	uint8_t i = 0;
	for(i=0;i<6;i++) TxMsg.Data[i] = *acc_array++;

	/* Put message in Tx Mailbox and store the mailbox number. */
	TransmitMailbox = CAN_Transmit(CAN1, &TxMsg);

	/* Wait on Transmit */
	while((CAN_TransmitStatus(CAN1, TransmitMailbox) != CAN_TxStatus_Ok));
}

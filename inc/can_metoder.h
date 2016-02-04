/**
  **************************************************************************************
  * @file    ADC_metoder.h
  * @author  Sivert Sliper, Stian Soerensen
  * @version V01
  * @date    03-February-2016
  * @brief   This file contains local variables and macros for CAN_metoder.c
  **************************************************************************************
  */

/* Include -----------------------------------------------------------------------------*/
#include "stm32f30x_can.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_misc.h"

/* Private macro -----------------------------------------------------------------------*/


/* ID list, RANGE = [0...0x7FF] *********************************************************/

#define TOPSIDE_BASE 					0x200
#define TOPSIDE_JOYSTICK				TOPSIDE_BASE
#define TOPSIDE_COMMANDS				(TOPSIDE_BASE + 1)

#define SENSOR_BASE 					0x300
#define SENSOR_AN_RAW					SENSOR_BASE
#define SENSOR_PROCESSED_DATA			(SENSOR_BASE + 1)
#define SENSOR_ACCELERATION				(SENSOR_BASE + 2)
#define SENSOR_DEPTH_TEMP				(SENSOR_BASE + 3)
#define SENSOR_LEAKAGE_ALARM			(SENSOR_BASE + 4)

#define POWR_BASE 						0x400
#define POWR_STATUS						POWR_BASE

#define ESC_BASE						0x500
#define ESC_1							ESC_BASE

/* Standard ID filters ******************************************************************/
#define CAN_RX_FILTER_NONE				0x7FF

/* Filter bank 0 */
#define CAN_RX_FILTER_0					CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_1					CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_2					SENSOR_PROCESSED_DATA
#define CAN_RX_FILTER_3					CAN_RX_FILTER_NONE

/* Filter bank 1 */
#define CAN_RX_FILTER_4					CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_5					CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_6					CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_7					CAN_RX_FILTER_NONE

/* Filter bank 3 */
#define CAN_RX_FILTER_8					CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_9					CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_10				CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_11				CAN_RX_FILTER_NONE

/* Filter bank 4 */
#define CAN_RX_FILTER_12				CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_13				CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_14				CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_15				CAN_RX_FILTER_NONE

/* Extended ID filters ********************************************************/

/* Private variables ---------------------------------------------------------*/
CanRxMsg RxMsg;
CanTxMsg TxMsg = {0};
uint8_t rx_messages = 0;
uint8_t TransmitMailbox = 0;

/* Array for incomming messages, messages are stored according to filter match
 * indicator(FMI). */
uint8_t Rx_Array[16][8];


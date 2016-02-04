/* Includes ----------------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_misc.h"
#include "extern_decl_global_vars.h"

/* Private macros ----------------------------------------------------------------------*/
#define RX_BUFFER_SIZE						10

/* Private variables -------------------------------------------------------------------*/

/* Receive buffer */
uint8_t rx_buffer[RX_BUFFER_SIZE] = {0};
uint8_t rx_buffer_write_counter = 0;
uint8_t rx_buffer_read_counter = 0;
uint8_t new_bytes = 0;

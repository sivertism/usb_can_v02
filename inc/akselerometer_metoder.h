
/* Private variables -------------------------------------------------------------------*/

uint8_t ACC_HIGHPASS_DISABLE = 0x80; // Innhold i CTRL_REG2_A for � deaktivere h�ypassfilter
uint8_t akselerometer_mottaksbuffer[6];
uint8_t aks_bufferteller = 0;

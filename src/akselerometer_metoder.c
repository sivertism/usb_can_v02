/**
  ******************************************************************************
  * @file    akselerometer_metoder.c
  * @author
  * @version V1.0
  * @date    8-September-2015
  * @brief   This file uses functions from the stm32f3_discovery_lsm303dlhc
  * 			library to enable reading of accelerometer data
  ******************************************************************************
  */

/* Inklusjoner */
#include "stm32f3_discovery_lsm303dlhc.h"

/* Globale variabler */
#include "extern_decl_global_vars.h"

/* Lokale variabler */
#include "akselerometer_metoder.h"

/* Funksjonsprototyper */
void accelerometer_init(void);
void accelerometer_readValue(void);


////////////////////////////////////////////////////////////////////////////////////////
/* Funksjonsdeklarasjoner */
////////////////////////////////////////////////////////////////////////////////////////


/* akselerometer_init()
 @Beskrivelse
 	Initialiserer akselerometerkretsen med følgende nøkkelparametre:
 		- Sampler akselerasjon med en frekvens på 100 Hz.
 		- Måler akselerasjon i alle 3 akser (x, y, z)
 		- +/- 2g måleområde.
 		- 12 bit oppløsning dvs. 1mg oppløsning

 @Inngangsvariabler:
 	-
*/
void accelerometer_init(void){

	/* Lager her en initialiseringstruktur som sendes inn i metoden LSM303DLHC_AccInit()
	 * fra biblioteket
	 */
	LSM303DLHCAcc_InitTypeDef aks_init;
	aks_init.Power_Mode = LSM303DLHC_NORMAL_MODE; /*dvs ikke lowpower mode*/
	aks_init.AccOutput_DataRate = LSM303DLHC_ODR_100_HZ;

	/*Plattformen skal i utgangspunktet kun beveges i y-retning.
	 * Vil uansett måle akselerasjon i alle retninger.
	 * x-aksen peker i retning fra usb-portene på kortet mot kompassdiodene
	 * y-aksen peker i retning fra blå knapp(USER) til svart knapp(RESET)
	 * z-aksen peker oppover*/
	aks_init.Axes_Enable = LSM303DLHC_AXES_ENABLE;

	/* Velger her hvor stort mAleomrAde vi Onsker A bruke.
	 * Det kan tenkes at dette mA endres etter testing for A utnytte
	 * akselerometeret best mulig.
	 * Kan velge mellom +/- 2G, 4G,8G og 16G.*/

	aks_init.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;

	/*Velger sA opplOsningen til mAlingene, starter med A bruke
	 * full opplOsning, dvs 12bit. Dette kan endres senere,
	 * dersom vi ser at vi ikke trenger full opplosning
	 */
	aks_init.High_Resolution = LSM303DLHC_HR_ENABLE;

	/* Velger kontinuerlig sampling, i stedet for at sampling trigges av
	 * at MSB og LSB av utgangsregisteret i aks.meteret leses.
	 */
	aks_init.High_Resolution = LSM303DLHC_BlockUpdate_Continous;
	/*Velger LSB i laveste adresse*/
	aks_init.Endianness = LSM303DLHC_BLE_LSB;
	/*Naa er oppsettet klart for aa sendes inn i initialiseringsmetoden fra
	 * biblioteket. Metoden LSM303DLHC_AccInit() setter også opp I2C kommunikasjon
	 * til ADC'en for oss. Sender en peker til aks_init inn i metoden. */
	LSM303DLHC_AccInit(&aks_init);
} // end akselerometer_init()


/**
 * @brief  Reads the acceleration in 3 axes from the accelerometer.
 * @param
 * @retval
 */
void accelerometer_readValue(void){

	/* Akselerometerdataen ligger i de 12 øverste bit-ene av de to bytene
	 * altså 0bDDDDDDDDDDDDXXXX der D er data og X er uønsket. Derfor
	 * skifter vi den nederste byten 0bDDDDXXXX 4 mot høyre for å få
	 * 0b0000DDDD, mens den øverste byten forblir uendret.
	 */

	/* Leser 6 byte som starter fra adressen OUT_X_L_A, får da 3*12 bit akselerasjonsdata
	 * pakket inn i 2 og 2 byte. */
	LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A,akselerometer_mottaksbuffer,6);
	accelerometer_data[0] = akselerometer_mottaksbuffer[0] >> 4;
	accelerometer_data[1] = akselerometer_mottaksbuffer[1];
	accelerometer_data[2] = akselerometer_mottaksbuffer[2] >> 4;
	accelerometer_data[3] = akselerometer_mottaksbuffer[3];
	accelerometer_data[4] = akselerometer_mottaksbuffer[4] >> 4;
	accelerometer_data[5] = akselerometer_mottaksbuffer[5];
} // end akselerometer_lesVerdi()


/**
 * @brief  Returns the acceleration data for the selected axis.
 * @param  uint8_t axis - The wanted axis, X=0, Y=1, Z=2
 * @retval The acceleration data for the selected axis (int16_t).
 */
int16_t accelerometer_getData(uint8_t axis){
	uint16_t temp;
	if(axis == 0) temp   = (akselerometer_mottaksbuffer[1] << 8) | akselerometer_mottaksbuffer[0];
	else if(axis == 1) temp   = (akselerometer_mottaksbuffer[3] << 8) | akselerometer_mottaksbuffer[2];
	else if(axis == 2) temp   = (akselerometer_mottaksbuffer[5] << 8) | akselerometer_mottaksbuffer[4];
	return ((int16_t)(temp)) >> 4;
}

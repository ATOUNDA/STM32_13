/*
 * MY_DHT22.h
 *
 *  Created on: Apr 22, 2021
 *      Author: utilisateur
 */

#ifndef _MY_DHT22_H_
#define _MY_DHT22_H_

//Header files
#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>

//Pin Mode enum
typedef enum
{
	ONE_OUTPUT = 0,
	ONE_INPUT,
}OnePinMode_Typedef;



//*** Functions prototypes ***//
//OneWire Initialise
void DHT22_Init(GPIO_TypeDef* DataPort, uint16_t DataPin);
//Change pin mode
static void ONE_WIRE_PinMode(OnePinMode_Typedef mode);
//One Wire pin HIGH/LOW Write
static void ONE_WIRE_Pin_Write(bool state);
static bool ONE_WIRE_Pin_Read(void);
//Microsecond delay
static void DelayMicroSeconds(uint32_t uSec);
//Begin function
static void DHT22_StartAcquisition(void);
//Read 5 bytes
static void DHT22_ReadRaw(uint8_t *data);

//Get Temperature and Humidity data
bool DHT22_GetTemp_Humidity(float *Temp, float *Humidity);



#endif /* INC_MY_DHT22_H_ */

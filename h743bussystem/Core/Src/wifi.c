/*
 * wifi.c
 *
 *  Created on: Mar 9, 2021
 *      Author: 62786
 */
#include "wifi.h"
#include "stm32h7xx_hal.h"
#include "../../Drivers/OV7670/OV7670.h"
#include "main.h"
UART_HandleTypeDef huart5;
void wifi_send(uint8_t *data)
{
	for(int i=0;i<OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT*2;i++)
		HAL_UART_Transmit(&huart5, data+i, 1, 10);
}




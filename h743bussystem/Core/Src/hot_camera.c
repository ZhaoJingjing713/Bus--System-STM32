/*
 * hot_camera.c
 *
 *  Created on: Mar 5, 2021
 *      Author: 62786
 */
#include "hot_camera.h"
#include "stm32h7xx_hal.h"
#include "main.h"
#include <stdlib.h>
#include <time.h>

void hot_camera_init(UART_HandleTypeDef huart)
{
	  uint8_t tem[8];
	  tem[0]=0xEE;
	  tem[1]=0xE1;
	  tem[2]=0x01;
	  tem[3]=0x55;
	  tem[4]=0xFF;
	  tem[5]=0xFC;
	  tem[6]=0xFD;
	  tem[7]=0xFF;
	  HAL_UART_Transmit(&huart, &tem, 8, 100);
}

float hot_camer_get_temperature(uint8_t *hot_data)
{
	uint16_t tem[32*32];
	float high=0;
	if(hot_data[0]==0xe1){
		int j=0;
		for(int i=1;i<32*32+1;i++){
			tem[j]=hot_data[i]<<8|hot_data[++i];
			float current_tempurature=(tem[j]-2731)/10;
			if(current_tempurature>high)
				high=current_tempurature;
		}
	}else if(hot_data[0]=='0xb'){
		int j=0;
		for(int i=0;i<32*32;i++){
			tem[j]=hot_data[i]<<8|hot_data[++i];
			float current_tempurature=(tem[j]-2731)/10;
			if(current_tempurature>high)
			high=current_tempurature;
		}
	}
	return high;
}
int get_high(uint8_t *hot_data)
{

	uint16_t real_temperature[32*32];
	float out_temperature[32*32];
	int j=0;
	for(int i=0;i<32*32;i++){
		real_temperature[j]=hot_data[i]<<8|hot_data[++i];
		out_temperature[j++]=(real_temperature[j]-2731)/10;
	}

	srand((unsigned)time(NULL));
	float high=(rand()%(350-360+1)+350);
	return high;
}

/*
 * hot_camera.h
 *
 *  Created on: Mar 5, 2021
 *      Author: 62786
 */

#ifndef INC_HOT_CAMERA_H_
#define INC_HOT_CAMERA_H_

#include "stm32h7xx_hal.h"
#define PI 3.1415926
#define MIDDLE(x,y,z) ((x)<(y)?((y)<(z)?(y):(x)<(z)?(z):(x)):((y)>(z)?(y):(x)>(z)?(z):(x)))

void hot_camera_init(UART_HandleTypeDef huart1);
float hot_camer_get_temperature(uint8_t *hot_data);
int get_high(uint8_t *hot_data);

//获得高斯核


void interpolation();

#endif /* INC_HOT_CAMERA_H_ */

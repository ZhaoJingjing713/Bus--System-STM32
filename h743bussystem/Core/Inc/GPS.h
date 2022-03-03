/*
 * GPS.h
 *
 *  Created on: Mar 9, 2021
 *      Author: 62786
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#define RX_BUF_MAXLEN 255
#define GPS_Buf_Length	80
#define true	1
#define false	0
#define latitude_Length	15
#define longitude_Length	15
#define address_Length	55
#define messageLength	60

typedef struct GPS_Data{
	char GPS_Buffer[GPS_Buf_Length];
	char isGetData;		//判断是否正在获取数据
	char isUseful; 		//判断是否可用数据
	char latitude[latitude_Length];		//纬度
	char longitude[longitude_Length];	//经度
	char address[address_Length];		//经纬度
}GPS_Data;


void transfer();
void GPS_Init();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void parseGPS();	//解析GPS
void printGPS();







#endif /* INC_GPS_H_ */

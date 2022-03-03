/*
 * fonts.h
 *
 *  Created on: Mar 3, 2021
 *      Author: 62786
 */

#ifndef INC_FONTS_H_
#define INC_FONTS_H_

#include "stm32h7xx_hal.h"
#define LINE(x) ((x) * (((sFONT *)LCD_GetFont())->Height))
#define LINEY(x) ((x) * (((sFONT *)LCD_GetFont())->Width))

/** @defgroup FONTS_Exported_Types
  * @{
  */
typedef struct _tFont
{
  const uint8_t *table;
  uint16_t Width;
  uint16_t Height;

} sFONT;

extern sFONT Font24x32;
extern sFONT Font16x24;
extern sFONT Font8x16;



#endif /* INC_FONTS_H_ */

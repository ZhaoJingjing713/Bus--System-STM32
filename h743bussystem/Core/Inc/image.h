/*
 * image.h
 *
 *  Created on: Mar 3, 2021
 *      Author: 62786
 */

#ifndef INC_IMAGE_H_
#define INC_IMAGE_H_

#include <stdint.h>

typedef struct _tImage
{
  uint16_t 					xSize;
  uint16_t 					ySize;
  uint16_t 					bytesPerLine;
  uint8_t					bitsPerPixel;
  const unsigned char* 		pData;
} sImage;

#define GUI_BITMAP			sImage
#define GUI_CONST_STORAGE	const

extern GUI_CONST_STORAGE GUI_BITMAP bmSTLogo;





#endif /* INC_IMAGE_H_ */

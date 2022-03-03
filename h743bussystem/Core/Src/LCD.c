/*
 * LCD.c
 *
 *  Created on: Mar 3, 2021
 *      Author: 62786
 */
#include "LCD.h"
#include "main.h"
#include "fonts.h"
#include "hot_camera.h"
lcd_params_t lcd_params;
//初始化完会使用本默认值
uint8_t LCD_SCAN_MODE = 6;
//调用函数设置方向时会自动更改（LIL9341_GramScan)
uint16_t LCD_X_LENGTH = ILI9341_LESS_PIXEL;
uint16_t LCD_Y_LENGTH = ILI9341_MORE_PIXEL;

volatile uint16_t LCD_HEIGHT = ILI9341_SCREEN_HEIGHT;
volatile uint16_t LCD_WIDTH	 = ILI9341_SCREEN_WIDTH;
static sFONT *LCD_Currentfonts = &Font8x16;
static uint16_t CurrentTextColor   = BLACK;
static uint16_t CurrentBackColor   = WHITE;
static void lcd_write_cmd(volatile uint16_t cmd)
{
    cmd = cmd;      //make compiler happy
    LCD->lcd_reg = cmd;
}


//static void lcd_write_data(volatile uint16_t data)
//{
//    data = data;    //make compiler happy
//    LCD->lcd_ram = data;
//}
void lcd_write_data(volatile uint16_t data)
{
    data = data;    //make compiler happy
    LCD->lcd_ram = data;
}
static uint16_t lcd_read_data(void)
{
    volatile uint16_t data;

    data = LCD->lcd_ram;

    return data;
}


static void lcd_write_reg(uint16_t reg, uint16_t data)
{
    LCD->lcd_reg = reg;
    LCD->lcd_ram = data;
}

static uint16_t lcd_read_reg(uint16_t reg)
{
    uint16_t data;

    LCD->lcd_reg = reg;
    HAL_Delay(1);
    data = LCD->lcd_ram;

    return data;
}



static int lcd_read_id(void)
{
    lcd_write_cmd(0XD3);
    lcd_params.lcd_id = lcd_read_data();
    lcd_params.lcd_id = lcd_read_data();
    lcd_params.lcd_id = lcd_read_data();
    lcd_params.lcd_id <<= 8;
    lcd_params.lcd_id |= lcd_read_data();
    if (lcd_params.lcd_id == 0x9341) {
        return 0;
    }

    lcd_params.lcd_id = 0;
    return -1;
}

//Initialize
static void ILI9341_REG_Config ( void )
{
	  lcd_write_cmd(0xCF);
	lcd_write_data(0x00);
	lcd_write_data(0xC1);
	lcd_write_data(0X30);
	lcd_write_cmd(0xED);
	lcd_write_data(0x64);
	lcd_write_data(0x03);
	lcd_write_data(0X12);
	lcd_write_data(0X81);
	lcd_write_cmd(0xE8);
	lcd_write_data(0x85);
	lcd_write_data(0x10);
	lcd_write_data(0x7A);
	lcd_write_cmd(0xCB);
	lcd_write_data(0x39);
	lcd_write_data(0x2C);
	lcd_write_data(0x00);
	lcd_write_data(0x34);
	lcd_write_data(0x02);
	lcd_write_cmd(0xF7);
	lcd_write_data(0x20);
	lcd_write_cmd(0xEA);
	lcd_write_data(0x00);
	lcd_write_data(0x00);
	lcd_write_cmd(0xC0);    //Power control
	lcd_write_data(0x1B);   //VRH[5:0]
	lcd_write_cmd(0xC1);    //Power control
	lcd_write_data(0x01);   //SAP[2:0];BT[3:0]
	lcd_write_cmd(0xC5);    //VCM control
	lcd_write_data(0x30); 	 //3F
	lcd_write_data(0x30); 	 //3C
	lcd_write_cmd(0xC7);    //VCM control2
	lcd_write_data(0XB7);
	lcd_write_cmd(0x36);    // Memory Access Control
	lcd_write_data(0x48);
	lcd_write_cmd(0x3A);
	lcd_write_data(0x55);
	lcd_write_cmd(0xB1);
	lcd_write_data(0x00);
	lcd_write_data(0x1A);
	lcd_write_cmd(0xB6);    // Display Function Control
	lcd_write_data(0x0A);
	lcd_write_data(0xA2);
	lcd_write_cmd(0xF2);    // 3Gamma Function Disable
	lcd_write_data(0x00);
	lcd_write_cmd(0x26);    //Gamma curve selected
	lcd_write_data(0x01);
	lcd_write_cmd(0xE0);    //Set Gamma
	lcd_write_data(0x0F);
	lcd_write_data(0x2A);
	lcd_write_data(0x28);
	lcd_write_data(0x08);
	lcd_write_data(0x0E);
	lcd_write_data(0x08);
	lcd_write_data(0x54);
	lcd_write_data(0XA9);
	lcd_write_data(0x43);
	lcd_write_data(0x0A);
	lcd_write_data(0x0F);
	lcd_write_data(0x00);
	lcd_write_data(0x00);
	lcd_write_data(0x00);
	lcd_write_data(0x00);
	lcd_write_cmd(0XE1);    //Set Gamma
	lcd_write_data(0x00);
	lcd_write_data(0x15);
	lcd_write_data(0x17);
	lcd_write_data(0x07);
	lcd_write_data(0x11);
	lcd_write_data(0x06);
	lcd_write_data(0x2B);
	lcd_write_data(0x56);
	lcd_write_data(0x3C);
	lcd_write_data(0x05);
	lcd_write_data(0x10);
	lcd_write_data(0x0F);
	lcd_write_data(0x3F);
	lcd_write_data(0x3F);
	lcd_write_data(0x0F);
	lcd_write_cmd(0x2B);
	lcd_write_data(0x00);
	lcd_write_data(0x00);
	lcd_write_data(0x01);
	lcd_write_data(0x3f);
	lcd_write_cmd(0x2A);
	lcd_write_data(0x00);
	lcd_write_data(0x00);
	lcd_write_data(0x00);
	lcd_write_data(0xef);
	lcd_write_cmd(0x11); //Exit Sleep
	HAL_Delay(120);
	lcd_write_cmd(0x29); //display on
//		/*  Power control B (CFh)  */
//	lcd_write_cmd ( 0xCF  );
//	lcd_write_data ( 0x00  );
//	lcd_write_data ( 0x81  );
//	lcd_write_data ( 0x30  );
//
//
//	/*  Power on sequence control (EDh) */
//	lcd_write_cmd ( 0xED );
//	lcd_write_data ( 0x64 );
//	lcd_write_data ( 0x03 );
//	lcd_write_data ( 0x12 );
//	lcd_write_data ( 0x81 );
//
//	/*  Driver timing control A (E8h) */
//	lcd_write_cmd ( 0xE8 );
//	lcd_write_data ( 0x85 );
//	lcd_write_data ( 0x10 );
//	lcd_write_data ( 0x78 );
//
//	/*  Power control A (CBh) */
//	lcd_write_cmd ( 0xCB );
//	lcd_write_data ( 0x39 );
//	lcd_write_data ( 0x2C );
//	lcd_write_data ( 0x00 );
//	lcd_write_data ( 0x34 );
//	lcd_write_data ( 0x02 );
//
//	/* Pump ratio control (F7h) */
//	lcd_write_cmd ( 0xF7 );
//	lcd_write_data ( 0x20 );
//
//	/* Driver timing control B */
//	lcd_write_cmd ( 0xEA );
//	lcd_write_data ( 0x00 );
//	lcd_write_data ( 0x00 );
//
//	/* Frame Rate Control (In Normal Mode/Full Colors) (B1h) */
//	lcd_write_cmd ( 0xB1 );
//	lcd_write_data ( 0x00 );
//	lcd_write_data ( 0x1B );
//
//	/*  Display Function Control (B6h) */
//	lcd_write_cmd ( 0xB6 );
//	lcd_write_data ( 0x0A );
//	lcd_write_data ( 0xA2 );
//
//	/* Power Control 1 (C0h) */
//	lcd_write_cmd ( 0xC0 );
//	lcd_write_data ( 0x35 );
//
//	/* Power Control 2 (C1h) */
//	lcd_write_cmd ( 0xC1 );
//	lcd_write_data ( 0x11 );
//
//	/* VCOM Control 1 (C5h) */
//	lcd_write_cmd ( 0xC5 );
//	lcd_write_data ( 0x45 );
//	lcd_write_data ( 0x45 );
//
//	/*  VCOM Control 2 (C7h)  */
//	lcd_write_cmd ( 0xC7 );
//	lcd_write_data ( 0xA2 );
//
//	/* Enable 3G (F2h) */
//	lcd_write_cmd ( 0xF2 );
//	lcd_write_data ( 0x00 );
//
//	/* Gamma Set (26h) */
//	lcd_write_cmd ( 0x26 );
//	lcd_write_data ( 0x01 );
//
//	/* Positive Gamma Correction */
//	lcd_write_cmd ( 0xE0 ); //Set Gamma
//	lcd_write_data ( 0x0F );
//	lcd_write_data ( 0x26 );
//	lcd_write_data ( 0x24 );
//	lcd_write_data ( 0x0B );
//	lcd_write_data ( 0x0E );
//	lcd_write_data ( 0x09 );
//	lcd_write_data ( 0x54 );
//	lcd_write_data ( 0xA8 );
//	lcd_write_data ( 0x46 );
//	lcd_write_data ( 0x0C );
//	lcd_write_data ( 0x17 );
//	lcd_write_data ( 0x09 );
//	lcd_write_data ( 0x0F );
//	lcd_write_data ( 0x07 );
//	lcd_write_data ( 0x00 );
//
//	/* Negative Gamma Correction (E1h) */
//	lcd_write_cmd ( 0XE1 ); //Set Gamma
//	lcd_write_data ( 0x00 );
//	lcd_write_data ( 0x19 );
//	lcd_write_data ( 0x1B );
//	lcd_write_data ( 0x04 );
//	lcd_write_data ( 0x10 );
//	lcd_write_data ( 0x07 );
//	lcd_write_data ( 0x2A );
//	lcd_write_data ( 0x47 );
//	lcd_write_data ( 0x39 );
//	lcd_write_data ( 0x03 );
//	lcd_write_data ( 0x06 );
//	lcd_write_data ( 0x06 );
//	lcd_write_data ( 0x30 );
//	lcd_write_data ( 0x38 );
//	lcd_write_data ( 0x0F );
//
//	/* memory access control set */
//	lcd_write_cmd ( 0x36 );
//	lcd_write_data ( 0xC8 );    /*ÊúÆÁ  ×óÉÏ½Çµ½ (Æðµã)µ½ÓÒÏÂ½Ç (ÖÕµã)É¨Ãè·½Ê½*/


//	/* column address control set */
//	lcd_write_cmd ( CMD_SetCoordinateX );
//	lcd_write_data ( 0x00 );
//	lcd_write_data ( 0x00 );
//	lcd_write_data ( 0x00 );
//	lcd_write_data ( 0xEF );
//
//	/* page address control set */
//	lcd_write_cmd ( CMD_SetCoordinateY );
//	lcd_write_data ( 0x00 );
//	lcd_write_data ( 0x00 );
//	lcd_write_data ( 0x01 );
//	lcd_write_data ( 0x3F );
//
//	/*  Pixel Format Set (3Ah)  */
//	lcd_write_cmd ( 0x3a );
//	lcd_write_data ( 0x55 );
//
//	/* Sleep Out (11h)  */
//	lcd_write_cmd ( 0x11 );
//	HAL_Delay ( 100 );
//
//	/* Display ON (29h) */
//	lcd_write_cmd ( 0x29 );
}


/**
 * @brief  ILI9341G 软复位
 * @param
 * @retval
 */
void ILI9341_Rst( void )
{
	HAL_GPIO_WritePin( RST_GPIO_Port,RST_Pin,GPIO_PIN_RESET);

	HAL_Delay ( 500 );

	HAL_GPIO_WritePin( RST_GPIO_Port,RST_Pin,GPIO_PIN_SET);

	HAL_Delay ( 500 );

}



/**
 * @brief  设置ILI9341的Gram扫描方向
 * @param  ucOption 选择gram的扫描方向
 *     @arg 0-7 ：参数0-7
 *
 *	其中 0、3、5、6适合从左到右显示文字
 *
 *	1、3、5、7 X方向320，Y240
 *	2，4，6，0  Y方向320 x240
 *
 *	其中 6 大部分液晶例程
 *	其中 3摄像头
 *	其中 0为BMP图片
 *0--Lto R, Up to Down
 *1--L to R, D to U
 *2--R to L, U to D
 *3--R to L, D to U
 *4--U to D, L to R
 *5--U to D, R to L
 *6--D to U, L to R
 *7--D to U, R to L
 *******************************************************/

void ILI9341_GramScan ( uint8_t ucOption )
{
	//参数，0-7
	if(ucOption >7 )
		return;

	LCD_SCAN_MODE = ucOption;

	if(ucOption%2 == 0)// *	2，4，6，0  Y方向320 x240
	{
		LCD_X_LENGTH = ILI9341_LESS_PIXEL;
		LCD_Y_LENGTH =	ILI9341_MORE_PIXEL;
	}
	else
	{
		//1、3、5、7 X方向320，Y240
		LCD_X_LENGTH = ILI9341_MORE_PIXEL;
		LCD_Y_LENGTH =	ILI9341_LESS_PIXEL;
	}

	lcd_write_cmd ( 0x36 );
	lcd_write_data ( 0x08 |(ucOption<<5));
	lcd_write_cmd ( CMD_SetCoordinateX );
	lcd_write_data ( 0x00 );
	lcd_write_data ( 0x00 );
	lcd_write_data ( ((LCD_X_LENGTH-1)>>8)&0xFF );
	lcd_write_data ( (LCD_X_LENGTH-1)&0xFF );

	lcd_write_cmd ( CMD_SetCoordinateY );
	lcd_write_data ( 0x00 );
	lcd_write_data ( 0x00 );
	lcd_write_data ( ((LCD_Y_LENGTH-1)>>8)&0xFF );
	lcd_write_data ( (LCD_Y_LENGTH-1)&0xFF );
	/* write gram start */
	lcd_write_cmd ( CMD_SetPixel );
}




void lcd_init(void)
{

    HAL_GPIO_WritePin(BG_GPIO_Port, BG_Pin, GPIO_PIN_SET);
	ILI9341_Rst();
	ILI9341_REG_Config();
	ILI9341_GramScan(LCD_SCAN_MODE);
}



//开窗口函数
void ILI9341_OpenWindow ( uint16_t usX, uint16_t usY, uint16_t usWidth, uint16_t usHeight )
{
	lcd_write_cmd ( CMD_SetCoordinateX ); 				 /* ÉèÖÃX×ø±ê */
	lcd_write_data ( usX >> 8  );	 /* ÏÈ¸ß8Î»£¬È»ºóµÍ8Î» */
	lcd_write_data ( usX & 0xff  );	 /* ÉèÖÃÆðÊ¼µãºÍ½áÊøµã*/
	lcd_write_data ( ( usX + usWidth - 1 ) >> 8  );
	lcd_write_data ( ( usX + usWidth - 1 ) & 0xff  );

	lcd_write_cmd ( CMD_SetCoordinateY ); 			     /* ÉèÖÃY×ø±ê*/
	lcd_write_data ( usY >> 8  );
	lcd_write_data ( usY & 0xff  );
	lcd_write_data ( ( usY + usHeight - 1 ) >> 8 );
	lcd_write_data ( ( usY + usHeight - 1) & 0xff );

}


//屏幕光标设置
static void ILI9341_SetCursor ( uint16_t usX, uint16_t usY )
{
	ILI9341_OpenWindow ( usX, usY, 1, 1 );
}


//填充像素
static __inline void ILI9341_FillColor ( uint32_t ulAmout_Point, uint16_t usColor )
{
	uint32_t i = 0;


	/* memory write */
	lcd_write_cmd ( CMD_SetPixel );

	for ( i = 0; i < ulAmout_Point; i ++ )
		lcd_write_data ( usColor );


}



void ILI9341_Clear ( uint16_t usX, uint16_t usY, uint16_t usWidth, uint16_t usHeight , uint16_t usColor)
{
	ILI9341_OpenWindow ( usX, usY, usWidth, usHeight );

	ILI9341_FillColor ( usWidth * usHeight, usColor );

}



void ILI9341_SetPointPixel ( uint16_t usX, uint16_t usY , uint16_t usColor)
{
	if ( ( usX < LCD_X_LENGTH ) && ( usY < LCD_Y_LENGTH ) )
  {
		ILI9341_SetCursor ( usX, usY );

		ILI9341_FillColor ( 1, usColor );
	}

}

void lcd_auto_clear(uint16_t period_ms)
{
    ILI9341_Clear(0,0,320,240,BLACK);
    HAL_Delay(period_ms);
    ILI9341_Clear(0,0,320,240,BLUE);
    HAL_Delay(period_ms);
    ILI9341_Clear(0,0,320,240,GREEN);
    HAL_Delay(period_ms);
    ILI9341_Clear(0,0,320,240,GBLUE);
    HAL_Delay(period_ms);
    ILI9341_Clear(0,0,320,240,CYAN);
    HAL_Delay(period_ms);
}
void ILI9341_DrawLine ( uint16_t usX1, uint16_t usY1, uint16_t usX2, uint16_t usY2 )
{
	uint16_t us;
	uint16_t usX_Current, usY_Current;

	int32_t lError_X = 0, lError_Y = 0, lDelta_X, lDelta_Y, lDistance;
	int32_t lIncrease_X, lIncrease_Y;


	lDelta_X = usX2 - usX1; //¼ÆËã×ø±êÔöÁ¿
	lDelta_Y = usY2 - usY1;

	usX_Current = usX1;
	usY_Current = usY1;


	if ( lDelta_X > 0 )
		lIncrease_X = 1; //ÉèÖÃµ¥²½·½Ïò

	else if ( lDelta_X == 0 )
		lIncrease_X = 0;//´¹Ö±Ïß

	else
  {
    lIncrease_X = -1;
    lDelta_X = - lDelta_X;
  }


	if ( lDelta_Y > 0 )
		lIncrease_Y = 1;

	else if ( lDelta_Y == 0 )
		lIncrease_Y = 0;//Ë®Æ½Ïß

	else
  {
    lIncrease_Y = -1;
    lDelta_Y = - lDelta_Y;
  }


	if (  lDelta_X > lDelta_Y )
		lDistance = lDelta_X; //Ñ¡È¡»ù±¾ÔöÁ¿×ø±êÖá

	else
		lDistance = lDelta_Y;


	for ( us = 0; us <= lDistance + 1; us ++ )//»­ÏßÊä³ö
	{
		ILI9341_SetPointPixel ( usX_Current, usY_Current,BLACK );//»­µã

		lError_X += lDelta_X ;
		lError_Y += lDelta_Y ;

		if ( lError_X > lDistance )

		{
			lError_X -= lDistance;
			usX_Current += lIncrease_X;
		}

		if ( lError_Y > lDistance )
		{
			lError_Y -= lDistance;
			usY_Current += lIncrease_Y;
		}

	}


}
void ILI9341_DrawRectangle ( uint16_t usX_Start, uint16_t usY_Start, uint16_t usWidth, uint16_t usHeight, uint8_t ucFilled )
{
	if ( ucFilled )
	{
		ILI9341_OpenWindow ( usX_Start, usY_Start, usWidth, usHeight );
		ILI9341_FillColor ( usWidth * usHeight ,RED);
	}
	else
	{
		ILI9341_DrawLine ( usX_Start, usY_Start, usX_Start + usWidth - 1, usY_Start );
		ILI9341_DrawLine ( usX_Start, usY_Start + usHeight - 1, usX_Start + usWidth - 1, usY_Start + usHeight - 1 );
		ILI9341_DrawLine ( usX_Start, usY_Start, usX_Start, usY_Start + usHeight - 1 );
		ILI9341_DrawLine ( usX_Start + usWidth - 1, usY_Start, usX_Start + usWidth - 1, usY_Start + usHeight - 1 );
	}

}

void ILI9341_DispChar_EN ( uint16_t usX, uint16_t usY, const char cChar )
{
	uint8_t  byteCount, bitCount,fontLength;
	uint16_t ucRelativePositon;
	uint8_t *Pfont;


	ucRelativePositon = cChar - ' ';
	fontLength = (LCD_Currentfonts->Width*LCD_Currentfonts->Height)/8;

	Pfont = (uint8_t *)&LCD_Currentfonts->table[ucRelativePositon * fontLength];


	ILI9341_OpenWindow ( usX, usY, LCD_Currentfonts->Width, LCD_Currentfonts->Height);

	lcd_write_cmd ( CMD_SetPixel );


	for ( byteCount = 0; byteCount < fontLength; byteCount++ )
	{

			for ( bitCount = 0; bitCount < 8; bitCount++ )
			{
					if ( Pfont[byteCount] & (0x80>>bitCount) )
						lcd_write_data ( CurrentTextColor );
					else
						lcd_write_data ( CurrentBackColor );
			}
	}
}
void ILI9341_DispStringLine_EN (  uint16_t line,  char * pStr )
{
	uint16_t usX = 0;

	while ( * pStr != '\0' )
	{
		if ( ( usX - ILI9341_DispWindow_X_Star + LCD_Currentfonts->Width ) > LCD_X_LENGTH )
		{
			usX = ILI9341_DispWindow_X_Star;
			line += LCD_Currentfonts->Height;
		}

		if ( ( line - ILI9341_DispWindow_Y_Star + LCD_Currentfonts->Height ) > LCD_Y_LENGTH )
		{
			usX = ILI9341_DispWindow_X_Star;
			line = ILI9341_DispWindow_Y_Star;
		}

		ILI9341_DispChar_EN ( usX, line, * pStr);

		pStr ++;

		usX += LCD_Currentfonts->Width;

	}

}


void ILI9341_DispString_EN ( 	uint16_t usX ,uint16_t usY,  char * pStr )
{
	while ( * pStr != '\0' )
	{
		if ( ( usX - ILI9341_DispWindow_X_Star + LCD_Currentfonts->Width ) > LCD_X_LENGTH )
		{
			usX = ILI9341_DispWindow_X_Star;
			usY += LCD_Currentfonts->Height;
		}

		if ( ( usY - ILI9341_DispWindow_Y_Star + LCD_Currentfonts->Height ) > LCD_Y_LENGTH )
		{
			usX = ILI9341_DispWindow_X_Star;
			usY = ILI9341_DispWindow_Y_Star;
		}

		ILI9341_DispChar_EN ( usX, usY, * pStr);

		pStr ++;

		usX += LCD_Currentfonts->Width;

	}

}


void ILI9341_DispString_EN_YDir (	 uint16_t usX,uint16_t usY ,  char * pStr )
{
	while ( * pStr != '\0' )
	{
		if ( ( usY - ILI9341_DispWindow_Y_Star + LCD_Currentfonts->Height ) >LCD_Y_LENGTH  )
		{
			usY = ILI9341_DispWindow_Y_Star;
			usX += LCD_Currentfonts->Width;
		}

		if ( ( usX - ILI9341_DispWindow_X_Star + LCD_Currentfonts->Width ) >  LCD_X_LENGTH)
		{
			usX = ILI9341_DispWindow_X_Star;
			usY = ILI9341_DispWindow_Y_Star;
		}

		ILI9341_DispChar_EN ( usX, usY, * pStr);

		pStr ++;

		usY += LCD_Currentfonts->Height;
	}
}


void LCD_SetFont(sFONT *fonts)
{
  LCD_Currentfonts = fonts;
}


sFONT *LCD_GetFont(void)
{
  return LCD_Currentfonts;
}



void LCD_SetColors(uint16_t TextColor, uint16_t BackColor)
{
  CurrentTextColor = TextColor;
  CurrentBackColor = BackColor;
}


void LCD_GetColors(uint16_t *TextColor, uint16_t *BackColor)
{
  *TextColor = CurrentTextColor;
  *BackColor = CurrentBackColor;
}


void LCD_SetTextColor(uint16_t Color)
{
  CurrentTextColor = Color;
}


void LCD_SetBackColor(uint16_t Color)
{
  CurrentBackColor = Color;
}


void LCD_ClearLine(uint16_t Line)
{
  ILI9341_Clear(0,Line,LCD_X_LENGTH,((sFONT *)LCD_GetFont())->Height,CurrentBackColor);	/* ÇåÆÁ£¬ÏÔÊ¾È«ºÚ */

}

void LCD_WriteRAM_Prepare(void)
{
	lcd_write_cmd(lcd_params.wram_cmd);
}



void LCD_WriteData_Color(uint16_t color)
{

	lcd_write_data(color);

}

void LCD_Display_Dir(uint8_t dir)
{

}


void  LCD_ImgShow(const uint8_t* p)
{
	uint8_t k=0;
	unsigned char picH,picL;
	ILI9341_OpenWindow(0, 0, 320,240);//窗口设置
	lcd_write_cmd ( CMD_SetPixel );
	for(int i=0;i<320*240;i++){
//			uint16_t color=p[k];
//			color<<=8;
//			color|=p[k+1];
//			k+=2;
	 	picL=*(p+i*2);	//数据低位在前
		picH=*(p+i*2+1);
		lcd_write_data(picH<<8|picL);
	}
	ILI9341_OpenWindow(0,0, 320,240);//恢复显示窗口为全屏
}

void  LCD_ImgShow_gray(const uint8_t* p)
{
	uint8_t k=0;
	unsigned char picH,picL;
	ILI9341_OpenWindow(0, 0, 320,240);//窗口设置
	lcd_write_cmd ( CMD_SetPixel );
	for(int i=1;i<320*240*2;i++){
		uint16_t gray = (uint16_t) p[i++];
		uint16_t rb = (gray * 31) / 255; // red and blue are 5 bits
		uint16_t g = (gray *50) / 255;	// green is 6 bits
		uint16_t color = (rb<<11)|(g<<5)|(rb<<0);

//	 	uint16_t gray=(uint16_t)*(p+i);
//	 	uint16_t rb = (gray * 31) / 255; // red and blue are 5 bits
//		uint16_t g = (gray *63) / 255;	// green is 6 bits
//		uint16_t color = (rb<<11)|(g<<5)|(rb<<0);

		lcd_write_data(color);
	}
	ILI9341_OpenWindow(0,0, 320,240);//恢复显示窗口为全屏
}
static void get_gau_kernel(float kernel[9][9],int size,float sigma)
{
	int x,y;
	int m=size/2;
	float sum=0;
	for(y=0;y<size;y++)
	{
		for(x=0;x<size;x++)
		{
			kernel[y][x]=(1/(2*PI*sigma*sigma))*exp(-((x-m)*(x-m)+(y-m)*(y-m))/(2*sigma*sigma));
			sum+=kernel[y][x];
		}
	}
	for(y=0;y<size;y++)
	{
		for(x=0;x<size;x++)
		{
			kernel[y][x]/=sum;
		}
	}
}
void LCD_ImgShow_hot(const uint8_t *data)
{
	float data1[1024];
	float data2[32][32];
	float data3[100][100];
	float data4[100][100];
	float a,b;
	int i,j,x,y;
	float kernel[9][9];
	int size=9;
	float sigma;
	sigma=0.3*((size-1)*0.5-1)+0.8;
//翻译数据
	for(i=0;i<1024;i++)
	{
		data1[i]=(((int)data[2*i+1])*16*16+(int)data[2*i+2]-2700)/10.0;
	}
//化为32X32矩阵
	for(i=0;i<32;i++)
	{
		for(j=0;j<32;j++)
		{
			data2[i][j]=data1[i*32+j];
		}
	}
//二维线性插值
	for(i=0;i<100;i++)
	{
		for(j=0;j<100;j++)
		{
			a=(data2[(int)(i*0.31)+1][(int)(j*0.31)+1]-data2[(int)(i*0.31)+1][(int)(j*0.31)])*(j*0.31-(int)(j*0.31))+data2[(int)(i*0.31)+1][(int)(j*0.31)];
			b=(data2[(int)(i*0.31)][(int)(j*0.31)+1]-data2[(int)(i*0.31)][(int)(j*0.31)])*(j*0.31-(int)(j*0.31))+data2[(int)(i*0.31)][(int)(j*0.31)];
			data3[i][j]=(a-b)*(i*0.31-(int)(i*0.31))+b;
		}
	}
//高斯模糊
	get_gau_kernel(kernel,size,sigma);
	for(x=0;x<100;x++)
	{
		for(y=0;y<100;y++)
		{
			data4[x][y]=0;
			for(i=0;i<9;i++)
			{
				for(j=0;j<9;j++)
				{
					data4[x][y]+=data3[MIDDLE(0,(x+i-4),99)][MIDDLE(0,(y+j-4),99)]*kernel[i][j];
				}
			}
		}
	}
	uint32_t tem[320*240/2];
	uint8_t m=0;
	for(int i=0;i<100;i++)
	{
		for (int j=0;j<100;j++){
			if(j==99)
				j=0;
			tem[m]=data4[i][j];
		}
	}
	LCD_ImgShow_gray((uint8_t*) tem);
}

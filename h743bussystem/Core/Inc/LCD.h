
#ifndef INC_LCD_H_
#define INC_LCD_H_


#include "stm32h7xx_hal.h"
#include "fonts.h"


typedef struct lcd_fmc_address_st {
    uint16_t lcd_reg;
    uint16_t lcd_ram;
} lcd_fmc_address_t;

#define ILI9341_SCREEN_HEIGHT 	240
#define ILI9341_SCREEN_WIDTH 	320
#define ILI9341_COLOR_RGB565	0
#define ILI9341_COLOR_GRAYSCALE 1

//CHIP SELECT PIN AND PORT, STANDARD GPIO
#define LCD_CS_PORT		myLCD_CS_GPIO_Port
#define LCD_CS_PIN		myLCD_CS_Pin

//DATA COMMAND PIN AND PORT, STANDARD GPIO
#define LCD_DC_PORT		myLCD_DC_GPIO_Port
#define LCD_DC_PIN		myLCD_DC_Pin

//RESET PIN AND PORT, STANDARD GPIO
#define	LCD_RST_PORT	myLCD_RST_GPIO_Port
#define	LCD_RST_PIN		myLCD_RST_Pin


#define BURST_MAX_SIZE 	500

#define BLACK       0x0000
#define NAVY        0x000F
#define DARKGREEN   0x03E0
#define DARKCYAN    0x03EF
#define MAROON      0x7800
#define PURPLE      0x780F
#define OLIVE       0x7BE0
#define LIGHTGREY   0xC618
#define DARKGREY    0x7BEF
#define BLUE        0x001F
#define GREEN       0x07E0
#define CYAN        0x07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define WHITE       0xFFFF
#define ORANGE      0xFD20
#define GREENYELLOW 0xAFE5
#define PINK        0xF81F

#define SCREEN_VERTICAL_1		0
#define SCREEN_HORIZONTAL_1		1
#define SCREEN_VERTICAL_2		2
#define SCREEN_HORIZONTAL_2		3
//Above are from the formal project, maybe works

#define LCD_BASE    ((uint32_t)(0x60000000 | 0x0007FFFE))
#define LCD         ((lcd_fmc_address_t*)LCD_BASE)

/* LCD MPU protect parameters*/
#define LCD_REGION_NUMBER        MPU_REGION_NUMBER0        //LCD USE region0
#define LCD_ADDRESS_START        (0X60000000)            //LCD area Head address
#define LCD_REGION_SIZE            MPU_REGION_SIZE_256MB   //LCD area size


/***************************** ILI934***************************/
#define      ILI9341_DispWindow_X_Star		    0     //start X
#define      ILI9341_DispWindow_Y_Star		    0     //start Y

#define 			ILI9341_LESS_PIXEL	 240
#define 			ILI9341_MORE_PIXEL	 320

extern uint16_t LCD_X_LENGTH,LCD_Y_LENGTH;


extern uint8_t LCD_SCAN_MODE;


#define      BACKGROUND		                BLACK   //默认背景色

#define      BLACK                         0x0000
#define      GREY                          0xF7DE
#define      BLUE                          0x001F
#define      BLUE2                         0x051F
#define      RED                           0xF800
#define      MAGENTA                       0xF81F
#define      GREEN                         0x07E0
#define      CYAN                          0x7FFF
#define      YELLOW                        0xFFE0
#define      BRED                          0xF81F
#define      GRED                          0xFFE0
#define      GBLUE                         0x07FF



/******************************* 常用命令 ILI934 ********************************/
#define      CMD_SetCoordinateX		 		    0x2A	     //设置坐标x
#define      CMD_SetCoordinateY		 		    0x2B	     //设置坐标y
#define      CMD_SetPixel		 		          0x2C	     //Ìî³äÏñËØ

#define 			ILI9341_LESS_PIXEL	  	240
#define 			ILI9341_MORE_PIXEL	    320

/**
 * @brief    保存LCD屏幕参数
 * @param    lcd_width
 * @param    lcd_height
 * @param    lcd_id
 * @param    lcd_direction LCD横屏还是竖屏，0-竖，1-横
 * @param    wram_cmd      开始写gram指令
 * @param    set_x_cmd     设置x坐标指令
 * @param    set_y_cmd     设置y坐标指令
*/
typedef struct lcd_params_st {
    uint16_t lcd_width;
    uint16_t lcd_height;
    uint16_t lcd_id;
    uint8_t  lcd_direction;
    uint16_t wram_cmd;
    uint16_t set_x_cmd;
    uint16_t set_y_cmd;
} lcd_params_t;





extern lcd_params_t lcd_params;

void lcd_init(void);
void lcd_auto_clear(uint16_t period_ms);
void lcd_write_data(volatile uint16_t data);
void       ILI9341_Clear     			   ( uint16_t usX, uint16_t usY, uint16_t usWidth, uint16_t usHeight ,uint16_t usColor);
void       ILI9341_GramScan                ( uint8_t ucOtion );
void       ILI9341_OpenWindow              ( uint16_t usX, uint16_t usY, uint16_t usWidth, uint16_t usHeight );
uint16_t   ILI9341_GetPointPixel           ( uint16_t usX , uint16_t usY );
void       ILI9341_DrawLine                ( uint16_t usX1, uint16_t usY1, uint16_t usX2, uint16_t usY2 );
void       ILI9341_DrawRectangle           ( uint16_t usX_Start, uint16_t usY_Start, uint16_t usWidth, uint16_t usHeight,uint8_t ucFilled );
void       ILI9341_DrawCircle              ( uint16_t usX_Center, uint16_t usY_Center, uint16_t usRadius, uint8_t ucFilled );
void       ILI9341_DispChar_EN             ( uint16_t usX, uint16_t usY, const char cChar );
void       ILI9341_DispStringLine_EN       ( uint16_t line, char * pStr );
void       ILI9341_DispString_EN      			( uint16_t usX, uint16_t usY, char * pStr );
void       ILI9341_DispChar_CH             ( uint16_t usX, uint16_t usY, uint16_t usChar );
void       ILI9341_DispString_CH           ( uint16_t usX, uint16_t usY,  char * pStr );
void       ILI9341_DispString_EN_CH        (	uint16_t usX, uint16_t usY,  char * pStr );
void 	   ILI9341_DispStringLine_EN_CH 	(  uint16_t line, char * pStr );
void 	   ILI9341_DispString_EN_YDir 		(   uint16_t usX,uint16_t usY ,  char * pStr );
void 	   ILI9341_DispString_EN_CH_YDir 	(   uint16_t usX,uint16_t usY , char * pStr );
void 	   LCD_SetFont			(sFONT *fonts);
sFONT 	   *LCD_GetFont			(void);
void 	   LCD_ClearLine		(uint16_t Line);
void 	   LCD_SetBackColor		(uint16_t Color);
void 	   LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);
void 	   LCD_SetTextColor		(uint16_t Color);
void 	   LCD_SetColors		(uint16_t TextColor, uint16_t BackColor);
void 	   LCD_GetColors		(uint16_t *TextColor, uint16_t *BackColor);
void 	   LCD_WriteRAM_Prepare();
void       LCD_WriteData_Color(uint16_t color);
void 	   LCD_DisPlay_Dir(uint8_t dir);
void       LCD_ImgShow(const uint8_t* p);
void  	   LCD_ImgShow_gray(const uint8_t* p);
void       LCD_ImgShow_hot(const uint8_t *hot_data);
void ILI9341_DisplayStringEx(uint16_t x, 		//字符显示位置x
							uint16_t y, 				//字符显示位置y
							uint16_t Font_width,		//字体宽度  为偶数 英文字符/2
							uint16_t Font_Height,	//字体高度 为偶数
							uint8_t *ptr,					//内容
							uint16_t DrawModel);  //是否反色

void ILI9341_DisplayStringEx_YDir(uint16_t x,uint16_t y,uint16_t Font_width,uint16_t Font_Height,
								  uint8_t *ptr,uint16_t DrawModel);

#ifdef __cplusplus
}
#endif




#endif /* INC_LCD_H_ */

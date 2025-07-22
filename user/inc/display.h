#ifndef DISPLAY_H
#define DISPLAY_H

#define USE_HORIZONTAL 2  //ɨփºᆁ»򕟊úƁϔʾ 0»򱎪ʺƁ 2»򳎪ºᆁ


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_Width 135
#define LCD_Height 240

#else
#define LCD_Width 240
#define LCD_Height 135
#endif

#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40
#define BRRED 			 0XFC07
#define GRAY  			 0X8430

#define LCD_DC_PIN  GPIO_PIN_08
#define LCD_DC_PORT GPIO_PORT_B

#define LCD_RST_PIN GPIO_PIN_09
#define LCD_RST_PORT GPIO_PORT_B

#define LCD_PWR_PORT GPIO_PORT_B
#define LCD_PWR_PIN GPIO_PIN_03

#define LCD_CS_PORT GPIO_PORT_H
#define LCD_CS_PIN GPIO_PIN_02

#define	LCD_PWR(n)		(n?GPIO_SetPins(LCD_PWR_PORT, LCD_PWR_PIN):GPIO_ResetPins(LCD_PWR_PORT, LCD_PWR_PIN))
#define	LCD_RST(n)		(n?GPIO_SetPins(LCD_RST_PORT, LCD_RST_PIN):GPIO_ResetPins(LCD_RST_PORT, LCD_RST_PIN))
#define	LCD_DC(n)		(n?GPIO_SetPins(LCD_DC_PORT, LCD_DC_PIN):GPIO_ResetPins(LCD_DC_PORT, LCD_DC_PIN))

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void lcd_interface_init();
void LCD_Init(void);
void LCD_Clear(uint16_t color);
void LCD_Fill(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t color);
#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !DISPLAY_H
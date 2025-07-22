#include <Arduino.h>
#include "display.h"
#include "drivers/spi/SPI.h"
#include "hc32_ll_gpio.h"

#define LCD_Buf_Size 1152
static uint8_t lcd_buf[LCD_Buf_Size];

void lcd_interface_init()
{
    SPI3.set_pins(PB7, 0xff, PB6);
    SPI3.begin(25000000, true);
    stc_gpio_init_t stcGpioInit;
    GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir   = PIN_DIR_OUT;
    stcGpioInit.u16PinState = PIN_RESET;

    GPIO_Init(LCD_DC_PORT, LCD_DC_PIN, &stcGpioInit);
    GPIO_Init(LCD_RST_PORT, LCD_RST_PIN, &stcGpioInit);
    GPIO_Init(LCD_PWR_PORT, LCD_PWR_PIN, &stcGpioInit);
    GPIO_Init(LCD_CS_PORT, LCD_CS_PIN, &stcGpioInit);
}

static void LCD_SPI_Send(uint8_t *data, uint16_t size)
{
    SPI3.push_inDMA(data, size);
}

/**
 * @brief	дüÁLCD
 *
 * @param   cmd		ШҪ·¢ˍµăüÁ *
 * @return  void
 */
static void LCD_Write_Cmd(uint8_t cmd)
{
    LCD_DC(0);

    LCD_SPI_Send(&cmd, 1);
}

/**
 * @brief	дʽ¾ݵ½LCD
 *
 * @param   cmd		ШҪ·¢ˍµĊý¾ݍ
 *
 * @return  void
 */
static void LCD_Write_Data(uint8_t data)
{
    LCD_DC(1);

    LCD_SPI_Send(&data, 1);
}

/**
 * @brief	д°븶זµĊý¾ݵ½LCD
 *
 * @param   cmd		ШҪ·¢ˍµĊý¾ݍ
 *
 * @return  void
 */
void LCD_Write_HalfWord(const uint16_t da)
{
    uint8_t data[2] = {0};

    data[0] = da >> 8;
    data[1] = da;

    LCD_DC(1);
    LCD_SPI_Send(data, 2);
}

void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    if (USE_HORIZONTAL == 0) {
        LCD_Write_Cmd(0x2a); // Áеؖ·ɨփ
        LCD_Write_HalfWord(x1 + 52);
        LCD_Write_HalfWord(x2 + 52);
        LCD_Write_Cmd(0x2b); // Аµؖ·ɨփ
        LCD_Write_HalfWord(y1 + 40);
        LCD_Write_HalfWord(y2 + 40);
        LCD_Write_Cmd(0x2c); // ´¢´憷д
    } else if (USE_HORIZONTAL == 1) {
        LCD_Write_Cmd(0x2a); // Áеؖ·ɨփ
        LCD_Write_HalfWord(x1 + 53);
        LCD_Write_HalfWord(x2 + 53);
        LCD_Write_Cmd(0x2b); // Аµؖ·ɨփ
        LCD_Write_HalfWord(y1 + 40);
        LCD_Write_HalfWord(y2 + 40);
        LCD_Write_Cmd(0x2c); // ´¢´憷д
    } else if (USE_HORIZONTAL == 2) {
        LCD_Write_Cmd(0x2a); // Áеؖ·ɨփ
        LCD_Write_HalfWord(x1 + 40);
        LCD_Write_HalfWord(x2 + 40);
        LCD_Write_Cmd(0x2b); // Аµؖ·ɨփ
        LCD_Write_HalfWord(y1 + 53);
        LCD_Write_HalfWord(y2 + 53);
        LCD_Write_Cmd(0x2c); // ´¢´憷д
    } else {
        LCD_Write_Cmd(0x2a); // Áеؖ·ɨփ
        LCD_Write_HalfWord(x1 + 40);
        LCD_Write_HalfWord(x2 + 40);
        LCD_Write_Cmd(0x2b); // Аµؖ·ɨփ
        LCD_Write_HalfWord(y1 + 52);
        LCD_Write_HalfWord(y2 + 52);
        LCD_Write_Cmd(0x2c); // ´¢´憷д
    }
}

void LCD_Clear(uint16_t color)
{
    LCD_Fill(0, 0, LCD_Width, LCD_Height, color);
}

/**
 * Ӄһ¸öѕɫ̮³䕻¸öǸӲ
 *
 * @param   x_start,y_start     ưµ㗸±ꍊ * @param   x_end,y_end			֕µ㗸±ꍊ * @param   color       		̮³䑕ɫ
 *
 * @return  void
 */
void LCD_Fill(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t color)
{
    uint16_t i    = 0;
    uint32_t size = 0, size_remain = 0;

    size = (x_end - x_start + 1) * (y_end - y_start + 1) * 2;

    if (size > LCD_Buf_Size) {
        size_remain = size - LCD_Buf_Size;
        size        = LCD_Buf_Size;
    }

    LCD_Address_Set(x_start, y_start, x_end, y_end);

    while (1) {
        for (i = 0; i < size / 2; i++) {
            lcd_buf[2 * i]     = color >> 8;
            lcd_buf[2 * i + 1] = color;
        }

        LCD_DC(1);
        LCD_SPI_Send(lcd_buf, size);

        if (size_remain == 0)
            break;

        if (size_remain > LCD_Buf_Size) {
            size_remain = size_remain - LCD_Buf_Size;
        }

        else {
            size        = size_remain;
            size_remain = 0;
        }
    }
}

void LCD_Init(void)
{
    lcd_interface_init();

    LCD_PWR(1);
    delay_ms(120);
    LCD_RST(0);
    delay_ms(120);
    LCD_RST(1);

    delay_ms(120);
    /* Sleep Out */
    LCD_Write_Cmd(0x11);
    /* wait for power stability */
    delay_ms(120);

    /* Memory Data Access Control */
    LCD_Write_Cmd(0x36);
    if (USE_HORIZONTAL == 0)
        LCD_Write_Data(0x00);
    else if (USE_HORIZONTAL == 1)
        LCD_Write_Data(0xC0);
    else if (USE_HORIZONTAL == 2)
        LCD_Write_Data(0x70);
    else
        LCD_Write_Data(0xA0);

    /* RGB 5-6-5-bit  */
    LCD_Write_Cmd(0x3A);
    LCD_Write_Data(0x05);

    /* Porch Setting */
    LCD_Write_Cmd(0xB2);
    LCD_Write_Data(0x0C);
    LCD_Write_Data(0x0C);
    LCD_Write_Data(0x00);
    LCD_Write_Data(0x33);
    LCD_Write_Data(0x33);

    /*  Gate Control */
    LCD_Write_Cmd(0xB7);
    LCD_Write_Data(0x35);

    /* VCOM Setting */
    LCD_Write_Cmd(0xBB);
    LCD_Write_Data(0x19); // Vcom=1.625V

    /* LCM Control */
    LCD_Write_Cmd(0xC0);
    LCD_Write_Data(0x2C);

    /* VDV and VRH Command Enable */
    LCD_Write_Cmd(0xC2);
    LCD_Write_Data(0x01);

    /* VRH Set */
    LCD_Write_Cmd(0xC3);
    LCD_Write_Data(0x12);

    /* VDV Set */
    LCD_Write_Cmd(0xC4);
    LCD_Write_Data(0x20);

    /* Frame Rate Control in Normal Mode */
    LCD_Write_Cmd(0xC6);
    LCD_Write_Data(0x0F); // 60MHZ

    /* Power Control 1 */
    LCD_Write_Cmd(0xD0);
    LCD_Write_Data(0xA4);
    LCD_Write_Data(0xA1);

    /* Positive Voltage Gamma Control */
    LCD_Write_Cmd(0xE0);
    LCD_Write_Data(0xD0);
    LCD_Write_Data(0x04);
    LCD_Write_Data(0x0D);
    LCD_Write_Data(0x11);
    LCD_Write_Data(0x13);
    LCD_Write_Data(0x2B);
    LCD_Write_Data(0x3F);
    LCD_Write_Data(0x54);
    LCD_Write_Data(0x4C);
    LCD_Write_Data(0x18);
    LCD_Write_Data(0x0D);
    LCD_Write_Data(0x0B);
    LCD_Write_Data(0x1F);
    LCD_Write_Data(0x23);

    /* Negative Voltage Gamma Control */
    LCD_Write_Cmd(0xE1);
    LCD_Write_Data(0xD0);
    LCD_Write_Data(0x04);
    LCD_Write_Data(0x0C);
    LCD_Write_Data(0x11);
    LCD_Write_Data(0x13);
    LCD_Write_Data(0x2C);
    LCD_Write_Data(0x3F);
    LCD_Write_Data(0x44);
    LCD_Write_Data(0x51);
    LCD_Write_Data(0x2F);
    LCD_Write_Data(0x1F);
    LCD_Write_Data(0x1F);
    LCD_Write_Data(0x20);
    LCD_Write_Data(0x23);

    /* Display Inversion On */
    LCD_Write_Cmd(0x21);

    LCD_Write_Cmd(0x29);

    LCD_Address_Set(0, 0, LCD_Width - 1, LCD_Height - 1);

    /*´򿪏Ԋ¾*/
    LCD_PWR(0);
}
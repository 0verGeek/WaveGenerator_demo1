#include "My_lcd.h"

char str[10][20];
extern uint8_t para;
extern float amplitude_v;
extern float frequency_khz;
extern uint8_t mode;

const char* wave_symbols[] = {"Sina", "Square", "Tri", "Saw"};

void lcd_text_init(void)
{
    sprintf(str[1], "   Wave Generator    ");
    sprintf(str[3], "     Wave: %s        ", wave_symbols[mode]);
    sprintf(str[5], " Amplitude: %.2fV    ", amplitude_v);
    sprintf(str[7], " Frequency: %.2fkHz  ", frequency_khz);
}
void lcd_high_light_line(uint8_t Linex)
{
    for (uint8_t i = 0; i < 10; i++)
    {
        if (i != Linex)
        {
            LCD_DisplayStringLine(Line0 + 24 * i, (uint8_t *)str[i]);
        }
        else
        {
            LCD_SetBackColor(Blue);
            LCD_DisplayStringLine(Line0 + 24 * Linex, (uint8_t *)str[Linex]);
            LCD_SetBackColor(Black);
        }
    }
}
void lcd_high_light_ctrl(void)
{
    if (para ==0)
    {
        lcd_high_light_line(5);
    }
    else
    {
        lcd_high_light_line(7);
    }
}
void lcd_proc(void)
{
    lcd_text_init();
    lcd_high_light_ctrl();
}
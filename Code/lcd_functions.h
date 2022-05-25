/*
 * lcd_function.h
 *
 * Created: 20.05.2022 09:26:44
 *  Author: beatf
 */ 


#ifndef LCD_FUNCTIONS
#define LCD_FUNCTIONS

void lcd_cmd(uint8_t cmd);

void lcd_init();

void lcd_zahl(uint8_t zahl,char* text);

void lcd_text(char* ztext);



#endif /* LCD_FUNCTIONS */
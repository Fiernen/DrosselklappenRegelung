#ifndef LCD_FUNCTIONS_H_
#define LCD_FUNCTIONS_H_

void lcd_zahl(uint8_t zahl,char* text);
void lcd_zahl_16(uint16_t num, char* written);
void lcd_zahl_s16(int16_t num, char* written);
void lcd_angle(uint32_t num, char* written);
void lcd_text(char* ztext);
void lcd_cmd(unsigned char cmd);
void lcd_init();

#endif /* LCD_FUNCTIONS_H_ */
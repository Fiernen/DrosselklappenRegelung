#ifndef EEPROM_FUNCTION_H_
#define EEPROM_FUNCTION_H_


void save_ctrl_params2EEPROM(uint8_t kP_position_2_save, uint8_t kP_speed_2_save, uint8_t TN_speed_2_save);
void read_ctrl_params_from_EEPROM();


#endif /* EEPROM_FUNCTION_H_ */
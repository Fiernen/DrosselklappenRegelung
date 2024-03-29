#include "project_header.h"
#include "EEPROM_function.h"
#include <avr/eeprom.h>

uint8_t ee_kP_position EEMEM = default_kP_position;
uint8_t ee_kP_speed EEMEM = default_kP_speed;
uint8_t ee_TN_speed EEMEM = default_kP_TN_speed;

/* save_ctrl_params2EEPROM() updates the Values for the control parameters in the EEPROM
	updating reduces number of write cycles
*/
void save_ctrl_params2EEPROM(uint8_t kP_position, uint8_t kP_speed, uint8_t TN_speed)
{
	eeprom_update_byte(&ee_kP_position, kP_position);
	eeprom_update_byte(&ee_kP_speed, kP_speed);
	eeprom_update_byte(&ee_TN_speed, TN_speed);
}

/* read_ctrl_params_from_EEPROM() updates the control parameters from the EEPROM
	Used in the initialization after power loss to recover control parameters.
*/
void read_ctrl_params_from_EEPROM(uint8_t* kP_position, uint8_t* kP_speed, uint8_t* TN_speed)
{
	*kP_position = eeprom_read_byte(&ee_kP_position);
	*kP_speed = eeprom_read_byte(&ee_kP_speed);
	*TN_speed = eeprom_read_byte(&ee_TN_speed);
}


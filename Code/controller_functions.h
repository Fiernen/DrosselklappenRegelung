#ifndef CONTROLLER_FUNCTIONS_H_
#define CONTROLLER_FUNCTIONS_H_

// For documentation go to the .c-file

void TimerController_init();
void ADConverter_init();
void TimerPWM_init(void);
uint16_t setpoint_measure(void);
int16_t position_measure(void);
uint16_t FIR_filter(uint16_t new_value);
uint16_t FIR_filter2(uint16_t new_value);
int16_t check_int16_overunderflow(int32_t var);
int16_t limit_int16(int32_t var, int16_t MIN, int16_t MAX);
int64_t limit_integral(int64_t var, int32_t MIN, int32_t MAX);
uint16_t Motor_controller(uint16_t position, uint16_t position_setpoint, uint8_t kP_position, uint8_t kP_speed, uint8_t TN_speed);
void reset_integral(void);

#endif /* CONTROLLER_FUNCTIONS_H_ */
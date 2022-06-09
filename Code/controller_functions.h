#ifndef CONTROLLER_FUNCTIONS_H_
#define CONTROLLER_FUNCTIONS_H_

// For documentation go to the .c-file

void TimerController_init();
void ADConverter_init();
void TimerPWM_init(void);
int16_t position_measure(void);
uint16_t FIR_filter(uint16_t new_value);
int16_t check_int16_overunderflow(int32_t var);
int16_t limit_int16(int32_t var, int16_t MIN, int16_t MAX);
int32_t limit_integral(int32_t var, int32_t MIN, int32_t MAX);
int16_t Motor_controller(uint16_t position, struct controller_params *params);

#endif /* CONTROLLER_FUNCTIONS_H_ */
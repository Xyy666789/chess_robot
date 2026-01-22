#pragma once

void gpio_init(void);
void bt_detection(void);
void Input_detection(void);

void set_pwm_duty(uint32_t duty);
void setup_pwm(void);
void airpump_enable(void);
void airpump_disable(void);
void motz_op(uint8_t type);
void motor_28BYJ_Zaxis(uint8_t type) ;//控制吸盘电机

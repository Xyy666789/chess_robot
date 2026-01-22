#ifndef __MOTOR_H__
#define __MOTOR_H__


void motor_gpio_init();
void MotorClockwise();                                // 顺时针一直转
void MotorCounterClockwise();                         // 逆时针一直转
void MotorClockwise_Angle(unsigned int angle);        // 顺时针转x度
void MotorCounterClockwise_Angle(unsigned int angle); // 逆时针转x度
void MotorClockwise_Lap(unsigned int Lap);            // 顺时针转x圈
void MotorCounterClockwise_Lap(unsigned int Lap);     // 逆时针转x圈
void MotorStop();                                     // 停止
extern unsigned char Speed_28BYJ;
#endif
/******************************************************************************
	··· 接线：P20~P23 ---> IN1~IN4

	··· 驱动方式：1-2相励磁法(4相8拍)
			0000 0001
			0000 0011
			0000 0010
			0000 0110
			0000 0100
			0000 1100
			0000 1000
			0000 1001

	··· 28BYJ-48参数计算逻辑：
			减速比1:64
			步进角5.625度，也就是里面的转子每走一步是转动5.625度
			轴的步进角：因为减速比是1:64，里面的转子走一步是转动5.625度，也就是说转子走一步实际外部的轴是转动5.625/64 = 0.08789度
			那么举例外部轴转一圈360度需要里面的转子走多少步呢？ 答：360/0.087489 = 4096步
			使用的是8拍励磁法，那转一圈就是需要遍历4096 / 8 = 512 次，这样才能走够一圈。

*******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <stdio.h>
#define MOTOR_IN1 GPIO_NUM_9
#define MOTOR_IN2 GPIO_NUM_10
#define MOTOR_IN3 GPIO_NUM_11
#define MOTOR_IN4 GPIO_NUM_12
float Speed_28BYJ = 0.6; // 速度控制 越小越快（单位ms）
// int Speed_28BYJ = 1;  // 速度控制 越小越快（单位ms）
unsigned char PinData[] = {0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09};

void motor_delay_ms(float ms)
{
	esp_rom_delay_us((int)(ms * 1000));
}

// void motor_delay_ms(int ms)
// {
// 	vTaskDelay(pdMS_TO_TICKS(ms));
// }

void motor_gpio_init(void)
{
	gpio_set_direction(MOTOR_IN1, GPIO_MODE_OUTPUT);
	gpio_set_direction(MOTOR_IN2, GPIO_MODE_OUTPUT);
	gpio_set_direction(MOTOR_IN3, GPIO_MODE_OUTPUT);
	gpio_set_direction(MOTOR_IN4, GPIO_MODE_OUTPUT);
}

void set_motor_pins(uint8_t value)
{
	gpio_set_level(MOTOR_IN1, (value >> 0) & 0x01);
	gpio_set_level(MOTOR_IN2, (value >> 1) & 0x01);
	gpio_set_level(MOTOR_IN3, (value >> 2) & 0x01);
	gpio_set_level(MOTOR_IN4, (value >> 3) & 0x01);
}

// 顺时针转
void MotorClockwise()
{
	unsigned char i;

	for (i = 0; i < 8; i++)
	{
		set_motor_pins(PinData[i]);
		motor_delay_ms(Speed_28BYJ);
	}
}

// 逆时针转
void MotorCounterClockwise()
{
	unsigned char i;

	for (i = 0; i < 8; i++)
	{
		set_motor_pins(PinData[7 - i]);
		motor_delay_ms(Speed_28BYJ);
	}
}

// 顺时针转x度
void MotorClockwise_Angle(unsigned int angle)
{
	unsigned char i;
	unsigned int j;
	for (j = 0; j < angle / (5.625 * 8 / 64); j++)
	{
		for (i = 0; i < 8; i++)
		{
			set_motor_pins(PinData[i]);
			motor_delay_ms(Speed_28BYJ);
		}
	}
	// unsigned int j;
	// // 计算总步数：angle / (5.625/64) = angle * 64 / 5.625
	// unsigned int total_steps = (unsigned int)(angle * 64.0 / 5.625 + 0.5);
	// for (j = 0; j < total_steps; j++)
	// {
	// 	set_motor_pins(PinData[j % 8]);
	// 	motor_delay_ms(Speed_28BYJ);
	// 	if (j % 50 == 0)
	// 		vTaskDelay(pdMS_TO_TICKS(1));
	// }
}

// 逆时针转x度
void MotorCounterClockwise_Angle(unsigned int angle)
{
	unsigned char i;
	unsigned int j;
	for (j = 0; j < angle / (5.625 * 8 / 64); j++)
	{
		for (i = 0; i < 8; i++)
		{
			set_motor_pins(PinData[7 - i]);
			motor_delay_ms(Speed_28BYJ);
		}
	}
	// unsigned int j;
	// // 计算总步数：angle / (5.625/64) = angle * 64 / 5.625
	// unsigned int total_steps = (unsigned int)(angle * 64.0 / 5.625 + 0.5);
	// for (j = 0; j < total_steps; j++)
	// {
	// 	set_motor_pins(PinData[j % 8]);
	// 	motor_delay_ms(Speed_28BYJ);
	// 	if (j % 50 == 0)
	// 		vTaskDelay(pdMS_TO_TICKS(1));
	// }
}

// 顺时针转x圈
void MotorClockwise_Lap(unsigned int Lap)
{
	// unsigned char i;
	// unsigned int j;
	// for (j = 0; j < (Lap* ((360 / (5.625 / 64)) / 8)); j++)
	// {
	// 	for (i = 0; i < 8; i++)
	// 	{
	// 		set_motor_pins(PinData[i]);
	// 		motor_delay_ms(Speed_28BYJ);
	// 	}
	// }
	unsigned char i;
	unsigned int j;
	unsigned int total_steps = Lap * ((360 / (5.625 / 64)) / 8) * 8;
	for (j = 0; j < total_steps; j++)
	{
		set_motor_pins(PinData[j % 8]);
		motor_delay_ms(Speed_28BYJ); // 亚毫秒延时
		if (j % 50 == 0)
			vTaskDelay(pdMS_TO_TICKS(1));
	}
}

// 逆时针转x圈
void MotorCounterClockwise_Lap(unsigned int Lap)
{
	// unsigned char i;
	// unsigned int j;
	// for (j = 0; j < (Lap * ((360 / (5.625 / 64)) / 8)); j++)
	// {
	// 	for (i = 0; i < 8; i++)
	// 	{
	// 		set_motor_pins(PinData[7-i]);
	// 		motor_delay_ms(Speed_28BYJ);
	// 	}
	// }
	unsigned int j;
	unsigned int total_steps = Lap * ((360 / (5.625 / 64)) / 8) * 8;
	for (j = 0; j < total_steps; j++)
	{
		set_motor_pins(PinData[7 - (j % 8)]);
		
		if (j % 70 == 0)
			vTaskDelay(pdMS_TO_TICKS(1));
		else 
			motor_delay_ms(Speed_28BYJ);
	}
}

// 停止
void MotorStop()
{
	set_motor_pins(0);
}

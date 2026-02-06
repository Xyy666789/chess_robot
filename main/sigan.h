#pragma once
#ifndef __SIGAN_H__
#define __SIGAN_H__

// 1. 硬件引脚
#define SIGAN_PIN_EN    6
#define SIGAN_PIN_STEP  7
#define SIGAN_PIN_DIR   15

// 2. 机械参数
#define SIGAN_LEAD_MM      3.3f   // 导程
#define SIGAN_STEPS_REV    200.0f // 一圈步数
#define SIGAN_PULSE_DELAY  500    // 速度 (us)

// 3. 函数接口
void sigan_init(void);
void sigan_move_up(float mm);    // 向上移动
void sigan_move_down(float mm);  // 向下移动

#endif
#include "sigan.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

static const char *TAG = "SIGAN";

// 初始化
void sigan_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << SIGAN_PIN_EN) | (1ULL << SIGAN_PIN_STEP) | (1ULL << SIGAN_PIN_DIR);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(SIGAN_PIN_EN, 1); // 默认释放
    ESP_LOGI(TAG, "丝杆电机初始化完成");
}

// 内部驱动函数
static void sigan_step_run(int steps, int dir)
{
    gpio_set_level(SIGAN_PIN_EN, 0); // 使能
    ets_delay_us(50);
    
    gpio_set_level(SIGAN_PIN_DIR, dir); // 方向
    ets_delay_us(50);

    for (int i = 0; i < steps; i++) {
        gpio_set_level(SIGAN_PIN_STEP, 1);
        ets_delay_us(SIGAN_PULSE_DELAY);
        gpio_set_level(SIGAN_PIN_STEP, 0);
        ets_delay_us(SIGAN_PULSE_DELAY);
        
        // 防止长距离移动时卡死系统
        if (i % 100 == 0) vTaskDelay(1);
    }
    
    gpio_set_level(SIGAN_PIN_EN, 1); // 释放省电
}

// 向上接口
void sigan_move_up(float mm)
{
    if (mm <= 0) return;
    int steps = (int)((mm / SIGAN_LEAD_MM) * SIGAN_STEPS_REV);
    ESP_LOGI(TAG, "UP: %.2fmm (%d steps)", mm, steps);
    sigan_step_run(steps, 1); // 1 = UP
}

// 向下接口
void sigan_move_down(float mm)
{
    if (mm <= 0) return;
    int steps = (int)((mm / SIGAN_LEAD_MM) * SIGAN_STEPS_REV);
    ESP_LOGI(TAG, "DOWN: %.2fmm (%d steps)", mm, steps);
    sigan_step_run(steps, 0); // 0 = DOWN
}
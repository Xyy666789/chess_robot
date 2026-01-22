#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "main.h"
#include "motor.h"
#include "gpio.h"
#include "Motor_28BYJ.h"

// 按钮定义，从左到右
static const char *TAG = "GPIO";

void gpio_init(void)
{
    grt.bt_pin[0] = GPIO_NUM_40;
    grt.bt_pin[1] = GPIO_NUM_41;
    grt.bt_pin[2] = GPIO_NUM_42;
    grt.bt_pin[3] = GPIO_NUM_2;
    grt.bt_pin[4] = GPIO_NUM_39;
    grt.bt_pin[5] = GPIO_NUM_1; // 下棋测试按钮

    esp_rom_gpio_pad_select_gpio(AIP_PUMP_PIN);
    gpio_set_direction(AIP_PUMP_PIN, GPIO_MODE_OUTPUT);

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;                                                       // 关闭中断
    io_conf.mode = GPIO_MODE_OUTPUT;                                                             // 设、为输出模式
    io_conf.pin_bit_mask = (1ULL << AIP_PUMP_PIN | 1ULL << MOTZ_FWD_PIN | 1ULL << MOTZ_REV_PIN); // 设置要操作的GPIO引脚，这里以GPIO2为例
    // io_conf.pin_bit_mask = (1ULL << GPIO_NUM_38 | 1ULL << MOTZ_FWD_PIN | 1ULL << MOTZ_REV_PIN); // 设置要操作的GPIO引脚，这里以GPIO2为例

    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // 禁用下拉电阻
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // 禁用上拉电阻
    gpio_config(&io_conf);
    gpio_set_level(AIP_PUMP_PIN, 0);
    // gpio_set_level(GPIO_NUM_38, 0);

    // 配置传感器引脚上拉
    io_conf.intr_type = GPIO_INTR_DISABLE;                                                                                                                                                      // 禁用中断
    io_conf.mode = GPIO_MODE_INPUT;                                                                                                                                                             // 输入模式
    io_conf.pin_bit_mask = (1ULL << grt.bt_pin[0] | 1ULL << grt.bt_pin[1] | 1ULL << grt.bt_pin[2] | 1ULL << grt.bt_pin[3] | 1ULL << grt.bt_pin[4] | 1ULL << grt.bt_pin[5] | 1ULL << VACUO_PIN); // 设置引脚
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;                                                                                                                                                    // 启用上拉
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                                                                                                                                               // 禁用下拉
    gpio_config(&io_conf);

    setup_pwm(); // 初始化 PWM
}

uint16_t bt_time[6] = {0};
bool air_sta = 0;
uint16_t jitter_0 = 0, jitter_1 = 0;
// 输入检测
void bt_detection(void)
{
    // 按钮
    for (int i = 0; i < 6; i++)
    {
        if (gpio_get_level(grt.bt_pin[i]) == 0)
        {

            bt_time[i]++;
            if (bt_time[i] == 130)
            {
                grt.bt_state[i] = 2; // 触发长按
                ESP_LOGI("BT", "BT%d long press", bt_time[i]);
            }
            if (bt_time[i] > 130)
            {
                bt_time[i] = 151;
            }
            jitter_1 = 0;
        }
        else
        {
            jitter_1++;
            if (jitter_1 > 2)
            {
                if (bt_time[i] > 8 && bt_time[i] < 100)
                {
                    grt.bt_state[i] = 1; // 触发短按
                    ESP_LOGI("BT", "BT%d short press", bt_time[i]);
                }
                bt_time[i] = 0;
            }
        }
    }
    if (grt.bt_state[5] == 2)
    {
        grt.test = 1 - grt.test; // 测试标志
        ESP_LOGI("bt", "测试, grt.test = %d", grt.test);
    }
    if (grt.test == 0)
    { // 正常模式
        if (grt.bt_state[0] == 1)
        {
            grt.event[0] = 1; // 开始对局
            ESP_LOGI("bt", "grt.event = 1, 开始对局");
        }
        else if (grt.bt_state[0] == 2)
        {
            grt.event[1] = 2; // 结束对局
            ESP_LOGI("bt", "grt.event = 2, 结束对局");
        }

        if (grt.bt_state[1] == 1)
        {
            grt.event[2] = 3; // 选择先后手  这里应该全是bool型变量
            ESP_LOGI("bt", "grt.event = 3, 选择先后手");
        }

        if (grt.bt_state[2] == 1)
        {
            grt.event[3] = 4; // 音量调节
            ESP_LOGI("bt", "grt.event = 4, 音量调节");
        }
        else if (grt.bt_state[2] == 2)
        {
            grt.event[4] = 5; // 复位
            ESP_LOGI("bt", "grt.event = 5, 复位");
        }

        if (grt.bt_state[3] == 1)
        {
            grt.event[5] = 6; // 提示
            ESP_LOGI("bt", "grt.event = 6, 提示");
        }
        else if (grt.bt_state[3] == 2)
        {
            grt.event[6] = 7; // 悔棋
            ESP_LOGI("bt", "grt.event = 7, 悔棋");
        }

        if (grt.bt_state[4] == 1)
        {
            grt.event[7] = 8; // 确认/下一步
            ESP_LOGI("bt", "grt.event = 8, 确认/下一步");
        }

        if (grt.bt_state[5] == 1)
        { // 单次下棋动作测试
            grt.play_posx = -((float)1 * UNIT_DISTANCE_X + ORIGIN_OFFSET_X);
            grt.play_posy = (float)1 * UNIT_DISTANCE_Y + ORIGIN_OFFSET_Y;
            grt.cmd = 1;
        }
    }
    else
    { // 测试模式
        if (grt.bt_state[0] == 1)
        {
            motz_op(1);
            ESP_LOGI("bt", "Z轴正转");
        }
        if (grt.bt_state[1] == 1)
        {
            motz_op(2);
            ESP_LOGI("bt", "Z轴反转");
        }
        if (grt.bt_state[2] == 1)
        {
            if (air_sta == 0)
            {
                air_sta = 1;
                airpump_enable();
                ESP_LOGI("bt", "气泵开");
            }
            else
            {
                air_sta = 0;
                airpump_disable();
                ESP_LOGI("bt", "气泵关");
            }
        }
    }
    for (int i = 0; i < 8; i++)
    {
        grt.bt_state[i] = 0; // 按钮状态清零
    }

    // 真空检测：读取原始电平并打印结果
    {
        int vac_level = gpio_get_level(VACUO_PIN);
        grt.vacuo_state = (vac_level == 0) ? 0 : 1;
        ESP_LOGI(TAG, "VACUO_PIN level=%d -> vacuo_state=%d", vac_level, grt.vacuo_state);
    }
}

void Input_detection()
{
    int vac_level = gpio_get_level(VACUO_PIN);
    grt.vacuo_state = (vac_level == 0) ? 0 : 1;
    // ESP_LOGI(TAG, "VACUO_PIN level=%d -> vacuo_state=%d", vac_level, grt.vacuo_state);
}
void motz_op(uint8_t type)
{                  // 控制吸盘电机
    if (type == 1) // 正转
    {
        gpio_set_level(MOTZ_FWD_PIN, 1); //
        gpio_set_level(MOTZ_REV_PIN, 0); //
        vTaskDelay(pdMS_TO_TICKS(1400));
        gpio_set_level(MOTZ_FWD_PIN, 0); //
        gpio_set_level(MOTZ_REV_PIN, 0); //
    }
    else if (type == 2) // 反转
    {
        gpio_set_level(MOTZ_FWD_PIN, 0); //
        gpio_set_level(MOTZ_REV_PIN, 1); //
        vTaskDelay(pdMS_TO_TICKS(2400));
        gpio_set_level(MOTZ_FWD_PIN, 0); //
        gpio_set_level(MOTZ_REV_PIN, 0); //
    }
    else
    {
        gpio_set_level(MOTZ_FWD_PIN, 0); //
        gpio_set_level(MOTZ_REV_PIN, 0); //
    }
}

void motor_28BYJ_Zaxis(uint8_t type)
{ // 控制吸盘电机
    Speed_28BYJ = 1;
    if (type == 1)
    {
        // MotorCounterClockwise_Angle(100);
        MotorClockwise_Angle(115);
    }
    else if (type == 2)
    {
        // MotorClockwise_Angle(80);
         MotorCounterClockwise_Angle(110);
    }
    else if (type == 3)
    {
        MotorClockwise_Angle(70);
    }
    else if (type == 4)
    {
        MotorCounterClockwise_Angle(70);
    }
    else if (type == 5)
    {
        MotorCounterClockwise_Angle(45);
    }
    // 停止
    MotorStop();
}

// 打开气泵
void airpump_enable(void)
{
    // gpio_set_level(AIP_PUMP_PIN, 1);
    uint32_t duty = (1 << 13) * grt.air_duty / 100; // 13 位分辨率，50% 占空比
    set_pwm_duty(duty);

    // ESP_LOGI("airpump", "气泵开, duty = %d", grt.air_duty);
}

// 关闭气泵
void airpump_disable(void)
{
    // gpio_set_level(AIP_PUMP_PIN, 0);
    uint32_t duty = 0; // 13 位分辨率，50% 占空比
    set_pwm_duty(duty);
    // ESP_LOGI("airpump", "气泵关");
}

// 初始化 PWM
void setup_pwm(void)
{
    // 配置 LEDC 定时器
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // 配置 LEDC 通道
    ledc_channel_config_t channel_conf = {
        .gpio_num = LEDC_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0, // 初始占空比为 0
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
}

// 修改占空比，控制气泵电机转速
void set_pwm_duty(uint32_t duty)
{
    // 设置 PWM 占空比
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    // 更新占空比
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

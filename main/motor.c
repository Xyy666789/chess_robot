#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <math.h>
#include "motor.h"
#include "main.h"
#include "uart.h"

static const int RX_BUF_SIZE = 1024;
static const int TX_BUF_SIZE = 1024;
static const char *TAG_TX = "COM_MOT_TX";
static const char *TAG_RX = "COM_MOT_RX";
static const char *TAG_CTL = "MOT_CTL";
ZDTMOT_t ZMOT;
motor_t motor[MOT_COT];

uint32_t overtime = 0;              // 超时计时
uint8_t cmd_ori[MOT_COT] = {0};     // 电机复位命令标志
uint8_t cmd_pos[MOT_COT] = {0};     // 电机位置命令标志
uint8_t cmd_sta[MOT_COT] = {0};     // 电机状态查询命令标志
uint8_t cmd_rel[MOT_COT] = {0};     // 电机解除堵转保护命令标志
uint8_t cmd_enab[MOT_COT] = {0};    // 电机使能命令标志
uint8_t cmd_disable[MOT_COT] = {0}; // 电机失能命令标志
uint8_t cmd_stop[MOT_COT] = {0};    // 电机立即停止命令标志
uint8_t cmd_int_ori[MOT_COT] = {0}; // 电机中断复位命令标志
uint8_t cmd_sync = 0;               // 多电机同步运动
uint8_t cmd_set_origin = 0;         // 原点设置命令标志
uint8_t cmd_type = 0;

// 电机命令发送控制
void zmot_cmd(void *arg)
{
    while (1)
    {
        // 立即停止命令控制
        for (int i = 0; i < MOT_COT; i++)
        {
            while (cmd_stop[i])
            {
                if (cmd_stop[i] == 1)
                {
                    ZMOT.stop(i, 0);
                    cmd_stop[i] = 2;
                    motor[i].ack = 0;
                    overtime = 0;
                }
                else if (cmd_stop[i] == 2)
                {
                    if (motor[i].ack == 1)
                    {
                        motor[i].ack = 0;
                        cmd_stop[i] = 0;
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                    overtime++;
                    if (overtime >= 100)
                    {
                        cmd_stop[i] = 1;
                        // ESP_LOGE(TAG, "电机%d解除堵转保护ack回复超时, 跳过此电机", i);
                    }
                }
                else
                {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        // 中断复位命令
        for (int i = 0; i < MOT_COT; i++)
        {
            while (cmd_int_ori[i])
            {
                if (cmd_int_ori[i] == 1)
                {
                    ZMOT.forced_int(i);
                    cmd_int_ori[i] = 2;
                    motor[i].ack = 0;
                    overtime = 0;
                }
                else if (cmd_int_ori[i] == 2)
                {
                    if (motor[i].ack == 1)
                    {
                        motor[i].ack = 0;
                        cmd_int_ori[i] = 0;
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                    overtime++;
                    if (overtime >= 100)
                    {
                        cmd_int_ori[i] = 1;
                        // ESP_LOGE(TAG, "电机%d解除堵转保护ack回复超时, 跳过此电机", i);
                    }
                }
                else
                {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        // 解除堵转保护命令控制
        for (int i = 0; i < MOT_COT; i++)
        {
            while (cmd_rel[i])
            {
                if (cmd_rel[i] == 1)
                {
                    ZMOT.unlock(i);
                    cmd_rel[i] = 2;
                    motor[i].ack = 0;
                    overtime = 0;
                }
                else if (cmd_rel[i] == 2)
                {
                    if (motor[i].ack == 1)
                    {
                        motor[i].ack = 0;
                        cmd_rel[i] = 0;
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                    overtime++;
                    if (overtime >= 100)
                    {
                        cmd_rel[i] = 1;
                        // ESP_LOGE(TAG, "电机%d解除堵转保护ack回复超时, 跳过此电机", i);
                    }
                }
                else
                {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        // 使能命令控制
        for (int i = 0; i < MOT_COT; i++)
        {
            while (cmd_enab[i])
            {
                if (cmd_enab[i] == 1)
                {
                    ZMOT.enable(i, 1);
                    // ESP_LOGE(TAG, "电机%d使能命令已发送", i);
                    motor[i].ack = 0;
                    cmd_enab[i] = 2;
                    overtime = 0;
                }
                else if (cmd_enab[i] == 2)
                {
                    if (motor[i].ack == 1)
                    {
                        motor[i].ack = 0;
                        cmd_enab[i] = 0;
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                    overtime++;
                    if (overtime >= 100)
                    {
                        cmd_enab[i] = 1;
                        // ESP_LOGE(TAG, "电机%d使能ack回复超时, 跳过此电机", i);
                    }
                }
                else
                {
                    // ESP_LOGE(TAG, "电机%d无动作，退出使能命令", i);
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }

        // 失能命令控制
        for (int i = 0; i < MOT_COT; i++)
        {
            while (cmd_disable[i])
            {
                if (cmd_disable[i] == 1)
                {
                    ZMOT.enable(i, 0);
                    ESP_LOGE(TAG_CTL, "电机%d失能命令已发送", i);
                    motor[i].ack = 0;
                    cmd_disable[i] = 2;
                    overtime = 0;
                }
                else if (cmd_disable[i] == 2)
                {
                    if (motor[i].ack == 1)
                    {
                        motor[i].ack = 0;
                        cmd_disable[i] = 0;
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                    overtime++;
                    if (overtime >= 100)
                    {
                        cmd_disable[i] = 1;
                        ESP_LOGE(TAG_CTL, "电机%d失能ack回复超时, 跳过此电机", i);
                    }
                }
                else
                {
                    // ESP_LOGE(TAG, "电机%d无动作，退出失能命令", i);
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        /* 复位命令控制*/
        for (int i = 0; i < MOT_COT; i++)
        {
            while (cmd_ori[i])
            {
                if (cmd_ori[i] == 1)
                {
                    ZMOT.origin(i, motor[i].omode, 1);
                    // ESP_LOGI(TAG, "电机%d复位命令已发送", i);
                    motor[i].ack = 0;
                    cmd_ori[i] = 2;
                    overtime = 0;
                }
                else if (cmd_ori[i] == 2)
                { // ESP_LOGE(TAG, "电机%d等待ack", i);
                    if (motor[i].ack == 1)
                    {
                        motor[i].ack = 0;
                        // ESP_LOGE(TAG, "电机%dack收到", i);
                        cmd_ori[i] = 0;
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                    overtime++;
                    if (overtime >= 100)
                    {
                        cmd_ori[i] = 1;
                        // ESP_LOGE(TAG, "电机%d复位ack回复超时, 重新发送一次", i);
                    }
                }
                else
                {
                    // ESP_LOGE(TAG, "电机%d无动作，退出复位命令", i);
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        /* 位置命令控制*/
        for (int i = 0; i < MOT_COT; i++)
        {
            while (cmd_pos[i])
            {
                if (cmd_pos[i] == 1)
                {
                    ZMOT.set_pos(i, motor[i].pos_dir, motor[i].tar_sp, motor[i].acc, motor[i].tar_pul, motor[i].abs_mode, motor[i].pos_sync);
                    // ZMOT.set_pos(i, motor[i].pos_dir, motor[i].acc, motor[i].acc, motor[i].tar_sp, motor[i].tar_pos, motor[i].abs_mode, motor[i].pos_sync);
                    ESP_LOGE(TAG_CTL, "电机%d位置命令已发送,", i);
                    motor[i].ack = 0;
                    cmd_pos[i] = 2;
                    overtime = 0;
                }
                else if (cmd_pos[i] == 2)
                {
                    if (motor[i].ack == 1)
                    {
                        motor[i].ack = 0;
                        motor[i].confirm = 1;
                        // ESP_LOGE(TAG, "电机%dack收到", i);
                        cmd_pos[i] = 0;
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                    overtime++;
                    if (overtime >= 100)
                    {
                        cmd_pos[i] = 1;
                        ESP_LOGE(TAG_CTL, "电机%d位置命令回复超时, 重新发送一次", i);
                    }
                }
                else
                {
                    // ESP_LOGE(TAG, "电机%d无动作，退出位置命令", i);
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        sync_while();
        /* 状态查询命令 */
        for (int i = 0; i < MOT_COT; i++)
        {
            cmd_sta[i] = 1;
            while (cmd_sta[i])
            {
                if (cmd_sta[i] == 1)
                {
                    ZMOT.get_sys_status(i);
                    // ESP_LOGE(TAG, "电机%d获取状态信息命令已发送", i);
                    motor[i].ack = 0;
                    cmd_sta[i] = 2;
                    overtime = 0;
                }
                else if (cmd_sta[i] == 2)
                {
                    if (motor[i].ack == 1)
                    {
                        motor[i].ack = 0;
                        // ESP_LOGE(TAG, "电机%dack收到", i);
                        cmd_sta[i] = 3;
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                    overtime++;
                    if (overtime >= 20)//电机状态获取间隔太长导致响应速度慢？
                    {
                        cmd_sta[i] = 0;
                        // ESP_LOGE(TAG, "电机%d状态信息获取超时, 跳过此电机", i);
                        break;
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// SYNC同步命令控制
void sync_while(void)
{
    // ESP_LOGE(TAG, "SYNC命令检查");
    if (cmd_sync == 1)
    {
        while (1)
        {
            if (cmd_sync == 1)
            {
                ZMOT.sync(0);
                ESP_LOGE(TAG_CTL, "已发送SYNC命令");
                motor[0].ack = 0;
                cmd_sync = 2;
                overtime = 0;
            }
            if (cmd_sync == 2)
            {
                if (motor[0].ack == 1)
                {
                    cmd_sync = 0;
                    motor[0].ack = 0;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
                overtime++;
                if (overtime >= 100)
                {
                    cmd_sync = 1;
                    ESP_LOGE(TAG_CTL, "SYNC回复超时, 重新发送一次");
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// 原点设置命令控制
void set_origion_pos_while(void)
{
    if (cmd_set_origin == 1)
    {
        while (1)
        {
            if (cmd_set_origin == 1)
            {
                ZMOT.set_origion_pos();
                ESP_LOGE(TAG_CTL, "已发送原点设置命令");
                motor[0].ack = 0;
                cmd_set_origin = 2;
                overtime = 0;
            }
            if (cmd_set_origin == 2)
            {
                if (motor[0].ack == 1)
                {
                    cmd_set_origin = 0;
                    motor[0].ack = 0;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
                overtime++;
                if (overtime >= 100)
                {
                    cmd_set_origin = 1;
                    ESP_LOGE(TAG_CTL, "原点设置回复超时, 重新发送一次");
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// 电机控制串口初始化
void motor_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,                   // 设置波特率
        .data_bits = UART_DATA_8_BITS,         // 数据位数
        .parity = UART_PARITY_DISABLE,         // 奇偶校验
        .stop_bits = UART_STOP_BITS_1,         // 停止位数
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // 流控关闭
        .source_clk = UART_SCLK_DEFAULT,
        .rx_flow_ctrl_thresh = 0, // 流控阈值
    };

    // 初始化UART1
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, MOTOR_TX_PIN, MOTOR_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE, TX_BUF_SIZE, 20, NULL, 0); // 安装驱动，缓冲区大小可以根据需要调整
}

// 发送数据到电机
void motor_tx_send(uint8_t *data, uint8_t len)
{
    uart_write_bytes(UART_NUM_2, data, len);
    ESP_LOG_BUFFER_HEX(TAG_TX, data, len);
}

// 电机串口接收任务
void motor_rx_task(void *arg)
{
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    // ESP_LOGI(TAG_RX, "电机串口接收启动");
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 100 / portTICK_PERIOD_MS);//100
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            // ESP_LOGE(TAG, "收到电机串口消息 %d bytes:", rxBytes);
            ESP_LOG_BUFFER_HEX(TAG_RX, data, rxBytes);
            // ESP_LOG_BUFFER_HEXDUMP(TAG, data, rxBytes, ESP_LOG_INFO);
            grt.mot_wait = 0;
            if (rxBytes >= 4 && data[2] == 0x02 && data[3] == 0x6B)
            { // 确保至少有两个字母且都是字母  收到电机简单应答数据
                motor[data[0] - 1].ack = 1;
            }
            else if (rxBytes >= 31 && data[1] == 0x43 && data[30] == 0x6B)
            { // 收到电机详细状态数据

                uint8_t id = data[0] - 1; // 电机编号
                motor[id].ack = 1;
                motor[id].cur_sp = data[16] * 0x100 + data[17];
                if (data[15])
                {
                    motor[id].cur_sp = -motor[id].cur_sp;
                }
                motor[id].cur_pul = (data[19] << 24) | (data[20] << 16) | (data[21] << 8) | data[22];
                motor[id].err_pul = (data[24] << 24) | (data[25] << 16) | (data[26] << 8) | data[27];
                // motor[id].cur_pul = data[19] * 0x1000000 + data[20] * 0x10000 + data[21] * 0x100 + data[22];
                // motor[id].err_pul = data[24] * 0x1000000 + data[25] * 0x10000 + data[26] * 0x100 + data[27];
                int8_t mdir = 1;
                if (data[18])
                {
                    mdir = -1;
                }
                switch (id)
                {
                case 0:
                    motor[id].cur_pos = (float)motor[id].cur_pul * DPR_X / 65536;
                    motor[id].cur_pos = mdir * motor[id].back_dir * motor[id].cur_pos;
                    motor[id].err_pos = (float)motor[id].err_pul * DPR_X / 65536;
                    break;
                case 1:
                    motor[id].cur_pos = (float)motor[id].cur_pul * DPR_Y / 65536;
                    motor[id].cur_pos = mdir * motor[id].back_dir * motor[id].cur_pos;
                    motor[id].err_pos = (float)motor[id].err_pul * DPR_Y / 65536;
                    break;
                case 2:
                case 3:
                    motor[id].cur_pos = (float)motor[id].cur_pul * DPR_T / 65536 / 3;
                    motor[id].cur_pos = mdir * motor[id].back_dir * motor[id].cur_pos;
                    motor[id].err_pos = (float)motor[id].err_pul * DPR_T / 65536 / 3;
                    break;
                default:
                    break;
                }
                motor[id].cur_sp = (float)motor[id].cur_sp / 65536 * 60;
                // ESP_LOGE(TAG, "motor: %d..sp = %d, pos = %f, pul = %d, mdir = %d, dir = %d", id,motor[id].cur_sp, motor[id].cur_pos, motor[id].cur_pul, mdir, motor[id].dir);
                motor[id].ostate.encoder_ready = data[28] >> 0 & 0x01;
                motor[id].ostate.calib_ready = data[28] >> 1 & 0x01;
                motor[id].ostate.origining = data[28] >> 2 & 0x01;
                motor[id].ostate.origin_failed = data[28] >> 3 & 0x01;

                motor[id].mstate.enable = data[29] >> 0 & 0x01;
                motor[id].mstate.reached = data[29] >> 1 & 0x01;
                motor[id].mstate.stalled = data[29] >> 2 & 0x01;
                motor[id].mstate.stallProt = data[29] >> 3 & 0x01;

                // if (motor[id].ostate.orif && motor[id].mstate.reached && !motor[id].ostate.origin_failed && abs(motor[id].cur_pos) <= 10) {
                if (motor[id].ostate.orif && motor[id].mstate.reached && abs(motor[id].cur_pos) <= 10)
                {
                    motor[id].ostate.origined = 1;
                    motor[id].ostate.orif = 0;
                }
                // ESP_LOGE("MOTOR", "motor[%d].ostate.orif = %d, mstate.reached = %d, ostate.origin_failed = %d, cur_pos = %.2f",
                //          id, motor[id].ostate.orif, motor[id].mstate.reached, motor[id].ostate.origin_failed, motor[id].cur_pos);
                // ESP_LOGI("MOTOR", "motor: %d..%d,%d,%d,%d,%d,%d,%d,%d", id,motor[id].ostate.encoder_ready,motor[id].ostate.calib_ready,motor[id].ostate.origining,motor[id].ostate.origin_failed,motor[id].mstate.enable,motor[id].mstate.reached,motor[id].mstate.stalled,motor[id].mstate.stallProt);
            }
        }
    }
    free(data);
}

// 复位
void back_origin(uint8_t m_num)
{
    cmd_ori[m_num] = 1; // 电机复位标志
    motor[m_num].ostate.orif = true;
}

// ZMOT初始化参数
void ZMOT_init(void)
{
    ZMOT.enable = ZDTMOT_enable;
    ZMOT.set_sp = ZDTMOT_sp;
    ZMOT.set_pos = ZDTMOT_pos;
    // ZMOT.set_pos = ZDT_X42_V2_Traj_Position_Control;
    ZMOT.stop = ZDTMOT_stop;
    ZMOT.sync = ZDTMOT_sync;
    ZMOT.origin = ZDTMOT_origin;
    ZMOT.forced_int = ZDTMOT_forced_int;
    ZMOT.get_origin_flag = ZDTMOT_get_origin_flag;
    ZMOT.calibrate = ZDTMOT_calibrate;
    ZMOT.clear_pos = ZDTMOT_clear_pos;
    ZMOT.unlock = ZDTMOT_unlock;
    ZMOT.factory_reset = ZDTMOT_factory_reset;
    ZMOT.get_tar_pos = ZDTMOT_get_tar_pos;
    ZMOT.get_cur_sp = ZDTMOT_get_cur_sp;
    ZMOT.get_cur_pos = ZDTMOT_get_cur_pos;
    ZMOT.get_cur_err = ZDTMOT_get_cur_err;
    ZMOT.get_status = ZDTMOT_get_status;
    ZMOT.get_sys_status = ZDTMOT_get_sys_status;
    ZMOT.set_origion_pos = ZDTMOT_set_origion_pos;

    motor[0].back_dir = 1;
    motor[1].back_dir = 1;
    motor[2].back_dir = 1;
    motor[3].back_dir = 1;
    motor[0].omode = 0;
    motor[1].omode = 1;
    motor[2].omode = 1;
    motor[3].omode = 1;
    motor[0].pos_sync = 1;
    motor[1].pos_sync = 0;
    motor[2].pos_sync = 0;
    motor[3].pos_sync = 0;
    motor[0].acc = 1;
    motor[1].acc = 2;
    motor[2].acc = 100;
    motor[3].acc = 100;
    motor[0].abs_mode = 1;
    motor[1].abs_mode = 1;
    motor[2].abs_mode = 1;
    motor[3].abs_mode = 1;
    motor[0].pos_dir = 0; //
    motor[1].pos_dir = 1; // 电机二为负方向
    motor[2].pos_dir = 0;
    motor[3].pos_dir = 1;
    motor[0].tar_sp = 8;
    motor[1].tar_sp = 25;
    motor[2].tar_sp = 60;
    motor[3].tar_sp = 60;
}

int check_origin(void)
{
    for (int i = 0; i < 2; i++)
    {
        if (motor[i].ostate.origined != 1)
        {
            return 0;
        }
    }
    return 1;
}

// 设置目标位置
void set_pos(uint8_t m_num, float pos, uint16_t sp)
{
    motor[m_num].tar_pos = pos;
    motor[m_num].tar_sp = sp;
    motor[m_num].err_pos = pos - motor[m_num].cur_pos;
    ESP_LOGI(TAG_CTL, "给定目标位置：%f, 速度：%d, 差值%f", motor[m_num].tar_pos, motor[m_num].tar_sp, motor[m_num].err_pos);
}

// 设置目标角度
void set_angle(uint8_t m_num, float angle, uint16_t sp)
{
    motor[m_num].tar_pos = angle;
    motor[m_num].tar_sp = sp;
    motor[m_num].err_pos = motor[m_num].tar_pos - motor[m_num].cur_pos;
}

// 检查是否到达目标位置
bool check_reach(uint8_t m_num)
{
    if (m_num == 0 && motor[m_num].pos_dir == 1)
        motor[m_num].err_pos = -motor[m_num].tar_pos - motor[m_num].cur_pos;
    else
        motor[m_num].err_pos = motor[m_num].tar_pos - motor[m_num].cur_pos; // 位置误差考虑旋转方向的正负号 不然跳不出
    bool reached = (fabs(motor[m_num].err_pos) < 1.1 && motor[m_num].mstate.reached);
    ESP_LOGI("CHECK_REACH", "motor[%d] err_pos=%.2f, reached=%d", m_num, motor[m_num].err_pos, reached);
    return reached;
}

// 串口控制电机
// 使能电机，state = 01使能，正确返回01 F3 02 6B，条件不满足返回01 F3 E2 6B，错误命令返回01 00 EE 6B
void ZDTMOT_enable(uint8_t adr, uint8_t state)
{
    uint8_t txbuff[6] = {adr + 1, 0xF3, 0xAB, state, 0x00, 0x6B};
    motor_tx_send((uint8_t *)txbuff, 6);
}

// 速度控制,命令正确返回01 F6 02 6B，条件不满足返回01 F6 E2 6B，错误命令返回01 00 EE 6B
void ZDTMOT_sp(uint8_t adr, uint8_t dir, int16_t sp, uint8_t acc, uint8_t sync)
{
    uint8_t txbuff[8] = {adr + 1, 0xF6, dir, (uint8_t)(sp >> 8), (uint8_t)sp, acc, sync, 0x6B};
    ESP_LOG_BUFFER_HEX(TAG_TX, txbuff, 8);
    motor_tx_send((uint8_t *)txbuff, 8);
}

// 位置控制，16细分 3200脉冲一圈，正确返回01 FD 02 6B，条件不满足返回01 FD E2 6B，错误命令返回01 00 EE 6B
void ZDTMOT_pos(uint8_t adr, uint8_t dir, int16_t sp, uint8_t acc, uint32_t pos, uint8_t abs_mode, uint8_t sync)
{
    uint8_t txbuff[13] = {adr + 1, 0xFD, dir, (uint8_t)(sp >> 8), (uint8_t)sp, acc, (uint8_t)(pos >> 24), (uint8_t)(pos >> 16), (uint8_t)(pos >> 8), (uint8_t)pos, abs_mode, sync, 0x6B};
    motor_tx_send((uint8_t *)txbuff, 13);
    // ESP_LOG_BUFFER_HEX(TAG, txbuff, 13);
}


// 立即停止，正确返回01 FE 02 6B，条件不满足返回01 FE E2 6B，错误命令返回01 00 EE 6B
void ZDTMOT_stop(uint8_t adr, uint8_t sync)
{
    uint8_t txbuff[5] = {adr + 1, 0xFE, 0x98, sync, 0x6B};
    motor_tx_send((uint8_t *)txbuff, 5);
}

// 多机同步运动，发送01 FF 66 6B，正确返回01 FF 02 6B，条件不满足返回01 FF E2 6B，错误命令返回01 00 EE 6B
void ZDTMOT_sync(uint8_t adr)
{
    uint8_t txbuff[4] = {adr, 0xFF, 0x66, 0x6B};
    motor_tx_send((uint8_t *)txbuff, 4);
}

// 触发回零，发送01 9A 00 00 6B，正 确返回01 9A 02 6B，条件不满足返回01 9A E2 6B，错误命令返回01 00 EE 6B
// 00表示触发单圈就近回零，01表示触发单圈方向回零，02表示触发多圈无限位碰撞回零，03 表示触发多圈有限位开关回零
void ZDTMOT_origin(uint8_t adr, uint8_t mode, uint8_t sync)
{
    uint8_t txbuff[5] = {adr + 1, 0x9A, mode, sync, 0x6B};
    ESP_LOG_BUFFER_HEX(TAG_TX, txbuff, 5);
    motor_tx_send((uint8_t *)txbuff, 5);
}

// 强制中断并退出回零操作，发送019C 0x48 6B，正 确返回01 9C 02 6B，条件不满足返回01 9C E2 6B，错误命令返回01 00 EE 6B
void ZDTMOT_forced_int(uint8_t adr)
{
    uint8_t txbuff[4] = {adr + 1, 0x9C, 0x48, 0x6B};
    motor_tx_send((uint8_t *)txbuff, 4);
}

// 读取回零状态标志位, 发送01 3B 6B，正确返回01 3B 回零状态标志字节 6B，错误命令返回 01 00 EE 6B
// 回零状态标志字节，0位：编码器就绪状态标志位，1位：校准表就绪状态标志位，2位：正在回零标志位，3位：回零失败标志位
void ZDTMOT_get_origin_flag(uint8_t adr)
{
    uint8_t txbuff[3] = {adr + 1, 0x3B, 0x6B};
    motor_tx_send((uint8_t *)txbuff, 3);
}

// 触发编码器校准, 发送01 06 45 6B，正确返回01 06 02 6B，条件不满足返回01 06 E2 6B，错误命令返回01 00 EE 6B
void ZDTMOT_calibrate(uint8_t adr)
{
    uint8_t txbuff[4] = {adr + 1, 0x06, 0x45, 0x6B};
    motor_tx_send((uint8_t *)txbuff, 4);
}

// 将当前的位置角度清零，发送01 0A 6D 6B，正确返回01 0A 02 6B，错误命令返回01 00 EE 6B
void ZDTMOT_clear_pos(uint8_t adr)
{
    uint8_t txbuff[4] = {adr + 1, 0x0A, 0x6D, 0x6B};
    motor_tx_send((uint8_t *)txbuff, 4);
}

// 解除堵转保护，发送01 0E 52 6B，正确返回01 0E 02 6B，条件不满足返回01 0E E2 6B，错误命令返回01 00 EE 6B
void ZDTMOT_unlock(uint8_t adr)
{
    uint8_t txbuff[4] = {adr + 1, 0x0E, 0x52, 0x6B};
    motor_tx_send((uint8_t *)txbuff, 4);
}

// 恢复出厂设置，发送01 0F 5F 6B，正确返回01 0F 02 6B，错误命令返回01 00 EE 6B，触发恢复出厂设置后，蓝灯亮起，需要断电重新上电校准编码器。
void ZDTMOT_factory_reset(uint8_t adr)
{
    uint8_t txbuff[4] = {adr + 1, 0x0F, 0x5F, 0x6B};
    motor_tx_send((uint8_t *)txbuff, 4);
}

// 读取电机目标位置，发送01 33 6B，正确返回01 33 01 00 01 00 00 6B，错误命令返回01 00 EE 6B
void ZDTMOT_get_tar_pos(uint8_t adr)
{
    uint8_t txbuff[3] = {adr + 1, 0x33, 0x6B};
    motor_tx_send((uint8_t *)txbuff, 3);
}

// 获取电机当前转速，发送01 35 6B，正确返回01 35 01 05 DC 6B，错误命令返回01 00 EE 6B
void ZDTMOT_get_cur_sp(uint8_t adr)
{
    uint8_t txbuff[3] = {adr + 1, 0x35, 0x6B};
    motor_tx_send((uint8_t *)txbuff, 3);
}

// 获取电机当前位置，发送01 36 6B，正确返回01 36 01 00 01 00 00 6B，错误命令返回01 00 EE 6B
void ZDTMOT_get_cur_pos(uint8_t adr)
{
    uint8_t txbuff[3] = {adr + 1, 0x36, 0x6B};
    motor_tx_send((uint8_t *)txbuff, 3);
}

// 获取电机位置误差，发送01 37 6B，正确返回01 37 01 00 00 00 08 6B，错误命令返回01 00 EE 6B
void ZDTMOT_get_cur_err(uint8_t adr)
{
    uint8_t txbuff[3] = {adr + 1, 0x37, 0x6B};
    motor_tx_send((uint8_t *)txbuff, 3);
}

// 获取电机状态标志位，发送01 3A 6B，正确返回01 3A 电机状态标志字节 6B，错误命令返回01 00 EE 6B
// 电机状态标志字节,0位：电机使能状态标志位，1位：电机到位标志位，2位：电机堵转标志位，3位：电机堵转保护标志
void ZDTMOT_get_status(uint8_t adr)
{
    uint8_t txbuff[3] = {adr + 1, 0x3A, 0x6B};
    motor_tx_send((uint8_t *)txbuff, 3);
}

// 获取系统状态参数，发送01 43 7A 6B，正确返回01 43 系统状态参数 6B，错误命令返回 01 00 EE 6B
// 返回例子01 43 1F(数据字节数) 09(参数个数) 5C 67(总线电压mV) 00 03(总线相电流Ma) 43 EB(校准后编码器值) 01(符号) 00 01 00 00(电机目标位置) 00(符号) 00 00(电机实时转速)
// 01(符号) 00 01 00 00(电机实时位置) 01(符号) 00 00 00 08(电机误差位置) 03(回零状态标志) 03(电机状态标志) 6B
void ZDTMOT_get_sys_status(uint8_t adr)
{
    uint8_t txbuff[4] = {adr + 1, 0x43, 0x7A, 0x6B};
    uart_write_bytes(UART_NUM_2, txbuff, 4);
    // motor_tx_send((uint8_t*) txbuff, 4);
}
// 命令功能：设置单圈回零的零点位置
// 命令格式：地址 + 0x93 + 0x88 + 是否存储标志 + 校验字节
// 命令返回：地址 + 0x93 + 命令状态 + 校验字节
// 命令示例：发送 01 93 88 01 6B，正确返回 01 93 02 6B，错误命令返回 01 00 EE 6B
// 数据解析：可以让电机转到想要的位置，然后发送该命令设置单圈回零的零点位置。
void ZDTMOT_set_origion_pos()
{
    uint8_t txbuff[5] = {0x00, 0x93, 0x88, 0x01, 0x6B};
    motor_tx_send((uint8_t *)txbuff, 5);
}


/**
  * @brief    梯形曲线位置模式
  * @param    addr  ：电机地址
  * @param    dir     ：方向                   ，0为CW，其余值为CCW
  * @param    acc     ：加速加速度(RPM/s)     ，0为CW，其余值为CCW
  * @param    dec     ：减速加速度(RPM/s)     ，0为CW，其余值为CCW
  * @param    velocity：最大速度(RPM)          ，范围0.0 - 4000.0RPM
  * @param    position：位置(°)                ，范围0.0°- (2^32 - 1)°
  * @param    raf     ：相位位置/绝对位置标志  ，0为相对位置，其余值为绝对位置
  * @param    snF     ：多机同步标志           ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Traj_Position_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF)
{
  uint8_t cmd[32] = {0}; uint16_t vel = 0; uint32_t pos = 0;

  // 将速度和位置放大10倍发送过去
  vel = (uint16_t)fabs(velocity * 10.0f); pos = (uint32_t)fabs(position * 10.0f);

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 符号（方向）
  cmd[3]  =  (uint8_t)(acc >> 8);       // 加速加速度(RPM/s)高8位字节
  cmd[4]  =  (uint8_t)(acc >> 0);       // 加速加速度(RPM/s)低8位字节  
  cmd[5]  =  (uint8_t)(dec >> 8);       // 减速加速度(RPM/s)高8位字节
  cmd[6]  =  (uint8_t)(dec >> 0);       // 减速加速度(RPM/s)低8位字节  
  cmd[7]  =  (uint8_t)(vel >> 8);       // 最大速度(RPM)高8位字节
  cmd[8]  =  (uint8_t)(vel >> 0);       // 最大速度(RPM)低8位字节 
  cmd[9]  =  (uint8_t)(pos >> 24);      // 位置(bit24 - bit31)
  cmd[10] =  (uint8_t)(pos >> 16);      // 位置(bit16 - bit23)
  cmd[11] =  (uint8_t)(pos >> 8);       // 位置(bit8  - bit15)
  cmd[12] =  (uint8_t)(pos >> 0);       // 位置(bit0  - bit7 )
  cmd[13] =  raf;                       // 相位位置/绝对位置标志
  cmd[14] =  snF;                       // 多机同步运动标志
  cmd[15] =  0x6B;                      // 校验字节
  
  // 发送命令
  motor_tx_send((uint8_t *)cmd, 32);
}

// /**
//   * @brief    触发回零
//   * @param    addr   ：电机地址
//   * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
//   * @param    snF   ：多机同步标志，false为不启用，true为启用
//   * @retval   地址 + 功能码 + 命令状态 + 校验字节
//   */
// void ZDT_X42_V2_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
// {
//   uint8_t cmd[16] = {0};
  
//   // 装载命令
//   cmd[0] =  addr;                       // 地址
//   cmd[1] =  0x9A;                       // 功能码
//   cmd[2] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
//   cmd[3] =  snF;                        // 多机同步运动标志，false为不启用，true为启用
//   cmd[4] =  0x6B;                       // 校验字节
  
//   // 发送命令
//   motor_tx_send((uint8_t *)cmd, 32);
// }
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
#include <ctype.h>
#include "motor.h"
#include "uart.h"
#include "main.h"

static const char *TAG = "UART";
static const int RX_BUF_SIZE = 1024;
static const int TX_BUF_SIZE = 1024;
#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

// 初始化UART
void uart_init(void)
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
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE, TX_BUF_SIZE, 20, NULL, 0); // 安装驱动，缓冲区大小可以根据需要调整
    // uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // 使用默认的GPIO引脚
    // uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // 使用默认的GPIO引脚
    // uart_driver_install(UART_NUM_0, RX_BUF_SIZE, 0, 0, NULL, 0); // 安装驱动，缓冲区大小可以根据需要调整
}

// 发送数据到串口
void uart_send(const char *data)
{
    uart_write_bytes(UART_NUM_1, data, strlen(data));
    ESP_LOGI(TAG, "向上位机串口发送消息: %s", data);
}

void rx_task(void *arg)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            ESP_LOGI(TAG, "收到串口消息 %d bytes: %s", rxBytes, data);
            // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            // if (rxBytes >= 4 && data[0] == 0x41 && data[1] == 0x42 && isalpha(data[2]) && isalpha(data[3]))
            // { // 前俩是0x41 0x42 确保至少有两个字母且都是字母 如发送0x41 0x42 0x41 0x41 则X=0 Y=0
            //     // 将字母转换为大写，以防串口 数据中的字母是小写形式
            //     char upperLetter1 = toupper(data[2]);
            //     char upperLetter2 = toupper(data[3]);

            //     // 计算字母对应的数值，A=1, B=2, ..., Z=26
            //     int num1 = upperLetter1 - 'A' + 0; // 利用ASCII码差值转换
            //     int num2 = upperLetter2 - 'A' + 0;

            //     if (num1 == 25 && num2 == 25)
            //     { // ZZ
            //     }
            //     else if (num1 == 24 && num2 == 24)
            //     { // YY
            //         // motor_disable();
            //         set_state(1);
            //     }
            //     else
            //     {
            //         ESP_LOGI(TAG, "X:%d,Y:%d", num1, num2); // 上位机下棋点的接收
            //         // grt.play_posx = -((float)num1 * UNIT_DISTANCE_X + ORIGIN_OFFSET_X);
            //         grt.play_posx = ((float)num1 * UNIT_DISTANCE_X + ORIGIN_OFFSET_X);
            //         grt.play_posy = (float)num2 * UNIT_DISTANCE_Y + ORIGIN_OFFSET_Y;
            //         grt.cmd = 1;
            //         // uart_send((const char*)data);
            //     }
            // }
            float x = 0, y = 0; // 棋盘上的某一位置
            if (sscanf((char *)data, "MOV %f %f", &x, &y) == 2)
            {
                if (x < 0 || y < 0)
                {
                    ESP_LOGW(TAG, "收到非法坐标: MOV %f %f，已忽略", x, y);
                }
                else
                {
                    ESP_LOGI(TAG, "收到移动命令: MOV %f %f", x, y);
                    grt.play_posx = ((float)x * UNIT_DISTANCE_X + ORIGIN_OFFSET_X);
                    grt.play_posy = (float)y * UNIT_DISTANCE_Y + ORIGIN_OFFSET_Y;
                    grt.poscmd_flag = true;
                }
            }
            else if (sscanf((char *)data, "MOVP %f %f", &x, &y) == 2)
            {
                grt.pickup_from_box = false;
                if (y < 0)
                {
                    ESP_LOGW(TAG, "收到非法坐标: MOV %f %f，已忽略", x, y);
                }
                else
                {
                    ESP_LOGI(TAG, "收到移动命令: MOV %f %f", x, y);
                    grt.play_posx = ((float)x * UNIT_DISTANCE_X + ORIGIN_OFFSET_X);
                    grt.play_posy = (float)y * UNIT_DISTANCE_Y + ORIGIN_OFFSET_Y;
                    grt.poscmd_flag = true;
                    grt.pickup_flag = true;
                }
            }
            // else if (sscanf((char *)data, "MOVD %f %f", &x, &y) == 2)
            // {
            //     if (y < 0)
            //     {
            //         ESP_LOGW(TAG, "收到非法坐标: MOV %f %f，已忽略", x, y);
            //     }
            //     else
            //     {
            //         ESP_LOGI(TAG, "收到移动命令: MOV %f %f", x, y);
            //         grt.play_posx = ((float)x * UNIT_DISTANCE_X + ORIGIN_OFFSET_X);
            //         grt.play_posy = (float)y * UNIT_DISTANCE_Y + ORIGIN_OFFSET_Y;
            //         grt.poscmd_flag = true;
            //         grt.drop_flag = true;
            //     }
            // }
            // else if (sscanf((char *)data, "MOVD %f %f", &x, &y) == 2)
            else if (sscanf((char *)data, "MOVD %f %f", &x, &y) == 2)//跟上位机反了一下
            {

                if (y < 0 || x < 0)
                {
                    ESP_LOGW(TAG, "收到非法坐标: MOVD %f %f，已忽略", x, y);
                }
                else
                {
                    ESP_LOGI(TAG, "收到放子命令: MOVD %f %f", x, y);
                    // 如果手中已有棋子，直接执行放子
                    if (grt.chessonhand)
                    {
                        grt.play_posx = ((float)x * UNIT_DISTANCE_X + ORIGIN_OFFSET_X);
                        grt.play_posy = (float)y * UNIT_DISTANCE_Y + ORIGIN_OFFSET_Y;
                        grt.poscmd_flag = true;
                        grt.drop_flag = true;
                    }
                    else
                    {
                        // 否则保存为待处理 MOVD，先去棋盒取子，取得后会自动执行
                        grt.pending_movd = true;
                        grt.pending_movd_x = x;
                        grt.pending_movd_y = y;
                        grt.pickup_from_box = true;
                        grt.pickup_flag = true;
                        ESP_LOGI(TAG, "手中无子，已加入待处理 MOVD，并触发 GETCHESS");
                    }
                }
            }
            else if (strcmp((char *)data, "ORG") == 0)
            {
                grt.origin_flag = true;
            }
            else if (strcmp((char *)data, "STORGST") == 0) // 手动设置单圈零点位置开始
            {
                grt.setorigin_flags.start = true;
            }
            else if (strcmp((char *)data, "STORGFIN") == 0) // 手动设置单圈零点位置完成
            {
                grt.setorigin_flags.finish = true;
            }
            else if (strcmp((char *)data, "TESTST") == 0)
            {
                grt.test_flag = 1;
            }
            else if (strcmp((char *)data, "TESTFIN") == 0)
            {
                grt.test_flag = 2;
            }
            else if (strcmp((char *)data, "PICKUP") == 0)
            {
                grt.pickup_from_box = false;
                grt.pickup_flag = true;
            }
            else if (strcmp((char *)data, "DROP") == 0)
            {
                grt.drop_flag = true;
            }
            else if (strcmp((char *)data, "GETCHESS") == 0)
            {
                grt.pickup_from_box = true;
                grt.pickup_flag = true;
            }
        }
    }
    free(data);
}

// Below is unused code from some other project. Safely ignore it.  -- Song

// const static char *UART_THREAD = "UART";

// void K210_uart_packet_parser(uint8_t *p_data, int size)
// {
// }

// // K210 UART begin

// #define K210_UART_NUM UART_NUM_2
// #define K210_TXD_PIN GPIO_NUM_42
// #define K210_RXD_PIN GPIO_NUM_21
// #define K210_UART_BUFFER_SZIE 1024
// #define K210_UART_QUEUE_SIZE 16
// #define K210_UART_PATTERN_LEN 1
// #define K210_UART_PATTERN_TIMEOUT_VALUE 10
// #define K210_UART_PATTERN_POST_IDLE_VALUE 50

// void k210_uart_tx(char *packet_buffer, uint16_t buffered_size)
// {
//     uart_write_bytes(K210_UART_NUM, (const char *)packet_buffer, buffered_size);
// }

// static QueueHandle_t k210_uart_event_queue;

// #define K210_PACKET_END (0xbb)

// static uint8_t packet_buffer[100];
// static uint8_t slip_buffer[100];
// int8_t uart_init_flag = 0;
// void uart_event_task(void *pvParameters)
// {
//     uart_event_t event;
//     size_t buffered_size;
//     uart_config_t uart_update_k210_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

//     if (ESP_OK != uart_driver_delete(K210_UART_NUM))
//     {
//         uart_init_flag = -1;
//     }

//     if (ESP_OK != uart_param_config(K210_UART_NUM, &uart_update_k210_config))
//     {
//         uart_init_flag = -2;
//     }

//     if (ESP_OK != uart_set_pin(K210_UART_NUM, K210_TXD_PIN, K210_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE))
//     {
//         uart_init_flag = -3;
//     }

//     if (ESP_OK != uart_driver_install(K210_UART_NUM, K210_UART_BUFFER_SZIE, K210_UART_BUFFER_SZIE, K210_UART_QUEUE_SIZE, &k210_uart_event_queue, 0))
//     {
//         uart_init_flag = -4;
//     }

//     while (uart_init_flag != 0)
//     {
//         ESP_LOGI(UART_THREAD, "uart intialization false:%d", uart_init_flag);
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
//     uart_enable_pattern_det_baud_intr(K210_UART_NUM, K210_PACKET_END, K210_UART_PATTERN_LEN, K210_UART_PATTERN_TIMEOUT_VALUE, K210_UART_PATTERN_POST_IDLE_VALUE, 0);
//     uart_pattern_queue_reset(K210_UART_NUM, K210_UART_QUEUE_SIZE);
//     ESP_LOGI(UART_THREAD, "uart_init() intialization completed");
//     uart_init_flag = 1;
//     while (true)
//     {
//         if (xQueueReceive(k210_uart_event_queue, &event, portMAX_DELAY))
//         {
//             if (UART_PATTERN_DET == event.type)
//             {
//                 ESP_ERROR_CHECK_WITHOUT_ABORT(uart_get_buffered_data_len(K210_UART_NUM, &buffered_size));
//                 int pos = uart_pattern_pop_pos(K210_UART_NUM);
//                 if (pos == -1)
//                 {
//                     // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
//                     // record the position. We should set a larger queue size.
//                     // As an example, we directly flush the rx buffer here.
//                     uart_flush_input(K210_UART_NUM);
//                     ESP_LOGI(UART_THREAD, "queue size is too small");
//                 }
//                 else
//                 {
//                     if (pos == uart_read_bytes(K210_UART_NUM, slip_buffer, pos, 100 / portTICK_PERIOD_MS))
//                     {
//                         uint8_t SLIP_end[K210_UART_PATTERN_LEN + 1];
//                         if (K210_UART_PATTERN_LEN == uart_read_bytes(K210_UART_NUM, SLIP_end, K210_UART_PATTERN_LEN, 100 / portTICK_PERIOD_MS))
//                         {
//                             if ((buffered_size == pos + 1) && (buffered_size - 1 > 0))
//                             {
//                                 memset(packet_buffer, 0, sizeof(packet_buffer));
//                                 memcpy(packet_buffer, slip_buffer, buffered_size - 1);
//                                 packet_buffer[buffered_size - 1] = SLIP_end[0];
//                                 // uart_write_bytes(K210_UART_NUM, (const char *)packet_buffer, buffered_size);
//                                 // ESP_LOGE(UART_THREAD, "11111");
//                                 K210_uart_packet_parser(packet_buffer, buffered_size);
//                             }
//                         }
//                         else
//                         {
//                             ESP_LOGE(UART_THREAD, "uart_read_bytes() ERROR");
//                         }
//                     }
//                     else
//                     {
//                         ESP_LOGE(UART_THREAD, "uart_read_bytes() ERROR");
//                     }
//                 }
//             }
//         }
//     }
//     vTaskDelete(NULL);
// }

// void report_car_status_to_k210(void *pvParameters)
// { // |  协议头 |载包长度| 舵轮RPM |舵轮角度| 左腿角度|右腿角度|超声波距离  | 协议尾   |
//     // 0xEE,0XAA,  0X0A, | L   H   | 									| 0x00 0xbb|
//     uint8_t report_buf[20] = {0};
//     while (1)
//     {
//         if (uart_init_flag == 1)
//         {
//             k210_uart_tx((char *)report_buf, 15);
//         }
//         vTaskDelay(100 / portTICK_PERIOD_MS);
//     }
// }

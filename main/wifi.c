/* Created 19 Nov 2016 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 * Uses the RMT peripheral on the ESP32 for very accurate timing of
 * signals sent to the WS2812 LEDs.
 *
 * This code is placed in the public domain (or CC0 licensed, at your option).
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include <string.h>
#include <sys/socket.h>
#include "freertos/event_groups.h"
#include "wifi.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "motor.h"
#include "uart.h"
#include "main.h"

static const char* TAG = "WIFI";
/*
电脑端连接此AP，
网络助手选择TCP Client
服务器地址：192.168.4.1
远程主机端口：9527
发送数据，会收到同样的数据
*/

EventGroupHandle_t tcp_event_group; // wifi建立成功信号量
//socket
static int server_socket = 0;					   // 服务器socket
static struct sockaddr_in server_addr;			   // server地址
static struct sockaddr_in client_addr;			   // client地址
static socklen_t socklen = sizeof(client_addr); // 地址长度
static int connect_socket = 0;					   // 连接socket
bool g_rxtx_need_restart = false;				   // 异常后，重新连接标记
uint8_t work_mode = MODE_STA;
// FreeRTOS event group to signal when we are connected to wifi
#define WIFI_CONNECTED_BIT BIT0



void close_socket();									   // 关闭socket
void recv_data(void* pvParameters);						   // 接收数据任务
int get_socket_error_code(int socket);					   // 获取socket错误代码
int show_socket_error_reason(const char* str, int socket); // 获取socket错误原因
int check_working_socket();								   // 检查socket
esp_err_t create_tcp_client();							   // 建立tcp client

// // wifi 事件
// static esp_err_t event_ap_handler(void *ctx, system_event_t *event)
// {
// 	switch (event->event_id)
// 	{
// 	case SYSTEM_EVENT_AP_STACONNECTED: //AP模式-有STA连接成功
// 		// 作为ap，有sta连接
// 		ESP_LOGI(TAG, "station:" MACSTR " join,AID=%d\n", MAC2STR(event->event_info.sta_connected.mac), event->event_info.sta_connected.aid);
// 		xEventGroupSetBits(tcp_event_group, WIFI_CONNECTED_BIT);
// 		break;
// 	case SYSTEM_EVENT_AP_STADISCONNECTED: //AP模式-有STA断线
// 		ESP_LOGI(TAG, "station:" MACSTR "leave,AID=%d\n", MAC2STR(event->event_info.sta_disconnected.mac), event->event_info.sta_disconnected.aid);
// 		//重新建立server
// 		g_rxtx_need_restart = true;
// 		xEventGroupClearBits(tcp_event_group, WIFI_CONNECTED_BIT);
// 		break;
// 	default:
// 		break;
// 	}
// 	return ESP_OK;
// }

// wifi 事件
static void event_sta_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(tcp_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        xEventGroupSetBits(tcp_event_group, WIFI_CONNECTED_BIT);
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(tcp_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_send_data(uint8_t* buf, int32_t msg_length) {
    send(connect_socket, buf, msg_length, 0); //接收数据回发
}

// 接收数据任务
void recv_data(void* pvParameters) {
    int len = 0;
    uint8_t databuff[1024];
    // char cmd_buf[128]={0};
    if (work_mode == MODE_STA) {
        // send(connect_socket, json_buf, msg_length + 12, 0); //接收数据回发
    }
    while (1) {
        memset(databuff, 0x00, sizeof(databuff));				   //清空缓存
        len = recv(connect_socket, databuff, sizeof(databuff), 0); //读取接收数据
        g_rxtx_need_restart = false;
        if (len > 0) {
            ESP_LOGI(TAG, "server rx:%s", databuff);
            if (len >= 2 && isalpha(databuff[0]) && isalpha(databuff[1])) { // 确保至少有两个字母且都是字母
                // 将字母转换为大写，以防串口数据中的字母是小写形式
                char upperLetter1 = toupper(databuff[0]);
                char upperLetter2 = toupper(databuff[1]);

                // 计算字母对应的数值，A=1, B=2, ..., Z=26
                int num1 = upperLetter1 - 'A' + 0; // 利用ASCII码差值转换
                int num2 = upperLetter2 - 'A' + 0;
                if (num1 == 25 && num2 == 25) {

                }
                else if (num1 == 24 && num2 == 24) {
                    // motor_disable();
                    set_state(1);
                }
                else {
                    ESP_LOGI(TAG, "X:%d,Y:%d", num1, num2);
                    play_chess_pos[0] = (float)num1 * UNIT_DISTANCE_X;
                    play_chess_pos[1] = (float)num2 * UNIT_DISTANCE_Y;

                }
            }
        }
        else {
            show_socket_error_reason("recv_data", connect_socket); //打印错误信息
            g_rxtx_need_restart = true;							   //服务器故障，标记重连
            vTaskDelete(NULL);
        }
    }
    close_socket();
    g_rxtx_need_restart = true; //标记重连
    vTaskDelete(NULL);
}
// 建立tcp server
esp_err_t create_tcp_server(bool isCreatServer) {
    //首次建立server
    if (isCreatServer) {
        ESP_LOGI(TAG, "server socket....,port=%d", TCP_CLIENT_PORT);
        server_socket = socket(AF_INET, SOCK_STREAM, 0); //新建socket
        if (server_socket < 0) {
            show_socket_error_reason("create_server", server_socket);
            close(server_socket); //新建失败后，关闭新建的socket，等待下次新建
            return ESP_FAIL;
        }
        //配置新建server socket参数
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(TCP_CLIENT_PORT);
        server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        //bind:地址的绑定
        if (bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            show_socket_error_reason("bind_server", server_socket);
            close(server_socket); //bind失败后，关闭新建的socket，等待下次新建
            return ESP_FAIL;
        }
    }
    //listen，下次时，直接监听
    if (listen(server_socket, 5) < 0) {
        show_socket_error_reason("listen_server", server_socket);
        close(server_socket); //listen失败后，关闭新建的socket，等待下次新建
        return ESP_FAIL;
    }
    //accept，搜寻全连接队列
    connect_socket = accept(server_socket, (struct sockaddr*)&client_addr, &socklen);
    if (connect_socket < 0) {
        show_socket_error_reason("accept_server", connect_socket);
        close(server_socket); //accept失败后，关闭新建的socket，等待下次新建
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "tcp connection established!");
    return ESP_OK;
}

// 建立tcp client
uint8_t tcp_state;
esp_err_t create_tcp_client() {
    ESP_LOGI(TAG, "will connect gateway ssid : %s port:%d", TCP_SERVER_ADRESS, TCP_CLIENT_PORT);
    //新建socket
    connect_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (connect_socket < 0) {
        show_socket_error_reason("create client", connect_socket); //打印报错信息
        close(connect_socket);									   //新建失败后，关闭新建的socket，等待下次新建
        return ESP_FAIL;
    }
    //配置连接服务器信息
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(TCP_SERVER_PORT);
    server_addr.sin_addr.s_addr = inet_addr(TCP_SERVER_ADRESS);
    ESP_LOGI(TAG, "connectting server...");
    //连接服务器
    if (connect(connect_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        show_socket_error_reason("client connect", connect_socket); //打印报错信息
        ESP_LOGE(TAG, "connect failed!");
        tcp_state = 0;
        //连接失败后，关闭之前新建的socket，等待下次新建
        close(connect_socket);
        return ESP_FAIL;
    }
    tcp_state = 1;
    ESP_LOGI(TAG, "connect success!");
    return ESP_OK;
}

// WIFI作为AP的初始化
// void wifi_init_softap()
// {
// 	tcp_event_group = xEventGroupCreate();
// 	tcpip_adapter_init();
// 	ESP_ERROR_CHECK(esp_event_loop_init(event_ap_handler, NULL));
// 	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
// 	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
// 	wifi_config_t wifi_config = {
// 		.ap = {
// 			.ssid = SOFT_AP_SSID,
// 			.password = SOFT_AP_PAS,
// 			.ssid_len = 0,
// 			.max_connection = SOFT_AP_MAX_CONNECT,
// 			.authmode = WIFI_AUTH_WPA_WPA2_PSK,
// 		},
// 	};
// 	if (strlen(SOFT_AP_PAS) == 0)
// 	{
// 		wifi_config.ap.authmode = WIFI_AUTH_OPEN;
// 	}
// 	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
// 	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
// 	ESP_ERROR_CHECK(esp_wifi_start());
// 	ESP_LOGI(TAG, "SoftAP set finish,SSID:%s password:%s \n", wifi_config.ap.ssid, wifi_config.ap.password);
// }

// WIFI作为STA的初始化

esp_event_handler_instance_t instance_any_id;
esp_event_handler_instance_t instance_got_ip;
void wifi_init_sta() {
    tcp_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &event_sta_handler,
        NULL,
        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &event_sta_handler,
        NULL,
        &instance_got_ip));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PAS},
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s \n", WIFI_SSID, WIFI_PAS);
}

// 获取socket错误代码
int get_socket_error_code(int socket) {
    int result;
    u32_t optlen = sizeof(int);
    int err = getsockopt(socket, SOL_SOCKET, SO_ERROR, &result, &optlen);
    if (err == -1) {
        //WSAGetLastError();
        ESP_LOGE(TAG, "socket error code:%d", err);
        ESP_LOGE(TAG, "socket error code:%s", strerror(err));
        return -1;
    }
    return result;
}

// 获取socket错误原因
int show_socket_error_reason(const char* str, int socket) {
    int err = get_socket_error_code(socket);
    if (err != 0) {
        ESP_LOGW(TAG, "%s socket error reason %d %s", str, err, strerror(err));
    }
    return err;
}
// 检查socket
int check_working_socket() {
    int ret;
    if (work_mode == MODE_AP) {
        ESP_LOGD(TAG, "check server_socket");
        ret = get_socket_error_code(server_socket);
        if (ret != 0) {
            ESP_LOGW(TAG, "server socket error %d %s", ret, strerror(ret));
        }
        if (ret == ECONNRESET) {
            return ret;
        }
    }
    if (work_mode == MODE_STA) {
        ESP_LOGD(TAG, "check connect_socket");
        ret = get_socket_error_code(connect_socket);
        if (ret != 0) {
            ESP_LOGW(TAG, "connect socket error %d %s", ret, strerror(ret));
        }
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
}
// 关闭socket
void close_socket() {
    close(connect_socket);
    close(server_socket);
}
// 建立TCP连接并从TCP接收数据
void tcp_great_tcp_server(void* pvParameters) {
    while (1) {
        g_rxtx_need_restart = false;
        // 等待WIFI连接信号量，死等
        xEventGroupWaitBits(tcp_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
        ESP_LOGI(TAG, "start tcp connected");
        TaskHandle_t tx_rx_task = NULL;
        vTaskDelay(3000 / portTICK_PERIOD_MS); // 延时3S准备建立server
        ESP_LOGI(TAG, "create tcp server");
        int socket_ret = create_tcp_server(true); // 建立server
        if (socket_ret == ESP_FAIL) { // 建立失败
            ESP_LOGI(TAG, "create tcp socket error,stop...");
            continue;
        }
        else { // 建立成功
            ESP_LOGI(TAG, "create tcp socket succeed...");
            // 建立tcp接收数据任务
            if (pdPASS != xTaskCreate(&recv_data, "recv_data", 4096 * 2, NULL, 4, &tx_rx_task)) {
                ESP_LOGI(TAG, "Recv task create fail!");
            }
            else {
                ESP_LOGI(TAG, "Recv task create succeed!");
            }
        }
        while (1) {
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            if (g_rxtx_need_restart) { // 重新建立server，流程和上面一样
                ESP_LOGI(TAG, "tcp server error,some client leave,restart...");
                // 建立server
                if (ESP_FAIL != create_tcp_server(false)) {
                    if (pdPASS != xTaskCreate(&recv_data, "recv_data", 4096 * 2, NULL, 4, &tx_rx_task)) {
                        ESP_LOGE(TAG, "tcp server Recv task create fail!");
                    }
                    else {
                        ESP_LOGI(TAG, "tcp server Recv task create succeed!");
                        g_rxtx_need_restart = false; //重新建立完成，清除标记
                    }
                }
            }
        }
    }
    vTaskDelete(NULL);
}

// 任务：建立TCP连接并从TCP接收数据
void tcp_create_tcp_client(void* pvParameters) {
    while (1) {
        g_rxtx_need_restart = false;
        //等待WIFI连接信号量，死等
        xEventGroupWaitBits(tcp_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
        ESP_LOGI(TAG, "start tcp connected");
        TaskHandle_t tx_rx_task = NULL;
        //延时3S准备建立clien
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "create tcp Client");
        //建立client
        int socket_ret = create_tcp_client();
        if (socket_ret == ESP_FAIL) {
            ESP_LOGI(TAG, "create tcp socket error,stop...");
            continue;
        }
        else {
            ESP_LOGI(TAG, "create tcp socket succeed...");
            //建立tcp接收数据任务
            if (pdPASS != xTaskCreate(&recv_data, "recv_data", 4096 * 2, NULL, 4, &tx_rx_task)) {
                ESP_LOGI(TAG, "Recv task create fail!");
            }
            else {
                ESP_LOGI(TAG, "Recv task create succeed!");
            }
        }
        while (1) {
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            //重新建立client，流程和上面一样
            if (g_rxtx_need_restart) {
                vTaskDelay(3000 / portTICK_PERIOD_MS);
                ESP_LOGI(TAG, "reStart create tcp client...");
                //建立client
                int socket_ret = create_tcp_client();
                if (socket_ret == ESP_FAIL) {
                    ESP_LOGE(TAG, "reStart create tcp socket error,stop...");
                    continue;
                }
                else {
                    ESP_LOGI(TAG, "reStart create tcp socket succeed...");
                    //重新建立完成，清除标记
                    g_rxtx_need_restart = false;
                    //建立tcp接收数据任务
                    if (pdPASS != xTaskCreate(&recv_data, "recv_data", 4096 * 2, NULL, 4, &tx_rx_task)) {
                        ESP_LOGE(TAG, "reStart Recv task create fail!");
                    }
                    else {
                        ESP_LOGI(TAG, "reStart Recv task create succeed!");
                    }
                }
            }
        }
    }
    vTaskDelete(NULL);
}

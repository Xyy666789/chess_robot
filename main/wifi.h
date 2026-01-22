/* Created 19 Nov 2016 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 * This is a driver for the WS2812 RGB LEDs using the RMT peripheral on the ESP32.
 *
 * This code is placed in the public domain (or CC0 licensed, at your option).
 */

#ifndef WIFI_H
#define WIFI_H

#include <stdint.h>

#define MODE_AP 1
#define MODE_STA 0

 //AP热点模式的配置信息
#define SOFT_AP_SSID "DC_AP"	  		// AP 网络名称
#define SOFT_AP_PAS "1234567890" 		// AP 密码
#define SOFT_AP_MAX_CONNECT 2	  		// 最多的连接点
#define TCP_CLIENT_PORT 9527	  		// 监听客户端端口

#define WIFI_SSID "ChinaNet-2.4G-5986"			  	// WIFI 网络名称
#define WIFI_PAS "88888888"	  	// WIFI 密码
#define TCP_SERVER_PORT 9001		  	// 电脑端的服务器监听端口
#define TCP_SERVER_ADRESS "192.168.124.20" 	// 作为client，要连接TCP服务器地址

void wifi_init_softap();
void wifi_init_sta();
void tcp_great_tcp_server(void* pvParameters);
void tcp_create_tcp_client(void* pvParameters);
void wifi_send_data(uint8_t* buf, int32_t msg_length);

#endif 

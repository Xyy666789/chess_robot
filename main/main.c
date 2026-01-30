#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_flash.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "sigan.h"

#include "main.h"
#include "motor.h"
#include "uart.h"
#include "wifi.h"
#include "gpio.h"
#include "soft_i2c.h"
#include "Motor_28BYJ.h"

static const char *TAG = "MAIN";

go_robot_t grt;

void Sigan_Control_Task(void *arg)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 每50ms检测一次标志位
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        // 检测上升标志位
        if (grt.sigan_up_flag) 
        {
            ESP_LOGI("SIGAN_TASK", "执行上升动作: %.2f mm", grt.sigan_val);
            sigan_move_up(grt.sigan_val); 
            grt.sigan_up_flag = false;    
        }
        // 检测下降标志位
        if (grt.sigan_down_flag) 
        {
            ESP_LOGI("SIGAN_TASK", "执行下降动作: %.2f mm", grt.sigan_val);
            sigan_move_down(grt.sigan_val); 
            grt.sigan_down_flag = false;    
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void app_main(void)
{
    ESP_LOGI("APP_MAIN", "APP Start......");
    /*初始化flash*/
    ESP_LOGI("APP_MAIN", "初始化FLASH......");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // /*初始化WIFI*/
    // ESP_LOGI("APP_MAIN", "初始化FLASH......");
    // wifi_init_sta();
    // xTaskCreate(&tcp_create_tcp_client, "great tcp client", 4096, NULL, 5, NULL);
    // work_mode = MODE_STA;

    /*配置while循环运行时间间隔*/
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 4000 / portTICK_PERIOD_MS; // 打印间隔时间，ms
    xLastWakeTime = xTaskGetTickCount();                     // 初始化上一次唤醒时间

    grt.air_duty = 100; // 默认气泵占空比，80%
    grt.move_order = 0; // 默认先后手·
    grt.pending_movd = false;
    grt.pending_movd_x = 0;
    grt.pending_movd_y = 0;
    grt.pickup_retry = 0;
    grt.pickup_retry_limit = 3; // 可根据需要调整最大重试次数
    grt.arm_config.L1 = 243.61; // 机械臂第一段臂长，单位mm 243.61
    grt.arm_config.L2 = 180.0;  // 机械臂第二段臂长，单位mm
    grt.arm_config.theta1_max = 140;
    grt.arm_config.theta1_min = -45; // 到时候设计的-30度为归零原点 所以所有的都要加30度 也就是电机1坐标顺时针旋转30度
    grt.arm_config.theta2_max = 160; // 到时候设计的20度为归零原点 所以所有的都要减20度
    grt.arm_config.theta2_min = 10;  // 关节角度限制
    // esp_log_level_set("*", ESP_LOG_NONE); //关闭日志打印
    sigan_init();                                                              // 确保丝杆初始化
    xTaskCreate(Sigan_Control_Task, "Sigan_Control_Task", 4096, NULL, 5, NULL);// 启动丝杆控制任务
    gpio_init();                                                               // 初始化输入输出
    uart_init();                                                               // 初始化串口
    motor_uart_init();                                                         // 初始化电机控制串口
    ZMOT_init();                                                               // 初始化电机控制参数
    motor_gpio_init();                                                         // 初始化28BYJ电机GPIO
    xTaskCreate(Arm_motion_test, "Arm_motion_test", 1024 * 20, NULL, 5, NULL); // 创建臂部运动测试线程
    xTaskCreate(Detection_task, "Detection_task", 1024 * 6, NULL, 5, NULL);    // 创建输入检测线程
    // xTaskCreate(play_chess_task, "play_chess_task", 1024 * 20, NULL, 5, NULL); //创建下棋线程
    xTaskCreatePinnedToCore(rx_task, "rx_task", 1024 * 6, NULL, 2, NULL, 1); // 创建上位机串口接收线程，使用核心1
    xTaskCreate(motor_rx_task, "motor_rx_task", 1024 * 6, NULL, 3, NULL);    // 创建电机串口接收线程
    // xTaskCreate(soft_i2c_task1, "soft_i2c_task1", 1024 * 6, NULL, 10, NULL);//创建i2c线程，读取APDS9960传感器数据
    // xTaskCreatePinnedToCore(soft_i2c_task2, "soft_i2c_task2", 1024 * 6, NULL, 11, NULL, 1);//创建i2c线程，读取APDS9960传感器数据，使用核心1
    xTaskCreate(zmot_cmd, "zmot_cmd", 1024 * 20, NULL, 21, NULL); // 创建电机命令控制线程
    ESP_LOGI(TAG, "上电时间: %lld us", esp_timer_get_time());

    while (1)
    {
        for (int id = 0; id < MOT_COT; id++)
        {
            // ESP_LOGI("电机", "%d:复位状态%d,当前位置%.1f,编码器%d,目标位置%.1f,位置误差%d,O状态%d%d%d%d,M状态%d%d%d%d", id, motor[id].ostate.origined, motor[id].cur_pos, motor[id].cur_pul, motor[id].tar_pos, motor[id].err_pul, motor[id].ostate.encoder_ready, motor[id].ostate.calib_ready, motor[id].ostate.origining, motor[id].ostate.origin_failed, motor[id].mstate.enable, motor[id].mstate.reached, motor[id].mstate.stalled, motor[id].mstate.stallProt);
        }
        // ESP_LOGI(TAG, "运行时间: %lld us,step_p: %d, step_o: %d, tl= %d | grid= %d, tr = %d | grid= %d, proximity L= %d|R= %d, vacuo_state = %d", esp_timer_get_time(), grt.step_p, grt.step_o, grt.step_t[0], grt.grid[0], grt.step_t[1], grt.grid[1], grt.proximity[0], grt.proximity[1], grt.vacuo_state);
        // ESP_LOGI(TAG, "event: %d, %d, %d, %d, %d, %d, %d, %d, %d", grt.event[0], grt.event[1], grt.event[2], grt.event[3], grt.event[4], grt.event[5], grt.event[6], grt.event[7], grt.event[8]);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void Detection_task(void *arg)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10U;   // 线程运行间隔时间
    xLastWakeTime = xTaskGetTickCount(); // 初始化上一次唤醒时间
    while (1)
    {
        Input_detection();
        // 在每次检测后让出 CPU，按固定频率运行输入检测
        vTaskDelay(pdMS_TO_TICKS(1000));
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xFrequency));
    }
}
void Arm_motion_test(void *arg)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10U;   // 线程运行间隔时间
    xLastWakeTime = xTaskGetTickCount(); // 初始化上一次唤醒时间

    // for (int i = 0; i < MOT_COT; i++)
    // { // 状态清零
    //     motor[i].ostate.origined = 0;
    //     motor[i].ostate.origin_failed = 0;
    // }
    int init_count = 0;
    static int first_box_drop_done = 0; // 0: 未首次下放，1: 已首次下放
    // 让两个电机归零
    ESP_LOGE("让两个电机归零", "start");
    back_origin(MOT_X);
    back_origin(MOT_Y);
    cmd_sync = 1;

    while ((check_origin()) == 0)
    {
        ESP_LOGE("检查回零", "waiting...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    MotorClockwise_Angle(40);
    MotorStop();
    ESP_LOGE("归零完成", "finish");
    
    while (1)
    {

        Input_detection();
        // 如果存在待处理的 MOVD 并且手中已有棋子，则执行该 MOVD（移动并放子）
        if (grt.pending_movd && grt.chessonhand)
        {
            grt.pending_movd = false;
            grt.play_posx = ((float)grt.pending_movd_x * UNIT_DISTANCE_X + ORIGIN_OFFSET_X);
            grt.play_posy = (float)grt.pending_movd_y * UNIT_DISTANCE_Y + ORIGIN_OFFSET_Y;
            grt.poscmd_flag = true;
            grt.drop_flag = true;
            ESP_LOGI(TAG, "执行待处理 MOVD: %.2f, %.2f", grt.play_posx, grt.play_posy);
        }
        if (grt.poscmd_flag == true)
        {
            ESP_LOGE("测试", "ERROR");
            grt.poscmd_flag = false; // 清除命令标志
            Point2D target = {grt.play_posx, grt.play_posy};
            bool success = inverse_kinematics(&grt.arm_config, target, &grt.joint_angles, true);
            if (success)
            {
                ESP_LOGE("可达", "X坐标: %.2f, Y坐标: %.2f", grt.play_posx, grt.play_posy);
                // 机械臂移动到目标位置
                // 这里需要根据你的电机控制方式设置目标角度
                motor[MOT_X].tar_pos = grt.joint_angles.theta1;
                if (motor[MOT_X].tar_pos < 0)
                {
                    motor[MOT_X].pos_dir = 1;
                    motor[MOT_X].tar_pos = -motor[MOT_X].tar_pos;
                }

                else
                    motor[MOT_X].pos_dir = 0;
                motor[MOT_X].tar_pul = motor[MOT_X].tar_pos * SPR_XY / 360;
                motor[MOT_Y].tar_pos = -grt.joint_angles.theta2;
                motor[MOT_Y].tar_pul = abs(motor[MOT_Y].tar_pos) * SPR_XY / 360; // 第二个电机加负号 反向转动
                ESP_LOGE("已到达指定位置", "MOT_X目标角度: %.2f, MOT_Y目标角度: %.2f", motor[MOT_X].tar_pos, motor[MOT_Y].tar_pos);
                if (motor[MOT_X].tar_pul < 0 || motor[MOT_Y].tar_pul < 0)
                {
                    ESP_LOGE("脉冲数错误", "ERROR");
                    vTaskDelay(pdMS_TO_TICKS(200));
                    break;
                }

                // 先运动 Y 轴，等 Y 到位后再运动 X 轴
                cmd_pos[MOT_X] = 1;
                cmd_pos[MOT_Y] = 1;
                cmd_sync = 1;
                // 等待到达
                while (!(check_reach(MOT_X) && check_reach(MOT_Y)))
                {
                    ESP_LOGE("等待电机", "waiting...");
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
                if (grt.pickup_from_box == true && grt.pickup_from_box == true)
                    grt.getchessarived = true;

                ESP_LOGE("已到达指定位置", "MOT_X目标角度: %.2f, MOT_Y目标角度: %.2f", motor[MOT_X].tar_pos, motor[MOT_Y].tar_pos);
                uart_send("POSarrived\n");
            }
            else
            {
                ESP_LOGE("目标不可达", "X坐标: %.2f, Y坐标: %.2f", grt.play_posx, grt.play_posy);
                uart_send("POSillegal\n");
            }
        }
        if (grt.origin_flag == true)
        {
            ESP_LOGE("检测到复位命令", "开始复位");
            back_origin(MOT_X);
            back_origin(MOT_Y);
            cmd_sync = 1;
            while ((check_origin()) == 0)
            {
                ESP_LOGE("等待归零", "waiting...");
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            ESP_LOGE("归零完成", "finish");
            uart_send("ORGfinish\n"); //
            grt.origin_flag = false;
        }
        if (grt.setorigin_flags.start == true)
        {
            cmd_set_origin = 1; // 设置电机0单圈零点位置
            ESP_LOGE("单圈零点位置设置完成", "finish");
            uart_send("STORGfinish\n");
            grt.setorigin_flags.start = false;
        }

        if (grt.test_flag == 1)
        {
        }

        if (grt.test_flag == 2)
        {

            MotorCounterClockwise_Angle(60); // 逆时针转动3圈
            grt.test_flag = 1;
            // MotorStop();
        }

        if (grt.drop_flag == true)
        {
            ESP_LOGE("棋子下放", "开始");
            if (init_count == 0)
            {
                motor_28BYJ_Zaxis(2); // 下放
                vTaskDelay(pdMS_TO_TICKS(400));
                airpump_disable();
                init_count = 1;
            }

            if (grt.vacuo_state == 1)
            {
                // 放置完成，上拉并清理状态
                motor_28BYJ_Zaxis(1); // 上拉
                grt.drop_flag = false;
                init_count = 0;
                // 放下棋子后手中无子
                grt.chessonhand = false;
                uart_send("DROPSUCCESS\n");
                // 放子完成后，自动执行 GETCHESS（从棋盒取下一颗棋子）
                grt.pickup_from_box = true;
                grt.pickup_flag = true;
                ESP_LOGI(TAG, "放子完成，自动触发 GETCHESS（从棋盒取子）");
            }
        }

        Input_detection();
        if (grt.pickup_flag == true)
        {
            if (grt.pickup_from_box == true)
            {
                // 棋盒取子搜索
                static int pickup_attempt = 0;
                static float base_x = 0, base_y = 0;
                static const float step = 20.0f;
                static const int max_radius = 60;
                static const int max_attempts = 16;
                static float spiral_points_x[16];
                static float spiral_points_y[16];
                static int spiral_count = 0; // 实际生成的有效点数
                if (init_count == 0)
                {
                    ESP_LOGE("棋盒取子", "开始");
                    init_count = 1;
                    // 随机初始点
                    base_x = 170.0f + (rand() % 31);
                    base_y = 100.0f + (rand() % 161);
                    grt.play_posx = base_x;
                    grt.play_posy = base_y;
                    grt.poscmd_flag = true;
                    // 生成螺旋点（优先按螺旋策略），若不足则用随机采样填充，保证有足够的候选点
                    int idx = 0;
                    // 先加入初始点
                    if (base_x >= 170.0f && base_x <= 200.0f && base_y >= 100.0f && base_y <= 260.0f && idx < max_attempts)
                    {
                        spiral_points_x[idx] = base_x;
                        spiral_points_y[idx] = base_y;
                        idx++;
                    }
                    // 螺旋点从半径step开始
                    for (int r = (int)step; r <= max_radius && idx < max_attempts; r += (int)step)
                    {
                        for (int angle = 0; angle < 360 && idx < max_attempts; angle += 40)
                        {
                            float rad = angle * 3.1415926f / 180.0f;
                            float x = base_x + r * cosf(rad);
                            float y = base_y + r * sinf(rad);
                            if (x >= 170.0f && x <= 200.0f && y >= 100.0f && y <= 260.0f)
                            {
                                spiral_points_x[idx] = x;
                                spiral_points_y[idx] = y;
                                idx++;
                            }
                        }
                    }

                    // 如果螺旋策略未生成足够点，则通过随机采样在允许区域内填充，避免重复（基于最小距离）
                    int fallback_tries = 0;
                    const int max_fallback_tries = 1000;
                    const float min_dist_sq = 25.0f; // 最小平方距离，约5mm
                    while (idx < max_attempts && fallback_tries < max_fallback_tries)
                    {
                        float rx = 170.0f + (rand() % 31);  // [170,200]
                        float ry = 100.0f + (rand() % 161); // [100,260]
                        bool unique = true;
                        for (int j = 0; j < idx; j++)
                        {
                            float dx = spiral_points_x[j] - rx;
                            float dy = spiral_points_y[j] - ry;
                            if ((dx * dx + dy * dy) < min_dist_sq)
                            {
                                unique = false;
                                break;
                            }
                        }
                        if (unique)
                        {
                            spiral_points_x[idx] = rx;
                            spiral_points_y[idx] = ry;
                            idx++;
                        }
                        fallback_tries++;
                    }

                    // 若仍不足，则放宽去重限制，直接填充剩余点
                    if (idx < max_attempts)
                    {
                        for (int j = idx; j < max_attempts; j++)
                        {
                            float rx = 170.0f + (rand() % 31);
                            float ry = 100.0f + (rand() % 161);
                            spiral_points_x[j] = rx;
                            spiral_points_y[j] = ry;
                        }
                        idx = max_attempts;
                    }

                    // 记录实际生成的点数，后续只使用 [0, spiral_count) 范围内的点
                    spiral_count = idx;
                }

                if (grt.getchessarived == true)
                {
                    airpump_enable();
                    if (first_box_drop_done == 0)
                    {
                        motor_28BYJ_Zaxis(2); // 第一次下放使用动作2
                        first_box_drop_done = 1;
                    }
                    else
                    {
                        motor_28BYJ_Zaxis(4); // 其他次数使用动作4
                    }
                }
                int confirmed = 0;
                TickType_t pick_start = xTaskGetTickCount();
                while (1)
                {
                    Input_detection();
                    if (grt.vacuo_state == 0) // 防抖
                    {

                        motor_28BYJ_Zaxis(1);
                        vTaskDelay(pdMS_TO_TICKS(200));
                        Input_detection();
                        if (grt.vacuo_state != 0)
                        {
                            motor_28BYJ_Zaxis(2);
                            continue;
                        }
                        grt.pickup_flag = false;
                        init_count = 0;
                        pickup_attempt = 0;
                        grt.getchessarived = false;
                        grt.pickup_from_box = false;
                        grt.chessonhand = true;  // 手中有棋子标志
                        grt.pickup_retry = 0;    // 成功后重置重试计数
                        first_box_drop_done = 0; // 每次取子初始化搜索点时重置首次下放标志
                        ESP_LOGE("棋子已经抓起", "finish");
                        break;
                    }

                    if ((xTaskGetTickCount() - pick_start) >= pdMS_TO_TICKS(900))
                    {
                        ESP_LOGW("棋盒取子", "2s未检测到真空，抬起并搜索下一个点");
                        motor_28BYJ_Zaxis(3);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        pickup_attempt++;
                        if (pickup_attempt < spiral_count)
                        {
                            grt.play_posx = spiral_points_x[pickup_attempt];
                            grt.play_posy = spiral_points_y[pickup_attempt];
                            grt.poscmd_flag = true;
                            grt.getchessarived = false;
                            ESP_LOGW("棋盒取子", "尝试第%d个点: X=%.1f, Y=%.1f", pickup_attempt + 1, grt.play_posx, grt.play_posy);
                        }
                        else
                        {
                            ESP_LOGE("棋盒取子", "所有搜索点尝试失败");
                            // 若未超过重试限制，则进行重试：重新触发取子流程
                            if (grt.pickup_retry < grt.pickup_retry_limit)
                            {
                                grt.pickup_retry++;
                                ESP_LOGW("棋盒取子", "第 %d 次重试取子", grt.pickup_retry);
                                // 重置状态以便下次重新初始化搜索点与位置
                                pickup_attempt = 0;
                                init_count = 0;
                                grt.getchessarived = false;
                                grt.pickup_flag = true;
                                grt.pickup_from_box = true;
                                // 随机化下一次起始点
                                // 注意：下一次循环会重新生成 spiral_points
                            }
                            else
                            {
                                ESP_LOGE("棋盒取子", "所有重试均失败，宣布取子失败");
                                uart_send("PICKUP_FAIL\n");
                                grt.pickup_from_box = false;
                                grt.pickup_flag = false;
                                pickup_attempt = 0;
                                grt.getchessarived = false;
                                init_count = 0;
                                grt.pickup_retry = 0; // 重置重试计数
                            }
                        }
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }
            else
            {
                // 棋盘取子
                static int pickup_attempt = 0;
                static float base_x = 0, base_y = 0;
                static const float offset = 5.0f;
                if (init_count == 0)
                {
                    ESP_LOGE("棋子抓取", "开始");
                    airpump_enable();
                    init_count = 1;
                    base_x = grt.play_posx;
                    base_y = grt.play_posy;
                }
                TickType_t pick_start = xTaskGetTickCount();
                motor_28BYJ_Zaxis(2);
                vTaskDelay(pdMS_TO_TICKS(100));
                while (1)
                {
                    Input_detection();
                    if (grt.vacuo_state == 0)
                    {
                        motor_28BYJ_Zaxis(1);
                        grt.pickup_flag = false;
                        init_count = 0;
                        pickup_attempt = 0;
                        ESP_LOGE("棋子已经抓起", "finish");
                        break;
                    }
                    if ((xTaskGetTickCount() - pick_start) >= pdMS_TO_TICKS(2000))
                    {
                        ESP_LOGW("棋子抓取", "2.5s未检测到真空，抬起并重试");
                        motor_28BYJ_Zaxis(1);
                        // init_count = 0;
                        vTaskDelay(pdMS_TO_TICKS(50));
                        pickup_attempt++;
                        if (pickup_attempt == 1)
                        {
                            grt.play_posx = base_x + offset;
                            grt.play_posy = base_y;
                            grt.poscmd_flag = true;
                        }
                        else if (pickup_attempt == 2)
                        {
                            grt.play_posx = base_x - offset;
                            grt.play_posy = base_y;
                            grt.poscmd_flag = true;
                        }
                        else if (pickup_attempt == 3)
                        {
                            grt.play_posx = base_x;
                            grt.play_posy = base_y + offset;
                            grt.poscmd_flag = true;
                        }
                        else if (pickup_attempt == 4)
                        {
                            grt.play_posx = base_x;
                            grt.play_posy = base_y - offset;
                            grt.poscmd_flag = true;
                        }
                        else if (pickup_attempt > 4)
                        {
                            ESP_LOGE("棋子抓取", "连续4次尝试失败，宣布取子失败");
                            uart_send("PICKUP_FAIL\n");
                            grt.pickup_flag = false;
                            pickup_attempt = 0;
                            init_count = 0;
                            airpump_disable();
                        }
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xFrequency));
    }
}

void play_chess_task(void *arg)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10U;   // 线程运行间隔时间
    xLastWakeTime = xTaskGetTickCount(); // 初始化上一次唤醒时间
    grt.chess_x[1] = CHESS_BLACK_X;
    grt.chess_y[1] = CHESS_BLACK_Y;
    grt.chess_x[0] = CHESS_WHITE_X;
    grt.chess_y[0] = CHESS_WHITE_Y; // 初始棋子坐标

    while (1)
    {
        bt_detection();                 // 按钮触发检测
        origin_task();                  // 复位
        turntable_task(grt.move_order); // 转盘任务处理
        grt_event_dispose();            // 按钮事件处理

        if (grt.boxerr == 1)
        { // 棋盒无子
            if (grt.step_p == 5)
            { // grt.step_p == 5 等待上位机命令阶段
                grt.last_step_p = grt.step_p;
                grt.event[8] = 1;
                grt.boxerr = 0;
                grt.step_p = 14; // case 14 无子处理相关流程
            }
        }

        // 下棋主流程
        switch (grt.step_p)
        {
        case 0: // 等待开始对局
            if (grt.event[0] == 1)
            { // 如果开始对局
                if (check_origin())
                { // 检测是否回零
                    grt.event[0] = 0;
                    // grt.step_p = 1
                }
                else
                {
                    grt.event[0] = 0;
                    grt.event[4] = 1;
                    grt.step_p = 1;
                }
            }
            break;
        case 1:
            if (check_origin())
            {                         // 等待复位完成
                uart_send("START\n"); // 开始对局
                ESP_LOGI(TAG, "检测到复位完成，开始对局");
                grt.step_p = 2;
            }
            break;
        case 2: // 移动至棋子坐标起点
            vTaskDelay(pdMS_TO_TICKS(50));
            motor[MOT_X].tar_pos = grt.chess_x[grt.move_order];
            motor[MOT_X].tar_pul = motor[MOT_X].tar_pos * SPR_XY / DPR_X;
            motor[MOT_Y].tar_pos = grt.chess_y[grt.move_order];
            motor[MOT_Y].tar_pul = motor[MOT_Y].tar_pos * SPR_XY / DPR_Y; // 改成二自由度运动学解算

            cmd_pos[MOT_X] = 1; // 位置命令 就是说向电机发送位置 让电机运动
            cmd_pos[MOT_Y] = 1;
            cmd_sync = 1; // 多电机同步运动
            grt.step_p++;
            break;
        case 3:
            if (grt.event[8] == 1)
            { // 这里还不太明白
                grt.event[8] = 0;
                grt.step_p++;
            }
            break;
        case 4: // 等待到达棋子坐标起点
            if (check_reach(MOT_X) && check_reach(MOT_Y))
            {                      // 检查是否到达目标位置
                uart_send("OK\n"); // 准备完成后发这一条,临时用OK，后期READY
                grt.step_p++;
                grt.start = 1;
            }
            break;
        case 5: // 等待上位机命令
            if (grt.cmd)
            { // grt.cmd==1 收到上位机棋子位置命令
                grt.cmd = 0;
                grt.step_p++;
            }
            break;
        case 6: // 吸盘吸,吸盘降
            if (grt.grid[grt.move_order])
            {
                airpump_enable();
                motz_op(1); // 吸盘降
                grt.step_p++;
            }
            break;
        case 7: // 等待到达，延时吸或着加入吸盘检测，吸盘升
            if (grt.vacuo_state == 0)
            {
                motz_op(2); // 吸盘升
                grt.step_p++;
            }
            break;
        case 8: // 等待到达，移动到落子点

            grt.grid[grt.move_order] = false;
            motor[MOT_X].tar_pos = grt.play_posx;
            motor[MOT_X].tar_pul = motor[MOT_X].tar_pos * SPR_XY / DPR_X;
            motor[MOT_Y].tar_pos = grt.play_posy;
            motor[MOT_Y].tar_pul = motor[MOT_Y].tar_pos * SPR_XY / DPR_Y;
            cmd_pos[MOT_X] = 1;
            cmd_pos[MOT_Y] = 1;
            cmd_sync = 1;
            grt.step_p++;
            grt.lost_cnt = 0;
            // ESP_LOGW("对弈", "关气泵，step_p = %d", grt.step_p);
            // uint32_t duty = 0;  // 13 位分辨率，50% 占空比
            // set_pwm_duty(duty);

            break;
        case 9: // 等待到达，吸盘降
            if (check_reach(MOT_X) && check_reach(MOT_Y))
            {
                motz_op(1);
                grt.step_p++;
            }
            else
            {
                if (grt.vacuo_state == 1)
                {
                    grt.lost_cnt++;
                    if (grt.lost_cnt == 80)
                    { // 不是真空状态就是棋子丢失了
                        ESP_LOGW("对弈", "棋子丢失");
                        uart_send("ERR02\n"); // 中途棋子掉发这一条
                    }
                }
            }
            break;
        case 10: // 等到到达，吸盘松，延时或加入吸盘检测
            airpump_disable();
            if (grt.vacuo_state == 1)
            {
                grt.step_p++;
            }
            break;
        case 11: // 吸盘升
            motz_op(2);
            grt.step_p++;
            break;
        case 12: // 等待到达，跳转到第二步
            grt.step_p = 2;
            grt.event[8] = 1; // 等待坐标指令
            break;
        case 13: // grt_event_dispose() 中检测到结束对局按钮 grt.step_p = 13
            if (grt.event[7] == 1)
            {
                grt.event[7] = 0;
                grt.start = 0;
                uart_send("CONFIRM\n"); // 确认退出
                ESP_LOGW("对弈", "确认退出");
                grt.step_p = 0;
            }
            else if (grt.event[0] == 1)
            {
                grt.event[0] = 0;
                uart_send("DENY\n"); // 取消
                ESP_LOGW("对弈", "取消退出");
                grt.step_p = grt.last_step_p;
            }
            break;
        case 14: // 无子处理，移动到补子点
            vTaskDelay(pdMS_TO_TICKS(50));
            motor[MOT_X].tar_pos = 217.5f;
            motor[MOT_X].tar_pul = motor[MOT_X].tar_pos * SPR_XY / DPR_X;
            motor[MOT_Y].tar_pos = 80.0f;
            motor[MOT_Y].tar_pul = motor[MOT_Y].tar_pos * SPR_XY / DPR_Y;

            cmd_pos[MOT_X] = 1;
            cmd_pos[MOT_Y] = 1;
            cmd_sync = 1;
            grt.step_p++;
            break;
        case 15: // 等待到达补子点，跳转到第二步
            if (check_reach(MOT_X) && check_reach(MOT_Y))
            {
                grt.step_p++;
            }
            break;
        case 16: // 等待棋盒检测到棋子
            if (grt.grid[grt.move_order] == true)
            {
                grt.step_p = 2;
                ESP_LOGW("对弈", "棋盒检测到棋子");
            }
            break;
        default:
            break;
        }
        // ESP_LOGI(TAG, " grt.step_p: %d, time_count = %ld", grt.step_p, time_count);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xFrequency));
    }
}

void grt_event_dispose(void)
{ // 按钮事件处理
    if (grt.event[1] == 1)
    { // grt.event[1]按钮事件 0开始对局 1结束对局 2先后手选择 3音量调节 4复位 5提示 6悔棋 7确认/下一步 8等待坐标指令
        grt.event[1] = 0;
        if (grt.step_p != 0)
        {
            grt.last_step_p = grt.step_p;
            grt.step_p = 13;
            uart_send("END\n");
            ESP_LOGW("对弈", "结束对局");
        }
        else
        {
            ESP_LOGW("对弈", "不在对局中，操作无效");
        }
    }
    if (grt.event[2] == 1)
    {
        grt.event[2] = 0;
        if (grt.step_p >= 1 && grt.step_p <= 3)
        {
            grt.move_order = !grt.move_order;
            if (grt.move_order)
            {
                uart_send("WHITE\n"); //
                ESP_LOGW("对弈", "用户后手");
            }
            else
            {
                uart_send("BLACK\n"); //
                ESP_LOGW("对弈", "用户先手");
            }
        }
        else
        {
            ESP_LOGW("对弈", "对局没有开始，切换先后手不生效");
        }
        if (grt.step_p == 3)
        {
            grt.step_p = 2;
        }
    }
    if (grt.event[3] == 1)
    {
        grt.event[3] = 0;
        uart_send("VOLUME\n");
        ESP_LOGW("对弈", "音量调节");
    }
    if (grt.event[4] == 1)
    {
        grt.event[4] = 0;
        grt.step_o = 1;
        ESP_LOGW("复位", "开始");
    }
    if (grt.event[5] == 1)
    {
        grt.event[5] = 0;
        if (grt.step_p == 5)
        {
            uart_send("HINT\n");
            ESP_LOGW("对弈", "提示");
        }
        else
        {
            ESP_LOGW("对弈", "此阶段不支持提示");
        }
    }
    if (grt.event[6] == 1)
    {
        grt.event[6] = 0;
        if (grt.step_p >= 5)
        {
            uart_send("UNDO\n");
            ESP_LOGW("对弈", "悔棋");
        }
        else
        {
            ESP_LOGW("对弈", "此阶段不支持悔棋");
        }
    }
    if (grt.event[7] == 1)
    {
        if (grt.step_p >= 2 && grt.step_p <= 12 && grt.start == 1)
        {
            grt.event[7] = 0;
            uart_send("READY\n");
            ESP_LOGW("对弈", "确认");
        }
        if (grt.step_p >= 1 && grt.step_p <= 4 && grt.start == 0)
        {
            grt.event[7] = 0;
            grt.event[8] = 1;
            uart_send("CONFIRM\n"); // 确认
            ESP_LOGW("对弈", "对弈开始");
        }
    }
}

// if (grt.bt_state[3]) { //指定点位下棋测试
//     grt.bt_state[3] = 0;
//     grt.play_posx = (float)1 * UNIT_DISTANCE_X + ORIGIN_OFFSET_X;
//     grt.play_posy = (float)1 * UNIT_DISTANCE_Y + ORIGIN_OFFSET_YL;
//     grt.cmd = 1;
// }
// if (grt.bt_state[2]) { //气泵开关测试
//     grt.bt_state[2] = 0;
// if (air_sta) {
//     air_sta = 0;
//     airpump_disable();
// }
//     else {
//         air_sta = 1;
//         airpump_enable();
//     }
// }
// if (grt.bt_state[1]) { //气泵电压调节测试
//     grt.bt_state[1] = 0;
//     grt.air_duty += 5;
//     if(grt.air_duty > 100){
//         grt.air_duty = 20;
//     }
//     ESP_LOGI("PWM", "PWM duty占空比: %d", grt.air_duty);
//     airpump_enable();
//     air_sta = 1;
// }

void origin_task(void)
{ // 电机原点校准
    switch (grt.step_o)
    {
    case 0:
        // grt.step_o++;
        break;
    case 1:
        for (int i = 0; i < MOT_COT; i++)
        { // 状态清零
            motor[i].ostate.origined = 0;
            motor[i].ostate.origin_failed = 0;
            motor[i].ostate.calib_ready = 0;
            motor[i].ostate.encoder_ready = 0;
            motor[i].ostate.orif = 0;
            motor[i].ostate.origining = 0;
            motor[i].mstate.reached = 0;
            motor[i].mstate.stalled = 0;
            motor[i].mstate.stallProt = 0;
        }
        motor[MOT_TL].confirm = 0;
        motor[MOT_TR].confirm = 0;
        if (grt.step_p >= 4)
        {
            grt.last_step_p = 2;
            grt.event[8] = 1;
        }
        else
        {
            grt.last_step_p = grt.step_p;
        }
        // if (grt.step_p != 1) {
        //     grt.step_p = 0;
        // }

        grt.step_t[0] = 0;
        grt.step_t[1] = 0;
        grt.step_o = 1;
        airpump_disable();
        ESP_LOGE("运行方向", "复位前清零");
        vTaskDelay(pdMS_TO_TICKS(100));
        grt.step_o++;
        break;
    case 2:
        uart_send("RST\n");
        motz_op(2);

        motor[MOT_TL].tar_pos = motor[MOT_TL].cur_pos + 10;
        if (motor[MOT_TL].tar_pos < 0)
        {
            motor[MOT_TL].pos_dir = 1;
            ESP_LOGE("运行方向", "负方向%f", motor[MOT_TL].tar_pos);
        }
        else
        {
            motor[MOT_TL].pos_dir = 0;
            ESP_LOGE("运行方向", "正方向%f", motor[MOT_TL].tar_pos);
        }
        motor[MOT_TL].tar_pul = abs(motor[MOT_TL].tar_pos) * SPR_T / DPR_T;
        motor[MOT_TL].pos_sync = false;
        cmd_pos[MOT_TL] = 1;

        motor[MOT_TR].tar_pos = motor[MOT_TR].cur_pos + 10;
        if (motor[MOT_TR].tar_pos < 0)
        {
            motor[MOT_TR].pos_dir = 1;
            ESP_LOGE("运行方向", "负方向%f", motor[MOT_TR].tar_pos);
        }
        else
        {
            motor[MOT_TR].pos_dir = 0;
            ESP_LOGE("运行方向", "正方向%f", motor[MOT_TR].tar_pos);
        }
        motor[MOT_TR].tar_pul = abs(motor[MOT_TR].tar_pos) * SPR_T / DPR_T;

        motor[MOT_TR].pos_sync = false;
        cmd_pos[MOT_TR] = 1;
        vTaskDelay(pdMS_TO_TICKS(1000));
        grt.step_o++;
        break;
    case 3:
        back_origin(MOT_X);
        back_origin(MOT_Y);
        back_origin(MOT_TL);
        back_origin(MOT_TR);
        ESP_LOGI("复位", "等待完成");
        cmd_sync = 1;
        vTaskDelay(pdMS_TO_TICKS(2000));
        grt.step_o++;
        break;
    case 4:
        if (check_origin())
        { //
            ESP_LOGI("复位", "完成");
            uart_send("RSTOK\n"); // 复位完成后发这一条
            // vTaskDelay(pdMS_TO_TICKS(1000));
            grt.step_o = 0;
            grt.step_p = grt.last_step_p;
        }
        break;
    default:
        break;
    }
}

uint16_t overtimet = 0;
void turntable_task(uint8_t num)
{
    // 转盘控制
    switch (grt.step_t[num])
    {
    case 0:
        if (grt.step_p == 3)
        { // 只有当主流程 grt.step_p == 3 时才会进入该步骤
            if (num == 0)
            {
                motor[MOT_TL + num].tar_pos = ORIGIN_OFFSET_TL;
            }
            else
            {
                motor[MOT_TL + num].tar_pos = ORIGIN_OFFSET_TR;
            }
            grt.grid[num] = false; // 清空棋盒有子标志
            grt.step_t[num]++;
        }
        break;
    case 1: // 检测棋子状态，控制转盘旋转
        if (grt.grid[num] == true && grt.proximity[num] < DISTANCE_VAL)
        { // 有子且距离传感器检测到距离小于设定值
            overtimet++;
            if (overtimet >= 10)
            {
                grt.grid[num] = false;
                overtimet = 0;
            }
        }
        else
        {
            overtimet = 0;
        }
        if (grt.grid[num] == false)
        { // 无子，转动转盘取子
            vTaskDelay(pdMS_TO_TICKS(100));
            motor[MOT_TL + num].last_tar_pos = motor[MOT_TL + num].tar_pos;
            motor[MOT_TL + num].tar_pos += ANGLE_T;
            if (motor[MOT_TL + num].tar_pos < 0)
            {
                motor[MOT_TL + num].pos_dir = 1;
                ESP_LOGE("运行方向", "负方向%f", motor[MOT_TL + num].tar_pos);
            }
            else
            {
                motor[MOT_TL + num].pos_dir = 0;
                ESP_LOGE("运行方向", "正方向%f", motor[MOT_TL + num].tar_pos);
            }
            motor[MOT_TL + num].tar_pul = abs(motor[MOT_TL + num].tar_pos) * SPR_T / DPR_T;
            motor[MOT_TL + num].pos_sync = false;
            cmd_pos[MOT_TL + num] = 1;
            vTaskDelay(pdMS_TO_TICKS(300));
            grt.step_t[num]++;
            overtimet = 0;
        }
        break;
    case 2: // 等待转盘到达目标位置
        overtimet++;
        if (motor[MOT_TL + num].mstate.reached && motor[MOT_TL + num].confirm)
        {
            motor[MOT_TL + num].confirm = 0;
            motor[MOT_TL + num].locked = 0;
            grt.step_t[num]++;
            overtimet = 0;
        }
        if (overtimet >= 600 || (motor[MOT_TL + num].err_pul > 500 && motor[MOT_TL + num].cur_sp < 6))
        {
            grt.grid[num] = false;
            if (motor[MOT_TL + num].locked == 1)
            {
                motor[MOT_TL + num].tar_pos = motor[MOT_TL + num].tar_pos;
            }
            else
            {
                motor[MOT_TL + num].locked = 1;
                motor[MOT_TL + num].tar_pos = motor[MOT_TL + num].tar_pos - ANGLE_T;
            }
            ESP_LOGE("转盘", "位置错误, 疑似卡住, step %d, overtimet = %d, err_pul = %d", grt.step_t[num], overtimet, motor[MOT_TL + num].err_pul);
            grt.step_t[num] = 1;
            overtimet = 0;
        }
        break;
    case 3: // 检测棋子状态
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (grt.proximity[num] > DISTANCE_VAL)
        {
            grt.step_t[num] = 1;
            grt.grid[num] = true;
            ESP_LOGI("棋盒", "有子");
        }
        else
        {
            grt.step_t[num] = 0;
            grt.box[num]++;
            if (grt.box[num] >= 10)
            {
                grt.box[num] = 0;
                grt.boxerr = 1;
                ESP_LOGE("棋盒", "无子");
                uart_send("ERR01\n"); // 棋盒无子发送ERR01
            }
        }
        grt.step_t[num] = 1;
        break;
    default:
        break;
    }
}

// 设置状态
void set_state(uint8_t state)
{
    grt.step_p = state;
}

// 速度匹配相关
//  motor[MOT_X].time = abs(motor[MOT_X].tar_pul - motor[MOT_X].cur_pul) / motor[MOT_X].tar_sp;
//  motor[MOT_Y].time = abs(motor[MOT_Y].tar_pul - motor[MOT_Y].cur_pul) / motor[MOT_Y].tar_sp;
//  motor[MOT_YR].time = abs(motor[MOT_YR].tar_pul - motor[MOT_YR].cur_pul) / motor[MOT_YR].tar_sp;

// motor[MOT_X].tar_sp = 500;
// motor[MOT_Y].tar_sp = 500;
// motor[MOT_YR].tar_sp = 500;
// float time_max = 0;
// if (motor[MOT_X].time >= motor[MOT_Y].time) {
//     time_max = motor[MOT_X].time;
//     motor[MOT_Y].tar_sp = abs(motor[MOT_Y].tar_pul - motor[MOT_Y].cur_pul) / time_max;
//     motor[MOT_YR].tar_sp = motor[MOT_Y].tar_sp;
//     ESP_LOGI("速度匹配", "X慢");
// }
// else {
//     time_max = motor[MOT_Y].time;
//     motor[MOT_X].tar_sp = abs(motor[MOT_X].tar_pul - motor[MOT_X].cur_pul) / time_max;
//     ESP_LOGI("速度匹配", "Y慢");
// }

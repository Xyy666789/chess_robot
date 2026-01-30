#pragma once

#include "driver/gptimer.h"
#include "arm_kinematics.h"

/*IO分配
上位机通讯TX            GPIO_NUM_4
上位机通讯RX            GPIO_NUM_5

APDS-9960_SCL          GPIO_NUM_6
APDS-9960_SDA          GPIO_NUM_7

APDS-9960_SCL          GPIO_NUM_15
APDS-9960_SDA          GPIO_NUM_16

电机控制TX              GPIO_NUM_17
电机控制RX              GPIO_NUM_18

IO扩展板SCL             GPIO_NUM_6
IO扩展板SDA             GPIO_NUM_7
    {
        P0：0~7     按钮0~8
        P1：0
    }
气泵控制               GPIO_NUM_46
电磁阀控制             GPIO_NUM_9

*/

#define AIP_PUMP_PIN    GPIO_NUM_46 //气泵、电磁阀控制引脚
#define MOTZ_FWD_PIN   GPIO_NUM_9 //Z轴电机正转引脚Forward
#define MOTZ_REV_PIN  GPIO_NUM_10 //Z轴电机反转引脚Reverse
#define VACUO_PIN       GPIO_NUM_20//真空检测引脚
#define DISTANCE_VAL    100   //白子有子判定值
#define B_DISTANCE_VAL    245   //黑子有子判定值

/* PWM 配置 */ //控制气泵 占空比
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT  // 分辨率为 13 位
#define LEDC_FREQUENCY 20              // PWM 频率为 5000 Hz
#define LEDC_GPIO GPIO_NUM_46          // 连接 PWM 输出的 GPIO 引脚
/* PWM 配置 */

typedef struct {
    bool start;
    bool finish;
} SetOrgFlags;

typedef struct {
    uint8_t step_p;// play chess step// 下棋主流程步骤
    uint8_t last_step_p;// 记录最近play chess step
    uint8_t step_o;// 复位step 
    uint8_t step_t[2];// 左右转盘step，0左白，1右黑
    bool grid[2];// 格子内棋子状态 0:无 1:有
    uint8_t box[2];//左棋盒没有检测到棋子次数累计，6次后提醒放子
    uint8_t proximity[2];// 接近传感器数据 有无子检测
    float chess_x[2];// 棋子坐标` 
    float chess_y[2];// 棋子坐标 棋子存放的那个坐标
    uint8_t move_order;// 先后手选择，上电默认为0先手
    uint8_t cmd;// 上位机命令到达标志
    uint8_t bt_state[8];// 按钮状态 0无触发 1短按 2长按
    gpio_num_t bt_pin[8];// 按钮引脚
    uint8_t air_duty;// 气泵占空比 0~100
    bool vacuo_state;// 真空状态
    uint8_t mot_wait;// 电机等待时间
    float play_posx;// 下棋坐标 要下的那个坐标
    float play_posy;// 下棋坐标
    uint16_t lost_cnt;// 丢失检测计数 
    uint8_t start;// 开始标志
    uint8_t boxerr;//棋盒无子
    bool event[10];//按钮事件 0开始对局 1结束对局 2先后手选择 3音量调节 4复位 5提示 6悔棋 7确认/下一步 8等待坐标指令
    bool test;// 测试标志
    ArmConfig arm_config;// 机械臂参数
    JointAngles joint_angles;// 机械臂关节角度结果
    bool origin_flag;// 回零命令到达标志
    bool poscmd_flag;// 位置命令到达标志
    SetOrgFlags setorigin_flags; // 手动设置单圈零点标志
    int test_flag;// 测试功能标志位
    bool drop_flag;
    bool pickup_flag;
    bool pickup_from_box;
    bool pending_movd; // 上位机下发的 MOVD 待处理标志
    float pending_movd_x; // 待处理 MOVD 的目标网格坐标 x
    float pending_movd_y; // 待处理 MOVD 的目标网格坐标 y
    uint8_t pickup_retry; // 当前从棋盒重试计数
    uint8_t pickup_retry_limit; // 最大重试次数
    bool getchessarived;//抓取时是否到达指定位置
    bool chessonhand;//手中有棋子标志
    bool sigan_up_flag;    // 丝杆上升标志位
    bool sigan_down_flag;  // 丝杆下降标志位
    float sigan_val;       // 丝杆移动的距离数值
} go_robot_t;

extern go_robot_t grt;

typedef struct {
    uint64_t event_count;
} example_queue_element_t;

void play_chess_task(void* arg);
void set_state(uint8_t state);
void origin_task(void);
void turntable_task(uint8_t num);
void grt_event_dispose(void);
void Arm_motion_test(void* arg);
void Detection_task(void* arg);

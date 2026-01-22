#pragma once

// 电机控制引脚定义
#define MOTOR_TX_PIN GPIO_NUM_17
#define MOTOR_RX_PIN GPIO_NUM_18

#define MOT_COT 4 //步进电机数量
#define SPR_XY 3200 //XY电机每圈脉冲数//完整STEPS_PER_REVOLUTION
// #define DPR_X 40 //X电机每圈移动距离，毫米//完整DISTANCE_PER_REVOLUTION
// #define DPR_Y 20 //Y电机每圈移动距离，毫米//完整DISTANCE_PER_REVOLUTION
#define DPR_X 360 //X电机每圈移动距离，毫米//完整DISTANCE_PER_REVOLUTION
#define DPR_Y 360 //Y电机每圈移动距离，毫米//完整DISTANCE_PER_REVOLUTION
#define SPR_T 3200*2 //转盘电机每圈脉冲数
#define DPR_T 360 //转盘电机每圈角度，°
#define ANGLE_T 36 //转盘单格转动角度

// #define UNIT_DISTANCE_X 22.6f//棋格间距，单位：毫米
// #define UNIT_DISTANCE_Y 23.8f//棋格间距，单位：毫米
// #define ORIGIN_OFFSET_X -425.7f//X原点偏移，单位：毫米//
// #define ORIGIN_OFFSET_X -120.0f//X原点偏移，单位：毫米//
// #define ORIGIN_OFFSET_Y 45.8f//Y原点偏移，单位：毫米
#define UNIT_DISTANCE_X 20.1f//棋格间距，单位：毫米
#define UNIT_DISTANCE_Y 21.07f//棋格间距，单位：毫米
#define ORIGIN_OFFSET_X -120.0f//X原点偏移，单位：毫米//横13格点  
#define ORIGIN_OFFSET_Y 120.0f//Y原点偏移，单位：毫米 纵13格点
#define ORIGIN_OFFSET_TL 0//左转盘原点偏移，脉冲数
#define ORIGIN_OFFSET_TR -1//右转盘原点偏移，脉冲数

#define CHESS_WHITE_X 351.6f //白棋X坐标位置，单位：毫米//91.5
#define CHESS_WHITE_Y 1.5f //白棋Y坐标位置，单位：毫米
#define CHESS_BLACK_X 90.8f //黑棋X坐标位置，单位：毫米
#define CHESS_BLACK_Y 1.5f //黑棋Y坐标位置，单位：毫米

#define MOT_X 0
#define MOT_Y 1
#define MOT_TL 2
#define MOT_TR 3

typedef enum {
    MODE_STOP,
    MODE_ORIGIN,
    MODE_SPEED,
    MODE_POSITION
} motor_mode_t;

// 电机状态结构体
typedef struct {
    bool enable;            // 电机使能状态
    bool reached;           // 是否到达目标位置状态
    bool stalled;           // 电机堵转状态
    bool stallProt;         // 电机堵转保护状态
} moter_state_t;

// 回零状态结构体
typedef struct {
    bool encoder_ready;     // 编码器就绪状态
    bool calib_ready;       // 校准表就绪状态
    bool origining;         // 回零中状态
    bool origin_failed;     // 回零失败状态
    bool origined;          // 已回零状态
    bool orif;              // 回零标志位
} origin_state_t;

// 电机结构体
typedef struct {
    moter_state_t mstate;   //电机状态
    origin_state_t ostate;  //回零状态
    uint8_t omode;          //回零模式
    uint8_t ack;            //确认标志
    uint8_t confirm;        //确认标志
    uint8_t mode;           //电机模式
    int8_t back_dir;        //电机反馈位置运动方向纠正
    int8_t cur_dir;         //电机当前运动方向
    float cur_pos;          //电机当前位置
    float tar_pos;          //电机目标位置
    float last_tar_pos;     //电机目标角度
    float err_pos;          //电机当前误差
    uint err_pul;           //电机当前误差脉冲数
    uint cur_pul;           //电机当前脉冲数
    uint tar_pul;           //电机目标脉冲数
    int cur_sp;             //电机当前速度
    int tar_sp;             //电机目标速度
    bool pos_sync;          //电机位置同步标志
    bool pos_dir;           //电机位置运动方向
    uint8_t abs_mode;       //电机位置运行模式，0相对 or 1绝对
    uint8_t acc;            //电机加速度
    uint16_t time;          //电机运行时间
    uint32_t last_pul;      //电机上次脉冲数
    uint8_t locked;         //电机堵转标志
} motor_t;

typedef struct {
    int x;
    int y;
} coordinate_t;


extern motor_t motor[MOT_COT];
extern float play_chess_pos[2];

void motor_uart_init(void);
void back_origin(uint8_t m_num);
bool check_reach(uint8_t m_num);
void ZMOT_init(void);
void zmot_cmd(void *arg);
void set_pos(uint8_t m_num, float pos, uint16_t sp);
void set_angle(uint8_t m_num, float angle, uint16_t sp);

void ZDTMOT_enable(uint8_t adr, uint8_t state);
void ZDTMOT_sp(uint8_t adr, uint8_t dir, int16_t sp, uint8_t acc, uint8_t sync);
void ZDTMOT_pos(uint8_t adr, uint8_t dir, int16_t sp, uint8_t acc, uint32_t pos, uint8_t abs_mode, uint8_t sync);
void ZDTMOT_stop(uint8_t adr, uint8_t sync);
void ZDTMOT_sync(uint8_t adr);
void ZDTMOT_origin(uint8_t adr, uint8_t mode, uint8_t sync);
void ZDTMOT_forced_int (uint8_t adr);
void ZDTMOT_get_origin_flag(uint8_t adr);
void ZDTMOT_calibrate(uint8_t adr);
void ZDTMOT_clear_pos(uint8_t adr);
void ZDTMOT_unlock(uint8_t adr);
void ZDTMOT_factory_reset(uint8_t adr);
void ZDTMOT_get_tar_pos(uint8_t adr);
void ZDTMOT_get_cur_sp(uint8_t adr);  
void ZDTMOT_get_cur_pos(uint8_t adr);
void ZDTMOT_get_cur_err(uint8_t adr);
void ZDTMOT_get_status(uint8_t adr);
void ZDTMOT_get_sys_status(uint8_t adr);
void ZDTMOT_set_origion_pos(void);
void ZDT_X42_V2_Traj_Position_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF);

int check_origin(void);
void grt_event_dispose(void);
void motor_rx_task(void* arg);
void motor_tx_send(uint8_t* data, uint8_t len);

void sync_while(void);

typedef struct {
    void (*enable)(uint8_t adr, uint8_t state);
    void (*set_sp)(uint8_t adr, uint8_t dir, int16_t sp, uint8_t acc, uint8_t sync);
    void (*set_pos)(uint8_t adr, uint8_t dir, int16_t sp, uint8_t acc, uint32_t pos, uint8_t abs_mode, uint8_t sync);
    // void (*set_pos)(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF);
    void (*stop)(uint8_t adr, uint8_t sync);
    void (*sync)(uint8_t adr);
    void (*origin)(uint8_t adr, uint8_t mode, uint8_t sync);
    void (*forced_int)(uint8_t adr);
    void (*get_origin_flag)(uint8_t adr);
    void (*calibrate)(uint8_t adr);
    void (*clear_pos)(uint8_t adr);
    void (*unlock)(uint8_t adr);
    void (*factory_reset)(uint8_t adr);
    void (*get_tar_pos)(uint8_t adr);
    void (*get_cur_sp)(uint8_t adr);
    void (*get_cur_pos)(uint8_t adr);
    void (*get_cur_err)(uint8_t adr);
    void (*get_status)(uint8_t adr);
    void (*get_sys_status)(uint8_t adr);
    void (*set_origion_pos)();
} ZDTMOT_t;

extern ZDTMOT_t ZMOT;
extern uint8_t cmd_ori[MOT_COT];
extern uint8_t cmd_disable[MOT_COT];
extern uint8_t cmd_enab[MOT_COT];
extern uint8_t cmd_pos[MOT_COT];
extern uint8_t cmd_sta[MOT_COT];
extern uint8_t cmd_sync;
extern uint8_t cmd_set_origin;
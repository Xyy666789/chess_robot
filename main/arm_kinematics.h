#pragma once

#include <math.h>
#include <stdbool.h>

// 定义PI和角度弧度转换
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)
#define RAD_TO_DEG(x) ((x) * 180.0 / M_PI)
#define GEAR_RATIO 0.5// 齿轮比 小于1，若无齿轮则为1.0

// 二维点结构
typedef struct {
    double x;
    double y;
} Point2D;

// 关节角度结构
typedef struct {
    double theta1;  // 关节1角度(度)
    double theta2;  // 关节2角度(度)
} JointAngles;

// 机械臂配置结构
typedef struct {
    double L1;      // 第一段臂长
    double L2;      // 第二段臂长
    double theta1_min, theta1_max;  // 关节1角度限制
    double theta2_min, theta2_max;  // 关节2角度限制
} ArmConfig;

// 函数声明
Point2D forward_kinematics(const ArmConfig* arm, double theta1, double theta2);
bool inverse_kinematics(const ArmConfig* arm, Point2D target, 
                       JointAngles* result, bool elbow_up);
bool is_point_reachable(const ArmConfig* arm, Point2D point);
// void plan_linear_trajectory(Point2D start, Point2D end, 
//                            Point2D* trajectory, int steps);


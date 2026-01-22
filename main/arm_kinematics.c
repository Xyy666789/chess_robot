// arm_kinematics.c
#include "arm_kinematics.h"
#include <stdio.h>

// 正运动学：关节角度 -> 末端位置
Point2D forward_kinematics(const ArmConfig* arm, double theta1, double theta2) {
    Point2D result;
    result.x = arm->L1 * cos(theta1) + arm->L2 * cos(theta1 + theta2);
    result.y = arm->L1 * sin(theta1) + arm->L2 * sin(theta1 + theta2);
    return result;
}

// 检查点是否可达
bool is_point_reachable(const ArmConfig* arm, Point2D point) {
    double distance_sq = point.x * point.x + point.y * point.y;
    double max_reach = arm->L1 + arm->L2;
    double min_reach = fabs(arm->L1 - arm->L2);
    
    return (distance_sq <= max_reach * max_reach) && 
           (distance_sq >= min_reach * min_reach);
}

// 逆运动学：末端位置 -> 关节角度
bool inverse_kinematics(const ArmConfig* arm, Point2D target, 
                       JointAngles* result, bool elbow_up) {
    
    // 输入的角度限制是度，需转换为弧度
    double theta1_min_rad = arm->theta1_min * M_PI / 180.0;
    double theta1_max_rad = arm->theta1_max * M_PI / 180.0;
    double theta2_min_rad = arm->theta2_min * M_PI / 180.0;
    double theta2_max_rad = arm->theta2_max * M_PI / 180.0;

    // 检查是否可达
    if (!is_point_reachable(arm, target)) {
        return false;
    }

    double x = target.x;
    double y = target.y;

    // 计算第二个关节角度
    double D = (x*x + y*y - arm->L1*arm->L1 - arm->L2*arm->L2) / (2 * arm->L1 * arm->L2);

    // 数值稳定性检查
    if (D > 1.0) D = 1.0;
    if (D < -1.0) D = -1.0;

    double theta2 = acos(D);
    if (!elbow_up) {
        theta2 = -theta2;
    }

    // 计算第一个关节角度
    double beta = atan2(y, x);
    double alpha = atan2(arm->L2 * sin(theta2), arm->L1 + arm->L2 * cos(theta2));
    double theta1 = beta - alpha;
    printf("[ERROR] 发生错误！\n");
    // 角度限制检查（全部用弧度）
    if (theta1 < theta1_min_rad || theta1 > theta1_max_rad ||
        theta2 < theta2_min_rad || theta2 > theta2_max_rad) {
        return false;
    }

    result->theta1 = theta1 * 180.0 / M_PI;
    result->theta2 = theta2 * 180.0 / M_PI / GEAR_RATIO; // 输出角度并考虑齿轮比
    return true;
}

// // 直线轨迹规划
// void plan_linear_trajectory(Point2D start, Point2D end, 
//                            Point2D* trajectory, int steps) {
//     for (int i = 0; i <= steps; i++) {
//         double t = (double)i / steps;
//         trajectory[i].x = start.x + t * (end.x - start.x);
//         trajectory[i].y = start.y + t * (end.y - start.y);
//     }
// }
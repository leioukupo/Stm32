#ifndef __MECANUM_H
#define __MECANUM_H

#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////
// 麦克纳姆轮运动学逆解 (Phase 3, 计划 §2/§3)
//
// 轮索引约定: 0=FL(前左) 1=FR(前右) 2=RL(后左) 3=RR(后右)
//
// 逆解公式 (vx 前为正, vy 左为正, wz 逆时针为正; k = 轴距/2 + 轮距/2):
//   FL = vx - vy - k*wz
//   FR = vx + vy + k*wz
//   RL = vx + vy - k*wz
//   RR = vx - vy + k*wz
// 验证: (100,0,0) 前进四轮同向; (0,100,0) 横移 FL/RR 反转;
//       (0,0,100) 原地左转 左轮反转右轮正转.
//////////////////////////////////////////////////////////////////////////////////

#define MECANUM_WHEELS      4

typedef struct {
    int half_wheelbase;     // 轴距一半 l (mm)
    int half_track;         // 轮距一半 w (mm)
    int wheel_radius;       // 车轮半径 r (mm), 预留
    int max_wheel_speed;    // 100% 占空比对应的轮子线速度 (mm/s), 装机实测校准
} mecanum_cfg_t;

// 默认几何参数 (占位值, 装机后按实测修改)
extern const mecanum_cfg_t g_mecanum_default_cfg;

// 逆解: vx/vy 车体线速度 mm/s, wz 角速度 deg/s (逆时针为正)
// 输出 pwm_out[4] = -100..100 (符号=方向, 绝对值=占空比百分比)
void mecanum_ik(const mecanum_cfg_t *cfg, int vx, int vy, int wz, int pwm_out[4]);

#endif

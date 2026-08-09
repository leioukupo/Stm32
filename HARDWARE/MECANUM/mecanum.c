#include "mecanum.h"

//////////////////////////////////////////////////////////////////////////////////
// 麦克纳姆轮运动学逆解实现 (Phase 3)
//////////////////////////////////////////////////////////////////////////////////

const mecanum_cfg_t g_mecanum_default_cfg = {
    120,    // half_wheelbase (mm) 占位
    120,    // half_track     (mm) 占位
    50,     // wheel_radius   (mm) 占位
    1000,   // max_wheel_speed (mm/s) 占位, 满占空比对应轮速, 实测校准
};

void mecanum_ik(const mecanum_cfg_t *cfg, int vx, int vy, int wz, int pwm_out[4])
{
    int k, wz_lin, wheel[MECANUM_WHEELS];
    int i, duty;

    if (cfg == 0 || pwm_out == 0)
        return;

    if (cfg->max_wheel_speed <= 0) {
        for (i = 0; i < MECANUM_WHEELS; i++)
            pwm_out[i] = 0;
        return;
    }

    k = cfg->half_wheelbase + cfg->half_track;
    if (k <= 0)
        k = 1;

    // 角速度(deg/s) -> 轮位线速度(mm/s): k * ω, ω 换算 rad/s (π ≈ 31416/10000)
    wz_lin = (wz * k * 31416) / 1800000;

    wheel[0] = vx - vy - wz_lin;    // FL
    wheel[1] = vx + vy + wz_lin;    // FR
    wheel[2] = vx + vy - wz_lin;    // RL
    wheel[3] = vx - vy + wz_lin;    // RR

    // 归一化到 -100..100 (独立限幅, 保证不超出占空比范围)
    for (i = 0; i < MECANUM_WHEELS; i++) {
        duty = (wheel[i] * 100) / cfg->max_wheel_speed;
        if (duty > 100)
            duty = 100;
        if (duty < -100)
            duty = -100;
        pwm_out[i] = duty;
    }
}

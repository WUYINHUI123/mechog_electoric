#include "gait.h"
#include "main.h"
#include <stdint.h>
#include <math.h>
#include <stdbool.h>

// 安全的限幅宏定义
#ifndef constrainValue
#define constrainValue(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

// 快速相位取模宏
#define WRAP_PHASE(p) ((p) >= 1.0f ? (p) - 1.0f : (p))

// ==============================================================
// ?? 1. 仿造开源库引入三维向量与腿部数据结构封装 (OOP思想)
// ==============================================================
typedef struct {
    float x;
    float y; // Y轴向下，代表高度
    float z; // Z轴代表横向（侧滑/转弯使用）
} Vec3;

typedef struct {
    Vec3 pos;       // 当前足端位置
    float phase;    // 当前腿的局部相位 (0~1)
    bool isStance;  // 是否处于支撑相
} LegData_t;

// 实例化4条腿的数据对象
LegData_t LegLF, LegRF, LegLB, LegRB;

float t_global = 0;    

// 兼容原有的全局 Node 输出（将内部 Vec3 映射给外部调用的 LeftFront 等变量）
Node LeftFront;
Node RightFront;
Node LeftBack;
Node RightBack;

// ==============================================================
// ?? 2. 核心数学库：五次多项式 (5th-Order Polynomial)
// ==============================================================
static float poly5(float u) {
    float u2 = u * u;
    float u3 = u2 * u;
    float u4 = u3 * u;
    float u5 = u4 * u;
    return 10.0f * u3 - 15.0f * u4 + 6.0f * u5;
}

// ==============================================================
// ?? 3. 足端轨迹发生器 (类似开源库的 FootSwingTrajectory 类)
// ==============================================================
static void computeFootTrajectory(LegData_t* leg, uint8_t height, int8_t stride_x, int8_t stride_z, float offset_x) {
    float stride_f_x = (float)stride_x;
    float stride_f_z = (float)stride_z; // 为未来的横向移动预留
    float height_f = (float)height;

    // 判断是摆动相还是支撑相
    if (leg->phase < faai) {
        leg->isStance = false;
        // 归一化摆动相时间 [0, 1]
        float s = leg->phase / faai; 

        // X轴：从后向前平滑迈出
        leg->pos.x = offset_x - (stride_f_x / 2.0f) + stride_f_x * poly5(s);
        // Z轴：侧向移动预留
        leg->pos.z = -(stride_f_z / 2.0f) + stride_f_z * poly5(s);

        // Y轴：非对称优化 (快抬腿，缓落足)
        float peak_ratio = 0.45f; 
        float y_start = HEIGHT_MAX;
        float y_peak  = HEIGHT_MAX - height_f; 

        if (s <= peak_ratio) {
            float u = s / peak_ratio;
            leg->pos.y = y_start + (y_peak - y_start) * poly5(u);
        } else {
            float u = (s - peak_ratio) / (1.0f - peak_ratio);
            leg->pos.y = y_peak + (y_start - y_peak) * poly5(u);
        }
    } else {
        leg->isStance = true;
        // 归一化支撑相时间 [0, 1]
        float s = (leg->phase - faai) / (1.0f - faai); 

        // X轴、Z轴：严格匀速向后划动，推动机身恒速平稳前进
        leg->pos.x = offset_x + (stride_f_x / 2.0f) - stride_f_x * s;
        leg->pos.z = (stride_f_z / 2.0f) - stride_f_z * s;
        leg->pos.y = HEIGHT_MAX;
    }

    // 结构安全绝对限幅 (防止 IK 逆解崩溃)
    leg->pos.y = constrainValue(leg->pos.y, 120.0f, 280.0f);
}

// ==============================================================
// ?? 4. 全局相位调度器 (类似开源库的 Gait 类)
// ==============================================================
static void updateGaitPhase(float offset_LF, float offset_RF, float offset_LB, float offset_RB, float freq) {
    t_global += freq;
    if (t_global >= Ts) t_global -= Ts; 

    float phi = t_global / Ts;
    if (phi >= 1.0f) phi -= 1.0f; 

    // 分配并映射局部相位
    LegLF.phase = WRAP_PHASE(phi + offset_LF);
    LegRF.phase = WRAP_PHASE(phi + offset_RF);
    LegLB.phase = WRAP_PHASE(phi + offset_LB);
    LegRB.phase = WRAP_PHASE(phi + offset_RB);
}

// 供外部调用的统一接口，更新完内部3D数据后，映射给2D的Node
static void processGait(float freq, uint8_t height, int8_t stride) {
    // 计算轨迹 (注意这里引入了侧滑 stride_z 为0，为以后拓展全向移动留好接口)
    computeFootTrajectory(&LegLF, height, stride, 0,  X_ZERO);
    computeFootTrajectory(&LegRF, height, stride, 0,  X_ZERO);
    computeFootTrajectory(&LegLB, height, stride, 0,  -X_ZERO);
    computeFootTrajectory(&LegRB, height, stride, 0,  -X_ZERO);

    // 映射回外部的全局变量，保证你的 main.c 完全不用改
    LeftFront.x  = LegLF.pos.x; LeftFront.y  = LegLF.pos.y;
    RightFront.x = LegRF.pos.x; RightFront.y = LegRF.pos.y;
    LeftBack.x   = LegLB.pos.x; LeftBack.y   = LegLB.pos.y;
    RightBack.x  = LegRB.pos.x; RightBack.y  = LegRB.pos.y;
}

// ==============================================================
// 5. 步态模式接口
// ==============================================================
void trot(float freq, uint8_t height, int8_t stride) {
    
    updateGaitPhase(0.0f, 0.5f, 0.5f, 0.0f, freq);
    processGait(freq, height, stride);
}

void pace(float freq, int8_t height, int8_t stride) {
    // PACE: 同侧同步相序 (骆驼步)
    updateGaitPhase(0.0f, 0.5f, 0.0f, 0.5f, freq);
    processGait(freq, height, stride);
}
void bound(float freq, int8_t height, int8_t stride) {
    // 兔子跳保持前后同步不变
    updateGaitPhase(0.0f, 0.0f, 0.5f, 0.5f, freq);
    processGait(freq, height, stride);
}

void stand(void) {
    LeftFront.y  = constrainValue(BASE_HEIGHT, 120.0f, 280.0f);
    RightFront.y = constrainValue(BASE_HEIGHT, 120.0f, 280.0f);
    LeftBack.y   = constrainValue(BASE_HEIGHT, 120.0f, 280.0f);
    RightBack.y  = constrainValue(BASE_HEIGHT, 120.0f, 280.0f);
    
    LeftFront.x  =  X_ZERO; RightFront.x =  X_ZERO;
    LeftBack.x   =  -X_ZERO; RightBack.x  =  -X_ZERO;
}

void gaitMode(MODE_SET mode, float freq, uint8_t height, int8_t stride) {
    switch (mode) {
        case TROT:  trot(freq, height, stride);  break;
        case PACE:  pace(freq, height, stride);  break;
        case BOUND: bound(freq, height, stride); break;
        case STAND: stand();                     break;
        default:    break;
    }
}
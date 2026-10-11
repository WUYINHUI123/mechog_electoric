#ifndef _GAIT_H
#define _GAIT_H

#include "main.h"
#include <stdint.h>
#include <math.h>
#include <stdbool.h>

#define pi      3.1415926   //π
#define faai    0.50         /* duty: swing-phase fraction. 0.60 made all 4 legs swing for 20% of the cycle (body dropped) */
#define Ts      1.2 

// 【修改点】：为了适配你们的长腿机器狗，调高了站立和抬腿的高度
#define HEIGHT_MAX   180      // 走路时抬腿的最大高度
#define BASE_HEIGHT    220  // 默认站立时的基础高度
#define X_ZERO    62.5f


typedef struct 
{
    float x;
    float y;
} Node;


typedef struct 
{
  float xSwing;    // 前半周期生成的摆动相的x坐标
  float ySwing;    // 前半周期生成的摆动相的y坐标
  float xSupport;  // 前半周期生成的支撑相的x坐标
  float ySupport;  // 前半周期生成的支撑相的x坐标
  float sigma;     // 轨迹生成三角函数中的相位
}GaitPhasesState;

typedef enum{
  TROT = 1,
  PACE,
  WALK,       //WALK比pace频率低一点
  BOUND,
  STAND,
  JUMP,
  CHECK,
}MODE_SET;

#define constrainValue(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数

/* ===== 原地踏步自转(差速转向)控制量 =====
 * 由 main.c 每个控制周期写入(单位 mm)
 * g_turn > 0 : 左侧腿步幅大于右侧,机身向一侧缓慢自转
 * g_turn < 0 : 转向相反(真机若转反,把符号取反即可)
 * = 0        : 普通直线踏步/前进
 * 安全建议范围: -8 ~ +8,越小越慢越稳 */
extern float g_turn;

void gaitMode(MODE_SET mode, float freq, uint8_t height, int8_t stride);

#endif
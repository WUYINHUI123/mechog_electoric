#ifndef __LEG_IK_H__
#define __LEG_IK_H__

#include "main.h"
#include <math.h>
#include <stdbool.h>

// ==========================================
// 物理参数配置区 (mm)
// ==========================================
#define L1  113.64f
#define L2  189.34f
#define L3  189.34f
#define L4  113.64f
#define L5  125.00f

// 运动安全限制
#define Y_MIN   120.0f
#define Y_MAX   285.0f

// ==========================================
// 电机方向配置（根据实际安装调整）
// ==========================================
#define LEG1_ALPHA_DIR -1.0f    // 腿1左电机方向
#define LEG1_BETA_DIR   1.0f    // 腿1右电机方向
#define LEG2_ALPHA_DIR  -1.0f    // 腿2左电机方向  
#define LEG2_BETA_DIR   1.0f    // 腿2右电机方向
#define LEG3_ALPHA_DIR  -1.0f    // 腿3左电机方向
#define LEG3_BETA_DIR   1.0f    // 腿3右电机方向
#define LEG4_ALPHA_DIR  -1.0f    // 腿4左电机方向
#define LEG4_BETA_DIR   1.0f    // 腿4右电机方向

// ==========================================
// 零点偏移配置 (OFFSET)
// ?? 注意：这里我先写了 0.0f，请务必把你原来工程里
// 自己标定测量的真实 OFFSET 数值填回这里！
// ==========================================
#define LEG1_ALPHA_OFFSET   -0.698f    //1号
#define LEG1_BETA_OFFSET    0.43625f 
#define LEG2_ALPHA_OFFSET   -0.08725f   //3号
#define LEG2_BETA_OFFSET    0.29665f 
#define LEG3_ALPHA_OFFSET   0.349f         //5号
#define LEG3_BETA_OFFSET    -0.64565f    //6号
#define LEG4_ALPHA_OFFSET   0.57585f      //7号
#define LEG4_BETA_OFFSET    -0.5235f 

// ==========================================
// 核心算法函数声明区
// ==========================================

/**
 * @brief 1. 逆运动学求解 (IK - 用于算出脚尖坐标对应的电机角度)
 */
bool solveInverseKinematics(float x, float y, float *alpha, float *beta);

/**
 * @brief 2. 雅可比矩阵与虚拟模型控制 (VMC - 用于算出需要多少力矩来抵抗地面)
 */
bool calcJacobianAndTorque(float x, float y, float alpha, float beta, 
                           float Fx, float Fy, 
                           float *tau1, float *tau2);

/**
 * @brief 3. 本体感觉/触觉核心 (Proprioception - 用于根据电机反馈电流，猜出脚底踩了多大的石头)
 */
bool estimateFootForce(float x, float y, float alpha, float beta, 
                       float tau1, float tau2, 
                       float *Fx, float *Fy);

#endif
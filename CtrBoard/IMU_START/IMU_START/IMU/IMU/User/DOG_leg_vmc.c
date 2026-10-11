#include "DOG_leg_vmc.h"
#include <math.h>

/**
 * @brief 浮点数限制
 */
#define CLAMP(x, min, max)  ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 五连杆逆运动学求解 (IK - 用于位置控制)
 * @param x: 足端X坐标（相对髋关节）
 * @param y: 足端Y坐标（垂直地面，向上为正）
 * @param alpha: 输出左电机角度（弧度）
 * @param beta: 输出右电机角度（弧度）
 * @return: true成功，false无解（超出工作空间）
 */
bool solveInverseKinematics(float x, float y, float *alpha, float *beta) {
    // 限制Y在安全范围
    y = CLAMP(y, Y_MIN, Y_MAX);

    // 左腿逆解 (O-A-B)
    float distO = sqrtf(x * x + y * y);
    if (distO > (L1 + L2) || distO < fabsf(L1 - L2)) return false;
    
    float phiO = atan2f(y, x);
    float cosA = (L1 * L1 + distO * distO - L2 * L2) / (2.0f * L1 * distO);
    *alpha = phiO + acosf(CLAMP(cosA, -1.0f, 1.0f));

    // 右腿逆解 (D-C-B)
    float dx = x - L5;
    float distD = sqrtf(dx * dx + y * y);
    if (distD > (L4 + L3) || distD < fabsf(L4 - L3)) return false;
    
    float phiD = atan2f(y, dx);
    float cosC = (L4 * L4 + distD * distD - L3 * L3) / (2.0f * L4 * distD);
    *beta = phiD - acosf(CLAMP(cosC, -1.0f, 1.0f));

    return true;
}

/**
 * @brief 五连杆雅可比矩阵与虚拟模型控制 (VMC - 用于力矩/柔顺控制)
 * @note  基于虚功原理：[Tau1; Tau2] = J^T * [Fx; Fy]
 * @param x: 当前足端真实 X 坐标 (需通过正运动学算出，或假设等于期望X)
 * @param y: 当前足端真实 Y 坐标 
 * @param alpha: 当前左电机真实角度 (由电机 CAN 反馈获取)
 * @param beta:  当前右电机真实角度 (由电机 CAN 反馈获取)
 * @param Fx: 足端期望在 X 轴输出的虚拟力 (N，向后蹬地为正)
 * @param Fy: 足端期望在 Y 轴输出的虚拟力 (N，向下蹬地为正)
 * @param tau1: 输出左电机应下发的扭矩 (Nm)
 * @param tau2: 输出右电机应下发的扭矩 (Nm)
 * @return: true成功计算，false处于奇异位形(死点)无法计算
 */
bool calcJacobianAndTorque(float x, float y, float alpha, float beta, 
                           float Fx, float Fy, 
                           float *tau1, float *tau2) {
    
    // 1. 计算两个电机末端 (膝盖 A 和 C) 的坐标
    float xA = L1 * cosf(alpha);
    float yA = L1 * sinf(alpha);
    float xC = L5 + L4 * cosf(beta);
    float yC = L4 * sinf(beta);

    // 2. 构造速度约束雅可比矩阵的中间元素 (隐式求导)
    float J11 = x - xA;
    float J12 = y - yA;
    float J21 = x - xC;
    float J22 = y - yC;

    float K1 = L1 * (cosf(alpha) * J12 - sinf(alpha) * J11);
    float K2 = L4 * (cosf(beta) * J22  - sinf(beta)  * J21);

    // 3. 计算微积分矩阵行列式 (判断是否处于机械死点)
    float detA = J11 * J22 - J12 * J21;

    // 安全保护：如果行列式接近0，说明腿伸得太直或者折叠到底了，强行输出力矩会导致电机暴走
    if (fabsf(detA) < 1e-4f) {
        *tau1 = 0.0f;
        *tau2 = 0.0f;
        return false;
    }

    // 4. 核心魔法：通过雅可比矩阵的转置 (J^T) 将足端推力 (Fx, Fy) 映射为关节力矩 (tau1, tau2)
    // ?? 核心修复：由于长度单位是 mm，算出的力矩是 N·mm。必须除以 1000 换算成电机需要的 N·m ！！！
    *tau1 = ((K1 / detA) * ( J22 * Fx - J21 * Fy)) / 1000.0f;
    *tau2 = ((K2 / detA) * (-J12 * Fx + J11 * Fy)) / 1000.0f;

    return true;
}

/**
 * @brief 本体感觉(触觉)核心：利用逆雅可比矩阵反推脚尖受力
 * @param x, y: 脚尖当前坐标 (mm)
 * @param alpha, beta: 当前电机真实角度 (rad)
 * @param tau1, tau2: 电机当前真实反馈力矩 (Nm)
 * @param Fx, Fy: 输出反推得到的脚尖受力 (N)
 */
bool estimateFootForce(float x, float y, float alpha, float beta, 
                       float tau1, float tau2, 
                       float *Fx, float *Fy) {
    // 1. 算出两膝盖点
    float xA = L1 * cosf(alpha);
    float yA = L1 * sinf(alpha);
    float xC = L5 + L4 * cosf(beta);
    float yC = L4 * sinf(beta);

    // 2. 构造雅可比偏导元素
    float J11 = x - xA;
    float J12 = y - yA;
    float J21 = x - xC;
    float J22 = y - yC;

    float K1 = L1 * (cosf(alpha) * J12 - sinf(alpha) * J11);
    float K2 = L4 * (cosf(beta) * J22  - sinf(beta)  * J21);

    // 安全保护：奇异点检测
    if (fabsf(K1) < 1e-4f || fabsf(K2) < 1e-4f) {
        *Fx = 0.0f; *Fy = 0.0f;
        return false;
    }

    // 3. 逆向魔法：F = (J^T)^-1 * tau
    // 乘以 1000 是为了将单位从 (Nm/mm) 还原成标准的牛顿 (N)
    *Fx = ((J11 / K1) * tau1 + (J21 / K2) * tau2) * 1000.0f;
    *Fy = ((J12 / K1) * tau1 + (J22 / K2) * tau2) * 1000.0f;

    return true;
}
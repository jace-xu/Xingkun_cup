#ifndef PID_H
#define PID_H

typedef struct{
	
	// pid系数
	float Kp;       
	float Ki;
	float Kd;
	
	// 目标值,上个周期的测量值,积分值
	float Target;	
	float Last_measured;
	float integral;
	
	// 积分限幅和输出限幅
	float integralLimit;
	float outputLimit;
}PID;

// 初始化PID函数
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float integralLimit, float outputLimit);
// 改变PID设定值
void PID_SetTarget(PID *pid, float Target);
// PID计算
float PID_compute(PID *pid, float measured,float dt);
// 改变pid参数（模糊pid算法）
void setPID_parameter(PID *pid, float Kp, float Ki, float Kd, float integralLimit, float outputLimit);
// pid参数Reset
void ResetPID(PID *pid);
#endif


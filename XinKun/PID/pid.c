#include "pid.h"

/*
	@brief 初始化PID函数
*/
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float integralLimit,float outputLimit){
	
	pid -> Kp = Kp;
	pid -> Ki = Ki;
	pid -> Kd = Kd;
	
	pid -> outputLimit = outputLimit;
	pid -> integralLimit = integralLimit;
	
	pid -> Target = 0.0f;
	pid -> Last_measured = 0.0f;
	pid -> integral = 0.0f;
}


/*
	@brief 改变设定值
*/
void PID_SetTarget(PID *pid, float Target){
	pid -> Target = Target;
}



/*
	@brief 执行一次PID运算
*/
float PID_compute(PID *pid, float measured, float dt){
	
	// 计算误差
	float error = pid->Target - measured;
	// 计算积分
	pid->integral = error * dt + pid->integral;
	// 积分限幅
	if(pid->integral >= pid ->integralLimit) pid->integral = pid ->integralLimit;
	if(pid->integral <= -pid ->integralLimit) pid->integral = - pid ->integralLimit;
	
	// 初始赋值
	float Kp = pid->Kp;
	float Ki = pid->Ki;
	float Kd = pid->Kd;
	float integral = pid->integral;
	float last_measured = pid -> Last_measured;
	
	// 计算pid
	float output = Kp * error + Ki * integral + Kd * (last_measured - measured)/dt;
	// 输出限幅
	if(output >= pid -> outputLimit) output = pid -> outputLimit;
	if(output <= -pid -> outputLimit) output = - pid -> outputLimit;
	
	// 存储这次测量值
	pid -> Last_measured = measured;
	return output;
}


/*
	@brief 改变pid参数
*/
void setPID_parameter(PID *pid, float Kp, float Ki, float Kd, float integralLimit, float outputLimit){
	// 改变参数
	pid -> Kp = Kp;
	pid -> Ki = Ki;
	pid -> Kd = Kd;
	pid -> outputLimit = outputLimit;
	pid -> integralLimit = integralLimit;

	// 清空缓存
	pid ->integral = 0;
}

/*
	@brief ResetPID
*/
void ResetPID(PID *pid){
	pid -> Target = 0.0f;
	pid -> Last_measured = 0.0f;
	pid -> integral = 0.0f;
}


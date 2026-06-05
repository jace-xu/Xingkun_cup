#ifndef FILTER_H
#define FILTER_H

typedef struct{
	float lastData;
	float alpha;
}LowPassfilter;

/*
	@brief 一阶低通滤波函数
	@parameter *LowPassfilter为数据结构体指针，使用函数前要先初始化结构体
							LowPassfilter中alpha的计算：
							alpha = Δt/(1/Ω + Δt)
						    Ω为截止频率
	@parameter	data是本次传入的实际数据
*/
float LowPassfilter_First(LowPassfilter *LowPassfilter,float data);

#endif

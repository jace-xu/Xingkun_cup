#include "filter.h"

/*
	@brief 一阶低通滤波函数
*/
float LowPassfilter_First(LowPassfilter *LowPassfilter,float data){
	float output = LowPassfilter->alpha * data + (1 - LowPassfilter->alpha)*LowPassfilter->lastData;
	LowPassfilter->lastData = output;
	return output;
}
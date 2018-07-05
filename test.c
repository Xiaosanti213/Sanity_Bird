#include <stdio.h>
#include "AnalogInLookup.h"


extern const float analog_in_angle_deg[68];
extern const float servo_out_angle_deg[68];

int main()
{
	float a;
	printf("input an input degree from 0 to 360:  ");
	scanf("%f", &a);
	
	if (a > 360)
		a = 360;
	else if(a < 0)
		a = 0;
	
	int index_left = a/5.334;
	int index_right = index_left+1;
	
	// 可能出现两个问题：5.334会造成累积误差 是否分段
	printf("index_left: %d\n", index_left);
	// 在找到索引的角度看下是否在两个范围之间
	printf("the left degree is: %.2f\n", analog_in_angle_deg[index_left]);
	printf("the right one is: %.2f\n", analog_in_angle_deg[index_right]);
	// 测试几个比较刁钻的角度
	
	// 执行线性插值
	float temp = servo_out_angle_deg[index_right]-servo_out_angle_deg[index_left];
	float temp1 = analog_in_angle_deg[index_right]-analog_in_angle_deg[index_left];
	
	printf("%.2f\n",temp);
	temp /= temp1;//斜率
	printf("%.2f\n",temp1);
	printf("%.2f\n",temp);
	
	// 这个结果来看没啥问题了
	float result = (a - analog_in_angle_deg[index_left])*temp+servo_out_angle_deg[index_left];
	printf("The final result is: %.2f\n", result);
	
	return 0;
	
}



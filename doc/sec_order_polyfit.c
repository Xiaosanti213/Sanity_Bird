
#include <stdio.h>

int main()
{
  float angle_input = 0;
  // 计算电机角度和舵机指令映射关系
  const float ctrl_points[4] = { 74.6740, 197.353, 261.36, 357.37 };//样点索引

  const float coeff01[3] = { 0.0087, -1.1790, 1.2563 };
  const float coeff12[3] = { -0.0028, 1.1334, -106.7529 };
  const float coeff23[3] = { 0.0016, -0.4938, 42.0787 };
  const float coeff34[3] = { -0.0051, 2.9076, -387.0546 };
  
  float temp_c[3] = {0,0,0};
  int i;
  
  
  while(1)
	  
	  {
  printf("Enter a float angle: \n");
  scanf("%f", &angle_input);
  if (angle_input <= ctrl_points[0])
	  for (i = 0; i < 3; i++)
		  temp_c[i] = coeff01[i];
  else if (ctrl_points[0] < angle_input && angle_input <= ctrl_points[1])
	for (i = 0; i < 3; i++)
		  temp_c[i] = coeff12[i];
  else if (ctrl_points[1] < angle_input && angle_input <= ctrl_points[2])
	  for (i = 0; i < 3; i++)
		  temp_c[i] = coeff23[i];
  else
	  for (i = 0; i < 3; i++)
		  temp_c[i] = coeff34[i];

  float poly_temp = temp_c[0] * angle_input * angle_input;
  poly_temp += temp_c[1] * angle_input;
  poly_temp += temp_c[2];

  //debug[2] = angle_input;
  //debug[3] = poly_temp;
  
  printf("poly value: %.2f\n", poly_temp);
  for (i = 0; i < 3; i++)
	printf("coeff: %.4f\n", temp_c[i]);
	  }
}






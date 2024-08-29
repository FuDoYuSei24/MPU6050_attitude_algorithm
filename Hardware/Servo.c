#include "stm32f10x.h"                  // Device header
#include "PWM.h"

void Servo_Init(void)
{
	
	PWM_Init();
	
}


void Servo_SetAngle1(float Angle)
{
	PWM_SetComparel1(Angle/180*2000+500);
}

void Servo_SetAngle2(float Angle)
{
	PWM_SetComparel2(Angle/180*2000+500);
}


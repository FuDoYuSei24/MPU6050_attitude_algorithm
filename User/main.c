#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"


int16_t AX,AY,AZ,GX,GY,GZ;
static float ax, ay, az, gx, gy, gz, yaw, roll, pitch;
uint8_t ID;
float pAccelOut[3];
float pEularAngleOut[3];
float pGyroOut[3];

int main()
{
	OLED_Init();
	MPU6050_Init();
	ID=MPU6050_GetID();
	OLED_ShowHexNum(1,1,ID,2);
	OLED_ShowString(2,1,"X:");
	OLED_ShowString(3,1,"Y:");
	OLED_ShowString(4,1,"Z:");
	
	while(1)
	{
		//MPU6050_GetData(&AX,&AY,&AZ,&GX,&GY,&GZ);
		MPU6050_Proc();
		App_MPU6050_GetResult(pAccelOut, pGyroOut, pEularAngleOut);
		OLED_ShowSignedNum(2,3,pEularAngleOut[0],5);
		OLED_ShowSignedNum(3,3,pEularAngleOut[1],5);
		OLED_ShowSignedNum(4,3,pEularAngleOut[2],5);
		
		
	}
}

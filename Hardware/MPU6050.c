#include "stm32f10x.h"                  // Device header
#include "MPU6050_Reg.h"
#include "math.h"


#define MPU6050_ADDRESS  0xD0
static float ax, ay, az, gx, gy, gz, yaw, roll, pitch;


void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data)//指定地址写入一个字节的数据
{
	 //uint32_t Timeout;
	//起始条件
    I2C_GenerateSTART(I2C2,ENABLE);
	  while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT)!=SUCCESS);//检查EV5事件
    
	  //发送从机地址
	  I2C_Send7bitAddress(I2C2,MPU6050_ADDRESS,I2C_Direction_Transmitter);
	//	MyI2C_ReceiveAckByte();函数自带应答功能，不用再写
	  while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)!=SUCCESS);//检查EV6事件
    

    //发送数据
	  I2C_SendData(I2C2,RegAddress);
	  while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING)!=SUCCESS);//检查EV8事件
    
    I2C_SendData(I2C2,Data);
	  while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED)!=SUCCESS);//检查EV5事件
    	
	  //结束条件
	  I2C_GenerateSTOP(I2C2,ENABLE);

}


uint8_t MPU6050_ReadReg(uint8_t RegAddress)//指定地址读一个字节的数据
{
	uint8_t Data;

  I2C_GenerateSTART(I2C2,ENABLE);
	while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT)!=SUCCESS);//检查EV5事件
    
	//发送从机地址
	I2C_Send7bitAddress(I2C2,MPU6050_ADDRESS,I2C_Direction_Transmitter);
	//	MyI2C_ReceiveAckByte();函数自带应答功能，不用再写
	while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)!=SUCCESS);//检查EV6事件
    

  //发送数据
	I2C_SendData(I2C2,RegAddress);
	while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED)!=SUCCESS);//检查EV8事件

	
	I2C_GenerateSTART(I2C2,ENABLE);
	while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT)!=SUCCESS);//检查EV5事件

	I2C_Send7bitAddress(I2C2,MPU6050_ADDRESS,I2C_Direction_Receiver);//接收方向
	while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)!=SUCCESS);//检查EV6事件

  //
	I2C_AcknowledgeConfig(I2C2,DISABLE);//配置应答位
  I2C_GenerateSTOP(I2C2,ENABLE);//配置停止位
	while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED)!=SUCCESS);//检查EV7事件
	
	//读取DR
	Data=I2C_ReceiveData(I2C2);


  I2C_AcknowledgeConfig(I2C2,ENABLE);
	return Data;
}


void MPU6050_Init(void)
{
//	MyI2C_Init();
	//对硬件I2C进行初始化
	
	  //一：开启I2C和GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	//把PB10和PB11都初始化为复用开漏模式
	//开漏：I2C协议要求；复用：GPIO的控制权交给硬件
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;//复用开漏模式
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
  //初始化I2C2外设
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode=I2C_Mode_I2C;//I2C模式
	I2C_InitStructure.I2C_ClockSpeed=50000;//时钟速度
	I2C_InitStructure.I2C_DutyCycle=I2C_DutyCycle_2;//时钟占空比
	I2C_InitStructure.I2C_Ack=I2C_Ack_Enable;//应答位配置
	I2C_InitStructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;//指定STM32作为从机可以响应几位的地址
	I2C_InitStructure.I2C_OwnAddress1=0x00;//用于指定STM32的自身地址，和上面相呼应
	I2C_Init(I2C2,&I2C_InitStructure);
	
	//使能I2C
	I2C_Cmd(I2C2,ENABLE);
	
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x09);
	MPU6050_WriteReg(MPU6050_CONFIG,0x06);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG,0x18);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0x18);
	
}


void MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,
	int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ)
{
	uint8_t DataH,DataL;
	DataH=MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	*AccX=(DataH<<8)|DataL;
	
	
	DataH=MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	*AccY=(DataH<<8)|DataL;
	
	
	DataH=MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	*AccZ=(DataH<<8)|DataL;
	
	
	DataH=MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	*GyroX=(DataH<<8)|DataL;
	
	
	DataH=MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	*GyroY=(DataH<<8)|DataL;
	
	
	DataH=MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	*GyroZ=(DataH<<8)|DataL;
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}


void MPU6050_Proc(void)
{
	uint8_t DataH,DataL;
	DataH=MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	ax=(int16_t)((DataH<<8)|DataL)*16*10/32768.0f;
	
	
	DataH=MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
  ay=(int16_t)((DataH<<8)|DataL)*16*10/32768.0f;
	
	
	DataH=MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
  az=(int16_t)((DataH<<8)|DataL)*16*10/32768.0f;
	
	DataH=MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	gx=(int16_t)((DataH<<8)|DataL)*1.95/32;
	
	
	DataH=MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	gy=(int16_t)((DataH<<8)|DataL)*1.95/32;
	
	
	DataH=MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	gz=(int16_t)((DataH<<8)|DataL)*1.95/32;
	
 
	float roll_a = atan2(ay, az) / 3.141593f * 180.0f;
	float pitch_a = -atan2(ax, az) / 3.141593f * 180.0f;
	
	float yaw_g = yaw + gz * 0.005;
	float roll_g = roll + gx * 0.005;
	float pitch_g = pitch + gy * 0.005;
	
	const float alpha = 0.95238;
	
	yaw = yaw_g;
	roll = alpha * roll_g + (1-alpha) * roll_a;
	pitch = alpha * pitch_g + (1-alpha) * pitch_a;
}


void App_MPU6050_GetResult(float *pAccelOut, float *pGyroOut, float *pEularAngleOut)
{
	pAccelOut[0] = ax;
	pAccelOut[1] = ay;
	pAccelOut[2] = az;
	
	
	pGyroOut[0] = gx;
	pGyroOut[1] = gy;
	pGyroOut[2] = gz;
	
	pEularAngleOut[0] = yaw*4;
	pEularAngleOut[1] = roll;
	pEularAngleOut[2] = pitch;
}


#include "mytask.h"
#include "mpu6050.h"
#include "main.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
#include "inv_mpu_user.h"

struct mpu6050_data OutMpu;
struct ruifen ruifen;
u8 compass_get[5]={0x68,0x04,0x00,0x04,0x08};
extern uint8_t USART_RX_STA;
uint8_t txbuf2[50];
extern uint8_t compass_receive[14];


void task_200hz()
{
	
//	mpu_dmp_get_data();
//	MPU_Get_Accelerometer(&OutMpu.acc_x,&OutMpu.acc_y,&OutMpu.acc_z);
//	MPU_Get_Gyroscope(&OutMpu.gyro_x,&OutMpu.gyro_y,&OutMpu.gyro_z);
//	
//	printf ("acc_x=%d\n",OutMpu.acc_x);
//	printf ("acc_y=%d\n",OutMpu.acc_y);
//	printf ("acc_z=%d\n",OutMpu.acc_z);
//	printf ("gyro_x=%d\n",OutMpu.gyro_x);
//	printf ("gyro_y=%d\n",OutMpu.gyro_y);
//	printf ("gyro_z=%d\n",OutMpu.gyro_z);

//	printf ("\r\n");
}

void task_100hz()
{

	
//		compass_receive[i]=aRxBuffer;
//	i++;
//	if(i==14)
//	{
//	i=0;
//	HAL_UART_Transmit(&huart2,compass_receive,14,1000);
//	}
	
//  //HAL_UART_Transmit(&huart2,&aRxBuffer,1,0);
//  HAL_UART_Receive_IT(&huart2,&aRxBuffer,1);
	
	
	
}


void task_20hz()
{
	
if(USART_RX_STA==0x00)
{HAL_UART_Transmit(&huart2,compass_get,5,1000);}
	

	if(USART_RX_STA&0x80)
	{
	
memcpy(txbuf2,"	显示角度\n",50);
HAL_UART_Transmit(&huart1,txbuf2,strlen((char *)txbuf2),1000);
  
ruifen.Roll=(compass_receive[4]%16)*100+(compass_receive[5]/16)*10+(compass_receive[5]%16)+(float)(compass_receive[6]/16)/10+(float)(compass_receive[6]%16)/100;
if((compass_receive[4]/16)==1) ruifen.Roll=-ruifen.Roll;
printf ("roll=%f\n",ruifen.Roll);

ruifen.Pitch=(compass_receive[7]%16)*100+(compass_receive[8]/16)*10+(compass_receive[8]%16)+(float)(compass_receive[9]/16)/10+(float)(compass_receive[9]%16)/100;
if((compass_receive[7]/16)==1) ruifen.Pitch=-ruifen.Pitch;
printf ("Pitch=%f\n",ruifen.Pitch);

ruifen.Heading=(compass_receive[10]%16)*100+(compass_receive[11]/16)*10+(compass_receive[11]%16)+(float)(compass_receive[12]/16)/10+(float)(compass_receive[12]%16)/100;
if((compass_receive[10]/16)==1) ruifen.Heading=-ruifen.Heading;
printf ("Heading=%f\n",ruifen.Heading);


	USART_RX_STA=0x00;	//接收完成了
	
}
	
}



void task_5hz()
{
//	MPU_Get_Accelerometer(&OutMpu.acc_x,&OutMpu.acc_y,&OutMpu.acc_z);
//	MPU_Get_Gyroscope(&OutMpu.gyro_x,&OutMpu.gyro_y,&OutMpu.gyro_z);
//	
//	printf ("acc_x=%d\n",OutMpu.acc_x);
//	printf ("acc_y=%d\n",OutMpu.acc_y);
//	printf ("acc_z=%d\n",OutMpu.acc_z);
//	printf ("gyro_x=%d\n",OutMpu.gyro_x);
//	printf ("gyro_y=%d\n",OutMpu.gyro_y);
//	printf ("gyro_z=%d\n",OutMpu.gyro_z);
//	
//	printf ("pitch=%f\n",OutMpu.pitch);
//	printf ("roll=%f\n",OutMpu.roll);
//	printf ("yaw=%f\n",OutMpu.yaw);
//	printf ("\r\n");
	
}




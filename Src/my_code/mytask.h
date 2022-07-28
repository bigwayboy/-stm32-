#ifndef _MYTASK_H
#define _MYTASK_H


struct mpu6050_data{
	
		short acc_x;
		short acc_y;
		short acc_z;
		
		short gyro_x;
		short gyro_y;
		short gyro_z;
	
		float pitch;    //¸©Ñö½Ç
	  float roll;     //·­¹ö½Ç
	  float yaw;      //Æ«º½½Ç
};
struct ruifen{
	
		int pit;
		int acc_y;
		int acc_z;
		
		short gyro_x;
		short gyro_y;
		short gyro_z;
	
		float Pitch;    //¸©Ñö½Ç
	  float Roll;     //·­¹ö½Ç
	  float Heading;      //Æ«º½½Ç
};

extern struct mpu6050_data OutMpu;



void task_200hz(void);
void task_100hz(void);
void task_20hz(void);
void task_5hz(void);



#endif




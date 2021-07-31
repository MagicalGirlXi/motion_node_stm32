#ifndef MOTION_FUNC_H
#define MOTION_FUNC_H


#define ADDRESS_W 0xD0   //??? 0x68
#define ADDRESS_R 0xD1   //???  0x69
#define MPU_PWR_MGMT1_REG		0X6B	//???????1
#define MPU_GYRO_CFG_REG		0X1B	//????????
#define MPU_ACCEL_CFG_REG		0X1C	//?????????
#define MPU_SAMPLE_RATE_REG		0X19	//??????????
#define MPU_INT_EN_REG			0X38	//???????
#define MPU_USER_CTRL_REG		0X6A	//???????
#define MPU_FIFO_EN_REG			0X23	//FIFO?????
#define MPU_INTBP_CFG_REG		0X37	//??/???????
#define MPU_DEVICE_ID_REG		0X75	//??ID???
#define MPU_PWR_MGMT2_REG		0X6C	//???????2
#define MPU_CFG_REG				0X1A	//????? ??????????
#define MPU_TEMP_OUTH_REG		0X41	//?????????
#define MPU_TEMP_OUTL_REG		0X42	//????8????

#define MPU_ACCEL_XOUTH_REG		0X3B	//????,X??8????
#define MPU_ACCEL_XOUTL_REG		0X3C	//????,X??8????
#define MPU_ACCEL_YOUTH_REG		0X3D	//????,Y??8????
#define MPU_ACCEL_YOUTL_REG		0X3E	//????,Y??8????
#define MPU_ACCEL_ZOUTH_REG		0X3F	//????,Z??8????
#define MPU_ACCEL_ZOUTL_REG		0X40	//????,Z??8????

#define MPU_GYRO_XOUTH_REG		0X43	//????,X??8????
#define MPU_GYRO_XOUTL_REG		0X44	//????,X??8????
#define MPU_GYRO_YOUTH_REG		0X45	//????,Y??8????
#define MPU_GYRO_YOUTL_REG		0X46	//????,Y??8????
#define MPU_GYRO_ZOUTH_REG		0X47	//????,Z??8????
#define MPU_GYRO_ZOUTL_REG		0X48	//????,Z??8????

#define DEFAULT_MPU_HZ  (100)

#define q30  1073741824.0f


void MPU_6050_Init(void);

unsigned char  MPU_Set_LPF(unsigned short lpf);
void read_all(void);
unsigned char HAL_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
unsigned char HAL_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx);
unsigned char mpu_dmp_init(void);
unsigned short inv_row_2_scale(const signed char *row);
unsigned char run_self_test(void);
unsigned char mpu_dmp_get_data(float *pitch,float *roll,float *yaw);
void read_acc(short* accx, short* accy, short* accz,float* temper);

static signed char gyro_orientation[9] = { 1, 0, 0,
																					 0, 1, 0,
																					 0, 0, 1
																				 };

extern float pitch,roll,yaw;

#endif

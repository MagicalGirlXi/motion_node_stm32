//-----------------------------------------------------------------
// LED驱动头文件
// 头文件名: spi.h
// 作    者: 凌智电子
// 开始日期: 2014-01-28
// 完成日期: 2014-01-28
// 修改日期: 2014-02-16
// 当前版本: V1.0.1
// 历史版本:
//   - V1.0: (2014-02-07)spi驱动头文件
// - V1.0.1:(2014-02-16)头文件中不包含其他头文件
//-----------------------------------------------------------------
#ifndef __ADS1292_H
#define __ADS1292_H
#include "main.h"
#define u8 unsigned char

//-----------------------------------------------------------------
// 位操作
//-----------------------------------------------------------------
#define PWDN_L      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET)
#define PWDN_H     	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET)
#define START_L     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET)
#define START_H     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET)
#define CS_L      	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET)
#define CS_H     		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET)

//-----------------------------------------------------------------
// ADS1292命令定义
//-----------------------------------------------------------------
// 系统命令
#define WAKEUP			0X02	// 从待机模式唤醒
#define STANDBY			0X04	// 进入待机模式
#define RESET				0X06	// 复位ADS1292R
#define START				0X08	// 启动或转换
#define STOP				0X0A	// 停止转换
#define OFFSETCAL		0X1A	// 通道偏移校准
// 数据读取命令
#define RDATAC			0X10	// 启用连续的数据读取模式,默认使用此模式
#define SDATAC			0X11	// 停止连续的数据读取模式
#define RDATA				0X12	// 通过命令读取数据;支持多种读回。
// 寄存器读取命令
#define	RREG				0X20	// 读取001r rrrr 000n nnnn  其中r rrrr是起始寄存器地址
#define WREG				0X40	// 写入010r rrrr 000n nnnn	其中r rrrr是起始寄存器地址 n nnnn=是要写的数据*/

// ADS1292R内部寄存器地址定义
#define ID					0X00	// ID控制寄存器
#define CONFIG1			0X01	// 配置寄存器1
#define CONFIG2			0X02	// 配置寄存器2
#define LOFF				0X03	// 导联脱落控制寄存器
#define CH1SET			0X04	// 通道1设置寄存器
#define CH2SET			0X05	// 通道2设置寄存器
#define RLD_SENS		0X06	// 右腿驱动选择寄存器
#define LOFF_SENS		0X07	// 导联脱落检测选择寄存器
#define LOFF_STAT		0X08	// 导联脱落检测状态寄存器
#define	RESP1				0X09	// 呼吸检测控制寄存器1
#define	RESP2				0X0A	// 呼吸检测控制寄存器2
#define	GPIO				0X0B	// GPIO控制寄存器			


/////////////////////////////////////////////////////////////////////////////////////////////
		//ADS1292R 具有呼吸和导联脱落检测的功能
		//ID
		#define	ADS1292_DEVICE	DEVICE_ID_ADS1292R		//注意1292芯片不能使用呼吸相关的功能，会导致数据不正常
		//1		CONFIG1	
		#define DATA_RATE  				DATA_RATE_125SPS		//由于时钟原因，采样率会有误差
		//2		CONFIG2		
		#define	PDB_LOFF_COMP 		PDB_LOFF_COMP_ON		//导联脱落比较器
		#define	PDB_REFBUF				PDB_REFBUF_ON				//基准电压缓冲器
		#define	VREF							VREF_242V						//设置内部参考电压
		#define	CLK_EN						CLK_EN_OFF					//使用内部时钟时，CLK引脚是否输出时钟信号
		#define	INT_TEST					INT_TEST_OFF				//是否打开内部测试信号
		//4	5		CHSET	
		#define	CNNNLE1_POWER			PD_ON								//通道电源
		#define	CNNNLE1_GAIN			GAIN_2						//增益
		#define	CNNNLE1_MUX				MUX_Normal_input		//输入方式
		#define	CNNNLE2_POWER			PD_ON							
		#define	CNNNLE2_GAIN			GAIN_12					
		#define	CNNNLE2_MUX				MUX_Normal_input		
		//6		RLD_SENS	
		#define	PDB_RLD						PDB_RLD_ON					//RLD缓冲电源		
		#define	RLD_LOFF_SENSE		RLD_LOFF_SENSE_OFF	//RLD导联脱落功能（测试的时候发现右腿导联脱落检测和右腿驱动输出好像不能同时工作）
		#define	RLD2N							RLD_CANNLE_ON				//通道的右腿驱动输出
		#define	RLD2P							RLD_CANNLE_ON
		#define	RLD1N							RLD_CANNLE_OFF			//呼吸通道不需要右腿驱动
		#define	RLD1P							RLD_CANNLE_OFF
		//7		LOFF_SENS	
		#define	FLIP2							FLIP2_OFF						//这个位用于控制导联脱落检测通道1的电流的方向
		#define	FLIP1							FLIP1_OFF						//这个位用于控制导联脱落检测通道2的电流的方向
		#define	LOFF2N						RLD_CANNLE_ON			//通道导联脱落检测功能
		#define	LOFF2P						RLD_CANNLE_ON
		#define	LOFF1N						RLD_CANNLE_OFF			//呼吸通道不需要导联脱落检测
		#define	LOFF1P						RLD_CANNLE_OFF
		//9		RSP1	
		#define	RESP_DEMOD_EN1		RESP_DEMOD_ON		//启用通道1解调电路
		#define	RESP_MOD_EN				RESP_MOD_ON			//启用通道1调制电路
		#define	RESP_PH						0X0D					//控制解调信号的相位	135																					
		#define	RESP_CTRL					RESP_CTRL_CLOCK_INTERNAL//内部呼吸内部时钟
		//10		RSP2	
		#define	CALIB							CALIB_OFF								//通道偏移校正 //运行过程中如果改变增益需要打开通道偏移校正，并执行 OFFSETCAL 指令
		#define	FREQ							FREQ_32K								//呼吸调制频率	64K/32K
		#define	RLDREF_INT				RLDREF_INT_INTERNALLY		//RLD参考 内部馈电  右腿驱动的参考电压可以选择内部产生(AVDD + AVSS) / 2，也可以选择外部提供

/////////////////////////////////////////////////////////////////////////////////////////////
//寄存器设置
//ID
#define	DEVICE_ID_ADS1292		0X53
#define	DEVICE_ID_ADS1292R	0X73
//CONFIG1
#define	DATA_RATE_125SPS	0X00	//采样率
#define	DATA_RATE_250SPS	0X01
#define	DATA_RATE_500SPS	0X02
#define	DATA_RATE_1kSPS		0X03
#define	DATA_RATE_2kSPS		0X04
#define	DATA_RATE_4kSPS		0X05
#define	DATA_RATE_8kSPS		0X06
//CONFIG2
#define	PDB_LOFF_COMP_OFF	0	//导联脱落比较器掉电（默认）
#define	PDB_LOFF_COMP_ON	1
#define	PDB_REFBUF_OFF	0		//基准电压缓冲器掉电（默认）
#define	PDB_REFBUF_ON		1
#define	VREF_242V		0	//内部参考电压2.42V（默认）
#define	VREF_4V			1	//内部参考电压4.033V
#define	CLK_EN_OFF	0	//振荡器时钟输出禁用（默认）
#define	CLK_EN_ON		1
#define	INT_TEST_OFF	0	//关闭内部测试信号（默认）
#define	INT_TEST_ON		1
//CHSET
#define	PD_ON			0	//通道正常运行（默认）	
#define	PD_OFF		1
#define	GAIN_6		0//增益6（默认）
#define	GAIN_1		1
#define	GAIN_2		2
#define	GAIN_3		3
#define	GAIN_4		4
#define	GAIN_8		5
#define	GAIN_12		6
#define	MUX_Normal_input 	0		//普通电极输入（默认）
#define	MUX_input_shorted 1		//输入短路
#define	MUX_Test_signal 	5		//测试信号输入
#define	MUX_RLD_DRP				6	
#define	MUX_RLD_DRM				7		
#define	MUX_RLD_DRPM			8	
#define	MUX_RSP_IN3P			9		//呼吸道通道1
//RLD_SENS
#define	PDB_RLD_OFF					0//（默认）
#define	PDB_RLD_ON					1
#define	RLD_LOFF_SENSE_OFF	0//（默认）
#define	RLD_LOFF_SENSE_ON		1
#define	RLD_CANNLE_OFF			0//（默认）
#define	RLD_CANNLE_ON				1
//LOFF_SENS
#define	FLIP2_OFF			0//（默认）
#define	FLIP2_ON			1							
#define	FLIP1_OFF			0//（默认）
#define	FLIP1_ON			1			
#define	LOFF_CANNLE_OFF		0//（默认）
#define	LOFF_CANNLE_ON		1	
//RSP1
#define	RESP_DEMOD_OFF	0	//（默认）
#define	RESP_DEMOD_ON		1	
#define	RESP_MOD_OFF 		0	//（默认）
#define	RESP_MOD_ON			1
#define	RESP_CTRL_CLOCK_INTERNAL	0
#define	RESP_CTRL_CLOCK_EXTERNAL	1
//RSP2
#define	CALIB_OFF		0//（默认）
#define	CALIB_ON		1		
#define	FREQ_32K		0	//（默认）
#define	FREQ_64K		1
#define	RLDREF_INT_EXTERN				0//外部馈电RLDREF
#define	RLDREF_INT_INTERNALLY		1//内部


typedef struct
{
		u8 Data_Rate; //ADC通道采样率  
}ADS1292_CONFIG1;
typedef struct
{
		u8	Pdb_Loff_Comp; 	//导联脱落比较器是否掉电
		u8	Pdb_Refbuf;			//内部参考缓冲器是否掉电
		u8	Vref;				//内部参考电压设置
		u8	Clk_EN;					//振荡器时钟输出设置
		u8	Int_Test;				//内部测试信号使能位	
}ADS1292_CONFIG2;
typedef struct
{
		u8  PD;			//通道断电？
		u8	GAIN;		//设置PGA增益
		u8	MUX;		//设置通道输入方式
}ADS1292_CHSET;
typedef struct
{
		u8	Pdb_Rld;				//该位决定RLD缓冲电源状态
		u8	Rld_Loff_Sense;	//该位使能RLD导联脱落检测功能
		u8	Rld2N;					//这个位控制通道2负输入 用于右腿驱动的输出
		u8	Rld2P;					//该位控制通道2正输入 用于右腿驱动的输出
		u8	Rld1N;					//这个位控制通道1负输入 用于右腿驱动的输出
		u8	Rld1P;					//该位控制通道1正输入 用于右腿驱动的输出
}ADS1292_RLD_SENS;
typedef struct
{
		u8	Flip2;//这个位用于控制导联脱落检测通道2的电流的方向
		u8	Flip1;//这个位控制用于导联脱落检测通道1的电流的方向
		u8	Loff2N;//该位控制通道2负输入端的导联脱落检测
		u8	Loff2P;//该位控制通道2正输入端的导联脱落检测
		u8	Loff1N;//该位控制通道1负输入端的导联脱落检测
		u8	Loff1P;//该位控制通道1正输入端的导联脱落检测
}ADS1292_LOFF_SENS;
typedef struct
{	
		u8	RESP_DemodEN;		//通道偏移校准
		u8	RESP_modEN;	//RLDREF信号源
		u8	RESP_ph;	
		u8	RESP_Ctrl;	
}ADS1292_RESP1;
typedef struct
{	
		u8	Calib;		//通道偏移校准
		u8	freq;			//频率 32k 64k
		u8	Rldref_Int;	//RLDREF信号源
}ADS1292_RESP2;

//-----------------------------------------------------------------
// 外部函数声明
//-----------------------------------------------------------------
void ADS1292_Init(void);
void ADS1292_PowerOnInit(void);
void ADS1292_Write_Reg(u8 addr, u8 data);
u8 ADS1292_Read_Reg(u8 addr);
void ADS1292_Read_Data(u8 *data);
void		ADS1292_SET_REGBUFF(void);//设置寄存器数组	
u8		ADS1292_WRITE_REGBUFF(void);//设置寄存器数组	
void ADS1292_WR_REGS(u8 reg,u8 len,u8 *data);

u8 ADS1292_SPI(u8 com);
		 
#endif

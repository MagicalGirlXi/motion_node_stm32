#ifndef __USER_ADS1118_H
#define __USER_ADS1118_H

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"

#define CS_H	HAL_GPIO_WritePin(GPIOC, ADS1118_CS_Pin, GPIO_PIN_SET)
#define CS_L	HAL_GPIO_WritePin(GPIOC, ADS1118_CS_Pin, GPIO_PIN_RESET)
#define SCLK_H	HAL_GPIO_WritePin(GPIOC, ADS1118_SCK_Pin, GPIO_PIN_SET)
#define SCLK_L	HAL_GPIO_WritePin(GPIOC, ADS1118_SCK_Pin, GPIO_PIN_RESET)
#define DIN_H	HAL_GPIO_WritePin(GPIOC, ADS1118_DIN_Pin, GPIO_PIN_SET)
#define DIN_L	HAL_GPIO_WritePin(GPIOC, ADS1118_DIN_Pin, GPIO_PIN_RESET)
#define DOUT	HAL_GPIO_ReadPin(ADS1118_OUT_Port, ADS1118_OUT_Pin)

/**单次转换启动**/
#define ADS1118_SS_NONE  0		// 无效 //ADS1118Init(ADS1118_SS_NONE, ADS1118_MODE_LX ,ADS1118_DR_128,ADS1118_PULL_UP_EN_E,0x01);
#define ADS1118_SS_ONCE  1		// 启动单次转换

/**输入多路复用器配置**/
#define ADS1118_MUX_01	0		// 000 = AINP 为 AIN0 且 AINN 为 AIN1（默认）
#define ADS1118_MUX_03	1		// 001 = AINP 为 AIN0 且 AINN 为 AIN3
#define ADS1118_MUX_13	2		// 010 = AINP 为 AIN1 且 AINN 为 AIN3
#define ADS1118_MUX_23	3		// 011 = AINP 为 AIN2 且 AINN 为 AIN3
#define ADS1118_MUX_0G	4		// 100 = AINP 为 AIN0 且 AINN 为 GND
#define ADS1118_MUX_1G	5		// 101 = AINP 为 AIN1 且 AINN 为 GND
#define ADS1118_MUX_2G	6		// 110 = AINP 为 AIN2 且 AINN 为 GND
#define ADS1118_MUX_3G	7		// 111 = AINP 为 AIN3 且 AINN 为 GND

/**可编程增益放大器配置**/
#define ADS1118_PGA_61  0		// 000 = FSR 为 ±6.144V
#define ADS1118_PGA_40  1		// 001 = FSR 为 ±4.096V
#define ADS1118_PGA_20  2		// 010 = FSR 为 ±2.048V（默认）
#define ADS1118_PGA_10  3		// 011 = FSR 为 ±1.024V
#define ADS1118_PGA_05  4		// 100 = FSR 为 ±0.512V
#define ADS1118_PGA_02  5		// 101 = FSR 为 ±0.256V

/**器件工作模式配置**/
#define ADS1118_MODE_LX  0		// 连续转换模式
#define ADS1118_MODE_DC  1 		// 断电并采用单次转换模式（默认）

/**数据传输速率**/
#define ADS1118_DR_8      0		// 000 = 8SPS
#define ADS1118_DR_16     1		// 001 = 16SPS
#define ADS1118_DR_32     2		// 010 = 32SPS
#define ADS1118_DR_64     3		// 011 = 64SPS
#define ADS1118_DR_128    4		// 100 = 128SPS（默认）
#define ADS1118_DR_250    5		// 101 = 250SPS
#define ADS1118_DR_475    6		// 110 = 475SPS
#define ADS1118_DR_860    7		// 111 = 860SPS

/**温度传感器模式**/
#define ADS1118_TS_MODE_ADC		0		// 0 = ADC 模式（默认）
#define ADS1118_TS_MODE_T			1		// 1 = 温度传感器模式

/**上拉使能**/
#define ADS1118_PULL_UP_EN_N	0		// 禁用 DOUT/DRDY 引脚的上拉电阻
#define ADS1118_PULL_UP_EN_E	1		// 使能 DOUT/DRDY 引脚的上拉电阻（默认）

/**控制数据是否写入配置寄存器**/
#define ADS1118_NOP_N	0		// 00 = 无效数据， 不更新配置寄存器内容
#define ADS1118_NOP_W	1		// 01 = 有效数据， 更新配置寄存器（默认）

/**保留**/
#define ADS1118_CNV_RDY_FL    1		// 始终写入 1h

/***************************定义ADS1118中的四个16位寄存器********************************/
typedef union
{
	struct
	{
		uint16_t CNV_RDY_FL	: 1 ;				// [0]		转换完成标志
		uint16_t NOP				: 2 ; 			// [1:2]	无操作
		uint16_t PULL_UP_EN	: 1 ; 			// [3]		上拉使能
		uint16_t TS_MODE		: 1 ; 			// [4]		温度传感器模式
		uint16_t DR					: 3 ;				// [7:5]	数据传输速率
		uint16_t MODE				: 1 ;				// [8]		设备运行模式
		uint16_t PGA				: 3 ;				// [11:9]	可编程增益放大器配置
		uint16_t MUX				: 3 ;				// [14:12]输入多路复用器配置
		uint16_t SS					: 1 ;				// [15]		操作状态或单次转换开始
	}ConfigDef_T ;
	uint16_t Bytes ;
}ConfigDef ;

typedef enum
{
	CS_0 = 0 ,
	CS_1
}
chipselect;

/***************************声明ADS1118中的四个16位寄存器********************************/
extern ConfigDef Config ;

extern float DP[8];						// 显示处理后的八通道数据
extern float Voltage;    			// 存储需要显示的测量电压值

uint16_t ReadAds1118(uint16_t Data);
void ADS1118Init(uint8_t ss,uint8_t mode ,uint8_t dr,uint8_t pue,uint8_t nop) ;
void GetData(uint8_t mux,uint8_t pga,uint8_t tsmode) ;

#ifdef __cplusplus
}
#endif

#endif 



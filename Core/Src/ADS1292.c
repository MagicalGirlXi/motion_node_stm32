//-----------------------------------------------------------------
// 程序描述:
//     ADS1292驱动程序
// 作    者: 凌智电子
// 开始日期: 2020-09-15
// 完成日期: 2020-09-15
// 当前版本: V1.0.0
// 历史版本:
// - V1.0.0: (2020-09-15) ADS1292驱动
// 调试工具: 凌智STM32核心开发板、LZE_ST_LINK2
// 说    明:
//
//-----------------------------------------------------------------
//-----------------------------------------------------------------
// 头文件包含
//-----------------------------------------------------------------
#include "ADS1292.h"	
#include "main.h"
#include "spi.h"
#include "stdio.h"
#include "rtthread.h"
#define delay_ms HAL_Delay
//#define GAP_VARIFY
u8 ADS1292_REG[12];		//ads1292寄存器数组
ADS1292_CONFIG1 	Ads1292_Config1		={DATA_RATE};																				//CONFIG1
ADS1292_CONFIG2 	Ads1292_Config2		={PDB_LOFF_COMP,PDB_REFBUF,VREF,CLK_EN,INT_TEST};		//CONFIG2
ADS1292_CHSET 		Ads1292_Ch1set		={CNNNLE1_POWER,CNNNLE1_GAIN,CNNNLE1_MUX};					//CH1SET
ADS1292_CHSET 		Ads1292_Ch2set		={CNNNLE2_POWER,CNNNLE2_GAIN,CNNNLE2_MUX};					//CH2SET
ADS1292_RLD_SENS	Ads1292_Rld_Sens	={PDB_RLD,RLD_LOFF_SENSE,RLD2N,RLD2P,RLD1N,RLD1P};	//RLD_SENS
ADS1292_LOFF_SENS	Ads1292_Loff_Sens	={FLIP2,FLIP1,LOFF2N,LOFF2P,LOFF1N,LOFF1P};					//LOFF_SENS
ADS1292_RESP1			Ads1292_Resp1			={RESP_DEMOD_EN1,RESP_MOD_EN,RESP_PH,RESP_CTRL};		//RSP1
ADS1292_RESP2			Ads1292_Resp2			={CALIB,FREQ,RLDREF_INT};			


u8 ADS1292_SPI(u8 com)
{	
		return SPI2_ReadWrite_Byte(com);
}
void ADS1292_WR_REGS(u8 reg,u8 len,u8 *data)
{
		u8 i;
		//ADS_CS=0;	
	CS_L;
		delay_us(100);
		ADS1292_SPI(reg);
		delay_us(100);
		ADS1292_SPI(len-1);
		if(reg&0x40) //写
		{
				for(i=0;i<len;i++)
				{
						delay_us(1000);		
						ADS1292_SPI(*data);
						data++;				
				}			
		}
		else //读		
		{
				for(i=0;i<len;i++)
				{
						delay_us(1000);		
						*data = ADS1292_SPI(0);
						data++;
				}
		}			
		delay_us(100);	
		CS_H;
}
u8 ADS1292_WRITE_REGBUFF(void)
{
		u8 i,res=0;
		u8 REG_Cache[12];	//存储寄存器数据
		ADS1292_SET_REGBUFF();//设置寄存器数组		
		ADS1292_WR_REGS(WREG|CONFIG1,11,ADS1292_REG+1);//数组变量写入寄存器
		delay_ms(10);		
		ADS1292_WR_REGS(RREG|ID,12,REG_Cache);//读寄存器
		delay_ms(10);	
		
	#ifdef DEBUG_ADS1292	
		printf("WRITE REG:\r\n");
		for(i=0;i<12;i++	)//要写的数据								
				printf("%d %x\r\n",i,ADS1292_REG[i]);	
		printf("READ REG:\r\n");
	#endif	
	
	
		for(i=0;i<12;i++	)	//检查寄存器	
		{						
				if(ADS1292_REG[i] != REG_Cache[i])
				{
						if(i!= 0 && i!=8 && i != 11)	//0 8 和11是ID 导联脱落和GPIO相关
								res=1;
						else
								continue;
				}					
			#ifdef DEBUG_ADS1292
				printf("%d %x\r\n",i,REG_Cache[i]); //读到的数据			
			#endif
		}	
			#ifdef DEBUG_ADS1292

			if(res == 0)
					printf("REG write success\r\n");
			else		
					printf("REG write err\r\n");		
			#endif
		return res;				
}

void ADS1292_SET_REGBUFF(void)
{
	ADS1292_REG[ID] =	ADS1292_DEVICE;//ID只读
	 
	ADS1292_REG[CONFIG1] =	0x00;		//0000 0aaa	[7] 0连续转换模式  [6:3] 必须为0 
	ADS1292_REG[CONFIG1] |=	Ads1292_Config1.Data_Rate;//[2:0] aaa 采样率设置采样率

	ADS1292_REG[CONFIG2] =	0x00;		//1abc d0e1	[7] 必须为1  [2] 必须为0  [0] 设置测试信号为1HZ、±1mV方波 
	ADS1292_REG[CONFIG2] |=	Ads1292_Config2.Pdb_Loff_Comp<<6;	//[6]a 导联脱落比较器是否掉电
	ADS1292_REG[CONFIG2] |=	Ads1292_Config2.Pdb_Refbuf<<5;		//[5]b 内部参考缓冲器是否掉电
	ADS1292_REG[CONFIG2] |=	Ads1292_Config2.Vref<<4;					//[4]c 内部参考电压设置，默认2.42V
	ADS1292_REG[CONFIG2] |=	Ads1292_Config2.Clk_EN<<3;				//[3]d CLK引脚输出时钟脉冲？
	ADS1292_REG[CONFIG2] |=	Ads1292_Config2.Int_Test<<1;			//[1]e 是否打开内部测试信号,
	ADS1292_REG[CONFIG2] |=	0x81;//设置默认位
	
	ADS1292_REG[LOFF] =	0x10;//[7:5]	设置导联脱落比较器阈值 [4]	必须为1 		[3:2] 导联脱落电流幅值		[1]	必须为0	[0]	导联脱落检测方式 0 DC 1 AC 

	ADS1292_REG[CH1SET] =	0x00;	 //abbb cccc
	ADS1292_REG[CH1SET] |=Ads1292_Ch1set.PD<<7;		//[7]  a 		通道1断电？
	ADS1292_REG[CH1SET] |=Ads1292_Ch1set.GAIN<<4;	//[6:4]bbb	设置PGA增益
	ADS1292_REG[CH1SET] |=Ads1292_Ch1set.MUX;			//[3:0]cccc	设置通道1输入方式

	ADS1292_REG[CH2SET] =	0x00;	//abbb cccc
	ADS1292_REG[CH2SET] |=Ads1292_Ch2set.PD<<7;		//[7]  a 		通道2断电？
	ADS1292_REG[CH2SET] |=Ads1292_Ch2set.GAIN<<4;	//[6:4]bbb	设置PGA增益
	ADS1292_REG[CH2SET] |=Ads1292_Ch2set.MUX;			//[3:0]cccc	设置通道2输入方式
	
	ADS1292_REG[RLD_SENS] = 0X00; //11ab cdef	[7:6] 11 PGA斩波频率	fMOD/4 
	ADS1292_REG[RLD_SENS] |=Ads1292_Rld_Sens.Pdb_Rld<<5;					//[5]a	该位决定RLD缓冲电源状态
	ADS1292_REG[RLD_SENS] |=Ads1292_Rld_Sens.Rld_Loff_Sense<<4;	//[4]b	该位使能RLD导联脱落检测功能
	ADS1292_REG[RLD_SENS] |=Ads1292_Rld_Sens.Rld2N<<3;						//[3]c	这个位控制通道2负输入	用于右腿驱动的输出
	ADS1292_REG[RLD_SENS] |=Ads1292_Rld_Sens.Rld2P<<2;						//[2]d	该位控制通道2正输入		用于右腿驱动的输出
	ADS1292_REG[RLD_SENS] |=Ads1292_Rld_Sens.Rld1N<<1;						//[1]e	这个位控制通道1负输入	用于右腿驱动的输出
	ADS1292_REG[RLD_SENS] |=Ads1292_Rld_Sens.Rld1P;							//[0]f	该位控制通道1正输入		用于右腿驱动的输出	
	ADS1292_REG[RLD_SENS] |=	0xc0;//设置默认位

	ADS1292_REG[LOFF_SENS] = 0X00;  //00ab cdef	[7:6] 必须为0
	ADS1292_REG[LOFF_SENS] |=Ads1292_Loff_Sens.Flip2<<5;		//[5]a	这个位用于控制导联脱落检测通道2的电流的方向
	ADS1292_REG[LOFF_SENS] |=Ads1292_Loff_Sens.Flip1<<4;		//[4]b	这个位控制用于导联脱落检测通道1的电流的方向
	ADS1292_REG[LOFF_SENS] |=Ads1292_Loff_Sens.Loff2N<<3;	//[3]c	该位控制通道2负输入端的导联脱落检测
	ADS1292_REG[LOFF_SENS] |=Ads1292_Loff_Sens.Loff2P<<2;	//[2]d	该位控制通道2正输入端的导联脱落检测
	ADS1292_REG[LOFF_SENS] |=Ads1292_Loff_Sens.Loff1N<<1;	//[1]e	该位控制通道1负输入端的导联脱落检测
	ADS1292_REG[LOFF_SENS] |=Ads1292_Loff_Sens.Loff1P;			//[0]f	该位控制通道1正输入端的导联脱落检测
	
	ADS1292_REG[LOFF_STAT] =	0x00;		//[6]0 设置fCLK和fMOD之间的模分频比 fCLK=fMOD/4  [4:0]只读，导联脱落和电极连接状态
	
	ADS1292_REG[RESP1] = 0X00;//abcc cc1d
	ADS1292_REG[RESP1] |=Ads1292_Resp1.RESP_DemodEN<<7;//[7]a		这个位启用和禁用通道1上的解调电路		
	ADS1292_REG[RESP1] |=Ads1292_Resp1.RESP_modEN<<6;	//[6]b		这个位启用和禁用通道1上的调制电路	
	ADS1292_REG[RESP1] |=Ads1292_Resp1.RESP_ph<<2;			//[5:2]c	这些位控制呼吸解调控制信号的相位	
	ADS1292_REG[RESP1] |=Ads1292_Resp1.RESP_Ctrl;			//[0]d		这个位设置呼吸回路的模式
	ADS1292_REG[RESP1] |=	0x02;//设置默认位	
	
	ADS1292_REG[RESP2] = 0x00; //a000 0bc1	[6:3]必须为0 [0]必须为1
	ADS1292_REG[RESP2] |=	Ads1292_Resp2.Calib<<7;				//[7]a 启动通道偏移校正？
	ADS1292_REG[RESP2] |=	Ads1292_Resp2.freq<<2;				//[2]b 呼吸频率设置
	ADS1292_REG[RESP2] |=	Ads1292_Resp2.Rldref_Int<<1;	//[1]c RLDREF信号源外部馈电？
	ADS1292_REG[RESP2] |= 0X01;//设置默认位	
 
	ADS1292_REG[GPIO] =	0x0C;			//GPIO设为输入		[7:4]必须为0	 [3:2]11 GPIO为输入 [1:0] 设置输入时，指示引脚电平，设置输出时控制引脚电平
}
//-----------------------------------------------------------------
// void GPIO_LED_Configuration(void)
//-----------------------------------------------------------------
//
// 函数功能: ADS1292引脚初始化
// 入口参数: 无
// 返 回 值: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void ADS1292_Init(void)
{
  /*GPIO_InitTypeDef GPIO_InitStructure;

  __HAL_RCC_GPIOA_CLK_ENABLE();

	// ADS1292_DRDY -> PA3
//	GPIO_InitStructure.Pin   = GPIO_PIN_3;					// 配置ADS1292_DRDY
//  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;		// 高速
//  GPIO_InitStructure.Mode  = GPIO_Mode_IPU;			// 上拉输入
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);							// 初始化
  
	// ADS1292_PWDN  -> PA0
	// ADS1292_START -> PA1
  // ADS1292_CS  	 -> PA2
  GPIO_InitStructure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;		// 高速
  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;		// 推挽输出
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);							// 初始化
	
	SPI1_Init(); // SPI初始化*/
	
}




//-----------------------------------------------------------------
// void ADS1292_Write_Reg(u8 com, u8 data)
//-----------------------------------------------------------------
//
// 函数功能: 对ADS1292的内部寄存器进行写操作
// 入口参数: 无
// 返 回 值: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void ADS1292_Write_Reg(u8 addr, u8 data)
{
	CS_L;				// 片选拉低
  SPI2_Send_Byte(addr);	// 包含命令操作码和寄存器地址
  delay_us(10);
  SPI2_Send_Byte(0x00);	// 要读取的寄存器数+1
  delay_us(10);
  SPI2_Send_Byte(data);	// 写入的数据
	delay_us(10);
	CS_H;				// 片选置高
}
//写命令
void ADS1292_Send_CMD(u8 data)
{
		CS_L;
		delay_us(100);
		ADS1292_SPI(data);		
		delay_us(100);	
		CS_H;
}
//-----------------------------------------------------------------
// u8 ADS1292_Read_Reg(u8 addr)
//-----------------------------------------------------------------
//
// 函数功能: 对ADS1292的内部寄存器进行读操作
// 入口参数: 无
// 返 回 值: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
u8 ADS1292_Read_Reg(u8 addr)
{
  u8 Rxdata;
	CS_L;					// 片选拉低
  SPI2_Send_Byte(addr); 			// 包含命令操作码和寄存器地址
  delay_us(10);
  SPI2_Send_Byte(0x00); 			// 要读取的寄存器数+1
  delay_us(10);
  Rxdata = SPI2_Read_Byte(); 	// 读取的数据
	delay_us(10);
	CS_H;					// 片选置高
  return Rxdata;
}

//-----------------------------------------------------------------
// void ADS1292_PowerOnInit(void)
//-----------------------------------------------------------------
//
// 函数功能: ADS1292上电复位
// 入口参数: 无
// 返 回 值: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void ADS1292_PowerOnInit(void)
{
	u8 device_id;

  START_H;
  CS_H;
  PWDN_L; // 进入掉电模式
  delay_ms(1000);
  PWDN_H; // 退出掉电模式
  delay_ms(1000);   // 等待稳定
  PWDN_L; // 发出复位脉冲
  delay_us(10);
  PWDN_H;
  delay_ms(1000); // 等待稳定，可以开始使用ADS1292R
	
	START_L;
	CS_L;
	delay_us(10);
  SPI2_Send_Byte(SDATAC); // 发送停止连续读取数据命令
	delay_us(10);
	CS_H;
	
	// ADS1292_Write_Reg(WREG | ID, 0x74);
	// 获取芯片ID
	device_id = ADS1292_Read_Reg(RREG | ID);
	#ifndef GAP_VARIFY
	while(device_id != 0x73)
	{
		device_id = ADS1292_Read_Reg(RREG | ID);
		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_12);
		delay_ms(1000);
	}
	#endif
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,GPIO_PIN_SET);
	
	delay_us(100);
  ADS1292_Write_Reg(WREG | CONFIG2,  0XE0); // 使用内部参考电压
  delay_ms(100);                            // 等待内部参考电压稳定
  ADS1292_Write_Reg(WREG | CONFIG1,  0X01); // 设置转换速率为250SPS
  delay_us(100);
  ADS1292_Write_Reg(WREG | LOFF,     0XF0);	// 该寄存器配置引出检测操作
  delay_us(100);
  ADS1292_Write_Reg(WREG | CH1SET,   0X60); // 增益12，连接到电极
  delay_us(100);
  ADS1292_Write_Reg(WREG | CH2SET,   0X00); // 增益6，连接到电极
  delay_us(100);
  ADS1292_Write_Reg(WREG | RLD_SENS, 0xEF);
  delay_us(100);
  ADS1292_Write_Reg(WREG | LOFF_SENS,0x0F);
  delay_us(100);
	ADS1292_Write_Reg(WREG | LOFF_STAT,0x00);
  delay_us(100);
  ADS1292_Write_Reg(WREG | RESP1,    0xEA); // 开启呼吸检测（ADS1292R特有）
  delay_us(100);
  ADS1292_Write_Reg(WREG | RESP2,    0x03);
  delay_us(100);
  ADS1292_Write_Reg(WREG | GPIO,     0x0C);
  delay_us(100);
}

//-----------------------------------------------------------------
// u8 ADS1292_Read_Data(u8 addr)
//-----------------------------------------------------------------
//
// 函数功能: 读取ADS1292的数据
// 入口参数: 无
// 返 回 值: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void ADS1292_Read_Data(u8 *data)
{
  u8 i;
	CS_L;
	delay_us(10);
  SPI2_Send_Byte(RDATAC);		// 发送启动连续读取数据命令
  delay_us(10);
	CS_H;						
  START_H; 				// 启动转换
  while (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == 1);	// 等待DRDY信号拉低
  CS_L;
	delay_us(10);
  for (i = 0; i < 9; i++)		// 连续读取9个数据
  {
    *data = SPI2_Read_Byte();
    data++;
  }
  START_L;				// 停止转换
  SPI2_Send_Byte(SDATAC);		// 发送停止连续读取数据命令
	delay_us(10);
	CS_H;
}
//-----------------------------------------------------------------
// End Of File
//----------------------------------------------------------------- 
u8 ADS1292_Single_Test(void) //注意1292R开了呼吸解调，会对通道一的内部测试信号波形造成影响，这里只参考通道2即可，1292不受影响
{
		u8 res=0;
		Ads1292_Config2.Int_Test = INT_TEST_ON;//打开内部测试信号
		Ads1292_Ch1set.MUX=MUX_Test_signal;//测试信号输入	
		Ads1292_Ch2set.MUX=MUX_Test_signal;//测试信号输入	
		
		if(ADS1292_WRITE_REGBUFF())//写入寄存器
				res=1;	
		delay_ms(10);			
		return res;		
}
//设置内部噪声测试
u8 ADS1292_Noise_Test(void)
{
		u8 res=0;
		Ads1292_Config2.Int_Test = INT_TEST_OFF;//关内部测试信号
		Ads1292_Ch1set.MUX = MUX_input_shorted;//输入短路	
		Ads1292_Ch2set.MUX = MUX_input_shorted;//输入短路	

		if(ADS1292_WRITE_REGBUFF())//写入寄存器
				res=1;	
		delay_ms(10);			
		return res;			
}

//正常信号采集模式
u8 ADS1292_Single_Read(void)
{
		u8 res=0;
		Ads1292_Config2.Int_Test = INT_TEST_OFF;//关内部测试信号
		Ads1292_Ch1set.MUX = MUX_Normal_input;//普通电极输入
		Ads1292_Ch2set.MUX = MUX_Normal_input;//普通电极输入
	
		if(ADS1292_WRITE_REGBUFF())//写入寄存器
				res=1;
		delay_ms(10);		
		return res;		
}	

//配置ads1292采集方式
u8 Set_ADS1292_Collect(u8 mode)
{
		u8 res;
		
		delay_ms(10);	
		switch(mode)//设置采集方式
		{
				case 0:
					res =ADS1292_Single_Read();												
				break;
				case 1:
					res =ADS1292_Single_Test();											
				break;
				case 2:
					res =ADS1292_Noise_Test();											
				break;
		}		
		if(res)return 1;//寄存器设置失败		
		ADS1292_Send_CMD(RDATAC); //启动连续模式
		delay_ms(10);		
		ADS1292_Send_CMD(START);	//发送开始数据转换（等效于拉高START引脚）	
		delay_ms(10);		
		return 0;
}


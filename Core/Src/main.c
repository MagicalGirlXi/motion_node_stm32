/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "rtthread.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <math.h>
#include "dist.h"
#include "func.h"
#include "ads1118.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "ads1292.h"

#define TEMP_A (-1.809628)
#define TEMP_B (-3.325395)
#define TEMP_C (-181.4103)
#define TEMP_D (205.5894)
#define TEMP_A_LIN (-193)
#define TEMP_B_LIN 212.009
#define CACHE_LEN 300
#define SAMPLE_SPS 70
//心率相关定义
#define Samples_Number  1    				// 采样点数
#define Block_Size      1     			// 调用一次arm_fir_f32处理的采样点个数
#define NumTaps        	129     		// 滤波器系数个数

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

UART_HandleTypeDef *cur_uart;
#ifdef __GNUC_
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(cur_uart, (uint8_t *)&ch, 1, 0xFFFF);
 
  return ch;
}
#define SystemClock				72				// 系统时钟（单位：MHz）
#define FAC_us				SystemClock/4	// 延时因子

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static struct rt_thread led_thread;
static char led_thread_stack[1024];
static void led_thread_entry(void *parameter);

static struct rt_thread upload_thread;
static char upload_thread_stack[8000];
static void upload_thread_entry(void *parameter);

static struct rt_thread display_thread;
static char display_thread_stack[1024];
static void display_thread_entry(void *parameter);

static struct rt_thread receive_thread;
static char receive_thread_stack[1024];
static void receive_thread_entry(void *parameter);

int cur_time = 0;
int wifi_ready = 0;
unsigned long step = 0;
unsigned long dist = 0;
short accx, accy, accz;
float mpu_temper;

float hu_temper;
float buffer_temper;
//心率相关变量
int ch1_data;	// 通道1的数据-呼吸测量数据
int ch2_data;	// 通道2的数据-心电图数据
uint8_t flog;			// 触发中断标志位
uint16_t point_cnt;									// 两个峰值之间的采集点数，用于计算心率
	uint8_t rd_data[10];
uint32_t blockSize = Block_Size;											// 调用一次arm_fir_f32处理的采样点个数
uint32_t numBlocks = Samples_Number/Block_Size;       // 需要调用arm_fir_f32的次数
int ch1_point_200[200];
uint16_t ch1_index;
int ch2_point_200[200];
uint16_t ch2_index;
static float32_t Input_data1; 					// 输入缓冲区
int data_max=0;
int data_max_temp=0;
int data_min=0xFFFFFFFF;
int data_min_temp=0;
static float32_t Output_data1;         // 输出缓冲区
static float32_t firState1[Block_Size + NumTaps - 1]; 	// 状态缓存，大小numTaps + blockSize - 1
static float32_t Input_data2; 					// 输入缓冲区
static float32_t Output_data2;         // 输出缓冲区
static float32_t firState2[Block_Size + NumTaps - 1]; 	// 状态缓存，大小numTaps + blockSize - 1

	arm_fir_instance_f32 S1;
	arm_fir_instance_f32 S2;
	int display_mid=128000;
	int display_amp=5000;
	long display_avr_sum=0;
	long display_avr_cnt=0;
float bpm_cache[1000];                   //计算心率的数据缓存
int bpm_cache_index = 0;
int pn_npks;                           //心率峰值检测函数峰值数量
int pn_locs[100];                       //心率峰值检测函数输出峰值点	
int BPM;
const float32_t BPF_5Hz_40Hz[NumTaps]  = {
  3.523997657e-05,0.0002562592272,0.0005757701583,0.0008397826459, 0.000908970891,
  0.0007304374012,0.0003793779761,4.222582356e-05,-6.521392788e-05,0.0001839015895,
  0.0007320778677, 0.001328663086, 0.001635892317, 0.001413777587,0.0006883906899,
  -0.0002056905651,-0.0007648666506,-0.0005919140531,0.0003351111372, 0.001569915912,
   0.002375603188, 0.002117323689,0.0006689901347,-0.001414557919,-0.003109993879,
  -0.003462586319, -0.00217742566,8.629632794e-05, 0.001947802957, 0.002011778764,
  -0.0002987752669,-0.004264956806, -0.00809297245,-0.009811084718,-0.008411717601,
  -0.004596390296,-0.0006214127061,0.0007985962438,-0.001978532877,-0.008395017125,
   -0.01568987407, -0.02018531598, -0.01929843985, -0.01321159769,-0.005181713495,
  -0.0001112028476,-0.001950757345, -0.01125541423,  -0.0243169684, -0.03460548073,
   -0.03605531529, -0.02662901953, -0.01020727865, 0.004513713531, 0.008002913557,
  -0.004921500571, -0.03125274926, -0.05950148031, -0.07363011688, -0.05986980721,
   -0.01351031102,  0.05752891302,   0.1343045086,   0.1933406889,   0.2154731899,
     0.1933406889,   0.1343045086,  0.05752891302, -0.01351031102, -0.05986980721,
   -0.07363011688, -0.05950148031, -0.03125274926,-0.004921500571, 0.008002913557,
   0.004513713531, -0.01020727865, -0.02662901953, -0.03605531529, -0.03460548073,
    -0.0243169684, -0.01125541423,-0.001950757345,-0.0001112028476,-0.005181713495,
   -0.01321159769, -0.01929843985, -0.02018531598, -0.01568987407,-0.008395017125,
  -0.001978532877,0.0007985962438,-0.0006214127061,-0.004596390296,-0.008411717601,
  -0.009811084718, -0.00809297245,-0.004264956806,-0.0002987752669, 0.002011778764,
   0.001947802957,8.629632794e-05, -0.00217742566,-0.003462586319,-0.003109993879,
  -0.001414557919,0.0006689901347, 0.002117323689, 0.002375603188, 0.001569915912,
  0.0003351111372,-0.0005919140531,-0.0007648666506,-0.0002056905651,0.0006883906899,
   0.001413777587, 0.001635892317, 0.001328663086,0.0007320778677,0.0001839015895,
  -6.521392788e-05,4.222582356e-05,0.0003793779761,0.0007304374012, 0.000908970891,
  0.0008397826459,0.0005757701583,0.0002562592272,3.523997657e-05
	};

// 呼吸波低通滤波器系数：采样频率为250Hz，截止频率为2Hz 通过filterDesigner获取
const float32_t LPF_2Hz[NumTaps]  = {
  -0.0004293085367,-0.0004170549801,-0.0004080719373,-0.0004015014856,-0.0003963182389,
  -0.000391335343,-0.0003852125083,-0.0003764661378,-0.0003634814057,-0.0003445262846,
  -0.0003177672043,-0.0002812864841,-0.0002331012802,-0.0001711835939,-9.348169988e-05,
  2.057720394e-06,0.0001174666468, 0.000254732382,0.0004157739459,0.0006024184986,
   0.000816378044, 0.001059226575, 0.001332378131,  0.00163706555,  0.00197432097,
   0.002344956854, 0.002749550389, 0.003188427072, 0.003661649302, 0.004169005435,
    0.00471000094, 0.005283853505, 0.005889489781, 0.006525543984, 0.007190360688,
   0.007882000878, 0.008598247543, 0.009336617775,  0.01009437256,  0.01086853724,
    0.01165591553,  0.01245311089,  0.01325654797,  0.01406249963,  0.01486710832,
    0.01566641964,  0.01645640284,  0.01723298989,  0.01799209975,  0.01872966997,
    0.01944169216,  0.02012423798,  0.02077349275,  0.02138578519,  0.02195761539,
     0.0224856846,  0.02296692133,  0.02339850739,  0.02377789468,  0.02410283685,
    0.02437139489,  0.02458196506,  0.02473328263,  0.02482444048,  0.02485488541,
    0.02482444048,  0.02473328263,  0.02458196506,  0.02437139489,  0.02410283685,
    0.02377789468,  0.02339850739,  0.02296692133,   0.0224856846,  0.02195761539,
    0.02138578519,  0.02077349275,  0.02012423798,  0.01944169216,  0.01872966997,
    0.01799209975,  0.01723298989,  0.01645640284,  0.01566641964,  0.01486710832,
    0.01406249963,  0.01325654797,  0.01245311089,  0.01165591553,  0.01086853724,
    0.01009437256, 0.009336617775, 0.008598247543, 0.007882000878, 0.007190360688,
   0.006525543984, 0.005889489781, 0.005283853505,  0.00471000094, 0.004169005435,
   0.003661649302, 0.003188427072, 0.002749550389, 0.002344956854,  0.00197432097,
    0.00163706555, 0.001332378131, 0.001059226575, 0.000816378044,0.0006024184986,
  0.0004157739459, 0.000254732382,0.0001174666468,2.057720394e-06,-9.348169988e-05,
  -0.0001711835939,-0.0002331012802,-0.0002812864841,-0.0003177672043,-0.0003445262846,
  -0.0003634814057,-0.0003764661378,-0.0003852125083,-0.000391335343,-0.0003963182389,
  -0.0004015014856,-0.0004080719373,-0.0004170549801,-0.0004293085367
	};

//Acceleration A[];
//Velocity V[];
//Position P[];
extern float DP[8];						// 显示处理后的八通道数据
extern float Voltage;    			// 存储需要显示的测量电压值
	int out_ch2;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{__disable_irq() ;
	if(GPIO_Pin==GPIO_PIN_11)
	{
		u8 j;
		u8 read_data[9];
		
		for (j = 0; j < 9; j++)		// 连续读取9个数据
		{
			read_data[j] = SPI2_ReadWrite_Byte(0xFF);
		}
			ch1_data=0;
			ch2_data=0;
			ch1_data = (int)(((read_data[3] << 16) | (read_data[4] << 8) | (read_data[5] & 0xff)));  
			ch2_data = (int)(((read_data[6] << 16) | (read_data[7] << 8) | (read_data[8] & 0xff)));  
			ch1_point_200[ch1_index++] = ch1_data;
			if (ch1_index %200==0)
			{
				ch1_index%=200;
			}
			ch2_point_200[ch2_index++] = ch2_data;
			if (ch2_index %200==0)
			{
				ch2_index%=200;
			}
		flog=1;
		point_cnt++;out_ch2 = ch2_data;
			Input_data2=(float32_t)(ch2_data^0x800000);
			arm_fir_f32(&S2, &Input_data2, &Output_data2, blockSize);
		display_avr_sum+=Output_data2;
			if (point_cnt % 100 == 0)
			{
				point_cnt = 0;
				display_mid = display_avr_sum/100;display_avr_sum=0;
				
			}
	}__enable_irq() ;
}

void maxim_peaks_above_min_height(int *pn_locs, int *pn_npks, float  *pn_x, int n_size, int n_min_height)
{
    int i = 1, n_width;
    *pn_npks = 0;
    
    while (i < n_size-1){
        if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i-1]){            // find left edge of potential peaks
            n_width = 1;
            while (i+n_width < n_size && pn_x[i] == pn_x[i+n_width])    // find flat peaks
                n_width++;
            if (pn_x[i] > pn_x[i+n_width]){                     // find right edge of peaks
                pn_locs[(*pn_npks)++] = i;        
                // for flat peaks, peak location is left edge
                i += n_width+1;
            }
            else
                i += n_width;
        }
        else
            i++;
    }
}


float bpm_calculate(int *loc,int size)
{
	float bpm=0;
	bpm = 60.0/((loc[size-1]-loc[0])/(size-1))*SAMPLE_SPS;                              //计算心率 算法:两峰之间点数*采样率
	
	return bpm;
}

void delay_us(uint32_t nus)
{		
	uint32_t i;
	while(nus--)
	{
		i=FAC_us-2;
		while(i--);
	}
}

void sendend()
{
	uint8_t cc = 0xff;
	HAL_UART_Transmit(cur_uart,  &cc, 1, 0xFFFF);
	HAL_UART_Transmit(cur_uart,  &cc, 1, 0xFFFF);
	HAL_UART_Transmit(cur_uart,  &cc, 1, 0xFFFF);
}

static void led_thread_entry(void *parameter)
{
			ADS1118Init(ADS1118_SS_NONE, ADS1118_MODE_LX ,ADS1118_DR_8,ADS1118_PULL_UP_EN_E,0x01);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    while(1)
    {
			//GetData(ADS1118_MUX_0G,ADS1118_PGA_20,ADS1118_TS_MODE_ADC); 
			
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
        rt_thread_mdelay(500);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
        rt_thread_mdelay(500);
			
			
			
    }
}

static void upload_thread_entry(void *parameter)
{
    //rt_thread_mdelay(2000);
	
		
		
		
		
		rt_enter_critical();
		cur_uart = &huart1;
		printf("AT+RST\r\n");
		rt_exit_critical();
	
    rt_thread_mdelay(10000);
	
		/*rt_enter_critical();
		cur_uart = &huart1;
		printf("AT+CWJAP=\"magicgirlxi\",\"zxjnz3cw74\"\r\n");
		rt_exit_critical();
    rt_thread_mdelay(6000);*/
	
	
		rt_enter_critical();
		cur_uart = &huart1;
		printf("AT+CIPMODE=1\r\n");
		rt_exit_critical();
	
    rt_thread_mdelay(300);
	
		rt_enter_critical();
		cur_uart = &huart1;
		printf("AT+CIPSTART=\"TCP\",\"192.168.137.1\",9908\r\n");
		rt_exit_critical();
	
    rt_thread_mdelay(500);
	
		rt_enter_critical();
		cur_uart = &huart1;
		printf("AT+CIPSEND\r\n");
		rt_exit_critical();
	
    rt_thread_mdelay(300);
		
		rt_enter_critical();
		cur_uart = &huart1;
		printf("CONNECT SUCCEESS!\r\n");
		rt_exit_critical();
		
				rt_enter_critical();
				cur_uart = &huart2;
        printf("dist.txt=\"OK\"");
				sendend();
				rt_exit_critical();
				
				
				rt_enter_critical();
				CS_L;
				delay_us(10);
				SPI2_ReadWrite_Byte(RDATAC);		// 发送启动连续读取数据命令
				delay_us(10);
				CS_H;						
				START_H; 				// 启动转换
				CS_L;
				rt_exit_critical();
				wifi_ready = 1;
				
		rt_enter_critical();
		cur_uart = &huart1;
		printf("START CONVERT\r\n");
		rt_exit_critical();
	
    rt_thread_mdelay(300);
    while(1)
    {
			/*rt_enter_critical();
				cur_uart = &huart1;
        printf("cl");
				//sendend();
			rt_exit_critical();
			*/
			
    rt_thread_mdelay(1000);
			
			
    }
}
static void receive_thread_entry(void *parameter)
{
		int len;
    while(1)
    {
			/*rt_enter_critical();
				cur_uart = &huart1;
        printf("cl");
				//sendend();
			rt_exit_critical();
			*/
			if(USART_RX_STA&0x8000)
			{					   
				len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
				//process_command(USART_RX_BUF, len);
				
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
				rt_enter_critical();
				cur_uart = &huart1;
        printf("press!");
				//sendend();
				rt_exit_critical();
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
				
				USART_RX_STA=0;
			}
		
      rt_thread_mdelay(500);
			
    }
}

//extern unsigned char DMP_INT_FLAG;
extern unsigned char rev_flag;

static struct rt_thread mpu6050_thread;
static char mpu6050_thread_stack[5000];
static void mpu6050_thread_entry(void *parameter);

static void mpu6050_thread_entry(void *parameter)
{
	while(wifi_ready == 0)
	{
      rt_thread_mdelay(1000);
	}
		rt_enter_critical();
		cur_uart = &huart1;
    printf("Initializing Step Calcer\r\n");
		rt_exit_critical();
	
	
    //MPU_6050_Init();
		//mpu_dmp_init();
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
    //DMP_INT_FLAG=mpu_dmp_init();
	
		//rt_enter_critical();
		//cur_uart = &huart1;
    //printf("DMP_INT_FLAG %d \n",DMP_INT_FLAG);
		//rt_exit_critical();

    while(1)
    {
			//read_acc(&accx,&accy,&accz,&mpu_temper);
			if (dmp_get_pedometer_step_count(&step) == 0)
			{
				/*rt_enter_critical();
				cur_uart = &huart1;
				printf("step is %ld\r\n",step);
				rt_exit_critical();
				*/
				dist = step*77;
				dist/=100;
				rt_enter_critical();
				cur_uart = &huart2;
				printf("step.txt=\"%ld\"",step);//串口屏显示步数
				sendend();
				printf("temp.txt=\"%f\"",hu_temper);//串口屏显示温度
				sendend();
				printf("beat.txt=\"%d\"",BPM);//串口屏显示心率
				sendend();																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																										
				printf("dist.txt=\"%ld\"",dist );//串口屏显示距离
				sendend();
				rt_exit_critical();
			}
			buffer_temper = DP[0];//*DP[0]*DP[0];
			hu_temper = (-1.809628)*buffer_temper*buffer_temper*buffer_temper;//
			hu_temper = hu_temper+ (-3.325395)*buffer_temper*buffer_temper;// 
			hu_temper =  hu_temper+ (-181.4103)*buffer_temper ;//  
			hu_temper += (205.5894);//
			
      rt_thread_mdelay(50);
    }
}


static void display_thread_entry(void *parameter)
{
	while(wifi_ready == 0)
	{
      rt_thread_mdelay(1000);
	}
		int loop_i;
    while(1)
    {
			
			for (loop_i = 0;loop_i < 180;loop_i++)
			{
				rt_enter_critical();
				cur_uart = &huart2;
				printf("add 1,0,%d", ((int)Output_data2-display_mid+display_amp/2)*180/display_amp);
				sendend();
				rt_exit_critical();
				rt_thread_mdelay(8);
				if (flog == 1)
				{
		rt_enter_critical();
				cur_uart = &huart1;
				printf("%.0f,%.2f,%ld,%ld,%d\r\n",Output_data2,hu_temper,step,dist,BPM);//发送到上位机
		rt_exit_critical();
					bpm_cache[bpm_cache_index++] = Output_data2;
					if (bpm_cache_index > CACHE_LEN)
					{
						maxim_peaks_above_min_height(pn_locs,&pn_npks,bpm_cache,CACHE_LEN,display_mid+1000); 
						
						BPM = bpm_calculate(pn_locs,pn_npks); 
						bpm_cache_index = 0;
					}
					flog = 0;
				}
			}
			//rt_thread_mdelay(500);
			
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	
	flog=0;
	ADS1292_PowerOnInit();
	arm_fir_init_f32(&S1, NumTaps, (float32_t *)LPF_2Hz, firState1, blockSize);
	// 初始化结构体S2, 心电波使用
	arm_fir_init_f32(&S2, NumTaps, (float32_t *)BPF_5Hz_40Hz, firState2, blockSize);
	HAL_Delay(100);
	ADS1292_Single_Read();
	HAL_Delay(100);
	
	
	rt_err_t rst;
	rst = rt_thread_init(&led_thread,
											 (const char *)"ledShine",
											 led_thread_entry,
											 RT_NULL,
											 &led_thread_stack[0],
											 sizeof(led_thread_stack),
											 RT_THREAD_PRIORITY_MAX-10,
											 30);
	if(rst == RT_EOK)
	{
			rt_thread_startup(&led_thread);
	}
	rst = rt_thread_init(&upload_thread,
											 (const char *)"uploadData",
											 upload_thread_entry,
											 RT_NULL,
											 &upload_thread_stack[0],
											 sizeof(upload_thread_stack),
											 RT_THREAD_PRIORITY_MAX-5,
											 1);
	if(rst == RT_EOK)
	{
			rt_thread_startup(&upload_thread);
	}
	rst = rt_thread_init(&display_thread,
											 (const char *)"displayData",
											 display_thread_entry,
											 RT_NULL,
											 &display_thread_stack[0],
											 sizeof(display_thread_stack),
											 RT_THREAD_PRIORITY_MAX-6,
											 1);
	if(rst == RT_EOK)
	{
			rt_thread_startup(&display_thread);
	}
	rst = rt_thread_init(&mpu6050_thread,
											 (const char *)"mpu6050",
											 mpu6050_thread_entry,
											 RT_NULL,
											 &mpu6050_thread_stack[0],
											 sizeof(mpu6050_thread_stack),
											 RT_THREAD_PRIORITY_MAX-10,
											 20);
	if(rst == RT_EOK)
	{
			rt_thread_startup(&mpu6050_thread);
	}
	rst = rt_thread_init(&receive_thread,
											 (const char *)"recuart",
											 receive_thread_entry,
											 RT_NULL,
											 &receive_thread_stack[0],
											 sizeof(receive_thread_stack),
											 RT_THREAD_PRIORITY_MAX-10,
											 20);
	if(rst == RT_EOK)
	{
			rt_thread_startup(&receive_thread);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (flog==1)
		{
		}
        rt_thread_mdelay(5000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "sdram.h"
#include "malloc.h"
#include "includes.h"
#include "myiic.h"
#include "24cxx.h"
#include "w25qxx.h"
#include "stm32h743xx.h"
#include "pcf8574.h"
#include "rs485.h"
#include "fdcan.h"
/************************************************
要实现的功能：
1.分别实现用IIC和QSPI对EEROM和FLASH的读写  							√
2.写入的数据由主机提供												√
3.通过key来选择是读写EEROM还是FLASH									√
4.切换时要将原来存储器中的数据传递给新的存储器（进程间通信）		    √
5.通过485或CAN实现两个板子之间的通信，LED由另一块板子的key控制		   √
6.通过key来选择是使用485还是CAN									    √
7.通过信号量实现对一个缓冲区的互斥访问								 √
8.按键扫描任务通过邮箱向其它任务发送按键信息						  √
消息队列、消息队列集、软件定时器
************************************************/

u8 TEXT_Buffer[16] = {0};
// const u8 Refresh[16] = {0};
#define SIZE sizeof(TEXT_Buffer) 
u32 flashsize=32*1024*1024;
u8 buffer[16];

OS_EVENT * msg_key;			//按键邮箱事件块指针
OS_EVENT * sem_buf;			//蜂鸣器信号量指针

void clear_buffer()
{
	memset(buffer,0,16*sizeof(u8));
}

/////////////////////////UCOSII任务设置///////////////////////////////////
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			10 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				128
//任务堆栈	
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);	
 			   
//LED任务
//设置任务优先级
#define LED_TASK_PRIO       			9
//设置任务堆栈大小
#define LED_STK_SIZE  		    		128
//任务堆栈
OS_STK LED_TASK_STK[LED_STK_SIZE];
//任务函数
void led_task(void *pdata);

#define SR_TASK_PRIO		8
#define SR_STK_SIZE			128
OS_STK SR_TASK_STK[SR_STK_SIZE];
void sr_task(void *pdata);

#define SS_TASK_PRIO			7
#define SS_STK_SIZE			128
OS_STK SS_TASK_STK[SS_STK_SIZE];
void ss_task(void *pdata);

#define RECEIVE_TASK_PRIO		6
#define RECEIVE_STK_SIZE		128
OS_STK RECEIVE_TASK_STK[RECEIVE_STK_SIZE];
void receive_task(void *pdata);

#define SEND_TASK_PRIO			5
#define SEND_STK_SIZE			128
OS_STK SEND_TASK_STK[SEND_STK_SIZE];
void send_task(void *pdata);

#define CAN_TASK_PRIO			4
#define CAN_STK_SIZE			128
OS_STK CAN_TASK_STK[CAN_STK_SIZE];
void can_task(void *pdata);

#define RS485_TASK_PRIO			3
#define RS485_STK_SIZE			128
OS_STK RS485_TASK_STK[RS485_STK_SIZE];
void rs485_task(void *pdata);

#define MAIN_TASK_PRIO			2
#define MAIN_STK_SIZE			128
OS_STK MAIN_TASK_STK[MAIN_STK_SIZE];
void main_task(void *pdata);

#define KEY_TASK_PRIO		1
#define KEY_STK_SIZE			128
OS_STK KEY_TASK_STK[KEY_STK_SIZE];
void key_task(void *pdata);


//////////////////////////////////////////////////////////////////////////////

int main(void)
{
	Cache_Enable();                 //打开L1-Cache
	HAL_Init();				        //初始化HAL库
	Stm32_Clock_Init(160,5,2,4);    //设置时钟,400Mhz 
	delay_init(400);				//延时初始化
	uart_init(115200);				//串口初始化
    LED_Init();                     //初始化LED灯
    KEY_Init();                     //初始化按键
	AT24CXX_Init();					//初始化AT24CXX
	W25QXX_Init();		            //初始化W25QXX
	RS485_Init(9600);				//初始化RS485
	FDCAN1_Mode_Init(10,8,31,8,FDCAN_MODE_NORMAL); //回环测试
	
	OSInit();                       //UCOS初始化

	
    OSTaskCreateExt((void(*)(void*) )start_task,                //任务函数
                    (void*          )0,                         //传递给任务函数的参数
                    (OS_STK*        )&START_TASK_STK[START_STK_SIZE-1],//任务堆栈栈顶
                    (INT8U          )START_TASK_PRIO,           //任务优先级
                    (INT16U         )START_TASK_PRIO,           //任务ID，这里设置为和优先级一样
                    (OS_STK*        )&START_TASK_STK[0],        //任务堆栈栈底
                    (INT32U         )START_STK_SIZE,            //任务堆栈大小
                    (void*          )0,                         //用户补充的存储区
                    (INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);//任务选项,为了保险起见，所有任务都保存浮点寄存器的值
	OSStart(); //开始任务
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//开始任务
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0; 
	pdata=pdata;
	printf("task started\n\r");
	msg_key=OSMboxCreate((void*)0);	//创建消息邮箱
	sem_buf=OSSemCreate(1);
	OSStatInit();  //开启统计任务
	OS_ENTER_CRITICAL();  //进入临界区(关闭中断)
    //LED任务
    OSTaskCreateExt((void(*)(void*) )led_task,                 
                    (void*          )0,
                    (OS_STK*        )&LED_TASK_STK[LED_STK_SIZE-1],
                    (INT8U          )LED_TASK_PRIO,            
                    (INT16U         )LED_TASK_PRIO,            
                    (OS_STK*        )&LED_TASK_STK[0],         
                    (INT32U         )LED_STK_SIZE,             
                    (void*          )0,                         
                    (INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);

	OSTaskCreateExt((void(*)(void*)	)sr_task,
					(void*			)0,
					(OS_STK*		)&SR_TASK_STK[SR_STK_SIZE-1],
					(INT8U			)SR_TASK_PRIO,
					(INT16U			)SR_TASK_PRIO,
					(OS_STK*		)&SR_TASK_STK[0],
					(INT32U			)SR_STK_SIZE,
					(void*			)0,
					(INT16U			)OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);

	OSTaskCreateExt((void(*)(void*)	)ss_task,
					(void*			)0,
					(OS_STK*		)&SS_TASK_STK[SS_STK_SIZE-1],
					(INT8U			)SS_TASK_PRIO,
					(INT16U			)SS_TASK_PRIO,
					(OS_STK*		)&SS_TASK_STK[0],
					(INT32U			)SS_STK_SIZE,
					(void*			)0,
					(INT16U			)OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);

	OSTaskCreateExt((void(*)(void*)	)receive_task,
					(void*			)0,
					(OS_STK*		)&RECEIVE_TASK_STK[RECEIVE_STK_SIZE-1],
					(INT8U			)RECEIVE_TASK_PRIO,
					(INT16U			)RECEIVE_TASK_PRIO,
					(OS_STK*		)&RECEIVE_TASK_STK[0],
					(INT32U			)RECEIVE_STK_SIZE,
					(void*			)0,
					(INT16U			)OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);

	OSTaskCreateExt((void(*)(void*)	)send_task,
					(void*			)0,
					(OS_STK*		)&SEND_TASK_STK[SEND_STK_SIZE-1],
					(INT8U			)SEND_TASK_PRIO,
					(INT16U			)SEND_TASK_PRIO,
					(OS_STK*		)&SEND_TASK_STK[0],
					(INT32U			)SEND_STK_SIZE,
					(void*			)0,
					(INT16U			)OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);

	OSTaskCreateExt((void(*)(void*)	)can_task,
					(void*			)0,
					(OS_STK*		)&CAN_TASK_STK[CAN_STK_SIZE-1],
					(INT8U			)CAN_TASK_PRIO,
					(INT16U			)CAN_TASK_PRIO,
					(OS_STK*		)&CAN_TASK_STK[0],
					(INT32U			)CAN_STK_SIZE,
					(void*			)0,
					(INT16U			)OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);

	OSTaskCreateExt((void(*)(void*)	)rs485_task,
					(void*			)0,
					(OS_STK*		)&RS485_TASK_STK[RS485_STK_SIZE-1],
					(INT8U			)RS485_TASK_PRIO,
					(INT16U			)RS485_TASK_PRIO,
					(OS_STK*		)&RS485_TASK_STK[0],
					(INT32U			)RS485_STK_SIZE,
					(void*			)0,
					(INT16U			)OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);

	OSTaskCreateExt((void(*)(void*)	)main_task,
					(void*			)0,
					(OS_STK*		)&MAIN_TASK_STK[MAIN_STK_SIZE-1],
					(INT8U			)MAIN_TASK_PRIO,
					(INT16U			)MAIN_TASK_PRIO,
					(OS_STK*		)&MAIN_TASK_STK[0],
					(INT32U			)MAIN_STK_SIZE,
					(void*			)0,
					(INT16U			)OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);
					
	OSTaskCreateExt((void(*)(void*) )key_task,                 
                    (void*          )0,
                    (OS_STK*        )&KEY_TASK_STK[KEY_STK_SIZE-1],
                    (INT8U          )KEY_TASK_PRIO,            
                    (INT16U         )KEY_TASK_PRIO,            
                    (OS_STK*        )&KEY_TASK_STK[0],         
                    (INT32U         )KEY_STK_SIZE,             
                    (void*          )0,                         
                    (INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);

	OSTaskSuspend(SR_TASK_PRIO);
	OSTaskSuspend(SS_TASK_PRIO);
	OSTaskSuspend(RECEIVE_TASK_PRIO);
	OSTaskSuspend(SEND_TASK_PRIO);
	OSTaskSuspend(CAN_TASK_PRIO);
	OSTaskSuspend(RS485_TASK_PRIO);

    OS_EXIT_CRITICAL();             //退出临界区(开中断)
	OSTaskSuspend(START_TASK_PRIO); //挂起开始任务
}
 
//LED任务
void led_task(void *pdata)
{
	u8 t;
	while(1)
	{
		t++;
		delay_ms(10);
		if(t==8)LED0(1);	//LED0灭
		if(t==100)		//LED0亮
		{
			t=0;
			LED0(0);
		}
	}									 
}   

void key_task(void *pdata)
{
	u8 key;
	while(1)
	{
		key=KEY_Scan(0);
		if(key) OSMboxPost(msg_key,(void*)key);
		delay_ms(10);
	}
}

void main_task(void *pdata)
{
	u32 key=0;
	u8 err;
	while(1)
	{
		key=(u32)OSMboxPend(msg_key,10,&err);
		switch(key)
		{
			case KEY0_PRES:
				// printf("KEY0_PRES\n");
				printf("IIC\n\r");
				OSTaskSuspend(SR_TASK_PRIO);
				OSTaskSuspend(SS_TASK_PRIO);
				OSTaskResume(RECEIVE_TASK_PRIO);
				break;
			case KEY1_PRES:
				// printf("KEY1_PRES\n");
				printf("QSPI\n\r");
				OSTaskSuspend(RECEIVE_TASK_PRIO);
				OSTaskSuspend(SEND_TASK_PRIO);
				OSTaskResume(SR_TASK_PRIO);
				break;
			case KEY2_PRES:
				// printf("KEY2_PRES\n");
				delay_ms(10);
				break;
			case WKUP_PRES:
				// printf("WKUP_PRES\n");
				printf("MAIN_CTRL->rs485\n\r");
				OSTaskResume(RS485_TASK_PRIO);
				OSTaskSuspend(MAIN_TASK_PRIO);
				delay_ms(10);
				break;
			default:
				delay_ms(10);
				break;
		}
	}
}

void receive_task(void *pdata)
{
	int i;
	printf("USART_ISR_EOBF=%X\n",(USART1->ISR&USART_ISR_EOBF)>>12);
	while(1)
	{
		while(AT24CXX_Check())
		{
			printf("24C02 Check Failed!\n\r");
			delay_ms(1000);
			LED0_Toggle;
		}
		//当UART接收到数据
		if(USART_RX_STA&0x8000)
		{
			printf("USART_ISR_EOBF=%X\n",(USART1->ISR&USART_ISR_EOBF)>>12);
			//将接收到的数据写入EEROM中
			AT24CXX_Write(0,(u8*)USART_RX_BUF,SIZE);
			printf("写入EEPROM 24C02:");
			i=0;
			//将UART接收寄存器中的内容打印到屏幕上，并清空接收寄存器
			while(i<SIZE)
			{
				printf("%c",USART_RX_BUF[i]);
				USART_RX_BUF[i]=USART_RX_BUF[i]&0;
				i++;
			}
			printf("\n\r");
			//数据已写入，恢复读任务
			OSTaskResume(SEND_TASK_PRIO);
		}
	}
}

void send_task(void *pdata)
{
	while(1)
	{
		while(AT24CXX_Check())
		{
			printf("24C02 Check Failed!\n\r");
			delay_ms(1000);
			LED0_Toggle;
		}
		//将EEROM中的数据读出，写入TEXT_Buffer中
		AT24CXX_Read(0,(u8*)TEXT_Buffer,SIZE);
		printf("EEPROM 24C02中的数据为:\n");
		//将EEROM中的数据通过串口打印到主机屏幕上
		HAL_UART_Transmit(&UART1_Handler,(uint8_t*)TEXT_Buffer,SIZE,1000);
		printf("\n\r");
		//清除UART接收状态标记（USART_ISR_EOBF位会置0）
		USART_RX_STA=0;
		//挂起读任务，等待数据写入
		OSTaskSuspend(SEND_TASK_PRIO);
	}
	
}

void sr_task(void *pdata)
{
	int i;
	while(1)
	{
		while(W25QXX_ReadID()!=W25Q256)
		{
			printf("W25Q256 Check Failed!\n\r");
			delay_ms(1000);
			LED0_Toggle;
		}
		//当UART接收到数据
		if(USART_RX_STA&0x8000)
		{
			//当接收到完整块时，ISR寄存器的第12位为1，USART_RX_STA&0x8000才成立
			printf("USART_ISR_EOBF=%X\n",(USART1->ISR&USART_ISR_EOBF)>>12);
			//将接收到的数据写入FLASH中
			W25QXX_Write((u8*)USART_RX_BUF,flashsize-100,SIZE);
			printf("写入FLASH W25Q256:");
			i=0;
			//将UART接收寄存器中的内容打印到屏幕上，并清空接收寄存器
			while(i<SIZE)
			{
				printf("%c",USART_RX_BUF[i]);
				USART_RX_BUF[i]=USART_RX_BUF[i]&0;
				i++;
			}
			printf("\n\r");
			//数据已写入，恢复读任务
			OSTaskResume(SS_TASK_PRIO);
		}
	}
}

void ss_task(void *pdata)
{
	while(1)
	{
		while(W25QXX_ReadID()!=W25Q256)
		{
			printf("W25Q256 Check Failed!\n\r");
			delay_ms(1000);
			LED0_Toggle;
		}
		//将FLASH中的数据读出，写入buffer中
		W25QXX_Read(TEXT_Buffer,flashsize-100,SIZE);
		printf("FLASH W25Q256中的数据为:\n");
		//将FLASH中的数据通过串口打印到主机屏幕上
		HAL_UART_Transmit(&UART1_Handler,(uint8_t*)TEXT_Buffer,SIZE,1000);
		printf("\n\r");
		//清除UART接收状态标记（USART_ISR_EOBF位会置0）
		USART_RX_STA=0;
		//挂起读任务，等待数据写入
		OSTaskSuspend(SS_TASK_PRIO);
	}
}

void rs485_task(void *pdata)
{
	u32 key=0;
	u8 err;
	while(1)
	{
		key=(u32)OSMboxPend(msg_key,10,&err);
		if(key)
		{
			OSSemPend(sem_buf,0,&err);
			buffer[0]=key;
			RS485_Send_Data(buffer,1);
			if(key==KEY2_PRES)
			{
				printf("rs485->CAN\n\r");
				clear_buffer();
				OSSemPost(sem_buf);
				OSTaskResume(CAN_TASK_PRIO);
				OSTaskSuspend(RS485_TASK_PRIO);
			}
			else if(key==WKUP_PRES)
			{
				printf("rs485->MAIN_CTRL\n\r");
				clear_buffer();
				OSSemPost(sem_buf);
				OSTaskResume(MAIN_TASK_PRIO);
				OSTaskSuspend(RS485_TASK_PRIO);
			}
			else
			{
				clear_buffer();
				OSSemPost(sem_buf);
			}
		}

		OSSemPend(sem_buf,0,&err);
		RS485_Receive_Data(buffer,(u8*)&key);
		if(key==1)
		{
			printf("rs485 receive %d\n",buffer[0]);
			switch(buffer[0])
			{
				case KEY0_PRES:
					LED0_Toggle;
					break;
				case KEY1_PRES:
					LED1_Toggle;
					break;
				case KEY2_PRES:
					LED0_Toggle;
					LED1_Toggle;
					delay_ms(10);
					break;
				case WKUP_PRES:
					delay_ms(10);
					break;
				default:
					delay_ms(10);
					break;
			}

		}
		clear_buffer();
		OSSemPost(sem_buf);
	}
}

void can_task(void *pdata)
{
	u32 key=0;
	u8 err;
	u8 res=0;
	while(1)
	{
		key=(u32)OSMboxPend(msg_key,10,&err);
		if(key)
		{
			OSSemPend(sem_buf,0,&err);
			buffer[0]=key;
			res=FDCAN1_Send_Msg(buffer,FDCAN_DLC_BYTES_8);
			if(res) printf("CAN Failed!\n");
			if(key==KEY2_PRES)
			{
				printf("CAN->rs485\n\r");
				clear_buffer();
				OSSemPost(sem_buf);
				OSTaskResume(RS485_TASK_PRIO);
				OSTaskSuspend(CAN_TASK_PRIO);
			}
			else if(key==WKUP_PRES)
			{
				printf("CAN->MAIN_CTRL\n\r");
				clear_buffer();
				OSSemPost(sem_buf);
				OSTaskResume(MAIN_TASK_PRIO);
				OSTaskSuspend(CAN_TASK_PRIO);
			}
			else
			{
				clear_buffer();
				OSSemPost(sem_buf);
			}
		}
		
		OSSemPend(sem_buf,0,&err);
		key=FDCAN1_Receive_Msg(buffer);
		if(key)
		{
			printf("CAN receive %d\n",buffer[0]);
			switch(buffer[0])
			{
				case KEY0_PRES:
					LED0_Toggle;
					break;
				case KEY1_PRES:
					LED1_Toggle;
					break;
				case KEY2_PRES:
					LED0_Toggle;
					LED1_Toggle;
					delay_ms(10);
					break;
				case WKUP_PRES:
					delay_ms(10);
					break;
				default:
					delay_ms(10);
					break;
			}

		}
		OSSemPost(sem_buf);
	}
}



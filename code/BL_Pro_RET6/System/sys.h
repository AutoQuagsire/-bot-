#ifndef _MYSYS_H
#define _MYSYS_H

#include "main.h"
#include "tim.h"
#include "adc.h"
#include "spi.h"
#include "gpio.h"

#include "stdarg.h"
#include "stdio.h"
#include "stdint.h"
#include <string.h>
#include <math.h>




#include "INT.h"
#include "FOC.h"
#include "Filter.h"
#include "usb_debug.h"

/* 单开关选择调参对象：改这个宏即可切换左右电机 */
#define PID_TUNE_SIDE_LEFT   0U
#define PID_TUNE_SIDE_RIGHT  1U

#ifndef PID_TUNE_SIDE
#define PID_TUNE_SIDE PID_TUNE_SIDE_RIGHT
#endif

typedef struct {
    float Kp, Ki, Kd;      // 参数
    float error_integral;  // 内部：误差累计值 (外部不要修改)
    float last_error;      // 内部：上次误差 (外部不要修改)
    float output;          // 输出
	float integral_limit;  // 积分限幅幅度
    float output_limit;  // 输出限幅幅度
    float I_SEP_RATIO;  //积分带比例
    float I_ERR_MIN;//控制量量纲大小，避免积分带过窄
} PID_t;
// PID控制函数
void PID_Calculate(PID_t *pid, float target, float measure, uint8_t freeze_external);
void PID_CalculateTest(PID_t *pid, float target, float measure);
void PID_ParameterInit(PID_t *pid,float kp,float ki,float kd,float integral_limit);
void PID_ParameterInitEx(PID_t *pid,float kp,float ki,float kd,float integral_limit,
                 float output_limit,float i_err_min,float i_sep_ratio);























#define Pin_H(port,pin) 	((port)->BSRR = pin)
#define Pin_L(port,pin)     ((port)->BSRR = (uint32_t)(pin) << 16)
#define Pin_READ(port,pin)  (((port)->IDR & pin) != 0)

//////////////////////////////////////////////////////////////////////////////////     
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板           
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/8/18
//版本：V1.7
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////      

//0,不支持ucos
//1,支持ucos
#define SYSTEM_SUPPORT_OS        0        //定义系统文件夹是否支持UCOS
                                                                        
     
//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t


//以下为汇编函数
void WFI_SET(void);        //执行WFI指令
void INTX_DISABLE(void);//关闭所有中断
void INTX_ENABLE(void);    //开启所有中断
void MSR_MSP(u32 addr);    //设置堆栈地址






typedef struct 
{
    u32 HZCFlag;
    u8 passwd1[7];
    u8 passwd2[7];
    u8 cardid[10][6];
    u8 errCnt;//错误计数
    u8 errTime;//等待错误时间
}SysTemPat;






void GPIO_ResetBits(GPIO_TypeDef *GPIO_PORT,uint16_t GPIO_PIN);
void GPIO_SetBits(GPIO_TypeDef *GPIO_PORT,uint16_t GPIO_PIN);
uint32_t getMicros(void) ;
// 初始化SysTick为微秒计数
void SysTick_Init_us(void) ;
void DWT_Init_us(void) ;

// 快速获取原始计数（内联函数）
static inline uint32_t DWT_GetTicks(void) {
    return DWT->CYCCNT;
}

// 计算时间差（单位：ticks，内联函数）
static inline uint32_t DWT_GetElapsedTicks(uint32_t start) {
    return DWT->CYCCNT - start;  // 自动处理溢出
}

// 将 ticks 转换为微秒（仅在需要时调用）
uint32_t DWT_TicksToMicros(uint32_t ticks);


#endif
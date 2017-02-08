/********************************************
龙丘MC9S12XS128多功能开发板 
Designed by Chiu Sir
E-mail:chiusir@yahoo.cn
软件版本:V2.2
最后更新:2014年1月5日
相关信息参考下列地址:
网站:  http://www.lqist.cn
论坛:  http://smartcar.5d6d.com
淘宝店:http://shop36265907.taobao.com   
------------------------------------
Code Warrior 5.0
Target : MC9S12XS128
Crystal: 16.000Mhz
busclock:32.000MHz
pllclock:64.000MHz 
**********************************************/
#ifndef _PWM_H
#define _PWM_H

#define BUS_FREQ 64000000  /* //总线频率，运用了PLL超频技术*/
#define PWM_PRCLK 0x22         //CH_A=CH_B=BUS_RERQ/4
#define CH_A_FREQ 16000000    //时钟A频率
#define CH_B_FREQ 16000000    //时钟B频率
#define CH_SA_FREQ 2000000   //时钟SA频率
#define CH_SB_FREQ 2000000   //时钟SB频率
#define CH_SA_DIV (CH_A_FREQ/(CH_SA_FREQ*2)) // CH_SA_DIV=4
#define CH_SB_DIV (CH_B_FREQ/(CH_SB_FREQ*2)) // CH_SB_DIV=4
#define STEER_FREQ 100       //舵机频率100Hz
#define MOTOR_FREQ 10000      //直流电机频率10KHz

struct _pid;
void PWM_Init(void);  
void PID_init(void);
float PID_realizer(int speed);
float PID_realizel(int speed);
void motorr(int a);
void motorl(int a); 

 
#endif
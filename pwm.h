/********************************************
����MC9S12XS128�๦�ܿ����� 
Designed by Chiu Sir
E-mail:chiusir@yahoo.cn
����汾:V2.2
������:2014��1��5��
�����Ϣ�ο����е�ַ:
��վ:  http://www.lqist.cn
��̳:  http://smartcar.5d6d.com
�Ա���:http://shop36265907.taobao.com   
------------------------------------
Code Warrior 5.0
Target : MC9S12XS128
Crystal: 16.000Mhz
busclock:32.000MHz
pllclock:64.000MHz 
**********************************************/
#ifndef _PWM_H
#define _PWM_H

#define BUS_FREQ 64000000  /* //����Ƶ�ʣ�������PLL��Ƶ����*/
#define PWM_PRCLK 0x22         //CH_A=CH_B=BUS_RERQ/4
#define CH_A_FREQ 16000000    //ʱ��AƵ��
#define CH_B_FREQ 16000000    //ʱ��BƵ��
#define CH_SA_FREQ 2000000   //ʱ��SAƵ��
#define CH_SB_FREQ 2000000   //ʱ��SBƵ��
#define CH_SA_DIV (CH_A_FREQ/(CH_SA_FREQ*2)) // CH_SA_DIV=4
#define CH_SB_DIV (CH_B_FREQ/(CH_SB_FREQ*2)) // CH_SB_DIV=4
#define STEER_FREQ 100       //���Ƶ��100Hz
#define MOTOR_FREQ 10000      //ֱ�����Ƶ��10KHz

struct _pid;
void PWM_Init(void);  
void PID_init(void);
float PID_realizer(int speed);
float PID_realizel(int speed);
void motorr(int a);
void motorl(int a); 

 
#endif
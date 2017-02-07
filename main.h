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
#ifndef _MAIN_H
#define _MAIN_H
#include <hidef.h>           /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <MC9S12XS128.h>
#include <math.h>
#include <stdio.h>

#include "Set_Bus.h"
#include "Uart.h"
#include "pwm.h" 
#include "LQ_OV7620.h" 
#include "OLED.h"
#include "Encoder.h"
#include "servo.h"
#include "sccb.h"
   
#pragma LINK_INFO DERIVATIVE "MC9S12XS128"
#endif
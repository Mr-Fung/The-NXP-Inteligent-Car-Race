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
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
#ifndef _ATD_H
#define _ATD_H

void delay(void) ;
unsigned int AD_capture(unsigned char s); 
void ATD_Init(void); 
 
#endif
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
#ifndef _LQ_OV7620_H
#define _LQ_OV7620_H

void delay(int ms);
void LQ_OV7620_Init(void);
void filter(void);
void Get_Pixel(void);
void SendPicture(void);
void oledpicture(void);
void Binaryzation(void); 
void doubleline(void);
void doublecontrol(void);
void singleline(void);
void singlecontrol(void);
void zhicontrol(void);
int judgement(void); 

#endif
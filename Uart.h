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
#ifndef _Uart_H
#define _Uart_H


void delayms(int ms);
void SCI_send(unsigned char data);
void send_string(unsigned char *putchar); 
void INIT_SCI(void);




#endif

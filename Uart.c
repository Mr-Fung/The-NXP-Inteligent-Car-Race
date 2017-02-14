#include "main.h"

#define BAUD 115200                   //串口波特率
extern long BUS_CLOCK;

void INIT_SCI(void) 
{
  SCI1BD = BUS_CLOCK/16/BAUD;  
  SCI1CR1 = 0x00;                                           
  SCI1CR2 = 0x28;              
}

void SCI_send(unsigned char data) 
{
  while(!SCI1SR1_TDRE);       //等待发送数据寄存器（缓冲器）为空
  SCI1DRL = data;             //把数据放入SCI数据存储器
}

void send_string(unsigned char *putchar) 
{
  while(*putchar!=0x00)       //判断字符串是否发送完毕
  {
   SCI_send(*putchar++);      //发送字符串
  }
}
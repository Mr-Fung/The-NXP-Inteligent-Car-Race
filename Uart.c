#include "main.h"

#define BAUD 115200                   //���ڲ�����
extern long BUS_CLOCK;

void INIT_SCI(void) 
{
  SCI1BD = BUS_CLOCK/16/BAUD;  
  SCI1CR1 = 0x00;                                           
  SCI1CR2 = 0x28;              
}

void SCI_send(unsigned char data) 
{
  while(!SCI1SR1_TDRE);       //�ȴ��������ݼĴ�������������Ϊ��
  SCI1DRL = data;             //�����ݷ���SCI���ݴ洢��
}

void send_string(unsigned char *putchar) 
{
  while(*putchar!=0x00)       //�ж��ַ����Ƿ������
  {
   SCI_send(*putchar++);      //�����ַ���
  }
}
#include "main.h"

void ATD_Init(void) 
 {
 ATD0CTL2 = 0xc0;  //7λ��
                   //6λAFFC=1�� ת����־λ��������
                   //5λICLKSTP=0����ֹͣģʽ�£�ATDģ��ֹͣ��ǰ��ת��
                   //432λETRIGP��ETRIGLE��ETRIGE=000����ֹ�ⲿ����
                   //1λASCIE=0  ��ֹAD����жϡ
                   //0λACMPIE=0 ��ֹA/D �Ƚ��ж�
 delay();          //��ʱ��������Ӳ��һ���ķ�Ӧʱ��
 ATD0CTL1_SRES=2;//����ATD0CTL1_SRES=0ѡ��8λģ��ת��
                   //   SRES        AD����
                   //    00           8λ
                   //    01          10λ
                   //    10          12λ
                   //    11           ��
 ATD0CTL3 = 0x88;  //7λDJM=1 ת������Ҷ���
                   //6543λS8C��S4C��S2C��S1C=0001ת�����г���Ϊ1
                   //2λFIFO=0 ��ֹFIFOģʽ
                   //10λFRZ1-0=00������ģʽ�¼���ת��

 ATD0CTL4 = 0x01;   //765λSMP2��SMP��SMP1=000 ���ò���ʱ��Ϊ4��ADʱ������
                    //43210λ PPS[4:0]=00001    ����ADʱ������
                    //��ʽ��ATDClock = BusClock/(PRS[4 : 0] + 1) �� 0.5
                    //      ATDClock = 8M      /(1 + 1) �� 0.5 =2M 

}

void delay(void) 
{
unsigned int i;
for(i=0;i<50;i++) 
 {
  asm("nop");           //������ִ��һ���������ڵĿ�ָ��
 }
}


/*************************************************************************
*                    �����������ܿƼ� ����濪����           
*
*  �������ƣ�unsigned char AD_capture(unsigned char s) 
*  ����˵��������ADת��
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2014��1��5��
*  ��    ע��С��ʱ����
*************************************************************************/
unsigned int AD_capture(unsigned char s) 
{
 unsigned int AD_data;
 switch(s)
 { 
  case 1:
    ATD0CTL5 = 0x01;    //6λ3210λSC��CD��CC��CB��CA=00001  ����ģ��������ͨ��ΪAN1
                        //5λSCAN=0  ADת������ֻת��һ��
                        //4λMULT=0  ADת��Ϊ��ͨ������

    while(!ATD0STAT0_SCF);  //SCF��ת��������ɱ�־λ����һ��ת��������ɺ󣬸ñ�־λ��1
                            //�ȴ�ADת�����
    AD_data = ATD0DR0;     //�ӽ��ת���Ĵ�����ȡADת�����������AD_data�/?????
    break;

  case 2:
    ATD0CTL5 = 0x00;    //6λ3210λSC��CD��CC��CB��CA=00000  ����ģ��������ͨ��ΪAN0
                        //5λSCAN=0  ADת������ֻת��һ��
                        //4λMULT=0  ADת��Ϊ��ͨ������
    while(!ATD0STAT0_SCF);  //SCF��ת��������ɱ�־λ����һ��ת��������ɺ󣬸ñ�־λ��1
                            //�ȴ�ADת�����
    AD_data = ATD0DR0;     //�ӽ��ת���Ĵ�����ȡADת�����������AD_data
    break;
 }
 return(AD_data);           //���ر���AD_data
}
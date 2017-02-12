#include "main.h"
  
uint synr=0;
long BUS_CLOCK=0;   //����Ƶ��

void INIT_PLL(PLLx pllx)
{   
   //clksel.7(PLLSELѡ��λ)�����ڲ�����ʱ����Դ  =0,BusClock=OSCCLK/2;   =1,BusClock=PLLCLK/2
   CLKSEL_PLLSEL=0;	  
   PLLCTL_PLLON=0;  //PLLCTL.6(pllon)��Ϊ0;�ȹر�PLL
   //������Ҫ��ʱ��Ƶ������SYNR��REFDV�Ĵ���  BusClock=PLLCLK/2;
   //���㹫ʽ: FVOC=2*OSCCLK*((SYNR+1)/(REFDV+1)) 
   //          PLLCLK=FVCO/(2*POSTDIV) 
   //          BusClock=PLLCLK/2
   //SYNR(.7.6)  32MHZ<FVOC<48MHZ   00   0
   //SYNR(.7.6)  48MHZ<FVOC<80MHZ   01   4
   //SYNR(.7.6)  80MHZ<FVOC<120MHZ  10   8
   //SYNR(.7.6)  120MHZ<FVOC        11   C		
   switch(pllx)
    {
      case PLL16: {
         synr=0x00|0x01;  BUS_CLOCK=16000000;  break;}
      case PLL32: {
         synr=0x40|0x03;  BUS_CLOCK=32000000;  break;}
      case PLL40: {
         synr=0xc0|0x01;  BUS_CLOCK=40000000;  break;}
      case PLL48: {
         synr=0xc0|0x05;  BUS_CLOCK=48000000;  break;}
      case PLL64: {
         synr=0xc0|0x07;  BUS_CLOCK=64000000;  break;}
      case PLL80: {
         synr=0xc0|0x09;  BUS_CLOCK=80000000;  break;}
      case PLL88: {
         synr=0xc0|0x0a;  BUS_CLOCK=88000000;  break;}
      case PLL96: {
         synr=0xc0|0x0b;  BUS_CLOCK=96000000;  break;}
      case PLL104:{
         synr=0xc0|0x0c;  BUS_CLOCK=104000000; break;}
      case PLL120:{
         synr=0xc0|0x0d;  BUS_CLOCK=120000000; break;}
         default :
                 break;
    }
   SYNR=synr; 	
    //���㹫ʽ FREF=OSCCLK/(REDIV*2)	
    //REFDV(.7.6)  1MHZ<FREF<2MHZ    00  0
    //REFDV(.7.6)  2MHZ<FREF<6MHZ    01  4
    //REFDV(.7.6)  6MHZ<FREF<12MHZ   10  8
    //REFDV(.7.6)  12MHZ<FREF	       11  C
   REFDV=0x80 | 0x01;
   //�����Ĵ���PLLCLK=FVCO/(2*POSTDIV)   ���POSTDIV=0x00��PLLCLK=FVCO             
   POSTDIV=0x00;     

   PLLCTL=0x70;  //PLLCTL.6(pllon)��Ϊ1;��PLL

   _asm(nop);         
   _asm(nop);
   while(CRGFLG_LOCK==0); //�Ÿ���CRGFLG�Ĵ�����LOCKλ��ȷ��PLL�Ƿ��ȶ�	 LOCK==1 �ȶ���==0 ���ȶ�  
   CLKSEL_PLLSEL =1;     //ѡ��PLL��Ϊʱ��Դ		       
}

#include "main.h"
#include "OLED.h"

extern pidr,pidl,stop,second,startj,start;

uint num=0;
uint d=0;
int speedr=0;
int speedl=0;
volatile uint valuer;
volatile uint valuel;

void PIT_Init(void)
{
    PITCFLMT=0X00;        //PIT模块禁止
    PITCE=0x03;         //通道使能
    PITMUX=0X02;          //选择微定时基准0
    PITMTLD0=9;          //t=(39+1)/FBUS=40/40M=1us
    PITLD0=999;           //t=1us*(9999+1)=10ms
    PITMTLD1=159;       //t=(9+1)/FBUS=10/80M=0.125us
    PITLD1=9999;       //t=0.125us*(999+1)=0.125ms
    PITINTE=0x03;
    PITCFLMT=0x80;
}

void Pulse_Init(void)
{
    DDRM = 0x00;
    DDRK_DDRK0 = 1;  PORTK_PK0 = 0;   
    DDRK_DDRK1 = 1;  PORTK_PK1 = 1;   
    DDRK_DDRK2 = 0;  DDRK_DDRK3 = 0;
        
    PACTL = 0;
    PACTL_PAEN = 1;
    PACNT = 0;                
}

void Speed_Read(void)
{
  speedr = PACNT+3;
  PACNT = 0;
  speedl = PTM;

  PORTK_PK0 = 1;
  _asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop);
  _asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop);
  _asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop);
  PORTK_PK0 = 0;              

}

void Show_Speed(void)
{
   volatile uint valuer,valuel;
   valuer=speedr;
   valuel=speedl;
   LED_P6x8Char(70,1,'0'+(valuer/10000));
   LED_P6x8Char(76,1,'0'+(valuer/1000-valuer/10000*10));
   LED_P6x8Char(82,1,'0'+(valuer/100-valuer/1000*10));
   LED_P6x8Char(88,1,'0'+(valuer/10-valuer/100*10));
   LED_P6x8Char(94,1,'0'+(valuer%10));
   LED_P6x8Char(70,2,'0'+(pidr/10000));
   LED_P6x8Char(76,2,'0'+(pidr/1000-pidr/10000*10));
   LED_P6x8Char(82,2,'0'+(pidr/100-pidr/1000*10));
   LED_P6x8Char(88,2,'0'+(pidr/10-pidr/100*10));
   LED_P6x8Char(94,2,'0'+(pidr%10));

   LED_P6x8Char(10,1,'0'+(valuel/10000));
   LED_P6x8Char(16,1,'0'+(valuel/1000-valuel/10000*10));
   LED_P6x8Char(22,1,'0'+(valuel/100-valuel/1000*10));
   LED_P6x8Char(28,1,'0'+(valuel/10-valuel/100*10));
   LED_P6x8Char(34,1,'0'+(valuel%10));  
   LED_P6x8Char(10,2,'0'+(pidl/10000));
   LED_P6x8Char(16,2,'0'+(pidl/1000-pidl/10000*10));
   LED_P6x8Char(22,2,'0'+(pidl/100-pidl/1000*10));
   LED_P6x8Char(28,2,'0'+(pidl/10-pidl/100*10));
   LED_P6x8Char(34,2,'0'+(pidl%10));  
}

#pragma CODE_SEG __NEAR_SEG NON_BANKED
__interrupt 66 void PIT_delay (void)
{
   num++;
   if(num>105)
   {
      num=0;
      Speed_Read();
   }
   PITTF_PTF0=1;
}

#pragma CODE_SEG DEFAULT


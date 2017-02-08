#include "main.h"      /* common defines and macros */

#define SCL PORTB_PB6
#define SDA PORTB_PB7
#define SDA_DR  DDRB_DDRB7
#define SCL_DR  DDRB_DDRB6
#define OUT  1
#define IN  0

/*************************************************
Function: SCCB_Wait
Description: delay
Input: no
Output: no
More:no
*************************************************/
void SCCB_Wait(void)    
{
  unsigned char i;

  for(i=0;i<10;i++)
  {
    _asm nop;
  }
}

/*************************************************
Function: SCCB_Start
Description: signal of start
Input: no
Output: no
More:no
*************************************************/
void SCCB_Start(void)   
{
  SDA_DR=OUT;
  SDA = 1;
  SCL = 1;
  SCCB_Wait();
  SDA = 0;
  SCCB_Wait();
  SCL = 0;
}

/*************************************************
Function: SCCB_Stop
Description: signal of stop
Input: no
Output: no
More:no
*************************************************/
void SCCB_Stop(void)    
{
  SDA_DR=OUT;
  SDA = 0;
  SCCB_Wait();
  SCL = 1;
  SCCB_Wait();
  SDA = 1;
  SCCB_Wait();
}

/*************************************************
Function: SCCB_SendAck
Description: send ack to slave
Input: signal of ack
Output: no
More:no
*************************************************/
void SCCB_SendAck(byte ack)
{
  
  SDA_DR=OUT;
  SDA = ack;
  SCL = 1;
  SCCB_Wait();
  SCL = 0;
}

/*************************************************
Function: SCCB_SendByte
Description: send data  to SCCB register
Input: byte of data
Output: return ack  1:receive ack  0:no ack
More:no
*************************************************/
byte SCCB_SendByte(unsigned char bytedata)
{
  
  unsigned char i;
  byte ack;
  SDA_DR=OUT;
  for(i=0;i<8;i++)
  {
    if(bytedata & 0x80)
      SDA = 1;
    else
      SDA = 0;

    bytedata <<= 1;
    SCCB_Wait();

    SCL = 1;
    SCCB_Wait();
    SCL = 0;
    SCCB_Wait();
  }
 
  SDA = 1;
  SDA_DR=IN;
  SCCB_Wait();
  SCL = 1;
  SCCB_Wait();

  ack = SDA;    

  SCL = 0;
  SCCB_Wait();

  return ack;
}


/******************************************************
Function: SCCB_ReceiveByte
Description: receive data  from SCCB register
Input: no
Output: data 
More:no
********************************************************/
unsigned char SCCB_ReceiveByte(void)  
{
  unsigned char i;
  unsigned char bytedata = 0;
  SDA_DR=IN;
  for(i=0;i<8;i++)
  {
    SCL = 1;
    SCCB_Wait();

    bytedata <<= 1;

    if(SDA)
    {
      bytedata |= 0x01;
    }
    SCL = 0;
    SCCB_Wait();
  }

  return bytedata;
}

/******************************************************
Function: SCCB_ByteWrite
Description: write the data to the address 
Input: device  0xC0  write to  OV6620
               0XC1  read from OV6620
               0x42  write to  OV7620
               0x43  read from OV7620
Output: no
More:no
********************************************************/

void SCCB_ByteWrite(unsigned char device,unsigned char address,unsigned char bytedata)
{     
   unsigned char i;

   byte ack;

   for(i=0;i<20;i++)
   {
     SCCB_Start();
     ack = SCCB_SendByte(device);
     if(ack==1)
     {
       SCCB_Stop();
       continue;
     }
     
     ack = SCCB_SendByte(address);
     if(ack==1)
     {
       SCCB_Stop();
       continue;
     }
     
     ack = SCCB_SendByte(bytedata);
     if(ack==1)
     {
       SCCB_Stop();
       continue;
     }
     SCCB_Stop();
     if(ack==0)  break;    //file://正/常，跳出循环
   }
}

void sccb_init(void)
{
    SDA_DR=1;
    SCL_DR=1;
    SCCB_ByteWrite(0x42,0x06,0x09);     //7620亮度调节  0xff最大 0x00最小 
    SCCB_ByteWrite(0x42,0x07,0x80);
    SCCB_Wait();
}
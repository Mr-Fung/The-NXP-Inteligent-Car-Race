#ifndef _SCCB_H
#define _SCCB_H

void SCCB_Wait(void);    
void SCCB_Start(void);   
void SCCB_Stop(void);    
void SCCB_SendAck(byte ack);
byte SCCB_SendByte(unsigned char bytedata);
unsigned char SCCB_ReceiveByte(void);  
void SCCB_ByteWrite(unsigned char device,unsigned char address,unsigned char bytedata);
void sccb_init(void);

#endif
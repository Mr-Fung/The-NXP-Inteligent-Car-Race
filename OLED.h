#ifndef _OLED_H
#define _OLED_H

void delay(int ms);
void delays(int s);
void PORTJ_Init(void);
void LED_WrCmd(uchar ucData);
void LED_WrDat(uchar ucData);
void LED_SetPos(uchar ucIdxX,uchar ucIdxY); 
void LED_Fill(uchar ucData); 
void SetDisplayOnOff(uchar ucData);
void SetChargePump(uchar ucData);
void SetAddressingMode(uchar ucData); 
void SetSegmentRemap(uchar ucData);
void SetCommonRemap(uchar ucData);
void SetContrastControl(uchar ucData);
void SetEntireDisplayOn(void);
void SetInverseDispaly(uchar ucData);
void oled_Init(void);
const byte F6x8[][6];
const uchar F8X16[];
void LED_P6x8Str(uchar ucIdxX,uchar ucIdxY,char ucDataStr[]);
void LED_P6x8Char(uchar ucIdxX,uchar ucIdxY,char ucData);
void LED_P8x16Char(byte x,byte y,uchar ch);
void LED_P8x16Str(byte x,byte y,char ch[]);


#define LED_SCL PORTB_PB0
#define LED_SDA PORTB_PB1
#define LED_RST PORTB_PB2
#define LED_DC  PORTB_PB3
#define LED_CS  PORTB_PB4
#define LED_PORT DDRB

#endif
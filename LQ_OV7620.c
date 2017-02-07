#include "main.h"
#include "OLED.h"
#include "servo.h"
#include "math.h"
#include "pwm.h"

#define  ROWS 40          
#define  LINES 128         
extern uchar Threshold_Value;
extern slo; 
extern straight,turn,slow,singlespeed;
extern zhi,zhir,zhil,baseline;
extern turnskp,kp,ki,kd,dsp;
extern float turndkp;

int    e=0;
int    pe=0;
int    ei=0;
int    se=0;
int    spe=0;

int    midpwm=7560;
int    leftpwm=8310;
int    rightpwm=6810;
int    effective=0;          //有效前瞻
int    effectives=12;
int    pwm=7600;
 
byte   Line_C=0;
byte   Rows_C=0;
byte   ROW_Space=0;
uchar  Line_Cont=0;
byte   Get_Imge_Flog=0;
byte   Image_Data[ROWS][LINES];
int    mid_line[40];
int    leftline[40];
int    rightline[40];
int baseleft=0;     //第一行左线采集标志
int baseright=0;    //第一行右线采集标志
int rcount=0;
int lcount=0;

void LQ_OV7620_Init(void)
{
      DDRA   = 0x00;
      TIOS   = 0x00;         //场同步的引脚0为输入捕捉
      TSCR2  = 0x00;         //总线时钟1分频
      TCTL3  = 0x00;         //PT4-7关闭捕捉
      TCTL4  = 0x05;         //TI1行同步为上升沿捕捉   TI0场同步为上升沿捕捉 
      TSCR1  = 0x80;         //时钟打开 
      TIE_C1I= 0x01;         //关闭场同步输入捕捉中断
}

void filter(void)
{
    int i,j;
    for(j=39;j>=0;j--)
    {
        for(i=1;i<127;i++)
        {
            if(((Image_Data[j][i]-Image_Data[j][i-1])>50 && (Image_Data[j][i]-Image_Data[j][i+1])>50)||
               ((Image_Data[j][i]-Image_Data[j][i-1])<-50 && (Image_Data[j][i]-Image_Data[j][i+1])<-50))
            {
                Image_Data[j][i]=(Image_Data[j][i+1]+Image_Data[j][i-1])/2;    
            }
        }
    }
}

void Binaryzation(void) 
{
    int i,j;
    for(j=0;j<ROWS;j++)
    {     
       for(i=0;i<LINES;i++)
        {    
          if(Image_Data[j][i]<Threshold_Value) 
          { 
            Image_Data[j][i]=0x00;
          } 
          else
          {
            Image_Data[j][i]=0x01;
          }   
        }
    }
}

void doubleline(void)    //双指引线
{  
    int i,j,k;
    int cl=0;           //左边线找到标志位
    int cr=0;           //右边线找到标志位
    int prel=0;         //上一行左线位置
    int prer=0;         //上一行右线位置
    int bpl=0;          //左线跳变点
    int bpr=0;          //右线跳变点
    int fl=0;           //左线突变标志
    int fr=0;           //右线突变标志
    int downr=0;
    int downl=0;
    
    rcount=0;
    lcount=0;
    baseleft=0;
    baseright=0;
      //先扫描第一行，从中间向两边找边线
      for(i=50;i<=126;i++)
      {
         if(Image_Data[39][i]>Threshold_Value&&Image_Data[39][i+1]<Threshold_Value) 
         {
             prer=i;
             cr=1;
             baseright=1;
             break;
         } 
      }
      for(k=76;k>=0;k--)
      {                                                                
         if(Image_Data[39][k]<Threshold_Value&&Image_Data[39][k+1]>Threshold_Value) 
         {
             prel=k;
             cl=1;
             baseleft=1;
             break;
         } 
      }
      if(cr==0)prer=124;        //没找到右边线，设图像最右边为右边线
      if(cl==0)prel=3;          //没找到左边线，设图像最左边为左边线
      if(prer>124)prer=124;
      if(prel<3)prel=3;
      leftline[39]=prel;
      rightline[39]=prer; 
      //根据第一行找到的边线，沿着线的附近找下一行的边线
      for(j=38;j>=effective;j--)
      {    
           cl=0;cr=0;  
           for(i=(prer-3);i<=(prer+3);i++)
           {
              if(Image_Data[j][i]<Threshold_Value) 
              {
                 prer=i;
                 cr=1;
                 break;
              } 
           }
           
           for(k=(prel+3);k>=(prel-3);k--)
           {                                                                
              if(Image_Data[j][k]<Threshold_Value) 
              {
                 prel=k;
                 cl=1;
                 break;
              } 
           }
           if(cl==0)prel=3;
           else if(cl==1)lcount++;  //左有效边采集数量
           if(cr==0)prer=124;
           else if(cr==1)rcount++;  //右有效边采集数量
           if(prer>124)prer=124;
           if(prel<3)prel=3;
           leftline[j]=prel;
           rightline[j]=prer;       
      }
      //丢线后补线
      for(j=36;j>effective;j--)
      { 
          if(leftline[j]-leftline[j-1]>5) 
          {
              bpl=j+1;
              fl=1;         
              break;
          }
      }
      if(fl==1)
      {
          for(j=bpl;j>effective;j--) 
          {  
                leftline[j-1]=leftline[j]+(leftline[bpl]-leftline[bpl+2])/2; 
          }
      }
      for(j=36;j>effective;j--)
      { 
          if(rightline[j]-rightline[j-1]<(-5)) 
          {
              bpr=j+1;
              fr=1;          
              break;
          }
      }
      if(fr==1)
      {
          for(j=bpr;j>effective;j--) 
          {
                rightline[j-1]=rightline[j]+(rightline[bpr]-rightline[bpr+2])/2;
          } 
      }  
      downr=(rightline[10]+rightline[15]+rightline[20]-rightline[25]-rightline[30]-rightline[35])/3;
      downl=(leftline[10]+leftline[15]+leftline[20]-leftline[25]-leftline[30]-leftline[35])/3;
      if(downr<0)downr=-downr;LED_P6x8Char(10,5,'0'+(downr/1000));LED_P6x8Char(16,5,'0'+(downr/100-downr/1000*10));LED_P6x8Char(22,5,'0'+(downr/10-downr/100*10));LED_P6x8Char(28,5,'0'+(downr%10));
      if(downl<0)downl=-downl;LED_P6x8Char(50,5,'0'+(downl/1000));LED_P6x8Char(56,5,'0'+(downl/100-downl/1000*10));LED_P6x8Char(62,5,'0'+(downl/10-downl/100*10));LED_P6x8Char(68,5,'0'+(downl%10));
      //直角
      if(baseright==1 && rcount>30 && lcount<25 && downl<10 && downr<15 && Image_Data[1][20]<Threshold_Value&&Image_Data[1][108]<Threshold_Value && Image_Data[1][40]<Threshold_Value && Image_Data[1][90]<Threshold_Value && Image_Data[1][64]<Threshold_Value)
      { 
          zhil=1;
          //for(j=39;j>effective;j--){ leftline[j-1]=leftline[j]-10;} 
          zhi=1;
      } 
      if(baseleft==1 && lcount>30 && rcount<25 && downr<10 && downl<15 && Image_Data[1][20]<Threshold_Value&&Image_Data[1][108]<Threshold_Value && Image_Data[1][40]<Threshold_Value && Image_Data[1][90]<Threshold_Value && Image_Data[1][64]<Threshold_Value)
      { 
          zhir=1; 
          //for(j=39;j>effective;j--){ rightline[j-1]=rightline[j]+10;}
          zhi=1; 
      } 
      for(j=effective;j<40;j++) mid_line[j]=(leftline[j]+rightline[j])/2;
      LED_P6x8Char(10,3,'0'+(lcount/10));LED_P6x8Char(16,3,'0'+(lcount%10));LED_P6x8Char(50,3,'0'+(rcount/10));LED_P6x8Char(56,3,'0'+(rcount%10));
      LED_P6x8Char(90,4,'0'+baseleft);LED_P6x8Char(100,4,'0'+baseright);
}

void singleline(void)         //单指引线
{
      int i,j,k;
      for(i=30;i<=97;i++)
      {
         if(Image_Data[39][i]>Threshold_Value&&Image_Data[39][i+1]<Threshold_Value) 
         {
             mid_line[39]=i+1;
             k=i;
             break;
         } 
      }

      for(j=38;j>=effectives;j--)
      {     
           for(i=k-3;i<=k+3;i++)
           {
              if(Image_Data[j][i]<Threshold_Value) 
              {
                 mid_line[j]=i+1;
                 k=i;
                 break;
              } 
           }
      }
}

int judgement(void)          //判断单双指引线
{
      int i;
      int j=0;
      int k=0;
      int l=0;
      for(i=35;i<=93;i++)
      {
          if(Image_Data[35][i]<Threshold_Value)k++;
          if(Image_Data[36][i]<Threshold_Value)l++;
      }
      if(k>5 && k<10 && l>5 && l<10)j=1;
     // else if(k>100&&baseline==0){baseline=1;}
     // else if(k>100&&baseline!=0){baseline=0;}
      else j=0;
      return j;      
}

void singlecontrol(void)
{
      int s,down,spwm,center,spd,up;
      down=(mid_line[32]+mid_line[33]+mid_line[34]-mid_line[36]-mid_line[37]-mid_line[38]);
      center=(mid_line[22]+mid_line[24]-mid_line[26]-mid_line[28]);
      up=(mid_line[12]+mid_line[14]-mid_line[16]-mid_line[18]);
      s=down*3+center*4;//+up*2;
      //s=mid_line[10];
      //if(s<0)s=-s;LED_P6x8Char(10,1,'0'+(s/10000));LED_P6x8Char(16,1,'0'+(s/1000-s/10000*10));LED_P6x8Char(22,1,'0'+(s/100-s/1000*10));LED_P6x8Char(28,1,'0'+(s/10-s/100*10));LED_P6x8Char(34,1,'0'+(s%10));
      //if(top<0)top=-top;LED_P6x8Char(10,4,'0'+(top/10000));LED_P6x8Char(16,4,'0'+(top/1000-top/10000*10));LED_P6x8Char(22,4,'0'+(top/100-top/1000*10));LED_P6x8Char(28,4,'0'+(top/10-top/100*10));LED_P6x8Char(34,4,'0'+(top%10));
      //if(center<0)center=-center;LED_P6x8Char(10,6,'0'+(center/190000));LED_P6x8Char(16,6,'0'+(center/1000-center/10000*10));LED_P6x8Char(22,6,'0'+(center/100-center/1000*10));LED_P6x8Char(28,6,'0'+(center/10-center/100*10));LED_P6x8Char(34,6,'0'+(center%10)); 
      //if(up<0)up=-up;LED_P6x8Char(10,5,'0'+(up/10000));LED_P6x8Char(16,5,'0'+(up/1000-up/10000*10));LED_P6x8Char(22,5,'0'+(up/100-up/1000*10));LED_P6x8Char(28,5,'0'+(up/10-up/100*10));LED_P6x8Char(34,5,'0'+(up%10));  
      //if(down<0)down=-down;LED_P6x8Char(10,7,'0'+(down/10000));LED_P6x8Char(16,7,'0'+(down/1000-down/10000*10));LED_P6x8Char(22,7,'0'+(down/100-down/1000*10));LED_P6x8Char(28,7,'0'+(down/10-down/100*10));LED_P6x8Char(34,7,'0'+(down%10));
      
      //if(s<30&&s>-30)pwm=midpwm-20*(mid_line[35]-64);    
      //else 
      se=64-mid_line[36];
      spe=se;
      spd=30*se+50*(se-spe);
      spwm=(int)(midpwm-turnskp*s+spd);
      
      if(spwm<=rightpwm)spwm=rightpwm;
      if(spwm>=leftpwm)spwm=leftpwm;
      PWMDTY23=spwm;
      motorr(singlespeed);motorl(singlespeed);
}

void doublecontrol(void) 
{
      int s,dpwm,down,up,center,top,pd,brake;
      
      //static int e,pe;
      //static int ei=0;
      down=(mid_line[30]+mid_line[32]+mid_line[34]-mid_line[35]-mid_line[37]-mid_line[39]);
      center=(mid_line[20]+mid_line[22]+mid_line[24]-mid_line[25]-mid_line[27]-mid_line[29]);
      up=(mid_line[10]+mid_line[12]+mid_line[14]-mid_line[15]-mid_line[17]-mid_line[19]);
      top=(mid_line[0]+mid_line[2]+mid_line[4]-mid_line[5]-mid_line[7]-mid_line[9]);
      s=(int)(down*5.9+center*4.8+up*1.2);
      e=64-mid_line[39];
      ei+=e;
      pe=e;
      pd=kp*e+ki*ei+kd*(e-pe);
     
      //s=mid_line[35]-64;      
      if(s<30&&s>-30)dpwm=midpwm-20*(mid_line[35]-64);    
      else 
      dpwm=midpwm-turndkp*s+pd;
      
      if(dpwm<=rightpwm)dpwm=rightpwm;
      if(dpwm>=leftpwm)dpwm=leftpwm;
      //if(zhil==1){pwm=8300;zhil=0;delay(10);}
      //if(zhir==1){pwm=8300;zhir=0;delay(10);}
      PWMDTY23=dpwm;
      brake=(dpwm-midpwm)/30;
      if(brake<0)brake=-brake;
      if(s<0)s=-s;LED_P6x8Char(10,0,'0'+(s/10000));LED_P6x8Char(16,0,'0'+(s/1000-s/10000*10));LED_P6x8Char(22,0,'0'+(s/100-s/1000*10));LED_P6x8Char(28,0,'0'+(s/10-s/100*10));LED_P6x8Char(34,0,'0'+(s%10));
      //if(top<0)top=-top;LED_P6x8Char(10,4,'0'+(top/10000));LED_P6x8Char(16,4,'0'+(top/1000-top/10000*10));LED_P6x8Char(22,4,'0'+(top/100-top/1000*10));LED_P6x8Char(28,4,'0'+(top/10-top/100*10));LED_P6x8Char(34,4,'0'+(top%10));
      if(up<0)up=-up;LED_P6x8Char(50,4,'0'+(up/10000));LED_P6x8Char(56,4,'0'+(up/1000-up/10000*10));LED_P6x8Char(62,4,'0'+(up/100-up/1000*10));LED_P6x8Char(68,4,'0'+(up/10-up/100*10));LED_P6x8Char(74,4,'0'+(up%10));  
      //if(center<0)center=-center;LED_P6x8Char(10,6,'0'+(center/10000));LED_P6x8Char(16,6,'0'+(center/1000-center/10000*10));LED_P6x8Char(22,6,'0'+(center/100-center/1000*10));LED_P6x8Char(28,6,'0'+(center/10-center/100*10));LED_P6x8Char(34,6,'0'+(center%10)); 
      //if(down<0)down=-down;LED_P6x8Char(10,7,'0'+(down/10000));LED_P6x8Char(16,7,'0'+(down/1000-down/10000*10));LED_P6x8Char(22,7,'0'+(down/100-down/1000*10));LED_P6x8Char(28,7,'0'+(down/10-down/100*10));LED_P6x8Char(34,7,'0'+(down%10));
      LED_P6x8Char(10,6,'0'+(brake/10000));LED_P6x8Char(16,6,'0'+(brake/1000-brake/10000*10));LED_P6x8Char(22,6,'0'+(brake/100-brake/1000*10));LED_P6x8Char(28,6,'0'+(brake/10-brake/100*10));LED_P6x8Char(34,6,'0'+(brake%10));
      if(s>100)
      {
          motorl(turn+s/dsp);motorr(turn);
      }
      else if(s<-100)
      {
          motorr(turn-s/dsp);motorl(turn);
      } 
      else
      {
          motorr(straight);motorl(straight);
      }
}

void zhicontrol(void)
{
      int dpwm;
      if(zhil==1)
      {  
          dpwm=leftpwm;
          PWMDTY23=dpwm;
          //PWMDTY6=0;PWMDTY1=turn;
          motorl(20);motorr(slow);
      } 
      if(zhir==1) 
      {
          dpwm=rightpwm;
          PWMDTY23=dpwm;
          //PWMDTY1=0;PWMDTY6=turn; 
          motorr(20);motorl(slow);
      }     
      if(lcount>28 && rcount>28)// && baseleft==1 && baseright==1)
      {
          zhi=0;
          zhir=0;
          zhil=0;
          motorr(straight);motorl(straight);
      }
}

void SendPicture(void) 
{   
    int i,j;
    /*uchar dot;
    Binaryzation();
    for(i=0;i<5;i++) 
    {
       LED_SetPos(0,i);
       for(j=0;j<LINES;j++)
       { 
           dot=Image_Data[i*8][j]
                  +Image_Data[i*8+1][j]*2
                      +Image_Data[i*8+2][j]*4
                          +Image_Data[i*8+3][j]*8
                              +Image_Data[i*8+4][j]*16
                                  +Image_Data[i*8+5][j]*32
                                      +Image_Data[i*8+6][j]*64
                                          +Image_Data[i*8+7][j]*128;
           LED_WrDat(dot);
       }
    }*/      
    for(j=0;j<ROWS;j++) 
    {
        for(i=0;i<LINES;i++)
        {  
           SCI_send(Image_Data[j][i]);
        }  
    } 
    SCI_send(0xff);
}

void oledpicture(void)
{
    uchar dot;
    int i,j;
    Binaryzation();
    for(i=0;i<5;i++) 
    {
       LED_SetPos(0,i);
       for(j=0;j<LINES;j++)
       { 
           dot=Image_Data[i*8][j]
                  +Image_Data[i*8+1][j]*2
                      +Image_Data[i*8+2][j]*4
                          +Image_Data[i*8+3][j]*8
                              +Image_Data[i*8+4][j]*16
                                  +Image_Data[i*8+5][j]*32
                                      +Image_Data[i*8+6][j]*64
                                          +Image_Data[i*8+7][j]*128;
           LED_WrDat(dot);
       }
    }     
}

/*备注：进场中断之后需要做的事是
1，关闭场中断，清除场中断标志位
2，打开行中断，清空行计数器
*/ 
#pragma CODE_SEG __NEAR_SEG NON_BANKED 
 __interrupt 9 void Vsync_INT(void)//场中断 
  {
      
      Line_C    = 0;         //行计数器
      Line_Cont = 0;
      TIE_C1I   = 0x00;      //关闭场同步输入捕捉中断
      TIE_C0I   = 0x01;      //打开行同步输入捕捉中断
      TFLG1_C1F = 0x01;      //清除场同步中断标志 
      Rows_C = 1;
      
  }
  
__interrupt 8 void CHsync_INT(void)//行中断 
  {
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      Line_Cont++;
      if((Line_Cont<=0)||(Line_Cont>240))    //
       {
          return;
       }
       
      ROW_Space=6;
      
      if(Line_Cont%ROW_Space==0) 
      {
        Get_Pixel();
        Line_C++;
      }
      
      if(Line_C>=ROWS)
      {
        TIE_C0I   = 0x00;      // 关闭行同步输入捕捉中断
        TIE_C1I   = 0x01;      // 打开场同步输入捕捉中断
      }
      TFLG1_C0F = 0x01;   //清除行同步中断标志 
      
  }	
#pragma  CODE_SEG DEFAULT 
 
void Get_Pixel(void)
 { 
   _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop);
   _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
    
   Image_Data[Line_C][0]   = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][1]   = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][2]   = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][3]   = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][4]   = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][5]   = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][6]   = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][7]   = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][8]   = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][9]   = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][10]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][11]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][12]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][13]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][14]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][15]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][16]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][17]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][18]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][19]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][20]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][21]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][22]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][23]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][24]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][25]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][26]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][27]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][28]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][29]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][30]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][31]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][32]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][33]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][34]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][35]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][36]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][37]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][38]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][39]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][40]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][41]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][42]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][43]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][44]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][45]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][46]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][47]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][48]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][49]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][50]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][51]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][52]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][53]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][54]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][55]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][56]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][57]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][58]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][59]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][60]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][61]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][62]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][63]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][64]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][65]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][66]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][67]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][68]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][69]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][70]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][71]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][72]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][73]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][74]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][75]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][76]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][77]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][78]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][79]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][80]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][81]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][82]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][83]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][84]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][85]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][86]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][87]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][88]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][89]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][90]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][91]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][92]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][93]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][94]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][95]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][96]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][97]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][98]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][99]  = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][100] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][101] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][102] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][103] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);   _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][104] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);   _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][105] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][106] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][107] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][108] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][109] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][110] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][111] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][112] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][113] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop); _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][114] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][115] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][116] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][117] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][118] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][119] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][120] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);     _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][121] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);    _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][122] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);   _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][123] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][124] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][125] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][126] = PORTA;  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);_asm(nop); _asm(nop);_asm(nop);  _asm(nop); _asm(nop);_asm(nop); _asm(nop);_asm(nop);_asm(nop);
   Image_Data[Line_C][127] = PORTA; 
}

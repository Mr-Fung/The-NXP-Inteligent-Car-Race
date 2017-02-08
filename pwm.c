#include "main.h"
#include "Encoder.h"

extern Speed;
extern speedr;
extern speedl;
extern int straight;
extern int midpwm;

int pidr;
int pidl;

void PWM_Init(void)
{  
      PWME=0x00;
      
      PWMCAE=0X00;
      PWMPOL=0XFF;         //先输出高电平
      PWMCLK=0X00;         
      PWMPRCLK=0X44;       //对A时钟进行32分频,B时钟进行32分频 A=2M B=2
      PWMCTL_CON23=1;
      PWMPER0=255;  
      PWMPER1=255;
      PWMPER6=255;
      PWMPER7=255;    
      PWMPER23=50000;
      PWMDTY23=midpwm;       //mid=8320,left=9120,right=7520
      PORTK_PK4=0;
      PORTK_PK5=0; 
      PWME=0XFF;           //通道使能
      
      PWMDTY6=0;         //leftmotor forward      
      PWMDTY7=0;                //leftmotor backward 
           
      PWMDTY1=0;         //rightmotor forward
      PWMDTY0=0;                //rightmotor backward
}

struct _pid
{ 
    float SetSpeed;            //定义设定值     
    float ActualSpeed;        //定义实际值     
    float rerr;                //定义偏差值 
    float rperr;
    float rpperr;
    float lerr;
    float lperr;
    float lpperr;
    float rpwm;
    float lpwm;
    float Kp,Ki,Kd;            //定义比例、积分、微分系数 
}pid;  

void PID_init(void)
{ 
    pid.SetSpeed=0.0;     
    pid.ActualSpeed=0.0;
    pid.rpwm=0.0;
    pid.lpwm=0.0;     
    pid.rerr=0.0; 
    pid.rperr=0.0;     
    pid.rpperr=0.0;
    pid.lerr=0.0; 
    pid.lperr=0.0;     
    pid.lpperr=0.0;     
    pid.Kp=0.5;     
    pid.Ki=0.3;     
    pid.Kd=0.5; 
}  

float PID_realizer(int speed)
{   
    float incrementr; 
    pid.rerr=speed-speedr;     
    incrementr=pid.Kp*(pid.rerr-pid.rperr)+pid.Ki*pid.rerr+pid.Kd*(pid.rerr-2*pid.rperr+pid.rpperr);     
    pid.rpwm=speed+incrementr;     
    pid.rpperr=pid.rperr;     
    pid.rperr=pid.rerr;     
    return pid.rpwm; 
} 
float PID_realizel(int speed)
{   
    float incrementl; 
    pid.lerr=speed-speedl;     
    incrementl=pid.Kp*(pid.lerr-pid.lperr)+pid.Ki*pid.lerr+pid.Kd*(pid.lerr-2*pid.lperr+pid.lpperr);     
    pid.lpwm=speed+incrementl;     
    pid.lpperr=pid.lperr;     
    pid.lperr=pid.lerr;     
    return pid.lpwm; 
} 


void motorl(int a) 
{
    pidl=(int)PID_realizel(a);
    if(pidl<-20)
    {
        pidl=-20;
        PWMDTY7=-pidl;        //leftmotor backward 
        PWMDTY6=0; 
    } 
    else if(pidl>130)
    {
        pidl=130;
        PWMDTY6=pidl;
        PWMDTY7=0;
    }
    else 
    {
        PWMDTY6=pidl;         //leftmotor forward
        PWMDTY7=0;
    }    
}
void motorr(int a)
{       
    pidr=(int)PID_realizer(a);
    if(pidr<-20)
    {
        pidr=-20;
        PWMDTY0=-pidr;        //leftmotor backward 
        PWMDTY1=0; 
    } 
    else if(pidr>130)
    {         
        pidr=130;
        PWMDTY1=pidr;
        PWMDTY0=0;
    }
    else 
    {
        PWMDTY1=pidr;         //leftmotor forward
        PWMDTY0=0;
    }    
}

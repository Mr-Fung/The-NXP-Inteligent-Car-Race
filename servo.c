#include "main.h"
#include "LQ_OV7620.h"
#include "math.h"

extern  int mid_line[40];
int slo=0;

void servo_delay(int ms)
{
    int i,j;
    for(i=0;i<500;i++)
      for(j=0;j<ms;j++);
}

void slope_left(int bottom,int top)
{
    float s;
    int xy=0;
    int xx=0;
    int x=0;
    int y=0;
    int count=0;
    int j;
    for(j=bottom;j<=top;j++)
    {
        xy+=mid_line[j]*j;
        y+=j;
        x+=mid_line[j];
        xx+=mid_line[j]*mid_line[j];
        count++;
    } 
    s=(float)100*((xy/count)-(x/count)*(y/count))/((xx/count)-(x/count)*(x/count));
    if(s<0)s=-s;
    slo=(int)s;
    LED_P6x8Char(10,7,'0'+(slo/10000));LED_P6x8Char(16,7,'0'+(slo/1000-slo/10000*10));LED_P6x8Char(22,7,'0'+(slo/100-slo/1000*10));LED_P6x8Char(28,7,'0'+(slo/10-slo/100*10));LED_P6x8Char(34,7,'0'+(slo%10));  
}
void slope_right(int bottom,int top)
{    int k;
    float s;
    int xy=0;
    int xx=0;
    int x=0;
    int y=0;
    int count=0;
    int j;
    for(j=bottom;j<=top;j--)
    {   
        k=55-j;
        xy+=mid_line[k]*k;
        y+=k;
        x+=mid_line[k];
        xx+=mid_line[k]*mid_line[k];
        count++;
    } 
    s=(float)100*((xy/count)-(x/count)*(y/count))/((xx/count)-(x/count)*(x/count));
    if(s<0)s=-s;
    slo=(int)s;
    LED_P6x8Char(10,7,'0'+(slo/10000));LED_P6x8Char(16,7,'0'+(slo/1000-slo/10000*10));LED_P6x8Char(22,7,'0'+(slo/100-slo/1000*10));LED_P6x8Char(28,7,'0'+(slo/10-slo/100*10));LED_P6x8Char(34,7,'0'+(slo%10));  
}

void slope(void)
{
    int s,up,down;
    down=mid_line[30]-mid_line[39];
    up=mid_line[20]-mid_line[29];
    s=down*3+up;
    if(s<0)s=-s;
    LED_P6x8Char(10,7,'0'+(s/10000));LED_P6x8Char(16,7,'0'+(s/1000-s/10000*10));LED_P6x8Char(22,7,'0'+(s/100-s/1000*10));LED_P6x8Char(28,7,'0'+(s/10-s/100*10));LED_P6x8Char(34,7,'0'+(s%10));  
   
}

void servo(void)
{ 
     int pwm;
     if(slo>70)pwm=465;
     else if(slo<30)pwm=390;        //mid=465  left=350   right=580
     else pwm=575;
     //if(pwm<385)pwm=leftbottom;
     //if(pwm>555)pwm=rightbottom;
     PWMDTY23=pwm;
}

struct _pida
{ 
    float Setangle;            //定义设定值
    float Actualangle;        //定义实际值
    float err;                //定义偏差值
    float err_last;            //定义上一个偏差值 
    float Kp,Ki,Kd;            //定义比例、积分、微分系数 
    float voltage;          //定义电压值（控制执行器的变量） 
    float integral;            //定义积分值 
}pida;
void PIDangle_init()
{ 
    pida.Setangle=0.0; 
    pida.Actualangle=0.0; 
    pida.err=0.0; 
    pida.err_last=0.0; 
    pida.voltage=0.0; 
    pida.integral=0.0; 
    pida.Kp=0.2; 
    pida.Ki=0.2; //快速接近
    pida.Kd=0; 
}
float PID_angle(float angle)
{   
	pida.Setangle=angle; 
	pida.err=pida.Setangle-pida.Actualangle;     
	pida.integral+=pida.err; 
	pida.voltage=pida.Kp*pida.err+pida.Ki*pida.integral+pida.Kd*(pida.err-pida.err_last); 
	pida.err_last=pida.err; 
	pida.Actualangle=pida.voltage;     
	return pida.Actualangle; 
}


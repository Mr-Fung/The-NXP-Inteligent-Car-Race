#include "main.h"

extern  byte  Rows_C;
extern  mid_line[40];
extern  slo;
extern  kp,kd;
extern long long startj;

int start=0;
int straight=75;
int turn=65;
int slow=30;
int singlespeed=70;
int judge;
uchar Threshold_Value=90; 
int zhir;
int zhil;
int baseline=0;
int zhi=0;
int stop=0;
int second=0;
int ok=0;
float  turndkp;
int    turnskp;                                               
int    kp;
int    ki;
int    kd;
int    dsp;
int    zuni=30;
static unsigned long long Pulse_Cnt=0;

void main(void) 
{   
    int see;
    //delay(150);
    DisableInterrupts;
    INIT_PLL(PLL80);  //初始化PLL为80M，总线80M
    INIT_SCI();
    LQ_OV7620_Init();
    oled_Init(); 
    PIT_Init();
    Pulse_Init();
    PID_init();
    PORTJ_Init();
    PWM_Init();
    sccb_init(); 
    EnableInterrupts;    //使能中断
    
    //for(;;){Show_Speed();LED_P6x8Char(10,0,'0'+(straight/1000));LED_P6x8Char(16,0,'0'+(straight/100-straight/1000*10));LED_P6x8Char(22,0,'0'+(straight/10-straight/100*10));LED_P6x8Char(28,0,'0'+(straight%10));motor(straight);_FEED_COP(); }
    //for(;;){LED_P6x8Char(10,0,'0'+(pwm/1000));LED_P6x8Char(16,0,'0'+(pwm/100-pwm/1000*10));LED_P6x8Char(22,0,'0'+(pwm/10-pwm/100*10));LED_P6x8Char(28,0,'0'+(pwm%10));PWMDTY23=pwm;_FEED_COP();}
    switch(PTH)
  	{
    	  case 0x00:
    	  {
            	for(;;) 
              {    
                      filter();
                      SendPicture(); 
                      //motorr(straight);motorl(straight);
                    PWMDTY6=straight;PWMDTY1=straight;
                      Show_Speed();
                      Rows_C=0;        
                      _FEED_COP();            
              } 
    	  }break;
    	  
    	  case 0x01:
    	  {
    	        for(;;)
              {
                  LED_P6x8Char(10,7,'0'+(startj/10000));LED_P6x8Char(16,7,'0'+(startj/1000-startj/10000*10));LED_P6x8Char(22,7,'0'+(startj/100-startj/1000*10));LED_P6x8Char(28,7,'0'+(startj/10-startj/100*10));LED_P6x8Char(34,7,'0'+startj%10);

                  if(start==1)
                  {   
                      if(Pulse_Cnt==startj && startj>500)
                      {
                          break;
                      } 
                      else
                      {
                          Pulse_Cnt=startj;
                      }
                      delay(20);
                  }
              }
    	        straight=75;
              turn=65;
              slow=30;
              singlespeed=70;
              turndkp=4.7;
              turnskp=4;
              kp=25;
              ki=0;
              kd=zuni;
              dsp=20;
    	        for(;;) 
              {   
                  if(Rows_C==1)
                  {
                      filter();
                      judge=judgement();
                      if(judge==0)
                      {  
                          doubleline();
                          if(zhi==0)
                          {                            
                              doublecontrol(); 
                          }
                          else
                          {
                              zhicontrol(); 
                          }
                          Rows_C=0;
                      }
                      else
                      {
                        singleline();
                        singlecontrol();
                        Rows_C=0;
                      }
                      //see=kd;
                      Show_Speed();
                      //LED_P6x8Char(50,7,'0'+(see/1000));LED_P6x8Char(56,7,'0'+(see/100-see/1000*10));LED_P6x8Char(62,7,'0'+(see/10-see/100*10));LED_P6x8Char(68,7,'0'+(see%10));
                      //if(start==0){PWMDTY1=0;PWMDTY6=0;}
                      _FEED_COP();
                  }
              }
    	  }break;
    	  
    	  case 0x03:
    	  {
    	        for(;;)
              {
                  LED_P6x8Char(10,7,'0'+(startj/10000));LED_P6x8Char(16,7,'0'+(startj/1000-startj/10000*10));LED_P6x8Char(22,7,'0'+(startj/100-startj/1000*10));LED_P6x8Char(28,7,'0'+(startj/10-startj/100*10));LED_P6x8Char(34,7,'0'+startj%10);

                  if(start==1)
                  {   
                      if(Pulse_Cnt==startj && startj>500)
                      {
                          break;
                      } 
                      else
                      {
                          Pulse_Cnt=startj;
                      }
                      delay(20);
                  }
              }
    	        straight=80;
              turn=65;
              slow=30;
              singlespeed=70;
              turndkp=5.0;
              turnskp=4;
              kp=25;
              ki=0;
              kd=zuni;
              dsp=18;
    	        for(;;) 
              {   
                  if(Rows_C==1)
                  {
                      filter();
                      judge=judgement();
                      if(judge==0)
                      {  
                          doubleline();
                          if(zhi==0)
                          {                            
                              doublecontrol(); 
                          }
                          else
                          {
                              zhicontrol(); 
                          }
                          Rows_C=0;
                      }
                      else
                      {
                        singleline();
                        singlecontrol();
                        Rows_C=0;
                      }
                      see=kd;
                      Show_Speed();
                      LED_P6x8Char(50,7,'0'+(see/1000));LED_P6x8Char(56,7,'0'+(see/100-see/1000*10));LED_P6x8Char(62,7,'0'+(see/10-see/100*10));LED_P6x8Char(68,7,'0'+(see%10));
                      //if(start==0){PWMDTY1=0;PWMDTY6=0;}
                      _FEED_COP(); 
                  }
              } 
    	  }break;
     
        case 0x07:
    	  {
    	        for(;;)
              {
                  LED_P6x8Char(10,7,'0'+(startj/10000));LED_P6x8Char(16,7,'0'+(startj/1000-startj/10000*10));LED_P6x8Char(22,7,'0'+(startj/100-startj/1000*10));LED_P6x8Char(28,7,'0'+(startj/10-startj/100*10));LED_P6x8Char(34,7,'0'+startj%10);

                  if(start==1)
                  {   
                      if(Pulse_Cnt==startj && startj>500)
                      {
                          break;
                      } 
                      else
                      {
                          Pulse_Cnt=startj;
                      }
                      delay(20);
                  }
              }
    	        straight=70;
              turn=60;
              slow=30;
              singlespeed=60;
              turndkp=4.5;
              turnskp=4;
              kp=25;
              ki=0;                                     
              kd=zuni;
              dsp=25;
    	        for(;;) 
              {   
                  if(Rows_C==1)
                  {
                      filter();
                      judge=judgement();
                      if(judge==0)
                      {  
                          doubleline();
                          if(zhi==0)
                          {                            
                              doublecontrol(); 
                          }
                          else
                          {
                              zhicontrol(); 
                          }
                          Rows_C=0;
                      }
                      else
                      {
                        singleline();
                        singlecontrol();
                        Rows_C=0;
                      }
                      see=kd;
                      Show_Speed();
                      LED_P6x8Char(50,7,'0'+(see/1000));LED_P6x8Char(56,7,'0'+(see/100-see/1000*10));LED_P6x8Char(62,7,'0'+(see/10-see/100*10));LED_P6x8Char(68,7,'0'+(see%10));
                      //if(start==0){PWMDTY1=0;PWMDTY6=0;}
                      _FEED_COP(); 
                      /*if(startj>Pulse_Cnt+10)
                      { 
                        for(;;)
                        {
                          if(Pulse_Cnt==startj && startj>1000)
                          {
                              PWMDTY1=0;PWMDTY6=0;
                              stop=1;
                          } 
                          else
                          {
                              Pulse_Cnt=startj;
                          }
                          delay(20);
                        }
                      }*/
                  }
              } 
    	  }break; 
    
      	case 0x80:
    	  {
            	for(;;) 
              {    
    	                LED_P6x8Char(50,7,'0'+(second/100));LED_P6x8Char(56,7,'0'+(second/10-second/100*10));LED_P6x8Char(62,7,'0'+second%10);
                      Threshold_Value=second;
                      filter();
                      oledpicture(); 
                      //motorr(straight);motorl(straight);
                      //PWMDTY6=straight;PWMDTY1=straight;
                      //Show_Speed();
                      //Rows_C=0;        
                      _FEED_COP();            
              } 
    	  }break;
    	  
    	  case 0x81:
    	  {
    	        while(ok==0)
    	        {
    	            LED_P6x8Char(50,7,'0'+(second/100));LED_P6x8Char(56,7,'0'+(second/10-second/100*10));LED_P6x8Char(62,7,'0'+second%10);
    	        };
    	        delays(second);
    	        straight=75;
              turn=65;
              slow=30;
              singlespeed=70;
              turndkp=4.5;
              turnskp=4;
              kp=25;
              ki=0;
              kd=zuni;
              dsp=23;
              //startj=0;
    	        for(;;) 
              {   
                  if(Rows_C==1)
                  {
                      filter();
                      judge=judgement();
                      if(judge==0)
                      {  
                          doubleline();
                          if(zhi==0)
                          {                            
                              doublecontrol(); 
                          }
                          else
                          {
                              zhicontrol(); 
                          }
                          Rows_C=0;
                      }
                      else
                      {
                        singleline();
                        singlecontrol();
                        Rows_C=0;
                      }
                      see=kd;
                      Show_Speed();
                      //LED_P6x8Char(50,7,'0'+(see/1000));LED_P6x8Char(56,7,'0'+(see/100-see/1000*10));LED_P6x8Char(62,7,'0'+(see/10-see/100*10));LED_P6x8Char(68,7,'0'+(see%10));
                      //if(start==0){PWMDTY1=0;PWMDTY6=0;}
                      _FEED_COP();
                      /*if(startj>Pulse_Cnt)
                      { 
                        for(;;)
                        {
                          if(Pulse_Cnt==startj && startj>50)
                          {
                              for(;;)
                              {
                                  PWMDTY1=0;PWMDTY6=0;
                              }
                          } 
                          else
                          {
                              Pulse_Cnt=startj;
                          }
                          delay(10); 
                        }
                      }*/
                  }
              }
    	  }break;
    	  
    	  case 0x83:
    	  {
    	        while(ok==0)
    	        {
    	            LED_P6x8Char(50,7,'0'+(second/100));LED_P6x8Char(56,7,'0'+(second/10-second/100*10));LED_P6x8Char(62,7,'0'+second%10);
    	        };
    	        delays(second);
    	        while(ok==0);
    	        straight=80;
              turn=65;
              slow=30;
              singlespeed=70;
              turndkp=5.0;
              turnskp=4;
              kp=25;
              ki=0;
              kd=zuni;
              dsp=18;
    	        for(;;) 
              {   
                  if(Rows_C==1)
                  {
                      filter();
                      judge=judgement();
                      if(judge==0)
                      {  
                          doubleline();
                          if(zhi==0)
                          {                            
                              doublecontrol(); 
                          }
                          else
                          {
                              zhicontrol(); 
                          }
                          Rows_C=0;
                      }
                      else
                      {
                        singleline();
                        singlecontrol();
                        Rows_C=0;
                      }
                      see=kd;
                      Show_Speed();
                      LED_P6x8Char(50,7,'0'+(see/1000));LED_P6x8Char(56,7,'0'+(see/100-see/1000*10));LED_P6x8Char(62,7,'0'+(see/10-see/100*10));LED_P6x8Char(68,7,'0'+(see%10));
                      //if(start==0){PWMDTY1=0;PWMDTY6=0;}
                      _FEED_COP(); 
                  }
              } 
    	  }break;
     
        case 0x87:
    	  {
    	        while(ok==0)
    	        {
    	            LED_P6x8Char(50,7,'0'+(second/100));LED_P6x8Char(56,7,'0'+(second/10-second/100*10));LED_P6x8Char(62,7,'0'+second%10);
    	        };
    	        delays(second);
    	        straight=70;
    	        while(ok==0);
              turn=60;
              slow=30;
              singlespeed=60;
              turndkp=4.3;
              turnskp=4;
              kp=25;
              ki=0;                                     
              kd=zuni;
              dsp=28;
    	        for(;;) 
              {   
                  if(Rows_C==1)
                  {
                      filter();
                      judge=judgement();
                      if(judge==0)
                      {  
                          doubleline();
                          if(zhi==0)
                          {                            
                              doublecontrol(); 
                          }
                          else
                          {
                              zhicontrol(); 
                          }
                          Rows_C=0;
                      }
                      else
                      {
                        singleline();
                        singlecontrol();
                        Rows_C=0;
                      }
                      see=kd;
                      Show_Speed();
                      LED_P6x8Char(50,7,'0'+(see/1000));LED_P6x8Char(56,7,'0'+(see/100-see/1000*10));LED_P6x8Char(62,7,'0'+(see/10-see/100*10));LED_P6x8Char(68,7,'0'+(see%10));
                      _FEED_COP(); 
                      /*if(startj>Pulse_Cnt+10)
                      { 
                        for(;;)
                        {
                          if(Pulse_Cnt==startj && startj>1000)
                          {
                              PWMDTY1=0;PWMDTY6=0;
                              stop=1;
                          } 
                          else
                          {
                              Pulse_Cnt=startj;
                          }
                          delay(20);
                        }
                      }*/
                  }
              } 
    	  }break; 
  	}  
}
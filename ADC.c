#include "main.h"

void ATD_Init(void) 
 {
 ATD0CTL2 = 0xc0;  //7位空
                   //6位AFFC=1， 转换标志位快速清零
                   //5位ICLKSTP=0，在停止模式下，ATD模块停止当前的转换
                   //432位ETRIGP、ETRIGLE、ETRIGE=000，禁止外部触发
                   //1位ASCIE=0  禁止AD完成中断�
                   //0位ACMPIE=0 禁止A/D 比较中断
 delay();          //延时函数，给硬件一定的反应时间
 ATD0CTL1_SRES=2;//设置ATD0CTL1_SRES=0选用8位模数转换
                   //   SRES        AD精度
                   //    00           8位
                   //    01          10位
                   //    10          12位
                   //    11           无
 ATD0CTL3 = 0x88;  //7位DJM=1 转换结果右对齐
                   //6543位S8C·S4C·S2C·S1C=0001转换序列长度为1
                   //2位FIFO=0 禁止FIFO模式
                   //10位FRZ1-0=00，冻结模式下继续转换

 ATD0CTL4 = 0x01;   //765位SMP2，SMP，SMP1=000 设置采样时间为4倍AD时钟周期
                    //43210位 PPS[4:0]=00001    设置AD时钟周期
                    //公式：ATDClock = BusClock/(PRS[4 : 0] + 1) × 0.5
                    //      ATDClock = 8M      /(1 + 1) × 0.5 =2M 

}

void delay(void) 
{
unsigned int i;
for(i=0;i<50;i++) 
 {
  asm("nop");           //汇编命令，执行一个总线周期的空指令
 }
}


/*************************************************************************
*                    北京龙邱智能科技 大家玩开发板           
*
*  函数名称：unsigned char AD_capture(unsigned char s) 
*  功能说明：启动AD转换
*  参数说明：无
*  函数返回：无
*  修改时间：2014年1月5日
*  备    注：小延时函数
*************************************************************************/
unsigned int AD_capture(unsigned char s) 
{
 unsigned int AD_data;
 switch(s)
 { 
  case 1:
    ATD0CTL5 = 0x01;    //6位3210位SC·CD·CC·CB·CA=00001  设置模拟量输入通道为AN1
                        //5位SCAN=0  AD转换序列只转换一次
                        //4位MULT=0  AD转换为单通道序列

    while(!ATD0STAT0_SCF);  //SCF：转换序列完成标志位。当一次转换序列完成后，该标志位置1
                            //等待AD转换完成
    AD_data = ATD0DR0;     //从结果转换寄存器读取AD转换结果到变量AD_data�/?????
    break;

  case 2:
    ATD0CTL5 = 0x00;    //6位3210位SC·CD·CC·CB·CA=00000  设置模拟量输入通道为AN0
                        //5位SCAN=0  AD转换序列只转换一次
                        //4位MULT=0  AD转换为单通道序列
    while(!ATD0STAT0_SCF);  //SCF：转换序列完成标志位。当一次转换序列完成后，该标志位置1
                            //等待AD转换完成
    AD_data = ATD0DR0;     //从结果转换寄存器读取AD转换结果到变量AD_data
    break;
 }
 return(AD_data);           //返回变量AD_data
}
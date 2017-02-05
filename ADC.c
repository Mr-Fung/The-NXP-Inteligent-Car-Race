#include "main.h"

void ATD_Init(void) 
 {
 ATD0CTL2 = 0xc0;  //7Î»¿Õ
                   //6Î»AFFC=1£¬ ×ª»»±êÖ¾Î»¿ìËÙÇåÁã
                   //5Î»ICLKSTP=0£¬ÔÚÍ£Ö¹Ä£Ê½ÏÂ£¬ATDÄ£¿éÍ£Ö¹µ±Ç°µÄ×ª»»
                   //432Î»ETRIGP¡¢ETRIGLE¡¢ETRIGE=000£¬½ûÖ¹Íâ²¿´¥·¢
                   //1Î»ASCIE=0  ½ûÖ¹ADÍê³ÉÖÐ¶Ï¡
                   //0Î»ACMPIE=0 ½ûÖ¹A/D ±È½ÏÖÐ¶Ï
 delay();          //ÑÓÊ±º¯Êý£¬¸øÓ²¼þÒ»¶¨µÄ·´Ó¦Ê±¼ä
 ATD0CTL1_SRES=2;//ÉèÖÃATD0CTL1_SRES=0Ñ¡ÓÃ8Î»Ä£Êý×ª»»
                   //   SRES        AD¾«¶È
                   //    00           8Î»
                   //    01          10Î»
                   //    10          12Î»
                   //    11           ÎÞ
 ATD0CTL3 = 0x88;  //7Î»DJM=1 ×ª»»½á¹ûÓÒ¶ÔÆë
                   //6543Î»S8C¡¤S4C¡¤S2C¡¤S1C=0001×ª»»ÐòÁÐ³¤¶ÈÎª1
                   //2Î»FIFO=0 ½ûÖ¹FIFOÄ£Ê½
                   //10Î»FRZ1-0=00£¬¶³½áÄ£Ê½ÏÂ¼ÌÐø×ª»»

 ATD0CTL4 = 0x01;   //765Î»SMP2£¬SMP£¬SMP1=000 ÉèÖÃ²ÉÑùÊ±¼äÎª4±¶ADÊ±ÖÓÖÜÆÚ
                    //43210Î» PPS[4:0]=00001    ÉèÖÃADÊ±ÖÓÖÜÆÚ
                    //¹«Ê½£ºATDClock = BusClock/(PRS[4 : 0] + 1) ¡Á 0.5
                    //      ATDClock = 8M      /(1 + 1) ¡Á 0.5 =2M 

}

void delay(void) 
{
unsigned int i;
for(i=0;i<50;i++) 
 {
  asm("nop");           //»ã±àÃüÁî£¬Ö´ÐÐÒ»¸ö×ÜÏßÖÜÆÚµÄ¿ÕÖ¸Áî
 }
}


/*************************************************************************
*                    ±±¾©ÁúÇñÖÇÄÜ¿Æ¼¼ ´ó¼ÒÍæ¿ª·¢°å           
*
*  º¯ÊýÃû³Æ£ºunsigned char AD_capture(unsigned char s) 
*  ¹¦ÄÜËµÃ÷£ºÆô¶¯AD×ª»»
*  ²ÎÊýËµÃ÷£ºÎÞ
*  º¯Êý·µ»Ø£ºÎÞ
*  ÐÞ¸ÄÊ±¼ä£º2014Äê1ÔÂ5ÈÕ
*  ±¸    ×¢£ºÐ¡ÑÓÊ±º¯Êý
*************************************************************************/
unsigned int AD_capture(unsigned char s) 
{
 unsigned int AD_data;
 switch(s)
 { 
  case 1:
    ATD0CTL5 = 0x01;    //6Î»3210Î»SC¡¤CD¡¤CC¡¤CB¡¤CA=00001  ÉèÖÃÄ£ÄâÁ¿ÊäÈëÍ¨µÀÎªAN1
                        //5Î»SCAN=0  AD×ª»»ÐòÁÐÖ»×ª»»Ò»´Î
                        //4Î»MULT=0  AD×ª»»Îªµ¥Í¨µÀÐòÁÐ

    while(!ATD0STAT0_SCF);  //SCF£º×ª»»ÐòÁÐÍê³É±êÖ¾Î»¡£µ±Ò»´Î×ª»»ÐòÁÐÍê³Éºó£¬¸Ã±êÖ¾Î»ÖÃ1
                            //µÈ´ýAD×ª»»Íê³É
    AD_data = ATD0DR0;     //´Ó½á¹û×ª»»¼Ä´æÆ÷¶ÁÈ¡AD×ª»»½á¹ûµ½±äÁ¿AD_data£/?????
    break;

  case 2:
    ATD0CTL5 = 0x00;    //6Î»3210Î»SC¡¤CD¡¤CC¡¤CB¡¤CA=00000  ÉèÖÃÄ£ÄâÁ¿ÊäÈëÍ¨µÀÎªAN0
                        //5Î»SCAN=0  AD×ª»»ÐòÁÐÖ»×ª»»Ò»´Î
                        //4Î»MULT=0  AD×ª»»Îªµ¥Í¨µÀÐòÁÐ
    while(!ATD0STAT0_SCF);  //SCF£º×ª»»ÐòÁÐÍê³É±êÖ¾Î»¡£µ±Ò»´Î×ª»»ÐòÁÐÍê³Éºó£¬¸Ã±êÖ¾Î»ÖÃ1
                            //µÈ´ýAD×ª»»Íê³É
    AD_data = ATD0DR0;     //´Ó½á¹û×ª»»¼Ä´æÆ÷¶ÁÈ¡AD×ª»»½á¹ûµ½±äÁ¿AD_data
    break;
 }
 return(AD_data);           //·µ»Ø±äÁ¿AD_data
}
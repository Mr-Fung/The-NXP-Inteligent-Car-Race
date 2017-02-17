/********************************************
Crystal: 16.000Mhz
busclock:32.000MHz
pllclock:64.000MHz 
**********************************************/
#ifndef _Set_Bus_H
#define _Set_Bus_H


typedef enum PLLx
{
    PLL16,
    PLL32,
    PLL40,
    PLL48,
    PLL64,
    PLL80,
    PLL88,
    PLL96,
    PLL104,
    PLL120
} PLLx;

void INIT_PLL(PLLx pllx);








#endif 
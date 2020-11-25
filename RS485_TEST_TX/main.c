#include "M0_Core.h"
#include "Code.h"
#include "Code.c"
/**************************************************************************************************/
//u8 *A;

void main(void)
{
  //A = (u8*)0x20001600;
  //for(u8 i = 0; i< 10; i++)
  //{
  //  A[i] = 5;
  //}
  
  RCC_CFG();
  PORT_CFG();
  UART_CFG();
  TIM_CFG();
  CFG_LOAD();
  MODE();       // Выбор режима, кнопки sw3 - master, sw4 -slave
                // sw2 - режим конфиг, sw1  - состояние индикации
  INIT();       // U1_RX2 = (u8*)&UR; UART_RX_IRQ_EN(UART1) = ON;
  
  UART_RX_EN(UART1) = ON;
  while(ON)
  {
    Button_Func();
    Func_0();
    //  LED_H2_R;
    //SET_PHY_STATE();
    //U1();
    //RS_TX();
    //UART_TX(UR.DSIZE*2+6, (u8*)&UR);
  }
}
/**************************************************************************************************/
/*
Версия ПО: 0.0 (16.07.2020)
1. Опытный образец.
*/
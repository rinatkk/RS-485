/****************************************** NVIC **************************************************/
void NVIC_IRQ_ON (u32 IRQ) {NVIC->ISER[IRQ >> 5] = (1 << (IRQ & 0x1F));}
void NVIC_IRQ_OFF(u32 IRQ) {NVIC->ICER[IRQ >> 5] = (1 << (IRQ & 0x1F));}
void NVIC_IRQ_SET(u32 IRQ) {NVIC->ISPR[IRQ >> 5] = (1 << (IRQ & 0x1F));}
void NVIC_IRQ_RST(u32 IRQ) {NVIC->ICPR[IRQ >> 5] = (1 << (IRQ & 0x1F));}
/**************************************************************************************************/
void M2M(void *A, void *B, u32 Size) 
{
  u8 *C = ((u8*)A); 
  u8 *D = ((u8*)B); 
  while(Size--) 
  {
    *D++ = *C++;
  }
} //Копирование памяти
/**************************************************************************************************/
u8 MCP(void *A, void *B, u32 Size) 
{
  u8 *C = ((u8*)A); 
  u8 *D = ((u8*)B); 
  while(Size--) 
  {
    if(*D++ != *C++) 
    {
      return OFF;
    }
  } 
  return ON;
} //Сравнение памяти
/**************************************************************************************************/
void MST(void *A, u32 Size, u8 Char) 
{
  u8 *B = ((u8*)A); 
  while(Size--) 
  {
    *B++ = Char;
  }
} //Заполнение памяти патерном
/**************************************************************************************************/
u8 CRC8(u32 Start, u32 Stop) 
{
  u8 CR = 0; 
  while(Start != Stop) 
  {
    CR += *(u8*)(Start); 
    Start ++;
  } 
  return CR;
} //Контрольная сумма CRC8
/**************************************************************************************************/
u16 CRC16(u32 Start, u32 Stop) 
{
  u16 CR = 0; 
  while(Start != Stop) 
  {
    CR += *(u16*)(Start); 
    Start +=2;
  } return CR;
} //Контрольная сумма CRC16
/**************************************************************************************************/
u32 CRC32(u32 Start, u32 Stop) 
{
  u32 CR = 0; 
  while(Start != Stop) 
  {
    CR += *(u32*)(Start); 
    Start +=4;
  } return CR;
} //Контрольная сумма CRC32
/**************************************************************************************************/
void RCC_CFG(void)
{
  RCC_HSI48_EN = ON; 
  RCC_HSI14_EN = ON;
  while(!RCC_HSI48_RDY && !RCC_HSI14_RDY);
  FLASH_PRFTB_EN = ON; 
  FLASH_LATENCY = FLASH_LTC1; 
  RCC_SYSCLK = RCC_SYS_HSI48;
  RCC_AHB_DIV = RCC_AHB_DIV001; 
  RCC_APB_DIV = RCC_APB_DIV01;
  RCC_GPIOA_EN = ON; 
  RCC_GPIOB_EN = ON; 
  RCC_GPIOC_EN = ON; 
  RCC_GPIOF_EN = ON;
  STK_EN = OFF; 
  STK_REL = 59999; 
  STK_CUR = OFF; 
  STK_IRQ = ON; 
  STK_EN = ON;
}
/*************************************** PORT_CFG *************************************************/
void PORT_CFG(void)
{
  GPIO_CFG_PIN00(GPIOA, GPIO_IN_PU);            //SW1
  GPIO_CFG_PIN01(GPIOA, GPIO_IN_PU);            //SW2
  GPIO_CFG_PIN02(GPIOA, GPIO_AF_PP_PU_H);       //TX2
  GPIO_CFG_PIN03(GPIOA, GPIO_AF_PP_PU_H);       //RX2
  GPIO_AF_PIN02(GPIOA) = GPIO_AF1;
  GPIO_AF_PIN03(GPIOA) = GPIO_AF1;
  GPIO_CFG_PIN04(GPIOA, GPIO_OUT_PP_PD_H);      //DIR2
  GPIO_CFG_PIN05(GPIOA, GPIO_OUT_PP_PD_H);      //H2-G
  GPIO_CFG_PIN06(GPIOA, GPIO_OUT_PP_PD_H);      //H2-R
  GPIO_CFG_PIN07(GPIOA, GPIO_OUT_PP_PD_H);      //H1-G
  GPIO_CFG_PIN08(GPIOA, GPIO_OUT_PP_PD_H);      //DIR1
  GPIO_CFG_PIN09(GPIOA, GPIO_AF_PP_PU_H);       //TX1
  GPIO_CFG_PIN10(GPIOA, GPIO_AF_PP_PU_H);       //RX1
  GPIO_AF_PIN09(GPIOA) = GPIO_AF1;
  GPIO_AF_PIN10(GPIOA) = GPIO_AF1;
  //GPIO_CFG_PIN11(GPIOA, GPIO_ANALOG);           //USB_DM
  //GPIO_CFG_PIN12(GPIOA, GPIO_ANALOG);           //USB_DP
  //GPIO_CFG_PIN13(GPIOA, GPIO_ANALOG);           //SWD
  //GPIO_CFG_PIN14(GPIOA, GPIO_ANALOG);           //SWC
  GPIO_CFG_PIN15(GPIOA, GPIO_IN_PU);            //SW3
  GPIO_CFG_PIN00(GPIOB, GPIO_OUT_PP_PD_H);      //H1-R
  GPIO_CFG_PIN01(GPIOB, GPIO_OUT_PP_PD_H);      //H4-G
  GPIO_CFG_PIN02(GPIOB, GPIO_OUT_PP_PD_H);      //H3-G
  GPIO_CFG_PIN03(GPIOB, GPIO_IN_PU);            //SW4
  GPIO_CFG_PIN04(GPIOB, GPIO_AF_PP_PD_H);       //ZP
  GPIO_AF_PIN04(GPIOB) = GPIO_AF1;
  GPIO_CFG_PIN05(GPIOB, GPIO_OUT_PP_PD_H);      //H5-R
  GPIO_CFG_PIN06(GPIOB, GPIO_OUT_PP_PD_H);      //H5-G
  GPIO_CFG_PIN07(GPIOB, GPIO_OUT_PP_PD_H);      //H6-G
  //GPIO_CFG_PIN08(GPIOB, GPIO_OUT_PP_PD_H);      //NOT USE
  //GPIO_CFG_PIN09(GPIOB, GPIO_ANALOG);           //NOT USE
  GPIO_CFG_PIN10(GPIOB, GPIO_OUT_PP_PD_H);      //H3-R
  GPIO_CFG_PIN11(GPIOB, GPIO_OUT_PP_PD_H);      //H4-R
  //GPIO_CFG_PIN12(GPIOB, GPIO_OUT_PP_PD_H);      //NOT USE
  //GPIO_CFG_PIN13(GPIOB, GPIO_OUT_PP_PD_H);      //NOT USE
  //GPIO_CFG_PIN14(GPIOB, GPIO_OUT_PP_PD_H);      //NOT USE
  //GPIO_CFG_PIN15(GPIOB, GPIO_OUT_PP_PD_H);      //NOT USE
  GPIO_CFG_PIN13(GPIOC, GPIO_OUT_PP_PD_H);      //H6-R
  GPIO_CFG_PIN14(GPIOC, GPIO_OUT_PP_PD_H);      //H7-G
  GPIO_CFG_PIN15(GPIOC, GPIO_OUT_PP_PD_H);      //H8-R
  GPIO_CFG_PIN00(GPIOF, GPIO_OUT_PP_PD_H);      //H8-G
  GPIO_CFG_PIN01(GPIOF, GPIO_OUT_PP_PD_H);      //H7-R
}
/****************************************** ADC ***************************************************/
void ADC_CFG(void)
{
  RCC_ADC_EN = ON; 
  ADC_CAL = ON; 
  while(ADC_CAL);
  ADC_CKMODE = ADC_MODE_ADCCLK; 
  ADC_ALIGN = ADC_ALIGN_R;
  ADC_DATARES = ADC_RES_12BIT; 
  ADC_SMP = ADC_SMP_T555;
  ADC_VREFEN = ON;
  ADC_TSEN = ON; 
  ADC_ON = ON;
  while(!ADC_RDY);
}
/**************************************************************************************************/
u16 ADC_GET(u32 CH) 
{
  ADC_CHSEL = CH; 
  ADC_START = ON; 
  while(!ADC_EOC_FLG) {} 
  return ADC_DATA;
}
/***************************************** USART **************************************************/
void UART_CFG(void)
{
  RCC_USART1_SW = RCC_USART_HSI;
  RCC_UART1_EN = ON;
  //UART_OVER8(UART1) = ON;
  //UART_DIV(UART1) = 0x681; 0x341;
  UART_DIV(UART1) = 208;//139;//69; //(8000000 + BR/2)/BR //115200
  UART_RX_EN(UART1) = ON; 
  UART_TX_EN(UART1) = ON;
  UART_TC_FLG(UART1) = OFF; 
  UART_RXNE_FLG(UART1) = OFF;
  UART_STOP(UART1) = UART_STOP_10; 
  UART_EN(UART1) = ON;
  NVIC_IRQ_ON(UART1_IRQ);
}
/**************************************************************************************************/
void Delay_us(u16 Time) 
{
  TIM_CNT(TIM14) = OFF; 
  TIM_ARR(TIM14) = Time; 
  TIM_UIF(TIM14) = OFF; 
  TIM_EN(TIM14) = ON; 
  while(!TIM_UIF(TIM14));
}
/**************************************************************************************************/
void UART_TX(u16 TX, u8 *Buf) 
{
  DIR1_ON; 
  U1_TX1 = TX; 
  U1_TX2 = Buf; 
  UART_TC_IRQ_EN(UART1) = ON;
}
/**************************************************************************************************/
void UART1_Handler(void)
{
  if(UART_FE_FLG(UART1)||UART_ORE_FLG(UART1))
  {
    UART_FE_CLR(UART1); UART_ORE_CLR(UART1);
  }
  if(UART_RXNE_FLG(UART1))
  {
    if(U1_RX1 >= 518)
    {
      UART_RXFRQ(UART1) = ON;
    }
    data_rx = UART_RXDATA(UART1);
    if(data_rx == PREAMB)
    {
      if(U1_RX1) 
      {
        U1_RX1 = 0;
      }
    }
    data_txrx[U1_RX1] = data_rx;
    
    if(data_txrx[RX_ADD_I] == MASTR_ADDR)
    {
      Stat.tx = PACK_RX_START;
    }
    if(Stat.tx == PACK_RX_START && U1_RX1 == data_txrx[LENGTH_I]-1)
    {
      Stat.tx = PACK_RX_COMPL;
    }
    U1_RX1++;
  }

  if(UART_TC_IRQ_EN(UART1))
  {
    if(U1_TX1)
    {
      UART_TXDATA(UART1) = *U1_TX2;//data_0[U1_TX1]; 
      U1_TX2++;
      U1_TX1--; 
      
    }
    else
    {
      DIR1_OFF; 
      UART_TC_IRQ_EN(UART1) = OFF;
    }
  }
}





void Func_0(void)
{
//  RS_RX();
  if(t_tx >= T_0C1 || Stat.tx == PACK_RX_COMPL)//T_003)
  {
    t_tx = OFF;
    if(Stat.bind == BIND_START && Stat.tx != PACK_RX_START)
    {
      data_txrx[LENGTH_I] = PACK_LEN;
      data_txrx[TX_ADD_I] = MASTR_ADDR;
      data_txrx[RX_ADD_I] = SLAVE_NULL_ADDR;// = 0x01;
      data_txrx[CMD_I] = SET_NULL_ADDR_CMD;
      data_txrx[DAT_0_I] = 0;
      data_txrx[DAT_1_I] = 0x9a;
      data_txrx[DAT_2_I] = 0xbc;
      Tx_Data();
    }
    if(Stat.tx == PACK_RX_COMPL)
    {
      switch(data_txrx[CMD_I])
      {
      case GET_NULL_ADDR_CMD:
        Stat.bind = BIND_NEWADDR;
        slave_addr++;
        data_txrx[LENGTH_I] = PACK_LEN;
        data_txrx[TX_ADD_I] = MASTR_ADDR;
        data_txrx[RX_ADD_I] = SLAVE_NULL_ADDR;
        data_txrx[CMD_I] = SET_NEW_ADDR_CMD;
        data_txrx[DAT_0_I] = slave_addr;
        data_txrx[DAT_1_I] = 0xde;
        data_txrx[DAT_2_I] = 0xf0;
        
        Tx_Data();
        break;
      case GET_NEW_ADDR_CMD:
        Stat.bind = BIND_READY;
        break;
      default: 
        break;
      }
      Stat.tx = PACK_DEFOLT;

    }
    //    if(PIN_SW2)
//    {
//    }
//    else
//    {//как только нажали, начнется запрос устройств с "0" адресом
//      LED_H2_R;
//      Delay_us(0xFFF); 
//      LED_H2_O;
//
//      Stat.bind = STAT_NULL_ADDR_MSK;
//      data_0[SLAVE_ADD_I] = 0x00;// = 0x01;
//      data_0[CMD_I] = SET_NULL_ADDR_CMD;
//      
//      UART_TX(PACK_LEN, data_0);
//    }
//    if(Stat.bind == STAT_NEW_ADDR_MSK)
//    {
//      new_addr++;
//      data_0[CMD_I] = SET_NEW_ADDR_CMD;
//      data_0[SET_SL_ADD_I] = new_addr;
//      if(new_addr >= 24)
//        new_addr = 0;
//      UART_TX(PACK_LEN, data_0);
//    }
  }
}

void Tx_Data(void)
{
  data_txrx[PREAMB_I] = PREAMB;
  UART_TX(PACK_LEN, data_txrx);
}

void Button_Func(void)
{
  if(t_sw >= T_0C2 || Stat.tx == PACK_RX_COMPL)//T_003)
  {
    t_sw = OFF;

    if(PIN_SW2)
    {
    }
    else
    {//как только нажали, начнется запрос устройств с "0" адресом
      if(Stat.bind)
      {
        Stat.bind = 0;
        LED_H2_O;
      }
      else
      {
        Stat.bind = BIND_START;
        LED_H2_R;
  //        Delay_us(0xFFF); 
  //        LED_H2_O;
      }
    }
  }
}
/******************************************* TIM **************************************************/
void TIM_CFG(void)
{
  RCC_TIM3_EN = ON; 
  TIM_PSC(TIM03) = 239; 
  TIM_CC1E(TIM03) = ON; 
  TIM_CC1P(TIM03) = OFF; 
  TIM_OC1M(TIM03) = TIM_MODE_LOW; 
  TIM_OC1M(TIM03) = TIM_MODE_PWM2;
  RCC_TIM14_EN = ON; 
  TIM_PSC(TIM14) = 47; 
  TIM_OPM(TIM14) = ON; 
  TIM_EN(TIM14) = ON;
}
/**************************************************************************************************/
void ZP_SET(u16 F) 
{
  ZP_OFF; 
  TIM_ARR(TIM03) = 100000/F; 
  TIM_CCR1(TIM03) = 50000/F; 
  ZP_ON;
}
/**************************************************************************************************/
void FLASH_WRITE(void *Buf_void, u32 ADR, u32 Size)
{
  u16 Data; 
  u8 *Buf = ((u8*)Buf_void);
  while(FLASH_BSY); 
  FLASH_UNLOCK; 
  FLASH_PG = ON;
  while(Size) 
  {
    Data = *Buf; 
    *Buf++; 
    Size--; 
    if(Size) 
    {
      Data |= *Buf << 8; 
      *Buf++; 
      Size--;
    } 
    *(u16*)(ADR) = Data; 
    ADR += 2;
  }
  FLASH_PG = OFF; 
  FLASH_EOP = ON; 
  FLASH_LOCK = ON;
}
/**************************************************************************************************/
void FLASH_CLEAR(u32 ADR)
{
  while(FLASH_BSY); 
  FLASH_UNLOCK;
  FLASH_EOP = ON; 
  FLASH_PER = ON; 
  FLASH_AR = ADR; 
  FLASH_STRT = ON;
  while(!FLASH_EOP); 
  FLASH_EOP = ON; 
  FLASH_PER = OFF; 
  FLASH_LOCK = ON;
}
/**************************************************************************************************/
void MODE(void)
{
  MSTR = ON;
//  while(ON)
//  {
//    if(PIN_SW3) 
//    {
//      if(SW3) 
//      {
//        SW3 = OFF; MSTR = ON; break;
//      }
//    } 
//    else 
//    {
//      SW3 = ON;
//    }
//    if(PIN_SW4) 
//    {
//      if(SW4) 
//      {
//        SW4 = OFF; MSTR = OFF; break;
//      }
//    } 
//    else 
//    {
//      SW4 = ON;
//    }
//  }
}
/**************************************************************************************************/
void INIT(void)
{
//  if(MSTR)
//  {
//    M2M((u32*)MEM_CFG, (u32*)SRAM_BASE, 12);
//    CONF *CFG = (void*)(SRAM_BASE);
//    MST(CFG, 12, OFF);
//    CFG->DEV.DEV = (INF[0] >> 16)&0x3F;
//    CFG->DEV.PCB = (INF[0] >> 8)&0x3F;
//    CFG->DEV.APP = (INF[0] >> 0)&0x3F;
//    CFG->DEV.CRD = CRC8(UID_BASE, UID_BASE + 12);
//    CFG->DEV.ADR = OFF;
//    CFG->CRC_CFG = ~CRC32(SRAM_BASE, SRAM_BASE + 8) + 1;
//    FLASH_CLEAR(MEM_CFG);
//    FLASH_WRITE((u32*)SRAM_BASE, MEM_CFG, 12);
//  }
  U1_RX2 = (u8*)&UR; 
  UART_RX_IRQ_EN(UART1) = ON;
}
/**************************************************************************************************/
void CFG_DEF(void)
{
  CONF *CFG = (void*)(SRAM_BASE);
  MST(CFG, sizeof(*CFG), OFF);
  CFG->Slave.DEV = (INF[0] >> 24)&0xFF;
  CFG->Slave.PCB = (INF[0] >> 16)&0xFF;
  CFG->Slave.APP = (INF[0] >> 8)&0xFF;
  CFG->Slave.VP = (INF[0] >> 0)&0xFF;
  CFG->Slave.CRD = CRC8(UID_BASE, UID_BASE + 12);
  CFG->Slave.ADR = OFF;
  //CFG->Slave.ADR = ON;
  for(u8 i = 0;i<63;i++)
  {
    CFG->DEV[i].DEV = OFF;
    CFG->DEV[i].PCB = OFF;
    CFG->DEV[i].APP = OFF;
    CFG->DEV[i].VP = OFF;
    CFG->DEV[i].CRD = OFF;
  }
  CFG->DEV[11].DEV = (INF[0] >> 24)&0xFF;
  CFG->DEV[11].PCB = (INF[0] >> 16)&0xFF;
  CFG->DEV[11].APP = (INF[0] >> 8)&0xFF;
  CFG->DEV[11].VP = (INF[0] >> 0)&0xFF;
  CFG->DEV[11].CRD = CRC8(UID_BASE, UID_BASE + 12);
  CFG->VTS_NUM = OFF;
  //CFG->VTS_NUM = ON;
  CFG->CRC_CFG = ~CRC32(SRAM_BASE, SRAM_BASE + sizeof(*CFG) - 4) + 1;
  FLASH_CLEAR(MEM_CFG);
  FLASH_WRITE((u32*)SRAM_BASE, MEM_CFG, sizeof(*CFG));
}
/**************************************************************************************************/
void CFG_LOAD(void)
{
  if(CRC32(MEM_CFG, MEM_CFG + sizeof(*CFG)))
  {
    CFG_DEF();
  }
}
/**************************************************************************************************/
void SET_PHY_STATE(void)
{
  if(t_sw >= T_0C1)
  {
    t_sw = OFF;
    if(!Status.SW)
    {
      if(Status.KEY1) 
      {
        Status.KEY1++;
      }
      if(PIN_SW2)
      {
        if(Status.KEY2 == 24) 
        {
          Status.KEY0 = OFF; 
          Status.KEY1 = OFF; 
          Status.KEY2 = OFF; 
          Status.KEY3 = OFF;
        }
        else
        {
          if(Status.KEY0) 
          {
            Status.KEY0 = OFF; 
            Status.KEY2 = OFF; 
            Status.KEY3++; 
            if(!Status.KEY1) 
            {
              Status.KEY1 = ON;
            }
          }
          if(Status.KEY1 == 7) 
          {
            Status.SW = Status.KEY3; 
            Status.KEY0 = OFF; 
            Status.KEY1 = OFF; 
            Status.KEY2 = OFF; 
            Status.KEY3 = OFF;
          }
        }
      }
      else
      {
        if(Status.KEY0 && !Status.KEY1)
        {
          if(Status.KEY2 == 23) 
          {
            SCH.ZP = ZP_GD; 
            Status.SW = SW_L1;
          }
          if(Status.KEY2 < 24) 
          {
            Status.KEY2++;
          }
        }
        if(!Status.KEY0) 
        {
          ZP_SET(800); 
          Delay_us(0xFFFF); 
          ZP_OFF; 
          Status.KEY0 = ON;
        }
      }
    }
  }
//  switch(SCH.LED_PW)
//  {
//    case LED_R: if(PHY[PHY_PW][1]) {if(PIN_PW_R) {if(PHY[PHY_PW][0] >= PHY[PHY_PW][1]) {PHY[PHY_PW][0] = OFF; LED_PW_O;}} else {if(PHY[PHY_PW][0] >= PHY[PHY_PW][2]) {PHY[PHY_PW][0] = OFF; LED_PW_R;}}} else {LED_PW_R;} break;
//    case LED_G: if(PHY[PHY_PW][1]) {if(PIN_PW_G) {if(PHY[PHY_PW][0] >= PHY[PHY_PW][1]) {PHY[PHY_PW][0] = OFF; LED_PW_O;}} else {if(PHY[PHY_PW][0] >= PHY[PHY_PW][2]) {PHY[PHY_PW][0] = OFF; LED_PW_G;}}} else {LED_PW_G;} break;
//    case LED_Y: if(PHY[PHY_PW][1]) {if(PIN_PW_R) {if(PHY[PHY_PW][0] >= PHY[PHY_PW][1]) {PHY[PHY_PW][0] = OFF; LED_PW_O;}} else {if(PHY[PHY_PW][0] >= PHY[PHY_PW][2]) {PHY[PHY_PW][0] = OFF; LED_PW_Y;}}} else {LED_PW_Y;} break;
//    default: LED_PW_O; break;
//  }
//  switch(SCH.LED_C1)
//  {
//    case LED_R: if(PHY[PHY_C1][1]) {if(PIN_C1_R) {if(PHY[PHY_C1][0] >= PHY[PHY_C1][1]) {PHY[PHY_C1][0] = OFF; LED_C1_O;}} else {if(PHY[PHY_C1][0] >= PHY[PHY_C1][2]) {PHY[PHY_C1][0] = OFF; LED_C1_R;}}} else {LED_C1_R;} break;
//    case LED_G: if(PHY[PHY_C1][1]) {if(PIN_C1_G) {if(PHY[PHY_C1][0] >= PHY[PHY_C1][1]) {PHY[PHY_C1][0] = OFF; LED_C1_O;}} else {if(PHY[PHY_C1][0] >= PHY[PHY_C1][2]) {PHY[PHY_C1][0] = OFF; LED_C1_G;}}} else {LED_C1_G;} break;
//    case LED_Y: if(PHY[PHY_C1][1]) {if(PIN_C1_R) {if(PHY[PHY_C1][0] >= PHY[PHY_C1][1]) {PHY[PHY_C1][0] = OFF; LED_C1_O;}} else {if(PHY[PHY_C1][0] >= PHY[PHY_C1][2]) {PHY[PHY_C1][0] = OFF; LED_C1_Y;}}} else {LED_C1_Y;} break;
//    default: LED_C1_O; break;
//  }
//  switch(SCH.LED_C2)
//  {
//    case LED_R: if(PHY[PHY_C2][1]) {if(PIN_C2_R) {if(PHY[PHY_C2][0] >= PHY[PHY_C2][1]) {PHY[PHY_C2][0] = OFF; LED_C2_O;}} else {if(PHY[PHY_C2][0] >= PHY[PHY_C2][2]) {PHY[PHY_C2][0] = OFF; LED_C2_R;}}} else {LED_C2_R;} break;
//    case LED_G: if(PHY[PHY_C2][1]) {if(PIN_C2_G) {if(PHY[PHY_C2][0] >= PHY[PHY_C2][1]) {PHY[PHY_C2][0] = OFF; LED_C2_O;}} else {if(PHY[PHY_C2][0] >= PHY[PHY_C2][2]) {PHY[PHY_C2][0] = OFF; LED_C2_G;}}} else {LED_C2_G;} break;
//    case LED_Y: if(PHY[PHY_C2][1]) {if(PIN_C2_R) {if(PHY[PHY_C2][0] >= PHY[PHY_C2][1]) {PHY[PHY_C2][0] = OFF; LED_C2_O;}} else {if(PHY[PHY_C2][0] >= PHY[PHY_C2][2]) {PHY[PHY_C2][0] = OFF; LED_C2_Y;}}} else {LED_C2_Y;} break;
//    default: LED_C2_O; break;
//  }
  if(t_zp >= C_ZP)
  {
    t_zp = OFF; 
    if(Status.ZP_FIX && SCH.ZP > ZP_BD) 
    {
      SCH.ZP = OFF;
    }; 
    if(SCH.ZP != Status.ZP_OLD) 
    {
      Status.ZPC = OFF; 
      Status.ZP_OLD = SCH.ZP;
    }
    switch(Status.ZP_OLD)
    {
      case ZP_GD: //Звук "Удачно"
        switch(Status.ZPC)
        {
          case 0:  
            ZP_SET(500); 
            Status.ZPC++; 
            break;
          default: 
            ZP_SET(800); 
            Status.ZPC = OFF; 
            SCH.ZP = OFF; 
            break;
        }
        C_ZP = T_0C1; 
        break;
      case ZP_BD: //Звук "Неудачно"
        ZP_SET(150); 
        SCH.ZP = OFF; 
        C_ZP = T_1C0; 
        break;
      case ZP_P1: //Звук "Пожар 1"
        switch(Status.ZPC)
        {
          case 0: 
            ZP_SET(250); 
            Status.ZPC++; 
            break;
          default: 
            ZP_SET(350); 
            Status.ZPC = OFF; 
            SCH.ZP = OFF; 
            break;
        }
        C_ZP = T_0C2; 
        break;
      case ZP_P2: //Звук "Пожар 2"
        switch(Status.ZPC)
        {
          case 0: 
            ZP_SET(250); 
            Status.ZPC++; 
            break;
          case 1: 
            ZP_SET(800); 
            Status.ZPC++; 
            break;
          case 2: 
            ZP_SET(500); 
            Status.ZPC++; 
            break;
          default: 
            ZP_SET(1000); 
            Status.ZPC = OFF; 
            SCH.ZP = OFF; 
            break;
        }
        C_ZP = T_0C1; 
        break;
      case ZP_ER: //Звук "Неисправность"
        switch(Status.ZPC)
        {
          case 0: 
            ZP_SET(500); 
            Status.ZPC++; 
            break;
          default: 
            ZP_SET(800); 
            Status.ZPC = OFF; 
            SCH.ZP = OFF; 
            break;
        }
        C_ZP = T_0C2; 
        break;
      case ZP_UP: //Звук "Постановка на охрану"
        switch(Status.ZPC)
        {
          case 0: 
            ZP_SET(150); 
            Status.ZPC++; 
            break;
          default: 
            ZP_SET(750); 
            Status.ZPC = OFF; 
            SCH.ZP = OFF; 
            break;
        }
        C_ZP = T_0C2; 
        break;
      default: 
        ZP_OFF; 
        C_ZP = OFF; 
        break;
    }
  }
}
/**************************************************************************************************/
void NEXT_VTS(void)
{
  if(Status.NUM + 1 < CFG->VTS_NUM)
  {
    Status.NUM++;
  }
  else
  {
    Status.NUM = OFF;
  }
}
/**************************************************************************************************/
void RS_RX(void)
{
//  if(U1_RX1 && T_RX >= T_0C1)
  if(t_rx >= T_0C1)
  {
    t_rx = OFF; 
    UART_RX_EN(UART1) = OFF;
    if(UR.SOF == 0xAA && U1_RX1 == UR.DSIZE*2+6 && !CRC16((u32)&UR, (u32)&UR + U1_RX1))
    {
      VTS[Status.NUM].RX_OK = ON;
      VTS[Status.NUM].RX_CRC = OFF;
    }
    else
    {
      VTS[Status.NUM].RX_OK = OFF;
      VTS[Status.NUM].RX_CRC = ON;
    }
    U1_RX2 = (u8*)&UR; 
    U1_RX1 = OFF; 
    UART_RX_EN(UART1) = ON;
  }
}
/**************************************************************************************************/
void RS_TX(void)
{
//  if(MSTR)
//  {
//    switch(VTS[Status.NUM].CMD)
//    {
//      case VTS_SET_ADR: 
//        UR.DSIZE = 3; 
//        UR.DATA[2] = Status.NUM + 1; 
//        break;
//      case VTS_GET_ST: 
//        UR.DSIZE = 2; 
//        break;
//      case VTS_SET_ST: 
//        UR.DSIZE = 3; 
//        break;
//      default: 
//        UR.DSIZE = 2; 
//        break;
//    }
  
  
  
//    data_0[0] = 0xAA;
//    data_0[1] = 0xFF;
//    data_0[3] = PACK_LEN;
//    if(PIN_SW2)
//    {
//    }
//    else
//    {
//      Stat.bind = STAT_BIND_MSK;
//      LED_H2_R;
//      Delay_us(0xFFF); 
//      LED_H2_O;
//    }
//    if(Stat.bind == STAT_BIND_MSK)
//    {
//      Stat.bind = STAT_NULL_ADDR_MSK;
//      data_0[SLAVE_ADD_I] = 0x00;// = 0x01;
//      data_0[CMD_I] = SET_NULL_ADDR_CMD;
//      Stat.bind = STAT_NULL_ADDR_MSK;
//      
//      UART_TX(PACK_LEN*2+6, data_0);
//    }
//    else
//    {
//    }
//    if(Stat.bind == STAT_NEW_ADDR_MSK)
//    {
//      new_addr++;
//      data_0[CMD_I] = SET_NEW_ADDR_CMD;
//      data_0[SET_SL_ADD_I] = new_addr;
//      if(new_addr >= 24)
//        new_addr = 0;
//      UART_TX(PACK_LEN*2+6, data_0);
//    }
    
//    UART_TX(10, data_0);
//
//    data_0[2]++;// = 0x01;
//    if(data_0[2] >= 8)
//    data_0[2] = 0;
//    
    
//    if(Status.RS_OPROS)
//    {
//      UR.AD_RX = OFF;
//    }
//    else
//    {
//      UR.AD_RX = Status.NUM + 1;
//    }
//    VTS[Status.NUM].RX_OK = OFF;
//    VTS[Status.NUM].RX_CRC = OFF;
//    VTS_CTR *CTR = (void*)(&UR.DATA);
//    CTR->CMD = VTS[Status.NUM].CMD;           //Комманда управления
//    CTR->CRD = CFG->DEV[Status.NUM].CRD;      //CRC уникального серийного номера процессора
//    CTR->VP = VTS[Status.NUM].VP;             //Версия протокола обмена (не более максимума)
//    CTR->MD = OFF;                            //Режим работы: 0 - Boot, 1 - APP
//    CTR->IND = OFF;                           //Индикатор изменения состояния
//    CTR->CTX = VTS[Status.NUM].CTX;           //Отклик только по CRC уникального серийного номера процессора
//    CTR->BTX = VTS[Status.NUM].BTX;           //Блокировка отклика на текущий адрес, работа только по CRC уникального серийного номера процессора
//    u16 CRC_0 = CRC16((u32)&UR, (u32)&UR + UR.DSIZE*2+4);
//    CRC_0  = ~CRC_0 ;
//    UR.DATA[UR.DSIZE] = CRC_0  + 1;
//    UART_TX(10, data_0);
//  }
//  else
//  {
//    switch(VTS[0].CMD)
//    {
//      case VTS_ID: 
//        UR.DSIZE = 4; 
//        break;
//      case VTS_GET_ST: 
//        UR.DSIZE = 3; 
//        break;
//      case VTS_SET_ST: 
//        UR.DSIZE = 2; 
//        break;
//      default: 
//        UR.DSIZE = 2; 
//        break;
//    }
//    UR.SOF = 0xAA;
//    UR.AD_TX = CFG->Slave.ADR;
//    if(Status.RS_OPROS)
//    {
//      UR.AD_RX = OFF;
//    }
//    else
//    {
//      UR.AD_TX = CFG->Slave.ADR;
//    }
//    UR.AD_RX = 0xFF;
    //VTS[Status.NUM].VTS_OK = OFF;
    //VTS[Status.NUM].VTS_CRC = OFF;
//    VTS_CTR *CTR = (void*)(&UR.DATA);
//    CTR->CMD = VTS[Status.NUM].CMD;           //Комманда управления
//    CTR->CRD = CFG->DEV[Status.NUM].CRD;      //CRC уникального серийного номера процессора
//    CTR->VP = VTS[Status.NUM].VP;             //Версия протокола обмена (не более максимума)
//    CTR->MD = OFF;                            //Режим работы: 0 - Boot, 1 - APP
//    CTR->IND = OFF;                           //Индикатор изменения состояния
//    CTR->CTX = VTS[Status.NUM].CTX;           //Отклик только по CRC уникального серийного номера процессора
//    CTR->BTX = VTS[Status.NUM].BTX;           //Блокировка отклика на текущий адрес, работа только по CRC уникального серийного номера процессора
//    UR.DATA[UR.DSIZE] = ~CRC16((u32)&UR, (u32)&UR + UR.DSIZE*2+4) + 1;
//    UART_TX(UR.DSIZE*2+6, (u8*)&UR);
//  }
}
/**************************************************************************************************/
void STATE(void)
{
  u8 ERR = 0;
  for(u8 i = 0;i<CFG->VTS_NUM;i++)
  {
    if(VTS[i].RX_ER)
    {
      ERR = ON;
    }
  }
  if(ERR)
  {
    LED_H4_R;
  }
  else
  {
    LED_H4_G;
  }
}




/**************************************************************************************************/
void U1(void)
{
//  if(Status.SW == SW_L1)
//  {
//    CFG_DEF(); SCH.ZP = ZP_GD;
//  }
//  if(Status.SW == SW_S1)
//  {
//    if(Status.RS_OPROS)
//    {
//      Status.RS_OPROS = OFF;
//    }
//    else
//    {
//      Status.RS_OPROS = ON;
//      M2M((u32*)MEM_CFG, (u32*)SRAM_BASE, sizeof(*CFG));
//    }
//    Status.NUM = OFF; 
//    VTS[Status.NUM].RX_OK = OFF; 
//    SCH.ZP = ZP_GD;
//  }
//  Status.SW = OFF;
  if(MSTR)
  { //Master start
//    STATE();
    RS_RX();
    if(t_tx >= T_0C3)//T_003)
//      || VTS[Status.NUM].RX_OK || VTS[Status.NUM].RX_CRC)
    {
      t_tx = OFF;
      RS_TX();
      
//      if(Status.RS_OPROS)
//      { //Master config start
//        if(VTS[Status.NUM].RX_OK)
//        {
//          VTS[Status.NUM].RX_OK = OFF;
//          VTS_CTR *CTR = (void*)(&UR.DATA);
//          VTS_DEV_ID *DEV_ID = (void*)(&UR.DATA[2]);
//          CONF *CFG = (void*)(SRAM_BASE);
//          switch(CTR->CMD)
//          {
//            case VTS_ID:
//              CFG->DEV[Status.NUM].DEV = DEV_ID->DEV;
//              CFG->DEV[Status.NUM].PCB = DEV_ID->PCB;
//              CFG->DEV[Status.NUM].APP = DEV_ID->APP;
//              CFG->DEV[Status.NUM].VP = DEV_ID->VP;
//              CFG->DEV[Status.NUM].CRD = CTR->CRD;
//              CFG->VTS_NUM = Status.NUM + 1;
//              CFG->CRC_CFG = ~CRC32(SRAM_BASE, SRAM_BASE + sizeof(*CFG) - 4) + 1;
//              FLASH_CLEAR(MEM_CFG);
//              FLASH_WRITE((u32*)SRAM_BASE, MEM_CFG, sizeof(*CFG));
//              VTS[Status.NUM].CMD = VTS_SET_ADR;
//              break;
//            case VTS_SET_ADR:
//              Status.NUM++;
//              VTS[Status.NUM].CMD = VTS_ID;
//              break;
//          }
//          VTS[Status.NUM].VP = CFG->DEV[Status.NUM].VP; //Версия протокола обмена (не более максимума)
//          VTS[Status.NUM].CTX = OFF;                    //Отклик только по CRC уникального серийного номера процессора
//          VTS[Status.NUM].BTX = OFF;                    //Блокировка отклика на текущий адрес, работа только по CRC уникального серийного номера процессора
//          RS_TX();
//        }
//        else
//        {
//          VTS[Status.NUM].CMD = VTS_ID;
//          VTS[Status.NUM].VP = CFG->DEV[Status.NUM].VP; //Версия протокола обмена (не более максимума)
//          VTS[Status.NUM].CTX = OFF;                    //Отклик только по CRC уникального серийного номера процессора
//          VTS[Status.NUM].BTX = OFF;                    //Блокировка отклика на текущий адрес, работа только по CRC уникального серийного номера процессора
//          RS_TX();
//        }
//      } //Master config end
//      else
        //Master Standby  start
//      {
//        if(CFG->VTS_NUM)
//        {
      
      
      
//          if(VTS[Status.NUM].RX_OK)
//          {

//            RS_TX();
//          }
          
          
          
//          if(VTS[Status.NUM].RX_OK)
//          {
//            VTS_CTR *CTR = (void*)(&UR.DATA);
//            VTS_STT *VST = (void*)(&UR.DATA[2]);
//            switch(CTR->CMD)
//            {
//              case VTS_GET_ST:
//                if(VST->IND)
//                {
//                  VST->IND = OFF; LED_H7_O;
//                }
//                else
//                {
//                  VST->IND = ON; LED_H7_G;
//                }
//                VTS[Status.NUM].CNT = ON;
//                VTS[Status.NUM].CMD = VTS_SET_ST;                //Комманда управления
//                break;
//              case VTS_SET_ST:
//                NEXT_VTS();
//                  if(VTS[Status.NUM].CNT < 4)
//                  {
//                    VTS[Status.NUM].CNT++;
//                    VTS[Status.NUM].CMD = VTS_ST;                 //Комманда управления
//                  }
//                  else
//                  {
//                    VTS[Status.NUM].RX_ER = ON;
//                    VTS[Status.NUM].CNT = ON;
//                    VTS[Status.NUM].CMD = VTS_ST;                 //Комманда управления
//                  }
//                break;
//              default:
//                if(CTR->IND)
//                {
//                  VTS[Status.NUM].CNT = ON;
//                  VTS[Status.NUM].CMD = VTS_GET_ST;                //Комманда управления
//                }
//                else
//                {
//                  VTS[Status.NUM].CNT = OFF;
//                  NEXT_VTS();
//                  if(VTS[Status.NUM].CNT < 4)
//                  {
//                    VTS[Status.NUM].CNT++;
//                    VTS[Status.NUM].CMD = VTS_ST;                 //Комманда управления
//                  }
//                  else
//                  {
//                    VTS[Status.NUM].RX_ER = ON;
//                    VTS[Status.NUM].CNT = ON;
//                    VTS[Status.NUM].CMD = VTS_ST;                 //Комманда управления
//                  }
//                }
//                break;
//            }
//            VTS[Status.NUM].VP = CFG->DEV[Status.NUM].VP; //Версия протокола обмена (не более максимума)
//            VTS[Status.NUM].CTX = OFF;                    //Отклик только по CRC уникального серийного номера процессора
//            VTS[Status.NUM].BTX = OFF;                    //Блокировка отклика на текущий адрес, работа только по CRC уникального серийного номера процессора
//            RS_TX();
//          }
//          else//нет ответа или ошибка CRC
//          {
//            if(VTS[Status.NUM].CNT)
//            {
//              if(VTS[Status.NUM].CNT < 4)
//              {
//                VTS[Status.NUM].CNT++;
//                //VTS[Status.NUM].CMD = VTS_ST;                 //Комманда управления
//              }
//              else
//              {
//                VTS[Status.NUM].RX_ER = ON;
//                VTS[Status.NUM].CNT = OFF;
//                NEXT_VTS();
//                VTS[Status.NUM].CNT++;
//                VTS[Status.NUM].CMD = VTS_ST;                 //Комманда управления
//              }
//            }
//            else
//            {
//              VTS[Status.NUM].CNT++;
//              VTS[Status.NUM].CMD = VTS_ST;                 //Комманда управления
//            }
//            VTS[Status.NUM].VP = CFG->DEV[Status.NUM].VP; //Версия протокола обмена (не более максимума)
//            VTS[Status.NUM].CTX = OFF;                    //Отклик только по CRC уникального серийного номера процессора
//            VTS[Status.NUM].BTX = OFF;                    //Блокировка отклика на текущий адрес, работа только по CRC уникального серийного номера процессора
//            RS_TX();
//          }
//        }
        //Master Standby  end
//      }
    }
  } //Master end
}
/****************************************** IRQ ***************************************************/
void NMI_Handler(void)          
{
  while(ON){}
}
void HardFault_Handler(void)    
{
  while(ON){}
}
void MemManage_Handler(void)    
{
  while(ON){}
}
void BusFault_Handler(void)     
{
  while(ON){}
}
void UsageFault_Handler(void)   
{
  while(ON){}
}
void SVC_Handler(void)          
{
  while(ON){}
}
void DebugMon_Handler(void)     
{
  while(ON){}
}
void PendSV_Handler(void)       
{
  while(ON){}
}
void SysTick_Handler(void)      
{
//  T_RX++; 
  t_sw++; 
  t_tx++; 
//  T_ZP++;
}
/**************************************************************************************************/
#ifdef __cplusplus
  extern "C" {
#endif
#define  M0                                                     //Core M0
#define  u1     unsigned                                        //0...1
#define  u8     unsigned char                                   //0...255
#define  s8     signed char                                     //-128...127
#define  u16    unsigned short                                  //0...65535
#define  s16    signed short                                    //-32768...2767
#define  u32    unsigned int                                    //0...4294967295
#define  s32    signed int                                      //-2147483647...2147483647
#define  u64    unsigned long long                              //0...18446744073709551615
#define  s64    signed long long                                //-9223372036854775807...9223372036854775807
#define  f32    float                                           //±1.18e-38 to ±3.40e38 (exp 8bit (-126...127), mant 23bit)
#define  f64    double                                          //±2.23e-308 to ±1.8e308 (exp 11bit (-1022...1023), mant 52bit)
/**************************************************************************************************/
#define ON                      ((u8)0x01)
#define OFF                     ((u8)0x00)
/************************************* Core M0 memory map *****************************************/
#define FLASH_BASE              ((u32) 0x08000000)              //FLASH base address
#define UID_BASE                ((u32) 0x1FFFF7AC)              //CPU UID Base Address
#define OB_BASE                 ((u32) 0x1FFFF800)              //Option byte base address
#define SRAM_BASE               ((u32) 0x20000000)              //SRAM base address
#define APB_BASE                ((u32) 0x40000000)              //APB base address 
#define AHB1_BASE               ((u32) 0x40020000)              //AHB1 base address
#define AHB2_BASE               ((u32) 0x48000000)              //AHB2 base address
#define DWT_BASE                ((u32) 0xE0001000)              //Data Watchpoint and Trace unit Space Base Address
#define STK_BASE                ((u32) 0xE000E010)              //SysTick Base Address
#define NVIC_BASE               ((u32) 0xE000E100)              //NVIC Base Address
#define SCB_BASE                ((u32) 0xE000ED00)              //SCB Base Address
#define SYS_RESET *(u32*)(SCB_BASE + 12) = 0x05FA0004           //System reset
#define FLASH_SIZE (*(u16*)0x1FFFF7CC)                          //FLASH size
#define CAL_VINT (*(u16*)0x1FFFF7BA)                            //VREFINT calibration value
#define CAL_T110 (*(u16*)0x1FFFF7C2)                            //Temperature sensor calibration value acquired at 110°C
#define CAL_T030 (*(u16*)0x1FFFF7B8)                            //temperature sensor calibration value acquired at 30°C
/*********************************** Peripheral memory map ****************************************/
#define TIM02_BASE              (APB_BASE + 0x00000000)
#define TIM03_BASE              (APB_BASE + 0x00000400)
#define TIM06_BASE              (APB_BASE + 0x00001000)
#define TIM07_BASE              (APB_BASE + 0x00001400)
#define TIM14_BASE              (APB_BASE + 0x00002000)
#define RTC_BASE                (APB_BASE + 0x00002800)
#define WWDG_BASE               (APB_BASE + 0x00002C00)
#define IWDG_BASE               (APB_BASE + 0x00003000)
#define SPI2_BASE               (APB_BASE + 0x00003800)
#define UART2_BASE              (APB_BASE + 0x00004400)
#define UART3_BASE              (APB_BASE + 0x00004800)
#define UART4_BASE              (APB_BASE + 0x00004C00)
#define UART5_BASE              (APB_BASE + 0x00005000)
#define I2C1_BASE               (APB_BASE + 0x00005400)
#define I2C2_BASE               (APB_BASE + 0x00005800)
#define USB_BASE                (APB_BASE + 0x00005C00)
#define USB_RAM_BASE            (APB_BASE + 0x00006000)
#define CAN_BASE                (APB_BASE + 0x00006400)
#define CRS_BASE                (APB_BASE + 0x00006C00)
#define PWR_BASE                (APB_BASE + 0x00007000)
#define DAC_BASE                (APB_BASE + 0x00007400)
#define CEC_BASE                (APB_BASE + 0x00007800)
#define SYSCFG_BASE             (APB_BASE + 0x00010000)
#define EXTI_BASE               (APB_BASE + 0x00010400)
#define UART6_BASE              (APB_BASE + 0x00011400)
#define UART7_BASE              (APB_BASE + 0x00011800)
#define UART8_BASE              (APB_BASE + 0x00011C00)
#define ADC_BASE                (APB_BASE + 0x00012400)
#define TIM01_BASE              (APB_BASE + 0x00012C00)
#define SPI1_BASE               (APB_BASE + 0x00013000)
#define UART1_BASE              (APB_BASE + 0x00013800)
#define TIM15_BASE              (APB_BASE + 0x00014000)
#define TIM16_BASE              (APB_BASE + 0x00014400)
#define TIM17_BASE              (APB_BASE + 0x00014800)
#define DBGMCU_BASE             (APB_BASE + 0x00015800)   
#define DMA1_BASE               (AHB1_BASE + 0x00000000)
#define DMA2_BASE               (AHB1_BASE + 0x00000400)
#define RCC_BASE                (AHB1_BASE + 0x00001000)
#define FLASH_R_BASE            (AHB1_BASE + 0x00002000)
#define CRC_BASE                (AHB1_BASE + 0x00003000)
#define TSC_BASE                (AHB1_BASE + 0x00004000)
#define GPIOA_BASE              (AHB2_BASE + 0x00000000)
#define GPIOB_BASE              (AHB2_BASE + 0x00000400)
#define GPIOC_BASE              (AHB2_BASE + 0x00000800)
#define GPIOD_BASE              (AHB2_BASE + 0x00000C00)
#define GPIOE_BASE              (AHB2_BASE + 0x00001000)
#define GPIOF_BASE              (AHB2_BASE + 0x00001400)
/****************************************** IRQ ***************************************************/
typedef enum
{
  WWDG_IRQ                      = 0,  //Window WatchDog Interrupt
  PVD_IRQ                       = 1,  //PVD through EXTI Line detection Interrupt
  RTC_IRQ                       = 2,  //RTC global Interrupt
  FLASH_IRQ                     = 3,  //FLASH global Interrupt
  RCC_IRQ                       = 4,  //RCC global Interrupt
  EXTI01_IRQ                    = 5,  //EXTI Line0..1 Interrupt
  EXTI23_IRQ                    = 6,  //EXTI Line2..3 Interrupt
  EXTI415_IRQ                   = 7,  //EXTI Line4..15 Interrupt
  TSC_IRQ                       = 8,  //Touch sensing Interrupt
  DMA1_CH1_IRQ                  = 9,  //DMA1 Channel 1 Interrupt
  DMA1_CH23_DMA2_CH12_IRQ       = 10, //DMA1 Channel 2..3, DMA2 Channel 1..2 Interrupt
  DMA1_CH47_DMA2_CH35_IRQ       = 11, //DMA1 Channel 4..7, DMA2 Channel 3..5 Interrupt
  ADC1_COMP_IRQ                 = 12, //ADC and COMP Interrupt (ADC interrupt combined with EXTI lines 21 and 22)
  TIM1_IRQ                      = 13, //TIM1 break, update, trigger and commutation Interrupt
  TIM1_CC_IRQ                   = 14, //TIM1 capture compare Interrupt
  TIM2_IRQ                      = 15, //TIM2 Interrupt
  TIM3_IRQ                      = 16, //TIM3 Interrupt
  TIM6_DAC_IRQ                  = 17, //TIM6 and DAC underrun  Interrupt
  TIM7_IRQ                      = 18, //TIM7 Interrupt
  TIM14_IRQ                     = 19, //TIM14 Interrupt
  TIM15_IRQ                     = 20, //TIM15 Interrupt
  TIM16_HIRQ                    = 21, //TIM16 Interrupt
  TIM17_IRQ                     = 22, //TIM17 Interrupt
  I2C1_IRQ                      = 23, //I2C1 Interrupt (combined with EXTI line 23)
  I2C2_IRQ                      = 24, //I2C2 Interrupt
  SPI1_IRQ                      = 25, //SPI1 Interrupt
  SPI2_IRQ                      = 26, //SPI2 Interrupt
  UART1_IRQ                     = 27, //USART1 Interrupt
  UART2_IRQ                     = 28, //USART2 Interrupt
  UART38_IRQ                    = 29, //USART3..8 Interrupt (combined with EXTI line 28)
  CEC_CAN_IRQ                   = 30, //CEC and CAN Interrupt (combined with EXTI line 27)
  USB_IRQ                       = 31, //USB Interrupt (combined with EXTI line 18)
}IRQ_Type;
/***************************************** SysTick ************************************************/
typedef struct
{
  struct
  {
    u1 ENABLE     : 1;  //Counter enable
    u1 TICKINT    : 1;  //SysTick exception request enable
    u1 CLKSOURCE  : 1;  //Clock source selection
    u1 RSVD1      : 13; //Reserved
    u1 COUNTFLAG  : 1;  //Returns 1 if timer counted to 0 since last time this was read
    u1 RSVD0      : 15; //Reserved
  }volatile CSR;              //SysTick control and status register (STK_CSR)
  struct
  {
    u1 RELOAD     : 24; //RELOAD value
    u1 RSVD0      : 8;  //Reserved
  }volatile RVR;              //SysTick reload value register (STK_RVR)
  struct
  {
    u1 CURRENT    : 24; //Current counter value
    u1 RSVD0      : 8;  //Reserved
  }volatile CVR;              //SysTick current value register (STK_CVR)
  struct
  {
    u1 TENMS      : 24; //Calibration value
    u1 RSVD0      : 6;  //Reserved
    u1 SKEW       : 1;  //SKEW flag
    u1 NOREF      : 1;  //NOREF flag
  }volatile CALIB;            //SysTick calibration value register (STK_CALIB)
}STK_Type;
#define STK ((STK_Type*) STK_BASE)
#define STK_EN                  STK->CSR.ENABLE                 //Counter enable
#define STK_IRQ                 STK->CSR.TICKINT                //SysTick exception request enable
#define STK_CLK                 STK->CSR.CLKSOURCE              //Clock source selection
#define STK_FLG                 STK->CSR.COUNTFLAG              //Returns 1 if timer counted to 0 since last time this was read
#define STK_REL                 STK->RVR.RELOAD                 //RELOAD value
#define STK_CUR                 STK->CVR.CURRENT                //Current counter value
#define STK_TENMS               STK->CALIB.TENMS                //Calibration value
#define STK_SKEW                STK->CALIB.SKEW                 //SKEW flag
#define STK_NOREF               STK->CALIB.NOREF                //NOREF flag
/******************************************* SCB **************************************************/
typedef struct
{
  struct
  {
    u1 REV        : 4;  //Indicates patch release: 0x0 = Patch 0
    u1 PART       : 12; //Indicates part number: 0xC20 = Cortex-M0
    u1 CONST      : 4;  //Reads as 0xF
    u1 VAR        : 4;  //Indicates processor revision: 0x2 = Revision 2
    u1 IMP        : 8;  //Indicates implementor: 0x41 = ARM
  }volatile CPUID;      //CPUID base register (CPUID)
  struct
  {
    u1 VECTACTIVE : 6;  //Active ISR number field
    u1 RSVD0      : 6;  //Reserved
    u1 VECTPENDING: 6;  //Pending ISR number field
    u1 RSVD1      : 4;  //Reserved
    u1 ISRPENDING : 1;  //Interrupt pending flag
    u1 RSVD2      : 2;  //Reserved
    u1 PENDSTCLR  : 1;  //Clear pending SysTick bit
    u1 PENDSTSET  : 1;  //Set a pending SysTick bit
    u1 PENDSVCLR  : 1;  //Clear pending pendSV bit
    u1 PENDSVSET  : 1;  //Set pending pendSV bit
    u1 RSVD3      : 2;  //Reserved
    u1 NMIPENDSET : 1;  //Set pending NMI bit
  }volatile ICSR;       //Interrupt control and state register (ICSR)
  volatile u32 RSVD0;   //Reserved
  struct
  {
    u1 RSVD0      : 1;  //Reserved
    u1 VECTCLRACT : 1;  //Clear active vector bit
    u1 SYSRESETREQ: 1;  //Causes a signal to be asserted to the outer system that indicates a reset is requested
    u1 RSVD1      : 12; //Reserved
    u1 ENDIANESS  : 1;  //Data endianness bit (1 = big endian, 0 = little endian)
    u1 VECTKEY    : 16; //Register key. Writing to this register requires 0x05FA in the VECTKEY field. Otherwise the write value is ignored. Reads as 0xFA05
  }volatile AIRCR;      //Application interrupt and reset control register (AIRCR)
  struct
  {
    u1 RSVD0      : 1;  //Reserved
    u1 SLPONEXIT  : 1;  //Sleep on exit when returning from Handler mode to Thread mode
    u1 SLEEPDEEP  : 1;  //Sleep deep bit
    u1 RSVD1      : 1;  //Reserved
    u1 SEVONPEND  : 1;  //When enabled, this causes WFE to wake up when an interrupt moves from inactive to pended
    u1 RSVD2      : 27; //Reserved
  }volatile SCR;        //System control register (SCR)
  struct
  {
    u1 RSVD0      : 3;  //Reserved
    u1 UNALIGNTRP : 1;  //Trap for unaligned access
    u1 RSVD1      : 5;  //Reserved
    u1 STKALIGN   : 1;  //1 = 8-byte, 1 = 4-byte
    u1 RSVD2      : 22; //Reserved
  }volatile CCR;        //Configuration and control register (CCR)
  volatile u32 RSVD1;   //Reserved
  struct
  {
    u1 RSVD0      : 24; //Reserved
    u1 PRI_11     : 8;  //Priority of system handler 11, SVCall
  }volatile SHPR2;      //System handler priority registers (SHPR2)
  struct
  {
    u1 RSVD0      : 16; //Reserved
    u1 PRI_14     : 8;  //Priority of system handler 14, PendSV
    u1 PRI_15     : 8;  //Priority of system handler 15, SysTick exception
  }volatile SHPR3;      //System handler priority registers (SHPR3)
}SCB_Type;
#define SCB ((SCB_Type*) SCB_BASE)
#define SCB_REV                 SCB->CPUID.REV                  //Indicates patch release: 0x0 = Patch 0
#define SCB_PART                SCB->CPUID.PART                 //Indicates part number: 0xC23 = Cortex-M3
#define SCB_CONST               SCB->CPUID.CONST                //Reads as 0xF
#define SCB_VAR                 SCB->CPUID.VAR                  //Indicates processor revision: 0x2 = Revision 2
#define SCB_IMP                 SCB->CPUID.IMP                  //Indicates implementor: 0x41 = ARM
#define SCB_VECTACTIVE          SCB->ICSR.VECTACTIVE            //Active ISR number field
#define SCB_VECTPENDING         SCB->ICSR.VECTPENDING           //Pending ISR number field
#define SCB_ISRPENDING          SCB->ICSR.ISRPENDING            //Interrupt pending flag
#define SCB_PENDSTCLR           SCB->ICSR.PENDSTCLR             //Clear pending SysTick bit
#define SCB_PENDSTSET           SCB->ICSR.PENDSTSET             //Set a pending SysTick bit
#define SCB_PENDSVCLR           SCB->ICSR.PENDSVCLR             //Clear pending pendSV bit
#define SCB_PENDSVSET           SCB->ICSR.PENDSVSET             //Set pending pendSV bit
#define SCB_NMIPENDSET          SCB->ICSR.NMIPENDSET            //Set pending NMI bit
#define SCB_VECTCLRACT          SCB->AIRCR.VECTCLRACT           //Clear active vector bit
#define SCB_SYSRESET            SCB->AIRCR.SYSRESETREQ          //Causes a signal to be asserted to the outer system that indicates a reset is requested
#define SCB_ENDIANESS           SCB->AIRCR.ENDIANESS            //Data endianness bit (1 = big endian, 0 = little endian)
#define SCB_VECTKEY             SCB->AIRCR.VECTKEY              //Register key. Writing to this register requires 0x5FA in the VECTKEY field. Otherwise the write value is ignored. Reads as 0xFA05
#define SCB_SLPONEXIT           SCB->SCR.SLPONEXIT              //Sleep on exit when returning from Handler mode to Thread mode
#define SCB_SLEEPDEEP           SCB->SCR.SLEEPDEEP              //Sleep deep bit
#define SCB_SEVONPEND           SCB->SCR.SEVONPEND              //When enabled, this causes WFE to wake up when an interrupt moves from inactive to pended
#define SCB_UNALIGNTRP          SCB->CCR.UNALIGNTRP             //Trap for unaligned access
#define SCB_STKALIGN            SCB->CCR.STKALIGN               //1 = 8-byte, 1 = 4-byte
#define SCB_PRI_11              SCB->SHPR2.PRI_11               //Priority of system handler 11, SVCall
#define SCB_PRI_14              SCB->SHPR3.PRI_14               //Priority of system handler 14, PendSV
#define SCB_PRI_15              SCB->SHPR3.PRI_15               //Priority of system handler 15, SysTick exception
/******************************************* NVIC *************************************************/
typedef struct
{
  volatile u32 ISER[1];         //Interrupt set-enable registers (NVIC_ISERx)
  volatile u32 RSVD0[31];       //Reserved
  volatile u32 ICER[1];         //Interrupt clear-enable registers (NVIC_ICERx)
  volatile u32 RSVD1[31];       //Reserved
  volatile u32 ISPR[1];         //Interrupt set-pending registers (NVIC_ISPRx)
  volatile u32 RSVD2[31];       //Reserved
  volatile u32 ICPR[1];         //Interrupt clear-pending registers (NVIC_ICPRx)
  volatile u32 RSVD3[31];       //Reserved
  volatile u32 RSVD4[64];       //Reserved
  volatile u32 IPR[8];          //Interrupt priority register (IPR0-IPR7)
}NVIC_Type;
#define NVIC ((NVIC_Type*) NVIC_BASE)
/******************************************* RCC **************************************************/
typedef struct
{
  struct
  {
    u1 HSION      : 1;  //Internal high-speed clock enable
    u1 HSIRDY     : 1;  //Internal high-speed clock ready flag
    u1 RSVD0      : 1;  //Reserved
    u1 HSITRIM    : 5;  //Internal high-speed clock trimming
    u1 HSICAL     : 8;  //Internal high-speed clock calibration
    u1 HSEON      : 1;  //External high-speed clock enable
    u1 HSERDY     : 1;  //External high-speed clock ready flag
    u1 HSEBYP     : 1;  //External high-speed clock bypass
    u1 CSSON      : 1;  //Clock security system enable
    u1 RSVD1      : 4;  //Reserved
    u1 PLLON      : 1;  //PLL enable
    u1 PLLRDY     : 1;  //PLL clock ready flag
    u1 RSVD2      : 6;  //Reserved
  }volatile CR;         //Clock control register (RCC_CR)
  struct
  {
    u1 SW         : 2;  //System clock switch
    u1 SWS        : 2;  //System clock switch status
    u1 HPRE       : 4;  //AHB prescaler
    u1 PPRE       : 3;  //APB prescaler
    u1 RSVD0      : 3;  //Reserved
    u1 ADCPRE     : 1;  //ADC prescaler
    u1 PLLSRC     : 2;  //PLL entry clock source
    u1 PLLXTPRE   : 1;  //HSE divider for PLL entry
    u1 PLLMUL     : 4;  //PLL multiplication factor
    u1 RSVD1      : 2;  //Reserved
    u1 MCO        : 4;  //Microcontroller clock output
    u1 MCOPRE     : 3;  //Microcontroller Clock Output Prescaler
    u1 PLLNODIV   : 1;  //PLL clock not divided for MCO
  }volatile CFGR;       //Clock configuration register (RCC_CFGR)
  struct
  {
    u1 LSIRDYF    : 1;  //LSI ready interrupt flag
    u1 LSERDYF    : 1;  //LSE ready interrupt flag
    u1 HSIRDYF    : 1;  //HSI ready interrupt flag
    u1 HSERDYF    : 1;  //HSE ready interrupt flag
    u1 PLLRDYF    : 1;  //PLL ready interrupt flag
    u1 HSI14RDYF  : 1;  //HSI14 ready interrupt flag
    u1 HSI48RDYF  : 1;  //HSI48 ready interrupt flag
    u1 CSSF       : 1;  //Clock security system interrupt flag
    u1 LSIRDYIE   : 1;  //LSI ready interrupt enable
    u1 LSERDYIE   : 1;  //LSE ready interrupt enable
    u1 HSIRDYIE   : 1;  //HSI ready interrupt enable
    u1 HSERDYIE   : 1;  //HSE ready interrupt enable
    u1 PLLRDYIE   : 1;  //PLL ready interrupt enable
    u1 HSI14RDYIE : 1;  //HSI14 ready interrupt enable
    u1 HSI48RDYIE : 1;  //HSI48 ready interrupt enable
    u1 RSVD0      : 1;  //Reserved
    u1 LSIRDYC    : 1;  //LSI ready interrupt clear
    u1 LSERDYC    : 1;  //LSE ready interrupt clear
    u1 HSIRDYC    : 1;  //HSI ready interrupt clear
    u1 HSERDYC    : 1;  //HSE ready interrupt clear
    u1 PLLRDYC    : 1;  //PLL ready interrupt clear
    u1 HSI14RDYC  : 1;  //HSI14 ready interrupt clear
    u1 HSI48RDYC  : 1;  //HSI48 ready interrupt clear
    u1 CSSC       : 1;  //Clock security system interrupt clear
    u1 RSVD1      : 8;  //Reserved
  }volatile CIR;        //Clock interrupt register (RCC_CIR)
  struct
  {
    u1 SYSCFGRST  : 1;  //SYSCFG reset
    u1 RSVD0      : 4;  //Reserved
    u1 USART6RST  : 1;  //USART6 reset
    u1 USART7RST  : 1;  //USART7 reset
    u1 USART8RST  : 1;  //USART8 reset
    u1 RSVD1      : 1;  //Reserved
    u1 ADCRST     : 1;  //ADC interface reset
    u1 RSVD2      : 1;  //Reserved
    u1 TIM1RST    : 1;  //TIM1 timer reset
    u1 SPI1RST    : 1;  //SPI1 reset
    u1 RSVD3      : 1;  //Reserved
    u1 USART1RST  : 1;  //USART1 reset
    u1 RSVD4      : 1;  //Reserved
    u1 TIM15RST   : 1;  //TIM15 timer reset
    u1 TIM16RST   : 1;  //TIM16 timer reset
    u1 TIM17RST   : 1;  //TIM17 timer reset
    u1 RSVD5      : 3;  //Reserved
    u1 DBGMCURST  : 1;  //Debug MCU reset
    u1 RSVD6      : 9;  //Reserved
  }volatile APB2RSTR;   //APB2 peripheral reset register (RCC_APB2RSTR)
  struct
  {
    u1 TIM2RST    : 1;  //TIM2 timer reset
    u1 TIM3RST    : 1;  //TIM3 timer reset
    u1 RSVD0      : 2;  //Reserved
    u1 TIM6RST    : 1;  //TIM6 timer reset
    u1 TIM7RST    : 1;  //TIM7 timer reset
    u1 RSVD1      : 2;  //Reserved
    u1 TIM14RST   : 1;  //TIM14 timer reset
    u1 RSVD2      : 2;  //Reserved
    u1 WWDGRST    : 1;  //Window watchdog reset
    u1 RSVD3      : 2;  //Reserved
    u1 SPI2RST    : 1;  //SPI2 reset
    u1 RSVD4      : 2;  //Reserved
    u1 UART2RST   : 1;  //USART2 reset
    u1 UART3RST   : 1;  //USART3 reset
    u1 UART4RST   : 1;  //UART4 reset
    u1 UART5RST   : 1;  //UART5 reset
    u1 I2C1RST    : 1;  //I2C1 reset
    u1 I2C2RST    : 1;  //I2C2 reset
    u1 USBRST     : 1;  //USB reset
    u1 RSVD5      : 1;  //Reserved
    u1 CANRST     : 1;  //CAN reset
    u1 RSVD6      : 1;  //Reserved
    u1 CRSRST     : 1;  //Clock Recovery System interface reset
    u1 PWRRST     : 1;  //Power interface reset
    u1 DACRST     : 1;  //DAC interface reset
    u1 CECRST     : 1;  //HDMI CEC reset
    u1 RSVD7      : 1;  //Reserved
  }volatile APB1RSTR;   //APB1 peripheral reset register (RCC_APB1RSTR)
  struct
  {
    u1 DMA1EN     : 1;  //DMA1 clock enable
    u1 DMA2EN     : 1;  //DMA2 clock enable
    u1 SRAMEN     : 1;  //SRAM interface clock enable
    u1 RSVD0      : 1;  //Reserved
    u1 FLITFEN    : 1;  //FLITF clock enable
    u1 RSVD1      : 1;  //Reserved
    u1 CRCEN      : 1;  //CRC clock enable
    u1 RSVD2      : 10; //Reserved
    u1 IOPAEN     : 1;  //IO port A clock enable
    u1 IOPBEN     : 1;  //IO port B clock enable
    u1 IOPCEN     : 1;  //IO port C clock enable
    u1 IOPDEN     : 1;  //IO port D clock enable
    u1 IOPEEN     : 1;  //IO port E clock enable
    u1 IOPFEN     : 1;  //IO port F clock enable
    u1 RSVD3      : 1;  //Reserved
    u1 TSCEN      : 1;  //Touch sensing controller clock enable
    u1 RSVD4      : 7;  //Reserved
  }volatile AHBENR;     //AHB peripheral clock enable register (RCC_AHBENR)
  struct
  {
    u1 SYSCFGEN   : 1;  //SYSCFG enable
    u1 RSVD0      : 4;  //Reserved
    u1 USART6EN   : 1;  //USART6 enable
    u1 USART7EN   : 1;  //USART7 enable
    u1 USART8EN   : 1;  //USART8 enable
    u1 RSVD1      : 1;  //Reserved
    u1 ADCEN      : 1;  //ADC interface enable
    u1 RSVD2      : 1;  //Reserved
    u1 TIM1EN     : 1;  //TIM1 timer enable
    u1 SPI1EN     : 1;  //SPI1 enable
    u1 RSVD3      : 1;  //Reserved
    u1 USART1EN   : 1;  //USART1 enable
    u1 RSVD4      : 1;  //Reserved
    u1 TIM15EN    : 1;  //TIM15 timer enable
    u1 TIM16EN    : 1;  //TIM16 timer enable
    u1 TIM17EN    : 1;  //TIM17 timer enable
    u1 RSVD5      : 3;  //Reserved
    u1 DBGMCUEN   : 1;  //Debug MCU enable
    u1 RSVD6      : 9;  //Reserved
  }volatile APB2ENR;    //APB2 peripheral clock enable register (RCC_APB2ENR)
  struct
  {
    u1 TIM2EN     : 1;  //TIM2 timer enable
    u1 TIM3EN     : 1;  //TIM3 timer enable
    u1 RSVD0      : 2;  //Reserved
    u1 TIM6EN     : 1;  //TIM6 timer enable
    u1 TIM7EN     : 1;  //TIM7 timer enable
    u1 RSVD1      : 2;  //Reserved
    u1 TIM14EN    : 1;  //TIM14 timer enable
    u1 RSVD2      : 2;  //Reserved
    u1 WWDGEN     : 1;  //Window watchdog enable
    u1 RSVD3      : 2;  //Reserved
    u1 SPI2EN     : 1;  //SPI2 enable
    u1 RSVD4      : 2;  //Reserved
    u1 UART2EN    : 1;  //USART2 enable
    u1 UART3EN    : 1;  //USART3 enable
    u1 UART4EN    : 1;  //UART4 enable
    u1 UART5EN    : 1;  //UART5 enable
    u1 I2C1EN     : 1;  //I2C1 enable
    u1 I2C2EN     : 1;  //I2C2 enable
    u1 USBEN      : 1;  //USB enable
    u1 RSVD5      : 1;  //Reserved
    u1 CANEN      : 1;  //CAN enable
    u1 RSVD6      : 1;  //Reserved
    u1 CRSEN      : 1;  //Clock Recovery System interface enable
    u1 PWREN      : 1;  //Power interface enable
    u1 DACEN      : 1;  //DAC interface enable
    u1 CECEN      : 1;  //HDMI CEC enable
    u1 RSVD7      : 1;  //Reserved
  }volatile APB1ENR;    //APB1 peripheral clock enable register (RCC_APB1ENR)
  struct
  {
    u1 LSEON      : 1;  //External low-speed oscillator enable
    u1 LSERDY     : 1;  //External low-speed oscillator ready
    u1 LSEBYP     : 1;  //External low-speed oscillator bypass
    u1 LSEDRV     : 2;  //LSE oscillator drive capability
    u1 RSVD0      : 3;  //Reserved
    u1 RTCSEL     : 2;  //RTC clock source selection
    u1 RSVD1      : 5;  //Reserved
    u1 RTCEN      : 1;  //RTC clock enable
    u1 BDRST      : 1;  //Backup domain software reset
    u1 RSVD2      : 15; //Reserved
  }volatile BDCR;       //Backup domain control register (RCC_BDCR)
  struct
  {
    u1 LSION      : 1;  //Internal low-speed oscillator enable
    u1 LSIRDY     : 1;  //Internal low-speed oscillator ready
    u1 RSVD0      : 21; //Reserved
    u1 V18PWRRSTF : 1;  //Reset flag of the 1.8 V domain.
    u1 RMVF       : 1;  //Remove reset flag
    u1 OBLRSTF    : 1;  //Option byte loader reset flag
    u1 PINRSTF    : 1;  //PIN reset flag
    u1 PORRSTF    : 1;  //POR/PDR reset flag
    u1 SFTRSTF    : 1;  //Software reset flag
    u1 IWDGRSTF   : 1;  //Independent watchdog reset flag
    u1 WWDGRSTF   : 1;  //Window watchdog reset flag
    u1 LPWRRSTF   : 1;  //Low-power reset flag
  }volatile CSR;        //Control/status register (RCC_CSR)
  struct
  {
    u1 RSVD0      : 17; //Reserved
    u1 IOPARST    : 1;  //I/O port A reset
    u1 IOPBRST    : 1;  //I/O port B reset
    u1 IOPCRST    : 1;  //I/O port C reset
    u1 IOPDRST    : 1;  //I/O port D reset
    u1 IOPERST    : 1;  //I/O port E reset
    u1 IOPFRST    : 1;  //I/O port F reset
    u1 RSVD1      : 1;  //Reserved
    u1 TSCRST     : 1;  //Touch sensing controller reset
    u1 RSVD2      : 7;  //Reserved
  }volatile AHBRSTR;    //AHB peripheral reset register (RCC_AHBRSTR)
  struct
  {
    u1 PREDIV     : 4;  //PREDIV division factor
    u1 RSVD0      : 28; //Reserved
  }volatile CFGR2;      //Clock configuration register 2 (RCC_CFGR2)
  struct
  {
    u1 USART1SW   : 2;  //USART1 clock source selection
    u1 RSVD0      : 2;  //Reserved
    u1 I2C1SW     : 1;  //I2C1 clock source selection
    u1 RSVD1      : 1;  //Reserved
    u1 CECSW      : 1;  //HDMI CEC clock source selection
    u1 USBSW      : 1;  //USB clock source selection
    u1 ADCSW      : 1;  //ADC clock source selection
    u1 RSVD2      : 7;  //Reserved
    u1 USART2SW   : 2;  //USART2 clock source selection (available only on STM32F07x and STM32F09x devices)
    u1 USART3SW   : 2;  //USART3 clock source selection (available only on STM32F09x devices)
    u1 RSVD3      : 12; //Reserved
  }volatile CFGR3;      //Clock configuration register 3 (RCC_CFGR3)
  struct
  {
    u1 HSI14ON    : 1;  //HSI14 clock enable
    u1 HSI14RDY   : 1;  //HSI14 clock ready flag
    u1 HSI14DIS   : 1;  //HSI14 clock request from ADC disable
    u1 HSI14TRIM  : 5;  //HSI14 clock trimming
    u1 HSI14CAL   : 8;  //HSI14 clock calibration
    u1 HSI48ON    : 1;  //HSI48 clock enable
    u1 HSI48RDY   : 1;  //HSI48 clock ready flag
    u1 RSVD0      : 6;  //Reserved
    u1 HSI48CAL   : 8;  //HSI48 factory clock calibration
  }volatile CR2;        //Clock control register 2 (RCC_CR2)
}RCC_Type;
#define RCC ((RCC_Type*) RCC_BASE)
#define RCC_HSI_EN              RCC->CR.HSION                   //Internal high-speed clock enable
#define RCC_HSI_RDY             RCC->CR.HSIRDY                  //Internal high-speed clock ready flag
#define RCC_HSI_CLK             RCC->CR.HSITRIM                 //Internal high-speed clock trimming
#define RCC_HSI_CAL             RCC->CR.HSICAL                  //Internal high-speed clock calibration
#define RCC_HSE_EN              RCC->CR.HSEON                   //External high-speed clock enable
#define RCC_HSE_RDY             RCC->CR.HSERDY                  //External high-speed clock ready flag
#define RCC_HSE_BYP             RCC->CR.HSEBYP                  //External high-speed clock bypass
#define RCC_CSS_EN              RCC->CR.CSSON                   //Clock security system enable
#define RCC_PLL_EN              RCC->CR.PLLON                   //PLL enable
#define RCC_PLL_RDY             RCC->CR.PLLRDY                  //PLL clock ready flag
#define RCC_SYSCLK              RCC->CFGR.SW                    //System clock switch
#define RCC_SYSCLK_FLG          RCC->CFGR.SWS                   //System clock switch status
#define RCC_AHB_DIV             RCC->CFGR.HPRE                  //AHB prescaler
#define RCC_APB_DIV             RCC->CFGR.PPRE                  //APB prescaler
#define RCC_ADC_DIV             RCC->CFGR.ADCPRE                //ADC prescaler
#define RCC_PLL_CLK             RCC->CFGR.PLLSRC                //PLL entry clock source
#define RCC_PLL_MUX             RCC->CFGR.PLLMUL                //PLL multiplication factor
#define RCC_MCO_CLK             RCC->CFGR.MCO                   //Microcontroller clock output
#define RCC_MCO_DIV             RCC->CFGR.MCOPRE                //Microcontroller Clock Output Prescaler
#define RCC_MCO_PLL_DIV         RCC->CFGR.PLLNODIV              //PLL clock not divided for MCO
#define RCC_LSI_IRQ_FLG         RCC->CIR.LSIRDYF                //LSI ready interrupt flag
#define RCC_LSE_IRQ_FLG         RCC->CIR.LSERDYF                //LSE ready interrupt flag
#define RCC_HSI_IRQ_FLG         RCC->CIR.HSIRDYF                //HSI ready interrupt flag
#define RCC_HSE_IRQ_FLG         RCC->CIR.HSERDYF                //HSE ready interrupt flag
#define RCC_PLL_IRQ_FLG         RCC->CIR.PLLRDYF                //PLL ready interrupt flag
#define RCC_HSI14_IRQ_FLG       RCC->CIR.HSI14RDYF              //HSI14 ready interrupt flag
#define RCC_HSI48_IRQ_FLG       RCC->CIR.HSI48RDYF              //HSI48 ready interrupt flag
#define RCC_CSS_IRQ_FLG         RCC->CIR.CSSF                   //Clock security system interrupt flag
#define RCC_LSI_IRQ             RCC->CIR.LSIRDYIE               //LSI ready interrupt enable
#define RCC_LSE_IRQ             RCC->CIR.LSERDYIE               //LSE ready interrupt enable
#define RCC_HSI_IRQ             RCC->CIR.HSIRDYIE               //HSI ready interrupt enable
#define RCC_HSE_IRQ             RCC->CIR.HSERDYIE               //HSE ready interrupt enable
#define RCC_PLL_IRQ             RCC->CIR.PLLRDYIE               //PLL ready interrupt enable
#define RCC_HSI14_IRQ           RCC->CIR.HSI14RDYIE             //HSI14 ready interrupt enable
#define RCC_HSI48_IRQ           RCC->CIR.HSI48RDYIE             //HSI48 ready interrupt enable
#define RCC_LSI_IRQ_CLR         RCC->CIR.LSIRDYC                //LSI ready interrupt clear
#define RCC_LSE_IRQ_CLR         RCC->CIR.LSERDYC                //LSE ready interrupt clear
#define RCC_HSI_IRQ_CLR         RCC->CIR.HSIRDYC                //HSI ready interrupt clear
#define RCC_HSE_IRQ_CLR         RCC->CIR.HSERDYC                //HSE ready interrupt clear
#define RCC_PLL_IRQ_CLR         RCC->CIR.PLLRDYC                //PLL ready interrupt clear
#define RCC_HSI14_IRQ_CLR       RCC->CIR.HSI14RDYC              //HSI14 ready interrupt clear
#define RCC_HSI48_IRQ_CLR       RCC->CIR.HSI48RDYC              //HSI48 ready interrupt clear
#define RCC_CSS_IRQ_CLR         RCC->CIR.CSSC                   //Clock security system interrupt clear
#define RCC_SYS_RST             RCC->APB2RSTR.SYSCFGRST         //SYSCFG reset
#define RCC_USART6_RST          RCC->APB2RSTR.USART6RST         //USART6 reset
#define RCC_USART7_RST          RCC->APB2RSTR.USART7RST         //USART7 reset
#define RCC_USART8_RST          RCC->APB2RSTR.USART8RST         //USART8 reset
#define RCC_ADC_RST             RCC->APB2RSTR.ADCRST            //ADC interface reset
#define RCC_TIM1_RST            RCC->APB2RSTR.TIM1RST           //TIM1 timer reset
#define RCC_SPI1_RST            RCC->APB2RSTR.SPI1RST           //SPI1 reset
#define RCC_USART1_RST          RCC->APB2RSTR.USART1RST         //USART1 reset
#define RCC_TIM15_RST           RCC->APB2RSTR.TIM15RST          //TIM15 timer reset
#define RCC_TIM16_RST           RCC->APB2RSTR.TIM16RST          //TIM16 timer reset
#define RCC_TIM17_RST           RCC->APB2RSTR.TIM17RST          //TIM17 timer reset
#define RCC_DBGMCU_RST          RCC->APB2RSTR.DBGMCURST         //Debug MCU reset
#define RCC_TIM2_RST            RCC->APB1RSTR.TIM2RST           //TIM2 timer reset
#define RCC_TIM3_RST            RCC->APB1RSTR.TIM3RST           //TIM3 timer reset
#define RCC_TIM6_RST            RCC->APB1RSTR.TIM6RST           //TIM6 timer reset
#define RCC_TIM7_RST            RCC->APB1RSTR.TIM7RST           //TIM7 timer reset
#define RCC_TIM14_RST           RCC->APB1RSTR.TIM14RST          //TIM14 timer reset
#define RCC_WWDG_RST            RCC->APB1RSTR.WWDGRST           //Window watchdog reset
#define RCC_SPI2_RST            RCC->APB1RSTR.SPI2RST           //SPI2 reset
#define RCC_UART2_RST           RCC->APB1RSTR.UART2RST          //USART2 reset
#define RCC_UART3_RST           RCC->APB1RSTR.UART3RST          //USART3 reset
#define RCC_UART4_RST           RCC->APB1RSTR.UART4RST          //UART4 reset
#define RCC_UART5_RST           RCC->APB1RSTR.UART5RST          //UART5 reset
#define RCC_I2C1_RST            RCC->APB1RSTR.I2C1RST           //I2C1 reset
#define RCC_I2C2_RST            RCC->APB1RSTR.I2C2RST           //I2C2 reset
#define RCC_USB_RST             RCC->APB1RSTR.USBRST            //USB reset
#define RCC_CAN_RST             RCC->APB1RSTR.CANRST            //CAN reset
#define RCC_CRS_RST             RCC->APB1RSTR.CRSRST            //Clock Recovery System interface reset
#define RCC_PWR_RST             RCC->APB1RSTR.PWRRST            //Power interface reset
#define RCC_DAC_RST             RCC->APB1RSTR.DACRST            //DAC interface reset
#define RCC_CEC_RST             RCC->APB1RSTR.CECRST            //HDMI CEC reset
#define RCC_DMA1_EN             RCC->AHBENR.DMA1EN              //DMA1 clock enable
#define RCC_DMA2_EN             RCC->AHBENR.DMA2EN              //DMA2 clock enable
#define RCC_SRAM_EN             RCC->AHBENR.SRAMEN              //SRAM interface clock enable
#define RCC_FLITF_EN            RCC->AHBENR.FLITFEN             //FLITF clock enable
#define RCC_CRC_EN              RCC->AHBENR.CRCEN               //CRC clock enable
#define RCC_GPIOA_EN            RCC->AHBENR.IOPAEN              //IO port A clock enable
#define RCC_GPIOB_EN            RCC->AHBENR.IOPBEN              //IO port B clock enable
#define RCC_GPIOC_EN            RCC->AHBENR.IOPCEN              //IO port C clock enable
#define RCC_GPIOD_EN            RCC->AHBENR.IOPDEN              //IO port D clock enable
#define RCC_GPIOE_EN            RCC->AHBENR.IOPEEN              //IO port E clock enable
#define RCC_GPIOF_EN            RCC->AHBENR.IOPFEN              //IO port F clock enable
#define RCC_TSC_EN              RCC->AHBENR.TSCEN               //Touch sensing controller clock enable
#define RCC_SYS_EN              RCC->APB2ENR.SYSCFGEN           //SYSCFG enable
#define RCC_UART6_EN            RCC->APB2ENR.USART6EN           //USART6 enable
#define RCC_UART7_EN            RCC->APB2ENR.USART7EN           //USART7 enable
#define RCC_UART8_EN            RCC->APB2ENR.USART8EN           //USART8 enable
#define RCC_ADC_EN              RCC->APB2ENR.ADCEN              //ADC interface enable
#define RCC_TIM1_EN             RCC->APB2ENR.TIM1EN             //TIM1 timer enable
#define RCC_SPI1_EN             RCC->APB2ENR.SPI1EN             //SPI1 enable
#define RCC_UART1_EN            RCC->APB2ENR.USART1EN           //USART1 enable
#define RCC_TIM15_EN            RCC->APB2ENR.TIM15EN            //TIM15 timer enable
#define RCC_TIM16_EN            RCC->APB2ENR.TIM16EN            //TIM16 timer enable
#define RCC_TIM17_EN            RCC->APB2ENR.TIM17EN            //TIM17 timer enable
#define RCC_DBGMCU_EN           RCC->APB2ENR.DBGMCUEN           //Debug MCU enable
#define RCC_TIM2_EN             RCC->APB1ENR.TIM2EN             //TIM2 timer enable
#define RCC_TIM3_EN             RCC->APB1ENR.TIM3EN             //TIM3 timer enable
#define RCC_TIM6_EN             RCC->APB1ENR.TIM6EN             //TIM6 timer enable
#define RCC_TIM7_EN             RCC->APB1ENR.TIM7REN            //TIM7 timer enable
#define RCC_TIM14_EN            RCC->APB1ENR.TIM14EN            //TIM14 timer enable
#define RCC_WWDG_EN             RCC->APB1ENR.WWDGEN             //Window watchdog enable
#define RCC_SPI2_EN             RCC->APB1ENR.SPI2EN             //SPI2 enable
#define RCC_UART2_EN            RCC->APB1ENR.UART2EN            //USART2 enable
#define RCC_UART3_EN            RCC->APB1ENR.UART3EN            //USART3 enable
#define RCC_UART4_EN            RCC->APB1ENR.UART4EN            //UART4 enable
#define RCC_UART5_EN            RCC->APB1ENR.UART5EN            //UART5 enable
#define RCC_I2C1_EN             RCC->APB1ENR.I2C1EN             //I2C1 enable
#define RCC_I2C2_EN             RCC->APB1ENR.I2C2EN             //I2C2 enable
#define RCC_USB_EN              RCC->APB1ENR.USBEN              //USB enable
#define RCC_CAN_EN              RCC->APB1ENR.CANEN              //CAN reset
#define RCC_CRS_EN              RCC->APB1ENR.CRSEN              //Clock Recovery System interface enable
#define RCC_PWR_EN              RCC->APB1ENR.PWREN              //Power interface enable
#define RCC_DAC_EN              RCC->APB1ENR.DACEN              //DAC interface enable
#define RCC_CEC_EN              RCC->APB1ENR.CECEN              //HDMI CEC enable
#define RCC_LSE_EN              RCC->BDCR.LSEON                 //External low-speed oscillator enable
#define RCC_LSE_RDY             RCC->BDCR.LSERDY                //External low-speed oscillator ready
#define RCC_LSE_BYP             RCC->BDCR.LSEBYP                //External low-speed oscillator bypass
#define RCC_LSE_DRV             RCC->BDCR.LSEDRV                //LSE oscillator drive capability
#define RCC_RTC_CLK             RCC->BDCR.RTCSEL                //RTC clock source selection
#define RCC_RTC_EN              RCC->BDCR.RTCEN                 //RTC clock enable
#define RCC_RTC_RST             RCC->BDCR.BDRST                 //Backup domain software reset
#define RCC_LSI_EN              RCC->CSR.LSION                  //Internal low-speed oscillator enable
#define RCC_LSI_RDY             RCC->CSR.LSIRDY                 //Internal low-speed oscillator ready
#define RCC_V18RST_FLG          RCC->CSR.V18PWRRSTF             //Reset flag of the 1.8 V domain.
#define RCC_RMVRST_FLG          RCC->CSR.RMVF                   //Remove reset flag
#define RCC_OBLRST_FLG          RCC->CSR.OBLRSTF                //Option byte loader reset flag
#define RCC_PINRST_FLG          RCC->CSR.PINRSTF                //PIN reset flag
#define RCC_PORRST_FLG          RCC->CSR.PORRSTF                //POR/PDR reset flag
#define RCC_SFTRST_FLG          RCC->CSR.SFTRSTF                //Software reset flag
#define RCC_IWDGRST_FLG         RCC->CSR.IWDGRSTF               //Independent watchdog reset flag
#define RCC_WWDGRST_FLG         RCC->CSR.WWDGRSTF               //Window watchdog reset flag
#define RCC_LPWRRST_FLG         RCC->CSR.LPWRRSTF               //Low-power reset flag
#define RCC_GPIOA_RST           RCC->AHBRSTR.IOPARST            //I/O port A reset
#define RCC_GPIOB_RST           RCC->AHBRSTR.IOPBRST            //I/O port B reset
#define RCC_GPIOC_RST           RCC->AHBRSTR.IOPCRST            //I/O port C reset
#define RCC_GPIOD_RST           RCC->AHBRSTR.IOPDRST            //I/O port D reset
#define RCC_GPIOE_RST           RCC->AHBRSTR.IOPERST            //I/O port E reset
#define RCC_GPIOF_RST           RCC->AHBRSTR.IOPFRST            //I/O port F reset
#define RCC_TSC_RST             RCC->AHBRSTR.TSCRST             //Touch sensing controller reset
#define RCC_HSE_DIV             RCC->CFGR2.PREDIV               //PREDIV division factor
#define RCC_USART1_SW           RCC->CFGR3.USART1SW             //USART1 clock source selection
#define RCC_I2C1_SW             RCC->CFGR3.I2C1SW               //I2C1 clock source selection
#define RCC_CEC_SW              RCC->CFGR3.CECSW                //HDMI CEC clock source selection
#define RCC_USB_SW              RCC->CFGR3.USBSW                //USB clock source selection
#define RCC_ADC_SW              RCC->CFGR3.ADCSW                //ADC clock source selection
#define RCC_USART2_SW           RCC->CFGR3.USART2SW             //USART2 clock source selection (available only on STM32F07x and STM32F09x devices)
#define RCC_USART3_SW           RCC->CFGR3.USART3SW             //USART3 clock source selection (available only on STM32F09x devices)
#define RCC_HSI14_EN            RCC->CR2.HSI14ON                //HSI14 clock enable
#define RCC_HSI14_RDY           RCC->CR2.HSI14RDY               //HSI14 clock ready flag
#define RCC_HSI14_DIS           RCC->CR2.HSI14DIS               //HSI14 clock request from ADC disable
#define RCC_HSI14_TRIM          RCC->CR2.HSI14TRIM              //HSI14 clock trimming
#define RCC_HSI14_CAL           RCC->CR2.HSI14CAL               //HSI14 clock calibration
#define RCC_HSI48_EN            RCC->CR2.HSI48ON                //HSI48 clock enable
#define RCC_HSI48_RDY           RCC->CR2.HSI48RDY               //HSI48 clock ready flag
#define RCC_HSI48_CAL           RCC->CR2.HSI48CAL               //HSI48 factory clock calibration
#define RCC_HSI_CLK_7360000     ((u8) 0x00)
#define RCC_HSI_CLK_7400000     ((u8) 0x01)
#define RCC_HSI_CLK_7440000     ((u8) 0x02)
#define RCC_HSI_CLK_7480000     ((u8) 0x03)
#define RCC_HSI_CLK_7520000     ((u8) 0x04)
#define RCC_HSI_CLK_7560000     ((u8) 0x05)
#define RCC_HSI_CLK_7600000     ((u8) 0x06)
#define RCC_HSI_CLK_7640000     ((u8) 0x07)
#define RCC_HSI_CLK_7680000     ((u8) 0x08)
#define RCC_HSI_CLK_7720000     ((u8) 0x09)
#define RCC_HSI_CLK_7760000     ((u8) 0x0A)
#define RCC_HSI_CLK_7800000     ((u8) 0x0B)
#define RCC_HSI_CLK_7840000     ((u8) 0x0C)
#define RCC_HSI_CLK_7880000     ((u8) 0x0D)
#define RCC_HSI_CLK_7920000     ((u8) 0x0E)
#define RCC_HSI_CLK_7960000     ((u8) 0x0F)
#define RCC_HSI_CLK_8000000     ((u8) 0x10)
#define RCC_HSI_CLK_8040000     ((u8) 0x11)
#define RCC_HSI_CLK_8080000     ((u8) 0x12)
#define RCC_HSI_CLK_8120000     ((u8) 0x13)
#define RCC_HSI_CLK_8160000     ((u8) 0x14)
#define RCC_HSI_CLK_8200000     ((u8) 0x15)
#define RCC_HSI_CLK_8240000     ((u8) 0x16)
#define RCC_HSI_CLK_8280000     ((u8) 0x17)
#define RCC_HSI_CLK_8320000     ((u8) 0x18)
#define RCC_HSI_CLK_8360000     ((u8) 0x19)
#define RCC_HSI_CLK_8400000     ((u8) 0x1A)
#define RCC_HSI_CLK_8440000     ((u8) 0x1B)
#define RCC_HSI_CLK_8480000     ((u8) 0x1C)
#define RCC_HSI_CLK_8520000     ((u8) 0x1D)
#define RCC_HSI_CLK_8560000     ((u8) 0x1E)
#define RCC_HSI_CLK_8600000     ((u8) 0x1F)
#define RCC_SYS_HSI             ((u8) 0x00)                     //HSI selected as system clock
#define RCC_SYS_HSE             ((u8) 0x01)                     //HSE selected as system clock
#define RCC_SYS_PLL             ((u8) 0x02)                     //PLL selected as system clock
#define RCC_SYS_HSI48           ((u8) 0x03)                     //HSI48 selected as system clock (when available)
#define RCC_AHB_DIV001          ((u8) 0x00)                     //SYSCLK not divided
#define RCC_AHB_DIV002          ((u8) 0x08)                     //SYSCLK divided by 2
#define RCC_AHB_DIV004          ((u8) 0x09)                     //SYSCLK divided by 4
#define RCC_AHB_DIV008          ((u8) 0x0A)                     //SYSCLK divided by 8
#define RCC_AHB_DIV016          ((u8) 0x0B)                     //SYSCLK divided by 16
#define RCC_AHB_DIV064          ((u8) 0x0C)                     //SYSCLK divided by 64
#define RCC_AHB_DIV128          ((u8) 0x0D)                     //SYSCLK divided by 128
#define RCC_AHB_DIV256          ((u8) 0x0E)                     //SYSCLK divided by 256
#define RCC_AHB_DIV512          ((u8) 0x0F)                     //SYSCLK divided by 512
#define RCC_APB_DIV01           ((u8) 0x00)                     //HCLK not divided
#define RCC_APB_DIV02           ((u8) 0x04)                     //HCLK divided by 2
#define RCC_APB_DIV04           ((u8) 0x05)                     //HCLK divided by 4
#define RCC_APB_DIV08           ((u8) 0x06)                     //HCLK divided by 8
#define RCC_APB_DIV16           ((u8) 0x07)                     //HCLK divided by 16
#define RCC_ADC_DIV2            ((u8) 0x00)                     //PCLK2 divided by 2
#define RCC_ADC_DIV4            ((u8) 0x01)                     //PCLK2 divided by 4
#define RCC_ADC_DIV6            ((u8) 0x02)                     //PCLK2 divided by 6
#define RCC_ADC_DIV8            ((u8) 0x03)                     //PCLK2 divided by 8
#define RCC_PLLSRC_HSI          ((u8) 0x00)                     //HSI/2 selected as PLL input clock
#define RCC_PLLSRC_HSIDIV       ((u8) 0x01)                     //HSI/PREDIV selected as PLL input clock
#define RCC_PLLSRC_HSE          ((u8) 0x02)                     //HSE/PREDIV selected as PLL input clock
#define RCC_PLLSRC_HSI48        ((u8) 0x03)                     //HSI48/PREDIV selected as PLL input clock
#define RCC_DIV01               ((u8) 0x00)                     //HSE clock not divided
#define RCC_DIV02               ((u8) 0x01)                     //HSE clock divided by 2
#define RCC_DIV03               ((u8) 0x02)                     //HSE clock divided by 3
#define RCC_DIV04               ((u8) 0x03)                     //HSE clock divided by 4
#define RCC_DIV05               ((u8) 0x04)                     //HSE clock divided by 5
#define RCC_DIV06               ((u8) 0x05)                     //HSE clock divided by 6
#define RCC_DIV07               ((u8) 0x06)                     //HSE clock divided by 7
#define RCC_DIV08               ((u8) 0x07)                     //HSE clock divided by 8
#define RCC_DIV09               ((u8) 0x08)                     //HSE clock divided by 9
#define RCC_DIV10               ((u8) 0x09)                     //HSE clock divided by 10
#define RCC_DIV11               ((u8) 0x0A)                     //HSE clock divided by 11
#define RCC_DIV12               ((u8) 0x0B)                     //HSE clock divided by 12
#define RCC_DIV13               ((u8) 0x0C)                     //HSE clock divided by 13
#define RCC_DIV14               ((u8) 0x0D)                     //HSE clock divided by 14
#define RCC_DIV15               ((u8) 0x0E)                     //HSE clock divided by 15
#define RCC_DIV16               ((u8) 0x0F)                     //HSE clock divided by 16
#define RCC_PLL_MUX02           ((u8) 0x00)                     //PLL input clock x 2
#define RCC_PLL_MUX03           ((u8) 0x01)                     //PLL input clock x 3
#define RCC_PLL_MUX04           ((u8) 0x02)                     //PLL input clock x 4
#define RCC_PLL_MUX05           ((u8) 0x03)                     //PLL input clock x 5
#define RCC_PLL_MUX06           ((u8) 0x04)                     //PLL input clock x 6
#define RCC_PLL_MUX07           ((u8) 0x05)                     //PLL input clock x 7
#define RCC_PLL_MUX08           ((u8) 0x06)                     //PLL input clock x 8
#define RCC_PLL_MUX09           ((u8) 0x07)                     //PLL input clock x 9
#define RCC_PLL_MUX10           ((u8) 0x08)                     //PLL input clock x 10
#define RCC_PLL_MUX11           ((u8) 0x09)                     //PLL input clock x 11
#define RCC_PLL_MUX12           ((u8) 0x0A)                     //PLL input clock x 12
#define RCC_PLL_MUX13           ((u8) 0x0B)                     //PLL input clock x 13
#define RCC_PLL_MUX14           ((u8) 0x0C)                     //PLL input clock x 14
#define RCC_PLL_MUX15           ((u8) 0x0D)                     //PLL input clock x 15
#define RCC_PLL_MUX16           ((u8) 0x0E)                     //PLL input clock x 16
#define RCC_USB_HSI48           ((u8) 0x00)                     //HSI48 clock selected as USB clock source (default)
#define RCC_USB_PLLCLK          ((u8) 0x01)                     //PLL clock (PLLCLK) selected as USB clock
#define RCC_MCO_NOCLK           ((u8) 0x00)                     //MCO output disabled, no clock on MCO
#define RCC_MCO_HSI14           ((u8) 0x01)                     //Internal RC 14 MHz (HSI14) oscillator clock selected
#define RCC_MCO_LSI             ((u8) 0x02)                     //Internal low speed (LSI) oscillator clock selected
#define RCC_MCO_LSE             ((u8) 0x03)                     //External low speed (LSE) oscillator clock selected
#define RCC_MCO_SYS             ((u8) 0x04)                     //System clock (SYSCLK) selected
#define RCC_MCO_HSI             ((u8) 0x05)                     //HSI clock selected
#define RCC_MCO_HSE             ((u8) 0x06)                     //HSE clock selected
#define RCC_MCO_PLL             ((u8) 0x07)                     //PLL clock selected (divided by 1 or 2, depending on PLLNODIV)
#define RCC_MCO_HSI48           ((u8) 0x08)                     //Internal RC 48 MHz (HSI48) oscillator clock selected
#define RCC_MCO_DIV001          ((u8) 0x00)                     //MCO is divided by 1
#define RCC_MCO_DIV002          ((u8) 0x01)                     //MCO is divided by 2
#define RCC_MCO_DIV004          ((u8) 0x02)                     //MCO is divided by 4
#define RCC_MCO_DIV008          ((u8) 0x03)                     //MCO is divided by 8
#define RCC_MCO_DIV016          ((u8) 0x04)                     //MCO is divided by 16
#define RCC_MCO_DIV032          ((u8) 0x05)                     //MCO is divided by 32
#define RCC_MCO_DIV064          ((u8) 0x06)                     //MCO is divided by 64
#define RCC_MCO_DIV128          ((u8) 0x07)                     //MCO is divided by 128
#define RCC_LSE_CAP1            ((u8) 0x00)                     //lower driving capability
#define RCC_LSE_CAP2            ((u8) 0x01)                     //medium low driving capability
#define RCC_LSE_CAP3            ((u8) 0x02)                     //medium high driving capability
#define RCC_LSE_CAP4            ((u8) 0x03)                     //higher driving capability (reset value)
#define RCC_USART_PCLK          ((u8) 0x00)                     //PCLK selected as USART3 clock source (default)
#define RCC_USART_SYSCLK        ((u8) 0x01)                     //System clock (SYSCLK) selected as USART3 clock
#define RCC_USART_LSE           ((u8) 0x02)                     //LSE clock selected as USART3 clock
#define RCC_USART_HSI           ((u8) 0x03)                     //HSI clock selected as USART3 clock
#define RCC_RTC_OFF             ((u8) 0x00)                     //No clock
#define RCC_RTC_LSE             ((u8) 0x01)                     //LSE oscillator clock used as RTC clock
#define RCC_RTC_LSI             ((u8) 0x02)                     //LSI oscillator clock used as RTC clock
#define RCC_RTC_HSE             ((u8) 0x03)                     //HSE oscillator clock divided by 32 used as RTC clock
/****************************************** GPIO **************************************************/
typedef struct
{
  struct
  {
    u1 MD00       : 2;  //Port x configuration bits 0
    u1 MD01       : 2;  //Port x configuration bits 1
    u1 MD02       : 2;  //Port x configuration bits 2
    u1 MD03       : 2;  //Port x configuration bits 3
    u1 MD04       : 2;  //Port x configuration bits 4
    u1 MD05       : 2;  //Port x configuration bits 5
    u1 MD06       : 2;  //Port x configuration bits 6
    u1 MD07       : 2;  //Port x configuration bits 7
    u1 MD08       : 2;  //Port x configuration bits 8
    u1 MD09       : 2;  //Port x configuration bits 9
    u1 MD10       : 2;  //Port x configuration bits 10
    u1 MD11       : 2;  //Port x configuration bits 11
    u1 MD12       : 2;  //Port x configuration bits 12
    u1 MD13       : 2;  //Port x configuration bits 13
    u1 MD14       : 2;  //Port x configuration bits 14
    u1 MD15       : 2;  //Port x configuration bits 15
  }volatile MODER;            //GPIO port mode register (GPIOx_MODER) (x=A..F)
  struct
  {
    u1 OT00       : 1;  //Port x configuration bits 0
    u1 OT01       : 1;  //Port x configuration bits 1
    u1 OT02       : 1;  //Port x configuration bits 2
    u1 OT03       : 1;  //Port x configuration bits 3
    u1 OT04       : 1;  //Port x configuration bits 4
    u1 OT05       : 1;  //Port x configuration bits 5
    u1 OT06       : 1;  //Port x configuration bits 6
    u1 OT07       : 1;  //Port x configuration bits 7
    u1 OT08       : 1;  //Port x configuration bits 8
    u1 OT09       : 1;  //Port x configuration bits 9
    u1 OT10       : 1;  //Port x configuration bits 10
    u1 OT11       : 1;  //Port x configuration bits 11
    u1 OT12       : 1;  //Port x configuration bits 12
    u1 OT13       : 1;  //Port x configuration bits 13
    u1 OT14       : 1;  //Port x configuration bits 14
    u1 OT15       : 1;  //Port x configuration bits 15
    u1 RSVD0      : 16; //Reserved
  }volatile OTYPER;           //GPIO port output type register (GPIOx_OTYPER) (x=A..F)
  struct
  {
    u1 SP00       : 2;  //Port x configuration bits 0
    u1 SP01       : 2;  //Port x configuration bits 1
    u1 SP02       : 2;  //Port x configuration bits 2
    u1 SP03       : 2;  //Port x configuration bits 3
    u1 SP04       : 2;  //Port x configuration bits 4
    u1 SP05       : 2;  //Port x configuration bits 5
    u1 SP06       : 2;  //Port x configuration bits 6
    u1 SP07       : 2;  //Port x configuration bits 7
    u1 SP08       : 2;  //Port x configuration bits 8
    u1 SP09       : 2;  //Port x configuration bits 9
    u1 SP10       : 2;  //Port x configuration bits 10
    u1 SP11       : 2;  //Port x configuration bits 11
    u1 SP12       : 2;  //Port x configuration bits 12
    u1 SP13       : 2;  //Port x configuration bits 13
    u1 SP14       : 2;  //Port x configuration bits 14
    u1 SP15       : 2;  //Port x configuration bits 15
  }volatile OSPEEDR;          //GPIO port output speed register (GPIOx_OSPEEDR) (x=A..F)
  struct
  {
    u1 PP00       : 2;  //Port x configuration bits 0
    u1 PP01       : 2;  //Port x configuration bits 1
    u1 PP02       : 2;  //Port x configuration bits 2
    u1 PP03       : 2;  //Port x configuration bits 3
    u1 PP04       : 2;  //Port x configuration bits 4
    u1 PP05       : 2;  //Port x configuration bits 5
    u1 PP06       : 2;  //Port x configuration bits 6
    u1 PP07       : 2;  //Port x configuration bits 7
    u1 PP08       : 2;  //Port x configuration bits 8
    u1 PP09       : 2;  //Port x configuration bits 9
    u1 PP10       : 2;  //Port x configuration bits 10
    u1 PP11       : 2;  //Port x configuration bits 11
    u1 PP12       : 2;  //Port x configuration bits 12
    u1 PP13       : 2;  //Port x configuration bits 13
    u1 PP14       : 2;  //Port x configuration bits 14
    u1 PP15       : 2;  //Port x configuration bits 15
  }volatile PUPDR;            //GPIO port pull-up/pull-down register (GPIOx_PUPDR) (x=A..F)
  struct
  {
    u1 IR         : 16; //Port input data register
    u1 RSVD0      : 16; //Reserved
  }volatile IDR;              //Port input data register (GPIO_IDR) (x=A..G)
  struct
  {
    u1 OR         : 16; //Port output data register
    u1 RSVD0      : 16; //Reserved
  }volatile ODR;              //Port output data register (GPIO_ODR) (x=A..G)
  struct
  {
    u1 BS         : 16; //Port x Set bit y (y= 0 .. 15)
    u1 BR         : 16; //Port x Reset bit y (y= 0 .. 15)
  }volatile BSRR;             //Port bit set/reset register (GPIO_BSRR) (x=A..G)
  struct
  {
    u1 LCK        : 16; //Port x Lock bit y (y= 0 .. 15)
    u1 LCKK       : 1;  //Lock key
    u1 RSVD0      : 15; //Reserved
  }volatile LCKR;             //Port configuration lock register (GPIO_LCKR) (x=A..G)
  struct
  {
    u1 AF00       : 4;  // Alternate function selection for port x pin y
    u1 AF01       : 4;  // Alternate function selection for port x pin y
    u1 AF02       : 4;  // Alternate function selection for port x pin y
    u1 AF03       : 4;  // Alternate function selection for port x pin y
    u1 AF04       : 4;  // Alternate function selection for port x pin y
    u1 AF05       : 4;  // Alternate function selection for port x pin y
    u1 AF06       : 4;  // Alternate function selection for port x pin y
    u1 AF07       : 4;  // Alternate function selection for port x pin y
  }volatile AFRL;             //GPIO alternate function low register (GPIOx_AFRL) (x=A..G)
  struct
  {
    u1 AF08       : 4;  // Alternate function selection for port x pin y
    u1 AF09       : 4;  // Alternate function selection for port x pin y
    u1 AF10       : 4;  // Alternate function selection for port x pin y
    u1 AF11       : 4;  // Alternate function selection for port x pin y
    u1 AF12       : 4;  // Alternate function selection for port x pin y
    u1 AF13       : 4;  // Alternate function selection for port x pin y
    u1 AF14       : 4;  // Alternate function selection for port x pin y
    u1 AF15       : 4;  // Alternate function selection for port x pin y
  }volatile AFRH;             //GPIO alternate function high register (GPIOx_AFRH) (x=A..G)
  struct
  {
    u1 BR         : 16; //Port x Reset bit y
    u1 RSVD0      : 16; //Reserved
  }volatile BRR;              //GPIO port bit reset register (GPIOx_BRR) (x=A..G)
}GPIO_Type;
#define GPIOA ((GPIO_Type*) GPIOA_BASE)
#define GPIOB ((GPIO_Type*) GPIOB_BASE)
#define GPIOC ((GPIO_Type*) GPIOC_BASE)
#define GPIOD ((GPIO_Type*) GPIOD_BASE)
#define GPIOE ((GPIO_Type*) GPIOE_BASE)
#define GPIOF ((GPIO_Type*) GPIOF_BASE)
#define GPIO_CFG_PIN00(PORT, MODE) PORT->MODER.MD00 = (MODE & 3); PORT->PUPDR.PP00 = ((MODE >> 2) & 3); PORT->OTYPER.OT00 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP00 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x0); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x0) //Port bit 00 configuration
#define GPIO_CFG_PIN01(PORT, MODE) PORT->MODER.MD01 = (MODE & 3); PORT->PUPDR.PP01 = ((MODE >> 2) & 3); PORT->OTYPER.OT01 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP01 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x1); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x1) //Port bit 01 configuration
#define GPIO_CFG_PIN02(PORT, MODE) PORT->MODER.MD02 = (MODE & 3); PORT->PUPDR.PP02 = ((MODE >> 2) & 3); PORT->OTYPER.OT02 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP02 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x2); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x2) //Port bit 02 configuration
#define GPIO_CFG_PIN03(PORT, MODE) PORT->MODER.MD03 = (MODE & 3); PORT->PUPDR.PP03 = ((MODE >> 2) & 3); PORT->OTYPER.OT03 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP03 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x3); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x3) //Port bit 03 configuration
#define GPIO_CFG_PIN04(PORT, MODE) PORT->MODER.MD04 = (MODE & 3); PORT->PUPDR.PP04 = ((MODE >> 2) & 3); PORT->OTYPER.OT04 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP04 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x4); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x4) //Port bit 04 configuration
#define GPIO_CFG_PIN05(PORT, MODE) PORT->MODER.MD05 = (MODE & 3); PORT->PUPDR.PP05 = ((MODE >> 2) & 3); PORT->OTYPER.OT05 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP05 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x5); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x5) //Port bit 05 configuration
#define GPIO_CFG_PIN06(PORT, MODE) PORT->MODER.MD06 = (MODE & 3); PORT->PUPDR.PP06 = ((MODE >> 2) & 3); PORT->OTYPER.OT06 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP06 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x6); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x6) //Port bit 06 configuration
#define GPIO_CFG_PIN07(PORT, MODE) PORT->MODER.MD07 = (MODE & 3); PORT->PUPDR.PP07 = ((MODE >> 2) & 3); PORT->OTYPER.OT07 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP07 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x7); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x7) //Port bit 07 configuration
#define GPIO_CFG_PIN08(PORT, MODE) PORT->MODER.MD08 = (MODE & 3); PORT->PUPDR.PP08 = ((MODE >> 2) & 3); PORT->OTYPER.OT08 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP08 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x8); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x8) //Port bit 08 configuration
#define GPIO_CFG_PIN09(PORT, MODE) PORT->MODER.MD09 = (MODE & 3); PORT->PUPDR.PP09 = ((MODE >> 2) & 3); PORT->OTYPER.OT09 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP09 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x9); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x9) //Port bit 09 configuration
#define GPIO_CFG_PIN10(PORT, MODE) PORT->MODER.MD10 = (MODE & 3); PORT->PUPDR.PP10 = ((MODE >> 2) & 3); PORT->OTYPER.OT10 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP10 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0xA); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0xA) //Port bit 10 configuration
#define GPIO_CFG_PIN11(PORT, MODE) PORT->MODER.MD11 = (MODE & 3); PORT->PUPDR.PP11 = ((MODE >> 2) & 3); PORT->OTYPER.OT11 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP11 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0xB); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0xB) //Port bit 11 configuration
#define GPIO_CFG_PIN12(PORT, MODE) PORT->MODER.MD12 = (MODE & 3); PORT->PUPDR.PP12 = ((MODE >> 2) & 3); PORT->OTYPER.OT12 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP12 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0xC); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0xC) //Port bit 12 configuration
#define GPIO_CFG_PIN13(PORT, MODE) PORT->MODER.MD13 = (MODE & 3); PORT->PUPDR.PP13 = ((MODE >> 2) & 3); PORT->OTYPER.OT13 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP13 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0xD); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0xD) //Port bit 13 configuration
#define GPIO_CFG_PIN14(PORT, MODE) PORT->MODER.MD14 = (MODE & 3); PORT->PUPDR.PP14 = ((MODE >> 2) & 3); PORT->OTYPER.OT14 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP14 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0xE); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0xE) //Port bit 14 configuration
#define GPIO_CFG_PIN15(PORT, MODE) PORT->MODER.MD15 = (MODE & 3); PORT->PUPDR.PP15 = ((MODE >> 2) & 3); PORT->OTYPER.OT15 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP15 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0xF); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0xF) //Port bit 15 configuration
#define GPIO_IN(PORT)           PORT->IDR.IR                    //Port input data register
#define GPIO_OUT(PORT)          PORT->ODR.OR                    //Port output data register
#define GPIO_BIT_SET(PORT)      PORT->BSRR.BS                   //Port Set bit
#define GPIO_BIT_RST(PORT)      PORT->BSRR.BR                   //Port Reset bit
#define GPIO_LCK(PORT)          PORT->LCKR.LCK                  //Port Lock bit
#define GPIO_LCK_KEY(PORT)      PORT->LCKR.LCKK                 //Port Lock key
#define GPIO_AF_PIN00(PORT)     PORT->AFRL.AF00                 // Alternate function selection for port x pin 0
#define GPIO_AF_PIN01(PORT)     PORT->AFRL.AF01                 // Alternate function selection for port x pin 1
#define GPIO_AF_PIN02(PORT)     PORT->AFRL.AF02                 // Alternate function selection for port x pin 2
#define GPIO_AF_PIN03(PORT)     PORT->AFRL.AF03                 // Alternate function selection for port x pin 3
#define GPIO_AF_PIN04(PORT)     PORT->AFRL.AF04                 // Alternate function selection for port x pin 4
#define GPIO_AF_PIN05(PORT)     PORT->AFRL.AF05                 // Alternate function selection for port x pin 5
#define GPIO_AF_PIN06(PORT)     PORT->AFRL.AF06                 // Alternate function selection for port x pin 6
#define GPIO_AF_PIN07(PORT)     PORT->AFRL.AF07                 // Alternate function selection for port x pin 7
#define GPIO_AF_PIN08(PORT)     PORT->AFRH.AF08                 // Alternate function selection for port x pin 8
#define GPIO_AF_PIN09(PORT)     PORT->AFRH.AF09                 // Alternate function selection for port x pin 9
#define GPIO_AF_PIN10(PORT)     PORT->AFRH.AF10                 // Alternate function selection for port x pin 10
#define GPIO_AF_PIN11(PORT)     PORT->AFRH.AF11                 // Alternate function selection for port x pin 11
#define GPIO_AF_PIN12(PORT)     PORT->AFRH.AF12                 // Alternate function selection for port x pin 12
#define GPIO_AF_PIN13(PORT)     PORT->AFRH.AF13                 // Alternate function selection for port x pin 13
#define GPIO_AF_PIN14(PORT)     PORT->AFRH.AF14                 // Alternate function selection for port x pin 14
#define GPIO_AF_PIN15(PORT)     PORT->AFRH.AF15                 // Alternate function selection for port x pin 15
#define GPIO_ANALOG             ((u8) 0x03)                     //Input/output Analog
#define GPIO_IN_FLOAT           ((u8) 0x00)                     //Input Floating
#define GPIO_IN_PU              ((u8) 0x04)                     //Input UP
#define GPIO_IN_PD              ((u8) 0x08)                     //Input PD
#define GPIO_OUT_PP_NO_L        ((u8) 0x01)                     //GP output PP
#define GPIO_OUT_PP_NO_M        ((u8) 0x41)                     //GP output PP
#define GPIO_OUT_PP_NO_H        ((u8) 0xC1)                     //GP output PP
#define GPIO_OUT_PP_PU_L        ((u8) 0x05)                     //GP output PP + PU
#define GPIO_OUT_PP_PU_M        ((u8) 0x45)                     //GP output PP + PU
#define GPIO_OUT_PP_PU_H        ((u8) 0xC5)                     //GP output PP + PU
#define GPIO_OUT_PP_PD_L        ((u8) 0x09)                     //GP output PP + PD
#define GPIO_OUT_PP_PD_M        ((u8) 0x49)                     //GP output PP + PD
#define GPIO_OUT_PP_PD_H        ((u8) 0xC9)                     //GP output PP + PD
#define GPIO_OUT_OD_NO_L        ((u8) 0x11)                     //GP output OD
#define GPIO_OUT_OD_NO_M        ((u8) 0x51)                     //GP output OD
#define GPIO_OUT_OD_NO_H        ((u8) 0xD1)                     //GP output OD
#define GPIO_OUT_OD_PU_L        ((u8) 0x15)                     //GP output OD + PU
#define GPIO_OUT_OD_PU_M        ((u8) 0x55)                     //GP output OD + PU
#define GPIO_OUT_OD_PU_H        ((u8) 0xD5)                     //GP output OD + PU
#define GPIO_OUT_OD_PD_L        ((u8) 0x19)                     //GP output OD + PD
#define GPIO_OUT_OD_PD_M        ((u8) 0x59)                     //GP output OD + PD
#define GPIO_OUT_OD_PD_H        ((u8) 0xD9)                     //GP output OD + PD
#define GPIO_AF_PP_NO_L         ((u8) 0x02)                     //AF PP
#define GPIO_AF_PP_NO_M         ((u8) 0x42)                     //AF PP
#define GPIO_AF_PP_NO_H         ((u8) 0xC2)                     //AF PP
#define GPIO_AF_PP_PU_L         ((u8) 0x06)                     //AF PP + PU
#define GPIO_AF_PP_PU_M         ((u8) 0x46)                     //AF PP + PU
#define GPIO_AF_PP_PU_H         ((u8) 0xC6)                     //AF PP + PU
#define GPIO_AF_PP_PD_L         ((u8) 0x0A)                     //AF PP + PD
#define GPIO_AF_PP_PD_M         ((u8) 0x4A)                     //AF PP + PD
#define GPIO_AF_PP_PD_H         ((u8) 0xCA)                     //AF PP + PD
#define GPIO_AF_OD_NO_L         ((u8) 0x12)                     //AF OD
#define GPIO_AF_OD_NO_M         ((u8) 0x52)                     //AF OD
#define GPIO_AF_OD_NO_H         ((u8) 0xD2)                     //AF OD
#define GPIO_AF_OD_PU_L         ((u8) 0x16)                     //AF OD + PU
#define GPIO_AF_OD_PU_M         ((u8) 0x56)                     //AF OD + PU
#define GPIO_AF_OD_PU_H         ((u8) 0xD6)                     //AF OD + PU
#define GPIO_AF_OD_PD_L         ((u8) 0x1A)                     //AF OD + PD
#define GPIO_AF_OD_PD_M         ((u8) 0x5A)                     //AF OD + PD
#define GPIO_AF_OD_PD_H         ((u8) 0xDA)                     //AF OD + PD
#define GPIO_PIN00              ((u16) 0x0001)
#define GPIO_PIN01              ((u16) 0x0002)
#define GPIO_PIN02              ((u16) 0x0004)
#define GPIO_PIN03              ((u16) 0x0008)
#define GPIO_PIN04              ((u16) 0x0010)
#define GPIO_PIN05              ((u16) 0x0020)
#define GPIO_PIN06              ((u16) 0x0040)
#define GPIO_PIN07              ((u16) 0x0080)
#define GPIO_PIN08              ((u16) 0x0100)
#define GPIO_PIN09              ((u16) 0x0200)
#define GPIO_PIN10              ((u16) 0x0400)
#define GPIO_PIN11              ((u16) 0x0800)
#define GPIO_PIN12              ((u16) 0x1000)
#define GPIO_PIN13              ((u16) 0x2000)
#define GPIO_PIN14              ((u16) 0x4000)
#define GPIO_PIN15              ((u16) 0x8000)
#define GPIO_AF0                ((u8) 0x00)
#define GPIO_AF1                ((u8) 0x01)
#define GPIO_AF2                ((u8) 0x02)
#define GPIO_AF3                ((u8) 0x03)
#define GPIO_AF4                ((u8) 0x04)
#define GPIO_AF5                ((u8) 0x05)
#define GPIO_AF6                ((u8) 0x06)
#define GPIO_AF7                ((u8) 0x07)
/******************************************** EXTI ************************************************/
typedef struct
{
  volatile u32 IMR;           //Interrupt mask register (EXTI_IMR)
  volatile u32 EMR;           //Event mask register (EXTI_EMR)
  struct
  {
    u1 TR         : 23; //Rising trigger event configuration bit of line x
    u1 RSVD0      : 9;  //Reserved
  }volatile RTSR;             //Rising trigger selection register (EXTI_RTSR)
  struct
  {
    u1 TR         : 23; //Falling trigger event configuration bit of line x
    u1 RSVD0      : 9;  //Reserved
  }volatile FTSR;             //Falling trigger selection register (EXTI_FTSR)
  struct
  {
    u1 SW         : 23; //Software interrupt on line x
    u1 RSVD0      : 9;  //Reserved
  }volatile SWIER;            //Software interrupt event register (EXTI_SWIER)
  struct
  {
    u1 PR         : 23; //Pending bit
    u1 RSVD0      : 9;  //Reserved
  }volatile PR;               //Pending register (EXTI_PR)
}EXTI_Type;
#define EXTI ((EXTI_Type*) EXTI_BASE)
#define EXTI_MASK_IRQ           EXTI->IMR                       //Interrupt Mask on line x
#define EXTI_MASK_EVENT         EXTI->EMR                       //Event mask on line x
#define EXTI_MASK_RIS           EXTI->RTSR.TR                   //Rising trigger event configuration bit of line x
#define EXTI_MASK_FAL           EXTI->FTSR.TR                   //Falling trigger event configuration bit of line x
#define EXTI_MASK_SW            EXTI->SWIER.SW                  //Software interrupt on line x
#define EXTI_MASK_PEND          EXTI->PR.PR                     //Pending bit
#define EXTI_LINE00             ((u32) 0x00000001)
#define EXTI_LINE01             ((u32) 0x00000002)
#define EXTI_LINE02             ((u32) 0x00000004)
#define EXTI_LINE03             ((u32) 0x00000008)
#define EXTI_LINE04             ((u32) 0x00000010)
#define EXTI_LINE05             ((u32) 0x00000020)
#define EXTI_LINE06             ((u32) 0x00000040)
#define EXTI_LINE07             ((u32) 0x00000080)
#define EXTI_LINE08             ((u32) 0x00000100)
#define EXTI_LINE09             ((u32) 0x00000200)
#define EXTI_LINE10             ((u32) 0x00000400)
#define EXTI_LINE11             ((u32) 0x00000800)
#define EXTI_LINE12             ((u32) 0x00001000)
#define EXTI_LINE13             ((u32) 0x00002000)
#define EXTI_LINE14             ((u32) 0x00004000)
#define EXTI_LINE15             ((u32) 0x00008000)
#define EXTI_LINE16             ((u32) 0x00010000)
#define EXTI_LINE17             ((u32) 0x00020000)
#define EXTI_LINE18             ((u32) 0x00040000)
#define EXTI_LINE19             ((u32) 0x00080000)
#define EXTI_LINE20             ((u32) 0x00100000)
#define EXTI_LINE21             ((u32) 0x00200000)
#define EXTI_LINE22             ((u32) 0x00400000)
/******************************************** ADC *************************************************/
typedef struct
{
  struct
  {
    u1 ADRDY      : 1;  //ADC ready
    u1 EOSMP      : 1;  //End of sampling flag
    u1 EOC        : 1;  //End of conversion flag
    u1 EOSEQ      : 1;  //End of sequence flag
    u1 OVR        : 1;  //ADC overrun
    u1 RSVD0      : 2;  //Reserved
    u1 AWD        : 1;  //Analog watchdog flag
    u1 RSVD1      : 24; //Reserved
  }volatile ISR;              //ADC interrupt and status register (ADC_ISR)
  struct
  {
    u1 ADRDYIE    : 1;  //ADC ready interrupt enable
    u1 EOSMPIE    : 1;  //End of sampling flag interrupt enable
    u1 EOCIE      : 1;  //End of conversion interrupt enable
    u1 EOSEQIE    : 1;  //End of conversion sequence interrupt enable
    u1 OVRIE      : 1;  //Overrun interrupt enable
    u1 RSVD0      : 2;  //Reserved
    u1 AWDIE      : 1;  //Analog watchdog interrupt enable
    u1 RSVD1      : 24; //Reserved
  }volatile IER;              //ADC interrupt enable register (ADC_IER)
  struct
  {
    u1 ADEN       : 1;  //ADC enable command
    u1 ADDIS      : 1;  //ADC disable command
    u1 ADSTART    : 1;  //ADC start conversion command
    u1 RSVD0      : 1;  //Reserved
    u1 ADSTP      : 1;  //ADC stop conversion command
    u1 RSVD1      : 26; //Reserved
    u1 ADCAL      : 1;  //ADC calibration
  }volatile CR;               //ADC control register (ADC_CR)
  struct
  {
    u1 DMAEN      : 1;  //Direct memory access enable
    u1 DMACFG     : 1;  //Direct memory access configuration
    u1 SCANDIR    : 1;  //Scan sequence direction
    u1 RES        : 2;  //Data resolution
    u1 ALIGN      : 1;  //Data alignment
    u1 EXTSEL     : 3;  //External trigger selection
    u1 RSVD0      : 1;  //Reserved
    u1 EXTEN      : 2;  //External trigger enable and polarity selection
    u1 OVRMOD     : 1;  //Overrun management mode
    u1 CONT       : 1;  //Single/continuous conversion mode
    u1 WAIT       : 1;  //Wait conversion mode
    u1 AUTOFF     : 1;  //Auto-off mode
    u1 DISCEN     : 1;  //Discontinuous mode
    u1 RSVD1      : 5;  //Reserved
    u1 AWDSGL     : 1;  //Enable the watchdog on a single channel or on all channels
    u1 AWDEN      : 1;  //Analog watchdog enable
    u1 RSVD2      : 2;  //Reserved
    u1 AWDCH      : 5;  //Analog watchdog channel selection
    u1 RSVD3      : 1;  //Reserved
  }volatile CFGR1;            //ADC configuration register 1 (ADC_CFGR1)
  struct
  {
    u1 RSVD0      : 30; //Reserved
    u1 CKMODE     : 2;  //ADC clock mode
  }volatile CFGR2;            //ADC configuration register 2 (ADC_CFGR2)
  struct
  {
    u1 SMP        : 3;  //Sampling time selection
    u1 RSVD0      : 29; //Reserved
  }volatile SMPR;             //ADC sampling time register (ADC_SMPR)
  volatile u32 RSVD0[2];
  struct
  {
    u1 LT         : 12; //Analog watchdog lower threshold
    u1 RSVD0      : 4;  //Reserved
    u1 HT         : 12; //Analog watchdog higher threshold
    u1 RSVD1      : 4;  //Reserved
  }volatile TR;               //ADC watchdog threshold register (ADC_TR)
  volatile u32 RSVD1;
  struct
  {
    u1 CHSEL      : 19; //Channel-x selection
    u1 RSVD0      : 13; //Reserved
  }volatile CHSELR;           //ADC channel selection register (ADC_CHSELR)
  volatile u32 RSVD2[5];
  struct
  {
    u1 DATA       : 16; //Converted data
    u1 RSVD0      : 16; //Reserved
  }volatile DR;               //ADC data register (ADC_DR)
  volatile u32 RSVD3[177];
  struct
  {
    u1 RSVD0      : 22; //Reserved
    u1 VREFEN     : 1;  //VREFINT enable
    u1 TSEN       : 1;  //Temperature sensor enable
    u1 VBATEN     : 1;  //VBAT enable
    u1 RSVD1      : 7;  //Reserved
  }volatile CCR;              //ADC common configuration register (ADC_CCR)
}ADC_Type;
#define ADC ((ADC_Type*) ADC_BASE)
#define ADC_RDY                 ADC->ISR.ADRDY                  //ADC ready
#define ADC_EOSMP_FLG           ADC->ISR.EOSMP                  //End of sampling flag
#define ADC_EOC_FLG             ADC->ISR.EOC                    //End of conversion flag
#define ADC_EOSEQ_FLG           ADC->ISR.EOSEQ                  //End of sequence flag
#define ADC_OVR_FLG             ADC->ISR.OVR                    //ADC overrun
#define ADC_AWD_FLG             ADC->ISR.AWD                    //Analog watchdog flag
#define ADC_RDY_IRQ             ADC->IER.ADRDYIE                //ADC ready interrupt enable
#define ADC_EOSMP_IRQ           ADC->IER.EOSMPIE                //End of sampling flag interrupt enable
#define ADC_EOC_IRQ             ADC->IER.EOCIE                  //End of conversion interrupt enable
#define ADC_EOSEQ_IRQ           ADC->IER.EOSEQIE                //End of conversion sequence interrupt enable
#define ADC_OVR_IRQ             ADC->IER.OVRIE                  //Overrun interrupt enable
#define ADC_AWD_IRQ             ADC->IER.AWDIE                  //Analog watchdog interrupt enable
#define ADC_ON                  ADC->CR.ADEN                    //ADC enable command
#define ADC_OFF                 ADC->CR.ADDIS                   //ADC disable command
#define ADC_START               ADC->CR.ADSTART                 //ADC start conversion command
#define ADC_STOP                ADC->CR.ADSTP                   //ADC stop conversion command
#define ADC_CAL                 ADC->CR.ADCAL                   //ADC calibration
#define ADC_DMA_EN              ADC->CFGR1.DMAEN                //Direct memory access enable
#define ADC_DMA_CFG             ADC->CFGR1.DMACFG               //Direct memory access configuration
#define ADC_SCANDIR             ADC->CFGR1.SCANDIR              //Scan sequence direction
#define ADC_DATARES             ADC->CFGR1.RES                  //Data resolution
#define ADC_ALIGN               ADC->CFGR1.ALIGN                //Data alignment
#define ADC_EXT_SEL             ADC->CFGR1.EXTSEL               //External trigger selection
#define ADC_EXT_EN              ADC->CFGR1.EXTEN                //External trigger enable and polarity selection
#define ADC_OVRMOD              ADC->CFGR1.OVRMOD               //Overrun management mode
#define ADC_CONT                ADC->CFGR1.CONT                 //Single/continuous conversion mode
#define ADC_WAIT                ADC->CFGR1.WAIT                 //Wait conversion mode
#define ADC_AUTOFF              ADC->CFGR1.AUTOFF               //Auto-off mode
#define ADC_DISCEN              ADC->CFGR1.DISCEN               //Discontinuous mode
#define ADC_AWDSGL              ADC->CFGR1.AWDSGL               //Enable the watchdog on a single channel or on all channels
#define ADC_AWD_EN              ADC->CFGR1.AWDEN                //Analog watchdog enable
#define ADC_AWDCH               ADC->CFGR1.AWDCH                //Analog watchdog channel selection
#define ADC_CKMODE              ADC->CFGR2.CKMODE               //ADC clock mode
#define ADC_SMP                 ADC->SMPR.SMP                   //Sampling time selection
#define ADC_LT                  ADC->TR.LT                      //Analog watchdog lower threshold
#define ADC_HT                  ADC->TR.HT                      //Analog watchdog higher threshold
#define ADC_CHSEL               ADC->CHSELR.CHSEL               //Channel-x selection
#define ADC_DATA                ADC->DR.DATA                    //Converted data
#define ADC_VREFEN              ADC->CCR.VREFEN                 //VREFINT enable
#define ADC_TSEN                ADC->CCR.TSEN                   //Temperature sensor enable
#define ADC_VBATEN              ADC->CCR.VBATEN                 //VBAT enable
#define ADC_CH00                ((u32) 0x00000001)              //ADC analog Channel0
#define ADC_CH01                ((u32) 0x00000002)              //ADC analog Channel1
#define ADC_CH02                ((u32) 0x00000004)              //ADC analog Channel2
#define ADC_CH03                ((u32) 0x00000008)              //ADC analog Channel3
#define ADC_CH04                ((u32) 0x00000010)              //ADC analog Channel4
#define ADC_CH05                ((u32) 0x00000020)              //ADC analog Channel5
#define ADC_CH06                ((u32) 0x00000040)              //ADC analog Channel6
#define ADC_CH07                ((u32) 0x00000080)              //ADC analog Channel7
#define ADC_CH08                ((u32) 0x00000100)              //ADC analog Channel8
#define ADC_CH09                ((u32) 0x00000200)              //ADC analog Channel9
#define ADC_CH10                ((u32) 0x00000400)              //ADC analog Channel10
#define ADC_CH11                ((u32) 0x00000800)              //ADC analog Channel11
#define ADC_CH12                ((u32) 0x00001000)              //ADC analog Channel12
#define ADC_CH13                ((u32) 0x00002000)              //ADC analog Channel13
#define ADC_CH14                ((u32) 0x00004000)              //ADC analog Channel14
#define ADC_CH15                ((u32) 0x00008000)              //ADC analog Channel15
#define ADC_TEMP                ((u32) 0x00010000)              //ADC analog Channel16 (Temperature sensor)
#define ADC_VREF                ((u32) 0x00020000)              //ADC analog Channel17 (VREF)
#define ADC_VBAT                ((u32) 0x00040000)              //ADC analog Channel18 (VBAT)
#define ADC_SMP_T015            ((u8) 0x00)                     //1.5 cycles
#define ADC_SMP_T075            ((u8) 0x01)                     //7.5 cycles
#define ADC_SMP_T135            ((u8) 0x02)                     //13.5 cycles
#define ADC_SMP_T285            ((u8) 0x03)                     //28.5 cycles
#define ADC_SMP_T415            ((u8) 0x04)                     //41.5 cycles
#define ADC_SMP_T555            ((u8) 0x05)                     //55.5 cycles
#define ADC_SMP_T715            ((u8) 0x06)                     //71.5 cycles
#define ADC_SMP_T239            ((u8) 0x07)                     //239.5 cycles
#define ADC_MODE_ADCCLK         ((u8) 0x00)                     //ADCCLK (Asynchronous clock mode)
#define ADC_MODE_PCLK_2         ((u8) 0x01)                     //PCLK/2 (Synchronous clock mode)
#define ADC_MODE_PCLK_4         ((u8) 0x02)                     //PCLK/4 (Synchronous clock mode)
#define ADC_EXT_T01_TRG         ((u8) 0x00)                     //TIM1_TRGO
#define ADC_EXT_T01_CC4         ((u8) 0x01)                     //TIM1_CC4
#define ADC_EXT_T02_TRG         ((u8) 0x02)                     //TIM2_TRGO
#define ADC_EXT_T03_TRG         ((u8) 0x03)                     //TIM3_TRGO
#define ADC_EXT_T15_TRG         ((u8) 0x04)                     //TIM15_TRGO
#define ADC_RES_12BIT           ((u8) 0x00)                     //Data resolution 12 bits
#define ADC_RES_10BIT           ((u8) 0x01)                     //Data resolution 10 bits
#define ADC_RES_08BIT           ((u8) 0x02)                     //Data resolution 08 bits
#define ADC_RES_06BIT           ((u8) 0x03)                     //Data resolution 06 bits
#define ADC_EXT_START           ((u8) 0x00)                     //Software
#define ADC_EXT_RISING          ((u8) 0x01)                     //Hardware trigger detection on the rising edge
#define ADC_EXT_FALLING         ((u8) 0x02)                     //Hardware trigger detection on the falling edge
#define ADC_EXT_BOTH            ((u8) 0x03)                     //Hardware trigger detection on both the rising and falling edges
#define ADC_ALIGN_R             ((u8) 0x00)                     //Right Alignment
#define ADC_ALIGN_L             ((u8) 0x01)                     //Left Alignment
/******************************************** CRC *************************************************/
typedef struct
{
  volatile u32 DR;            //Data register (CRC_DR)
  struct
  {
    u1 ID         : 8;  //General-purpose 8-bit data register bits
    u1 RSVD0      : 24; //Reserved
  }volatile IDR;              //Independent data register (CRC_IDR)
  struct
  {
    u1 RESET      : 1;  //Resets the CRC calculation unit
    u1 RSVD0      : 2;  //Reserved
    u1 POLYSIZE   : 2;  //Polynomial size
    u1 REV_IN     : 2;  //Reverse input data
    u1 REV_OUT    : 1;  //Reverse output data
    u1 RSVD1      : 24; //Reserved
  }volatile CR;               //Control register (CRC_CR)
  volatile u32 INIT;          //Initial CRC value (CRC_INIT)
  volatile u32 POL;           //CRC polynomial (CRC_POL)
}CRC_Type;
#define CRC ((CRC_Type*) CRC_BASE)
#define CRC_DR                  CRC->DR                         //Data register (CRC_DR)
#define CRC_IDR                 CRC->IDR.ID                     //Independent data register (CRC_IDR)
#define CRC_RESET               CRC->CR.RESET = ON              //Resets the CRC calculation unit
#define CRC_POLYSIZE            CRC->CR.POLYSIZE                //Polynomial size
#define CRC_REV_IN              CRC->CR.REV_IN                  //Reverse input data
#define CRC_REV_OUT             CRC->CR.REV_OUT                 //Reverse output data
#define CRC_INIT                CRC->INIT                       //Initial CRC value (CRC_INIT)
#define CRC_POL                 CRC->POL                        //CRC polynomial (CRC_POL)
/****************************************** DBGMCU ************************************************/
typedef struct
{
  struct
  {
    u1 DEVID      : 12; //Device identifier
    u1 RSVD0      : 4;  //Reserved
    u1 REVID      : 16; //Revision identifier
  }volatile IDCODE;           //DBGMCU_IDCODE
  struct
  {
    u1 RSVD0      : 1;  //Reserved
    u1 DBGSTOP    : 1;  //Debug Stop mode
    u1 DBGSTANDBY : 1;  //Debug Standby mode
    u1 RSVD1      : 29; //Reserved
  }volatile CR;               //DBGMCU_CR
  struct
  {
    u1 TIM2STOP   : 1;  //Debug TIM2 counter stopped when core is halted
    u1 TIM3STOP   : 1;  //Debug TIM3 counter stopped when core is halted
    u1 RSVD0      : 2;  //Reserved
    u1 TIM6STOP   : 1;  //Debug TIM6 counter stopped when core is halted
    u1 TIM7STOP   : 1;  //Debug TIM7 counter stopped when core is halted
    u1 RSVD1      : 2;  //Reserved
    u1 TIM14STOP  : 1;  //Debug TIM14 counter stopped when core is halted
    u1 RSVD2      : 1;  //Reserved
    u1 RTCSTOP    : 1;  //Debug RTC stopped when core is halted
    u1 WWDGSTOP   : 1;  //Debug window watchdog stopped when core is halted
    u1 IWDGSTOP   : 1;  //Debug independent watchdog stopped when core is halted
    u1 RSVD3      : 8;  //Reserved
    u1 I2C1TOUT   : 1;  //SMBUS timeout mode stopped when core is halted
    u1 RSVD4      : 3;  //Reserved
    u1 CANSTOP    : 1;  //Debug CAN stopped when core is halted
    u1 RSVD5      : 6;  //Reserved
  }volatile APB1_FZ;          //Debug MCU APB1 freeze register (DBGMCU_APB1_FZ)
  struct
  {
    u1 RSVD0      : 11; //Reserved
    u1 TIM1STOP   : 1;  //Debug TIM1 counter stopped when core is halted
    u1 RSVD1      : 4;  //Reserved
    u1 TIM15STOP  : 1;  //Debug TIM15 counter stopped when core is halted
    u1 TIM16STOP  : 1;  //Debug TIM16 counter stopped when core is halted
    u1 TIM17STOP  : 1;  //Debug TIM17 counter stopped when core is halted
    u1 RSVD2      : 13; //Reserved
  }volatile APB2_FZ;          //Debug MCU APB2 freeze register (DBGMCU_APB2_FZ)
}DBGMCU_Type;
#define DBGMCU ((DBGMCU_Type*) DBGMCU_BASE)
#define DBGMCU_DEVID            DBGMCU->IDCODE.DEVID            //Device identifier
#define DBGMCU_REVID            DBGMCU->IDCODE.REVID            //Revision identifier
#define DBGMCU_DBGSTOP          DBGMCU->CR.DBGSTOP              //Debug Stop mode
#define DBGMCU_DBGSTANDBY       DBGMCU->CR.DBGSTANDBY           //Debug Standby mode
#define DBGMCU_TIM2STOP         DBGMCU->APB1_FZ.TIM2STOP        //Debug TIM2 counter stopped when core is halted
#define DBGMCU_TIM3STOP         DBGMCU->APB1_FZ.TIM3STOP        //Debug TIM3 counter stopped when core is halted
#define DBGMCU_TIM6STOP         DBGMCU->APB1_FZ.TIM6STOP        //Debug TIM6 counter stopped when core is halted
#define DBGMCU_TIM7STOP         DBGMCU->APB1_FZ.TIM7STOP        //Debug TIM7 counter stopped when core is halted
#define DBGMCU_TIM14STOP        DBGMCU->APB1_FZ.TIM14STOP       //Debug TIM14 counter stopped when core is halted
#define DBGMCU_RTCSTOP          DBGMCU->APB1_FZ.RTCSTOP         //Debug RTC stopped when core is halted
#define DBGMCU_WWDGSTOP         DBGMCU->APB1_FZ.WWDGSTOP        //Debug window watchdog stopped when core is halted
#define DBGMCU_IWDGSTOP         DBGMCU->APB1_FZ.IWDGSTOP        //Debug independent watchdog stopped when core is halted
#define DBGMCU_I2C1TOUT         DBGMCU->APB1_FZ.I2C1TOUT        //SMBUS timeout mode stopped when core is halted
#define DBGMCU_CANSTOP          DBGMCU->APB1_FZ.CANSTOP         //Debug CAN stopped when core is halted
#define DBGMCU_TIM1STOP         DBGMCU->APB2_FZ.TIM1STOP        //Debug TIM1 counter stopped when core is halted
#define DBGMCU_TIM15STOP        DBGMCU->APB2_FZ.TIM15STOP       //Debug TIM15 counter stopped when core is halted
#define DBGMCU_TIM16STOP        DBGMCU->APB2_FZ.TIM16STOP       //Debug TIM16 counter stopped when core is halted
#define DBGMCU_TIM17STOP        DBGMCU->APB2_FZ.TIM17STOP       //Debug TIM17 counter stopped when core is halted
/******************************************** DMA *************************************************/
typedef struct
{
  struct
  {
    u1 GIF1       : 1;  //Channel x global interrupt flag 1
    u1 TCIF1      : 1;  //Channel x transfer complete flag 1
    u1 HTIF1      : 1;  //Channel x half transfer flag 1
    u1 TEIF1      : 1;  //Channel x transfer error flag 1
    u1 GIF2       : 1;  //Channel x global interrupt flag 2
    u1 TCIF2      : 1;  //Channel x transfer complete flag 2
    u1 HTIF2      : 1;  //Channel x half transfer flag 2
    u1 TEIF2      : 1;  //Channel x transfer error flag 2
    u1 GIF3       : 1;  //Channel x global interrupt flag 3
    u1 TCIF3      : 1;  //Channel x transfer complete flag 3
    u1 HTIF3      : 1;  //Channel x half transfer flag 3
    u1 TEIF3      : 1;  //Channel x transfer error flag 3    
    u1 GIF4       : 1;  //Channel x global interrupt flag 4
    u1 TCIF4      : 1;  //Channel x transfer complete flag 4
    u1 HTIF4      : 1;  //Channel x half transfer flag 4
    u1 TEIF4      : 1;  //Channel x transfer error flag 4
    u1 GIF5       : 1;  //Channel x global interrupt flag 5
    u1 TCIF5      : 1;  //Channel x transfer complete flag 5
    u1 HTIF5      : 1;  //Channel x half transfer flag 5
    u1 TEIF5      : 1;  //Channel x transfer error flag 5
    u1 GIF6       : 1;  //Channel x global interrupt flag 6
    u1 TCIF6      : 1;  //Channel x transfer complete flag 6
    u1 HTIF6      : 1;  //Channel x half transfer flag 6
    u1 TEIF6      : 1;  //Channel x transfer error flag 6
    u1 GIF7       : 1;  //Channel x global interrupt flag 7
    u1 TCIF7      : 1;  //Channel x transfer complete flag 7
    u1 HTIF7      : 1;  //Channel x half transfer flag 7
    u1 TEIF7      : 1;  //Channel x transfer error flag 7
    u1 RSVD       : 4;  //Reserved
  }volatile ISR;              //DMA interrupt status register (DMA_ISR)
  struct
  {
    u1 CGIF1      : 1;  //Channel x global interrupt clear 1
    u1 CTCIF1     : 1;  //Channel x transfer complete clear 1
    u1 CHTIF1     : 1;  //Channel x half transfer clear 1
    u1 CTEIF1     : 1;  //Channel x transfer error clear 1
    u1 CGIF2      : 1;  //Channel x global interrupt clear 2
    u1 CTCIF2     : 1;  //Channel x transfer complete clear 2
    u1 CHTIF2     : 1;  //Channel x half transfer clear 2
    u1 CTEIF2     : 1;  //Channel x transfer error clear 2
    u1 CGIF3      : 1;  //Channel x global interrupt clear 3
    u1 CTCIF3     : 1;  //Channel x transfer complete clear 3
    u1 CHTIF3     : 1;  //Channel x half transfer clear 3
    u1 CTEIF3     : 1;  //Channel x transfer error clear 3    
    u1 CGIF4      : 1;  //Channel x global interrupt clear 4
    u1 CTCIF4     : 1;  //Channel x transfer complete clear 4
    u1 CHTIF4     : 1;  //Channel x half transfer clear 4
    u1 CTEIF4     : 1;  //Channel x transfer error clear 4
    u1 CGIF5      : 1;  //Channel x global interrupt clear 5
    u1 CTCIF5     : 1;  //Channel x transfer complete clear 5
    u1 CHTIF5     : 1;  //Channel x half transfer clear 5
    u1 CTEIF5     : 1;  //Channel x transfer error clear 5
    u1 CGIF6      : 1;  //Channel x global interrupt clear 6
    u1 CTCIF6     : 1;  //Channel x transfer complete clear 6
    u1 CHTIF6     : 1;  //Channel x half transfer clear 6
    u1 CTEIF6     : 1;  //Channel x transfer error clear 6
    u1 CGIF7      : 1;  //Channel x global interrupt clear 7
    u1 CTCIF7     : 1;  //Channel x transfer complete clear 7
    u1 CHTIF7     : 1;  //Channel x half transfer clear 7
    u1 CTEIF7     : 1;  //Channel x transfer error clear 7
    u1 RSVD       : 4;  //Reserved
  }volatile IFCR;             //DMA interrupt flag clear register (DMA_IFCR)
  struct
  {
    struct
    {
      u1 EN         : 1;  //Channel enable
      u1 TCIE       : 1;  //Transfer complete interrupt enable
      u1 HTIE       : 1;  //Half transfer interrupt enable
      u1 TEIE       : 1;  //Transfer error interrupt enable
      u1 DIR        : 1;  //Data transfer direction
      u1 CIRC       : 1;  //Circular mode
      u1 PINC       : 1;  //Peripheral increment mode
      u1 MINC       : 1;  //Memory increment mode
      u1 PSIZE      : 2;  //Peripheral size
      u1 MSIZE      : 2;  //Memory size
      u1 PL         : 2;  //Channel priority level
      u1 MEM2MEM    : 1;  //Memory to memory mode  
      u1 RSVD       : 17; //Reserved
    }volatile CCR;              //DMA channel x configuration register (DMA_CCRx)
    struct
    {
      u1 NDT        : 16; //Number of data to transfer
      u1 RSVD       : 16; //Reserved
    }volatile CNDTR;            //DMA channel x number of data register (DMA_CNDTRx)
    volatile u32 CPAR;          //DMA channel x peripheral address register (DMA_CPARx)
    volatile u32 CMAR;          //DMA channel x memory address register (DMA_CMARx)
  }DMA_CH[7];
}DMA_Type;
#define DMA1 ((DMA_Type*) DMA1_BASE)
#define DMA2 ((DMA_Type*) DMA2_BASE)
#define DMA_CH1_IRQ_FLG(PORT)   PORT->ISR.GIF1                  //Channel 1 global interrupt flag
#define DMA_CH2_IRQ_FLG(PORT)   PORT->ISR.GIF2                  //Channel 2 global interrupt flag
#define DMA_CH3_IRQ_FLG(PORT)   PORT->ISR.GIF3                  //Channel 3 global interrupt flag
#define DMA_CH4_IRQ_FLG(PORT)   PORT->ISR.GIF4                  //Channel 4 global interrupt flag
#define DMA_CH5_IRQ_FLG(PORT)   PORT->ISR.GIF5                  //Channel 5 global interrupt flag
#define DMA_CH6_IRQ_FLG(PORT)   PORT->ISR.GIF6                  //Channel 6 global interrupt flag
#define DMA_CH7_IRQ_FLG(PORT)   PORT->ISR.GIF7                  //Channel 7 global interrupt flag
#define DMA_CH1_COMP_FLG(PORT)  PORT->ISR.TCIF1                 //Channel 1 transfer complete flag
#define DMA_CH2_COMP_FLG(PORT)  PORT->ISR.TCIF2                 //Channel 2 transfer complete flag
#define DMA_CH3_COMP_FLG(PORT)  PORT->ISR.TCIF3                 //Channel 3 transfer complete flag
#define DMA_CH4_COMP_FLG(PORT)  PORT->ISR.TCIF4                 //Channel 4 transfer complete flag
#define DMA_CH5_COMP_FLG(PORT)  PORT->ISR.TCIF5                 //Channel 5 transfer complete flag
#define DMA_CH6_COMP_FLG(PORT)  PORT->ISR.TCIF6                 //Channel 6 transfer complete flag
#define DMA_CH7_COMP_FLG(PORT)  PORT->ISR.TCIF7                 //Channel 7 transfer complete flag
#define DMA_CH1_HALF_FLG(PORT)  PORT->ISR.HTIF1                 //Channel 1 half transfer flag
#define DMA_CH2_HALF_FLG(PORT)  PORT->ISR.HTIF2                 //Channel 2 half transfer flag
#define DMA_CH3_HALF_FLG(PORT)  PORT->ISR.HTIF3                 //Channel 3 half transfer flag
#define DMA_CH4_HALF_FLG(PORT)  PORT->ISR.HTIF4                 //Channel 4 half transfer flag
#define DMA_CH5_HALF_FLG(PORT)  PORT->ISR.HTIF5                 //Channel 5 half transfer flag
#define DMA_CH6_HALF_FLG(PORT)  PORT->ISR.HTIF6                 //Channel 6 half transfer flag
#define DMA_CH7_HALF_FLG(PORT)  PORT->ISR.HTIF7                 //Channel 7 half transfer flag
#define DMA_CH1_ERR_FLG(PORT)   PORT->ISR.TEIF1                 //Channel 1 transfer error flag
#define DMA_CH2_ERR_FLG(PORT)   PORT->ISR.TEIF2                 //Channel 2 transfer error flag
#define DMA_CH3_ERR_FLG(PORT)   PORT->ISR.TEIF3                 //Channel 3 transfer error flag
#define DMA_CH4_ERR_FLG(PORT)   PORT->ISR.TEIF4                 //Channel 4 transfer error flag
#define DMA_CH5_ERR_FLG(PORT)   PORT->ISR.TEIF5                 //Channel 5 transfer error flag
#define DMA_CH6_ERR_FLG(PORT)   PORT->ISR.TEIF6                 //Channel 6 transfer error flag
#define DMA_CH7_ERR_FLG(PORT)   PORT->ISR.TEIF7                 //Channel 7 transfer error flag
#define DMA_CH1_IRQ_CLR(PORT)   PORT->IFCR.CGIF1 = ON           //Channel 1 global interrupt clear
#define DMA_CH2_IRQ_CLR(PORT)   PORT->IFCR.CGIF2 = ON           //Channel 2 global interrupt clear
#define DMA_CH3_IRQ_CLR(PORT)   PORT->IFCR.CGIF3 = ON           //Channel 3 global interrupt clear
#define DMA_CH4_IRQ_CLR(PORT)   PORT->IFCR.CGIF4 = ON           //Channel 4 global interrupt clear
#define DMA_CH5_IRQ_CLR(PORT)   PORT->IFCR.CGIF5 = ON           //Channel 5 global interrupt clear
#define DMA_CH6_IRQ_CLR(PORT)   PORT->IFCR.CGIF6 = ON           //Channel 6 global interrupt clear
#define DMA_CH7_IRQ_CLR(PORT)   PORT->IFCR.CGIF7 = ON           //Channel 7 global interrupt clear
#define DMA_CH1_COMP_CLR(PORT)  PORT->IFCR.CTCIF1 = ON          //Channel 1 transfer complete clear
#define DMA_CH2_COMP_CLR(PORT)  PORT->IFCR.CTCIF2 = ON          //Channel 2 transfer complete clear
#define DMA_CH3_COMP_CLR(PORT)  PORT->IFCR.CTCIF3 = ON          //Channel 3 transfer complete clear
#define DMA_CH4_COMP_CLR(PORT)  PORT->IFCR.CTCIF4 = ON          //Channel 4 transfer complete clear
#define DMA_CH5_COMP_CLR(PORT)  PORT->IFCR.CTCIF5 = ON          //Channel 5 transfer complete clear
#define DMA_CH6_COMP_CLR(PORT)  PORT->IFCR.CTCIF6 = ON          //Channel 6 transfer complete clear
#define DMA_CH7_COMP_CLR(PORT)  PORT->IFCR.CTCIF7 = ON          //Channel 7 transfer complete clear
#define DMA_CH1_HALF_CLR(PORT)  PORT->IFCR.CHTIF1 = ON          //Channel 1 half transfer clear
#define DMA_CH2_HALF_CLR(PORT)  PORT->IFCR.CHTIF2 = ON          //Channel 2 half transfer clear
#define DMA_CH3_HALF_CLR(PORT)  PORT->IFCR.CHTIF3 = ON          //Channel 3 half transfer clear
#define DMA_CH4_HALF_CLR(PORT)  PORT->IFCR.CHTIF4 = ON          //Channel 4 half transfer clear
#define DMA_CH5_HALF_CLR(PORT)  PORT->IFCR.CHTIF5 = ON          //Channel 5 half transfer clear
#define DMA_CH6_HALF_CLR(PORT)  PORT->IFCR.CHTIF6 = ON          //Channel 6 half transfer clear
#define DMA_CH7_HALF_CLR(PORT)  PORT->IFCR.CHTIF7 = ON          //Channel 7 half transfer clear
#define DMA_CH1_ERR_CLR(PORT)   PORT->IFCR.CTEIF1 = ON          //Channel 1 transfer error clear
#define DMA_CH2_ERR_CLR(PORT)   PORT->IFCR.CTEIF2 = ON          //Channel 2 transfer error clear
#define DMA_CH3_ERR_CLR(PORT)   PORT->IFCR.CTEIF3 = ON          //Channel 3 transfer error clear
#define DMA_CH4_ERR_CLR(PORT)   PORT->IFCR.CTEIF4 = ON          //Channel 4 transfer error clear
#define DMA_CH5_ERR_CLR(PORT)   PORT->IFCR.CTEIF5 = ON          //Channel 5 transfer error clear
#define DMA_CH6_ERR_CLR(PORT)   PORT->IFCR.CTEIF6 = ON          //Channel 6 transfer error clear
#define DMA_CH7_ERR_CLR(PORT)   PORT->IFCR.CTEIF7 = ON          //Channel 7 transfer error clear
#define DMA_EN(PORT,CH)         PORT->DMA_CH[CH].CCR.EN         //Channel enable
#define DMA_TC_IRQ(PORT,CH)     PORT->DMA_CH[CH].CCR.TCIE       //Transfer complete interrupt enable
#define DMA_HALF_IRQ(PORT,CH)   PORT->DMA_CH[CH].CCR.HTIE       //Half transfer interrupt enable
#define DMA_ERR_IRQ(PORT,CH)    PORT->DMA_CH[CH].CCR.TEIE       //Transfer error interrupt enable
#define DMA_DIR(PORT,CH)        PORT->DMA_CH[CH].CCR.DIR        //Data transfer direction
#define DMA_CIR(PORT,CH)        PORT->DMA_CH[CH].CCR.CIRC       //Circular mode
#define DMA_PER_INC(PORT,CH)    PORT->DMA_CH[CH].CCR.PINC       //Peripheral increment mode
#define DMA_MEM_INC(PORT,CH)    PORT->DMA_CH[CH].CCR.MINC       //Memory increment mode
#define DMA_PER_SIZE(PORT,CH)   PORT->DMA_CH[CH].CCR.PSIZE      //Peripheral size
#define DMA_MEM_SIZE(PORT,CH)   PORT->DMA_CH[CH].CCR.MSIZE      //Memory size
#define DMA_PRIOR_LVL(PORT,CH)  PORT->DMA_CH[CH].CCR.PL         //Channel priority level
#define DMA_MEM2MEM(PORT,CH)    PORT->DMA_CH[CH].CCR.MEM2MEM    //Memory to memory mode  
#define DMA_DATA_SIZE(PORT,CH)  PORT->DMA_CH[CH].CNDTR.NDT      //Number of data to transfer
#define DMA_PER_ADR(PORT,CH)    PORT->DMA_CH[CH].CPAR           //DMA channel x peripheral address register (DMA_CPARx)
#define DMA_MEM_ADR(PORT,CH)    PORT->DMA_CH[CH].CMAR           //DMA channel x memory address register (DMA_CMARx)
#define DMA_CH01                ((u8) 0x00)                     //DMA Channel1
#define DMA_CH02                ((u8) 0x01)                     //DMA Channel2
#define DMA_CH03                ((u8) 0x02)                     //DMA Channel3
#define DMA_CH04                ((u8) 0x03)                     //DMA Channel4
#define DMA_CH05                ((u8) 0x04)                     //DMA Channel5
#define DMA_CH06                ((u8) 0x05)                     //DMA Channel6
#define DMA_CH07                ((u8) 0x06)                     //DMA Channel7
/******************************************* SYSCFG ***********************************************/
typedef struct
{
  struct
  {
    u1 MEM_MODE           : 2;  //Memory mapping selection bits
    u1 RSVD0              : 2;  //Reserved
    u1 PA11_PA12_RMP      : 1;  //PA11 and PA12 remapping bit for small packages (28 and 20 pins).
    u1 RSVD1              : 1;  //Reserved
    u1 IRDA_ENV_SEL       : 2;  //IRDA Modulation Envelope signal source selection. Available on STM32F09x devices only.
    u1 ADC_DMA_RMP        : 1;  //ADC DMA request remapping bit. Available on STM32F03x, STM32F04x, STM32F05x and STM32F07x devices only.
    u1 USART1_TX_DMA_RMP  : 1;  //USART1_TX DMA request remapping bit. Available on STM32F03x, STM32F04x, STM32F05x and STM32F07x devices only.
    u1 USART1_RX_DMA_RMP  : 1;  //USART1_RX DMA request remapping bit. Available on STM32F03x, STM32F04x, STM32F05x and STM32F07x devices only.
    u1 TIM16_DMA_RMP      : 1;  //TIM16 DMA request remapping bit. Available on STM32F03x, STM32F04x, STM32F05x and STM32F07x devices only.
    u1 TIM17_DMA_RMP      : 1;  //TIM17 DMA request remapping bit. Available on STM32F03x, STM32F04x, STM32F05x and STM32F07x devices only.
    u1 TIM16_DMA_RMP2     : 1;  //TIM16 alternate DMA request remapping bit. Available on STM32F07x devices only.
    u1 TIM17_DMA_RMP2     : 1;  //TIM17 alternate DMA request remapping bit. Available on STM32F07x devices only.
    u1 RSVD2              : 1;  //Reserved
    u1 I2C_PB6_FMP        : 1;  //Fast Mode Plus (FM+) driving capability activation bits.
    u1 I2C_PB7_FMP        : 1;  //Fast Mode Plus (FM+) driving capability activation bits.
    u1 I2C_PB8_FMP        : 1;  //Fast Mode Plus (FM+) driving capability activation bits.
    u1 I2C_PB9_FMP        : 1;  //Fast Mode Plus (FM+) driving capability activation bits.
    u1 I2C1_FMP           : 1;  //FM+ driving capability activation for I2C1. Not available on STM32F05x devices.
    u1 I2C2_FMP           : 1;  //FM+ driving capability activation for I2C2. Available on STM32F07x and STM32F09x devices only.
    u1 I2C_PA9_FMP        : 1;  //Fast Mode Plus (FM+) driving capability activation bits. Available on STM32F03x, STM32F04x and STM32F09x devices only.
    u1 I2C_PA10_FMP       : 1;  //Fast Mode Plus (FM+) driving capability activation bits. Available on STM32F03x, STM32F04x and STM32F09x devices only.
    u1 SPI2_DMA_RMP       : 1;  //SPI2 DMA request remapping bit. Available on STM32F07x devices only.
    u1 USART2_DMA_RMP     : 1;  //USART2 DMA request remapping bit. Available on STM32F07x devices only.
    u1 USART3_DMA_RMP     : 1;  //USART3 DMA request remapping bit. Available on STM32F07x devices only.
    u1 I2C1_DMA_RMP       : 1;  //I2C1 DMA request remapping bit. Available on STM32F07x devices only.
    u1 TIM1_DMA_RMP       : 1;  //TIM1 DMA request remapping bit. Available on STM32F07x devices only.
    u1 TIM2_DMA_RMP       : 1;  //TIM2 DMA request remapping bit. Available on STM32F07x devices only.
    u1 TIM3_DMA_RMP       : 1;  //TIM3 DMA request remapping bit. Available on STM32F07x devices only.
    u1 RSVD3              : 1;  //Reserved
  }volatile CFGR1;                    //SYSCFG configuration register 1 (SYSCFG_CFGR1)
  volatile u32 RSVD0;                 //Reserved
  struct
  {
    u1 EXTI0              : 4;  //EXTI x configuration bits
    u1 EXTI1              : 4;  //EXTI x configuration bits
    u1 EXTI2              : 4;  //EXTI x configuration bits
    u1 EXTI3              : 4;  //EXTI x configuration bits
    u1 RSVD0              : 16; //Reserved
  }volatile EXTICR[4];                //SYSCFG external interrupt configuration register 1..4 (SYSCFG_EXTICR)
  struct
  {
    u1 LOCKUP_LOCK        : 1;  //Cortex-M0 LOCKUP bit enable bit
    u1 SRAM_PARITY_LOCK   : 1;  //SRAM parity lock bit
    u1 PVD_LOCK           : 1;  //PVD lock enable bit
    u1 RSVD0              : 5;  //Reserved
    u1 SRAM_PEF           : 1;  //SRAM parity error flag
    u1 RSVD1              : 23; //Reserved
  }volatile CFGR2;                    //SYSCFG configuration register 2 (SYSCFG_CFGR2)
  volatile u32 RSVD1[24];             //Reserved
  struct
  {
    u1 WWDG               : 1;  //Window watchdog interrupt pending flag
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE0;                  //SYSCFG interrupt line 0 status register (SYSCFG_ITLINE0)
  struct
  {
    u1 PVDOUT             : 1;  //PVD supply monitoring interrupt request pending (EXTI line 16). This bit is not available on STM32F0x8 devices.
    u1 VDDIO2             : 1;  //VDDIO2 supply monitoring interrupt request pending (EXTI line 31)
    u1 RSVD0              : 30; //Reserved
  }volatile ITLINE1;                  //SYSCFG interrupt line 1 status register (SYSCFG_ITLINE1)
  struct
  {
    u1 RTC_WAKEUP         : 1;  //RTC Wake Up interrupt request pending (EXTI line 20)
    u1 RTC_TSTAMP         : 1;  //RTC Tamper and TimeStamp interrupt request pending (EXTI line 19)
    u1 RTC_ALRA           : 1;  //RTC Alarm interrupt request pending (EXTI line 17)
    u1 RSVD0              : 29; //Reserved
  }volatile ITLINE2;                  //SYSCFG interrupt line 2 status register (SYSCFG_ITLINE2)
  struct
  {
    u1 FLASH_ITF          : 1;  //Flash interface interrupt request pending
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE3;                  //SYSCFG interrupt line 3 status register (SYSCFG_ITLINE3)
  struct
  {
    u1 RCCR               : 1;  //Reset and clock control interrupt request pending
    u1 CRS                : 1;  //Clock recovery system interrupt request pending
    u1 RSVD0              : 30; //Reserved
  }volatile ITLINE4;                  //SYSCFG interrupt line 4 status register (SYSCFG_ITLINE4)
  struct
  {
    u1 EXTI0              : 1;  //EXTI line 0 interrupt request pending
    u1 EXTI1              : 1;  //EXTI line 1 interrupt request pending
    u1 RSVD0              : 30; //Reserved
  }volatile ITLINE5;                  //SYSCFG interrupt line 5 status register (SYSCFG_ITLINE5)
  struct
  {
    u1 EXTI2              : 1;  //EXTI line 2 interrupt request pending
    u1 EXTI3              : 1;  //EXTI line 3 interrupt request pending
    u1 RSVD0              : 30; //Reserved
  }volatile ITLINE6;                  //SYSCFG interrupt line 6 status register (SYSCFG_ITLINE6)
  struct
  {
    u1 EXTI4              : 1;  //EXTI line 4 interrupt request pending
    u1 EXTI5              : 1;  //EXTI line 5 interrupt request pending
    u1 EXTI6              : 1;  //EXTI line 6 interrupt request pending
    u1 EXTI7              : 1;  //EXTI line 7 interrupt request pending
    u1 EXTI8              : 1;  //EXTI line 8 interrupt request pending
    u1 EXTI9              : 1;  //EXTI line 9 interrupt request pending
    u1 EXTI10             : 1;  //EXTI line 10 interrupt request pending
    u1 EXTI11             : 1;  //EXTI line 11 interrupt request pending
    u1 EXTI12             : 1;  //EXTI line 12 interrupt request pending
    u1 EXTI13             : 1;  //EXTI line 13 interrupt request pending
    u1 EXTI14             : 1;  //EXTI line 14 interrupt request pending
    u1 EXTI15             : 1;  //EXTI line 15 interrupt request pending
    u1 RSVD0              : 20; //Reserved
  }volatile ITLINE7;                  //SYSCFG interrupt line 7 status register (SYSCFG_ITLINE7)
  struct
  {
    u1 TCS_MCE            : 1;  //Touch sensing controller max count error interrupt request pending
    u1 TCS_EOA            : 1;  //Touch sensing controller end of acquisition interrupt request pending
    u1 RSVD0              : 30; //Reserved
  }volatile ITLINE8;                  //SYSCFG interrupt line 8 status register (SYSCFG_ITLINE8)
  struct
  {
    u1 DMA1_CH1           : 1;  //DMA1 channel 1 interrupt request pending
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE9;                  //SYSCFG interrupt line 9 status register (SYSCFG_ITLINE9)
  struct
  {
    u1 DMA1_CH2           : 1;  //DMA1 channel 2 interrupt request pending
    u1 DMA1_CH3           : 1;  //DMA1 channel 3 interrupt request pending
    u1 DMA2_CH1           : 1;  //DMA2 channel 1 interrupt request pending
    u1 DMA2_CH2           : 1;  //DMA2 channel 2 interrupt request pending
    u1 RSVD0              : 28; //Reserved
  }volatile ITLINE10;                 //SYSCFG interrupt line 10 status register (SYSCFG_ITLINE10)
  struct
  {
    u1 DMA1_CH4           : 1;  //DMA1 channel 4 interrupt request pending
    u1 DMA1_CH5           : 1;  //DMA1 channel 5 interrupt request pending
    u1 DMA1_CH6           : 1;  //DMA1 channel 6 interrupt request pending
    u1 DMA1_CH7           : 1;  //DMA1 channel 7 interrupt request pending
    u1 DMA2_CH3           : 1;  //DMA2 channel 3 interrupt request pending
    u1 DMA2_CH4           : 1;  //DMA2 channel 4 interrupt request pending
    u1 DMA2_CH5           : 1;  //DMA2 channel 5 interrupt request pending
    u1 RSVD0              : 25; //Reserved
  }volatile ITLINE11;                 //SYSCFG interrupt line 11 status register (SYSCFG_ITLINE11)
  struct
  {
    u1 ADCR               : 1;  //ADC interrupt request pending
    u1 COMP1              : 1;  //Comparator 1 interrupt request pending (EXTI line 21)
    u1 COMP2              : 1;  //Comparator 2 interrupt request pending (EXTI line 22)
    u1 RSVD0              : 29; //Reserved
  }volatile ITLINE12;                 //SYSCFG interrupt line 12 status register (SYSCFG_ITLINE12)
  struct
  {
    u1 TIM1_CCU           : 1;  //Timer 1 commutation interrupt request pending
    u1 TIM1_TRG           : 1;  //Timer 1 trigger interrupt request pending
    u1 TIM1_UPD           : 1;  //Timer 1 update interrupt request pending
    u1 TIM1_BRK           : 1;  //Timer 1 break interrupt request pending
    u1 RSVD0              : 28; //Reserved
  }volatile ITLINE13;                 //SYSCFG interrupt line 13 status register (SYSCFG_ITLINE13)
  struct
  {
    u1 TIM1_CC            : 1;  //Timer 1 capture compare interrupt request pending
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE14;                 //SYSCFG interrupt line 14 status register (SYSCFG_ITLINE14)
  struct
  {
    u1 TIM2               : 1;  //Timer 2 interrupt request pending
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE15;                 //SYSCFG interrupt line 15 status register (SYSCFG_ITLINE15)
  struct
  {
    u1 TIM3               : 1;  //Timer 3 interrupt request pending
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE16;                 //SYSCFG interrupt line 16 status register (SYSCFG_ITLINE16)
  struct
  {
    u1 TIM6               : 1;  //Timer 6 interrupt request pending
    u1 DAC                : 1;  //DAC underrun interrupt request pending
    u1 RSVD0              : 30; //Reserved
  }volatile ITLINE17;                 //SYSCFG interrupt line 17 status register (SYSCFG_ITLINE17)
  struct
  {
    u1 TIM7               : 1;  //Timer 7 interrupt request pending
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE18;                 //SYSCFG interrupt line 18 status register (SYSCFG_ITLINE18)
  struct
  {
    u1 TIM14              : 1;  //Timer 14 interrupt request pending
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE19;                 //SYSCFG interrupt line 19 status register (SYSCFG_ITLINE19)
  struct
  {
    u1 TIM15              : 1;  //Timer 15 interrupt request pending
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE20;                 //SYSCFG interrupt line 20 status register (SYSCFG_ITLINE20)
  struct
  {
    u1 TIM16              : 1;  //Timer 16 interrupt request pending
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE21;                 //SYSCFG interrupt line 21 status register (SYSCFG_ITLINE21)
  struct
  {
    u1 TIM17              : 1;  //Timer 17 interrupt request pending
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE22;                 //SYSCFG interrupt line 22 status register (SYSCFG_ITLINE22)
  struct
  {
    u1 I2C1               : 1;  //I2C1 interrupt request pending, combined with EXTI line 23
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE23;                 //SYSCFG interrupt line 23 status register (SYSCFG_ITLINE23)
  struct
  {
    u1 I2C2               : 1;  //I2C2 interrupt request pending
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE24;                 //SYSCFG interrupt line 24 status register (SYSCFG_ITLINE24)
  struct
  {
    u1 SPI1               : 1;  //SPI1 interrupt request pending
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE25;                 //SYSCFG interrupt line 25 status register (SYSCFG_ITLINE25)
  struct
  {
    u1 SPI2               : 1;  //SPI2 interrupt request pending
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE26;                 //SYSCFG interrupt line 26 status register (SYSCFG_ITLINE26)
  struct
  {
    u1 USART1             : 1;  //USART1 interrupt request pending, combined with EXTI line 25
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE27;                 //SYSCFG interrupt line 27 status register (SYSCFG_ITLINE27)
  struct
  {
    u1 USART2             : 1;  //USART2 interrupt request pending, combined with EXTI line 26
    u1 RSVD0              : 31; //Reserved
  }volatile ITLINE28;                 //SYSCFG interrupt line 28 status register (SYSCFG_ITLINE28)
  struct
  {
    u1 USART3             : 1;  //USART3 interrupt request pending, combined with EXTI line 28
    u1 USART4             : 1;  //USART4 interrupt request pending
    u1 USART5             : 1;  //USART5 interrupt request pending
    u1 USART6             : 1;  //USART6 interrupt request pending
    u1 USART7             : 1;  //USART7 interrupt request pending
    u1 USART8             : 1;  //USART8 interrupt request pending
    u1 RSVD0              : 26; //Reserved
  }volatile ITLINE29;                 //SYSCFG interrupt line 29 status register (SYSCFG_ITLINE29)
  struct
  {
    u1 CEC                : 1;  //CEC interrupt request pending, combined with EXTI line 27
    u1 CAN                : 1;  //CAN interrupt request pending
    u1 RSVD0              : 30; //Reserved
  }volatile ITLINE30;                 //SYSCFG interrupt line 30 status register (SYSCFG_ITLINE30)
}SYSCFG_Type;
#define SYSCFG ((SYSCFG_Type*) SYSCFG_BASE)
#define SYSCFG_MEM_MODE         SYSCFG->CFGR1.MEM_MODE          //Memory mapping selection bits
#define SYSCFG_PA11_PA12_RMP    SYSCFG->CFGR1.PA11_PA12_RMP     //PA11 and PA12 remapping bit for small packages (28 and 20 pins).
#define SYSCFG_IRDA_ENV_SEL     SYSCFG->CFGR1.IRDA_ENV_SEL      //IRDA Modulation Envelope signal source selection. Available on STM32F09x devices only.
#define SYSCFG_ADC_DMA_RMP      SYSCFG->CFGR1.ADC_DMA_RMP       //ADC DMA request remapping bit. Available on STM32F03x, STM32F04x, STM32F05x and STM32F07x devices only.
#define SYSCFG_USART1_TX_DMA    SYSCFG->CFGR1.USART1_TX_DMA_RMP //USART1_TX DMA request remapping bit. Available on STM32F03x, STM32F04x, STM32F05x and STM32F07x devices only.
#define SYSCFG_USART1_RX_DMA    SYSCFG->CFGR1.USART1_RX_DMA_RMP //USART1_RX DMA request remapping bit. Available on STM32F03x, STM32F04x, STM32F05x and STM32F07x devices only.
#define SYSCFG_TIM16_DMA_RMP    SYSCFG->CFGR1.TIM16_DMA_RMP     //TIM16 DMA request remapping bit. Available on STM32F03x, STM32F04x, STM32F05x and STM32F07x devices only.
#define SYSCFG_TIM17_DMA_RMP    SYSCFG->CFGR1.TIM17_DMA_RMP     //TIM17 DMA request remapping bit. Available on STM32F03x, STM32F04x, STM32F05x and STM32F07x devices only.
#define SYSCFG_TIM16_DMA_RMP2   SYSCFG->CFGR1.TIM16_DMA_RMP2    //TIM16 alternate DMA request remapping bit. Available on STM32F07x devices only.
#define SYSCFG_TIM17_DMA_RMP2   SYSCFG->CFGR1.TIM17_DMA_RMP2    //TIM17 alternate DMA request remapping bit. Available on STM32F07x devices only.
#define SYSCFG_I2C_PB6_FMP      SYSCFG->CFGR1.I2C_PB6_FMP       //Fast Mode Plus (FM+) driving capability activation bits.
#define SYSCFG_I2C_PB7_FMP      SYSCFG->CFGR1.I2C_PB7_FMP       //Fast Mode Plus (FM+) driving capability activation bits.
#define SYSCFG_I2C_PB8_FMP      SYSCFG->CFGR1.I2C_PB8_FMP       //Fast Mode Plus (FM+) driving capability activation bits.
#define SYSCFG_I2C_PB9_FMP      SYSCFG->CFGR1.I2C_PB9_FMP       //Fast Mode Plus (FM+) driving capability activation bits.
#define SYSCFG_I2C1_FMP         SYSCFG->CFGR1.I2C1_FMP          //FM+ driving capability activation for I2C1. Not available on STM32F05x devices.
#define SYSCFG_I2C2_FMP         SYSCFG->CFGR1.I2C2_FMP          //FM+ driving capability activation for I2C2. Available on STM32F07x and STM32F09x devices only.
#define SYSCFG_I2C_PA9_FMP      SYSCFG->CFGR1.I2C_PA9_FMP       //Fast Mode Plus (FM+) driving capability activation bits. Available on STM32F03x, STM32F04x and STM32F09x devices only.
#define SYSCFG_I2C_PA10_FMP     SYSCFG->CFGR1.I2C_PA10_FMP      //Fast Mode Plus (FM+) driving capability activation bits. Available on STM32F03x, STM32F04x and STM32F09x devices only.
#define SYSCFG_SPI2_DMA_RMP     SYSCFG->CFGR1.SPI2_DMA_RMP      //SPI2 DMA request remapping bit. Available on STM32F07x devices only.
#define SYSCFG_USART2_DMA_RMP   SYSCFG->CFGR1.USART2_DMA_RMP    //USART2 DMA request remapping bit. Available on STM32F07x devices only.
#define SYSCFG_USART3_DMA_RMP   SYSCFG->CFGR1.USART3_DMA_RMP    //USART3 DMA request remapping bit. Available on STM32F07x devices only.
#define SYSCFG_I2C1_DMA_RMP     SYSCFG->CFGR1.I2C1_DMA_RMP      //I2C1 DMA request remapping bit. Available on STM32F07x devices only.
#define SYSCFG_TIM1_DMA_RMP     SYSCFG->CFGR1.TIM1_DMA_RMP      //TIM1 DMA request remapping bit. Available on STM32F07x devices only.
#define SYSCFG_TIM2_DMA_RMP     SYSCFG->CFGR1.TIM2_DMA_RMP      //TIM2 DMA request remapping bit. Available on STM32F07x devices only.
#define SYSCFG_TIM3_DMA_RMP     SYSCFG->CFGR1.TIM3_DMA_RMP      //TIM3 DMA request remapping bit. Available on STM32F07x devices only.
#define SYSCFG_EXTI00           SYSCFG->EXTICR[0].EXTI0         //EXTI 0 configuration bits
#define SYSCFG_EXTI01           SYSCFG->EXTICR[0].EXTI1         //EXTI 1 configuration bits
#define SYSCFG_EXTI02           SYSCFG->EXTICR[0].EXTI2         //EXTI 2 configuration bits
#define SYSCFG_EXTI03           SYSCFG->EXTICR[0].EXTI3         //EXTI 3 configuration bits
#define SYSCFG_EXTI04           SYSCFG->EXTICR[1].EXTI0         //EXTI 4 configuration bits
#define SYSCFG_EXTI05           SYSCFG->EXTICR[1].EXTI1         //EXTI 5 configuration bits
#define SYSCFG_EXTI06           SYSCFG->EXTICR[1].EXTI2         //EXTI 6 configuration bits
#define SYSCFG_EXTI07           SYSCFG->EXTICR[1].EXTI3         //EXTI 7 configuration bits
#define SYSCFG_EXTI08           SYSCFG->EXTICR[2].EXTI0         //EXTI 8 configuration bits
#define SYSCFG_EXTI09           SYSCFG->EXTICR[2].EXTI1         //EXTI 9 configuration bits
#define SYSCFG_EXTI10           SYSCFG->EXTICR[2].EXTI2         //EXTI 10 configuration bits
#define SYSCFG_EXTI11           SYSCFG->EXTICR[2].EXTI3         //EXTI 11 configuration bits
#define SYSCFG_EXTI12           SYSCFG->EXTICR[3].EXTI0         //EXTI 12 configuration bits
#define SYSCFG_EXTI13           SYSCFG->EXTICR[3].EXTI1         //EXTI 13 configuration bits
#define SYSCFG_EXTI14           SYSCFG->EXTICR[3].EXTI2         //EXTI 14 configuration bits
#define SYSCFG_EXTI15           SYSCFG->EXTICR[3].EXTI3         //EXTI 15 configuration bits
#define SYSCFG_LOCKUP_LOCK      SYSCFG->CFGR2.LOCKUP_LOCK       //Cortex-M0 LOCKUP bit enable bit
#define SYSCFG_SRAM_LOCK        SYSCFG->CFGR2.SRAM_PARITY_LOCK  //SRAM parity lock bit
#define SYSCFG_PVD_LOCK         SYSCFG->CFGR2.PVD_LOCK          //PVD lock enable bit
#define SYSCFG_SRAM_PE_FLG      SYSCFG->CFGR2.SRAM_PEF          //SRAM parity error flag
#define SYSCFG_WWDG_IRQ         SYSCFG->ITLINE0.WWDG            //Window watchdog interrupt pending flag
#define SYSCFG_PVDOUT_IRQ       SYSCFG->ITLINE1.PVDOUT          //PVD supply monitoring interrupt request pending (EXTI line 16). This bit is not available on STM32F0x8 devices.
#define SYSCFG_VDDIO2_IRQ       SYSCFG->ITLINE1.VDDIO2          //VDDIO2 supply monitoring interrupt request pending (EXTI line 31)
#define SYSCFG_RTC_WAKEUP_IRQ   SYSCFG->ITLINE2.RTC_WAKEUP      //RTC Wake Up interrupt request pending (EXTI line 20)
#define SYSCFG_RTC_TSTAMP_IRQ   SYSCFG->ITLINE2.RTC_TSTAMP      //RTC Tamper and TimeStamp interrupt request pending (EXTI line 19)
#define SYSCFG_RTC_ALR_IRQ      SYSCFG->ITLINE2.RTC_ALRA        //RTC Alarm interrupt request pending (EXTI line 17)
#define SYSCFG_FLASH_IRQ        SYSCFG->ITLINE3.FLASH_ITF       //Flash interface interrupt request pending
#define SYSCFG_RCC_IRQ          SYSCFG->ITLINE4.RCCR            //Reset and clock control interrupt request pending
#define SYSCFG_CRS_IRQ          SYSCFG->ITLINE4.CRS             //Clock recovery system interrupt request pending
#define SYSCFG_EXTI00_IRQ       SYSCFG->ITLINE5.EXTI0           //EXTI line 0 interrupt request pending
#define SYSCFG_EXTI01_IRQ       SYSCFG->ITLINE5.EXTI1           //EXTI line 1 interrupt request pending
#define SYSCFG_EXTI02_IRQ       SYSCFG->ITLINE6.EXTI2           //EXTI line 2 interrupt request pending
#define SYSCFG_EXTI03_IRQ       SYSCFG->ITLINE6.EXTI3           //EXTI line 3 interrupt request pending
#define SYSCFG_EXTI04_IRQ       SYSCFG->ITLINE7.EXTI4           //EXTI line 4 interrupt request pending
#define SYSCFG_EXTI05_IRQ       SYSCFG->ITLINE7.EXTI5           //EXTI line 5 interrupt request pending
#define SYSCFG_EXTI06_IRQ       SYSCFG->ITLINE7.EXTI6           //EXTI line 6 interrupt request pending
#define SYSCFG_EXTI07_IRQ       SYSCFG->ITLINE7.EXTI7           //EXTI line 7 interrupt request pending
#define SYSCFG_EXTI08_IRQ       SYSCFG->ITLINE7.EXTI8           //EXTI line 8 interrupt request pending
#define SYSCFG_EXTI09_IRQ       SYSCFG->ITLINE7.EXTI9           //EXTI line 9 interrupt request pending
#define SYSCFG_EXTI10_IRQ       SYSCFG->ITLINE7.EXTI10          //EXTI line 10 interrupt request pending
#define SYSCFG_EXTI11_IRQ       SYSCFG->ITLINE7.EXTI11          //EXTI line 11 interrupt request pending
#define SYSCFG_EXTI12_IRQ       SYSCFG->ITLINE7.EXTI12          //EXTI line 12 interrupt request pending
#define SYSCFG_EXTI13_IRQ       SYSCFG->ITLINE7.EXTI13          //EXTI line 13 interrupt request pending
#define SYSCFG_EXTI14_IRQ       SYSCFG->ITLINE7.EXTI14          //EXTI line 14 interrupt request pending
#define SYSCFG_EXTI15_IRQ       SYSCFG->ITLINE7.EXTI15          //EXTI line 15 interrupt request pending
#define SYSCFG_TCS_MCE_IRQ      SYSCFG->ITLINE8.TCS_MCE         //Touch sensing controller max count error interrupt request pending
#define SYSCFG_TCS_EOA_IRQ      SYSCFG->ITLINE8.TCS_EOA         //Touch sensing controller end of acquisition interrupt request pending
#define SYSCFG_DMA1_CH1_IRQ     SYSCFG->ITLINE9.DMA1_CH1        //DMA1 channel 1 interrupt request pending
#define SYSCFG_DMA1_CH2_IRQ     SYSCFG->ITLINE10.DMA1_CH2       //DMA1 channel 2 interrupt request pending
#define SYSCFG_DMA1_CH3_IRQ     SYSCFG->ITLINE10.DMA1_CH3       //DMA1 channel 3 interrupt request pending
#define SYSCFG_DMA2_CH1_IRQ     SYSCFG->ITLINE10.DMA2_CH1       //DMA2 channel 1 interrupt request pending
#define SYSCFG_DMA2_CH2_IRQ     SYSCFG->ITLINE10.DMA2_CH2       //DMA2 channel 2 interrupt request pending
#define SYSCFG_DMA1_CH4_IRQ     SYSCFG->ITLINE11.DMA1_CH4       //DMA1 channel 4 interrupt request pending
#define SYSCFG_DMA1_CH5_IRQ     SYSCFG->ITLINE11.DMA1_CH5       //DMA1 channel 5 interrupt request pending
#define SYSCFG_DMA1_CH6_IRQ     SYSCFG->ITLINE11.DMA1_CH6       //DMA1 channel 6 interrupt request pending
#define SYSCFG_DMA1_CH7_IRQ     SYSCFG->ITLINE11.DMA1_CH7       //DMA1 channel 7 interrupt request pending
#define SYSCFG_DMA2_CH3_IRQ     SYSCFG->ITLINE11.DMA2_CH3       //DMA2 channel 3 interrupt request pending
#define SYSCFG_DMA2_CH4_IRQ     SYSCFG->ITLINE11.DMA2_CH4       //DMA2 channel 4 interrupt request pending
#define SYSCFG_DMA2_CH5_IRQ     SYSCFG->ITLINE11.DMA2_CH5       //DMA2 channel 5 interrupt request pending
#define SYSCFG_ADC_IRQ          SYSCFG->ITLINE12.ADCR           //ADC interrupt request pending
#define SYSCFG_COMP1_IRQ        SYSCFG->ITLINE12.COMP1          //Comparator 1 interrupt request pending (EXTI line 21)
#define SYSCFG_COMP2_IRQ        SYSCFG->ITLINE12.COMP2          //Comparator 2 interrupt request pending (EXTI line 22)
#define SYSCFG_TIM1_CCU_IRQ     SYSCFG->ITLINE13.TIM1_CCU       //Timer 1 commutation interrupt request pending
#define SYSCFG_TIM1_TRG_IRQ     SYSCFG->ITLINE13.TIM1_TRG       //Timer 1 trigger interrupt request pending
#define SYSCFG_TIM1_UPD_IRQ     SYSCFG->ITLINE13.TIM1_UPD       //Timer 1 update interrupt request pending
#define SYSCFG_TIM1_BRK_IRQ     SYSCFG->ITLINE13.TIM1_BRK       //Timer 1 break interrupt request pending
#define SYSCFG_TIM1_CC_IRQ      SYSCFG->ITLINE14.TIM1_CC        //Timer 1 capture compare interrupt request pending
#define SYSCFG_TIM2_IRQ         SYSCFG->ITLINE15.TIM2           //Timer 2 interrupt request pending
#define SYSCFG_TIM3_IRQ         SYSCFG->ITLINE16.TIM3           //Timer 3 interrupt request pending
#define SYSCFG_TIM6_IRQ         SYSCFG->ITLINE17.TIM6           //Timer 6 interrupt request pending
#define SYSCFG_DAC_IRQ          SYSCFG->ITLINE17.DAC            //DAC underrun interrupt request pending
#define SYSCFG_TIM7_IRQ         SYSCFG->ITLINE18.TIM7           //Timer 7 interrupt request pending
#define SYSCFG_TIM14_IRQ        SYSCFG->ITLINE19.TIM14          //Timer 14 interrupt request pending
#define SYSCFG_TIM15_IRQ        SYSCFG->ITLINE20.TIM15          //Timer 15 interrupt request pending
#define SYSCFG_TIM16_IRQ        SYSCFG->ITLINE21.TIM16          //Timer 16 interrupt request pending
#define SYSCFG_TIM17_IRQ        SYSCFG->ITLINE22.TIM17          //Timer 17 interrupt request pending
#define SYSCFG_I2C1_IRQ         SYSCFG->ITLINE23.I2C1           //I2C1 interrupt request pending, combined with EXTI line 23
#define SYSCFG_I2C2_IRQ         SYSCFG->ITLINE24.I2C2           //I2C2 interrupt request pending
#define SYSCFG_SPI1_IRQ         SYSCFG->ITLINE25.SPI1           //SPI1 interrupt request pending
#define SYSCFG_SPI2_IRQ         SYSCFG->ITLINE26.SPI2           //SPI2 interrupt request pending
#define SYSCFG_USART1_IRQ       SYSCFG->ITLINE27.USART1         //USART1 interrupt request pending, combined with EXTI line 25
#define SYSCFG_USART2_IRQ       SYSCFG->ITLINE28.USART2         //USART2 interrupt request pending, combined with EXTI line 26
#define SYSCFG_USART3_IRQ       SYSCFG->ITLINE29.USART3         //USART3 interrupt request pending, combined with EXTI line 28
#define SYSCFG_USART4_IRQ       SYSCFG->ITLINE29.USART4         //USART4 interrupt request pending
#define SYSCFG_USART5_IRQ       SYSCFG->ITLINE29.USART5         //USART5 interrupt request pending
#define SYSCFG_USART6_IRQ       SYSCFG->ITLINE29.USART6         //USART6 interrupt request pending
#define SYSCFG_USART7_IRQ       SYSCFG->ITLINE29.USART7         //USART7 interrupt request pending
#define SYSCFG_USART8_IRQ       SYSCFG->ITLINE29.USART8         //USART8 interrupt request pending
#define SYSCFG_CEC_IRQ          SYSCFG->ITLINE30.CEC            //CEC interrupt request pending, combined with EXTI line 27
#define SYSCFG_CAN_IRQ          SYSCFG->ITLINE30.CAN            //CAN interrupt request pending
#define SYSCFG_EXTI_PA          ((u8) 0x00)                     //PA[x] pin
#define SYSCFG_EXTI_PB          ((u8) 0x01)                     //PB[x] pin
#define SYSCFG_EXTI_PC          ((u8) 0x02)                     //PC[x] pin
#define SYSCFG_EXTI_PD          ((u8) 0x03)                     //PD[x] pin
#define SYSCFG_EXTI_PE          ((u8) 0x04)                     //PE[x] pin
#define SYSCFG_EXTI_PF          ((u8) 0x05)                     //PF[x] pin
/******************************************* FLASH ************************************************/
typedef struct
{
  struct
  {
    u1 LATENCY    : 3;  //Latency
    u1 RSVD0      : 1;  //Reserved
    u1 PRFTBE     : 1;  //Prefetch buffer enable
    u1 PRFTBS     : 1;  //Prefetch buffer status
    u1 RSVD1      : 26; //Reserved
  }volatile ACR;              //Flash access control register (FLASH_ACR)
  volatile u32 KEYR;          //Flash key register (FLASH_KEYR)
  volatile u32 OPTKEYR;       //Flash option key register (FLASH_OPTKEYR)
  struct
  {
    u1 BSY        : 1;  //Busy
    u1 RSVD0      : 1;  //Reserved
    u1 PGERR      : 1;  //Programming error
    u1 RSVD1      : 1;  //Reserved
    u1 WRPRTERR   : 1;  //Write protection error
    u1 EOP        : 1;  //End of operation
    u1 RSVD2      : 26; //Reserved
  }volatile SR;               //Flash status register (FLASH_SR)
  struct
  {
    u1 PG         : 1;  //Programming
    u1 PER        : 1;  //Page erase
    u1 MER        : 1;  //Mass erase
    u1 RSVD0      : 1;  //Reserved
    u1 OPTPG      : 1;  //Option byte programming
    u1 OPTER      : 1;  //Option byte erase
    u1 STRT       : 1;  //Start
    u1 LOCK       : 1;  //Lock
    u1 RSVD1      : 1;  //Reserved
    u1 OPTWRE     : 1;  //Option bytes write enable
    u1 ERRIE      : 1;  //Error interrupt enable
    u1 RSVD2      : 1;  //Reserved
    u1 EOPIE      : 1;  //End of operation interrupt enable
    u1 OBL_LAUNCH : 1;  //Force option byte loading
    u1 RSVD3      : 18; //Reserved
  }volatile CR;               //Flash control register (FLASH_CR)
  volatile u32 AR;            //Flash address register (FLASH_AR)
  volatile u32 RSVD0;         //Reserved
  struct
  {
    u1 OPTERR     : 1;  //Option byte error
    u1 RDPRT      : 2;  //Read protection level status
    u1 RSVD0      : 5;  //Reserved
    u1 WDG_SW     : 1;  //WDG_SW
    u1 RST_STOP   : 1;  //RST_STOP
    u1 RST_STDBY  : 1;  //RST_STDBY
    u1 BOOT0      : 1;  //(available on STM32F04x and STM32F09x devices only)
    u1 BOOT1      : 1;  //BOOT1
    u1 VDDA_MNTR  : 1;  //VDDA_MONITOR
    u1 RAM_PC     : 1;  //RAM_PARITY_CHECK
    u1 BOOT_SEL   : 1;  //(available on STM32F04x and STM32F09x devices only)
    u1 DATA0      : 8;  //DATA0
    u1 DATA1      : 8;  //DATA1
  }volatile OBR;              //Option byte register (FLASH_OBR)
  volatile u32 WRPR;          //Write protection register (FLASH_WRPR)
}FLASH_Type;
#define FLASH ((FLASH_Type*) FLASH_R_BASE)
#define FLASH_LATENCY           FLASH->ACR.LATENCY      //Latency
#define FLASH_PRFTB_EN          FLASH->ACR.PRFTBE       //Prefetch buffer enable
#define FLASH_PRFTB_FLG         FLASH->ACR.PRFTBS       //Prefetch buffer status
#define FLASH_KEY               FLASH->KEYR             //FPEC key register (FLASH_KEYR)
#define FLASH_OPTKEY            FLASH->OPTKEYR          //Flash OPTKEY register (FLASH_OPTKEYR)
#define FLASH_BSY               FLASH->SR.BSY           //Busy
#define FLASH_PGERR             FLASH->SR.PGERR         //Programming error
#define FLASH_WRPRTERR          FLASH->SR.WRPRTERR      //Write protection error
#define FLASH_EOP               FLASH->SR.EOP           //End of operation
#define FLASH_PG                FLASH->CR.PG            //Programming
#define FLASH_PER               FLASH->CR.PER           //Page erase
#define FLASH_MER               FLASH->CR.MER           //Mass erase
#define FLASH_OPTPG             FLASH->CR.OPTPG         //Option byte programming
#define FLASH_OPTER             FLASH->CR.OPTER         //Option byte erase
#define FLASH_STRT              FLASH->CR.STRT          //Start
#define FLASH_LOCK              FLASH->CR.LOCK          //Lock
#define FLASH_OPTWRE            FLASH->CR.OPTWRE        //Option bytes write enable
#define FLASH_ERRIE             FLASH->CR.ERRIE         //Error interrupt enable
#define FLASH_EOPIE             FLASH->CR.EOPIE         //End of operation interrupt enable
#define FLASH_OBL_LAUNCH        FLASH->CR.OBL_LAUNCH    //Force option byte loading
#define FLASH_AR                FLASH->AR               //Flash address register (FLASH_AR)
#define FLASH_OPTERR            FLASH->OBR.OPTERR       //Option byte error
#define FLASH_RDPRT             FLASH->OBR.RDPRT        //Read protection level status
#define FLASH_WDG_SW            FLASH->OBR.WDG_SW       //WDG_SW
#define FLASH_RST_STOP          FLASH->OBR.RST_STOP     //RST_STOP
#define FLASH_RST_STDBY         FLASH->OBR.RST_STDBY    //RST_STDBY
#define FLASH_BOOT0             FLASH->OBR.BOOT0        //(available on STM32F04x and STM32F09x devices only)
#define FLASH_BOOT1             FLASH->OBR.BOOT1        //BOOT1
#define FLASH_VDDA_MNTR         FLASH->OBR.VDDA_MNTR    //VDDA_MONITOR
#define FLASH_RAM_PC            FLASH->OBR.RAM_PC       //RAM_PARITY_CHECK
#define FLASH_BOOT_SEL          FLASH->OBR.BOOT_SEL     //(available on STM32F04x and STM32F09x devices only)
#define FLASH_DATA0             FLASH->OBR.DATA0        //DATA0
#define FLASH_DATA1             FLASH->OBR.DATA1        //DATA1
#define FLASH_WRPR              FLASH->WRPR             //Write protection register (FLASH_WRPR)
#define FLASH_KEY1              ((u32) 0x45670123)      //Flash unlock key 1
#define FLASH_KEY2              ((u32) 0xCDEF89AB)      //Flash unlock key 2
#define FLASH_UNLOCK            FLASH_KEY = FLASH_KEY1; FLASH_KEY = FLASH_KEY2
#define FLASH_OB_UNLOCK         FLASH_OPTKEY = FLASH_KEY1; FLASH_OPTKEY = FLASH_KEY2
#define FLASH_LTC0              ((u8) 0x00)             //Zero wait state, if 0 < SYSCLK < 24 MHz
#define FLASH_LTC1              ((u8) 0x01)             //One wait state, if 24 MHz < SYSCLK < 48 MHz
#define OB_RDP_ON               *(u16*)(OB_BASE + 0x00) = 0xAA55;       //Set Protect
#define OB_RDP_OFF              *(u16*)(OB_BASE + 0x00) = 0x55AA;       //Reset Protect
#define OB_USER(DT)             *(u16*)(OB_BASE + 0x02) = DT|(~DT << 8) //User option byte
#define OB_DATA0(DT)            *(u16*)(OB_BASE + 0x04) = DT|(~DT << 8) //User data byte 0 value
#define OB_DATA1(DT)            *(u16*)(OB_BASE + 0x06) = DT|(~DT << 8) //User data byte 1 value
#define OB_WRP0(DT)             *(u16*)(OB_BASE + 0x08) = DT|(~DT << 8) //Flash memory write protection option byte 0 value
#define OB_WRP1(DT)             *(u16*)(OB_BASE + 0x0A) = DT|(~DT << 8) //Flash memory write protection option byte 1 value
#define OB_WRP2(DT)             *(u16*)(OB_BASE + 0x0C) = DT|(~DT << 8) //Flash memory write protection option byte 2 value
#define OB_WRP3(DT)             *(u16*)(OB_BASE + 0x0E) = DT|(~DT << 8) //Flash memory write protection option byte 3 value
#define OB_WDG_HDW              ((u8) 0xFE)                             //0 - Hardware watchdog (1 - Software watchdog)
#define OB_RST_STOP             ((u8) 0xFD)                             //0 - Reset generated when entering Stop mode (1 - No reset generated)
#define OB_RST_STDBY            ((u8) 0xFB)                             //0 - Reset generated when entering Standby mode (1 - No reset generated)
#define OB_BOOT0                ((u8) 0xF7)                             //When BOOT_SEL is cleared, BOOT0 bit defines BOOT0 signal value used to select the device boot mode
#define OB_BOOT1                ((u8) 0xEF)                             //When BOOT_SEL is cleared, BOOT0 bit defines BOOT0 signal value used to select the device boot mode
#define OB_VDDA_MNTR            ((u8) 0xDF)                             //0 - VDDA power supply supervisor disabled (1 - enabled)
#define OB_RAM_PC               ((u8) 0xBF)                             //0 - RAM parity check enabled (1 - disabled)
#define OB_BOOT_SEL             ((u8) 0x7F)                             //0 - BOOT0 signal is defined by nBOOT0 option bit (1 - BOOT0 signal is defined by BOOT0 pin value (legacy mode))
/******************************************** I2C *************************************************/
typedef struct
{
  struct
  {
    u1 PE         : 1;  //Peripheral enable
    u1 TXIE       : 1;  //TX Interrupt enable
    u1 RXIE       : 1;  //RX Interrupt enable
    u1 ADDRIE     : 1;  //Address match Interrupt enable (slave only)
    u1 NACKIE     : 1;  //Not acknowledge received Interrupt enable
    u1 STOPIE     : 1;  //STOP detection Interrupt enable
    u1 TCIE       : 1;  //Transfer Complete interrupt enable
    u1 ERRIE      : 1;  //Error interrupts enable
    u1 DNF        : 4;  //Digital noise filter
    u1 ANFOFF     : 1;  //Analog noise filter OFF
    u1 RSVD0      : 1;  //Reserved
    u1 TXDMAEN    : 1;  //DMA transmission requests enable
    u1 RXDMAEN    : 1;  //DMA reception requests enable
    u1 SBC        : 1;  //Slave byte control
    u1 NOSTRETCH  : 1;  //Clock stretching disable
    u1 WUPEN      : 1;  //Wakeup from Stop mode enable
    u1 GCEN       : 1;  //General call enable
    u1 SMBHEN     : 1;  //SMBus Host address enable
    u1 SMBDEN     : 1;  //SMBus Device Default address enable
    u1 ALERTEN    : 1;  //SMBus alert enable
    u1 PECEN      : 1;  //PEC enable
    u1 RSVD1      : 8;  //Reserved
  }volatile CR1;              //Control register 1 (I2C_CR1)
  struct
  {
    u1 SADD       : 10; //Slave address (master mode)
    u1 RDWRN      : 1;  //Transfer direction (master mode)
    u1 ADD10      : 1;  //10-bit addressing mode (master mode)
    u1 HEAD10R    : 1;  //10-bit address header only read direction (master receiver mode)
    u1 START      : 1;  //Start generation
    u1 STOP       : 1;  //Stop generation (master mode)
    u1 NACK       : 1;  //NACK generation (slave mode)
    u1 NBYTES     : 8;  //Number of bytes
    u1 RELOAD     : 1;  //NBYTES reload mode
    u1 AUTOEND    : 1;  //Automatic end mode (master mode)
    u1 PECBYTE    : 1;  //Packet error checking byte
    u1 RSVD0      : 5;  //Reserved
  }volatile CR2;              //Control register 2 (I2C_CR2)
  struct
  {
    u1 ADD        : 10; //Interface address (slave mode)
    u1 OA1MODE    : 1;  //Own Address 1 10-bit mode
    u1 RSVD0      : 4;  //Reserved
    u1 OA1EN      : 1;  //Own Address 1 enable
    u1 RSVD1      : 16; //Reserved
  }volatile OAR1;             //Own address register 1 (I2C_OAR1)
  struct
  {
    u1 RSVD0      : 1;  //Reserved
    u1 OA2        : 7;  //Interface address
    u1 OA2MSK     : 3;  //Own Address 2 masks
    u1 RSVD1      : 4;  //Reserved
    u1 OA2EN      : 1;  //Own Address 2 enable
    u1 RSVD2      : 16; //Reserved
  }volatile OAR2;             //Own address register 2 (I2C_OAR2)
  struct
  {
    u1 SCLL       : 8;  //SCL low period (master mode)
    u1 SCLH       : 8;  //SCL high period (master mode)
    u1 SDADEL     : 4;  //Data hold time
    u1 SCLDEL     : 4;  //Data setup time
    u1 RSVD0      : 4;  //Reserved
    u1 PRESC      : 4;  //Timing prescaler
  }volatile TIMINGR;          //Timing register (I2C_TIMINGR)
  struct
  {
    u1 TIMEOUTA   : 12; //Bus Timeout A
    u1 TIDLE      : 1;  //Idle clock timeout detection
    u1 RSVD0      : 2;  //Reserved
    u1 TIMOUTEN   : 1;  //Clock timeout enable
    u1 TIMEOUTB   : 12; //Bus Timeout B
    u1 RSVD1      : 3;  //Reserved
    u1 TEXTEN     : 1;  //Extended clock timeout enable
  }volatile TIMEOUTR;         //Timeout register (I2C_TIMEOUTR)
  struct
  {
    u1 TXE        : 1;  //Transmit data register empty (transmitters)
    u1 TXIS       : 1;  //Transmit interrupt status (transmitters)
    u1 RXNE       : 1;  //Receive data register not empty (receivers)
    u1 ADDR       : 1;  //Address matched (slave mode)
    u1 NACKF      : 1;  //Not Acknowledge received flag
    u1 STOPF      : 1;  //Stop detection flag
    u1 TC         : 1;  //Transfer Complete (master mode)
    u1 TCR        : 1;  //Transfer Complete Reload
    u1 BERR       : 1;  //Bus error
    u1 ARLO       : 1;  //Arbitration lost
    u1 OVR        : 1;  //Overrun/Underrun (slave mode)
    u1 PECERR     : 1;  //PEC Error in reception
    u1 TIMEOUT    : 1;  //Timeout or tLOW detection flag
    u1 ALERT      : 1;  //SMBus alert
    u1 RSVD0      : 1;  //Reserved
    u1 BUSY       : 1;  //Bus busy
    u1 DIR        : 1;  //Transfer direction (Slave mode)
    u1 ADDCODE    : 7;  //Address match code (Slave mode)
    u1 RSVD1      : 8;  //Reserved
  }volatile ISR;              //Interrupt and status register (I2C_ISR)
  struct
  {
    u1 RSVD0      : 3;  //Reserved
    u1 ADDRCF     : 1;  //Address Matched flag clear
    u1 NACKCF     : 1;  //Not Acknowledge flag clear
    u1 STOPCF     : 1;  //Stop detection flag clear
    u1 RSVD1      : 2;  //Reserved
    u1 BERRCF     : 1;  //Bus error flag clear
    u1 ARLOCF     : 1;  //Arbitration Lost flag clear
    u1 OVRCF      : 1;  //Overrun/Underrun flag clear
    u1 PECCF      : 1;  //PEC Error flag clear
    u1 TIMOUTCF   : 1;  //Timeout detection flag clear
    u1 ALERTCF    : 1;  //Alert flag clear
    u1 RSVD2      : 18; //Reserved
  }volatile ICR;              //Interrupt clear register (I2C_ICR)
  struct
  {
    u1 PEC        : 8;  //Packet error checking register
    u1 RSVD0      : 24; //Reserved
  }volatile PECR;             //PEC register (I2C_PECR)
  struct
  {
    u1 RXDATA     : 8;  //8-bit receive data
    u1 RSVD0      : 24; //Reserved
  }volatile RXDR;             //Receive data register (I2C_RXDR)
  struct
  {
    u1 TXDATA     : 8;  //8-bit transmit data
    u1 RSVD0      : 24; //Reserved
  }volatile TXDR;             //Transmit data register (I2C_TXDR)
}I2C_Type;
#define I2C1 ((I2C_Type*) I2C1_BASE)
#define I2C2 ((I2C_Type*) I2C2_BASE)
#define I2C_EN(PORT)            PORT->CR1.PE            //Peripheral enable
#define I2C_TX_IRQ(PORT)        PORT->CR1.TXIE          //TX Interrupt enable
#define I2C_RX_IRQ(PORT)        PORT->CR1.RXIE          //RX Interrupt enable
#define I2C_ADD_IRQ(PORT)       PORT->CR1.ADDRIE        //Address match Interrupt enable (slave only)
#define I2C_NAC_IRQ(PORT)       PORT->CR1.NACKIE        //Not acknowledge received Interrupt enable
#define I2C_STOP_IRQ(PORT)      PORT->CR1.STOPIE        //STOP detection Interrupt enable
#define I2C_TC_IRQ(PORT)        PORT->CR1.TCIE          //Transfer Complete interrupt enable
#define I2C_ERR_IRQ(PORT)       PORT->CR1.ERRIE         //Error interrupts enable
#define I2C_DNF(PORT)           PORT->CR1.DNF           //Digital noise filter
#define I2C_ANFOFF(PORT)        PORT->CR1.ANFOFF        //Analog noise filter OFF
#define I2C_DMA_TX_EN(PORT)     PORT->CR1.TXDMAEN       //DMA transmission requests enable
#define I2C_DMA_RX_EN(PORT)     PORT->CR1.RXDMAEN       //DMA reception requests enable
#define I2C_SBC(PORT)           PORT->CR1.SBC           //Slave byte control
#define I2C_NOSTRETCH(PORT)     PORT->CR1.NOSTRETCH     //Clock stretching disable
#define I2C_WUP_EN(PORT)        PORT->CR1.WUPEN         //Wakeup from Stop mode enable
#define I2C_GC_EN(PORT)         PORT->CR1.GCEN          //General call enable
#define I2C_SMBH_EN(PORT)       PORT->CR1.SMBHEN        //SMBus Host address enable
#define I2C_SMBD_EN(PORT)       PORT->CR1.SMBDEN        //SMBus Device Default address enable
#define I2C_ALERT_EN(PORT)      PORT->CR1.ALERTEN       //SMBus alert enable
#define I2C_PEC_EN(PORT)        PORT->CR1.PECEN         //PEC enable
#define I2C_SADD(PORT)          PORT->CR2.SADD          //Slave address (master mode)
#define I2C_RDWRN(PORT)         PORT->CR2.RDWRN         //Transfer direction (master mode)
#define I2C_ADD10(PORT)         PORT->CR2.ADD10         //10-bit addressing mode (master mode)
#define I2C_HEAD10R(PORT)       PORT->CR2.HEAD10R       //10-bit address header only read direction (master receiver mode)
#define I2C_START(PORT)         PORT->CR2.START         //Start generation
#define I2C_STOP(PORT)          PORT->CR2.STOP          //Stop generation (master mode)
#define I2C_NACK(PORT)          PORT->CR2.NACK          //NACK generation (slave mode)
#define I2C_NBYTES(PORT)        PORT->CR2.NBYTES        //Number of bytes
#define I2C_RELOAD(PORT)        PORT->CR2.RELOAD        //NBYTES reload mode
#define I2C_AUTOEND(PORT)       PORT->CR2.AUTOEND       //Automatic end mode (master mode)
#define I2C_PECBYTE(PORT)       PORT->CR2.PECBYTE       //Packet error checking byte
#define I2C_ADD(PORT)           PORT->OAR1.ADD          //Interface address (slave mode)
#define I2C_OA1MODE(PORT)       PORT->OAR1.OA1MODE      //Own Address 1 10-bit mode
#define I2C_OA1EN(PORT)         PORT->OAR1.OA1EN        //Own Address 1 enable
#define I2C_OA2(PORT)           PORT->OAR2.OA2          //Interface address
#define I2C_OA2MSK(PORT)        PORT->OAR2.OA2MSK       //Own Address 2 masks
#define I2C_OA2EN(PORT)         PORT->OAR2.OA2EN        //Own Address 2 enable
#define I2C_SCLL(PORT)          PORT->TIMINGR.SCLL      //SCL low period (master mode)
#define I2C_SCLH(PORT)          PORT->TIMINGR.SCLH      //SCL high period (master mode)
#define I2C_SDADEL(PORT)        PORT->TIMINGR.SDADEL    //Data hold time
#define I2C_SCLDEL(PORT)        PORT->TIMINGR.SCLDEL    //Data setup time
#define I2C_PRESC(PORT)         PORT->TIMINGR.PRESC     //Timing prescaler
#define I2C_TIMEOUTA(PORT)      PORT->TIMEOUTR.TIMEOUTA //Bus Timeout A
#define I2C_TIDLE(PORT)         PORT->TIMEOUTR.TIDLE    //Idle clock timeout detection
#define I2C_TIMOUT_EN(PORT)     PORT->TIMEOUTR.TIMOUTEN //Clock timeout enable
#define I2C_TIMEOUTB(PORT)      PORT->TIMEOUTR.TIMEOUTB //Bus Timeout B
#define I2C_TEXT_EN(PORT)       PORT->TIMEOUTR.TEXTEN   //Extended clock timeout enable
#define I2C_TXE_FLG(PORT)       PORT->ISR.TXE           //Transmit data register empty (transmitters)
#define I2C_TXIS_FLG(PORT)      PORT->ISR.TXIS          //Transmit interrupt status (transmitters)
#define I2C_RXNE_FLG(PORT)      PORT->ISR.RXNE          //Receive data register not empty (receivers)
#define I2C_ADDR_FLG(PORT)      PORT->ISR.ADDR          //Address matched (slave mode)
#define I2C_NACK_FLG(PORT)      PORT->ISR.NACKF         //Not Acknowledge received flag
#define I2C_STOP_FLG(PORT)      PORT->ISR.STOPF         //Stop detection flag
#define I2C_TC_FLG(PORT)        PORT->ISR.TC            //Transfer Complete (master mode)
#define I2C_TCR_FLG(PORT)       PORT->ISR.TCR           //Transfer Complete Reload
#define I2C_BERR(PORT)          PORT->ISR.BERR          //Bus error
#define I2C_ARLO(PORT)          PORT->ISR.ARLO          //Arbitration lost
#define I2C_OVR(PORT)           PORT->ISR.OVR           //Overrun/Underrun (slave mode)
#define I2C_PECERR(PORT)        PORT->ISR.PECERR        //PEC Error in reception
#define I2C_TIMEOUT(PORT)       PORT->ISR.TIMEOUT       //Timeout or tLOW detection flag
#define I2C_ALERT(PORT)         PORT->ISR.ALERT         //SMBus alert
#define I2C_BUSY(PORT)          PORT->ISR.BUSY          //Bus busy
#define I2C_DIR(PORT)           PORT->ISR.DIR           //Transfer direction (Slave mode)
#define I2C_ADDCODE(PORT)       PORT->ISR.ADDCODE       //Address match code (Slave mode)
#define I2C_ADDR_CLR(PORT)      PORT->ICR.ADDRCF = ON   //Address Matched flag clear
#define I2C_NACK_CLR(PORT)      PORT->ICR.NACKCF = ON   //Not Acknowledge flag clear
#define I2C_STOP_CLR(PORT)      PORT->ICR.STOPCF = ON   //Stop detection flag clear
#define I2C_BERR_CLR(PORT)      PORT->ICR.BERRCF = ON   //Bus error flag clear
#define I2C_ARLO_CLR(PORT)      PORT->ICR.ARLOCF = ON   //Arbitration Lost flag clear
#define I2C_OVR_CLR(PORT)       PORT->ICR.OVRCF = ON    //Overrun/Underrun flag clear
#define I2C_PEC_CLR(PORT)       PORT->ICR.PECCF = ON    //PEC Error flag clear
#define I2C_TIMOUT_CLR(PORT)    PORT->ICR.TIMOUTCF = ON //Timeout detection flag clear
#define I2C_ALERT_CLR(PORT)     PORT->ICR.ALERTCF = ON  //Alert flag clear
#define I2C_PEC(PORT)           PORT->ICR.PEC           //Packet error checking register
#define I2C_RXDATA(PORT)        PORT->RXDR.RXDATA       //8-bit receive data
#define I2C_TXDATA(PORT)        PORT->TXDR.TXDATA       //8-bit transmit data
/******************************************** IWDG ************************************************/
typedef struct
{
  struct
  {
    u1 KEY        : 16; //Key value (write only, read 0000h)
    u1 RSVD0      : 16; //Reserved
  }volatile KR;         //Key register (IWDG_KR)
  struct
  {
    u1 PR         : 3;  //Prescaler divider
    u1 RSVD0      : 29; //Reserved
  }volatile PR;         //Prescaler register (IWDG_PR)
  struct
  {
    u1 RL         : 12; //Watchdog counter reload value
    u1 RSVD0      : 20; //Reserved
  }volatile RLR;        //Reload register (IWDG_RLR)
  struct
  {
    u1 PVU        : 1;  //Watchdog prescaler value update
    u1 RVU        : 1;  //Watchdog counter reload value update
    u1 WVU        : 1;  //Watchdog counter window value update
    u1 RSVD0      : 29; //Reserved
  }volatile SR;         //Status register (IWDG_SR)
  struct
  {
    u1 WIN        : 12; //Watchdog counter window value
    u1 RSVD0      : 20; //Reserved
  }volatile WINR;       //Window register (IWDG_WINR)
}IWDG_Type;
#define IWDG ((IWDG_Type*) IWDG_BASE)
#define IWDG_KEY                IWDG->KR.KEY            //Key value (write only, read 0000h)
#define IWDG_DIV                IWDG->PR.PR             //Prescaler divider
#define IWDG_RL                 IWDG->RLR.RL            //Watchdog counter reload value
#define IWDG_PVU                IWDG->SR.PVU            //Watchdog prescaler value update
#define IWDG_RVU                IWDG->SR.RVU            //Watchdog counter reload value update
#define IWDG_WVU                IWDG->SR.WVU            //Watchdog counter window value update
#define IWDG_WIN                IWDG->WINR.WIN          //Watchdog counter window value
#define IWDG_DIV_004            ((u8) 0x00)             //Divider 4
#define IWDG_DIV_008            ((u8) 0x01)             //Divider 8
#define IWDG_DIV_016            ((u8) 0x02)             //Divider 16
#define IWDG_DIV_032            ((u8) 0x03)             //Divider 32
#define IWDG_DIV_064            ((u8) 0x04)             //Divider 64
#define IWDG_DIV_128            ((u8) 0x05)             //Divider 128
#define IWDG_DIV_256            ((u8) 0x06)             //Divider 256
#define IWDG_ON                 IWDG_KEY = (u16) 0xCCCC //Enable the IWDG
#define IWDG_OFF                IWDG_KEY = (u16) 0x0FFF //Disable the IWDG
#define IWDG_REG                IWDG_KEY = (u16) 0x5555 //Enable register access
#define IWDG_RST                IWDG_KEY = (u16) 0xAAAA //Watchdog reset
/******************************************** PWR *************************************************/
typedef struct
{
  struct
  {
    u1 LPDS       : 1;  //Low-power deepsleep.
    u1 PDDS       : 1;  //Power down deepsleep.
    u1 CWUF       : 1;  //Clear wakeup flag.
    u1 CSBF       : 1;  //Clear standby flag.
    u1 PVDE       : 1;  //Power voltage detector enable.
    u1 PLS        : 3;  //PVD level selection.
    u1 DBP        : 1;  //Disable backup domain write protection.
    u1 RSVD0      : 23; //Reserved 
  }volatile CR;               //Power control register (PWR_CR)
  struct
  {
    u1 WUF        : 1;  //Wakeup flag
    u1 SBF        : 1;  //Standby flag
    u1 PVDO       : 1;  //PVD output
    u1 VREFINTRDY : 1;  //VREFINT reference voltage ready
    u1 RSVD0      : 4;  //Reserved
    u1 EWUP       : 8;  //Enable WKUP pin
    u1 RSVD1      : 16; //Reserved
  }volatile CSR;              //Power control/status register (PWR_CSR)
}PWR_Type;
#define PWR ((PWR_Type*) PWR_BASE)
#define PWR_LPDS                PWR->CR.LPDS            //Low-power deepsleep.
#define PWR_PDDS                PWR->CR.PDDS            //Power down deepsleep.
#define PWR_CWUF                PWR->CR.CWUF            //Clear wakeup flag.
#define PWR_CSBF                PWR->CR.CSBF            //Clear standby flag.
#define PWR_PVDE                PWR->CR.PVDE            //Power voltage detector enable.
#define PWR_PLS                 PWR->CR.PLS             //PVD level selection.
#define PWR_DBP                 PWR->CR.DBP             //Disable backup domain write protection.
#define PWR_WUF                 PWR->CSR.WUF            //Wakeup flag
#define PWR_SBF                 PWR->CSR.SBF            //Standby flag
#define PWR_PVDO                PWR->CSR.PVDO           //PVD output
#define PWR_REF_FLG             PWR->CSR.VREFINTRDY     //VREFINT reference voltage ready
#define PWR_EWUP                PWR->CSR.EWUP           //Enable WKUP pin
#define PWR_PVD_22              ((u8) 0x00)             //PVD level 2.2V
#define PWR_PVD_23              ((u8) 0x01)             //PVD level 2.3V
#define PWR_PVD_24              ((u8) 0x02)             //PVD level 2.4V
#define PWR_PVD_25              ((u8) 0x03)             //PVD level 2.5V
#define PWR_PVD_26              ((u8) 0x04)             //PVD level 2.6V
#define PWR_PVD_27              ((u8) 0x05)             //PVD level 2.7V
#define PWR_PVD_28              ((u8) 0x06)             //PVD level 2.8V
#define PWR_PVD_29              ((u8) 0x07)             //PVD level 2.9V
/******************************************** RTC *************************************************/
typedef struct
{
  struct
  {
    u1 SU         : 4;  //Second units in BCD format
    u1 ST         : 3;  //Second tens in BCD format
    u1 RSVD0      : 1;  //Reserved
    u1 MNU        : 4;  //Minute units in BCD format
    u1 MNT        : 3;  //Minute tens in BCD format
    u1 RSVD1      : 1;  //Reserved
    u1 HU         : 4;  //Hour units in BCD format
    u1 HT         : 2;  //Hour tens in BCD format
    u1 PM         : 1;  //AM/PM notation
    u1 RSVD2      : 9;  //Reserved
  }volatile TR;         //RTC time register (RTC_TR)
  struct
  {
    u1 DU         : 4;  //Date units in BCD format
    u1 DT         : 2;  //Date tens in BCD format
    u1 RSVD0      : 2;  //Reserved
    u1 MU         : 4;  //Month units in BCD format
    u1 MT         : 1;  //Month tens in BCD format
    u1 WDU        : 3;  //Week day units
    u1 YU         : 4;  //Year units in BCD format
    u1 YT         : 4;  //Year tens in BCD format
    u1 RSVD2      : 8;  //Reserved
  }volatile DR;         //RTC date register (RTC_DR)
  struct
  {
    u1 WUCKSEL    : 3;  //Wakeup clock selection
    u1 TSEDGE     : 1;  //Time-stamp event active edge
    u1 REFCKON    : 1;  //RTC_REFIN reference clock detection enable (50 or 60 Hz)
    u1 BYPSHAD    : 1;  //Bypass the shadow registers
    u1 FMT        : 1;  //Hour format
    u1 RSVD0      : 1;  //Reserved
    u1 ALRAE      : 1;  //Alarm A enable
    u1 RSVD1      : 1;  //Reserved
    u1 WUTE       : 1;  //Wakeup timer enable
    u1 TSE        : 1;  //timestamp enable
    u1 ALRAIE     : 1;  //Alarm A interrupt enable
    u1 RSVD2      : 1;  //Reserved
    u1 WUTIE      : 1;  //Wakeup timer interrupt enable
    u1 TSIE       : 1;  //Time-stamp interrupt enable
    u1 ADD1H      : 1;  //Add 1 hour (summer time change)
    u1 SUB1H      : 1;  //Subtract 1 hour (winter time change)
    u1 BKP        : 1;  //Backup
    u1 COSEL      : 1;  //Calibration output selection
    u1 POL        : 1;  //Output polarity
    u1 OSEL       : 2;  //Output selection
    u1 COE        : 1;  //Calibration output enable
    u1 RSVD3      : 8;  //Reserved
  }volatile CR;         //RTC control register high (RTC_CR)
  struct
  {
    u1 ALRAWF     : 1;  //Alarm A write flag
    u1 RSVD0      : 1;  //Reserved
    u1 WUTWF      : 1;  //Wakeup timer write flag
    u1 SHPF       : 1;  //Shift operation pending
    u1 INITS      : 1;  //Initialization status flag
    u1 RSF        : 1;  //Registers synchronization flag
    u1 INITF      : 1;  //Initialization flag
    u1 INIT       : 1;  //Initialization mode
    u1 ALRAF      : 1;  //Alarm A flag
    u1 RSVD1      : 1;  //Reserved
    u1 WUTF       : 1;  //Wakeup timer flag
    u1 TSF        : 1;  //Time-stamp flag
    u1 TSOVF      : 1;  //Time-stamp overflow flag
    u1 TAMP1F     : 1;  //RTC_TAMP1 detection flag
    u1 TAMP2F     : 1;  //RTC_TAMP2 detection flag
    u1 TAMP3F     : 1;  //RTC_TAMP3 detection flag
    u1 RECALPF    : 1;  //Recalibration pending Flag
    u1 RSVD2      : 15; //Reserved
  }volatile ISR;        //RTC initialization and status register (RTC_ISR)
  struct
  {
    u1 PREDIV_S   : 15; //Synchronous prescaler factor
    u1 RSVD0      : 1;  //Reserved
    u1 PREDIV_A   : 7;  //Asynchronous prescaler factor
    u1 RSVD1      : 9;  //Reserved
  }volatile PRER;       //RTC prescaler register (RTC_PRER)
  struct
  {
    u1 WUT        : 16; //Wakeup auto-reload value bits
    u1 RSVD0      : 16; //Reserved
  }volatile WUTR;       //RTC wakeup timer register (RTC_WUTR)
  struct
  {
    u1 SU         : 4;  //Second units in BCD format
    u1 ST         : 3;  //Second tens in BCD format
    u1 MSK1       : 1;  //Alarm A seconds mask
    u1 MNU        : 4;  //Minute units in BCD format
    u1 MNT        : 3;  //Minute tens in BCD format
    u1 MSK2       : 1;  //Alarm A minutes mask
    u1 HU         : 4;  //Hour units in BCD format
    u1 HT         : 2;  //Hour tens in BCD format
    u1 PM         : 1;  //AM/PM notation
    u1 MSK3       : 1;  //Alarm A hours mask
    u1 DU         : 4;  //Date units or day in BCD format
    u1 DT         : 2;  //Date tens in BCD format
    u1 WDSEL      : 1;  //Week day selection
    u1 MSK4       : 1;  //Alarm A date mask
  }volatile ALRMAR;     //RTC alarm A register (RTC_ALRMAR)
  volatile u32 RSVD0[2];
  struct
  {
    u1 KEY        : 8;  //Write protection key
    u1 RSVD0      : 24; //Reserved
  }volatile WPR;        //RTC write protection register (RTC_WPR)
  struct
  {
    u1 SS         : 16; //Sub second value
    u1 RSVD0      : 16; //Reserved
  }volatile SSR;        //RTC sub second register (RTC_SSR)
  struct
  {
    u1 SUBFS      : 15; //Subtract a fraction of a second
    u1 RSVD0      : 16; //Reserved
    u1 ADD1S      : 1;  //Add one second
  }volatile SHIFTR;     //RTC shift control register (RTC_SHIFTR)
  struct
  {
    u1 SU         : 4;  //Second units in BCD format
    u1 ST         : 3;  //Second tens in BCD format
    u1 RSVD0      : 1;  //Reserved
    u1 MNU        : 4;  //Minute units in BCD format
    u1 MNT        : 3;  //Minute tens in BCD format
    u1 RSVD1      : 1;  //Reserved
    u1 HU         : 4;  //Hour units in BCD format
    u1 HT         : 2;  //Hour tens in BCD format
    u1 PM         : 1;  //AM/PM notation
    u1 RSVD2      : 9;  //Reserved
  }volatile TSTR;       //RTC timestamp time register (RTC_TSTR)
  struct
  {
    u1 DU         : 4;  //Date units in BCD format
    u1 DT         : 2;  //Date tens in BCD format
    u1 RSVD0      : 2;  //Reserved
    u1 MU         : 4;  //Month units in BCD format
    u1 MT         : 1;  //Month tens in BCD format
    u1 WDU        : 3;  //Week day units
    u1 RSVD1      : 16; //Reserved
  }volatile TSDR;       //RTC timestamp date register (RTC_TSDR)
  struct
  {
    u1 SS         : 16; //Sub second value
    u1 RSVD0      : 16; //Reserved
  }volatile TSSSR;      //RTC time-stamp sub second register (RTC_TSSSR)
  struct
  {
    u1 CALM       : 9;  //Calibration minus
    u1 RSVD0      : 4;  //Reserved
    u1 CALW16     : 1;  //Use a 16-second calibration cycle period
    u1 CALW8      : 1;  //Use an 8-second calibration cycle period
    u1 CALP       : 1;  //Increase frequency of RTC by 488.5 ppm
    u1 RSVD1      : 16; //Reserved
  }volatile CALR;       //RTC calibration register (RTC_CALR)
  struct
  {
    u1 TAMP1E     : 1;  //RTC_TAMP1 input detection enable
    u1 TAMP1TRG   : 1;  //Active level for RTC_TAMP1 input
    u1 TAMPIE     : 2;  //Tamper interrupt enable
    u1 TAMP2E     : 1;  //RTC_TAMP2 input detection enable
    u1 TAMP2TRG   : 1;  //Active level for RTC_TAMP2 input
    u1 TAMP3E     : 1;  //RTC_TAMP3 detection enable
    u1 TAMP3TRG   : 1;  //Active level for RTC_TAMP3 input
    u1 TAMPTS     : 1;  //Activate timestamp on tamper detection event
    u1 TAMPFREQ   : 3;  //Tamper sampling frequency
    u1 TAMPFLT    : 2;  //RTC_TAMPx filter count
    u1 TAMPPRCH   : 1;  //RTC_TAMPx precharge duration
    u1 TAMPPUDIS  : 1;  //RTC_TAMPx pull-up disable
    u1 RSVD0      : 2;  //Reserved
    u1 PC13VALUE  : 1;  //RTC_ALARM output type/PC13 value
    u1 PC13MODE   : 1;  //PC13 mode
    u1 PC14VALUE  : 1;  //PC14 value
    u1 PC14MODE   : 1;  //PC14 mode
    u1 PC15VALUE  : 1;  //PC15 value
    u1 PC15MODE   : 1;  //PC15 mode
    u1 RSVD1      : 8;  //Reserved
  }volatile TAFCR;      //RTC tamper and alternate function configuration register (RTC_TAFCR)
  struct
  {
    u1 SS         : 15; //Sub seconds value
    u1 RSVD0      : 9;  //Reserved
    u1 MASKSS     : 4;  //Mask the most-significant bits starting at this bit
    u1 RSVD1      : 4;  //Reserved
  }volatile ALRMASSR;   //RTC alarm A sub second register (RTC_ALRMASSR)
  volatile u32 RSVD1[2];
  volatile u32 BKPR[5]; //RTC backup registers (RTC_BKPxR)
}RTC_Type;
#define RTC ((RTC_Type*) RTC_BASE)
#define RTC_SU                  RTC->TR.SU                      //Second units in BCD format
#define RTC_ST                  RTC->TR.ST                      //Second tens in BCD format
#define RTC_MNU                 RTC->TR.MNU                     //Minute units in BCD format
#define RTC_MNT                 RTC->TR.MNT                     //Minute tens in BCD format
#define RTC_HU                  RTC->TR.HU                      //Hour units in BCD format
#define RTC_HT                  RTC->TR.HT                      //Hour tens in BCD format
#define RTC_PM                  RTC->TR.PM                      //AM/PM notation
#define RTC_DU                  RTC->DR.DU                      //Date units in BCD format
#define RTC_DT                  RTC->DR.DT                      //Date tens in BCD format
#define RTC_MU                  RTC->DR.MU                      //Month units in BCD format
#define RTC_MT                  RTC->DR.MT                      //Month tens in BCD format
#define RTC_WDU                 RTC->DR.WDU                     //Week day units
#define RTC_YU                  RTC->DR.YU                      //Year units in BCD format
#define RTC_YT                  RTC->DR.YT                      //Year tens in BCD format
#define RTC_WUCKSEL             RTC->CR.WUCKSEL                 //Wakeup clock selection
#define RTC_TSEDGE              RTC->CR.TSEDGE                  //Time-stamp event active edge
#define RTC_REFCKON             RTC->CR.REFCKON                 //RTC_REFIN reference clock detection enable (50 or 60 Hz)
#define RTC_BYPASS              RTC->CR.BYPSHAD                 //Bypass the shadow registers
#define RTC_FMT                 RTC->CR.FMT                     //Hour format
#define RTC_ALRAE               RTC->CR.ALRAE                   //Alarm A enable
#define RTC_WUTE                RTC->CR.WUTE                    //Wakeup timer enable
#define RTC_TSE                 RTC->CR.TSE                     //timestamp enable
#define RTC_ALRAIE              RTC->CR.ALRAIE                  //Alarm A interrupt enable
#define RTC_WUTIE               RTC->CR.WUTIE                   //Wakeup timer interrupt enable
#define RTC_TSIE                RTC->CR.TSIE                    //Time-stamp interrupt enable
#define RTC_ADD1H               RTC->CR.ADD1H                   //Add 1 hour (summer time change)
#define RTC_SUB1H               RTC->CR.SUB1H                   //Subtract 1 hour (winter time change)
#define RTC_BKP                 RTC->CR.BKP                     //Backup
#define RTC_COSEL               RTC->CR.COSEL                   //Calibration output selection
#define RTC_POL                 RTC->CR.POL                     //Output polarity
#define RTC_OSEL                RTC->CR.OSEL                    //Output selection
#define RTC_COE                 RTC->CR.COE                     //Calibration output enable
#define RTC_ALRAWF              RTC->ISR.ALRAWF                 //Alarm A write flag
#define RTC_WUTWF               RTC->ISR.WUTWF                  //Wakeup timer write flag
#define RTC_SHPF                RTC->ISR.SHPF                   //Shift operation pending
#define RTC_INITS               RTC->ISR.INITS                  //Initialization status flag
#define RTC_RSF                 RTC->ISR.RSF                    //Registers synchronization flag
#define RTC_INITF               RTC->ISR.INITF                  //Initialization flag
#define RTC_INIT                RTC->ISR.INIT                   //Initialization mode
#define RTC_ALRAF               RTC->ISR.ALRAF                  //Alarm A flag
#define RTC_WUTF                RTC->ISR.WUTF                   //Wakeup timer flag
#define RTC_TSF                 RTC->ISR.TSF                    //Time-stamp flag
#define RTC_TSOVF               RTC->ISR.TSOVF                  //Time-stamp overflow flag
#define RTC_TAMP1F              RTC->ISR.TAMP1F                 //RTC_TAMP1 detection flag
#define RTC_TAMP2F              RTC->ISR.TAMP2F                 //RTC_TAMP2 detection flag
#define RTC_TAMP3F              RTC->ISR.TAMP3F                 //RTC_TAMP3 detection flag
#define RTC_RECALPF             RTC->ISR.RECALPF                //Recalibration pending Flag
#define RTC_PREDIV_S            RTC->PRER.PREDIV_S              //Synchronous prescaler factor
#define RTC_PREDIV_A            RTC->PRER.PREDIV_A              //Asynchronous prescaler factor
#define RTC_WUT                 RTC->WUTR.WUT                   //Wakeup auto-reload value bits
#define RTC_SU_ALRM             RTC->ALRMAR.SU                  //Second units in BCD format
#define RTC_ST_ALRM             RTC->ALRMAR.ST                  //Second tens in BCD format
#define RTC_MSK1                RTC->ALRMAR.MSK1                //Alarm A seconds mask
#define RTC_MNU_ALRM            RTC->ALRMAR.MNU                 //Minute units in BCD format
#define RTC_MNT_ALRM            RTC->ALRMAR.MNT                 //Minute tens in BCD format
#define RTC_MSK2                RTC->ALRMAR.MSK2                //Alarm A minutes mask
#define RTC_HU_ALRM             RTC->ALRMAR.HU                  //Hour units in BCD format
#define RTC_HT_ALRM             RTC->ALRMAR.HT                  //Hour tens in BCD format
#define RTC_PM_ALRM             RTC->ALRMAR.PM                  //AM/PM notation
#define RTC_MSK3                RTC->ALRMAR.MSK3                //Alarm A hours mask
#define RTC_DU_ALRM             RTC->ALRMAR.DU                  //Date units or day in BCD format
#define RTC_DT_ALRM             RTC->ALRMAR.DT                  //Date tens in BCD format
#define RTC_WDSEL               RTC->ALRMAR.WDSEL               //Week day selection
#define RTC_MSK4                RTC->ALRMAR.MSK4                //Alarm A date mask
#define RTC_KEY                 RTC->WPR.KEY                    //Write protection key
#define RTC_SS                  RTC->SSR.SS                     //Sub second value
#define RTC_SUBFS               RTC->SHIFTR.SUBFS               //Subtract a fraction of a second
#define RTC_ADD1S               RTC->SHIFTR.ADD1S               //Add one second
#define RTC_SU_TS               RTC->TSTR.SU                    //Second units in BCD format
#define RTC_ST_TS               RTC->TSTR.ST                    //Second tens in BCD format
#define RTC_MNU_TS              RTC->TSTR.MNU                   //Minute units in BCD format
#define RTC_MNT_TS              RTC->TSTR.MNT                   //Minute tens in BCD format
#define RTC_HU_TS               RTC->TSTR.HU                    //Hour units in BCD format
#define RTC_HT_TS               RTC->TSTR.HT                    //Hour tens in BCD format
#define RTC_PM_TS               RTC->TSTR.PM                    //AM/PM notation
#define RTC_DU_TS               RTC->TSDR.DU                    //Date units in BCD format
#define RTC_DT_TS               RTC->TSDR.DT                    //Date tens in BCD format
#define RTC_MU_TS               RTC->TSDR.MU                    //Month units in BCD format
#define RTC_MT_TS               RTC->TSDR.MT                    //Month tens in BCD format
#define RTC_WDU_TS              RTC->TSDR.WDU                   //Week day units
#define RTC_SS_TS               RTC->TSSSR.SS                   //Sub second value
#define RTC_CALM                RTC->CALR.CALM                  //Calibration minus
#define RTC_CALW16              RTC->CALR.CALW16                //Use a 16-second calibration cycle period
#define RTC_CALW8               RTC->CALR.CALW8                 //Use an 8-second calibration cycle period
#define RTC_CALP                RTC->CALR.CALP                  //Increase frequency of RTC by 488.5 ppm
#define RTC_TAMP1E              RTC->TAFCR.TAMP1E               //RTC_TAMP1 input detection enable
#define RTC_TAMP1TRG            RTC->TAFCR.TAMP1TRG             //Active level for RTC_TAMP1 input
#define RTC_TAMPIE              RTC->TAFCR.TAMPIE               //Tamper interrupt enable
#define RTC_TAMP2E              RTC->TAFCR.TAMP2E               //RTC_TAMP2 input detection enable
#define RTC_TAMP2TRG            RTC->TAFCR.TAMP2TRG             //Active level for RTC_TAMP2 input
#define RTC_TAMP3E              RTC->TAFCR.TAMP3E               //RTC_TAMP3 detection enable
#define RTC_TAMP3TRG            RTC->TAFCR.TAMP3TRG             //Active level for RTC_TAMP3 input
#define RTC_TAMPTS              RTC->TAFCR.TAMPTS               //Activate timestamp on tamper detection event
#define RTC_TAMPFREQ            RTC->TAFCR.TAMPFREQ             //Tamper sampling frequency
#define RTC_TAMPFLT             RTC->TAFCR.TAMPFLT              //RTC_TAMPx filter count
#define RTC_TAMPPRCH            RTC->TAFCR.TAMPPRCH             //RTC_TAMPx precharge duration
#define RTC_TAMPPUDIS           RTC->TAFCR.TAMPPUDIS            //RTC_TAMPx pull-up disable
#define RTC_PC13VALUE           RTC->TAFCR.PC13VALUE            //RTC_ALARM output type/PC13 value
#define RTC_PC13MODE            RTC->TAFCR.PC13MODE             //PC13 mode
#define RTC_PC14VALUE           RTC->TAFCR.PC14VALUE            //PC14 value
#define RTC_PC14MODE            RTC->TAFCR.PC14MODE             //PC14 mode
#define RTC_PC15VALUE           RTC->TAFCR.PC15VALUE            //PC15 value
#define RTC_PC15MODE            RTC->TAFCR.PC15MODE             //PC15 mode
#define RTC_SS_ALRM             RTC->ALRMASSR.SS                //Sub seconds value
#define RTC_MASKSS              RTC->ALRMASSR.MASKSS            //Mask the most-significant bits starting at this bit
#define RTC_BKPR0               RTC->BKPR[0]                    //RTC backup registers (RTC_BKPxR)
#define RTC_BKPR1               RTC->BKPR[1]                    //RTC backup registers (RTC_BKPxR)
#define RTC_BKPR2               RTC->BKPR[2]                    //RTC backup registers (RTC_BKPxR)
#define RTC_BKPR3               RTC->BKPR[3]                    //RTC backup registers (RTC_BKPxR)
#define RTC_BKPR4               RTC->BKPR[4]                    //RTC backup registers (RTC_BKPxR)
#define RTC_UNLOCK              RTC_KEY = 0xCA; RTC_KEY = 0x53  //RTC unlock
#define RTC_LOCK                RTC_KEY = 0xFF                  //RTC lock
/******************************************** SPI *************************************************/
typedef struct
{
  struct
  {
    u1 CPHA       : 1;  //Clock phase
    u1 CPOL       : 1;  //Clock polarity
    u1 MSTR       : 1;  //Master selection
    u1 BR         : 3;  //Baud rate control
    u1 SPE        : 1;  //SPI enable
    u1 LSBFIRST   : 1;  //Frame format
    u1 SSI        : 1;  //Internal slave select
    u1 SSM        : 1;  //Software slave management
    u1 RXONLY     : 1;  //Receive only
    u1 CRCL       : 1;  //CRC length
    u1 CRCNEXT    : 1;  //Transmit CRC next
    u1 CRCEN      : 1;  //Hardware CRC calculation enable
    u1 BIDIOE     : 1;  //Output enable in bidirectional mode
    u1 BIDIMODE   : 1;  //Bidirectional data mode enable
    u1 RSVD0      : 16; //Reserved 
  }volatile CR1;              //SPI control register 1 (SPI_CR1) (not used in I2S mode)
  struct
  {
    u1 RXDMAEN    : 1;  //Rx buffer DMA enable
    u1 TXDMAEN    : 1;  //Tx buffer DMA enable
    u1 SSOE       : 1;  //SS output enable
    u1 NSSP       : 1;  //NSS pulse management
    u1 FRF        : 1;  //Frame format
    u1 ERRIE      : 1;  //Error interrupt enable
    u1 RXNEIE     : 1;  //RX buffer not empty interrupt enable
    u1 TXEIE      : 1;  //Tx buffer empty interrupt enable
    u1 DS         : 4;  //Data size
    u1 FRXTH      : 1;  //FIFO reception threshold
    u1 LDMA_RX    : 1;  //Last DMA transfer for reception
    u1 LDMA_TX    : 1;  //Last DMA transfer for transmission
    u1 RSVD1      : 17; //Reserved 
  }volatile CR2;              //SPI control register 2 (SPI_CR2)
  struct
  {
    u1 RXNE       : 1;  //Receive buffer not empty
    u1 TXE        : 1;  //Transmit buffer empty
    u1 CHSIDE     : 1;  //Channel side
    u1 UDR        : 1;  //Underrun flag
    u1 CRCERR     : 1;  //CRC error flag
    u1 MODF       : 1;  //Mode fault
    u1 OVR        : 1;  //Overrun flag
    u1 BSY        : 1;  //Busy flag
    u1 FRE        : 1;  //Frame format error
    u1 FRLVL      : 2;  //FIFO reception level
    u1 FTLVL      : 2;  //FIFO Transmission Level
    u1 RSVD0      : 19; //Reserved 
  }volatile SR;               //SPI status register (SPI_SR)
  volatile u32 DR;            //SPI data register (SPI_DR)
  struct
  {
    u1 CRCPOLY    : 16; //CRC polynomial register
    u1 RSVD0      : 16; //Reserved
  }volatile CRCPR;            //SPI CRC polynomial register (SPI_CRCPR) (not used in I2S mode)
  struct
  {
    u1 RXCRC      : 16; //Rx CRC register
    u1 RSVD0      : 16; //Reserved
  }volatile RXCRCR;           //SPI Rx CRC register (SPI_RXCRCR) (not used in I2S mode)
  struct
  {
    u1 TXCRC      : 16; //Tx CRC register
    u1 RSVD0      : 16; //Reserved
  }volatile TXCRCR;           //SPI Tx CRC register (SPI_TXCRCR) (not used in I2S mode)
  struct
  {
    u1 CHLEN      : 1;  //Channel length (number of bits per audio channel)
    u1 DATLEN     : 2;  //Data length to be transferred
    u1 CKPOL      : 1;  //Steady state clock polarity
    u1 I2SSTD     : 2;  //I2S standard selection
    u1 RSVD0      : 1;  //Reserved
    u1 PCMSYNC    : 1;  //PCM frame synchronization
    u1 I2SCFG     : 2;  //I2S configuration mode
    u1 I2SE       : 1;  //I2S Enable
    u1 I2SMOD     : 1;  //I2S mode selection
    u1 RSVD1      : 20; //Reserved
  }volatile I2SCFGR;          //SPI_I2S configuration register (SPI_I2SCFGR)
  struct
  {
    u1 I2SDIV     : 8;  //I2S Linear prescaler
    u1 ODD        : 1;  //Odd factor for the prescaler
    u1 MCKOE      : 1;  //Master clock output enable
    u1 RSVD0      : 22; //Reserved
  }volatile I2SPR;            //SPI_I2S prescaler register (SPI_I2SPR)
}SPI_Type;
#define SPI1 ((SPI_Type*) SPI1_BASE)
#define SPI2 ((SPI_Type*) SPI2_BASE)
#define SPI_CPHA(PORT)          PORT->CR1.CPHA                  //Clock phase
#define SPI_CPOL(PORT)          PORT->CR1.CPOL                  //Clock polarity
#define SPI_MSTR(PORT)          PORT->CR1.MSTR                  //Master selection
#define SPI_DIV(PORT)           PORT->CR1.BR                    //Baud rate control
#define SPI_EN(PORT)            PORT->CR1.SPE                   //SPI enable
#define SPI_LSBFIRST(PORT)      PORT->CR1.LSBFIRST              //Frame format
#define SPI_SSI(PORT)           PORT->CR1.SSI                   //Internal slave select
#define SPI_SSM(PORT)           PORT->CR1.SSM                   //Software slave management
#define SPI_RXONLY(PORT)        PORT->CR1.RXONLY                //Receive only
#define SPI_CRCL(PORT)          PORT->CR1.CRCL                  //CRC length
#define SPI_CRC_NEXT(PORT)      PORT->CR1.CRCNEXT               //Transmit CRC next
#define SPI_CRC_EN(PORT)        PORT->CR1.CRCEN                 //Hardware CRC calculation enable
#define SPI_BIDIOE(PORT)        PORT->CR1.BIDIOE                //Output enable in bidirectional mode
#define SPI_BIDIMODE(PORT)      PORT->CR1.BIDIMODE              //Bidirectional data mode enable
#define SPI_RXDMA_EN(PORT)      PORT->CR2.RXDMAEN               //Rx buffer DMA enable
#define SPI_TXDMA_EN(PORT)      PORT->CR2.TXDMAEN               //Tx buffer DMA enable
#define SPI_SS_EN(PORT)         PORT->CR2.SSOE                  //SS output enable
#define SPI_NSSP(PORT)          PORT->CR2.NSSP                  //NSS pulse management
#define SPI_FRF(PORT)           PORT->CR2.FRF                   //Frame format 0: SPI Motorola mode, 1: SPI TI mode
#define SPI_ERR_IRQ(PORT)       PORT->CR2.ERRIE                 //Error interrupt enable
#define SPI_RXNE_IRQ(PORT)      PORT->CR2.RXNEIE                //RX buffer not empty interrupt enable
#define SPI_TXE_IRQ(PORT)       PORT->CR2.TXEIE                 //Tx buffer empty interrupt enable
#define SPI_FRAME(PORT)         PORT->CR2.DS                    //Data size
#define SPI_FRXTH(PORT)         PORT->CR2.FRXTH                 //FIFO reception threshold 0: RXNE event is generated if 16-bit, 1: RXNE event is generated if 8-bit
#define SPI_LDMA_RX(PORT)       PORT->CR2.LDMA_RX               //Last DMA transfer for reception
#define SPI_LDMA_TX(PORT)       PORT->CR2.LDMA_TX               //Last DMA transfer for transmission
#define SPI_RXNE_FLG(PORT)      PORT->SR.RXNE                   //Receive buffer not empty
#define SPI_TXE_FLG(PORT)       PORT->SR.TXE                    //Transmit buffer empty
#define SPI_CHSD_FLG(PORT)      PORT->SR.CHSIDE                 //Channel side
#define SPI_UDR_FLG(PORT)       PORT->SR.UDR                    //Underrun flag
#define SPI_CRC_FLG(PORT)       PORT->SR.CRCERR                 //CRC error flag
#define SPI_MODE_FLG(PORT)      PORT->SR.MODF                   //Mode fault
#define SPI_OVR_FLG(PORT)       PORT->SR.OVR                    //Overrun flag
#define SPI_BSY_FLG(PORT)       PORT->SR.BSY                    //Busy flag
#define SPI_FRE(PORT)           PORT->SR.FRE                    //Frame format error
#define SPI_FRLVL(PORT)         PORT->SR.FRLVL                  //FIFO reception level
#define SPI_FTLVL(PORT)         PORT->SR.FTLVL                  //FIFO Transmission Level
#define SPI_DATA(PORT)          PORT->DR                        //Data register
#define SPI_CRCPOLY(PORT)       PORT->CRCPR.CRCPOLY             //CRC polynomial register
#define SPI_RXCRC(PORT)         PORT->RXCRCR.RXCRC              //Rx CRC register
#define SPI_TXCRC(PORT)         PORT->TXCRCR.TXCRC              //Tx CRC register
#define SPI_CHLEN(PORT)         PORT->I2SCFGR.CHLEN             //Channel length (number of bits per audio channel)
#define SPI_DATLEN(PORT)        PORT->I2SCFGR.DATLEN            //Data length to be transferred
#define SPI_CKPOL(PORT)         PORT->I2SCFGR.CKPOL             //Steady state clock polarity
#define SPI_I2SSTD(PORT)        PORT->I2SCFGR.I2SSTD            //I2S standard selection
#define SPI_PCMSYNC(PORT)       PORT->I2SCFGR.PCMSYNC           //PCM frame synchronization
#define SPI_I2S_CFG(PORT)       PORT->I2SCFGR.I2SCFG            //I2S configuration mode
#define SPI_I2S_EN(PORT)        PORT->I2SCFGR.I2SE              //I2S Enable
#define SPI_I2S_MODE(PORT)      PORT->I2SCFGR.I2SMOD            //I2S mode selection
#define SPI_I2S_DIV(PORT)       PORT->I2SPR.I2SDIV              //I2S Linear prescaler
#define SPI_ODD(PORT)           PORT->I2SPR.ODD                 //Odd factor for the prescaler
#define SPI_MCKOE(PORT)         PORT->I2SPR.MCKOE               //Master clock output enable
#define SPI_MODE0               ((u8) 0x00)                     //Mode 0 - CPOL = 0, CPHA = 0
#define SPI_MODE1               ((u8) 0x10)                     //Mode 1 - CPOL = 0, CPHA = 1
#define SPI_MODE2               ((u8) 0x01)                     //Mode 2 - CPOL = 1, CPHA = 0
#define SPI_MODE3               ((u8) 0x11)                     //Mode 3 - CPOL = 1, CPHA = 1
#define SPI_DIV_002             ((u8) 0x00)                     //DIV 2
#define SPI_DIV_004             ((u8) 0x01)                     //DIV 4
#define SPI_DIV_008             ((u8) 0x02)                     //DIV 8
#define SPI_DIV_016             ((u8) 0x03)                     //DIV 16
#define SPI_DIV_032             ((u8) 0x04)                     //DIV 32
#define SPI_DIV_064             ((u8) 0x05)                     //DIV 64
#define SPI_DIV_128             ((u8) 0x06)                     //DIV 128
#define SPI_DIV_256             ((u8) 0x07)                     //DIV 256
#define SPI_Master              ((u8) 0x01)                     //Mode Master
#define SPI_Slave               ((u8) 0x00)                     //Mode Slave
#define SPI_FRAME4              ((u8) 0x03)                     //Data frame format 4 bit
#define SPI_FRAME5              ((u8) 0x04)                     //Data frame format 5 bit
#define SPI_FRAME6              ((u8) 0x05)                     //Data frame format 6 bit
#define SPI_FRAME7              ((u8) 0x06)                     //Data frame format 7 bit
#define SPI_FRAME8              ((u8) 0x07)                     //Data frame format 8 bit
#define SPI_FRAME9              ((u8) 0x08)                     //Data frame format 9 bit
#define SPI_FRAME10             ((u8) 0x09)                     //Data frame format 10 bit
#define SPI_FRAME11             ((u8) 0x0A)                     //Data frame format 11 bit
#define SPI_FRAME12             ((u8) 0x0B)                     //Data frame format 12 bit
#define SPI_FRAME13             ((u8) 0x0C)                     //Data frame format 13 bit
#define SPI_FRAME14             ((u8) 0x0D)                     //Data frame format 14 bit
#define SPI_FRAME15             ((u8) 0x0E)                     //Data frame format 15 bit
#define SPI_FRAME16             ((u8) 0x0F)                     //Data frame format 16 bit
/******************************************** TIM *************************************************/
typedef struct
{
  struct
  {
    u1 CEN        : 1;  //Counter enable
    u1 UDIS       : 1;  //Update disable
    u1 URS        : 1;  //Update request source
    u1 OPM        : 1;  //One pulse mode
    u1 DIR        : 1;  //Direction
    u1 CMS        : 2;  //Center-aligned mode selection
    u1 ARPE       : 1;  //Auto-reload preload enable
    u1 CKD        : 2;  //Clock division
    u1 RSVD0      : 22; //Reserved
  }volatile CR1;              //TIM1&TIM8 control register 1 (TIMx_CR1)
  struct
  {
    u1 CCPC       : 1;  //Capture/compare preloaded control
    u1 RSVD0      : 1;  //Reserved
    u1 CCUS       : 1;  //Capture/compare control update selection
    u1 CCDS       : 1;  //Capture/compare DMA selection
    u1 MMS        : 3;  //Master mode selection
    u1 TI1S       : 1;  //TI1 selection
    u1 OIS1       : 1;  //Output Idle state 1 (OC1 output)
    u1 OIS1N      : 1;  //Output Idle state 1 (OC1N output)
    u1 OIS2       : 1;  //Output Idle state 2 (OC2 output)
    u1 OIS2N      : 1;  //Output Idle state 2 (OC2N output)
    u1 OIS3       : 1;  //Output Idle state 3 (OC3 output)
    u1 OIS3N      : 1;  //Output Idle state 3 (OC3N output)
    u1 OIS4       : 1;  //Output Idle state 4 (OC4 output)
    u1 RSVD1      : 17; //Reserved
  }volatile CR2;              //TIM1&TIM8 control register 2 (TIMx_CR2)
  struct
  {
    u1 SMS        : 3;  //Slave mode selection
    u1 OCCS       : 1;  //OCREF clear selection
    u1 TS         : 3;  //Trigger selection
    u1 MSM        : 1;  //Master/slave mode
    u1 ETF        : 4;  //External trigger filter
    u1 ETPS       : 2;  //External trigger prescaler
    u1 ECE        : 1;  //External clock enable
    u1 ETP        : 1;  //External trigger polarity
    u1 RSVD0      : 16; //Reserved
  }volatile SMCR;             //TIM1&TIM8 slave mode control register (TIMx_SMCR)
  struct
  {
    u1 UIE        : 1;  //Update interrupt enable
    u1 CC1IE      : 1;  //Capture/Compare 1 interrupt enable
    u1 CC2IE      : 1;  //Capture/Compare 2 interrupt enable
    u1 CC3IE      : 1;  //Capture/Compare 3 interrupt enable
    u1 CC4IE      : 1;  //Capture/Compare 4 interrupt enable
    u1 COMIE      : 1;  //COM interrupt enable
    u1 TIE        : 1;  //Trigger interrupt enable
    u1 BIE        : 1;  //Break interrupt enable
    u1 UDE        : 1;  //Update DMA request enable
    u1 CC1DE      : 1;  //Capture/Compare 1 DMA request enable
    u1 CC2DE      : 1;  //Capture/Compare 2 DMA request enable
    u1 CC3DE      : 1;  //Capture/Compare 3 DMA request enable
    u1 CC4DE      : 1;  //Capture/Compare 4 DMA request enable
    u1 COMDE      : 1;  //COM DMA request enable
    u1 TDE        : 1;  //Trigger DMA request enable
    u1 RSVD0      : 17; //Reserved
  }volatile DIER;             //TIM1&TIM8 DMA/interrupt enable register (TIMx_DIER)
  struct
  {
    u1 UIF        : 1;  //Update interrupt enable
    u1 CC1IF      : 1;  //Capture/Compare 1 interrupt flag
    u1 CC2IF      : 1;  //Capture/Compare 2 interrupt flag
    u1 CC3IF      : 1;  //Capture/Compare 3 interrupt flag
    u1 CC4IF      : 1;  //Capture/Compare 4 interrupt flag
    u1 COMIF      : 1;  //COM interrupt flag
    u1 TIF        : 1;  //Trigger interrupt flag
    u1 BIF        : 1;  //Break interrupt flag
    u1 RSVD0      : 1;  //Reserved
    u1 CC1OF      : 1;  //Capture/Compare 1 overcapture flag
    u1 CC2OF      : 1;  //Capture/Compare 2 overcapture flag
    u1 CC3OF      : 1;  //Capture/Compare 3 overcapture flag
    u1 CC4OF      : 1;  //Capture/Compare 4 overcapture flag
    u1 RSVD1      : 19; //Reserved
  }volatile SR;               //TIM1&TIM8 status register (TIMx_SR)
  struct
  {
    u1 UG         : 1;  //Update generation
    u1 CC1G       : 1;  //Capture/Compare 1 generation
    u1 CC2G       : 1;  //Capture/Compare 2 generation
    u1 CC3G       : 1;  //Capture/Compare 3 generation
    u1 CC4G       : 1;  //Capture/Compare 4 generation
    u1 COMG       : 1;  //Capture/Compare control update generation
    u1 TG         : 1;  //Trigger generation
    u1 BG         : 1;  //Break generation
    u1 RSVD0      : 24; //Reserved
  }volatile EGR;              //TIM1&TIM8 status register (TIMx_SR)
  union
  {
    struct
    {
      u1 CC1S       : 2;  //Capture/Compare 1 Selection
      u1 IC1PSC     : 2;  //Input capture 1 prescaler
      u1 IC1F       : 4;  //Input capture 1 filter
      u1 CC2S       : 2;  //Capture/Compare 2 selection
      u1 IC2PSC     : 2;  //Input capture 2 prescaler
      u1 IC2F       : 4;  //Input capture 2 filter
      u1 RSVD0      : 16; //Reserved
    }volatile INP;              //Input capture mode
    struct
    {
      u1 CC1S       : 2;  //Capture/Compare 1 Selection
      u1 OC1FE      : 1;  //Output Compare 1 fast enable
      u1 OC1PE      : 1;  //Output Compare 1 preload enable
      u1 OC1M       : 3;  //Output Compare 1 mode
      u1 OC1CE      : 1;  //Output Compare 1 clear enable
      u1 CC2S       : 2;  //Capture/Compare 2 Selection
      u1 OC2FE      : 1;  //Output Compare 2 fast enable
      u1 OC2PE      : 1;  //Output Compare 2 preload enable
      u1 OC2M       : 3;  //Output Compare 2 mode
      u1 OC2CE      : 1;  //Output Compare 2 clear enable
      u1 RSVD0      : 16; //Reserved
    }volatile OUT;              //Output compare mode
  }volatile CCMR1;              //TIM1&TIM8 capture/compare mode register 1 (TIMx_CCMR1)
  union
  {
    struct
    {
      u1 CC3S       : 2;  //Capture/Compare 3 Selection
      u1 IC3PSC     : 2;  //Input capture 3 prescaler
      u1 IC3F       : 4;  //Input capture 3 filter
      u1 CC4S       : 2;  //Capture/Compare 4 selection
      u1 IC4PSC     : 2;  //Input capture 4 prescaler
      u1 IC4F       : 4;  //Input capture 4 filter
      u1 RSVD0      : 16; //Reserved
    }volatile INP;              //Input capture mode
    struct
    {
      u1 CC3S       : 2;  //Capture/Compare 3 Selection
      u1 OC3FE      : 1;  //Output Compare 3 fast enable
      u1 OC3PE      : 1;  //Output Compare 3 preload enable
      u1 OC3M       : 3;  //Output Compare 3 mode
      u1 OC3CE      : 1;  //Output Compare 3 clear enable
      u1 CC4S       : 2;  //Capture/Compare 4 Selection
      u1 OC4FE      : 1;  //Output Compare 4 fast enable
      u1 OC4PE      : 1;  //Output Compare 4 preload enable
      u1 OC4M       : 3;  //Output Compare 4 mode
      u1 OC4CE      : 1;  //Output Compare 4 clear enable
      u1 RSVD0      : 16; //Reserved
    }volatile OUT;              //Output compare mode
  }volatile CCMR2;              //TIM1&TIM8 capture/compare mode register 2 (TIMx_CCMR2)
  struct
  {
    u1 CC1E       : 1;  //Capture/Compare 1 output enable
    u1 CC1P       : 1;  //Capture/Compare 1 output polarity
    u1 CC1NE      : 1;  //Capture/Compare 1 complementary output enable
    u1 CC1NP      : 1;  //Capture/Compare 1 complementary output polarity
    u1 CC2E       : 1;  //Capture/Compare 2 output enable
    u1 CC2P       : 1;  //Capture/Compare 2 output polarity
    u1 CC2NE      : 1;  //Capture/Compare 2 complementary output enable
    u1 CC2NP      : 1;  //Capture/Compare 2 complementary output polarity
    u1 CC3E       : 1;  //Capture/Compare 3 output enable
    u1 CC3P       : 1;  //Capture/Compare 3 output polarity
    u1 CC3NE      : 1;  //Capture/Compare 3 complementary output enable
    u1 CC3NP      : 1;  //Capture/Compare 3 complementary output polarity
    u1 CC4E       : 1;  //Capture/Compare 4 output enable
    u1 CC4P       : 1;  //Capture/Compare 4 output polarity
    u1 CC4NE      : 1;  //Capture/Compare 4 complementary output enable
    u1 CC4NP      : 1;  //Capture/Compare 4 complementary output polarity
    u1 RSVD0      : 16; //Reserved
  }volatile CCER;             //TIM1&TIM8 capture/compare enable register (TIMx_CCER)
  struct
  {
    u1 CNT        : 16; //Counter value
    u1 RSVD0      : 16; //Reserved
  }volatile CNT;              //TIM1&TIM8 counter (TIMx_CNT)
  struct
  {
    u1 PSC        : 16; //Prescaler value
    u1 RSVD0      : 16; //Reserved
  }volatile PSC;              //TIM1&TIM8 prescaler (TIMx_PSC)
  struct
  {
    u1 ARR        : 16; //Prescaler value
    u1 RSVD0      : 16; //Reserved
  }volatile ARR;              //TIM1&TIM8 auto-reload register (TIMx_ARR)
  struct
  {
    u1 REP        : 8;  //Repetition counter value
    u1 RSVD0      : 24; //Reserved
  }volatile RCR;              //TIM1&TIM8 repetition counter register (TIMx_RCR)
  struct
  {
    u1 CCR1       : 16; //Capture/Compare 1 value
    u1 RSVD0      : 16; //Reserved
  }volatile CCR1;             //TIM1&TIM8 capture/compare register 1 (TIMx_CCR1)
  struct
  {
    u1 CCR2       : 16; //Capture/Compare 2 value
    u1 RSVD0      : 16; //Reserved
  }volatile CCR2;             //TIM1&TIM8 capture/compare register 2 (TIMx_CCR2)
  struct
  {
    u1 CCR3       : 16; //Capture/Compare 3 value
    u1 RSVD0      : 16; //Reserved
  }volatile CCR3;             //TIM1&TIM8 capture/compare register 3 (TIMx_CCR3)
  struct
  {
    u1 CCR4       : 16; //Capture/Compare 4 value
    u1 RSVD0      : 16; //Reserved
  }volatile CCR4;             //TIM1&TIM8 capture/compare register 4 (TIMx_CCR4)
  struct
  {
    u1 DTG        : 8;  //Dead-time generator setup
    u1 LOCK       : 2;  //Lock configuration
    u1 OSSI       : 1;  //Off-state selection for Idle mode
    u1 OSSR       : 1;  //Off-state selection for Run mode
    u1 BKE        : 1;  //Break enable
    u1 BKPOL      : 1;  //Break polarity
    u1 AOE        : 1;  //Automatic output enable
    u1 MOE        : 1;  //Main output enable
    u1 RSVD0      : 16; //Reserved
  }volatile BDTR;             //TIM1&TIM8 break and dead-time register (TIMx_BDTR)
  struct
  {
    u1 DBA        : 5;  //Dead-time generator setup
    u1 RSVD0      : 3;  //Reserved
    u1 DBL        : 5;  //DMA burst length
    u1 RSVD1      : 19; //Reserved
  }volatile DCR;              //TIM1&TIM8 DMA control register (TIMx_DCR)
  struct
  {
    u1 DMAB       : 16; //DMA register for burst accesses
    u1 RSVD0      : 16; //Reserved
  }volatile DMAR;             //TIM1&TIM8 DMA address for full transfer (TIMx_DMAR)
  struct
  {
    u1 TI1_RMP    : 2;  //Timer Input 1 remap
    u1 RSVD0      : 30; //Reserved
  }volatile OR;              //TIM14 option register (TIM14_OR)
}TIM_Type;
#define TIM01 ((TIM_Type*) TIM01_BASE)
#define TIM02 ((TIM_Type*) TIM02_BASE)
#define TIM03 ((TIM_Type*) TIM03_BASE)
#define TIM06 ((TIM_Type*) TIM06_BASE)
#define TIM07 ((TIM_Type*) TIM07_BASE)
#define TIM14 ((TIM_Type*) TIM14_BASE)
#define TIM15 ((TIM_Type*) TIM15_BASE)
#define TIM16 ((TIM_Type*) TIM16_BASE)
#define TIM17 ((TIM_Type*) TIM17_BASE)
#define TIM_EN(PORT)            PORT->CR1.CEN                   //Counter enable                        (TIM1 2 3 6 7 14 15 16 17)
#define TIM_UDIS(PORT)          PORT->CR1.UDIS                  //Update disable                        (TIM1 2 3 6 7 14 15 16 17)
#define TIM_URS(PORT)           PORT->CR1.URS                   //Update request source                 (TIM1 2 3 6 7 14 15 16 17)
#define TIM_OPM(PORT)           PORT->CR1.OPM                   //One pulse mode                        (TIM1 2 3 6 7    15 16 17)
#define TIM_DIR(PORT)           PORT->CR1.DIR                   //Direction                             (TIM1 2 3                )
#define TIM_CMS(PORT)           PORT->CR1.CMS                   //Center-aligned mode selection         (TIM1 2 3                )
#define TIM_ARPE(PORT)          PORT->CR1.ARPE                  //Auto-reload preload enable            (TIM1 2 3     14 15 16 17)
#define TIM_DIV(PORT)           PORT->CR1.CKD                   //Clock division                        (TIM1 2 3     14 15 16 17)
#define TIM_CCPC(PORT)          PORT->CR2.CCPC                  //C/C preloaded control                 (TIM1            15 16 17)
#define TIM_CCUS(PORT)          PORT->CR2.CCUS                  //C/C control update selection          (TIM1            15 16 17)
#define TIM_CCDS(PORT)          PORT->CR2.CCDS                  //C/C DMA selection                     (TIM1 2 3        15 16 17)
#define TIM_MMS(PORT)           PORT->CR2.MMS                   //Master mode selection                 (TIM1 2 3 6 7    15      )
#define TIM_TI1S(PORT)          PORT->CR2.TI1S                  //TI1 selection                         (TIM1 2 3                )
#define TIM_OIS1(PORT)          PORT->CR2.OIS1                  //OUT Idle state 1 (OC1 output)         (TIM1            15 16 17)
#define TIM_OIS1N(PORT)         PORT->CR2.OIS1N                 //OUT Idle state 1 (OC1N output)        (TIM1            15 16 17)
#define TIM_OIS2(PORT)          PORT->CR2.OIS2                  //OUT Idle state 2 (OC2 output)         (TIM1            15      )
#define TIM_OIS2N(PORT)         PORT->CR2.OIS2N                 //OUT Idle state 2 (OC2N output)        (TIM1                    )
#define TIM_OIS3(PORT)          PORT->CR2.OIS3                  //OUT Idle state 3 (OC3 output)         (TIM1                    )
#define TIM_OIS3N(PORT)         PORT->CR2.OIS3N                 //OUT Idle state 3 (OC3N output)        (TIM1                    )
#define TIM_OIS4(PORT)          PORT->CR2.OIS4                  //OUT Idle state 4 (OC4 output)         (TIM1                    )
#define TIM_SMS(PORT)           PORT->SMCR.SMS                  //Slave mode selection                  (TIM1 2 3        15      )
#define TIM_OCCS(PORT)          PORT->SMCR.OCCS                 //OCREF clear selection                 (TIM1 2 3                )
#define TIM_TS(PORT)            PORT->SMCR.TS                   //Trigger selection                     (TIM1 2 3        15      )
#define TIM_MSM(PORT)           PORT->SMCR.MSM                  //Master/slave mode                     (TIM1 2 3        15      )
#define TIM_ETF(PORT)           PORT->SMCR.ETF                  //External trigger filter               (TIM1 2 3                )
#define TIM_ETPS(PORT)          PORT->SMCR.ETPS                 //External trigger prescaler            (TIM1 2 3                )
#define TIM_ECE(PORT)           PORT->SMCR.ECE                  //External clock enable                 (TIM1 2 3                )
#define TIM_ETP(PORT)           PORT->SMCR.ETP                  //External trigger polarity             (TIM1 2 3                )
#define TIM_UIE(PORT)           PORT->DIER.UIE                  //Update interrupt enable               (TIM1 2 3 6 7 14 15 16 17)
#define TIM_CC1IE(PORT)         PORT->DIER.CC1IE                //C/C 1 interrupt enable                (TIM1 2 3     14 15 16 17)
#define TIM_CC2IE(PORT)         PORT->DIER.CC2IE                //C/C 2 interrupt enable                (TIM1 2 3        15      )
#define TIM_CC3IE(PORT)         PORT->DIER.CC3IE                //C/C 3 interrupt enable                (TIM1 2 3                )
#define TIM_CC4IE(PORT)         PORT->DIER.CC4IE                //C/C 4 interrupt enable                (TIM1 2 3                )
#define TIM_COMIE(PORT)         PORT->DIER.COMIE                //COM interrupt enable                  (TIM1            15 16 17)
#define TIM_TIE(PORT)           PORT->DIER.TIE                  //Trigger interrupt enable              (TIM1 2 3        15      )
#define TIM_BIE(PORT)           PORT->DIER.BIE                  //Break interrupt enable                (TIM1            15 16 17)
#define TIM_UDE(PORT)           PORT->DIER.UDE                  //Update DMA request enable             (TIM1 2 3 6 7    15 16 17)
#define TIM_CC1DE(PORT)         PORT->DIER.CC1DE                //C/C 1 DMA request enable              (TIM1 2 3        15 16 17)
#define TIM_CC2DE(PORT)         PORT->DIER.CC2DE                //C/C 2 DMA request enable              (TIM1 2 3        15      )
#define TIM_CC3DE(PORT)         PORT->DIER.CC3DE                //C/C 3 DMA request enable              (TIM1 2 3                )
#define TIM_CC4DE(PORT)         PORT->DIER.CC4DE                //C/C 4 DMA request enable              (TIM1 2 3                )
#define TIM_COMDE(PORT)         PORT->DIER.COMDE                //COM DMA request enable                (TIM1                    )
#define TIM_TDE(PORT)           PORT->DIER.TDE                  //Trigger DMA request enable            (TIM1 2 3        15      )
#define TIM_UIF(PORT)           PORT->SR.UIF                    //Update interrupt enable               (TIM1 2 3 6 7 14 15 16 17)
#define TIM_CC1IF(PORT)         PORT->SR.CC1IF                  //C/C 1 interrupt flag                  (TIM1 2 3     14 15 16 17)
#define TIM_CC2IF(PORT)         PORT->SR.CC2IF                  //C/C 2 interrupt flag                  (TIM1 2 3        15      )
#define TIM_CC3IF(PORT)         PORT->SR.CC3IF                  //C/C 3 interrupt flag                  (TIM1 2 3                )
#define TIM_CC4IF(PORT)         PORT->SR.CC4IF                  //C/C 4 interrupt flag                  (TIM1 2 3                )
#define TIM_COMIF(PORT)         PORT->SR.COMIF                  //COM interrupt flag                    (TIM1            15 16 17)
#define TIM_TIF(PORT)           PORT->SR.TIF                    //Trigger interrupt flag                (TIM1 2 3        15      )
#define TIM_BIF(PORT)           PORT->SR.BIF                    //Break interrupt flag                  (TIM1            15 16 17)
#define TIM_CC1OF(PORT)         PORT->SR.CC1OF                  //C/C 1 overcapture flag                (TIM1 2 3     14 15 16 17)
#define TIM_CC2OF(PORT)         PORT->SR.CC2OF                  //C/C 2 overcapture flag                (TIM1 2 3        15      )
#define TIM_CC3OF(PORT)         PORT->SR.CC3OF                  //C/C 3 overcapture flag                (TIM1 2 3                )
#define TIM_CC4OF(PORT)         PORT->SR.CC4OF                  //C/C 4 overcapture flag                (TIM1 2 3                )
#define TIM_UG(PORT)            PORT->EGR.UG                    //Update generation                     (TIM1 2 3 6 7 14 15 16 17)
#define TIM_CC1G(PORT)          PORT->EGR.CC1G                  //C/C 1 generation                      (TIM1 2 3     14 15 16 17)
#define TIM_CC2G(PORT)          PORT->EGR.CC2G                  //C/C 2 generation                      (TIM1 2 3        15      )
#define TIM_CC3G(PORT)          PORT->EGR.CC3G                  //C/C 3 generation                      (TIM1 2 3                )
#define TIM_CC4G(PORT)          PORT->EGR.CC4G                  //C/C 4 generation                      (TIM1 2 3                )
#define TIM_COMG(PORT)          PORT->EGR.COMG                  //C/C control update generation         (TIM1            15 16 17)
#define TIM_TG(PORT)            PORT->EGR.TG                    //Trigger generation                    (TIM1 2 3        15      )
#define TIM_BG(PORT)            PORT->EGR.BG                    //Break generation                      (TIM1            15 16 17)
#define TIM_CC1S(PORT)          PORT->CCMR1.INP.CC1S            //C/C 1 Selection                       (TIM1 2 3     14 15 16 17)
#define TIM_IC1PSC(PORT)        PORT->CCMR1.INP.IC1PSC          //INP capture 1 prescaler               (TIM1 2 3     14 15 16 17)
#define TIM_IC1F(PORT)          PORT->CCMR1.INP.IC1F            //INP capture 1 filter                  (TIM1 2 3     14 15 16 17)
#define TIM_CC2S(PORT)          PORT->CCMR1.INP.CC2S            //C/C 2 selection                       (TIM1 2 3        15      )
#define TIM_IC2PSC(PORT)        PORT->CCMR1.INP.IC2PSC          //INP capture 2 prescaler               (TIM1 2 3        15      )
#define TIM_IC2F(PORT)          PORT->CCMR1.INP.IC2F            //INP capture 2 filter                  (TIM1 2 3        15      )
#define TIM_OC1FE(PORT)         PORT->CCMR1.OUT.OC1FE           //OUT Compare 1 fast enable             (TIM1 2 3     14 15 16 17)
#define TIM_OC1PE(PORT)         PORT->CCMR1.OUT.OC1PE           //OUT Compare 1 preload enable          (TIM1 2 3     14 15 16 17)
#define TIM_OC1M(PORT)          PORT->CCMR1.OUT.OC1M            //OUT Compare 1 mode                    (TIM1 2 3     14 15 16 17)
#define TIM_OC1CE(PORT)         PORT->CCMR1.OUT.OC1CE           //OUT Compare 1 clear enable            (TIM1 2 3                )
#define TIM_OC2FE(PORT)         PORT->CCMR1.OUT.OC2FE           //OUT Compare 2 fast enable             (TIM1 2 3        15      )
#define TIM_OC2PE(PORT)         PORT->CCMR1.OUT.OC2PE           //OUT Compare 2 preload enable          (TIM1 2 3        15      )
#define TIM_OC2M(PORT)          PORT->CCMR1.OUT.OC2M            //OUT Compare 2 mode                    (TIM1 2 3        15      )
#define TIM_OC2CE(PORT)         PORT->CCMR1.OUT.OC2CE           //OUT Compare 2 clear enable            (TIM1 2 3                )
#define TIM_CC3S(PORT)          PORT->CCMR2.INP.CC3S            //C/C 3 Selection                       (TIM1 2 3                )
#define TIM_IC3PSC(PORT)        PORT->CCMR2.INP.IC3PSC          //INP capture 3 prescaler               (TIM1 2 3                )
#define TIM_IC3F(PORT)          PORT->CCMR2.INP.IC3F            //INP capture 3 filter                  (TIM1 2 3                )
#define TIM_CC4S(PORT)          PORT->CCMR2.INP.CC4S            //C/C 4 selection                       (TIM1 2 3                )
#define TIM_IC4PSC(PORT)        PORT->CCMR2.INP.IC4PSC          //INP capture 4 prescaler               (TIM1 2 3                )
#define TIM_IC4F(PORT)          PORT->CCMR2.INP.IC4F            //INP capture 4 filter                  (TIM1 2 3                )
#define TIM_OC3FE(PORT)         PORT->CCMR2.OUT.OC3FE           //OUT Compare 3 fast enable             (TIM1 2 3                )
#define TIM_OC3PE(PORT)         PORT->CCMR2.OUT.OC3PE           //OUT Compare 3 preload enable          (TIM1 2 3                )
#define TIM_OC3M(PORT)          PORT->CCMR2.OUT.OC3M            //OUT Compare 3 mode                    (TIM1 2 3                )
#define TIM_OC3CE(PORT)         PORT->CCMR2.OUT.OC3CE           //OUT Compare 3 clear enable            (TIM1 2 3                )
#define TIM_OC4FE(PORT)         PORT->CCMR2.OUT.OC4FE           //OUT Compare 4 fast enable             (TIM1 2 3                )
#define TIM_OC4PE(PORT)         PORT->CCMR2.OUT.OC4PE           //OUT Compare 4 preload enable          (TIM1 2 3                )
#define TIM_OC4M(PORT)          PORT->CCMR2.OUT.OC4M            //OUT Compare 4 mode                    (TIM1 2 3                )
#define TIM_OC4CE(PORT)         PORT->CCMR2.OUT.OC4CE           //OUT Compare 4 clear enable            (TIM1 2 3                )
#define TIM_CC1E(PORT)          PORT->CCER.CC1E                 //C/C 1 OUT enable                      (TIM1 2 3     14 15 16 17)
#define TIM_CC1P(PORT)          PORT->CCER.CC1P                 //C/C 1 OUT polarity                    (TIM1 2 3     14 15 16 17)
#define TIM_CC1NE(PORT)         PORT->CCER.CC1NE                //C/C 1 complementary OUT enable        (TIM1            15 16 17)
#define TIM_CC1NP(PORT)         PORT->CCER.CC1NP                //C/C 1 complementary OUT polarity      (TIM1 2 3     14 15 16 17)
#define TIM_CC2E(PORT)          PORT->CCER.CC2E                 //C/C 2 OUT enable                      (TIM1 2 3        15      )
#define TIM_CC2P(PORT)          PORT->CCER.CC2P                 //C/C 2 OUT polarity                    (TIM1 2 3        15      )
#define TIM_CC2NE(PORT)         PORT->CCER.CC2NE                //C/C 2 complementary OUT enable        (TIM1                    )
#define TIM_CC2NP(PORT)         PORT->CCER.CC2NP                //C/C 2 complementary OUT polarity      (TIM1 2 3        15      )
#define TIM_CC3E(PORT)          PORT->CCER.CC3E                 //C/C 3 OUT enable                      (TIM1 2 3                )
#define TIM_CC3P(PORT)          PORT->CCER.CC3P                 //C/C 3 OUT polarity                    (TIM1 2 3                )
#define TIM_CC3NE(PORT)         PORT->CCER.CC3NE                //C/C 3 complementary OUT enable        (TIM1                    )
#define TIM_CC3NP(PORT)         PORT->CCER.CC3NP                //C/C 3 complementary OUT polarity      (TIM1 2 3                )
#define TIM_CC4E(PORT)          PORT->CCER.CC4E                 //C/C 4 OUT enable                      (TIM1 2 3                )
#define TIM_CC4P(PORT)          PORT->CCER.CC4P                 //C/C 4 OUT polarity                    (TIM1 2 3                )
#define TIM_CC4NE(PORT)         PORT->CCER.CC4NE                //C/C 4 complementary OUT enable        (TIM1                    )
#define TIM_CC4NP(PORT)         PORT->CCER.CC4NP                //C/C 4 complementary OUT polarity      (TIM1 2 3                )
#define TIM_CNT(PORT)           PORT->CNT.CNT                   //Counter value                         (TIM1 2 3 6 7 14 15 16 17)
#define TIM_PSC(PORT)           PORT->PSC.PSC                   //Prescaler value                       (TIM1 2 3 6 7 14 15 16 17)
#define TIM_ARR(PORT)           PORT->ARR.ARR                   //Auto-reload value                     (TIM1 2 3 6 7 14 15 16 17)
#define TIM_REP(PORT)           PORT->RCR.REP                   //Repetition counter value              (TIM1            15 16 17)
#define TIM_CCR1(PORT)          PORT->CCR1.CCR1                 //C/C 1 value                           (TIM1 2 3     14 15 16 17)
#define TIM_CCR2(PORT)          PORT->CCR2.CCR2                 //C/C 2 value                           (TIM1 2 3        15      )
#define TIM_CCR3(PORT)          PORT->CCR3.CCR3                 //C/C 3 value                           (TIM1 2 3                )
#define TIM_CCR4(PORT)          PORT->CCR4.CCR4                 //C/C 4 value                           (TIM1 2 3                )
#define TIM_DTG(PORT)           PORT->BDTR.DTG                  //Dead-time generator setup             (TIM1            15 16 17)
#define TIM_LOCK(PORT)          PORT->BDTR.LOCK                 //Lock configuration                    (TIM1            15 16 17)
#define TIM_OSSI(PORT)          PORT->BDTR.OSSI                 //Off-state selection for Idle mode     (TIM1            15 16 17)
#define TIM_OSSR(PORT)          PORT->BDTR.OSSR                 //Off-state selection for Run mode      (TIM1            15 16 17)
#define TIM_BKE(PORT)           PORT->BDTR.BKE                  //Break enable                          (TIM1            15 16 17)
#define TIM_BKPOL(PORT)         PORT->BDTR.BKPOL                //Break polarity                        (TIM1            15 16 17)
#define TIM_AOE(PORT)           PORT->BDTR.AOE                  //Automatic output enable               (TIM1            15 16 17)
#define TIM_MOE(PORT)           PORT->BDTR.MOE                  //Main output enable                    (TIM1            15 16 17)
#define TIM_DBA(PORT)           PORT->DCR.DBA                   //Dead-time generator setup             (TIM1 2 3        15 16 17)
#define TIM_DBL(PORT)           PORT->DCR.DBL                   //DMA burst length                      (TIM1 2 3        15 16 17)
#define TIM_DMAB(PORT)          PORT->DMAR.DMAB                 //DMA register for burst accesses       (TIM1 2 3        15 16 17)
#define TIM_TI1_RMP(PORT)       PORT->OR.TI1_RMP                //Timer Input 1 remap                   (TIM          14         )
#define TIM_MODE_FROZEN         ((u8) 0x00)                     //The comparison between the output compare register TIMx_CCR1 and the counter TIMx_CNT has no effect on the outputs (this mode is used to generate a timing base).
#define TIM_MODE_HIGH           ((u8) 0x01)                     //Set channel 1 to active level on match. OC1REF signal is forced high when the counter TIMx_CNT matches the capture/compare register 1 (TIMx_CCR1).
#define TIM_MODE_LOW            ((u8) 0x02)                     //Set channel 1 to inactive level on match. OC1REF signal is forced low when the counter TIMx_CNT matches the capture/compare register 1 (TIMx_CCR1).
#define TIM_MODE_TOGGLE         ((u8) 0x03)                     //Toggle - OC1REF toggles when TIMx_CNT=TIMx_CCR1.
#define TIM_MODE_FLOW           ((u8) 0x04)                     //Force inactive level - OC1REF is forced low.
#define TIM_MODE_FHIGH          ((u8) 0x05)                     //Force active level - OC1REF is forced high.
#define TIM_MODE_PWM1           ((u8) 0x06)                     //PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1 else inactive. In downcounting, channel 1 is inactive (OC1REF=‘0’) as long as TIMx_CNT>TIMx_CCR1 else active (OC1REF=’1’).
#define TIM_MODE_PWM2           ((u8) 0x07)                     //PWM mode 2 - In upcounting, channel 1 is inactive as long as TIMx_CNT<TIMx_CCR1 else active. In downcounting, channel 1 is active as long as TIMx_CNT>TIMx_CCR1 else inactive.
/******************************************** UART ************************************************/
typedef struct
{
  struct
  {
    u1 UE         : 1;  //USART enable
    u1 UESM       : 1;  //USART enable in Stop mode
    u1 RE         : 1;  //Receiver enable
    u1 TE         : 1;  //Transmitter enable
    u1 IDLEIE     : 1;  //IDLE interrupt enable
    u1 RXNEIE     : 1;  //RXNE interrupt enable
    u1 TCIE       : 1;  //Transmission complete interrupt enable
    u1 TXEIE      : 1;  //interrupt enable
    u1 PEIE       : 1;  //PE interrupt enable
    u1 PS         : 1;  //Parity selection
    u1 PCE        : 1;  //Parity control enable
    u1 WAKE       : 1;  //Receiver wakeup method
    u1 M0         : 1;  //Word length
    u1 MME        : 1;  //Mute mode enable
    u1 CMIE       : 1;  //Character match interrupt enable
    u1 OVER8      : 1;  //Oversampling mode
    u1 DEDT       : 5;  //Driver Enable de-assertion time
    u1 DEAT       : 5;  //Driver Enable assertion time
    u1 RTOIE      : 1;  //Receiver timeout interrupt enable
    u1 EOBIE      : 1;  //End of Block interrupt enable
    u1 M1         : 1;  //Word length
    u1 RSVD0      : 3;  //Reserved
  }volatile CR1;              //Control register 1 (USARTx_CR1)
  struct
  {
    u1 RSVD0      : 4;  //Reserved
    u1 ADDM7      : 1;  //7-bit Address Detection/4-bit Address Detection
    u1 LBDL       : 1;  //LIN break detection length
    u1 LBDIE      : 1;  //LIN break detection interrupt enable
    u1 RSVD1      : 1;  //Reserved
    u1 LBCL       : 1;  //Last bit clock pulse
    u1 CPHA       : 1;  //Clock phase
    u1 CPOL       : 1;  //Clock polarity
    u1 CLKEN      : 1;  //Clock enable
    u1 STOP       : 2;  //STOP bits
    u1 LINEN      : 1;  //LIN mode enable
    u1 SWAP       : 1;  //Swap TX/RX pins
    u1 RXINV      : 1;  //RX pin active level inversion
    u1 TXINV      : 1;  //TX pin active level inversion
    u1 DATAINV    : 1;  //Binary data inversion
    u1 MSBFIRST   : 1;  //Most significant bit first
    u1 ABREN      : 1;  //Auto baud rate enable
    u1 ABRMOD     : 2;  //Auto baud rate mode
    u1 RTOEN      : 1;  //Receiver timeout enable
    u1 ADD        : 8;  //Address of the USART node
  }volatile CR2;              //Control register 2 (USARTx_CR2)
  struct
  {
    u1 EIE        : 1;  //Error interrupt enable
    u1 IREN       : 1;  //IrDA mode enable
    u1 IRLP       : 1;  //IrDA low-power
    u1 HDSEL      : 1;  //Half-duplex selection
    u1 NACK       : 1;  //Smartcard NACK enable
    u1 SCEN       : 1;  //Smartcard mode enable
    u1 DMAR       : 1;  //DMA enable receiver
    u1 DMAT       : 1;  //DMA enable transmitter
    u1 RTSE       : 1;  //RTS enable
    u1 CTSE       : 1;  //CTS enable
    u1 CTSIE      : 1;  //CTS interrupt enable
    u1 ONEBIT     : 1;  //One sample bit method enable
    u1 OVRDIS     : 1;  //Overrun Disable
    u1 DDRE       : 1;  //DMA Disable on Reception Error
    u1 DEM        : 1;  //Driver enable mode
    u1 DEP        : 1;  //Driver enable polarity selection
    u1 RSVD0      : 1;  //Reserved
    u1 SCARCNT    : 3;  //Smartcard auto-retry count
    u1 WUS        : 2;  //Wakeup from Stop mode interrupt flag selection
    u1 WUFIE      : 1;  //Wakeup from Stop mode interrupt enable
    u1 RSVD1      : 9;  //Reserved
  }volatile CR3;              //Control register 3 (USARTx_CR3)
  struct
  {
    u1 DIV        : 16; //4 fraction of USARTDIV & 12 mantissa of USARTDIV
    u1 RSVD0      : 16; //Reserved
  }volatile BRR;              //Baud rate register (USART_BRR)
  struct
  {
    u1 PSC        : 8;  //Prescaler value
    u1 GT         : 8;  //Guard time value
    u1 RSVD0      : 16; //Reserved
  }volatile GTPR;             //Guard time and prescaler register (USART_GTPR)
  struct
  {
    u1 RTO        : 24; //Receiver timeout value
    u1 BLEN       : 8;  //Block Length
  }volatile RTOR;             //Receiver timeout register (USARTx_RTOR)
  struct
  {
    u1 ABRRQ      : 1;  //Auto baud rate request
    u1 SBKRQ      : 1;  //Send break request
    u1 MMRQ       : 1;  //Mute mode request
    u1 RXFRQ      : 1;  //Receive data flush request
    u1 TXFRQ      : 1;  //Transmit data flush request
    u1 RSVD0      : 27; //Reserved
  }volatile RQR;              //Request register (USARTx_RQR)
  struct
  {
    u1 PE         : 1;  //Parity error
    u1 FE         : 1;  //Framing error
    u1 NF         : 1;  //START bit Noise detection flag
    u1 ORE        : 1;  //Overrun error
    u1 IDLE       : 1;  //IDLE line detected
    u1 RXNE       : 1;  //Read data register not empty
    u1 TC         : 1;  //Transmission complete
    u1 TXE        : 1;  //Transmit data register empty
    u1 LBDF       : 1;  //LIN break detection flag
    u1 CTSIF      : 1;  //CTS interrupt flag
    u1 CTS        : 1;  //CTS flag
    u1 RTOF       : 1;  //Receiver timeout
    u1 EOBF       : 1;  //End of block flag
    u1 RSVD0      : 1;  //Reserved
    u1 ABRE       : 1;  //Auto baud rate error
    u1 ABRF       : 1;  //Auto baud rate flag
    u1 BUSY       : 1;  //Busy flag
    u1 CMF        : 1;  //Character match flag
    u1 SBKF       : 1;  //Send break flag
    u1 RWU        : 1;  //Receiver wakeup from Mute mode
    u1 WUF        : 1;  //Wakeup from Stop mode flag
    u1 TEACK      : 1;  //Transmit enable acknowledge flag
    u1 REACK      : 1;  //Receive enable acknowledge flag
    u1 RSVD1      : 9;  //Reserved
  }volatile ISR;              //Interrupt and status register (USARTx_ISR)
  struct
  {
    u1 PECF       : 1;  //Parity error clear flag
    u1 FECF       : 1;  //Framing error clear flag
    u1 NCF        : 1;  //Noise detected clear flag
    u1 ORECF      : 1;  //Overrun error clear flag
    u1 IDLECF     : 1;  //Idle line detected clear flag
    u1 RSVD0      : 1;  //Reserved
    u1 TCCF       : 1;  //Transmission complete clear flag
    u1 RSVD1      : 1;  //Reserved
    u1 LBDCF      : 1;  //LIN break detection clear flag
    u1 CTSCF      : 1;  //CTS clear flag
    u1 RSVD2      : 1;  //Reserved
    u1 RTOCF      : 1;  //Receiver timeout clear flag
    u1 EOBCF      : 1;  //End of block clear flag
    u1 RSVD3      : 4;  //Reserved
    u1 CMCF       : 1;  //Character match clear flag
    u1 RSVD4      : 2;  //Reserved
    u1 WUCF       : 1;  //Wakeup from Stop mode clear flag
    u1 RSVD5      : 11; //Reserved
  }volatile ICR;              //Interrupt flag clear register (USARTx_ICR)
  struct
  {
    u1 RDR        : 8;  //Receive data value
    u1 RSVD0      : 24; //Reserved
  }volatile RDR;              //Receive data register (USARTx_RDR)
  struct
  {
    u1 TDR        : 8;  //Transmit data value
    u1 RSVD0      : 24; //Reserved
  }volatile TDR;              //Transmit data register (USARTx_TDR)
}UART_Type;
#define UART1 ((UART_Type*) UART1_BASE)
#define UART2 ((UART_Type*) UART2_BASE)
#define UART3 ((UART_Type*) UART3_BASE)
#define UART4 ((UART_Type*) UART4_BASE)
#define UART5 ((UART_Type*) UART5_BASE)
#define UART6 ((UART_Type*) UART6_BASE)
#define UART7 ((UART_Type*) UART7_BASE)
#define UART8 ((UART_Type*) UART8_BASE)
#define UART_EN(PORT)           PORT->CR1.UE                    //USART enable
#define UART_ENSM(PORT)         PORT->CR1.UESM                  //USART enable in Stop mode
#define UART_RX_EN(PORT)        PORT->CR1.RE                    //Receiver enable
#define UART_TX_EN(PORT)        PORT->CR1.TE                    //Transmitter enable
#define UART_ID_IRQ_EN(PORT)    PORT->CR1.IDLEIE                //IDLE interrupt enable
#define UART_RX_IRQ_EN(PORT)    PORT->CR1.RXNEIE                //RXNE interrupt enable
#define UART_TC_IRQ_EN(PORT)    PORT->CR1.TCIE                  //Transmission complete interrupt enable
#define UART_TX_IRQ_EN(PORT)    PORT->CR1.TXEIE                 //TX interrupt enable
#define UART_PE_IRQ_EN(PORT)    PORT->CR1.PEIE                  //PE interrupt enable
#define UART_PS(PORT)           PORT->CR1.PS                    //Parity selection
#define UART_PCE(PORT)          PORT->CR1.PCE                   //Parity control enable
#define UART_WAKE(PORT)         PORT->CR1.WAKE                  //Receiver wakeup method
#define UART_DB(PORT)           PORT->CR1.M0                    //Word length
#define UART_MMEN(PORT)         PORT->CR1.MME                   //Mute mode enable
#define UART_CMIE(PORT)         PORT->CR1.CMIE                  //Character match interrupt enable
#define UART_OVER8(PORT)        PORT->CR1.OVER8                 //Oversampling mode
#define UART_DEDT(PORT)         PORT->CR1.DEDT                  //Driver Enable de-assertion time
#define UART_DEAT(PORT)         PORT->CR1.DEAT                  //Driver Enable assertion time
#define UART_RTOIE(PORT)        PORT->CR1.RTOIE                 //Receiver timeout interrupt enable
#define UART_EOBIE(PORT)        PORT->CR1.EOBIE                 //End of Block interrupt enable
#define UART_LENGHT(PORT)       PORT->CR1.M1                    //Word length
#define UART_ADDM7(PORT)        PORT->CR2.ADDM7                 //7-bit Address Detection/4-bit Address Detection
#define UART_LBDL(PORT)         PORT->CR2.LBDL                  //LIN break detection length
#define UART_LBDIE(PORT)        PORT->CR2.LBDIE                 //LIN break detection interrupt enable
#define UART_LBCL(PORT)         PORT->CR2.LBCL                  //Last bit clock pulse
#define UART_CPHA(PORT)         PORT->CR2.CPHA                  //Clock phase
#define UART_CPOL(PORT)         PORT->CR2.CPOL                  //Clock polarity
#define UART_CLKEN(PORT)        PORT->CR2.CLKEN                 //Clock enable
#define UART_STOP(PORT)         PORT->CR2.STOP                  //STOP bits
#define UART_LINEN(PORT)        PORT->CR2.LINEN                 //LIN mode enable
#define UART_SWAP(PORT)         PORT->CR2.SWAP                  //Swap TX/RX pins
#define UART_RXINV(PORT)        PORT->CR2.RXINV                 //RX pin active level inversion
#define UART_TXINV(PORT)        PORT->CR2.TXINV                 //TX pin active level inversion
#define UART_DATAINV(PORT)      PORT->CR2.DATAINV               //Binary data inversion
#define UART_MSBFIRST(PORT)     PORT->CR2.MSBFIRST              //Most significant bit first
#define UART_ABREN(PORT)        PORT->CR2.ABREN                 //Auto baud rate enable
#define UART_ABRMOD(PORT)       PORT->CR2.ABRMOD                //Auto baud rate mode
#define UART_RTOEN(PORT)        PORT->CR2.RTOEN                 //Receiver timeout enable
#define UART_ADD(PORT)          PORT->CR2.ADD                   //Address of the USART node
#define UART_EIE(PORT)          PORT->CR3.EIE                   //Error interrupt enable
#define UART_IREN(PORT)         PORT->CR3.IREN                  //IrDA mode enable
#define UART_IRLP(PORT)         PORT->CR3.IRLP                  //IrDA low-power
#define UART_HDSEL(PORT)        PORT->CR3.HDSEL                 //Half-duplex selection
#define UART_NACK(PORT)         PORT->CR3.NACK                  //Smartcard NACK enable
#define UART_SCEN(PORT)         PORT->CR3.SCEN                  //Smartcard mode enable
#define UART_DMAR(PORT)         PORT->CR3.DMAR                  //DMA enable receiver
#define UART_DMAT(PORT)         PORT->CR3.DMAT                  //DMA enable transmitter
#define UART_RTSE(PORT)         PORT->CR3.RTSE                  //RTS enable
#define UART_CTSE(PORT)         PORT->CR3.CTSE                  //CTS enable
#define UART_CTSIE(PORT)        PORT->CR3.CTSIE                 //CTS interrupt enable
#define UART_ONEBIT(PORT)       PORT->CR3.ONEBIT                //One sample bit method enable
#define UART_OVRDIS(PORT)       PORT->CR3.OVRDIS                //Overrun Disable
#define UART_DDRE(PORT)         PORT->CR3.DDRE                  //DMA Disable on Reception Error
#define UART_DEM(PORT)          PORT->CR3.DEM                   //Driver enable mode
#define UART_DEP(PORT)          PORT->CR3.DEP                   //Driver enable polarity selection
#define UART_SCARCNT(PORT)      PORT->CR3.SCARCNT               //Smartcard auto-retry count
#define UART_WUS(PORT)          PORT->CR3.WUS                   //Wakeup from Stop mode interrupt flag selection
#define UART_WUFIE(PORT)        PORT->CR3.WUFIE                 //Wakeup from Stop mode interrupt enable
#define UART_DIV(PORT)          PORT->BRR.DIV                   //4 fraction of USARTDIV & 12 mantissa of USARTDIV
#define UART_PSC(PORT)          PORT->GTPR.PSC                  //Prescaler value
#define UART_GT(PORT)           PORT->GTPR.GT                   //Guard time value
#define UART_RTO(PORT)          PORT->RTOR.RTO                  //Receiver timeout value
#define UART_BLEN(PORT)         PORT->RTOR.BLEN                 //Block Length
#define UART_ABRRQ(PORT)        PORT->RQR.ABRRQ                 //Auto baud rate request
#define UART_SBKRQ(PORT)        PORT->RQR.SBKRQ                 //Send break request
#define UART_MMRQ(PORT)         PORT->RQR.MMRQ                  //Mute mode request
#define UART_RXFRQ(PORT)        PORT->RQR.RXFRQ                 //Receive data flush request
#define UART_TXFRQ(PORT)        PORT->RQR.TXFRQ                 //Transmit data flush request
#define UART_PE_FLG(PORT)       PORT->ISR.PE                    //Parity error
#define UART_FE_FLG(PORT)       PORT->ISR.FE                    //Framing error
#define UART_NE_FLG(PORT)       PORT->ISR.NF                    //START bit Noise detection flag
#define UART_ORE_FLG(PORT)      PORT->ISR.ORE                   //Overrun error
#define UART_IDLE_FLG(PORT)     PORT->ISR.IDLE                  //IDLE line detected
#define UART_RXNE_FLG(PORT)     PORT->ISR.RXNE                  //Read data register not empty
#define UART_TC_FLG(PORT)       PORT->ISR.TC                    //Transmission complete
#define UART_TXE_FLG(PORT)      PORT->ISR.TXE                   //Transmit data register empty
#define UART_LBD_FLG(PORT)      PORT->ISR.LBDF                  //LIN break detection flag
#define UART_CTSI_FLG(PORT)     PORT->ISR.CTSIF                 //CTS interrupt flag
#define UART_CTS_FLG(PORT)      PORT->ISR.CTS                   //CTS flag
#define UART_RTO_FLG(PORT)      PORT->ISR.RTOF                  //Receiver timeout
#define UART_EOB_FLG(PORT)      PORT->ISR.EOBF                  //End of block flag
#define UART_ABRE_FLG(PORT)     PORT->ISR.ABRE                  //Auto baud rate error
#define UART_ABR_FLG(PORT)      PORT->ISR.ABRF                  //Auto baud rate flag
#define UART_BUSY_FLG(PORT)     PORT->ISR.BUSY                  //Busy flag
#define UART_CM_FLG(PORT)       PORT->ISR.CMF                   //Character match flag
#define UART_SBK_FLG(PORT)      PORT->ISR.SBKF                  //Send break flag
#define UART_RW_FLG(PORT)       PORT->ISR.RWU                   //Receiver wakeup from Mute mode
#define UART_WU_FLG(PORT)       PORT->ISR.WUF                   //Wakeup from Stop mode flag
#define UART_TX_FLG(PORT)       PORT->ISR.TEACK                 //Transmit enable acknowledge flag
#define UART_RX_FLG(PORT)       PORT->ISR.REACK                 //Receive enable acknowledge flag
#define UART_PE_CLR(PORT)       PORT->ICR.PECF = ON             //Parity error clear flag
#define UART_FE_CLR(PORT)       PORT->ICR.FECF = ON             //Framing error clear flag
#define UART_NE_CLR(PORT)       PORT->ICR.NCF = ON              //Noise detected clear flag
#define UART_ORE_CLR(PORT)      PORT->ICR.ORECF = ON            //Overrun error clear flag
#define UART_IDLE_CLR(PORT)     PORT->ICR.IDLECF = ON           //Idle line detected clear flag
#define UART_TC_CLR(PORT)       PORT->ICR.TCCF = ON             //Transmission complete clear flag
#define UART_LBD_CLR(PORT)      PORT->ICR.LBDCF = ON            //LIN break detection clear flag
#define UART_CTS_CLR(PORT)      PORT->ICR.CTSCF = ON            //CTS clear flag
#define UART_RTO_CLR(PORT)      PORT->ICR.RTOCF = ON            //Receiver timeout clear flag
#define UART_EOB_CLR(PORT)      PORT->ICR.EOBCF = ON            //End of block clear flag
#define UART_CM_CLR(PORT)       PORT->ICR.CMCF = ON             //Character match clear flag
#define UART_WU_CLR(PORT)       PORT->ICR.WUCF = ON             //Wakeup from Stop mode clear flag
#define UART_RXDATA(PORT)       PORT->RDR.RDR                   //Receive data value
#define UART_TXDATA(PORT)       PORT->TDR.TDR                   //Transmit data value
#define UART_STOP_10            ((u8) 0x00)                     //1 Stop bit
#define UART_STOP_05            ((u8) 0x01)                     //0.5 Stop bit
#define UART_STOP_20            ((u8) 0x02)                     //2 Stop bit
#define UART_STOP_15            ((u8) 0x03)                     //1.5 Stop bit
/******************************************** WWDG ************************************************/
typedef struct
{
  struct
  {
    u1 T          : 7;  //7-bit counter (MSB to LSB)
    u1 WDGA       : 1;  //Activation bit
    u1 RSVD0      : 24; //Reserved
  }volatile CR;               //Control register (WWDG_CR)
  struct
  {
    u1 W          : 7;  //7-bit window value
    u1 WDGTB      : 2;  //Timer base
    u1 EWI        : 1;  //Early wakeup interrupt
    u1 RSVD0      : 22; //Reserved
  }volatile CFR;              //Configuration register (WWDG_CFR)
  struct
  {
    u1 EWIF       : 1;  //Early wakeup interrupt flag
    u1 RSVD0      : 31; //Reserved
  }volatile SR;               //Status register (WWDG_SR)
}WWDG_Type;
#define WWDG ((WWDG_Type*) WWDG_BASE)
#define WWDG_COUNT              WWDG->CR.T                      //7-bit counter (MSB to LSB)
#define WWDG_EN                 WWDG->CR.WDGA                   //Activation bit
#define WWDG_VALUE              WWDG->CFR.W                     //7-bit window value
#define WWDG_DIV                WWDG->CFR.WDGTB                 //Timer base
#define WWDG_HALF_IRQ_EN        WWDG->CFR.EWI                   //Early wakeup interrupt
#define WWDG_HALF_IRQ_FLG       WWDG->SR.EWIF                   //Early wakeup interrupt flag
#define WWDG_DIV1               ((u8) 0x00)                     // CK Counter Clock (PCLK1 div 4096) div 1
#define WWDG_DIV2               ((u8) 0x01)                     // CK Counter Clock (PCLK1 div 4096) div 2
#define WWDG_DIV4               ((u8) 0x02)                     // CK Counter Clock (PCLK1 div 4096) div 4
#define WWDG_DIV8               ((u8) 0x03)                     // CK Counter Clock (PCLK1 div 4096) div 8
/******************************************** DAC *************************************************/
typedef struct
{
  struct
  {
    u1 EN1        : 1;  //DAC channel1 enable
    u1 BOFF1      : 1;  //DAC channel1 output buffer disable
    u1 TEN1       : 1;  //DAC channel1 trigger enable
    u1 TSEL1      : 3;  //DAC channel1 trigger selection
    u1 WAVE1      : 2;  //DAC channel1 noise/triangle wave generation enable
    u1 MAMP1      : 4;  //DAC channel1 mask/amplitude selector
    u1 DMAEN1     : 1;  //DAC channel1 DMA enable
    u1 DMAUDRIE1  : 1;  //DAC channel1 DMA Underrun Interrupt enable
    u1 RSVD0      : 2;  //Reserved
    u1 EN2        : 1;  //DAC channel2 enable
    u1 BOFF2      : 1;  //DAC channel2 output buffer disable
    u1 TEN2       : 1;  //DAC channel2 trigger enable
    u1 TSEL2      : 3;  //DAC channel2 trigger selection
    u1 WAVE2      : 2;  //DAC channel2 noise/triangle wave generation enable
    u1 MAMP2      : 4;  //DAC channel2 mask/amplitude selector
    u1 DMAEN2     : 1;  //DAC channel2 DMA enable
    u1 DMAUDRIE2  : 1;  //DAC channel2 DMA Underrun Interrupt enable
    u1 RSVD1      : 2;  //Reserved
  }volatile CR;               //DAC control register (DAC_CR)
  struct
  {
    u1 SWTRIG1    : 1;  //DAC channel1 software trigger
    u1 SWTRIG2    : 1;  //DAC channel2 software trigger
    u1 RSVD0      : 30; //Reserved
  }volatile SWTRIGR;          //DAC software trigger register (DAC_SWTRIGR)
  struct
  {
    u1 DACC1DHR   : 12; //DAC channel1 12-bit right-aligned data
    u1 RSVD0      : 20; //Reserved
  }volatile DHR12R1;          //DAC channel1 12-bit right-aligned data holding register (DAC_DHR12R1)
  struct
  {
    u1 RSVD0      : 4;  //Reserved
    u1 DACC1DHR   : 12; //DAC channel1 12-bit right-aligned data
    u1 RSVD1      : 16; //Reserved
  }volatile DHR12L1;          //DAC channel1 12-bit left-aligned data holding register (DAC_DHR12L1)
  struct
  {
    u1 DACC1DHR   : 8;  //DAC channel1 8-bit right-aligned data
    u1 RSVD0      : 24; //Reserved
  }volatile DHR8R1;           //DAC channel1 8-bit right-aligned data holding register (DAC_DHR8R1)
  struct
  {
    u1 DACC2DHR   : 12; //DAC channel2 12-bit left-aligned data
    u1 RSVD0      : 20; //Reserved
  }volatile DHR12R2;          //DAC channel2 12-bit right-aligned data holding register (DAC_DHR12R2)
  struct
  {
    u1 DACC2DHR   : 8;  //DAC channel2 8-bit right-aligned data
    u1 RSVD0      : 24; //Reserved
  }volatile DHR8R2;           //DAC channel2 8-bit right-aligned data holding register (DAC_DHR8R2)
  struct
  {
    u1 DACC1DHR   : 12; //DAC channel1 12-bit right-aligned data
    u1 RSVD0      : 4;  //Reserved
    u1 DACC2DHR   : 12; //DAC channel2 12-bit right-aligned data
    u1 RSVD1      : 4;  //Reserved
  }volatile DHR12RD;          //Dual DAC 12-bit right-aligned data holding register (DAC_DHR12RD)
  struct
  {
    u1 RSVD0      : 4;  //Reserved
    u1 DACC1DHR   : 12; //DAC channel1 12-bit left-aligned data
    u1 RSVD1      : 4;  //Reserved
    u1 DACC2DHR   : 12; //DAC channel2 12-bit left-aligned data
  }volatile DHR12LD;          //Dual DAC 12-bit left-aligned data holding register (DAC_DHR12LD)
  struct
  {
    u1 DACC1DHR   : 8;  //DAC channel1 8-bit right-aligned data
    u1 DACC2DHR   : 8;  //DAC channel2 8-bit right-aligned data
    u1 RSVD1      : 16; //Reserved
  }volatile DHR8RD;           //Dual DAC 8-bit right-aligned data holding register (DAC_DHR8RD)
  struct
  {
    u1 DACC1DOR   : 12; //DAC channel1 data output
    u1 RSVD0      : 20; //Reserved
  }volatile DOR1;             //DAC channel1 data output register (DAC_DOR1)
  struct
  {
    u1 DACC2DOR   : 12; //DAC channel2 data output
    u1 RSVD0      : 20; //Reserved
  }volatile DOR2;             //DAC channel2 data output register (DAC_DOR2)
  struct
  {
    u1 RSVD0      : 13; //Reserved
    u1 DMAUDR1    : 1;  //DAC channel1 DMA underrun flag
    u1 RSVD1      : 15; //Reserved
    u1 DMAUDR2    : 1;  //DAC channel2 DMA underrun flag
    u1 RSVD2      : 2;  //Reserved
  }volatile SR;               //DAC status register (DAC_SR)
}DAC_Type;
#define DAC ((DAC_Type*) DAC_BASE)
#define DAC_EN1                 DAC->CR.EN1                     //DAC channel1 enable
#define DAC_BOFF1               DAC->CR.BOFF1                   //DAC channel1 output buffer disable
#define DAC_TEN1                DAC->CR.TEN1                    //DAC channel1 trigger enable
#define DAC_TSEL1               DAC->CR.TSEL1                   //DAC channel1 trigger selection
#define DAC_WAVE1               DAC->CR.WAVE1                   //DAC channel1 noise/triangle wave generation enable
#define DAC_MAMP1               DAC->CR.MAMP1                   //DAC channel1 mask/amplitude selector
#define DAC_DMAEN1              DAC->CR.DMAEN1                  //DAC channel1 DMA enable
#define DAC_DMAUDRIE1           DAC->CR.DMAUDRIE1               //DAC channel1 DMA Underrun Interrupt enable
#define DAC_EN2                 DAC->CR.EN2                     //DAC channel2 enable
#define DAC_BOFF2               DAC->CR.BOFF2                   //DAC channel2 output buffer disable
#define DAC_TEN2                DAC->CR.TEN2                    //DAC channel2 trigger enable
#define DAC_TSEL2               DAC->CR.TSEL2                   //DAC channel2 trigger selection
#define DAC_WAVE2               DAC->CR.WAVE2                   //DAC channel2 noise/triangle wave generation enable
#define DAC_MAMP2               DAC->CR.MAMP2                   //DAC channel2 mask/amplitude selector
#define DAC_DMAEN2              DAC->CR.DMAEN2                  //DAC channel2 DMA enable
#define DAC_DMAUDRIE2           DAC->CR.DMAUDRIE2               //DAC channel2 DMA Underrun Interrupt enable
#define DAC_SWTRIG1             DAC->SWTRIGR.SWTRIG1            //DAC channel1 software trigger
#define DAC_SWTRIG2             DAC->SWTRIGR.SWTRIG2            //DAC channel2 software trigger
#define DAC_DACC1DHR            DAC->DHR12R1.DACC1DHR           //DAC channel1 12-bit right-aligned data
#define DAC_DACC1DHR8           DAC->DHR12L1.DACC1DHR           //DAC channel1 12-bit right-aligned data
#define DAC_DACC1DHR9           DAC->DHR8R1.DACC1DHR            //DAC channel1 8-bit right-aligned data
#define DAC_DACC2DHR            DAC->DHR12R2.DACC2DHR           //DAC channel2 12-bit left-aligned data
#define DAC_DACC2DHR1           DAC->DHR8R2.DACC2DHR            //DAC channel2 8-bit right-aligned data
#define DAC_DACC1DHR2           DAC->DHR12RD.DACC1DHR           //DAC channel1 12-bit right-aligned data
#define DAC_DACC2DHR3           DAC->DHR12RD.DACC2DHR           //DAC channel2 12-bit right-aligned data
#define DAC_DACC1DHR4           DAC->DHR12LD.DACC1DHR           //DAC channel1 12-bit left-aligned data
#define DAC_DACC2DHR5           DAC->DHR12LD.DACC2DHR           //DAC channel2 12-bit left-aligned data
#define DAC_DACC1DHR6           DAC->DHR8RD.DACC1DHR            //DAC channel1 8-bit right-aligned data
#define DAC_DACC2DHR7           DAC->DHR8RD.DACC2DHR            //DAC channel2 8-bit right-aligned data
#define DAC_DACC1DOR            DAC->DOR1.DACC1DOR              //DAC channel1 data output
#define DAC_DACC2DOR            DAC->DOR2.DACC2DOR              //DAC channel2 data output
#define DAC_DMAUDR1             DAC->SR.DMAUDR1                 //DAC channel1 DMA underrun flag
#define DAC_DMAUDR2             DAC->SR.DMAUDR2                 //DAC channel2 DMA underrun flag
/******************************************** TSC *************************************************/
typedef struct
{
  struct
  {
    u1 TSCE       : 1;  //Touch sensing controller enable
    u1 START      : 1;  //Start a new acquisition
    u1 AM         : 1;  //Acquisition mode
    u1 SYNCPOL    : 1;  //Synchronization pin polarity
    u1 IODEF      : 1;  //I/O Default mode
    u1 MCV        : 3;  //Max count value
    u1 RSVD0      : 4;  //Reserved
    u1 PGPSC      : 3;  //pulse generator prescaler
    u1 SSPSC      : 1;  //Spread spectrum prescaler
    u1 SSE        : 1;  //Spread spectrum enable
    u1 SSD        : 7;  //Spread spectrum deviation
    u1 CTPL       : 4;  //Charge transfer pulse low
    u1 CTPH       : 4;  //Charge transfer pulse high
  }volatile CR;               //TSC control register (TSC_CR)
  struct
  {
    u1 EOAIE      : 1;  //End of acquisition interrupt enable
    u1 MCEIE      : 1;  //Max count error interrupt enable
    u1 RSVD0      : 30; //Reserved
  }volatile IER;              //TSC interrupt enable register (TSC_IER)
  struct
  {
    u1 EOAIC      : 1;  //End of acquisition interrupt clear
    u1 MCEIC      : 1;  //Max count error interrupt clear
    u1 RSVD0      : 30; //Reserved
  }volatile ICR;              //TSC interrupt clear register (TSC_ICR)
  struct
  {
    u1 EOAF       : 1;  //End of acquisition flag
    u1 MCEF       : 1;  //Max count error flag
    u1 RSVD0      : 30; //Reserved
  }volatile ISR;              //TSC interrupt status register (TSC_ISR)
  volatile u32 IOHCR;         //TSC I/O hysteresis control register (TSC_IOHCR)
  volatile u32 IOASCR;        //TSC I/O analog switch control register (TSC_IOASCR)
  volatile u32 IOSCR;         //TSC I/O sampling control register (TSC_IOSCR)
  volatile u32 IOCCR;         //TSC I/O channel control register (TSC_IOCCRTSC_IOCCR)
  struct
  {
    u1 GxE        : 8;  //Analog I/O group x enable
    u1 RSVD0      : 8;  //Reserved
    u1 GxS        : 8;  //Analog I/O group x status
    u1 RSVD1      : 8;  //Reserved
  }volatile IOGCSR;           //TSC I/O group control status register (TSC_IOGCSR)
  struct
  {
    u1 CNT        : 14; //Counter value
    u1 RSVD0      : 18; //Reserved
  }volatile IOGCR[8];         //TSC I/O group x counter register (TSC_IOGxCR)
}TSC_Type;
#define TSC ((TSC_Type*) TSC_BASE)
#define TSC_TSCE                TSC->CR.TSCE                    //Touch sensing controller enable
#define TSC_START               TSC->CR.START                   //Start a new acquisition
#define TSC_AM                  TSC->CR.AM                      //Acquisition mode
#define TSC_SYNCPOL             TSC->CR.SYNCPOL                 //Synchronization pin polarity
#define TSC_IODEF               TSC->CR.IODEF                   //I/O Default mode
#define TSC_MCV                 TSC->CR.MCV                     //Max count value
#define TSC_PGPSC               TSC->CR.PGPSC                   //pulse generator prescaler
#define TSC_SSPSC               TSC->CR.SSPSC                   //Spread spectrum prescaler
#define TSC_SSE                 TSC->CR.SSE                     //Spread spectrum enable
#define TSC_SSD                 TSC->CR.SSD                     //Spread spectrum deviation
#define TSC_CTPL                TSC->CR.CTPL                    //Charge transfer pulse low
#define TSC_CTPH                TSC->CR.CTPH                    //Charge transfer pulse high
#define TSC_EOAIE               TSC->IER.EOAIE                  //End of acquisition interrupt enable
#define TSC_MCEIE               TSC->IER.MCEIE                  //Max count error interrupt enable
#define TSC_EOAIC               TSC->ICR.EOAIC                  //End of acquisition interrupt clear
#define TSC_MCEIC               TSC->ICR.MCEIC                  //Max count error interrupt clear
#define TSC_EOAF                TSC->ISR.EOAF                   //End of acquisition flag
#define TSC_MCEF                TSC->ISR.MCEF                   //Max count error flag
#define TSC_IOHCR               TSC->IOHCR                      //TSC I/O hysteresis control register (TSC_IOHCR)
#define TSC_IOASCR              TSC->IOASCR                     //TSC I/O analog switch control register (TSC_IOASCR)
#define TSC_IOSCR               TSC->IOSCR                      //TSC I/O sampling control register (TSC_IOSCR)
#define TSC_IOCCR               TSC->IOCCR                      //TSC I/O channel control register (TSC_IOCCRTSC_IOCCR)
#define TSC_GxE                 TSC->IOGCSR.GxE                 //Analog I/O group x enable
#define TSC_GxS                 TSC->IOGCSR.GxS                 //Analog I/O group x status
#define TSC_(GR)                TSC->IOGCR[GR].CNT              //Counter value
/******************************************** CEC *************************************************/
typedef struct
{
  struct
  {
    u1 CECEN      : 1;  //CEC Enable
    u1 TXSOM      : 1;  //Tx Start Of Message
    u1 TXEOM      : 1;  //Tx End Of Message
    u1 RSVD0      : 29; //Reserved
  }volatile CR;               //CEC control register (CEC_CR)
  struct
  {
    u1 SFT        : 3;  //Signal Free Time
    u1 RXTOL      : 1;  //Rx-Tolerance
    u1 BRESTP     : 1;  //Rx-Stop on Bit Rising Error
    u1 BREGEN     : 1;  //Generate Error-Bit on Bit Rising Error
    u1 LBPEGEN    : 1;  //Generate Error-Bit on Long Bit Period Error
    u1 BRDNOGEN   : 1;  //Avoid Error-Bit Generation in Broadcast
    u1 SFTOP      : 1;  //SFT Option Bit
    u1 RSVD0      : 7;  //Reserved
    u1 OAR        : 15; //Own addresses configuration
    u1 LSTN       : 1;  //Listen mode
  }volatile CFGR;             //CEC configuration register (CEC_CFGR)
  struct
  {
    u1 TXD        : 8;  //Tx Data register
    u1 RSVD0      : 24; //Reserved
  }volatile TXDR;             //CEC Tx data register (CEC_TXDR)
  struct
  {
    u1 RXD        : 8;  //Rx Data register
    u1 RSVD0      : 24; //Reserved
  }volatile RXDR;             //CEC Rx Data Register (CEC_RXDR)
  struct
  {
    u1 RXBR       : 1;  //Rx-Byte Received
    u1 RXEND      : 1;  //End Of Reception
    u1 RXOVR      : 1;  //Rx-Overrun
    u1 BRE        : 1;  //Rx-Bit Rising Error
    u1 SBPE       : 1;  //Rx-Short Bit Period Error
    u1 LBPE       : 1;  //Rx-Long Bit Period Error
    u1 RXACKE     : 1;  //Rx-Missing Acknowledge
    u1 ARBLST     : 1;  //Arbitration Lost
    u1 TXBR       : 1;  //Tx-Byte Request
    u1 TXEND      : 1;  //End of Transmission
    u1 TXUDR      : 1;  //Tx-Buffer Underrun
    u1 TXERR      : 1;  //Tx-Error
    u1 TXACKE     : 1;  //Tx-Missing Acknowledge Error
    u1 RSVD0      : 19; //Reserved
  }volatile ISR;              //CEC Interrupt and Status Register (CEC_ISR)
  struct
  {
    u1 RXBRIE     : 1;  //Rx-Byte Received Interrupt Enable
    u1 RXENDIE    : 1;  //End Of Reception Interrupt Enable
    u1 RXOVRIE    : 1;  //Rx-Overrun Interrupt Enable
    u1 BREIE      : 1;  //Rx-Bit Rising Error Interrupt Enable
    u1 SBPEIE     : 1;  //Rx-Short Bit Period Error Interrupt Enable
    u1 LBPEIE     : 1;  //Rx-Long Bit Period Error Interrupt Enable
    u1 RXACKEIE   : 1;  //Rx-Missing Acknowledge Interrupt Enable
    u1 ARBLSTIE   : 1;  //Arbitration Lost Interrupt Enable
    u1 TXBRIE     : 1;  //Tx-Byte Request Interrupt Enable
    u1 TXENDIE    : 1;  //End of Transmission Interrupt Enable
    u1 TXUDRIE    : 1;  //Tx-Buffer Underrun Interrupt Enable
    u1 TXERRIE    : 1;  //Tx-Error Interrupt Enable
    u1 TXACKEIE   : 1;  //Tx-Missing Acknowledge Error Interrupt Enable
    u1 RSVD0      : 19; //Reserved
  }volatile IER;              //CEC interrupt enable register (CEC_IER)
}CEC_Type;
#define CEC ((CEC_Type*) CEC_BASE)
#define CEC_CECEN               CEC->CR.CECEN                   //CEC Enable
#define CEC_TXSOM               CEC->CR.TXSOM                   //Tx Start Of Message
#define CEC_TXEOM               CEC->CR.TXEOM                   //Tx End Of Message
#define CEC_SFT                 CEC->CFGR.SFT                   //Signal Free Time
#define CEC_RXTOL               CEC->CFGR.RXTOL                 //Rx-Tolerance
#define CEC_BRESTP              CEC->CFGR.BRESTP                //Rx-Stop on Bit Rising Error
#define CEC_BREGEN              CEC->CFGR.BREGEN                //Generate Error-Bit on Bit Rising Error
#define CEC_LBPEGEN             CEC->CFGR.LBPEGEN               //Generate Error-Bit on Long Bit Period Error
#define CEC_BRDNOGEN            CEC->CFGR.BRDNOGEN              //Avoid Error-Bit Generation in Broadcast
#define CEC_SFTOP               CEC->CFGR.SFTOP                 //SFT Option Bit
#define CEC_OAR                 CEC->CFGR.OAR                   //Own addresses configuration
#define CEC_LSTN                CEC->CFGR.LSTN                  //Listen mode
#define CEC_TXD                 CEC->TXDR.TXD                   //Tx Data register
#define CEC_RXD                 CEC->RXDR.RXD                   //Rx Data register
#define CEC_RXBR                CEC->ISR.RXBR                   //Rx-Byte Received
#define CEC_RXEND               CEC->ISR.RXEND                  //End Of Reception
#define CEC_RXOVR               CEC->ISR.RXOVR                  //Rx-Overrun
#define CEC_BRE                 CEC->ISR.BRE                    //Rx-Bit Rising Error
#define CEC_SBPE                CEC->ISR.SBPE                   //Rx-Short Bit Period Error
#define CEC_LBPE                CEC->ISR.LBPE                   //Rx-Long Bit Period Error
#define CEC_RXACKE              CEC->ISR.RXACKE                 //Rx-Missing Acknowledge
#define CEC_ARBLST              CEC->ISR.ARBLST                 //Arbitration Lost
#define CEC_TXBR                CEC->ISR.TXBR                   //Tx-Byte Request
#define CEC_TXEND               CEC->ISR.TXEND                  //End of Transmission
#define CEC_TXUDR               CEC->ISR.TXUDR                  //Tx-Buffer Underrun
#define CEC_TXERR               CEC->ISR.TXERR                  //Tx-Error
#define CEC_TXACKE              CEC->ISR.TXACKE                 //Tx-Missing Acknowledge Error
#define CEC_RXBRIE              CEC->IER.RXBRIE                 //Rx-Byte Received Interrupt Enable
#define CEC_RXENDIE             CEC->IER.RXENDIE                //End Of Reception Interrupt Enable
#define CEC_RXOVRIE             CEC->IER.RXOVRIE                //Rx-Overrun Interrupt Enable
#define CEC_BREIE               CEC->IER.BREIE                  //Rx-Bit Rising Error Interrupt Enable
#define CEC_SBPEIE              CEC->IER.SBPEIE                 //Rx-Short Bit Period Error Interrupt Enable
#define CEC_LBPEIE              CEC->IER.LBPEIE                 //Rx-Long Bit Period Error Interrupt Enable
#define CEC_RXACKEIE            CEC->IER.RXACKEIE               //Rx-Missing Acknowledge Interrupt Enable
#define CEC_ARBLSTIE            CEC->IER.ARBLSTIE               //Arbitration Lost Interrupt Enable
#define CEC_TXBRIE              CEC->IER.TXBRIE                 //Tx-Byte Request Interrupt Enable
#define CEC_TXENDIE             CEC->IER.TXENDIE                //End of Transmission Interrupt Enable
#define CEC_TXUDRIE             CEC->IER.TXUDRIE                //Tx-Buffer Underrun Interrupt Enable
#define CEC_TXERRIE             CEC->IER.TXERRIE                //Tx-Error Interrupt Enable
#define CEC_TXACKEIE            CEC->IER.TXACKEIE               //Tx-Missing Acknowledge Error Interrupt Enable
/******************************************** CRS *************************************************/
typedef struct
{
  struct
  {
    u1 SYNCOKIE   : 1;  //SYNC event OK interrupt enable
    u1 SYNCWARNIE : 1;  //SYNC warning interrupt enable
    u1 ERRIE      : 1;  //Synchronization or trimming error interrupt enable
    u1 ESYNCIE    : 1;  //Expected SYNC interrupt enable
    u1 RSVD0      : 1;  //Reserved
    u1 CEN        : 1;  //Frequency error counter enable
    u1 AUTOTRIMEN : 1;  //Automatic trimming enable
    u1 SWSYNC     : 1;  //Generate software SYNC event
    u1 TRIM       : 6;  //HSI48 oscillator smooth trimming
    u1 RSVD1      : 18; //Reserved
  }volatile CR;               //CRS control register (CRS_CR)
  struct
  {
    u1 RELOAD     : 16; //Counter reload value
    u1 FELIM      : 8;  //Frequency error limit
    u1 SYNCDIV    : 3;  //SYNC divider
    u1 RSVD0      : 1;  //Reserved
    u1 SYNCSRC    : 2;  //SYNC signal source selection
    u1 RSVD1      : 1;  //Reserved
    u1 SYNCPOL    : 1;  //SYNC polarity selection
  }volatile CFGR;             //CRS configuration register (CRS_CFGR)
  struct
  {
    u1 SYNCOKF    : 1;  //SYNC event OK flag
    u1 SYNCWARNF  : 1;  //SYNC warning flag
    u1 ERRF       : 1;  //Error flag
    u1 ESYNCF     : 1;  //Expected SYNC flag
    u1 RSVD0      : 4;  //Reserved
    u1 SYNCERR    : 1;  //SYNC error
    u1 SYNCMISS   : 1;  //SYNC missed
    u1 TRIMOVF    : 1;  //Trimming overflow or underflow
    u1 RSVD1      : 4;  //Reserved
    u1 FEDIR      : 1;  //Frequency error direction
    u1 FECAP      : 16; //Frequency error capture
  }volatile ISR;              //CRS interrupt and status register (CRS_ISR)
  struct
  {
    u1 SYNCOKC    : 1;  //SYNC event OK clear flag
    u1 SYNCWARNC  : 1;  //SYNC warning clear flag
    u1 ERRC       : 1;  //Error clear flag
    u1 ESYNCC     : 1;  //Expected SYNC clear flag
    u1 RSVD0      : 28; //Reserved
  }volatile ICR;              //CRS interrupt flag clear register (CRS_ICR)
}CRS_Type;
#define CRS ((CRS_Type*) CRS_BASE)
#define CRS_SYNCOKIE            CRS->CR.SYNCOKIE                //SYNC event OK interrupt enable
#define CRS_SYNCWARNIE          CRS->CR.SYNCWARNIE              //SYNC warning interrupt enable
#define CRS_ERRIE               CRS->CR.ERRIE                   //Synchronization or trimming error interrupt enable
#define CRS_ESYNCIE             CRS->CR.ESYNCIE                 //Expected SYNC interrupt enable
#define CRS_CEN                 CRS->CR.CEN                     //Frequency error counter enable
#define CRS_AUTOTRIM_EN         CRS->CR.AUTOTRIMEN              //Automatic trimming enable
#define CRS_SWSYNC              CRS->CR.SWSYNC                  //Generate software SYNC event
#define CRS_TRIM                CRS->CR.TRIM                    //HSI48 oscillator smooth trimming
#define CRS_RELOAD              CRS->CFGR.RELOAD                //Counter reload value
#define CRS_FELIM               CRS->CFGR.FELIM                 //Frequency error limit
#define CRS_SYNCDIV             CRS->CFGR.SYNCDIV               //SYNC divider
#define CRS_SYNCSRC             CRS->CFGR.SYNCSRC               //SYNC signal source selection
#define CRS_SYNCPOL             CRS->CFGR.SYNCPOL               //SYNC polarity selection
#define CRS_SYNCOKF             CRS->ISR.SYNCOKF                //SYNC event OK flag
#define CRS_SYNCWARNF           CRS->ISR.SYNCWARNF              //SYNC warning flag
#define CRS_ERRF                CRS->ISR.ERRF                   //Error flag
#define CRS_ESYNCF              CRS->ISR.ESYNCF                 //Expected SYNC flag
#define CRS_SYNCERR             CRS->ISR.SYNCERR                //SYNC error
#define CRS_SYNCMISS            CRS->ISR.SYNCMISS               //SYNC missed
#define CRS_TRIMOVF             CRS->ISR.TRIMOVF                //Trimming overflow or underflow
#define CRS_FEDIR               CRS->ISR.FEDIR                  //Frequency error direction
#define CRS_FECAP               CRS->ISR.FECAP                  //Frequency error capture
#define CRS_SYNCOKC             CRS->ICR.SYNCOKC                //SYNC event OK clear flag
#define CRS_SYNCWARNC           CRS->ICR.SYNCWARNC              //SYNC warning clear flag
#define CRS_ERRC                CRS->ICR.ERRC                   //Error clear flag
#define CRS_ESYNCC              CRS->ICR.ESYNCC                 //Expected SYNC clear flag
#define CRS_SYNCPOL_RIS         ((u8) 0x00)                     //SYNC active on rising edge (default)
#define CRS_SYNCPOL_FAL         ((u8) 0x01)                     //SYNC active on falling edge
#define CRS_SYNCSRC_GPIO        ((u8) 0x00)                     //GPIO selected as SYNC signal source
#define CRS_SYNCSRC_LSE         ((u8) 0x01)                     //LSE selected as SYNC signal source
#define CRS_SYNCSRC_USB         ((u8) 0x02)                     //USB SOF selected as SYNC signal source (default)
#define CRS_SYNCDIV_000         ((u8) 0x00)                     //SYNC not divided (default)
#define CRS_SYNCDIV_002         ((u8) 0x01)                     //SYNC divided by 2
#define CRS_SYNCDIV_004         ((u8) 0x02)                     //SYNC divided by 4
#define CRS_SYNCDIV_008         ((u8) 0x03)                     //SYNC divided by 8
#define CRS_SYNCDIV_016         ((u8) 0x04)                     //SYNC divided by 16
#define CRS_SYNCDIV_032         ((u8) 0x05)                     //SYNC divided by 32
#define CRS_SYNCDIV_064         ((u8) 0x06)                     //SYNC divided by 64
#define CRS_SYNCDIV_128         ((u8) 0x07)                     //SYNC divided by 128
/******************************************** CAN *************************************************/
typedef struct
{
  struct
  {
    u1 INRQ       : 1;  //Initialization request
    u1 SLEEP      : 1;  //Sleep mode request
    u1 TXFP       : 1;  //Transmit FIFO priority
    u1 RFLM       : 1;  //Receive FIFO locked mode
    u1 NART       : 1;  //No automatic retransmission
    u1 AWUM       : 1;  //Automatic wakeup mode
    u1 ABOM       : 1;  //Automatic bus-off management
    u1 TTCM       : 1;  //Time triggered communication mode
    u1 RSVD0      : 7;  //Reserved
    u1 RESET      : 1;  //bxCAN software master reset
    u1 DBF        : 1;  //Debug freeze
    u1 RSVD1      : 15; //Reserved
  }volatile MCR;              //CAN master control register (CAN_MCR)
  struct
  {
    u1 INAK       : 1;  //Initialization acknowledge
    u1 SLAK       : 1;  //Sleep acknowledge
    u1 ERRI       : 1;  //Error interrupt
    u1 WKUI       : 1;  //Wakeup interrupt
    u1 SLAKI      : 1;  //Sleep acknowledge interrupt
    u1 RSVD0      : 3;  //Reserved
    u1 TXM        : 1;  //Transmit mode
    u1 RXM        : 1;  //Receive mode
    u1 SAMP       : 1;  //Last sample point
    u1 RX         : 1;  //CAN Rx signal
    u1 RSVD1      : 20; //Reserved
  }volatile MSR;              //CAN master status register (CAN_MSR)
  struct
  {
    u1 RQCP0      : 1;  //Request completed mailbox0
    u1 TXOK0      : 1;  //Transmission OK of mailbox0
    u1 ALST0      : 1;  //Arbitration lost for mailbox0
    u1 TERR0      : 1;  //Transmission error of mailbox0
    u1 RSVD0      : 3;  //Reserved
    u1 ABRQ0      : 1;  //Abort request for mailbox0
    u1 RQCP1      : 1;  //Request completed mailbox1
    u1 TXOK1      : 1;  //Transmission OK of mailbox1
    u1 ALST1      : 1;  //Arbitration lost for mailbox1
    u1 TERR1      : 1;  //Transmission error of mailbox1
    u1 RSVD1      : 3;  //Reserved
    u1 ABRQ1      : 1;  //Abort request for mailbox 1
    u1 RQCP2      : 1;  //Request completed mailbox 2
    u1 TXOK2      : 1;  //Transmission OK of mailbox 2
    u1 ALST2      : 1;  //Arbitration lost for mailbox 2
    u1 TERR2      : 1;  //Transmission error of mailbox 2
    u1 RSVD2      : 3;  //Reserved
    u1 ABRQ2      : 1;  //Abort request for mailbox 2
    u1 CODE       : 2;  //Mailbox code
    u1 TME0       : 1;  //Transmit mailbox 0 empty
    u1 TME1       : 1;  //Transmit mailbox 1 empty
    u1 TME2       : 1;  //Transmit mailbox 2 empty
    u1 LOW0       : 1;  //Lowest priority flag for mailbox 0
    u1 LOW1       : 1;  //Lowest priority flag for mailbox 1
    u1 LOW2       : 1;  //Lowest priority flag for mailbox 2
  }volatile TSR;              //CAN transmit status register (CAN_TSR)
  struct
  {
    u1 FMP0       : 2;  //FIFO 0 message pending
    u1 RSVD0      : 1;  //Reserved
    u1 FULL0      : 1;  //FIFO 0 full
    u1 FOVR0      : 1;  //FIFO 0 overrun
    u1 RFOM0      : 1;  //Release FIFO 0 output mailbox
    u1 RSVD1      : 26; //Reserved
  }volatile RF0R;             //CAN receive FIFO 0 register (CAN_RF0R)
  struct
  {
    u1 FMP1       : 2;  //FIFO 1 message pending
    u1 RSVD0      : 1;  //Reserved
    u1 FULL1      : 1;  //FIFO 1 full
    u1 FOVR1      : 1;  //FIFO 1 overrun
    u1 RFOM1      : 1;  //Release FIFO 1 output mailbox
    u1 RSVD1      : 26; //Reserved
  }volatile RF1R;             //CAN receive FIFO 1 register (CAN_RF1R)
  struct
  {
    u1 TMEIE      : 1;  //Transmit mailbox empty interrupt enable
    u1 FMPIE0     : 1;  //FIFO message pending interrupt enable
    u1 FFIE0      : 1;  //FIFO full interrupt enable
    u1 FOVIE0     : 1;  //FIFO overrun interrupt enable
    u1 FMPIE1     : 1;  //FIFO message pending interrupt enable
    u1 FFIE1      : 1;  //FIFO full interrupt enable
    u1 FOVIE1     : 1;  //FIFO overrun interrupt enable
    u1 RSVD0      : 1;  //Reserved
    u1 EWGIE      : 1;  //Error warning interrupt enable
    u1 EPVIE      : 1;  //Error passive interrupt enable
    u1 BOFIE      : 1;  //Bus-off interrupt enable
    u1 LECIE      : 1;  //Last error code interrupt enable
    u1 RSVD1      : 3;  //Reserved
    u1 ERRIE      : 1;  //Error interrupt enable
    u1 WKUIE      : 1;  //Wakeup interrupt enable
    u1 SLKIE      : 1;  //Sleep interrupt enable
    u1 RSVD2      : 14; //Reserved
  }volatile IER;              //CAN interrupt enable register (CAN_IER)
  struct
  {
    u1 EWGF       : 1;  //Error warning flag
    u1 EPVF       : 1;  //Error passive flag
    u1 BOFF       : 1;  //Bus-off flag
    u1 RSVD0      : 1;  //Reserved
    u1 LEC        : 3;  //Last error code
    u1 RSVD1      : 9;  //Reserved
    u1 TEC        : 8;  //Least significant byte of the 9-bit transmit error counter
    u1 REC        : 8;  //Receive error counter
  }volatile ESR;              //CAN error status register (CAN_ESR)
  struct
  {
    u1 BRP        : 10; //Baud rate prescaler
    u1 RSVD0      : 6;  //Reserved
    u1 TS1        : 4;  //Time segment 1
    u1 TS2        : 3;  //Time segment 2
    u1 RSVD1      : 1;  //Reserved
    u1 SJW        : 2;  //Resynchronization jump width
    u1 RSVD2      : 4;  //Reserved
    u1 LBKM       : 1;  //Loop back mode (debug)
    u1 SILM       : 1;  //Silent mode (debug)
  }volatile BTR;              //CAN bit timing register (CAN_BTR)
  volatile u32 RSVD0[89];     //Reserved
  struct
  {
    struct
    {
      u1 TXRQ       : 1;  //Transmit mailbox request
      u1 RTR        : 1;  //Remote transmission request
      u1 IDE        : 1;  //Identifier extension
      u1 EXID       : 18; //Extended identifier
      u1 STID_EXID  : 11; //Standard identifier or extended identifier
    }volatile TIR;              //CAN TX mailbox identifier register (CAN_TIxR)
    struct
    {
      u1 DLC        : 4;  //Data length code
      u1 RSVD0      : 4;  //Reserved
      u1 TGT        : 1;  //Transmit global time
      u1 RSVD1      : 7;  //Reserved
      u1 TIME       : 16; //Message time stamp
    }volatile TDTR;             //CAN mailbox data length control and time stamp register (CAN_TDTxR)
    struct
    {
      u1 DATA0      : 8;  //Data byte 0
      u1 DATA1      : 8;  //Data byte 1
      u1 DATA2      : 8;  //Data byte 2
      u1 DATA3      : 8;  //Data byte 3
    }volatile TDLR;             //CAN mailbox data low register (CAN_TDLxR)
    struct
    {
      u1 DATA4      : 8;  //Data byte 4
      u1 DATA5      : 8;  //Data byte 5
      u1 DATA6      : 8;  //Data byte 6
      u1 DATA7      : 8;  //Data byte 7
    }volatile TDHR;             //CAN mailbox data high register (CAN_TDHxR)
  }volatile Mailbox[3];
  struct
  {
    struct
    {
      u1 RSVD0      : 1;  //Reserved
      u1 RTR        : 1;  //Remote transmission request
      u1 IDE        : 1;  //Identifier extension
      u1 EXID       : 18; //Extended identifier
      u1 STID_EXID  : 11; //Standard identifier or extended identifier
    }volatile RIR;              //CAN receive FIFO mailbox identifier register (CAN_RIxR)
    struct
    {
      u1 DLC        : 4;  //Data length code
      u1 RSVD0      : 4;  //Reserved
      u1 FMI        : 8;  //Filter match index
      u1 TIME       : 16; //Message time stamp
    }volatile RDTR;             //CAN receive FIFO mailbox data length control and time stamp register (CAN_RDTxR)
    struct
    {
      u1 DATA0      : 8;  //Data byte 0
      u1 DATA1      : 8;  //Data byte 1
      u1 DATA2      : 8;  //Data byte 2
      u1 DATA3      : 8;  //Data byte 3
    }volatile RDLR;             //CAN receive FIFO mailbox data low register (CAN_RDLxR)
    struct
    {
      u1 DATA4      : 8;  //Data byte 4
      u1 DATA5      : 8;  //Data byte 5
      u1 DATA6      : 8;  //Data byte 6
      u1 DATA7      : 8;  //Data byte 7
    }volatile RDHR;             //CAN receive FIFO mailbox data high register (CAN_RDHxR)
  }volatile FIFO[2];
  volatile u32 RSVD1[12];     //Reserved
  struct
  {
    u1 FINIT      : 1;  //Filter initialization mode
    u1 RSVD0      : 7;  //Reserved
    u1 CAN2SB     : 6;  //CAN2 start bank
    u1 RSVD1      : 18; //Reserved
  }volatile FMR;              //CAN filter master register (CAN_FMR)
  struct
  {
    u1 FBM        : 28; //Filter mode
    u1 RSVD1      : 4;  //Reserved
  }volatile FM1R;             //CAN filter mode register (CAN_FM1R)
  volatile u32 RSVD2;         //Reserved
  struct
  {
    u1 FSC        : 28; //Filter scale configuration
    u1 RSVD1      : 4;  //Reserved
  }volatile FS1R;             //CAN filter scale register (CAN_FS1R)
  volatile u32 RSVD3;         //Reserved
  struct
  {
    u1 FFA        : 28; //Filter FIFO assignment for filter x
    u1 RSVD1      : 4;  //Reserved
  }volatile FFA1R;            //CAN filter FIFO assignment register (CAN_FFA1R)
  volatile u32 RSVD4;         //Reserved
  struct
  {
    u1 FACT       : 28; //Filter active
    u1 RSVD1      : 4;  //Reserved
  }volatile FA1R;             //CAN filter activation register (CAN_FA1R)
  volatile u32 RSVD5[8];      //Reserved
  struct
  {
    volatile u32 FR1;             //Filter bank i register x (CAN_FxR1)
    volatile u32 FR2;             //Filter bank i register x (CAN_FxR2)
  }volatile Filter[28];
}CAN_Type;
#define CAN ((CAN_Type*) CAN_BASE)
#define CAN_INRQ                CAN->MCR.INRQ                   //Initialization request
#define CAN_SLEEP               CAN->MCR.SLEEP                  //Sleep mode request
#define CAN_TXFP                CAN->MCR.TXFP                   //Transmit FIFO priority
#define CAN_RFLM                CAN->MCR.RFLM                   //Receive FIFO locked mode
#define CAN_NART                CAN->MCR.NART                   //No automatic retransmission
#define CAN_AWUM                CAN->MCR.AWUM                   //Automatic wakeup mode
#define CAN_ABOM                CAN->MCR.ABOM                   //Automatic bus-off management
#define CAN_TTCM                CAN->MCR.TTCM                   //Time triggered communication mode
#define CAN_RESET               CAN->MCR.RESET                  //bxCAN software master reset
#define CAN_DBF                 CAN->MCR.DBF                    //Debug freeze
#define CAN_INAK                CAN->MSR.INAK                   //Initialization acknowledge
#define CAN_INAK                CAN->MSR.INAK                   //Sleep acknowledge
#define CAN_ERRI                CAN->MSR.ERRI                   //Error interrupt
#define CAN_WKUI                CAN->MSR.WKUI                   //Wakeup interrupt
#define CAN_SLAKI               CAN->MSR.SLAKI                  //Sleep acknowledge interrupt
#define CAN_TXM                 CAN->MSR.TXM                    //Transmit mode
#define CAN_RXM                 CAN->MSR.RXM                    //Receive mode
#define CAN_SAMP                CAN->MSR.SAMP                   //Last sample point
#define CAN_RX                  CAN->MSR.RX                     //CAN Rx signal
#define CAN_RQCP0               CAN->TSR.RQCP0                  //Request completed mailbox0
#define CAN_TXOK0               CAN->TSR.TXOK0                  //Transmission OK of mailbox0
#define CAN_ALST0               CAN->TSR.ALST0                  //Arbitration lost for mailbox0
#define CAN_TERR0               CAN->TSR.TERR0                  //Transmission error of mailbox0
#define CAN_ABRQ0               CAN->TSR.ABRQ0                  //Abort request for mailbox0
#define CAN_RQCP1               CAN->TSR.RQCP1                  //Request completed mailbox1
#define CAN_TXOK1               CAN->TSR.TXOK1                  //Transmission OK of mailbox1
#define CAN_ALST1               CAN->TSR.ALST1                  //Arbitration lost for mailbox1
#define CAN_TERR1               CAN->TSR.TERR1                  //Transmission error of mailbox1
#define CAN_ABRQ1               CAN->TSR.ABRQ1                  //Abort request for mailbox 1
#define CAN_RQCP2               CAN->TSR.RQCP2                  //Request completed mailbox 2
#define CAN_TXOK2               CAN->TSR.TXOK2                  //Transmission OK of mailbox 2
#define CAN_ALST2               CAN->TSR.ALST2                  //Arbitration lost for mailbox 2
#define CAN_TERR2               CAN->TSR.TERR2                  //Transmission error of mailbox 2
#define CAN_ABRQ2               CAN->TSR.ABRQ2                  //Abort request for mailbox 2
#define CAN_CODE                CAN->TSR.CODE                   //Mailbox code
#define CAN_TME0                CAN->TSR.TME0                   //Transmit mailbox 0 empty
#define CAN_TME1                CAN->TSR.TME1                   //Transmit mailbox 1 empty
#define CAN_TME2                CAN->TSR.TME2                   //Transmit mailbox 2 empty
#define CAN_LOW0                CAN->TSR.LOW0                   //Lowest priority flag for mailbox 0
#define CAN_LOW1                CAN->TSR.LOW1                   //Lowest priority flag for mailbox 1
#define CAN_LOW2                CAN->TSR.LOW2                   //Lowest priority flag for mailbox 2
#define CAN_FMP0                CAN->RF0R.FMP0                  //FIFO 0 message pending
#define CAN_FULL0               CAN->RF0R.FULL0                 //FIFO 0 full
#define CAN_FOVR0               CAN->RF0R.FOVR0                 //FIFO 0 overrun
#define CAN_RFOM0               CAN->RF0R.RFOM0                 //Release FIFO 0 output mailbox
#define CAN_FMP1                CAN->RF1R.FMP1                  //FIFO 1 message pending
#define CAN_FULL1               CAN->RF1R.FULL1                 //FIFO 1 full
#define CAN_FOVR1               CAN->RF1R.FOVR1                 //FIFO 1 overrun
#define CAN_RFOM1               CAN->RF1R.RFOM1                 //Release FIFO 1 output mailbox
#define CAN_TMEIE               CAN->IER.TMEIE                  //Transmit mailbox empty interrupt enable
#define CAN_FMPIE0              CAN->IER.FMPIE0                 //FIFO message pending interrupt enable
#define CAN_FFIE0               CAN->IER.FFIE0                  //FIFO full interrupt enable
#define CAN_FOVIE0              CAN->IER.FOVIE0                 //FIFO overrun interrupt enable
#define CAN_FMPIE1              CAN->IER.FMPIE1                 //FIFO message pending interrupt enable
#define CAN_FFIE1               CAN->IER.FFIE1                  //FIFO full interrupt enable
#define CAN_FOVIE1              CAN->IER.FOVIE1                 //FIFO overrun interrupt enable
#define CAN_EWGIE               CAN->IER.EWGIE                  //Error warning interrupt enable
#define CAN_EPVIE               CAN->IER.EPVIE                  //Error passive interrupt enable
#define CAN_BOFIE               CAN->IER.BOFIE                  //Bus-off interrupt enable
#define CAN_LECIE               CAN->IER.LECIE                  //Last error code interrupt enable
#define CAN_ERRIE               CAN->IER.ERRIE                  //Error interrupt enable
#define CAN_WKUIE               CAN->IER.WKUIE                  //Wakeup interrupt enable
#define CAN_SLKIE               CAN->IER.SLKIE                  //Sleep interrupt enable
#define CAN_EWGF                CAN->ESR.EWGF                   //Error warning flag
#define CAN_EPVF                CAN->ESR.EPVF                   //Error passive flag
#define CAN_BOFF                CAN->ESR.BOFF                   //Bus-off flag
#define CAN_LEC                 CAN->ESR.LEC                    //Last error code
#define CAN_TEC                 CAN->ESR.TEC                    //Least significant byte of the 9-bit transmit error counter
#define CAN_REC                 CAN->ESR.REC                    //Receive error counter
#define CAN_BRP                 CAN->BTR.BRP                    //Baud rate prescaler
#define CAN_TS1                 CAN->BTR.TS1                    //Time segment 1
#define CAN_TS2                 CAN->BTR.TS2                    //Time segment 2
#define CAN_SJW                 CAN->BTR.SJW                    //Resynchronization jump width
#define CAN_LBKM                CAN->BTR.LBKM                   //Loop back mode (debug)
#define CAN_SILM                CAN->BTR.SILM                   //Silent mode (debug)
#define CAN_TXRQ(BOX)           CAN->Mailbox[BOX].TIR.TXRQ      //Transmit mailbox request
#define CAN_RTR(BOX)            CAN->Mailbox[BOX].TIR.RTR       //Remote transmission request
#define CAN_IDE(BOX)            CAN->Mailbox[BOX].TIR.IDE       //Identifier extension
#define CAN_EXID(BOX)           CAN->Mailbox[BOX].TIR.EXID      //Extended identifier
#define CAN_STID_EXID(BOX)      CAN->Mailbox[BOX].TIR.STID_EXID //Standard identifier or extended identifier
#define CAN_DLC(BOX)            CAN->Mailbox[BOX].TDTR.DLC      //Data length code
#define CAN_TGT(BOX)            CAN->Mailbox[BOX].TDTR.TGT      //Transmit global time
#define CAN_TIME(BOX)           CAN->Mailbox[BOX].TDTR.TIME     //Message time stamp
#define CAN_DATA0(BOX)          CAN->Mailbox[BOX].TDLR.DATA0    //Data byte 0
#define CAN_DATA1(BOX)          CAN->Mailbox[BOX].TDLR.DATA1    //Data byte 1
#define CAN_DATA2(BOX)          CAN->Mailbox[BOX].TDLR.DATA2    //Data byte 2
#define CAN_DATA3(BOX)          CAN->Mailbox[BOX].TDLR.DATA3    //Data byte 3
#define CAN_DATA4(BOX)          CAN->Mailbox[BOX].TDHR.DATA4    //Data byte 4
#define CAN_DATA5(BOX)          CAN->Mailbox[BOX].TDHR.DATA5    //Data byte 5
#define CAN_DATA6(BOX)          CAN->Mailbox[BOX].TDHR.DATA6    //Data byte 6
#define CAN_DATA7(BOX)          CAN->Mailbox[BOX].TDHR.DATA7    //Data byte 7
#define CAN_RTR1(CH)            CAN->FIFO[CH].RIR.RTR           //Remote transmission request
#define CAN_IDE1(CH)            CAN->FIFO[CH].RIR.IDE           //Identifier extension
#define CAN_EXID1(CH)           CAN->FIFO[CH].RIR.EXID          //Extended identifier
#define CAN_STID1_EXID(CH)      CAN->FIFO[CH].RIR.STID_EXID     //Standard identifier or extended identifier
#define CAN_DLC1(CH)            CAN->FIFO[CH].RDTR.DLC          //Data length code
#define CAN_FMI(CH)             CAN->FIFO[CH].RDTR.FMI          //Filter match index
#define CAN_TIME1(CH)           CAN->FIFO[CH].RDTR.TIME         //Message time stamp
#define CAN_DATA01(CH)          CAN->FIFO[CH].RDLR.DATA0        //Data byte 0
#define CAN_DATA11(CH)          CAN->FIFO[CH].RDLR.DATA1        //Data byte 1
#define CAN_DATA21(CH)          CAN->FIFO[CH].RDLR.DATA2        //Data byte 2
#define CAN_DATA31(CH)          CAN->FIFO[CH].RDLR.DATA3        //Data byte 3
#define CAN_DATA41(CH)          CAN->FIFO[CH].RDHR.DATA4        //Data byte 4
#define CAN_DATA51(CH)          CAN->FIFO[CH].RDHR.DATA5        //Data byte 5
#define CAN_DATA61(CH)          CAN->FIFO[CH].RDHR.DATA6        //Data byte 6
#define CAN_DATA71(CH)          CAN->FIFO[CH].RDHR.DATA7        //Data byte 7
#define CAN_FINIT               CAN->FMR.FINIT                  //Filter initialization mode
#define CAN_CAN2SB              CAN->FMR.CAN2SB                 //CAN2 start bank
#define CAN_FBM                 CAN->FM1R.FBM                   //Filter mode
#define CAN_FSC                 CAN->FS1R.FSC                   //Filter scale configuration
#define CAN_FFA                 CAN->FFA1R.FFA                  //Filter FIFO assignment for filter x
#define CAN_FACT                CAN->FA1R.FACT                  //Filter active
#define CAN_FR1(CH)             CAN->Filter[CH].FR1             //Filter bank i register x (CAN_FxR1)
#define CAN_FR2(CH)             CAN->Filter[CH].FR2             //Filter bank i register x (CAN_FxR2)
/******************************************** USB *************************************************/
typedef struct
{
  struct
  {
    u1 ENDP       : 16; //Reserved
    u1 RSVD0      : 16; //Reserved
  }volatile EPR[8];           //USB endpoint n register (USB_EPnR), n=[0..7]
  volatile u32 RSVD0[8];      //Reserved
  struct
  {
    u1 FRES       : 1;  //Force USB Reset
    u1 PDWN       : 1;  //Power down
    u1 LP_MODE    : 1;  //Low-power mode
    u1 FSUSP      : 1;  //Force suspend
    u1 RESUME     : 1;  //Resume request
    u1 L1RESUME   : 1;  //LPM L1 Resume request
    u1 RSVD0      : 1;  //Reserved
    u1 L1REQM     : 1;  //LPM L1 state request interrupt mask
    u1 ESOFM      : 1;  //Expected start of frame interrupt mask
    u1 SOFM       : 1;  //Start of frame interrupt mask
    u1 RESETM     : 1;  //USB reset interrupt mask
    u1 SUSPM      : 1;  //Suspend mode interrupt mask
    u1 WKUPM      : 1;  //Wakeup interrupt mask
    u1 ERRM       : 1;  //Error interrupt mask
    u1 PMAOVRM    : 1;  //Packet memory area over / underrun interrupt mask
    u1 CTRM       : 1;  //Correct transfer interrupt mask
    u1 RSVD1      : 16; //Reserved
  }volatile CNTR;             //USB control register (USB_CNTR)
  struct
  {
    u1 EP_ID      : 4;  //Endpoint Identifier
    u1 DIR        : 1;  //Direction of transaction
    u1 RSVD0      : 2;  //Reserved
    u1 L1REQ      : 1;  //LPM L1 state request
    u1 ESOF       : 1;  //Expected start of frame
    u1 SOF        : 1;  //Start of frame
    u1 RESET      : 1;  //USB reset request
    u1 SUSP       : 1;  //Suspend mode request
    u1 WKUP       : 1;  //Wakeup
    u1 ERR        : 1;  //Error
    u1 PMAOVR     : 1;  //Packet memory area over / underrun
    u1 CTR        : 1;  //Correct transfer
    u1 RSVD1      : 16; //Reserved
  }volatile ISTR;             //USB interrupt status register (USB_ISTR)
  struct
  {
    u1 FN         : 11; //Frame number
    u1 LSOF       : 2;  //Lost SOF
    u1 LCK        : 1;  //Locked
    u1 RXDM       : 1;  //Receive data - line status
    u1 RXDP       : 1;  //Receive data + line status
    u1 RSVD0      : 16; //Reserved
  }volatile FNR;              //USB frame number register (USB_FNR)
  struct
  {
    u1 ADD        : 7;  //Device address
    u1 EF         : 1;  //Enable function
    u1 RSVD0      : 24; //Reserved
  }volatile DADDR;            //USB device address (USB_DADDR)
  struct
  {
    u1 RSVD0      : 3;  //Reserved
    u1 BTABLE     : 13; //Buffer table
    u1 RSVD1      : 16; //Reserved
  }volatile BTABLE;           //Buffer table address (USB_BTABLE)
  struct
  {
    u1 LPMEN      : 1;  //LPM support enable
    u1 LPMACK     : 1;  //LPM Token acknowledge enable
    u1 RSVD0      : 1;  //Reserved
    u1 REMWAKE    : 1;  //RemoteWake value
    u1 BESL       : 4;  //BESL value
    u1 RSVD1      : 24; //Reserved
  }volatile LPMCSR;           //LPM control and status register (USB_LPMCSR)
  struct
  {
    u1 BCDEN      : 1;  //Battery charging detector (BCD) enable
    u1 DCDEN      : 1;  //Data contact detection (DCD) mode enable
    u1 PDEN       : 1;  //Primary detection (PD) mode enable
    u1 SDEN       : 1;  //Secondary detection (SD) mode enable
    u1 DCDET      : 1;  //Data contact detection (DCD) status
    u1 PDET       : 1;  //Primary detection (PD) status
    u1 SDET       : 1;  //Secondary detection (SD) status
    u1 PS2DET     : 1;  //DM pull-up detection status
    u1 RSVD0      : 7;  //Reserved
    u1 DPPU       : 1;  //DP pull-up control
    u1 RSVD1      : 16; //Reserved
  }volatile BCDR;             //Battery charging detector (USB_BCDR)
}USB_Type;
#define USB ((USB_Type*) USB_BASE)
#define USB_GET_EA(EP)          USB->EPR[EP].ENDP&0x000F        //Endpoint address
#define USB_GET_STX(EP)         USB->EPR[EP].ENDP&0x0030        //Status bits, for transmission transfers
#define USB_GET_DTX(EP)         USB->EPR[EP].ENDP&0x0040        //Data Toggle, for transmission transfers
#define USB_GET_CTX(EP)         USB->EPR[EP].ENDP&0x0080        //Correct Transfer for transmission
#define USB_GET_KIND(EP)        USB->EPR[EP].ENDP&0x0100        //Endpoint kind
#define USB_GET_TYPE(EP)        USB->EPR[EP].ENDP&0x0600        //Endpoint type
#define USB_GET_SETUP(EP)       USB->EPR[EP].ENDP&0x0800        //Setup transaction completed
#define USB_GET_SRX(EP)         USB->EPR[EP].ENDP&0x3000        //Status bits, for reception transfers
#define USB_GET_DRX(EP)         USB->EPR[EP].ENDP&0x4000        //Data Toggle, for reception transfers
#define USB_GET_CRX(EP)         USB->EPR[EP].ENDP&0x8000        //Correct Transfer for reception
#define USB_SET_EA(EP,DT)       USB->EPR[EP].ENDP = (USB->EPR[EP].ENDP&0x8F80)|DT //Endpoint address
#define USB_SET_STX(EP,DT)      USB->EPR[EP].ENDP = (USB->EPR[EP].ENDP^DT)&0x8FBF //Status bits, for transmission transfers
#define USB_SET_DTX(EP,DT)      USB->EPR[EP].ENDP = (USB->EPR[EP].ENDP^DT)&0x8FCF //Data Toggle, for transmission transfers
#define USB_SET_CTX(EP,DT)      USB->EPR[EP].ENDP = (USB->EPR[EP].ENDP&0x8F0F)|DT //Correct Transfer for transmission
#define USB_SET_KIND(EP,DT)     USB->EPR[EP].ENDP = (USB->EPR[EP].ENDP&0x8E8F)|DT //Endpoint kind
#define USB_SET_TYPE(EP,DT)     USB->EPR[EP].ENDP = (USB->EPR[EP].ENDP&0x898F)|DT //Endpoint type
#define USB_SET_SRX(EP,DT)      USB->EPR[EP].ENDP = (USB->EPR[EP].ENDP^DT)&0xBF8F //Status bits, for reception transfers
#define USB_SET_DRX(EP,DT)      USB->EPR[EP].ENDP = (USB->EPR[EP].ENDP^DT)&0xCF8F //Data Toggle, for reception transfers
#define USB_SET_CRX(EP,DT)      USB->EPR[EP].ENDP = (USB->EPR[EP].ENDP&0x0F8F)|DT //Correct Transfer for reception
#define USB_FRES                USB->CNTR.FRES                  //Force USB Reset
#define USB_PDWN                USB->CNTR.PDWN                  //Power down
#define USB_LP_MODE             USB->CNTR.LP_MODE               //Low-power mode
#define USB_FSUSP               USB->CNTR.FSUSP                 //Force suspend
#define USB_RESUME              USB->CNTR.RESUME                //Resume request
#define USB_L1RESUME            USB->CNTR.L1RESUME              //LPM L1 Resume request
#define USB_L1REQM              USB->CNTR.L1REQM                //LPM L1 state request interrupt mask
#define USB_IRQ_ESOF_EN         USB->CNTR.ESOFM                 //Expected start of frame interrupt mask
#define USB_IRQ_SOF_EN          USB->CNTR.SOFM                  //Start of frame interrupt mask
#define USB_IRQ_RST_EN          USB->CNTR.RESETM                //USB reset interrupt mask
#define USB_IRQ_SUSP_EN         USB->CNTR.SUSPM                 //Suspend mode interrupt mask
#define USB_IRQ_WKUP_EN         USB->CNTR.WKUPM                 //Wakeup interrupt mask
#define USB_IRQ_ERR_EN          USB->CNTR.ERRM                  //Error interrupt mask
#define USB_IRQ_PMA_EN          USB->CNTR.PMAOVRM               //Packet memory area over / underrun interrupt mask
#define USB_IRQ_CTR_EN          USB->CNTR.CTRM                  //Correct transfer interrupt mask
#define USB_EP_FLG              USB->ISTR.EP_ID                 //Endpoint Identifier
#define USB_DIR_FLG             USB->ISTR.DIR                   //Direction of transaction
#define USB_L1REQ               USB->ISTR.L1REQ                 //LPM L1 state request
#define USB_ESOF_FLG            USB->ISTR.ESOF                  //Expected start of frame
#define USB_SOF_FLG             USB->ISTR.SOF                   //Start of frame
#define USB_RST_FLG             USB->ISTR.RESET                 //USB reset request
#define USB_SUSP_FLG            USB->ISTR.SUSP                  //Suspend mode request
#define USB_WKUP_FLG            USB->ISTR.WKUP                  //Wakeup
#define USB_ERR_FLG             USB->ISTR.ERR                   //Error
#define USB_PMA_FLG             USB->ISTR.PMAOVR                //Packet memory area over / underrun
#define USB_CTR_FLG             USB->ISTR.CTR                   //Correct transfer
#define USB_FN                  USB->FNR.FN                     //Frame number
#define USB_LSOF                USB->FNR.LSOF                   //Lost SOF
#define USB_LCK                 USB->FNR.LCK                    //Locked
#define USB_RXDM                USB->FNR.RXDM                   //Receive data - line status
#define USB_RXDP                USB->FNR.RXDP                   //Receive data + line status
#define USB_ADD                 USB->DADDR.ADD                  //Device address
#define USB_EN                  USB->DADDR.EF                   //Enable function
#define USB_BTABLE              USB->BTABLE.BTABLE              //Buffer table
#define USB_LPMEN               USB->LPMCSR.LPMEN               //LPM support enable
#define USB_LPMACK              USB->LPMCSR.LPMACK              //LPM Token acknowledge enable
#define USB_REMWAKE             USB->LPMCSR.REMWAKE             //RemoteWake value
#define USB_BESL                USB->LPMCSR.BESL                //BESL value
#define USB_BCD_EN              USB->BCDR.BCDEN                 //Battery charging detector (BCD) enable
#define USB_DCD_EN              USB->BCDR.DCDEN                 //Data contact detection (DCD) mode enable
#define USB_PD_EN               USB->BCDR.PDEN                  //Primary detection (PD) mode enable
#define USB_SD_EN               USB->BCDR.SDEN                  //Secondary detection (SD) mode enable
#define USB_DCDET               USB->BCDR.DCDET                 //Data contact detection (DCD) status
#define USB_PDET                USB->BCDR.PDET                  //Primary detection (PD) status
#define USB_SDET                USB->BCDR.SDET                  //Secondary detection (SD) status
#define USB_PS2DET              USB->BCDR.PS2DET                //DM pull-up detection status
#define USB_DPPU                USB->BCDR.DPPU                  //DP pull-up control
#define USB_TX_STALL            ((u16) 0x0010)
#define USB_TX_NAK              ((u16) 0x0020)
#define USB_TX_VALID            ((u16) 0x0030)
#define USB_DTX                 ((u16) 0x0040)
#define USB_CTX                 ((u16) 0x0080)
#define USB_KIND                ((u16) 0x0100)
#define USB_TYPE_BULK           ((u16) 0x0000)
#define USB_TYPE_CONTROL        ((u16) 0x0200)
#define USB_TYPE_ISO            ((u16) 0x0400)
#define USB_TYPE_INTERRUPT      ((u16) 0x0600)
#define USB_RX_STALL            ((u16) 0x1000)
#define USB_RX_NAK              ((u16) 0x2000)
#define USB_RX_VALID            ((u16) 0x3000)
#define USB_DRX                 ((u16) 0x4000)
#define USB_CRX                 ((u16) 0x8000)
#define STD_GET_STATUS          ((u8) 0x00)
#define STD_CLEAR_FEATURE       ((u8) 0x01)
#define STD_SET_FEATURE         ((u8) 0x03)
#define STD_SET_ADDRESS         ((u8) 0x05)
#define STD_GET_DESCRIPTOR      ((u8) 0x06)
#define STD_SET_DESCRIPTOR      ((u8) 0x07)
#define STD_GET_CONFIG          ((u8) 0x08)
#define STD_SET_CONFIG          ((u8) 0x09)
#define STD_GET_INTERFACE       ((u8) 0x0A)
#define STD_SET_INTERFACE       ((u8) 0x0B)
#define STD_SYNCH_FRAME         ((u8) 0x0C)
#define CLS_GET_REPORT          ((u8) 0x01)
#define CLS_GET_IDLE            ((u8) 0x02)
#define CLS_GET_PROTOCOL        ((u8) 0x03)
#define CLS_SET_REPORT          ((u8) 0x09)
#define CLS_SET_IDLE            ((u8) 0x0A)
#define CLS_SET_PROTOCOL        ((u8) 0x0B)
#define DESC_DEV                ((u8) 0x01)
#define DESC_CONF               ((u8) 0x02)
#define DESC_STR                ((u8) 0x03)
#define DESC_INT                ((u8) 0x04)
#define DESC_EP                 ((u8) 0x05)
#define DESC_HID                ((u8) 0x22)
#define TYPE_DEV_STD            ((u8) 0x00)
#define TYPE_DEV_CLS            ((u8) 0x20)
#define TYPE_DEV_VND            ((u8) 0x40)
#define TYPE_INT_STD            ((u8) 0x01)
#define TYPE_INT_CLS            ((u8) 0x21)
#define TYPE_INT_VND            ((u8) 0x41)
#define TYPE_EP_STD             ((u8) 0x02)
#define TYPE_EP_CLS             ((u8) 0x22)
#define TYPE_EP_VND             ((u8) 0x42)
/******************************************* END **************************************************/
#ifdef __cplusplus
}
#endif
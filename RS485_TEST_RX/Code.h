/******************************************* ��������� ********************************************/
static const u32 INF[4]@0x08000200 = { //������� ���������� � �������
  0x01010101, //IDD, PCB, APP, VP (��� �������, ������ �����, ������ ��, ������������ ������ ��������� ������)
  0x00000000, //������
  0x00000000, //������
  0x00000000  //���� ��������� CRC
};
/**************************************************************************************************/
#define MEM_LDR         ((u32) 0x08004000)      //��������� �������
#define MEM_CFG         ((u32) 0x08004800)      //������������ �������
/************************************ ���� ������� RS-485 *****************************************/
#define VTS_ST          ((u8) 0x00)             //������ ���
#define VTS_GET_ST      ((u8) 0x01)             //������ ��������� ���
#define VTS_SET_ST      ((u8) 0x02)             //������ ��������� ���
#define VTS_ID          ((u8) 0x03)             //������� ���������� � �������
#define VTS_SET_ADR     ((u8) 0x04)             //��������� ������ ����������

//#define VTS_RST         ((u8) 0x04)             //����� ����������
//#define VTS_LDR         ((u8) 0x05)             //������� �� ���������� �� ������
//#define BUF_RD          ((u8) 0x04)             //������ ������ (SUB - ����� �����, 16 ��������)
//#define BUF_WR          ((u8) 0x05)             //������ ������ (SUB - ����� �����, 16 ��������)
//#define DEV_STT         ((u8) 0x05)             //��������� ������� (SUB: 0 - ������, 1 - ������)
//#define DEV_CFG         ((u8) 0x06)             //������������ ������� (SUB: 0 - ������, 1 - ������)
//#define DEV_TM          ((u8) 0x07)             //����� ������� (SUB: 0 - ������, 1 - ������, 2 - �������)
//#define DEV_REG         ((u8) 0x08)             //����������� ������� (SUB: 0 - ����, 1 - ������, 2 - ������ �������) (DEV_CTR)
//#define DEV_MAD         ((u8) 0x09)             //��� ������� (SUB: 0 - ���� � ����� ����������������, 1 - ����� �� ������ ����������������, 2 - �������� ������, 3 - ������ ������, 4 - ������ IMEI) (DEV_CTR)
/******************************************* ��������� ********************************************/
#define T_0C1           ((u16) 12)              //�������� 0.125� (4��)
#define T_0C2           ((u16) 25)              //�������� 0.25� (2��)
#define T_0C3           ((u16) 30)              //�������� 0,3� (1.5��)
#define T_0C5           ((u16) 50)              //�������� 0.5� (1��)
#define T_1C0           ((u16) 100)             //�������� 1� (0.5��)
#define T_1C5           ((u16) 150)             //�������� 1,5� (0.3��)
#define T_003           ((u16) 300)             //�������� 3�
#define T_005           ((u16) 500)             //�������� 5�
#define T_010           ((u16) 1000)            //�������� 10�
#define T_015           ((u16) 1500)            //�������� 15�
#define T_030           ((u16) 3000)            //�������� 30�
#define T_060           ((u16) 6000)            //�������� 60�
#define T_120           ((u16) 12000)           //�������� 120�
#define T_300           ((u16) 30000)           //�������� 300�
#define T_1H0           ((u32) 360000)          //�������� 3600� (1 ���)
#define ZP_GD           ((u8) 0x01)             //���� "������"
#define ZP_BD           ((u8) 0x02)             //���� "��������"
#define ZP_P1           ((u8) 0x03)             //���� "����� 1"
#define ZP_P2           ((u8) 0x04)             //���� "����� 2"
#define ZP_ER           ((u8) 0x05)             //���� "�������������"
#define ZP_UP           ((u8) 0x06)             //���� "���������� �� ������"
#define SW_S1           ((u8) 0x01)             //�������� ������� ������ 1 ���
#define SW_S2           ((u8) 0x02)             //�������� ������� ������ 2 ����
#define SW_S3           ((u8) 0x03)             //�������� ������� ������ 3 ����
#define SW_L1           ((u8) 0x04)             //������� ������� ������ 1 ���
/*************************************** ���������� **********************************************/
typedef struct
{
  struct
  {
    u1 DEV                        : 8; //IDD - ��� ����������
    u1 PCB                        : 8; //PCB - ������ �����
    u1 APP                        : 8; //APP - ������ ��
    u1 VP                         : 8; //VP - ������������ ������ ��������� ������
    u1 CRD                        : 8; //CRC ����������� ��������� ������ ����������
    u1 ADR                        : 8; //����� ����������
    u1 RSVD                       : 16; //������
  }Slave;
  struct
  {
    u1 DEV                        : 8; //IDD - ��� ����������
    u1 PCB                        : 8; //PCB - ������ �����
    u1 APP                        : 8; //APP - ������ ��
    u1 VP                         : 8; //VP - ������������ ������ ��������� ������
    u1 CRD                        : 8; //CRC ����������� ��������� ������ ����������
    u1 RSVD                       : 24; //������
  }DEV[64];
  u32 VTS_NUM;                        //���������� ���������
  u32 CRC_CFG;                        //���� ��������� CRC
}CONF;
CONF *CFG = (void*)(MEM_CFG);
/**************************************************************************************************/
struct
{
  u8 SOF;                                       //��������� ������ (0���)
  u8 AD_TX;                                     //����� �����������
  u8 AD_RX;                                     //����� ���������
  u8 DSIZE;                                     //������ ������� DATA (�2 ����)
  u16 DATA[256];                                //������� DATA
}UR;
//= {0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 
//  0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55};
/**************************************************************************************************/
typedef struct
{
  u1 CMD                        : 8; //�������� ����������
  u1 CRD                        : 8; //CRC ����������� ��������� ������ ����������
  u1 VP                         : 8; //������ ��������� ������ (�� ����� ���������)
  u1 MD                         : 1; //����� ������: 0 - Boot, 1 - APP
  u1 IND                        : 1; //��������� ��������� ���������
  u1 CTX                        : 1; //������ ������ �� CRC ����������� ��������� ������ ����������
  u1 BTX                        : 1; //���������� ������� �� ������� �����, ������ ������ �� CRC ����������� ��������� ������ ����������
  u1 RSVD                       : 4; //������
}VTS_CTR;
/**************************************************************************************************/
typedef struct
{
  u1 IND                        : 8; //��������� ����������
  u1 RSVD                       : 24; //������
}VTS_STT;
/**************************************************************************************************/
typedef struct
{
  u1 DEV                        : 8; //IDD - ��� ����������
  u1 PCB                        : 8; //PCB - ������ �����
  u1 APP                        : 8; //APP - ������ ��
  u1 VP                         : 8; //VP - ������������ ������ ��������� ������
}VTS_DEV_ID;
/**************************************************************************************************/
struct
{
  u1 CNT                        : 3; //���������� ��������
  u1 RX_ER                      : 1; //��� ������� � ������� ��������� ���������� ��������
  u1 RX_OK                      : 1; //���� ������
  u1 RX_CRC                     : 1; //������ CRC
  u1 CMD                        : 8; //�������� ���������� (0 - ��� ������)
  u1 VP                         : 8; //������ ��������� ������ (�� ����� ���������)
  u1 CTX                        : 1; //������ ������ �� CRC ����������� ��������� ������ ����������
  u1 BTX                        : 1; //���������� ������� �� ������� �����, ������ ������ �� CRC ����������� ��������� ������ ����������
  u1 IND                        : 8; //��������� �������
}VTS[63]; //��������� ���
///******************************* ��������� ��������� ������� **************************************/
struct
{
  u1 ZP_OLD             : 4; //������ ��������� ZP
  u1 ZPC                : 2; //��������� ������ ZP
  u1 SW                 : 3; //��������� ������� ������
  u1 KEY0               : 1; //��������� ������� ������ SW
  u1 KEY1               : 3; //������� ������� ��������� ������� ������ SW
  u1 KEY2               : 5; //������� ������� ����������� ������ SW
  u1 KEY3               : 2; //������� ���������� ������� ������ SW
  u1 ZP_FIX             : 1; //���������� ����� ��������������
  u1 NUM                : 6; //������� ����� ���
  u1 RS_OPROS           : 1; //���������� ������
  u1 RSVD               : 9; //������
}Status;
/****************************** ��������� ��������� �������****************************************/
struct
{
  u1 O_ON               : 1;  //��������� ��������� - �������� ����� ���
  u1 P_ON               : 1;  //��������� ��������� - �������� ����� ���
  u1 O_OF               : 1;  //��������� ��������� - ������ ���
  u1 P_OF               : 1;  //��������� ��������� - ������ ���
  u1 AC                 : 1;  //��������� ��������� - ��� ����
  u1 BAT                : 1;  //��������� ��������� - ���������� ���
  u1 BAT_LOW            : 1;  //��������� ��������� - ����������� ������ ���
  u1 NR                 : 1;  //��������� ��������� - ���������
  u1 UP                 : 1;  //��������� ��������� - ����������
  u1 TR                 : 1;  //��������� ��������� - �������
  u1 TF                 : 1;  //��������� ��������� - ����� �������
  u1 P1                 : 1;  //��������� ��������� - ����� 1
  u1 ER                 : 1;  //��������� ��������� - �������������
  u1 P2                 : 1;  //��������� ��������� - ����� 2
  u1 LED_PW             : 2;  //��������� ������ - LED_PW
  u1 LED_SR             : 2;  //��������� ������ - LED_SR
  u1 LED_C1             : 2;  //��������� ������ - LED_C1
  u1 LED_C2             : 2;  //��������� ������ - LED_C2
  u1 BUZZ               : 1;  //��������� ������ - BUZZ
  u1 LAMP               : 1;  //��������� ������ - LAMP
  u1 PCN1               : 1;  //��������� ������ - PCN1
  u1 PCN2               : 1;  //��������� ������ - PCN2
  u1 PCN3               : 1;  //��������� ������ - PCN3
  u1 LTM                : 1;  //��������� ������ - LED_TM
  u1 ZP                 : 4;  //��������� ������ - ZP
}SCH;
/**************************************************************************************************/
u8 MSTR;                                        //����� ������: 1-������, 0-�����
u8 SW1;
u8 SW2;
u8 SW3;
u8 SW4;
//u8 VTS_ERROR;                                   //��������� ������ ���



u16 t_rx;
u16 t_tx;
u16 t_sw;
u16 t_zp;
u16 C_ZP;
/***************************************** USART1 *************************************************/
u16 U1_TX1;                                     //������� ���������� ��������
u16 U1_RX1;                                     //������� �������� ��������
u8 *U1_TX2;                                     //������� ����������� ������
u8 *U1_RX2;                                     //������� ��������� ������



#define PREAMB_I                0
#define LENGTH_I                1
#define TX_ADD_I                2
#define RX_ADD_I                3
#define CMD_I                   4
#define DAT_0_I                 5
#define DAT_1_I                 6
#define DAT_2_I                 7

#define SET_NULLADDR_CMD       1
#define GET_NULLADDR_CMD       2
#define SET_NEWADDR_CMD        3
#define GET_NEWADDR_CMD        4

#define PREAMB                  0xAA
#define SLAVE_NULL_ADDR         0x00
#define MASTR_ADDR              0xFF


#define BIND_START              0x01
#define BIND_NULLADDR           0x02
#define BIND_NEWADDR            0x03
#define BIND_READY              0x04

#define STEP_NULL_ADDR          0x01
#define STEP_NEW_ADDR           0x02

#define PACK_DEFOLT_MSK         0x00
#define PACK_RX_START           0x01
#define PACK_RX_COMPL           0x02
#define PACK_LEN                0x08

u8 data_txrx[12];
u8 data_rx = 0;
u8 slave_addr = 0;
struct{
  u1 bind       : 3;
  u1 tx         : 2;
  u1 RSV        :27;
}Stat;
/************************************ ������ ���������� *******************************************/
#define PIN_SW1         GPIO_IN(GPIOA)&GPIO_PIN00
#define PIN_SW2         GPIO_IN(GPIOA)&GPIO_PIN01
#define PIN_DIR2        GPIO_IN(GPIOA)&GPIO_PIN04
#define PIN_H2_G        GPIO_IN(GPIOA)&GPIO_PIN05
#define PIN_H2_R        GPIO_IN(GPIOA)&GPIO_PIN06
#define PIN_H1_G        GPIO_IN(GPIOA)&GPIO_PIN07
#define PIN_DIR1        GPIO_IN(GPIOA)&GPIO_PIN08
#define PIN_SW3         GPIO_IN(GPIOA)&GPIO_PIN15
#define PIN_H1_R        GPIO_IN(GPIOB)&GPIO_PIN00
#define PIN_H4_G        GPIO_IN(GPIOB)&GPIO_PIN01
#define PIN_H3_G        GPIO_IN(GPIOB)&GPIO_PIN02
#define PIN_SW4         GPIO_IN(GPIOB)&GPIO_PIN03
#define PIN_H5_R        GPIO_IN(GPIOB)&GPIO_PIN05
#define PIN_H5_G        GPIO_IN(GPIOB)&GPIO_PIN06
#define PIN_H6_G        GPIO_IN(GPIOB)&GPIO_PIN07
#define PIN_H3_R        GPIO_IN(GPIOB)&GPIO_PIN10
#define PIN_H4_R        GPIO_IN(GPIOB)&GPIO_PIN11
#define PIN_H6_R        GPIO_IN(GPIOC)&GPIO_PIN13
#define PIN_H7_G        GPIO_IN(GPIOC)&GPIO_PIN14
#define PIN_H8_R        GPIO_IN(GPIOC)&GPIO_PIN15
#define PIN_H8_G        GPIO_IN(GPIOF)&GPIO_PIN00
#define PIN_H7_R        GPIO_IN(GPIOF)&GPIO_PIN01
#define DIR1_ON         GPIO_BIT_SET(GPIOA) = GPIO_PIN08
#define DIR1_OFF        GPIO_BIT_RST(GPIOA) = GPIO_PIN08
#define DIR2_ON         GPIO_BIT_SET(GPIOA) = GPIO_PIN04
#define DIR2_OFF        GPIO_BIT_RST(GPIOA) = GPIO_PIN04
#define LED_H1_O        GPIO_BIT_RST(GPIOB) = GPIO_PIN00; GPIO_BIT_RST(GPIOA) = GPIO_PIN07
#define LED_H1_R        GPIO_BIT_SET(GPIOB) = GPIO_PIN00; GPIO_BIT_RST(GPIOA) = GPIO_PIN07
#define LED_H1_G        GPIO_BIT_RST(GPIOB) = GPIO_PIN00; GPIO_BIT_SET(GPIOA) = GPIO_PIN07
#define LED_H1_Y        GPIO_BIT_SET(GPIOB) = GPIO_PIN00; GPIO_BIT_SET(GPIOA) = GPIO_PIN07
#define LED_H2_O        GPIO_BIT_RST(GPIOA) = GPIO_PIN06; GPIO_BIT_RST(GPIOA) = GPIO_PIN05
#define LED_H2_R        GPIO_BIT_SET(GPIOA) = GPIO_PIN06; GPIO_BIT_RST(GPIOA) = GPIO_PIN05
#define LED_H2_G        GPIO_BIT_RST(GPIOA) = GPIO_PIN06; GPIO_BIT_SET(GPIOA) = GPIO_PIN05
#define LED_H2_Y        GPIO_BIT_SET(GPIOA) = GPIO_PIN06; GPIO_BIT_SET(GPIOA) = GPIO_PIN05
#define LED_H3_O        GPIO_BIT_RST(GPIOB) = GPIO_PIN10; GPIO_BIT_RST(GPIOB) = GPIO_PIN02
#define LED_H3_R        GPIO_BIT_SET(GPIOB) = GPIO_PIN10; GPIO_BIT_RST(GPIOB) = GPIO_PIN02
#define LED_H3_G        GPIO_BIT_RST(GPIOB) = GPIO_PIN10; GPIO_BIT_SET(GPIOB) = GPIO_PIN02
#define LED_H3_Y        GPIO_BIT_SET(GPIOB) = GPIO_PIN10; GPIO_BIT_SET(GPIOB) = GPIO_PIN02
#define LED_H4_O        GPIO_BIT_RST(GPIOB) = GPIO_PIN11; GPIO_BIT_RST(GPIOB) = GPIO_PIN01
#define LED_H4_R        GPIO_BIT_SET(GPIOB) = GPIO_PIN11; GPIO_BIT_RST(GPIOB) = GPIO_PIN01
#define LED_H4_G        GPIO_BIT_RST(GPIOB) = GPIO_PIN11; GPIO_BIT_SET(GPIOB) = GPIO_PIN01
#define LED_H4_Y        GPIO_BIT_SET(GPIOB) = GPIO_PIN11; GPIO_BIT_SET(GPIOB) = GPIO_PIN01
#define LED_H5_O        GPIO_BIT_RST(GPIOB) = GPIO_PIN06; GPIO_BIT_RST(GPIOB) = GPIO_PIN05
#define LED_H5_R        GPIO_BIT_SET(GPIOB) = GPIO_PIN06; GPIO_BIT_RST(GPIOB) = GPIO_PIN05
#define LED_H5_G        GPIO_BIT_RST(GPIOB) = GPIO_PIN06; GPIO_BIT_SET(GPIOB) = GPIO_PIN05
#define LED_H5_Y        GPIO_BIT_SET(GPIOB) = GPIO_PIN06; GPIO_BIT_SET(GPIOB) = GPIO_PIN05
#define LED_H6_O        GPIO_BIT_RST(GPIOC) = GPIO_PIN13; GPIO_BIT_RST(GPIOB) = GPIO_PIN07
#define LED_H6_R        GPIO_BIT_SET(GPIOC) = GPIO_PIN13; GPIO_BIT_RST(GPIOB) = GPIO_PIN07
#define LED_H6_G        GPIO_BIT_RST(GPIOC) = GPIO_PIN13; GPIO_BIT_SET(GPIOB) = GPIO_PIN07
#define LED_H6_Y        GPIO_BIT_SET(GPIOC) = GPIO_PIN13; GPIO_BIT_SET(GPIOB) = GPIO_PIN07
#define LED_H7_O        GPIO_BIT_RST(GPIOF) = GPIO_PIN01; GPIO_BIT_RST(GPIOC) = GPIO_PIN14
#define LED_H7_R        GPIO_BIT_SET(GPIOF) = GPIO_PIN01; GPIO_BIT_RST(GPIOC) = GPIO_PIN14
#define LED_H7_G        GPIO_BIT_RST(GPIOF) = GPIO_PIN01; GPIO_BIT_SET(GPIOC) = GPIO_PIN14
#define LED_H7_Y        GPIO_BIT_SET(GPIOF) = GPIO_PIN01; GPIO_BIT_SET(GPIOC) = GPIO_PIN14
#define LED_H8_O        GPIO_BIT_RST(GPIOC) = GPIO_PIN15; GPIO_BIT_RST(GPIOF) = GPIO_PIN00
#define LED_H8_R        GPIO_BIT_SET(GPIOC) = GPIO_PIN15; GPIO_BIT_RST(GPIOF) = GPIO_PIN00
#define LED_H8_G        GPIO_BIT_RST(GPIOC) = GPIO_PIN15; GPIO_BIT_SET(GPIOF) = GPIO_PIN00
#define LED_H8_Y        GPIO_BIT_SET(GPIOC) = GPIO_PIN15; GPIO_BIT_SET(GPIOF) = GPIO_PIN00
#define ZP_ON           TIM_EN(TIM03) = ON
#define ZP_OFF          TIM_EN(TIM03) = OFF; TIM_CNT(TIM03) = OFF;
/**************************************************************************************************/

void ZP_SET(u16 F);
void RS_TX(void);
void Func_0(void);
void Tx_Data(void);

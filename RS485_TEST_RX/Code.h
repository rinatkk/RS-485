/******************************************* Параметры ********************************************/
static const u32 INF[4]@0x08000200 = { //Таблица информации о приборе
  0x01010101, //IDD, PCB, APP, VP (Тип прибора, версия платы, версия ПО, максимальная версия протокола обмена)
  0x00000000, //Резерв
  0x00000000, //Резерв
  0x00000000  //Поле коррекции CRC
};
/**************************************************************************************************/
#define MEM_LDR         ((u32) 0x08004000)      //Загрузчик прибора
#define MEM_CFG         ((u32) 0x08004800)      //Конфигурация прибора
/************************************ Коды комманд RS-485 *****************************************/
#define VTS_ST          ((u8) 0x00)             //Статус ВТС
#define VTS_GET_ST      ((u8) 0x01)             //Чтение состояния ВТС
#define VTS_SET_ST      ((u8) 0x02)             //Запись состояния ВТС
#define VTS_ID          ((u8) 0x03)             //Таблица информации о приборе
#define VTS_SET_ADR     ((u8) 0x04)             //Установка адреса устройства

//#define VTS_RST         ((u8) 0x04)             //Сброс процессора
//#define VTS_LDR         ((u8) 0x05)             //Переход на исполнение из памяти
//#define BUF_RD          ((u8) 0x04)             //Чтение буфера (SUB - номер блока, 16 максимум)
//#define BUF_WR          ((u8) 0x05)             //Запись буфера (SUB - номер блока, 16 максимум)
//#define DEV_STT         ((u8) 0x05)             //Состояние прибора (SUB: 0 - чтение, 1 - запись)
//#define DEV_CFG         ((u8) 0x06)             //Конфигурация прибора (SUB: 0 - чтение, 1 - запись)
//#define DEV_TM          ((u8) 0x07)             //Ключи прибора (SUB: 0 - чтение, 1 - запись, 2 - считать)
//#define DEV_REG         ((u8) 0x08)             //Регистратор прибора (SUB: 0 - инфо, 1 - чтение, 2 - запись времени) (DEV_CTR)
//#define DEV_MAD         ((u8) 0x09)             //МАД прибора (SUB: 0 - вход в режим конфигурирования, 1 - выход из режима конфигурирования, 2 - удаление файлов, 3 - запись файлов, 4 - запрос IMEI) (DEV_CTR)
/******************************************* Параметры ********************************************/
#define T_0C1           ((u16) 12)              //Интервал 0.125с (4Гц)
#define T_0C2           ((u16) 25)              //Интервал 0.25с (2Гц)
#define T_0C3           ((u16) 30)              //Интервал 0,3с (1.5Гц)
#define T_0C5           ((u16) 50)              //Интервал 0.5с (1Гц)
#define T_1C0           ((u16) 100)             //Интервал 1с (0.5Гц)
#define T_1C5           ((u16) 150)             //Интервал 1,5с (0.3Гц)
#define T_003           ((u16) 300)             //Интервал 3с
#define T_005           ((u16) 500)             //Интервал 5с
#define T_010           ((u16) 1000)            //Интервал 10с
#define T_015           ((u16) 1500)            //Интервал 15с
#define T_030           ((u16) 3000)            //Интервал 30с
#define T_060           ((u16) 6000)            //Интервал 60с
#define T_120           ((u16) 12000)           //Интервал 120с
#define T_300           ((u16) 30000)           //Интервал 300с
#define T_1H0           ((u32) 360000)          //Интервал 3600с (1 час)
#define ZP_GD           ((u8) 0x01)             //Звук "Удачно"
#define ZP_BD           ((u8) 0x02)             //Звук "Неудачно"
#define ZP_P1           ((u8) 0x03)             //Звук "Пожар 1"
#define ZP_P2           ((u8) 0x04)             //Звук "Пожар 2"
#define ZP_ER           ((u8) 0x05)             //Звук "Неисправность"
#define ZP_UP           ((u8) 0x06)             //Звук "Постановка на охрану"
#define SW_S1           ((u8) 0x01)             //Короткое нажатие кнопки 1 раз
#define SW_S2           ((u8) 0x02)             //Короткое нажатие кнопки 2 раза
#define SW_S3           ((u8) 0x03)             //Короткое нажатие кнопки 3 раза
#define SW_L1           ((u8) 0x04)             //Длинное нажатие кнопки 1 раз
/*************************************** Переменные **********************************************/
typedef struct
{
  struct
  {
    u1 DEV                        : 8; //IDD - Тип устройства
    u1 PCB                        : 8; //PCB - Версия платы
    u1 APP                        : 8; //APP - Версия ПО
    u1 VP                         : 8; //VP - Максимальная версия протокола обмена
    u1 CRD                        : 8; //CRC уникального серийного номера процессора
    u1 ADR                        : 8; //Адрес устройства
    u1 RSVD                       : 16; //Резерв
  }Slave;
  struct
  {
    u1 DEV                        : 8; //IDD - Тип устройства
    u1 PCB                        : 8; //PCB - Версия платы
    u1 APP                        : 8; //APP - Версия ПО
    u1 VP                         : 8; //VP - Максимальная версия протокола обмена
    u1 CRD                        : 8; //CRC уникального серийного номера процессора
    u1 RSVD                       : 24; //Резерв
  }DEV[64];
  u32 VTS_NUM;                        //Количество устройств
  u32 CRC_CFG;                        //Поле коррекции CRC
}CONF;
CONF *CFG = (void*)(MEM_CFG);
/**************************************************************************************************/
struct
{
  u8 SOF;                                       //Преамбула пакета (0хАА)
  u8 AD_TX;                                     //Адрес передатчика
  u8 AD_RX;                                     //Адрес приемника
  u8 DSIZE;                                     //Размер области DATA (х2 байт)
  u16 DATA[256];                                //Область DATA
}UR;
//= {0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 
//  0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55};
/**************************************************************************************************/
typedef struct
{
  u1 CMD                        : 8; //Комманда управления
  u1 CRD                        : 8; //CRC уникального серийного номера процессора
  u1 VP                         : 8; //Версия протокола обмена (не более максимума)
  u1 MD                         : 1; //Режим работы: 0 - Boot, 1 - APP
  u1 IND                        : 1; //Индикатор изменения состояния
  u1 CTX                        : 1; //Отклик только по CRC уникального серийного номера процессора
  u1 BTX                        : 1; //Блокировка отклика на текущий адрес, работа только по CRC уникального серийного номера процессора
  u1 RSVD                       : 4; //Резерв
}VTS_CTR;
/**************************************************************************************************/
typedef struct
{
  u1 IND                        : 8; //Состояние индикатора
  u1 RSVD                       : 24; //Резерв
}VTS_STT;
/**************************************************************************************************/
typedef struct
{
  u1 DEV                        : 8; //IDD - Тип устройства
  u1 PCB                        : 8; //PCB - Версия платы
  u1 APP                        : 8; //APP - Версия ПО
  u1 VP                         : 8; //VP - Максимальная версия протокола обмена
}VTS_DEV_ID;
/**************************************************************************************************/
struct
{
  u1 CNT                        : 3; //Количество запросов
  u1 RX_ER                      : 1; //Нет отклика в течении указаного количества запросов
  u1 RX_OK                      : 1; //Есть отклик
  u1 RX_CRC                     : 1; //Ошибка CRC
  u1 CMD                        : 8; //Комманда управления (0 - нет команд)
  u1 VP                         : 8; //Версия протокола обмена (не более максимума)
  u1 CTX                        : 1; //Отклик только по CRC уникального серийного номера процессора
  u1 BTX                        : 1; //Блокировка отклика на текущий адрес, работа только по CRC уникального серийного номера процессора
  u1 IND                        : 8; //Состояние прибора
}VTS[63]; //Состояние ВТС
///******************************* Обобщеное состояние прибора **************************************/
struct
{
  u1 ZP_OLD             : 4; //Старое состояние ZP
  u1 ZPC                : 2; //Состояние выхода ZP
  u1 SW                 : 3; //Индикатор нажатой кнопки
  u1 KEY0               : 1; //Индикатор нажатия кнопки SW
  u1 KEY1               : 3; //Счетчик времени повторных нажатий кнопки SW
  u1 KEY2               : 5; //Счетчик времени удерживания кнопки SW
  u1 KEY3               : 2; //Счетчик количества нажатий кнопки SW
  u1 ZP_FIX             : 1; //Отключение звука принудительное
  u1 NUM                : 6; //Текущий номер ВТС
  u1 RS_OPROS           : 1; //Разрешение опроса
  u1 RSVD               : 9; //Резерв
}Status;
/****************************** Обобщеное состояние каналов****************************************/
struct
{
  u1 O_ON               : 1;  //Обобщеное состояние - Дежурный режим ШСО
  u1 P_ON               : 1;  //Обобщеное состояние - Дежурный режим ШСП
  u1 O_OF               : 1;  //Обобщеное состояние - Снятый ШСО
  u1 P_OF               : 1;  //Обобщеное состояние - Снятый ШСП
  u1 AC                 : 1;  //Обобщеное состояние - Нет сети
  u1 BAT                : 1;  //Обобщеное состояние - Неисправна АКБ
  u1 BAT_LOW            : 1;  //Обобщеное состояние - Критический разряд АКБ
  u1 NR                 : 1;  //Обобщеное состояние - Нарушение
  u1 UP                 : 1;  //Обобщеное состояние - Постановка
  u1 TR                 : 1;  //Обобщеное состояние - Тревога
  u1 TF                 : 1;  //Обобщеное состояние - Тихая тревога
  u1 P1                 : 1;  //Обобщеное состояние - Пожар 1
  u1 ER                 : 1;  //Обобщеное состояние - Неисправность
  u1 P2                 : 1;  //Обобщеное состояние - Пожар 2
  u1 LED_PW             : 2;  //Состояние канала - LED_PW
  u1 LED_SR             : 2;  //Состояние канала - LED_SR
  u1 LED_C1             : 2;  //Состояние канала - LED_C1
  u1 LED_C2             : 2;  //Состояние канала - LED_C2
  u1 BUZZ               : 1;  //Состояние канала - BUZZ
  u1 LAMP               : 1;  //Состояние канала - LAMP
  u1 PCN1               : 1;  //Состояние канала - PCN1
  u1 PCN2               : 1;  //Состояние канала - PCN2
  u1 PCN3               : 1;  //Состояние канала - PCN3
  u1 LTM                : 1;  //Состояние канала - LED_TM
  u1 ZP                 : 4;  //Состояние канала - ZP
}SCH;
/**************************************************************************************************/
u8 MSTR;                                        //Режим работы: 1-Мастер, 0-слейв
u8 SW1;
u8 SW2;
u8 SW3;
u8 SW4;
//u8 VTS_ERROR;                                   //Обобщеная ошибка ВТС



u16 t_rx;
u16 t_tx;
u16 t_sw;
u16 t_zp;
u16 C_ZP;
/***************************************** USART1 *************************************************/
u16 U1_TX1;                                     //счетчик переданных символов
u16 U1_RX1;                                     //счетчик принятых символов
u8 *U1_TX2;                                     //счетчик передающего буфера
u8 *U1_RX2;                                     //счетчик приемного буфера



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
/************************************ Выводы управления *******************************************/
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

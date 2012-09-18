#include "stm32f10x_conf.h"

GPIO_InitTypeDef GPIO_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
ADC_InitTypeDef ADC_InitStructure;

#define		_IN_	GPIOB
#define		IN1		(GPIOB, GPIO_Pin_12)							// Импульсный вход 1
#define		IN2		(GPIOB, GPIO_Pin_13)							// Импульсный вход 2
#define		IN3		(GPIOB, GPIO_Pin_14)							// Импульсный вход 3
#define		IN4		(GPIOB, GPIO_Pin_15)							// Импульсный вход 4
#define		VBAT	(GPIOA, GPIO_Pin_5)								// Датчик напряжения батареи. Вроде как резистивный делитель на 3
#define		ACT		GPIOA, GPIO_Pin_3								// LED


#define		SDI		(GPIOB, GPIO_Pin_7)								// Интерфейсные линии для приемо-передатчика MRF49XA
#define		SCK		(GPIOB, GPIO_Pin_6)
#define		CS		(GPIOB, GPIO_Pin_5)
#define		SDO		(GPIOB, GPIO_Pin_4)
#define		FSEL	(GPIOB, GPIO_Pin_3)
#define		FINT	(GPIOA, GPIO_Pin_15)

//#define		T_exchange	432000										// На самом деле это кол-во 50 мс-екундных периодов в 6 часах
#define		T_exchange	500										// На самом деле это кол-во 50 мс-екундных периодов в 6 часах

#define		ON		0xff
#define		OFF		0x00

void RTC_Configuration(void);
void RTC_Restart(unsigned int);
void GPIO_Configuration(void);
void LED_Blink(void);
unsigned int ADC_Read(void);
void ADC_Calibrate(void);
void Radio_Exchange(unsigned int);
int SPI_Read(void);
void SPI_Write(int);
void SPI_Command(unsigned int);
void SPI_Write16(unsigned int);
void MRF49XA_Power_Down(void);
void MRF49XA_Init(void);
void MRF49XA_Send_Packet(unsigned char *, unsigned int);
void DelayMs( uint16_t);
unsigned short Crc16(unsigned char *, unsigned int);
void RF_GPIO_Configuration(unsigned char);

/**
  * @brief  Configures RTC clock source and prescaler.
  * @param  None
  * @retval None
  */
void RTC_Configuration(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	/* Check if the StandBy flag is set */
	if(PWR_GetFlagStatus(PWR_FLAG_SB) != RESET)
	{
		/* System resumed from STANDBY mode */
// Выключим модем
		RF_GPIO_Configuration(ON);
		MRF49XA_Power_Down();
		RF_GPIO_Configuration(OFF);

		/* Clear StandBy flag */
		PWR_ClearFlag(PWR_FLAG_SB);

		/* Wait for RTC APB registers synchronisation */
		RTC_WaitForSynchro();
		/* No need to configure the RTC as the RTC configuration(clock source, enable,
		prescaler,...) is kept after wake-up from STANDBY */

		ADC_Calibrate();
	}
	else
	{
		/* StandBy flag is not set */



	    /* Allow access to BKP Domain */
	    PWR_BackupAccessCmd(ENABLE);

		/* RTC clock source configuration ----------------------------------------*/
		/* Reset Backup Domain */
		BKP_DeInit();

		RCC_LSICmd(ENABLE);
		/* Wait till LSI is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

		/* Select the RTC Clock Source */
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

		/* Enable the RTC Clock */
		RCC_RTCCLKCmd(ENABLE);

		/* RTC configuration -----------------------------------------------------*/
		/* Wait for RTC APB registers synchronisation */
		RTC_WaitForSynchro();

		/* Set the RTC time base to 1s */
		RTC_SetPrescaler(305);//2370
		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();
		// Запишем начальное состояние контактов, якобы всё разомкнуто
		BKP_WriteBackupRegister(BKP_DR2, 0x000F);

	    PWR_BackupAccessCmd(DISABLE);
	}

	/* Запустим всё на внутреннем RC генераторе */
	RCC_HSICmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
	RCC_ClockSecuritySystemCmd(DISABLE);
	/* Отключим внешний LowSpeed генератор чтобы не жрал энергию */
	RCC_LSEConfig(DISABLE);
	/* Запретим PLL. Всё-равно не используем */
	RCC_PLLCmd(DISABLE);
	/* Запретим супервизор питания */
	PWR_PVDCmd(DISABLE);
	/* Притормозим шины */
	RCC_PCLK1Config(RCC_HCLK_Div8);
	RCC_PCLK2Config(RCC_HCLK_Div8);
	/* Притормозим всю систему в целом */
	RCC_HCLKConfig(RCC_SYSCLK_Div1);

}

/* Перезапуск будильника через время t*10 мсек */
void RTC_Restart(unsigned int t)
{
    PWR_BackupAccessCmd(ENABLE);
    RTC_SetAlarm(RTC_GetCounter()+ t);
    RTC_WaitForLastTask();
    PWR_BackupAccessCmd(DISABLE);
}

/* Конфигурируем все выводы в режим вывода и на 0.
 * В режиме STOP этим достигается минимальное энергопотребление
 * Тактирование выводов отключается !
 */
void GPIO_Configuration(void)
{
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;

GPIO_Init(GPIOA, &GPIO_InitStructure);
GPIO_Init(GPIOB, &GPIO_InitStructure);
GPIO_Init(GPIOC, &GPIO_InitStructure);

GPIO_Write(GPIOA, 0x0);
GPIO_Write(GPIOB, 0x0);
GPIO_Write(GPIOC, 0x0);

RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, DISABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, DISABLE);
}

/* Мигнём светодиодом.
 * Длительность включения 0,4 сек
 */
void LED_Blink(void)
{
	unsigned int i;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_SetBits(ACT);
    for(i=0;i<10000;i++);
    GPIO_ResetBits(ACT);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);
}


unsigned int ADC_Read(void)
{
	unsigned int i;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 ;		// that's ADC1_IN5
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//clock for ADC ( 8/2=4MHz)
	RCC_ADCCLKConfig (RCC_PCLK2_Div2);
	// enable ADC system clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// define ADC config
		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	// we work in continuous sampling mode
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 1;

		ADC_RegularChannelConfig(ADC1,ADC_Channel_5, 1,ADC_SampleTime_239Cycles5); // define regular conversion config
		ADC_Init ( ADC1, &ADC_InitStructure);	//set config of ADC1

	// enable ADC
		ADC_Cmd (ADC1,ENABLE);	//enable ADC1

	    ADC_SoftwareStartConvCmd(ADC1, ENABLE);					// start ONE conversion
	    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);	// wait end of conversion
	    i=ADC_GetConversionValue(ADC1);					// get value
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 ;		// that's ADC1_IN5
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		return(i);
}

void ADC_Calibrate(void)
{
	//clock for ADC ( 8/2=4MHz)
	RCC_ADCCLKConfig (RCC_PCLK2_Div4);
	// enable ADC system clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// при калибровке АЦП должен быть выключен
	// но когда выключаешь - калибровка встает колом
	ADC_Cmd (ADC1,ENABLE);

	//	ADC calibration (optional, but recommended at power on)
	ADC_ResetCalibration(ADC1);	// Reset previous calibration
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);								// Start new calibration (ADC must be off at that time)
	while(ADC_GetCalibrationStatus(ADC1));
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);	// выелючаем тактирование АЦП
	return;
}

int main(void)
{
	unsigned int i,j;

	/* Configure RTC clock source and prescaler */
    RTC_Configuration();
    GPIO_Configuration();

// Восстановим картину перед сбросом
    PWR_BackupAccessCmd(ENABLE);

    i = BKP_ReadBackupRegister(BKP_DR2);

// Капчим состояние внешних входов
// Хорошо бы сделать что-то в виде побитового чтение нужных ног... Но у меня удачно
// получилось, что все ноги на одном порту и подряд
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(_IN_, &GPIO_InitStructure);

    j = (GPIO_ReadInputData(_IN_) & 0xF000) >> 12;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(_IN_, &GPIO_InitStructure);
    GPIO_Write(GPIOB, 0x0);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, DISABLE);
// Получилось что-то типа 0101 или 1000, т.е. младшая тетрада значимая

// Смотрим состояние выводов
    if ( j != i  ) {

    	// А вот что-то поменялось
    	if ( ( j^(1<<3)) & (i&(1<<3)))
    			{
    				// А изменилось состояние вывода 1
    				BKP_WriteBackupRegister( BKP_DR3, BKP_ReadBackupRegister(BKP_DR3) + 1);
    			}

    	if ( (j^(1<<2)) & (i&(1<<2)))
    	    	{
    	    		// А изменилось состояние вывода 2
    	    		BKP_WriteBackupRegister( BKP_DR4, BKP_ReadBackupRegister(BKP_DR4) + 1);
    	    	}

    	if ( (j^(1<<1)) & (i&(1<<1)))
    	    	{
    	    		// А изменилось состояние вывода 3
    	    		BKP_WriteBackupRegister( BKP_DR5, BKP_ReadBackupRegister(BKP_DR5) + 1);
    	    	}

    	if ( (j^(1<<0)) & (i&(1<<0)))
    	    	{
    	    		// А изменилось состояние вывода 4
    	    		BKP_WriteBackupRegister( BKP_DR6, BKP_ReadBackupRegister(BKP_DR6) + 1);
    	    	}
    }
// Запомним последнее состояние выходов
    BKP_WriteBackupRegister(BKP_DR2, j);

// Опрос датчиков тут окончен

// Проверим - а не пора ли на связь выходить ?
// Заодно проверим напряжение питания
    i = BKP_ReadBackupRegister(BKP_DR1);
    i++;
    if ( i >= T_exchange ) {
    	i=0;
    	Radio_Exchange(j);
    }
    BKP_WriteBackupRegister(BKP_DR1, i);

    // Перезапустим будильник, чтобы "разбудил" через 50 мсек.
    RTC_Restart(5);
    /* Request to enter STANDBY mode (Wake Up flag is cleared in PWR_EnterSTANDBYMode function) */
    PWR_EnterSTANDBYMode();
    while(1);

}


void Radio_Exchange(unsigned int j)
{
	unsigned char data[9];
	unsigned int i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// 1. Сконфигурировать линни для работы с MRF49
	RF_GPIO_Configuration(ON);

	data[0] = 0;
// Напряжение батареи питания
	if ( ADC_Read() > 1000 ) data[0] |= 0x80;
// Текущее состояние датчиков
	data[0] |= (j^0x000F)&0x000F;

	i = BKP_ReadBackupRegister(BKP_DR3);
	data[1] = (i&0x0000FF00)>>8;
	data[2] = (i&0x000000FF)>>0;

	i = BKP_ReadBackupRegister(BKP_DR4);
	data[3] = (i&0x0000FF00)>>8;
	data[4] = (i&0x000000FF)>>0;

	i = BKP_ReadBackupRegister(BKP_DR5);
	data[5] = (i&0x0000FF00)>>8;
	data[6] = (i&0x000000FF)>>0;

	i = BKP_ReadBackupRegister(BKP_DR6);
	data[7] = (i&0x0000FF00)>>8;
	data[8] = (i&0x000000FF)>>0;

	// 2. Проинициализируем модем
	MRF49XA_Init();

	// 3. Попытаемся сообщить инфо на главный модуль. Если не получиться это сделать в течении
	//    трёх попыток - отвалимся, в следующий раз сообщим.
	for(i=0; i<3; i++)
	{
		GPIO_SetBits(ACT);
		// a. Отправить инфо
		MRF49XA_Send_Packet(data, 9);
		GPIO_ResetBits(ACT);
		DelayMs(100*i);
	}
	// 4. Выключим модем
	MRF49XA_Power_Down();
	// 5. Выключим линии для работы с MRF49
	RF_GPIO_Configuration(OFF);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);

	return;
}


//
// Тут процедуры для работы с радиомодемом
//

// ------------------ Процедуры работы по SPI ------------------------
// Константы для управления радиомодулем
const long    GENCREG 		= 0x8018;		// Cload=12.5pF; TX registers & FIFO are disabled
const long    PMCREG 		= 0x8201;		// Everything off, uC clk disabled
const long    RXCREG 		= 0x94A1;		// BW=135kHz, DRSSI=-97dBm, pin8=VDI, fast VDI
const long    TXBREG 		= 0xB800;
const long    FIFORSTREG	= 0xCA81;		// Sync. latch cleared, limit=8bits, disable sensitive reset
const long    BBFCREG 		= 0xC22C;		// Digital LPF (default)
const long    AFCCREG		= 0xC4D7;		// Auto AFC (default)
const long    CFSREG 		= 0xA4D8;		// Fo=433.100MHz
const long    TXCREG		= 0x9830;		// df=60kHz, Pmax, normal modulation polarity
const long    DRSREG 		= 0xC623;		// 9579Baud (default)

// Определение выводов для программного SPI
#define SPI_SDO		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)

#define SPI_SDI_0	GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define SPI_SDI_1	GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define SPI_SDI		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)

#define SPI_SCK_0	GPIO_ResetBits(GPIOB,GPIO_Pin_6)
#define SPI_SCK_1	GPIO_SetBits(GPIOB,GPIO_Pin_6)

#define SPI_CS_0	GPIO_ResetBits(GPIOB,GPIO_Pin_5)
#define SPI_CS_1	GPIO_SetBits(GPIOB,GPIO_Pin_5)

// Вывод FSEL радиомодуля
#define RF_FSEL_0	GPIO_ResetBits(GPIOB,GPIO_Pin_3);
#define RF_FSEL_1	GPIO_SetBits(GPIOB,GPIO_Pin_3);

//--------------------------------------------------------------------
// SPI_Read ()
//--------------------------------------------------------------------
int SPI_Read(void)
{
  char i;
  int spidata = 0;

  SPI_SDI_0;
  SPI_SCK_0;
  for (i=0;i<8;i=i+1)
    {
    	spidata=spidata<<1;
	if(SPI_SDO)
		spidata |= 0x01;
	else
		spidata &= 0xFE;
    SPI_SCK_1;
    SPI_SCK_0;
    }
  return(spidata);
}

//--------------------------------------------------------------------
// SPI_Write ()
//--------------------------------------------------------------------
void SPI_Write(int spidata)
{
  char i;

  SPI_SDI_0;
  SPI_SCK_0;
  for (i=0;i<8;i=i+1)
    {
    	if(spidata & 0x80)
     		SPI_SDI_1;
	else
		SPI_SDI_0;
	SPI_SCK_1;
	SPI_SCK_0;
	spidata=spidata<<1;
    }
  SPI_SDI_0;
}

//--------------------------------------------------------------------
// SPI_Command ()
//--------------------------------------------------------------------
void SPI_Command(unsigned int spicmd)
{
  SPI_CS_0;
  SPI_Write ((spicmd & 0xFF00) >> 8);
  SPI_Write ((spicmd & 0x00FF));
  SPI_CS_1;
}

//--------------------------------------------------------------------
// SPI_Write16 ()
//--------------------------------------------------------------------
void SPI_Write16(unsigned int spicmd)
{
  SPI_Write ((spicmd & 0xFF00) >> 8);
  SPI_Write ((spicmd & 0x00FF));
}

// -------------- Процедуры работы с радиомодулем --------------------
//--------------------------------------------------------------------
// Power down the radio chip
//--------------------------------------------------------------------
void MRF49XA_Power_Down(void)
{
	SPI_Command(PMCREG);
}

//--------------------------------------------------------------------
// MRF49XA_Init
// Initializes the radio chip
//--------------------------------------------------------------------
void MRF49XA_Init() {
	//----  configuring the RF link --------------------------------
	//---- Send init cmd
	SPI_Command(FIFORSTREG);
	SPI_Command(FIFORSTREG | 0x0002);
	SPI_Command(AFCCREG);
	SPI_Command(GENCREG);
	SPI_Command(CFSREG);
	SPI_Command(PMCREG);
	SPI_Command(RXCREG);
	SPI_Command(TXCREG);
	//---- antenna tunning
	SPI_Command(PMCREG | 0x0020); 			// turn on tx
	DelayMs(4);
	//---- end of antenna tunning
	SPI_Command(PMCREG); 					// turn off Tx, turn off receiver
	SPI_Command(GENCREG | 0x0040); 			// enable the FIFO
	SPI_Command(FIFORSTREG);
	SPI_Command(FIFORSTREG | 0x0002); 		// enable syncron latch
	RF_FSEL_0;
}

//--------------------------------------------------------------------
// MRF49XA_Send_Packet
// Sends a data packet
//--------------------------------------------------------------------
void MRF49XA_Send_Packet(unsigned char *data, unsigned int length) {
	int a;
	unsigned short c;

	SPI_Command(PMCREG); 					// turn off the transmitter and receiver
	SPI_Command(GENCREG | 0x0080); 			// Enable the Tx register
	SPI_Command(PMCREG | 0x0020); 			// turn on tx

	SPI_CS_0; 								// chip select low
	while (!SPI_SDO);						// Ожидаем готовности радиомодуля
	SPI_Write16(TXBREG | 0xAA); 			// preamble

	while (!SPI_SDO);
	SPI_Write16(TXBREG | 0x2D); 			// sync pattern 1st byte
	while (!SPI_SDO);
	SPI_Write16(TXBREG | 0xD4); 			// sync pattern 2nd byte

	while (!SPI_SDO);
	SPI_Write16(TXBREG | length);

	for (a = 0; a < length; a++) { 			// Отправляем буфер побайтно
		while (!SPI_SDO);
		SPI_Write16(TXBREG | data[a]); 		// запись байта в TX регистр
	}

	c = Crc16(data, length);				// Считаем CRC и отправляем в эфир
	while (!SPI_SDO);						// Отправляем младший байт CRC
	SPI_Write16(TXBREG | (c % 256));
	while (!SPI_SDO);						// Отправляем старший байт CRC
	SPI_Write16(TXBREG | (c / 256));

	// Добиваем в эфир пустой символ
	while (!SPI_SDO);
	SPI_Write16(TXBREG | 0x00);

	while (!SPI_SDO);						// Ждём окончания передачи
	SPI_CS_1;								// Освобождаем шину обмена с приемо-передатчиком
	SPI_Command(PMCREG); 			// turn off Tx, turn on the receiver
	SPI_Command(GENCREG | 0x0040); 			// disable the Tx register, Enable the FIFO
}

// Организация задержки
void DelayMs( uint16_t a) {
	volatile uint16_t b;
	while(a--)
	{
		for(b=0;b<500;b++);
	}
}

// Подсчет CRC
/*
  Name  : CRC-16 CCITT
  Poly  : 0x1021    x^16 + x^12 + x^5 + 1
  Init  : 0xFFFF
  Revert: false
  XorOut: 0x0000
  Check : 0x29B1 ("123456789")
  MaxLen: 4095 байт (32767 бит) - обнаружение
    одинарных, двойных, тройных и всех нечетных ошибок
*/
unsigned short Crc16(unsigned char *pcBlock, unsigned int len)
{
    unsigned short crc = 0xFFFF;
    unsigned char i;

    while (len--)
    {
        crc ^= *pcBlock++ << 8;

        for (i = 0; i < 8; i++)
            crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
    }

    return crc;
}

// Конфигурирование периферии
void RF_GPIO_Configuration(unsigned char state) {
	GPIO_InitTypeDef GPIO_InitStructure;

	if( state == ON )
	{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	// Конфигурирование выводов SPI и радиомодема: SDI, SCK, #CS, FSEL
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// SDO
	// PORTB.4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

    // IRQ от радиомодема (PORTA.15)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	}
	else
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, DISABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_Write(GPIOA, 0x0);
		GPIO_Write(GPIOB, 0x0);

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, DISABLE);
	}
}


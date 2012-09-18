#include "stm32f10x_conf.h"

unsigned int bl;					// Используется при радиоприеме.

// ------------------ Процедуры работы по SPI ------------------------
// Константы для управления радиомодулем
const long    GENCREG 		= 0x8018;		// Cload=12.5pF; TX registers & FIFO are disabled
const long    PMCREG 		= 0x8200;		// Everything off, uC clk enabled
const long    RXCREG 		= 0x94A1;		// BW=135kHz, DRSSI=-97dBm, pin8=VDI, fast VDI
const long    TXBREG 		= 0xB800;
const long    FIFORSTREG	= 0xCA81;		// Sync. latch cleared, limit=8bits, disable sensitive reset
const long    BBFCREG 		= 0xC22C;		// Digital LPF (default)
const long    AFCCREG		= 0xC4D7;		// Auto AFC (default)
const long    CFSREG 		= 0xA4D8;		// Fo=433.100MHz
const long    TXCREG		= 0x9830;		// df=60kHz, Pmax, normal modulation polarity
const long    DRSREG 		= 0xC623;		// 9579Baud (default)

#define			dRXBufferLong	255		// Длина буфера приемника радиомодуля. Т.е. максимальная длина пакета

// Буфер приемника радиомодуля
struct {
	unsigned char	ucData[dRXBufferLong+2];
	unsigned int	uiWR;
	unsigned int	uiPL;
} sRXBuffer;

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


unsigned short Crc16(unsigned char *, unsigned int);
void DelayMs( uint16_t );

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
// FIFO syncron latch re-enable
//--------------------------------------------------------------------
void MRF49XA_Reset_Radio()
{
	RF_FSEL_0;
	SPI_Command(PMCREG);			// turn off tx and rx
	SPI_Command(FIFORSTREG);		// reset FIFO
	SPI_Command(GENCREG);			// disable FIFO , Tx_latch
	SPI_Command(GENCREG | 0x0040);		// enable the FIFO
	SPI_Command(PMCREG | 0x0080);		// turn on receiver
	SPI_Command(FIFORSTREG | 0x0002);   	// FIFO syncron latch re-enable
	sRXBuffer.uiPL = 0;			// Отчистить буфер приемника
	sRXBuffer.uiWR = 0;			//
}
//--------------------------------------------------------------------
// Power down the radio chip
//--------------------------------------------------------------------
void MRF49XA_Power_Down(void)
{
	SPI_Command(0x8201);
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
	SPI_Command(PMCREG | 0x0080); 			// turn off Tx, turn on receiver
	SPI_Command(GENCREG | 0x0040); 			// enable the FIFO
	SPI_Command(FIFORSTREG);
	SPI_Command(FIFORSTREG | 0x0002); 		// enable syncron latch
	//  RF_FSEL_1;
	RF_FSEL_0;
	sRXBuffer.uiPL = 0; 					// Отчистить буфер приемника
	sRXBuffer.uiWR = 0; 					//
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
	SPI_Command(PMCREG | 0x0080); 			// turn off Tx, turn on the receiver
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
void RF_GPIO_Configuration(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

	// Конфигурирование выводов SPI и радиомодема: SCK, MISO, MOSI и RF_SEL
	// PORTB.12, PORTB.13, PORTB.14, PORTB.8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// PORTB.15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

    // IRQ от радиомодема (PORTB.0)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
}

// Конфигурирование контроллера прерываний
void RF_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    // Таблица векторов прерывания во FLASH
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);


    // Вывод PORTB.0 настроен на прерывание по фронту
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/******************************************************************************
  * @brief  Обработчик прерывания от приемопередатчика MRF49X
  * @param  None
  * @retval None
  * Если прилетело с воздуха что-то - собираем
*******************************************************************************/
void EXTI0_IRQHandler(void) {
	static unsigned short crc;

	if (EXTI_GetITStatus(EXTI_Line0) == SET) {
			SPI_CS_1;
			RF_FSEL_0;

			if (sRXBuffer.uiPL) {								// Не первый байт пришел... Уже в процессе приема сообщения
				sRXBuffer.ucData[sRXBuffer.uiWR] = SPI_Read();	// Читаем байт в буфер
				sRXBuffer.uiWR++;								// Продвигаем указатель в буфере
				if (sRXBuffer.uiWR >= bl ) {					// Если прочитали последний байт, то ...
					SPI_Command(FIFORSTREG);					// reset FIFO
					RF_FSEL_1;
					// Считаем CRC
					crc = sRXBuffer.ucData[bl - 1] * 256 + sRXBuffer.ucData[bl - 2];
					if ( crc == Crc16(sRXBuffer.ucData,sRXBuffer.uiPL) ) {
						MRF49XA_Send_Packet(sRXBuffer.ucData, sRXBuffer.uiPL);
					}
					// Сбрасываем радиомодуль и буфер, выключаем светодиод RADIO (1)
					MRF49XA_Reset_Radio();
				}
			} else {												// Пришел первый байт
				bl = SPI_Read();								// Принимаем его
				if ((bl > 0) && (bl < dRXBufferLong)) {			// Анализируем на допустимый размер сообщения
					sRXBuffer.uiPL = bl;						// Если допустипое значение - продолжаем. Добавим два байта CRC
					bl+=2;										// Добавляем два байта CRC при приёме
				} else {										// Если не допустимое значение - сбрасываем приемник
					MRF49XA_Reset_Radio();						// Сбрасываем радиомодуль и буфер, выключаем светодиод RADIO (1)
				}
			}

			RF_FSEL_1;
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}

void RF_main(void)
{
	MRF49XA_Init();
}

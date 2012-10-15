// Приемо-передатчик
// Новый RF для STM32F100RBT
// Коммент.
/*
 * Приемо-передатчик с UART.
 *
 * Принцип. Устройство слушает эфир и если в нём пошла нормальная посылка, то первый байт считается длиной посылки,
 * потом сама посылка, и завершает два байта CRC 16. Если CRC подсчитанный не совпадает с переданной - пакет отбрасывается.
 *
 * Если что-то прилетает по UART... Захватываем символ в буфер. И продолжаем это делать пока не прекратиться поток и не пройдет
 * таймаут (TimeOutReceivePacket  мс). Далее считается CRC 16, добавляется в конец пакета и отправляется в эфир.
*/

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_exti.h"
#include "misc.h"

// Обработчик прерывания от системного таймера
void SysTick_Handler(void);
// Программный SPI
char  SPI_Read(void);
void SPI_Write(int spidata);
void SPI_Write16(unsigned int spicmd);
void SPI_Command(unsigned int spicmd);
// Для радиомодуля MRF49XA
void MRF49XA_Init();
void MRF49XA_Send_Packet(unsigned char *data, unsigned int length);
void MRF49XA_Reset_Radio(void);
// Системные функции
void DelayMs( uint16_t);
void putstr(char *, unsigned int);
unsigned short Crc16(unsigned char *pcBlock, unsigned int len);



// Константы для управления радиомодулем
#define			GENCREG 	0x8018		// Cload=12.5pF; TX registers & FIFO are disabled
#define    		PMCREG 		0x8200		// Everything off, uC clk enabled
#define    		RXCREG 		0x94A1		// BW=135kHz, DRSSI=-97dBm, pin8=VDI, fast VDI
#define    		TXBREG 		0xB800
#define    		FIFORSTREG	0xCA81		// Sync. latch cleared, limit=8bits, disable sensitive reset
#define    		BBFCREG 	0xC22C		// Digital LPF (default)
#define    		AFCCREG		0xC4D7		// Auto AFC (default)
#define    		CFSREG 		0xA4D8		// Fo=433.100MHz
#define    		TXCREG		0x9830		// df=60kHz, Pmax, normal modulation polarity
#define    		DRSREG 		0xC623		// 9579Baud (default)

#define			dRXBufferLong	255		// Длина буфера приемника радиомодуля. Т.е. максимальная длина пакета
#define			TimeOutReceivePacket	100	// Тайм-аут при приёме по USART. В мсек.

char str[100];

// Буфер приемника радиомодуля
struct {
	unsigned char	ucData[dRXBufferLong+2];
	unsigned int	uiWR;
	unsigned int	uiPL;
} sRXBuffer;

// Буфер приемника USART
struct {
	unsigned char	ucData[dRXBufferLong];
	unsigned int	uiWR;
	unsigned int	TimeOut;
	unsigned char	check;
} sTXBuffer;


struct {
	unsigned int	id;
	unsigned short	vbat;
	unsigned char	sense;
	unsigned int	counter1;
	unsigned int	counter2;
	unsigned int	npacket;
} Counter;


unsigned int bl;					// Используется при радиоприеме.


unsigned int i;

ErrorStatus HSEStartUpStatus;

// Определение выводов для программного SPI
#define SPI_SDO		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)

#define SPI_SDI_0	GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define SPI_SDI_1	GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define SPI_SDI		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)

#define SPI_SCK_0	GPIO_ResetBits(GPIOB,GPIO_Pin_6);
#define SPI_SCK_1	GPIO_SetBits(GPIOB,GPIO_Pin_6);

#define SPI_CS_0	GPIO_ResetBits(GPIOB,GPIO_Pin_5)
#define SPI_CS_1	GPIO_SetBits(GPIOB,GPIO_Pin_5)

// Вывод FSEL радиомодуля
#define RF_FSEL_0	GPIO_ResetBits(GPIOB,GPIO_Pin_3);
#define RF_FSEL_1	GPIO_SetBits(GPIOB,GPIO_Pin_3);

// Светодиод 1
#define LED1_ON		GPIO_SetBits(GPIOA,GPIO_Pin_3);
#define LED1_OFF	GPIO_ResetBits(GPIOA,GPIO_Pin_3);


// ------------------ Процедуры работы по SPI ------------------------

//--------------------------------------------------------------------
// SPI_Read ()
//--------------------------------------------------------------------
char SPI_Read(void)
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
	LED1_OFF;				// Погасить светодиод RADIO (1)
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
	LED1_OFF; 								// Погасить светодиод RADIO (1)
}

//--------------------------------------------------------------------
// MRF49XA_Send_Packet
// Sends a data packet
//--------------------------------------------------------------------
void MRF49XA_Send_Packet(unsigned char *data, unsigned int length) {
	int a;
	unsigned short c;

	LED1_ON; 								// Включить светодиод RADIO (1)

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
	LED1_OFF;								// выключаем светодиод RADIO (1)
}

// ------------------------ Системные функции -----------------------------

// Отправить буфер в UART
void putstr(char *a, unsigned int i) {
	LED1_ON;								// включаем светодиод UART (2)
	while (i--) {							// Отправляем буфер побайтно
		while( !USART_GetFlagStatus(USART1,USART_FLAG_TXE));
		USART_SendData(USART1,*a);
		a++;
	}
	LED1_OFF;								// выключаем светодиод UART (2)
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
void GPIO_Configuration(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	// Конфигурирование выводов SPI и радиомодема: SDI, SCK, #CS, FSEL
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
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
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);

	// LED's
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART1 Rx (PA10) as input floating                         */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART1 Tx (PA9) as alternate function push-pull            */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = 9600;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);					// Разрешим прерывания от USART1 при приеме в буфер
}


// Конфигурирование контроллера прерываний
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    // Таблица векторов прерывания во FLASH
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);


    // Вывод PORTB.0 настроен на прерывание по фронту
	EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    // Включение прерывания от USART
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    // Включение прерывания от системного таймера, 1 раз в 1 мсек
    SysTick_Config(SystemCoreClock / 1000);
}

/******************************************************************************
  * @brief  Обработчик прерывания от приемопередатчика MRF49X
  * @param  None
  * @retval None
  * Если прилетело с воздуха что-то - собираем
*******************************************************************************/
void EXTI15_10_IRQHandler(void) {
	static unsigned short crc;


//	LED1_ON;
	if (EXTI_GetITStatus(EXTI_Line15) == SET) {
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

								Counter.id=sRXBuffer.ucData[0] + (sRXBuffer.ucData[1]<<8) + (sRXBuffer.ucData[2]<<16) + (sRXBuffer.ucData[3]<<24);
								Counter.vbat=sRXBuffer.ucData[4] + sRXBuffer.ucData[5]*256;
								Counter.sense = sRXBuffer.ucData[6];
								Counter.counter1=sRXBuffer.ucData[7] + (sRXBuffer.ucData[8]<<8) + (sRXBuffer.ucData[9]<<16);
								Counter.counter2=sRXBuffer.ucData[10] + (sRXBuffer.ucData[11]<<8) + (sRXBuffer.ucData[12]<<16);
								Counter.npacket = sRXBuffer.ucData[13];
				}
				// Сбрасываем радиомодуль и буфер, выключаем светодиод RADIO (1)
				MRF49XA_Reset_Radio();
			}
		} else {												// Пришел первый байт
			bl = SPI_Read();								// Принимаем его
			if ((bl > 0) && (bl < dRXBufferLong)) {			// Анализируем на допустимый размер сообщения
				LED1_ON;									// Включаем светодиод RADIO (1)
				sRXBuffer.uiPL = bl;						// Если допустипое значение - продолжаем. Добавим два байта CRC
				bl+=2;										// Добавляем два байта CRC при приёме
			} else {										// Если не допустимое значение - сбрасываем приемник
				MRF49XA_Reset_Radio();						// Сбрасываем радиомодуль и буфер, выключаем светодиод RADIO (1)
			}
		}

		RF_FSEL_1;
	}

	EXTI_ClearITPendingBit(EXTI_Line15);
}


/******************************************************************************
  * @brief  Обработчик прерывания от USART1
  * @param  None
  * @retval None
*******************************************************************************/
void USART1_IRQHandler(void) {
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		if( sTXBuffer.uiWR == 0 ) LED1_ON;						// Первый символ. Включаем светодиод UART (2)

		// Читаем байт в буфер и сдвигаем указатель
		sTXBuffer.ucData[sTXBuffer.uiWR++] = USART_ReceiveData(USART1);
		sTXBuffer.TimeOut = 0;									// Сбрасываем таймер тайм-аута

		if (sTXBuffer.uiWR >= dRXBufferLong) { 					// Если прочитали последний байт, то ...
			USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);		// Запрещаем прерывание от USART чтобы не переполнить буфер
			sTXBuffer.TimeOut = TimeOutReceivePacket;			// Таймер тайм-аута устанавливаем в переполненное состояние
		}
	}
}


/******************************************************************************
  * @brief  Обработчик прерывания от SysTick
  * @param  None
  * @retval None
  * Если в буфере UART-а что-то появилось, выжидает паузу и отправляет пакет в воздух
*******************************************************************************/
void SysTick_Handler(void) {

	if (sTXBuffer.uiWR) {										// Если начался прием по USART
		sTXBuffer.TimeOut++;									// Увеличим таймер тайм-аута
		if (sTXBuffer.TimeOut >= TimeOutReceivePacket) {		// Если произошло переполнение таймера, то отправляем пакет в радио
			USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);		// Запрещаем прерывание от USART чтобы не переполнить буфер
			sTXBuffer.check=1;
//			MRF49XA_Send_Packet(sTXBuffer.ucData, sTXBuffer.uiWR);
			sTXBuffer.uiWR = 0;									// Отчищаем буфер USART
			LED1_OFF;											// Выключаем светодиод UART (2)
			USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);		// Разрешаем прерывание от USART
		}
	}
}




int main(void) {

	SystemInit();

	Counter.id =0;
	Counter.vbat =0;
	Counter.counter1 =0;
	Counter.counter2 =0;
	Counter.sense =0;
	Counter.npacket =0;
	GPIO_Configuration();

	LED1_ON;
	DelayMs(1000);
	DelayMs(1000);
	LED1_OFF;

	NVIC_Configuration();

	MRF49XA_Init();

	while(1) {
		if( sTXBuffer.check )
		{
			sTXBuffer.check = 0;

			if( sTXBuffer.ucData[0] =='R'  ) {

				putstr(&Counter.id,4);
				putstr(&Counter.vbat,2);
				putstr(&Counter.sense,1);
				putstr(&Counter.counter1,3);
				putstr(&Counter.counter2,3);
				putstr(&Counter.npacket,1);
			}
		}
	}
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

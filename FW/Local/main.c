// ������-����������
// ����� RF ��� STM32F100RBT
// �������.
/*
 * ������-���������� � UART.
 *
 * �������. ���������� ������� ���� � ���� � �� ����� ���������� �������, �� ������ ���� ��������� ������ �������,
 * ����� ���� �������, � ��������� ��� ����� CRC 16. ���� CRC ������������ �� ��������� � ���������� - ����� �������������.
 *
 * ���� ���-�� ��������� �� UART... ����������� ������ � �����. � ���������� ��� ������ ���� �� ������������ ����� � �� �������
 * ������� (TimeOutReceivePacket  ��). ����� ��������� CRC 16, ����������� � ����� ������ � ������������ � ����.
*/

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_exti.h"
#include "misc.h"

// ���������� ���������� �� ���������� �������
void SysTick_Handler(void);
// ����������� SPI
char  SPI_Read(void);
void SPI_Write(int spidata);
void SPI_Write16(unsigned int spicmd);
void SPI_Command(unsigned int spicmd);
// ��� ����������� MRF49XA
void MRF49XA_Init();
void MRF49XA_Send_Packet(unsigned char *data, unsigned int length);
void MRF49XA_Reset_Radio(void);
// ��������� �������
void DelayMs( uint16_t);
void putstr(char *, unsigned int);
unsigned short Crc16(unsigned char *pcBlock, unsigned int len);



// ��������� ��� ���������� ������������
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

#define			dRXBufferLong	255		// ����� ������ ��������� �����������. �.�. ������������ ����� ������
#define			TimeOutReceivePacket	100	// ����-��� ��� ����� �� USART. � ����.

char str[100];

// ����� ��������� �����������
struct {
	unsigned char	ucData[dRXBufferLong+2];
	unsigned int	uiWR;
	unsigned int	uiPL;
} sRXBuffer;

// ����� ��������� USART
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


unsigned int bl;					// ������������ ��� �����������.


unsigned int i;

ErrorStatus HSEStartUpStatus;

// ����������� ������� ��� ������������ SPI
#define SPI_SDO		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)

#define SPI_SDI_0	GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define SPI_SDI_1	GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define SPI_SDI		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)

#define SPI_SCK_0	GPIO_ResetBits(GPIOB,GPIO_Pin_6);
#define SPI_SCK_1	GPIO_SetBits(GPIOB,GPIO_Pin_6);

#define SPI_CS_0	GPIO_ResetBits(GPIOB,GPIO_Pin_5)
#define SPI_CS_1	GPIO_SetBits(GPIOB,GPIO_Pin_5)

// ����� FSEL �����������
#define RF_FSEL_0	GPIO_ResetBits(GPIOB,GPIO_Pin_3);
#define RF_FSEL_1	GPIO_SetBits(GPIOB,GPIO_Pin_3);

// ��������� 1
#define LED1_ON		GPIO_SetBits(GPIOA,GPIO_Pin_3);
#define LED1_OFF	GPIO_ResetBits(GPIOA,GPIO_Pin_3);


// ------------------ ��������� ������ �� SPI ------------------------

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

// -------------- ��������� ������ � ������������ --------------------

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
	sRXBuffer.uiPL = 0;			// ��������� ����� ���������
	sRXBuffer.uiWR = 0;			//
	LED1_OFF;				// �������� ��������� RADIO (1)
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
	sRXBuffer.uiPL = 0; 					// ��������� ����� ���������
	sRXBuffer.uiWR = 0; 					//
	LED1_OFF; 								// �������� ��������� RADIO (1)
}

//--------------------------------------------------------------------
// MRF49XA_Send_Packet
// Sends a data packet
//--------------------------------------------------------------------
void MRF49XA_Send_Packet(unsigned char *data, unsigned int length) {
	int a;
	unsigned short c;

	LED1_ON; 								// �������� ��������� RADIO (1)

	SPI_Command(PMCREG); 					// turn off the transmitter and receiver
	SPI_Command(GENCREG | 0x0080); 			// Enable the Tx register
	SPI_Command(PMCREG | 0x0020); 			// turn on tx

	SPI_CS_0; 								// chip select low
	while (!SPI_SDO);						// ������� ���������� �����������
	SPI_Write16(TXBREG | 0xAA); 			// preamble

	while (!SPI_SDO);
	SPI_Write16(TXBREG | 0x2D); 			// sync pattern 1st byte
	while (!SPI_SDO);
	SPI_Write16(TXBREG | 0xD4); 			// sync pattern 2nd byte

	while (!SPI_SDO);
	SPI_Write16(TXBREG | length);

	for (a = 0; a < length; a++) { 			// ���������� ����� ��������
		while (!SPI_SDO);
		SPI_Write16(TXBREG | data[a]); 		// ������ ����� � TX �������
	}

	c = Crc16(data, length);				// ������� CRC � ���������� � ����
	while (!SPI_SDO);						// ���������� ������� ���� CRC
	SPI_Write16(TXBREG | (c % 256));
	while (!SPI_SDO);						// ���������� ������� ���� CRC
	SPI_Write16(TXBREG | (c / 256));

	// �������� � ���� ������ ������
	while (!SPI_SDO);
	SPI_Write16(TXBREG | 0x00);

	while (!SPI_SDO);						// ��� ��������� ��������
	SPI_CS_1;								// ����������� ���� ������ � ������-������������
	SPI_Command(PMCREG | 0x0080); 			// turn off Tx, turn on the receiver
	SPI_Command(GENCREG | 0x0040); 			// disable the Tx register, Enable the FIFO
	LED1_OFF;								// ��������� ��������� RADIO (1)
}

// ------------------------ ��������� ������� -----------------------------

// ��������� ����� � UART
void putstr(char *a, unsigned int i) {
	LED1_ON;								// �������� ��������� UART (2)
	while (i--) {							// ���������� ����� ��������
		while( !USART_GetFlagStatus(USART1,USART_FLAG_TXE));
		USART_SendData(USART1,*a);
		a++;
	}
	LED1_OFF;								// ��������� ��������� UART (2)
}

// ����������� ��������
void DelayMs( uint16_t a) {
	volatile uint16_t b;
	while(a--)
	{
		for(b=0;b<500;b++);
	}
}

// ������� CRC
/*
  Name  : CRC-16 CCITT
  Poly  : 0x1021    x^16 + x^12 + x^5 + 1
  Init  : 0xFFFF
  Revert: false
  XorOut: 0x0000
  Check : 0x29B1 ("123456789")
  MaxLen: 4095 ���� (32767 ���) - �����������
    ���������, �������, ������� � ���� �������� ������
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


// ���������������� ���������
void GPIO_Configuration(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	// ���������������� ������� SPI � �����������: SDI, SCK, #CS, FSEL
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// SDO
	// PORTB.4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

    // IRQ �� ����������� (PORTA.15)
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

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);					// �������� ���������� �� USART1 ��� ������ � �����
}


// ���������������� ����������� ����������
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    // ������� �������� ���������� �� FLASH
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);


    // ����� PORTB.0 �������� �� ���������� �� ������
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


    // ��������� ���������� �� USART
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    // ��������� ���������� �� ���������� �������, 1 ��� � 1 ����
    SysTick_Config(SystemCoreClock / 1000);
}

/******************************************************************************
  * @brief  ���������� ���������� �� ����������������� MRF49X
  * @param  None
  * @retval None
  * ���� ��������� � ������� ���-�� - ��������
*******************************************************************************/
void EXTI15_10_IRQHandler(void) {
	static unsigned short crc;


//	LED1_ON;
	if (EXTI_GetITStatus(EXTI_Line15) == SET) {
		SPI_CS_1;
		RF_FSEL_0;

		if (sRXBuffer.uiPL) {								// �� ������ ���� ������... ��� � �������� ������ ���������
			sRXBuffer.ucData[sRXBuffer.uiWR] = SPI_Read();	// ������ ���� � �����
			sRXBuffer.uiWR++;								// ���������� ��������� � ������
			if (sRXBuffer.uiWR >= bl ) {					// ���� ��������� ��������� ����, �� ...
				SPI_Command(FIFORSTREG);					// reset FIFO
				RF_FSEL_1;
				// ������� CRC
				crc = sRXBuffer.ucData[bl - 1] * 256 + sRXBuffer.ucData[bl - 2];
				if ( crc == Crc16(sRXBuffer.ucData,sRXBuffer.uiPL) ) {

								Counter.id=sRXBuffer.ucData[0] + (sRXBuffer.ucData[1]<<8) + (sRXBuffer.ucData[2]<<16) + (sRXBuffer.ucData[3]<<24);
								Counter.vbat=sRXBuffer.ucData[4] + sRXBuffer.ucData[5]*256;
								Counter.sense = sRXBuffer.ucData[6];
								Counter.counter1=sRXBuffer.ucData[7] + (sRXBuffer.ucData[8]<<8) + (sRXBuffer.ucData[9]<<16);
								Counter.counter2=sRXBuffer.ucData[10] + (sRXBuffer.ucData[11]<<8) + (sRXBuffer.ucData[12]<<16);
								Counter.npacket = sRXBuffer.ucData[13];
				}
				// ���������� ����������� � �����, ��������� ��������� RADIO (1)
				MRF49XA_Reset_Radio();
			}
		} else {												// ������ ������ ����
			bl = SPI_Read();								// ��������� ���
			if ((bl > 0) && (bl < dRXBufferLong)) {			// ����������� �� ���������� ������ ���������
				LED1_ON;									// �������� ��������� RADIO (1)
				sRXBuffer.uiPL = bl;						// ���� ���������� �������� - ����������. ������� ��� ����� CRC
				bl+=2;										// ��������� ��� ����� CRC ��� �����
			} else {										// ���� �� ���������� �������� - ���������� ��������
				MRF49XA_Reset_Radio();						// ���������� ����������� � �����, ��������� ��������� RADIO (1)
			}
		}

		RF_FSEL_1;
	}

	EXTI_ClearITPendingBit(EXTI_Line15);
}


/******************************************************************************
  * @brief  ���������� ���������� �� USART1
  * @param  None
  * @retval None
*******************************************************************************/
void USART1_IRQHandler(void) {
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		if( sTXBuffer.uiWR == 0 ) LED1_ON;						// ������ ������. �������� ��������� UART (2)

		// ������ ���� � ����� � �������� ���������
		sTXBuffer.ucData[sTXBuffer.uiWR++] = USART_ReceiveData(USART1);
		sTXBuffer.TimeOut = 0;									// ���������� ������ ����-����

		if (sTXBuffer.uiWR >= dRXBufferLong) { 					// ���� ��������� ��������� ����, �� ...
			USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);		// ��������� ���������� �� USART ����� �� ����������� �����
			sTXBuffer.TimeOut = TimeOutReceivePacket;			// ������ ����-���� ������������� � ������������� ���������
		}
	}
}


/******************************************************************************
  * @brief  ���������� ���������� �� SysTick
  * @param  None
  * @retval None
  * ���� � ������ UART-� ���-�� ���������, �������� ����� � ���������� ����� � ������
*******************************************************************************/
void SysTick_Handler(void) {

	if (sTXBuffer.uiWR) {										// ���� ������� ����� �� USART
		sTXBuffer.TimeOut++;									// �������� ������ ����-����
		if (sTXBuffer.TimeOut >= TimeOutReceivePacket) {		// ���� ��������� ������������ �������, �� ���������� ����� � �����
			USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);		// ��������� ���������� �� USART ����� �� ����������� �����
			sTXBuffer.check=1;
//			MRF49XA_Send_Packet(sTXBuffer.ucData, sTXBuffer.uiWR);
			sTXBuffer.uiWR = 0;									// �������� ����� USART
			LED1_OFF;											// ��������� ��������� UART (2)
			USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);		// ��������� ���������� �� USART
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

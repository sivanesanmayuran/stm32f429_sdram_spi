/**
 *	Keil project for SDRAM connected on STM32F429 Discovery board
 *
 *	Works only for STM32F429-Discovery board or STM324x9-EVAL board
 *
 *	@author		Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://stm32f4-discovery.com
 *	@ide		Keil uVision 5
 *	@packs		STM32F4xx Keil packs version 2.2.0 or greater required
 *	@stdperiph	STM32F4xx Standard peripheral drivers version 1.4.0 or greater required
 */
/* Include core modules */
#include "stm32f4xx.h"
/* Include my libraries here */
#include "defines.h"
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_sdram.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_spi.h"


//#define RAM
#define SAMPLES_PER_SPI 16
#define SAMPLES_PER_MSEC (16368/SAMPLES_PER_SPI)
#define NO_OF_MSEC 2000
#define NO_OF_DATA_ACQ (SAMPLES_PER_MSEC*NO_OF_MSEC)
volatile uint32_t buff_index=0, buff_index_mag=0, buff_index_bit2=0, tx_buff_index =0, total_bytes_transmitted=0, testing0=0, testing1=0;
#ifdef RAM
	#define RX_BUFF_SIZE (16368+8184)
	int16_t Rx_Buf[RX_BUFF_SIZE];
	int16_t Rx_Buf_mag[RX_BUFF_SIZE];
#else
	#define RX_BUFF_SIZE SDRAM_MEMORY_SIZE/2 //(16368+8184)//(16368+128)/2
#endif
int calculate_free_area(void)
{
	int retval = 0;
	if(buff_index>=tx_buff_index)
	{
		retval = (RX_BUFF_SIZE - (buff_index - tx_buff_index));
	}
	else
	{
		retval = (tx_buff_index - buff_index);
	}
	return retval;
}

void Initilize_SPI1(void)
{
        GPIO_InitTypeDef GPIO_SPI;
        SPI_InitTypeDef My_SPI;
        NVIC_InitTypeDef nvic;

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	
				GPIO_SPI.GPIO_Pin=GPIO_Pin_4 |  GPIO_Pin_5| GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_SPI.GPIO_Mode=GPIO_Mode_AF;
        GPIO_SPI.GPIO_PuPd=GPIO_PuPd_NOPULL;
        GPIO_SPI.GPIO_OType=GPIO_OType_PP ;
        GPIO_SPI.GPIO_Speed=GPIO_Speed_100MHz;
        GPIO_Init(GPIOA, &GPIO_SPI);
				GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI1);
				GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
				GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
        

        SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
        nvic.NVIC_IRQChannel=SPI1_IRQn;
        nvic.NVIC_IRQChannelCmd = ENABLE;
        nvic.NVIC_IRQChannelPreemptionPriority= 0;
        NVIC_Init(&nvic);

        My_SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        My_SPI.SPI_Mode = SPI_Mode_Slave;
        My_SPI.SPI_DataSize = SPI_DataSize_16b;
        My_SPI.SPI_CPOL = SPI_CPOL_High;
        My_SPI.SPI_CPHA = SPI_CPHA_2Edge;
        My_SPI.SPI_NSS = SPI_NSS_Hard;
        My_SPI.SPI_FirstBit = SPI_FirstBit_MSB;

        SPI_Init(SPI1, &My_SPI);
        SPI_Cmd(SPI1, ENABLE);
}

void Initilize_SPI2(void)
{
        GPIO_InitTypeDef GPIO_SPI;
        SPI_InitTypeDef My_SPI;
        NVIC_InitTypeDef nvic;

        RCC_APB2PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	
				GPIO_SPI.GPIO_Pin=GPIO_Pin_12 |  GPIO_Pin_13| GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_SPI.GPIO_Mode=GPIO_Mode_AF;
        GPIO_SPI.GPIO_PuPd=GPIO_PuPd_NOPULL;
        GPIO_SPI.GPIO_OType=GPIO_OType_PP ;
        GPIO_SPI.GPIO_Speed=GPIO_Speed_100MHz;
        GPIO_Init(GPIOA, &GPIO_SPI);
				GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_SPI1);
				GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI1);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI1);
				GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI1);
        

        SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
        nvic.NVIC_IRQChannel=SPI2_IRQn;
        nvic.NVIC_IRQChannelCmd = ENABLE;
        nvic.NVIC_IRQChannelPreemptionPriority= 0;
        NVIC_Init(&nvic);

        My_SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        My_SPI.SPI_Mode = SPI_Mode_Slave;
        My_SPI.SPI_DataSize = SPI_DataSize_16b;
        My_SPI.SPI_CPOL = SPI_CPOL_High;
        My_SPI.SPI_CPHA = SPI_CPHA_2Edge;
        My_SPI.SPI_NSS = SPI_NSS_Hard;
        My_SPI.SPI_FirstBit = SPI_FirstBit_MSB;

        SPI_Init(SPI2, &My_SPI);
        SPI_Cmd(SPI2, ENABLE);
}

void SPI2_IRQHandler()
{
	int16_t dummy;
		if(buff_index_mag <NO_OF_DATA_ACQ)
		{
			//if(calculate_free_area()>0)
			{
				testing0++;
				buff_index_mag++;
#ifdef RAM				
        Rx_Buf[buff_index]=SPI_I2S_ReceiveData(SPI2);
#else
				TM_SDRAM_Write16(buff_index, SPI_I2S_ReceiveData(SPI2));
#endif
        SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_RXNE);
        SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_RXNE);
        //buff_index++;// = buff_index+1;//
        //if(buff_index>=RX_BUFF_SIZE)
        //       buff_index=0;
			}
		}
		else
		{
			dummy = SPI_I2S_ReceiveData(SPI2);
			SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_RXNE);
			SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_RXNE);
		}

}

void Initilize_SPI1_B(void)
{
        GPIO_InitTypeDef GPIO_SPI;
        SPI_InitTypeDef My_SPI;
        NVIC_InitTypeDef nvic;

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

        GPIO_SPI.GPIO_Pin=GPIO_Pin_3 | GPIO_Pin_4 |  GPIO_Pin_5;
        GPIO_SPI.GPIO_Mode=GPIO_Mode_AF;
        GPIO_SPI.GPIO_PuPd= GPIO_PuPd_NOPULL;
        GPIO_SPI.GPIO_OType=GPIO_OType_PP ;
        GPIO_SPI.GPIO_Speed=GPIO_Speed_100MHz;
        GPIO_Init(GPIOB, &GPIO_SPI);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);
	
				GPIO_SPI.GPIO_Pin=GPIO_Pin_4;
        GPIO_SPI.GPIO_Mode=GPIO_Mode_AF;
        GPIO_SPI.GPIO_PuPd=GPIO_PuPd_NOPULL;
        GPIO_SPI.GPIO_OType=GPIO_OType_PP ;
        GPIO_SPI.GPIO_Speed=GPIO_Speed_100MHz;
        GPIO_Init(GPIOA, &GPIO_SPI);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI1);

        SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
        nvic.NVIC_IRQChannel=SPI1_IRQn;
        nvic.NVIC_IRQChannelCmd = ENABLE;
        nvic.NVIC_IRQChannelPreemptionPriority= 0;
        NVIC_Init(&nvic);

        My_SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        My_SPI.SPI_Mode = SPI_Mode_Slave;
        My_SPI.SPI_DataSize = SPI_DataSize_16b;
        My_SPI.SPI_CPOL = SPI_CPOL_High;
        My_SPI.SPI_CPHA = SPI_CPHA_2Edge;
        My_SPI.SPI_NSS = SPI_NSS_Hard;
        My_SPI.SPI_FirstBit = SPI_FirstBit_MSB;

        SPI_Init(SPI1, &My_SPI);
        SPI_Cmd(SPI1, ENABLE);
}

void Initilize_SPI3(void)
{
        GPIO_InitTypeDef GPIO_SPI;
        SPI_InitTypeDef My_SPI;
        NVIC_InitTypeDef nvic;

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

        GPIO_SPI.GPIO_Pin=GPIO_Pin_10 | GPIO_Pin_11 |  GPIO_Pin_12;
        GPIO_SPI.GPIO_Mode=GPIO_Mode_AF;
        GPIO_SPI.GPIO_PuPd= GPIO_PuPd_NOPULL;
        GPIO_SPI.GPIO_OType=GPIO_OType_PP ;
        GPIO_SPI.GPIO_Speed=GPIO_Speed_100MHz;
        GPIO_Init(GPIOC, &GPIO_SPI);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
	
				GPIO_SPI.GPIO_Pin=GPIO_Pin_15;
        GPIO_SPI.GPIO_Mode=GPIO_Mode_AF;
        GPIO_SPI.GPIO_PuPd=GPIO_PuPd_NOPULL;
        GPIO_SPI.GPIO_OType=GPIO_OType_PP ;
        GPIO_SPI.GPIO_Speed=GPIO_Speed_100MHz;
        GPIO_Init(GPIOA, &GPIO_SPI);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_SPI3);

        SPI_I2S_ITConfig(SPI3, SPI_I2S_IT_RXNE, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
        nvic.NVIC_IRQChannel=SPI3_IRQn;
        nvic.NVIC_IRQChannelCmd = ENABLE;
        nvic.NVIC_IRQChannelPreemptionPriority= 0;
        NVIC_Init(&nvic);

        My_SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        My_SPI.SPI_Mode = SPI_Mode_Slave;
        My_SPI.SPI_DataSize = SPI_DataSize_16b;
        My_SPI.SPI_CPOL = SPI_CPOL_High;
        My_SPI.SPI_CPHA = SPI_CPHA_2Edge;
        My_SPI.SPI_NSS = SPI_NSS_Hard;
        My_SPI.SPI_FirstBit = SPI_FirstBit_MSB;

        SPI_Init(SPI3, &My_SPI);
        SPI_Cmd(SPI3, ENABLE);
}

void SPI1_IRQHandler()
{
	int16_t dummy;
		if(buff_index_mag <NO_OF_DATA_ACQ)
		{
			//if(calculate_free_area()>0)
			{
				testing0++;
				buff_index_mag++;
#ifdef RAM				
        Rx_Buf[buff_index]=SPI_I2S_ReceiveData(SPI1);
#else
				TM_SDRAM_Write16(buff_index, SPI_I2S_ReceiveData(SPI1));
#endif
        SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_RXNE);
        SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_RXNE);
        //buff_index++;// = buff_index+1;//
        //if(buff_index>=RX_BUFF_SIZE)
        //       buff_index=0;
			}
		}
		else
		{
			dummy = SPI_I2S_ReceiveData(SPI1);
			SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_RXNE);
			SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_RXNE);
		}

}

void SPI3_IRQHandler()
{
	int16_t dummy;
	if(buff_index_mag <NO_OF_DATA_ACQ)
	{
		//if(calculate_free_area()>0)
		{
			testing1++;
#ifdef RAM	
			Rx_Buf_mag[buff_index]=SPI_I2S_ReceiveData(SPI3);
			buff_index++;
#else			
			TM_SDRAM_Write16(buff_index+SDRAM_MEMORY_SIZE/2, SPI_I2S_ReceiveData(SPI3));
			buff_index = buff_index+2;
#endif
			SPI_I2S_ClearFlag(SPI3, SPI_I2S_FLAG_RXNE);
			SPI_I2S_ClearITPendingBit(SPI3, SPI_I2S_IT_RXNE);
		
			//if(buff_index >= RX_BUFF_SIZE) 
			//	buff_index = 0;
		}
	}
	else
	{
		dummy = SPI_I2S_ReceiveData(SPI3);
		SPI_I2S_ClearFlag(SPI3, SPI_I2S_FLAG_RXNE);
    SPI_I2S_ClearITPendingBit(SPI3, SPI_I2S_IT_RXNE);
	}
}


void USART1_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	USART_InitStructure.USART_BaudRate = 921600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE);
	
}

volatile int  rotate =0;
volatile uint32_t start_time;
volatile uint32_t time_stamp = 0;
int main(void) {


	GPIO_InitTypeDef GPIO_InitStruct;
	uint16_t data0, data1;
	char bit0,bit1, bitpos, val;
	int i,data;
	
	char ADC_data[] = {	 1/*00*/, 
											 3/*01*/, 
											-1/*10*/, 
											-3/*11*/
										};     //{3, 5, -3, -5};
	
	/* Initialize delay */
	SysTick_Config(SystemCoreClock/1000);//TM_DELAY_Init();

//#ifndef RAM		
	/* Initialize SDRAM */

//#endif	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOG, &GPIO_InitStruct);
	
	__disable_irq();
	//SysTick_Config(SystemCoreClock/1000);
	USART1_Configuration();
	
	Initilize_SPI1();
	//Initilize_SPI2();
	//Initilize_SPI1_B();
	Initilize_SPI3();
	if (TM_SDRAM_Init()) 
	{
		for(i=0;i<SDRAM_MEMORY_SIZE/4;i++) TM_SDRAM_Write32(i, 0xA5A5A5A5);//RAM clear
		GPIO_ToggleBits(GPIOG, GPIO_Pin_13);//Error:sdram init error
	} 
	Initilize_SPI1();
	
	buff_index_mag = 0;
	buff_index = 0;
	buff_index_bit2 =0;
	tx_buff_index = 0;
	testing0= testing1 = 0;
	total_bytes_transmitted = 0;
	data = 0;
	//for(i=0;i<SDRAM_MEMORY_SIZE/4;i++) TM_SDRAM_Write32(i, 0);//RAM clear
	__enable_irq();
	
	//Flat buffer implementation: This shoud solve the misalignment issue
	while(1)
	{
			__disable_irq();
			buff_index = 0;
			buff_index_mag = 0;
			testing0= testing1 = 0;
			data = 0;
		
			__enable_irq();
			start_time = time_stamp;
			while(testing1<NO_OF_DATA_ACQ);
			//Delayms(NO_OF_MSEC);//Time for acquiring the data
			rotate = time_stamp - start_time;
			__disable_irq();
#ifdef RAM	
		if(
			(testing0 == testing1)  
			&& (buff_index == testing0)  //single irq => 2 bytes
			//&& (testing0 > (SAMPLES_PER_MSEC*NO_OF_MSEC-1)) //giving 1msec tollerance
		)
#else		
		if(
			(testing0 == testing1)  
			&& (buff_index == testing0*2)  //single irq => 2 bytes
			//&& (testing0 > (SAMPLES_PER_MSEC*(NO_OF_MSEC-1))) //giving 1msec tollerance
		)
#endif		
		{
#ifdef RAM	
			GPIO_ToggleBits(GPIOG, GPIO_Pin_13);
			for(data=0;data<buff_index;data++)
#else			
			GPIO_ToggleBits(GPIOG, GPIO_Pin_13);
			for(data=0;data<buff_index;data += 2)
#endif
			{
				/*
				//8 bit read from SDRAM
				data0 = TM_SDRAM_Read8(data);
				data1 = TM_SDRAM_Read8(data+SDRAM_MEMORY_SIZE/2);
				for(bitpos=0;bitpos<8;bitpos++)
				{
					bit0 = bit1 = 0;
					if(data0 & (1<<bitpos)) bit0 = 1;//(Rx_Buf[data] & (1<<bitpos));
					if(data1 & (1<<bitpos)) bit1 = 1;//(Rx_Buf_mag[data] & (1<<bitpos));
					val = bit1<<1 | bit0;
					USART_SendData(USART1, ADC_data[val]);
					USART_WAIT(USART1);
				}*/
				
				//16 bit read from SDRAM
#ifdef RAM	
				data0 = Rx_Buf[data];//
				data1 = Rx_Buf_mag[data];//
#else
				data0 = TM_SDRAM_Read16(data);//
				data1 = TM_SDRAM_Read16(data+SDRAM_MEMORY_SIZE/2);//
#endif
				for(bitpos=0;bitpos<8;bitpos++)
				{
					bit0 = bit1 = 0;
					if(data0 & (1<<bitpos)) bit0 = 1;//(Rx_Buf[data] & (1<<bitpos));
					if(data1 & (1<<bitpos)) bit1 = 1;//(Rx_Buf_mag[data] & (1<<bitpos));
					val = bit1<<1 | bit0;
					USART_SendData(USART1, ADC_data[val]);
					USART_WAIT(USART1);
					tx_buff_index++;
				}
				for(bitpos=8;bitpos<16;bitpos++)
				{
					bit0 = bit1 = 0;
					if(data0 & (1<<bitpos)) bit0 = 1;//(Rx_Buf[data] & (1<<bitpos));
					if(data1 & (1<<bitpos)) bit1 = 1;//(Rx_Buf_mag[data] & (1<<bitpos));
					val = bit1<<1 | bit0;
					USART_SendData(USART1, ADC_data[val]);
					USART_WAIT(USART1);
					tx_buff_index++;
				}
				
			}
			__enable_irq();
			GPIO_ToggleBits(GPIOG, GPIO_Pin_13);//Finished: Successall bytes received OK.
		}
		else
		{
			__enable_irq();
			GPIO_ToggleBits(GPIOG, GPIO_Pin_13);//Error: ata mismatch
		}
	}
	
}

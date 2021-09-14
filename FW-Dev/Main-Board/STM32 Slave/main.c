/*
PERIPHERALS ON SLAVE
3 I2C 			1 UART(2 UART)	

Motherboard Slave I2C devices
1. Expiratory differential flow sensor - SDP31 (B10=SCL , B11=SDA, I2C2)
2. Proximal differential flow sensor - SDP31/SDP8 (C9=SDA , A8=SCL, I2C3)
3. Proximal pressure sensor - DLHR-L60G (B9=SDA , B8=SCL, I2C1) (B6=SCL, B7=SDA, I2C1)

//MASTER-SLAVE//
UART 6
C6=Tx
C7=Rx
*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "hal_gpio_driver.h"
#include "hal_uart_driver.h"

#define PLLM 8
#define PLLN 336
#define PLLP 0 																	//PLLP=2
#define PLLQ 7

//I2C VARIABLES////////////////////////////
float PROX_Pressure_Data[2];
float EXP_SDP31_Data[2];
float PROX_SDP31_Data[2];

//UART VARIABLES////////////////////////////
uart_handle_t uart_handle;

uint8_t UDMA4_BUFF[100];
uint8_t UDMA4_TX_BUFF[11];

uint8_t UDMA6_BUFF[100];
uint8_t UDMA6_TX_BUFF[11];

uint8_t Master_Slave_Send[11];
uint8_t Power_MCU_Send[11];

void Clock_Config (void)
{
//HSE
	RCC->CR |= RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY));
	
//ENABLE POWER AND REGULATOR
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;
	
	//FLASH
	 FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
	
	//PRESCALERS
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
	
	//CONFIGURE PLL
	RCC->PLLCFGR = (PLLM<<0) | (PLLN<<6) | (PLLP<<16) | (PLLQ<<24) | (RCC_PLLCFGR_PLLSRC_HSE);
	
	
//ENABLE PLL
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));

	//SET PLL AS CLOCK SOURCE
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void Timer2_Init(void)
{
	RCC->APB1ENR |= (1<<0);
	TIM2->PSC = 83;
	TIM2->ARR = 0xffff;
	TIM2->CR1 |= (1<<0);
	while(!(TIM2->SR & (1<<0)));	
}

void TIMER_DELAY_us(uint32_t delay)
{
	TIM2->CNT = 0;
	while(TIM2->CNT < delay);
}

void TIMER_DELAY_ms(uint32_t delay)
{
	TIM2->CNT = 0;
	for(uint32_t n = 0; n<delay; n++)
	{
		TIMER_DELAY_us(1000);
	}
}
	
void Delay (uint32_t time)
{
while(time--);
}
void I2C3_Config(void)
{
/*
	1. enable i2c clock and gpio clock
	2. configure i2c pins for alternate function
		a. alternate gunction
		b. opendrain o/p
		c. high speed for pins
		d. select pull-up
		e. alternate pin in afr register 
	3. reset the i2c
	4. program the peripheral input clock in i2c register to generate timming
	5. configure clock control register
	6. configure rise time register
	7. program I2C_CR1 register to enable peripheral
	*/
	//Clear reset pin
	//I2C1->CR1 &= ~(1<<15);
	
	//enable i2c CLOCK
	RCC->AHB1ENR |= (1<<2);										//PORT C
	RCC->AHB1ENR |= (1<<0);										//PORT A
	RCC->APB1ENR |= (1<<23);									//I2C3
	
	
	//Clear reset pin
	//I2C1->CR1 &= ~(1<<15);
	
	// CONFIGURE I2C PINS
	GPIOC->MODER |= (2<<18);  			//C9=SDA ALTERNATE FUNCTIONS
	GPIOA->MODER |= (2<<16);				//A8=SCL ALTERNATE FUNCTIONS
	GPIOC->OTYPER |= (1<<9);				//OPEN DRAIN OUTPUT
	GPIOA->OTYPER |= (1<<8);				//OPEN DRAIN OUTPUT
	GPIOC->OSPEEDR |= (1<<19);			//HIGH SPEED OUTPUT
	GPIOA->OSPEEDR |= (1<<17);			//HIGH SPEED OUTPUT
	GPIOC->PUPDR |= 0;												//PULLUP DISABLE
	GPIOA->PUPDR |= 0;												//PULLUP DISABLE
	GPIOC->AFR[1] |= (4<<4);									//AF4 IN C9
	GPIOA->AFR[1] |= (4<<0);									//AF4 IN A8
	
	//RESET I2C
	//I2C1->CR1 |= (1<<15);
	//I2C1->CR1 &= ~(1<<15);
	
	//PROGRAM PERIPHERAL INPUT CLOCK IN I2C 
	I2C3->CR2 |= (40<<0);											//PERIPHERAL CLOCK IN MHZ (40MHZ)

	//CONFIGURE CCR
	//for 100kHz SCL, Thigh = 1/200khz and pclk = 40Mhz,  ccr=100
	I2C3->CCR = 210<<0;							//51				//100khz
	 	
	//RISE TIME 
	I2C3->TRISE = 41;
		
	//DISABLE CLOCK STRETCH
	//I2C1->CR1 |= (1<<7);
		
	//ENABLE I2C PERIPHERAL
	I2C3->CR1 |= (1<<0);		
		
	//ENABLE ACK
	I2C3->CR1 |= (1<<10);
}

void I2C2_Config(void)
{
/*
	1. enable i2c clock and gpio clock
	2. configure i2c pins for alternate function
		a. alternate gunction
		b. opendrain o/p
		c. high speed for pins
		d. select pull-up
		e. alternate pin in afr register 
	3. reset the i2c
	4. program the peripheral input clock in i2c register to generate timming
	5. configure clock control register
	6. configure rise time register
	7. program I2C_CR1 register to enable peripheral
	*/
	//Clear reset pin
	//I2C1->CR1 &= ~(1<<15);
	
	//enable i2c CLOCK
	RCC->AHB1ENR |= (1<<1);										//PORT B:
	RCC->APB1ENR |= (1<<22);									//I2C2
	
	
	//Clear reset pin
	//I2C1->CR1 &= ~(1<<15);
	
	// CONFIGURE I2C PINS
	GPIOB->MODER |= (2<<20) | (2<<22);  			// B10=SCL, B11=SDA ALTERNATE FUNCTIONS
	GPIOB->OTYPER |= (1<<10) | (1<<11);				//OPEN DRAIN OUTPUT
	GPIOB->OSPEEDR |= (1<<21) | (1<<23);			//HIGH SPEED OUTPUT
	GPIOB->PUPDR |= 0;												//PULLUP DISABLE
	GPIOB->AFR[1] |= (4<<8);									//AF4 IN B10
	GPIOB->AFR[1] |= (4<<12);									//AF4 IN B11
	
	//RESET I2C
	//I2C1->CR1 |= (1<<15);
	//I2C1->CR1 &= ~(1<<15);
	
	//PROGRAM PERIPHERAL INPUT CLOCK IN I2C 
	I2C2->CR2 |= (40<<0);											//PERIPHERAL CLOCK IN MHZ (40MHZ)

	//CONFIGURE CCR
	//for 100kHz SCL, Thigh = 1/200khz and pclk = 40Mhz,  ccr=100
	I2C2->CCR = 210<<0;							//51				//100khz
	 	
	//RISE TIME 
	I2C2->TRISE = 41;
		
	//DISABLE CLOCK STRETCH
	//I2C1->CR1 |= (1<<7);
		
	//ENABLE I2C PERIPHERAL
	I2C2->CR1 |= (1<<0);		
		
	//ENABLE ACK
	I2C2->CR1 |= (1<<10);
}

void I2C1_Config(void)
{
/*
	1. enable i2c clock and gpio clock
	2. configure i2c pins for alternate function
		a. alternate gunction
		b. opendrain o/p
		c. high speed for pins
		d. select pull-up
		e. alternate pin in afr register 
	3. reset the i2c
	4. program the peripheral input clock in i2c register to generate timming
	5. configure clock control register
	6. configure rise time register
	7. program I2C_CR1 register to enable peripheral
	*/
	//Clear reset pin
	//I2C1->CR1 &= ~(1<<15);
	
	//enable i2c CLOCK
	RCC->AHB1ENR |= (1<<1);										//PORT B:
	RCC->APB1ENR |= (1<<21);									//I2C1
	
	
	//Clear reset pin
	//I2C1->CR1 &= ~(1<<15);
	
	// CONFIGURE I2C PINS
	// CONFIGURE I2C PINS
	//GPIOB->MODER |= (2<<16) | (2<<18);  			// B8=SCL, B9=SDA ALTERNATE FUNCTIONS
	GPIOB->MODER |= (2<<12) | (2<<14);  			// B6=SCL, B7=SDA ALTERNATE FUNCTIONS
	//GPIOB->OTYPER |= (1<<8) | (1<<9);					//OPEN DRAIN OUTPUT
	GPIOB->OTYPER |= (1<<6) | (1<<7);					//OPEN DRAIN OUTPUT
	//GPIOB->OSPEEDR |= (1<<17) | (1<<19);			//HIGH SPEED OUTPUT
	GPIOB->OSPEEDR |= (1<<13) | (1<<15);			//HIGH SPEED OUTPUT
	GPIOB->PUPDR |= 0;												//PULLUP DISABLE
	//GPIOB->AFR[1] |= (4<<0);									//AF4 IN B8
	//GPIOB->AFR[1] |= (4<<4);									//AF4 IN B9
	GPIOB->AFR[0] |= (4<<24);									//AF4 IN B6
	GPIOB->AFR[0] |= (4<<28);									//AF4 IN B7
	
	//RESET I2C
	//I2C1->CR1 |= (1<<15);
	//I2C1->CR1 &= ~(1<<15);
	
	//PROGRAM PERIPHERAL INPUT CLOCK IN I2C 
	I2C1->CR2 |= (40<<0);											//PERIPHERAL CLOCK IN MHZ (40MHZ)

	//CONFIGURE CCR
	//for 100kHz SCL, Thigh = 1/200khz and pclk = 40Mhz,  ccr=100
	I2C1->CCR = 210<<0;							//51				//100khz
	 	
	//RISE TIME 
	I2C1->TRISE = 41;
		
	//DISABLE CLOCK STRETCH
	//I2C1->CR1 |= (1<<7);
		
	//ENABLE I2C PERIPHERAL
	I2C1->CR1 |= (1<<0);		
		
	//ENABLE ACK
	I2C1->CR1 |= (1<<10);
}

void I2C_Reset(uint16_t n)
{
	if(n==1)
	{
		I2C1->CR1 |= (1<<15);
		TIMER_DELAY_us(1000);
		I2C1->CR1 &= ~(1<<15);
		I2C1_Config();
	}
	
	if(n==2)
	{
		I2C2->CR1 |= (1<<15);
		TIMER_DELAY_us(1000);
		I2C2->CR1 &= ~(1<<15);
		I2C2_Config();
	}
	
	if(n==3)
	{
		I2C3->CR1 |= (1<<15);
		TIMER_DELAY_us(1000);
		I2C3->CR1 &= ~(1<<15);
		I2C3_Config();
	}
}

void UART_GPIO_Init(void)
{
	gpio_pin_conf_t uart_pin_conf;
	
  /*enable the clock for the GPIO port A */ 
	 _HAL_RCC_GPIOC_CLK_ENABLE();

	//##################----UART6----################
	/*configure the GPIO_PORT_C_PIN_6 as TX */
	uart_pin_conf.pin = USART6_TX_PIN;
	uart_pin_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	uart_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	uart_pin_conf.speed =  GPIO_PIN_SPEED_HIGH;
	uart_pin_conf.pull  =  GPIO_PIN_NO_PULL_PUSH;
	hal_gpio_set_alt_function(GPIOC,USART6_TX_PIN,USART6_TX_AF);
	hal_gpio_init(GPIOC ,&uart_pin_conf);
	
	/*configure the GPIO_PORT_C_PIN_7 as RX */
	uart_pin_conf.pin = USART6_RX_PIN;
	hal_gpio_set_alt_function(GPIOC,USART6_RX_PIN,USART6_RX_AF);
	hal_gpio_init(GPIOC ,&uart_pin_conf);	
	
	//##################----UART4----################
	/*configure the GPIO_PORT_C_PIN_10 as TX */
	uart_pin_conf.pin = USART4_TX_PIN;
	hal_gpio_set_alt_function(GPIOC,USART4_TX_PIN,USART4_TX_AF);
	hal_gpio_init(GPIOC ,&uart_pin_conf);
	
	/*configure the GPIO_PORT_C_PIN_11 as RX */
	uart_pin_conf.pin = USART4_RX_PIN;
	hal_gpio_set_alt_function(GPIOC,USART4_RX_PIN,USART4_RX_AF);
	hal_gpio_init(GPIOC ,&uart_pin_conf);
}

void UART_4_init(void)
{
  _HAL_RCC_UART4_CLK_ENABLE();
	_HAL_RCC_DMA1_CLK_ENABLE() ;
	//_HAL_RCC_DMA2_CLK_ENABLE();
		
	
	uart_handle.Instance          = USART_4;

	uart_handle.Init.BaudRate     = USART_BAUD_115200 ;
	uart_handle.Init.WordLength   = USART_WL_1S8B;
	uart_handle.Init.StopBits     = USART_STOP_BITS_1;
	uart_handle.Init.Parity       = UART_PARITY_NONE;
	uart_handle.Init.Mode         = UART_MODE_TX_RX;
	uart_handle.Init.OverSampling = USART_OVER16_ENABLE;
	
	//##########################---UART_DMA_RX---###################
	
   	
		DMA1_Stream2->M0AR =(uint32_t)UDMA4_BUFF; //memary address
		DMA1_Stream2->PAR =(uint32_t)&UART4->DR;//peri address
		DMA1_Stream2->NDTR = 100;
		
		DMA1_Stream2->FCR |= (1<<2);			//direct mode off
		DMA1_Stream2->FCR =0x3<<0;				//fifo mode on
		
	  DMA1_Stream2->CR &= ~(0x4<<25);
	  DMA1_Stream2->CR |=(0x4<<25);  		//CHANNEL SELECTION 4 BY 100
	 
	  DMA1_Stream2->CR &= ~(1<<10);
	  DMA1_Stream2->CR |=(1<<10);				//MEM INCREMENT ENABLE

		DMA1_Stream2->CR |= (0x1<<16);		//PRIORITY MODERATE
		
		DMA1_Stream2->CR &= ~(1<<4);			//TC COMPLETE INTRRUPT ENABLE
		DMA1_Stream2->CR |= (1<<4);
	
		DMA1_Stream2->CR &= ~(1<<0);			//STREAM ENABLE IN THE END
		DMA1_Stream2->CR |=(1<<0);
	 
	//#################---DMA-RX-END--###############################
	//###################---DMA-TX-ST--################################  
	  DMA1_Stream4->M0AR =(uint32_t) UDMA4_TX_BUFF; //memary address
		DMA1_Stream4->PAR =(uint32_t)&UART4->DR;//peri address
		DMA1_Stream4->NDTR = 11;
	
	  DMA1_Stream4->FCR |= (1<<2);			//direct mode off
		DMA1_Stream4->FCR =0x3<<0;				//fifo mode on
		
		DMA1_Stream4->CR &= ~(0x4<<25);
	  DMA1_Stream4->CR |=(0x4<<25);  		//CHANNEL SELECTION 4 BY 100
		
		DMA1_Stream4->CR &= ~(1<<10);//
	  DMA1_Stream4->CR |=(1<<10);				//MEM INCREMENT ENABLE
		
		DMA1_Stream4->CR |= (0x01<<6);		//DATA TRANFAR MEM TO PERI 
		
		DMA1_Stream4->CR &= ~(1<<4);			//TC COMPLETE INTRRUPT ENABLE
		DMA1_Stream4->CR |= (1<<4);
	
		DMA1_Stream4->CR &= ~(1<<0);			//STREAM ENABLE IN THE END
		DMA1_Stream4->CR |=(1<<0);
	
	//#################---DMA-TX-END--###############################
	
	 hal_uart_init(&uart_handle);
	// USART_2->SR |=(1<<5);
	 USART_4->CR1 &=~(1<<4);
	
   NVIC_SetPriority(USART4_IRQn_EN, 2);  

	 NVIC_EnableIRQ(USART4_IRQn_EN);
	 
	 NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	 NVIC_EnableIRQ(DMA1_Stream2_IRQn);
 }

void UART_6_init(void)
{
  _HAL_RCC_USART6_CLK_ENABLE();
	//_HAL_RCC_DMA1_CLK_ENABLE() ;
	_HAL_RCC_DMA2_CLK_ENABLE();
		
	
	uart_handle.Instance          = USART_6;

	uart_handle.Init.BaudRate     = USART_BAUD_115200 ;
	uart_handle.Init.WordLength   = USART_WL_1S8B;
	uart_handle.Init.StopBits     = USART_STOP_BITS_1;
	uart_handle.Init.Parity       = UART_PARITY_NONE;
	uart_handle.Init.Mode         = UART_MODE_TX_RX;
	uart_handle.Init.OverSampling = USART_OVER16_ENABLE;
	
	//##########################---UART_DMA_RX---###################
	
   	
		DMA2_Stream1->M0AR =(uint32_t)UDMA6_BUFF; //memary address
		DMA2_Stream1->PAR =(uint32_t)&USART6->DR;//peri address
		DMA2_Stream1->NDTR = 100;
		
		DMA2_Stream1->FCR |= (1<<2);			//direct mode off
		DMA2_Stream1->FCR =0x3<<0;				//fifo mode on
		
	  DMA2_Stream1->CR &= ~(0x5<<25);
	  DMA2_Stream1->CR |=(0x5<<25);  		//CHANNEL SELECTION 4 BY 100
	 
	  DMA2_Stream1->CR &= ~(1<<10);
	  DMA2_Stream1->CR |=(1<<10);				//MEM INCREMENT ENABLE
	 
		DMA2_Stream1->CR |= (0x1<<16);		//PRIORITY MODERATE
		
		DMA2_Stream1->CR &= ~(1<<4);			//TC COMPLETE INTRRUPT ENABLE
		DMA2_Stream1->CR |= (1<<4);
	
		DMA2_Stream1->CR &= ~(1<<0);			//STREAM ENABLE IN THE END
		DMA2_Stream1->CR |=(1<<0);
	 
	//#################---DMA-RX-END--###############################
	//###################---DMA-TX-ST--################################  
	  DMA2_Stream6->M0AR =(uint32_t) UDMA6_TX_BUFF; //memary address
		DMA2_Stream6->PAR =(uint32_t)&USART6->DR;//peri address
		DMA2_Stream6->NDTR = 11;
	
	  DMA2_Stream6->FCR |= (1<<2);			//direct mode off
		DMA2_Stream6->FCR =0x3<<0;				//fifo mode on
		
		DMA2_Stream6->CR &= ~(0x5<<25);
	  DMA2_Stream6->CR |=(0x5<<25);  		//CHANNEL SELECTION 4 BY 100
	 
	  
		DMA2_Stream6->CR &= ~(1<<10);
	  DMA2_Stream6->CR |=(1<<10);				//MEM INCREMENT ENABLE
		
		DMA2_Stream6->CR |= (0x01<<6);		//DATA TRANFAR MEM TO PERI 
		
		DMA2_Stream6->CR &= ~(1<<4);			//TC COMPLETE INTRRUPT ENABLE
		DMA2_Stream6->CR |= (1<<4);
	
		DMA2_Stream6->CR &= ~(1<<0);			//STREAM ENABLE IN THE END
		DMA2_Stream6->CR |=(1<<0);
	
	//#################---DMA-TX-END--###############################
	
	 hal_uart_init(&uart_handle);
	// USART_2->SR |=(1<<5);
	 USART_6->CR1 &=~(1<<4);
	
   NVIC_SetPriority(USART6_IRQn_EN, 2);  

	 NVIC_EnableIRQ(USART6_IRQn_EN);
	 
	 NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	 NVIC_EnableIRQ(DMA2_Stream6_IRQn);
 }

void UART_Packet_Init(void)
{
	///ENABLE UART RX-TX DMA/////////////////////////////////////	
	USART_4->CR3 |= (1<<6);//ENABLE DMA BIT FOR TX
	USART_4->CR3 |= (1<<7);//ENABLE DMA BIT FOR RX
	
	USART_6->CR3 |= (1<<6);//ENABLE DMA BIT FOR TX
	USART_6->CR3 |= (1<<7);//ENABLE DMA BIT FOR RX
	
	///POWER MCU PACKET INITIALIZE///////////////////////////////
	Power_MCU_Send[0] =  0x23;		
	Power_MCU_Send[1] =  0xF5;		
	Power_MCU_Send[2] =	 0xF5;		
	Power_MCU_Send[3] =	 0xF5;		
	Power_MCU_Send[4] =	 0xF5;		
	Power_MCU_Send[5] =  0xF5;			
	Power_MCU_Send[6] =	 0xF5;		
	Power_MCU_Send[7] =	 0xF5;		
	Power_MCU_Send[8] =	 0xF5;		
	Power_MCU_Send[9] =	 0xF5;
	Power_MCU_Send[10] = 0x40;
	
	///MASTER-SLAVE PACKET INITIALIZE////////////////////////////
	Master_Slave_Send[0] =  0x23;		
	Master_Slave_Send[1] = 	0xF5;		
	Master_Slave_Send[2] =	0xF5;		
	Master_Slave_Send[3] =	0xF5;		
	Master_Slave_Send[4] =	0xF5;		
	Master_Slave_Send[5] = 	0xF5;			
	Master_Slave_Send[6] =	0xF5;		
	Master_Slave_Send[7] =	0xF5;		
	Master_Slave_Send[8] =	0xF5;		
	Master_Slave_Send[9] =	0xF5;
	Master_Slave_Send[10] = 0x40;

	///PUSH PACKETS TO DMA BUFFER////////////////////////////////	
	memcpy(UDMA4_TX_BUFF,Power_MCU_Send,11);
	memcpy(UDMA6_TX_BUFF,Master_Slave_Send,11);	
}
 
void Make_Packet(uint32_t mode, uint32_t para_id, float data)
{
	uint8_t Sign_byte = 0;
	uint8_t Byte_Zero = 0;
	uint8_t Byte_One = 0;
	uint8_t Byte_Two = 0;
	int Mantisa = 0;
	int Abs_Mantisa = 0;
	 
	Mantisa = 1000*data;
	Abs_Mantisa = abs(Mantisa);
	if((Mantisa/Abs_Mantisa) == -1)
		Sign_byte = 0x01;
	else
		Sign_byte = 0x00;
	Byte_Zero = Abs_Mantisa;
	Byte_One = Abs_Mantisa>>8;
	Byte_Two = Abs_Mantisa>>16;
	
	Master_Slave_Send[1] = mode;
	Master_Slave_Send[2] = para_id;
	Master_Slave_Send[3] = Sign_byte;
	Master_Slave_Send[4] = Byte_Two;
	Master_Slave_Send[5] = Byte_One;
	Master_Slave_Send[6] = Byte_Zero;
 }

 void Packet_Parser(uint8_t data[])
{
	//PARSING VARIABLES/////////////////////////////////////
	uint32_t Mode = 0;
	uint32_t Para_ID = 0;
	float Payload = 0.0;
	uint8_t Sign_Byte = 0;
	int Exponent = 1000;
	
	for(int i=0;i<100;i++)	
		{
			if(data[i]==0x23 && data[i+1]==0x32 && data[i+10]==0x40)
			{
				Para_ID = (data[i+2]);
				Sign_Byte = (data[i+3]);
				Payload = (data[i+4]<<16)+(data[i+5]<<8)+(data[i+6]);
				if(Para_ID==POWER_SEND)
				{
					Power_MCU_Send[1] =  0xBB;		
					Power_MCU_Send[2] =	 0xBB;		
					Power_MCU_Send[3] =	 0xBB;		
					Power_MCU_Send[4] =	 0xF5;		
					Power_MCU_Send[5] =  0xA9;					
					Power_MCU_Send[9] =	 Payload;
				}
				if(data[i]==0x23 && data[i+10]==0x40)
				{
					Mode = (data[i+1]<<16)+(data[i+2]<<8)+(data[i+3]);
					Para_ID =(data[i+4]<<8)+(data[i+5]);
					Payload = (data[i+6]<<24)+(data[i+7]<<16)+(data[i+8]<<8)+(data[i+9]);
					
					if(Mode==0xBBBBBB)
					{
						Master_Slave_Send[1] = data[i+1];
						Master_Slave_Send[2] = data[i+2];
						Master_Slave_Send[3] = data[i+3];
						Master_Slave_Send[4] = data[i+4];
						Master_Slave_Send[5] = data[i+5];
						Master_Slave_Send[6] = data[i+6];
						Master_Slave_Send[7] = data[i+7];
						Master_Slave_Send[8] = data[i+8];
						Master_Slave_Send[9] = data[i+9];
					}
				}				
			}
		}
}

_Bool I2C_Read_Timeout(uint16_t n)								
{
	uint32_t Timeout = 100000;
	_Bool status = 1;
	if(n==1)
	{
			while(!(I2C1->SR1 & (1<<6)))							//(I2C1->SR1 & 0x40) == 0x00
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(1);
					status = 0;
					return status;
				}
			}
		}
	if(n==2)
	{
			while(!(I2C2->SR1 & (1<<6)))							//(I2C1->SR1 & 0x40) == 0x00
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(2);
					status = 0;
					return status;
				}
			}
		}
	if(n==3)
	{
			while(!(I2C3->SR1 & (1<<6)))							//(I2C1->SR1 & 0x40) == 0x00
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(3);
					status = 0;
					return status;
				}
			}
	}
	return status;
}

_Bool I2C_Bus_Busy_Timeout(uint16_t n)								
{
	uint32_t Timeout = 100000;
	_Bool status = 1;
	if(n==1)
	{
			while(I2C1->SR2 & (1<<1))							//(I2C1->SR2 & 0x02) == 0x02
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(1);
					status = 0;
					return status;
				}
			}
		}
	if(n==2)
	{
			while(I2C2->SR2 & (1<<1))							//(I2C1->SR2 & 0x02) == 0x02
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(2);
					status = 0;
					return status;
				}
			}
		}
	if(n==3)
	{
			while(I2C3->SR2 & (1<<1))							//(I2C1->SR2 & 0x02) == 0x02
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(3);
					status = 0;
					return status;
				}
			}
	}
	return status;
}
_Bool I2C_Startbit_Timeout(uint16_t n)
{
	uint32_t Timeout = 100000;
	_Bool status = 1;
	if(n==1)
	{
			while(!(I2C1->SR1 & (1<<0)))									//(I2C1->SR1 & 0x01)==0x00
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(1);
					status = 0;
					return status;
				}
			}
		}
	if(n==2)
	{
		while(!(I2C2->SR1 & (1<<0)))									//(I2C1->SR1 & 0x01)==0x00
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(2);
					status = 0;
					return status;
				}
			}
	}
	if(n==3)
	{
		while(!(I2C3->SR1 & (1<<0)))									//(I2C1->SR1 & 0x01)==0x00
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(3);
					status = 0;
					return status;
				}
			}
		}
	return status;
}

_Bool I2C_Write_Timeout(uint16_t n)
{
	uint32_t Timeout = 100000;
	_Bool status = 1;
	if(n==1)
	{
		while(!(I2C1->SR1 & (1<<7)))								//(I2C1->SR1 & 0x80)==0x00
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(1);
					status = 0;
					return status;
				}
			}
	}
	if(n==2)
	{
			while(!(I2C2->SR1 & (1<<7)))								//(I2C1->SR1 & 0x80)==0x00
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(2);
					status = 0;
					return status;
				}
			}
	}
	if(n==3)
	{
			while(!(I2C3->SR1 & (1<<7)))								//(I2C1->SR1 & 0x80)==0x00
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(3);
					status = 0;
					return status;
				}
			}
	}
	return status;
}

_Bool I2C_Byte_transfer_Timeout(uint16_t n)
{
	uint32_t Timeout = 100000;
	_Bool status = 1;

	if(n==1)
	{
			while(!(I2C1->SR1 & (1<<2)))								//(I2C1->SR1 & 0x03)==0x00
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(1);
					status = 0;
					return status;
				}
			}
		}
	if(n==2)
	{
			while(!(I2C2->SR1 & (1<<2)))								//(I2C1->SR1 & 0x03)==0x00
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(2);
					status = 0;
					return status;
				}
			}
		}
	if(n==3)
	{
			while(!(I2C3->SR1 & (1<<2)))								//(I2C1->SR1 & 0x03)==0x00
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(3);
					status = 0;
					return status;
				}
			}
		}
	
	return status;
}

_Bool I2C_Address_Transmission_Timeout(uint16_t n)
{
	uint32_t Timeout = 100000;
	_Bool status = 1;
	uint8_t Temp = 0;
	if(n==1)
	{
			while(!(I2C1->SR1 & (1<<1)))								//(I2C1->SR1 & 0x02)==0x00
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(1);
					status = 0;
					return status;
				}
			}
			//dummy read
			Temp = I2C1->SR1 | I2C1->SR2;
	}
			
		if(n==2)
	{
			while(!(I2C2->SR1 & (1<<1)))								//(I2C1->SR1 & 0x02)==0x00
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(2);
					status = 0;
					return status;
				}
			}
			//dummy read
			Temp = I2C2->SR1 | I2C2->SR2;
	}
			
	if(n==3)
	{
			while(!(I2C3->SR1 & (1<<1)))								//(I2C1->SR1 & 0x02)==0x00
			{
				Timeout -= 1;
				if(Timeout == 0)
				{
					I2C_Reset(3);
					status = 0;
					return status;
				}
			}
			//dummy read
			Temp = I2C3->SR1 | I2C3->SR2;
	}
			
	return status;
}

void DMA1_Stream2_rx_callback(void)
{
	  USART_4->CR3 &= ~(1<<6);//DISBLE DMA BIT FOR RX
    DMA1_Stream2->NDTR = 100;//LENGTH OF RX
		DMA1_Stream2->CR &= ~(1<<0);//STREAM ENABLE IN THE END
		DMA1_Stream2->CR |=(1<<0);//
   
	  USART_4->CR3 |= (1<<6);//ENABLE DMA BIT FOR RX
}
void DMA1_Stream4_tx_callback(void)
{
	  USART_4->CR3 &= ~(1<<7);//DISBLE DMA BIT FOR TX
   
		DMA1_Stream4->NDTR = 11; //LENGTH OF TX
		DMA1_Stream4->CR &= ~(1<<0);//STREAM ENABLE IN THE END
		DMA1_Stream4->CR |=(1<<0);
   
    USART_4->CR3 |= (1<<7);//ENABLE DMA BIT FOR TX
    memcpy(UDMA4_TX_BUFF,Power_MCU_Send,11);
}


void DMA2_Stream1_rx_callback(void)
{
	  USART_6->CR3 &= ~(1<<6);//DISBLE DMA BIT FOR RX
    
		DMA2_Stream1->NDTR = 100;//LENGTH OF RX
		DMA2_Stream1->CR &= ~(1<<0);//STREAM ENABLE IN THE END
		DMA2_Stream1->CR |=(1<<0);//
   
	  USART_6->CR3 |= (1<<6);//ENABLE DMA BIT FOR RX
}
void DMA2_Stream6_tx_callback(void)
{
	  USART_6->CR3 &= ~(1<<7);//DISBLE DMA BIT FOR TX
   
		DMA2_Stream6->NDTR = 11; //LENGTH OF TX
		DMA2_Stream6->CR &= ~(1<<0);//STREAM ENABLE IN THE END
		DMA2_Stream6->CR |=(1<<0);
   
    USART_6->CR3 |= (1<<7);//ENABLE DMA BIT FOR TX
    memcpy(UDMA6_TX_BUFF,Master_Slave_Send,11);
}	

void DMA2_Stream6_IRQHandler (void)
{
 if(DMA2->HISR&(1<<21))
 {
 
	DMA2->HIFCR |= (1<<21);
	DMA2->HIFCR |= (1<<20);	 
	DMA2_Stream6_tx_callback();
 }
}

void DMA2_Stream1_IRQHandler (void)
{
 if(DMA2->LISR&(1<<11))
 {
	DMA2->LIFCR |= (1<<11);
	DMA2->LIFCR |= (1<<10);	 
	DMA2_Stream1_rx_callback();
 }
}

void DMA1_Stream4_IRQHandler (void)
{
 if(DMA1->HISR&(1<<5))
 {
 
	DMA1->HIFCR |= (1<<5);
	DMA1->HIFCR |= (1<<4);	 
	DMA1_Stream4_tx_callback();
 }
}

void DMA1_Stream2_IRQHandler (void)
{
 if(DMA1->LISR&(1<<21))
 {
	DMA1->LIFCR |= (1<<21);
	DMA1->LIFCR |= (1<<20);	 
	DMA1_Stream2_rx_callback();
 }
}

void SDP31_I2C2_Init(void)																					//Expiratory diff pressure
{												
	uint8_t Command1 = 0x36;															
	uint8_t Command0 = 0x08;
	//uint8_t Read_Address = 0x4B;
	uint8_t Write_Address = 0x4A;
	uint8_t Reset_Address = 0x00;
	uint8_t Reset_Command = 0x06;
//	uint8_t I2C2_Data_array[9];
//	int Scale_Factor_SDP31_Pascal = 60;
//	int Scale_Factor_SDP31_DegC = 200;
//	float Diff_Pressure = 0;
//	float Temperature = 0;
//	const float Compliance = 0.134247851;
//	const float Sq_Coeff = 0.00076976;
//	const float Var_Coeff = 0.553016781;
//	const float Constant = -0.267837698;
//	float Error = 0;
//	float Flow = 0;
	
	//ENABLE ACK
	I2C2->CR1 |= (1<<10);
	
	if(!(I2C_Bus_Busy_Timeout(2)))
		return ;
	
	//Start generation 																									//Reset routine start
	I2C2->CR1 |= (1<<8);
	
	if(!(I2C_Startbit_Timeout(2)))
		return ;
	
	//Reset sensor
	I2C2->DR = Reset_Address;

	if(!(I2C_Address_Transmission_Timeout(2)))
		 return ;
	
	if(!(I2C_Write_Timeout(2)))
		return ;
	//command for reset mode
	I2C2->DR = Reset_Command;

	if(!(I2C_Byte_transfer_Timeout(2)))
		return ;
	
	//Stop
	I2C2->CR1 |= (1<<9);

	TIMER_DELAY_ms(25);																									//!!!!!!!!!DANGER!!!!!!!!!!! DO NOT CHANGE //default value 25ms
	
	//Start generation 
	I2C2->CR1 |= (1<<8);
	
	if(!(I2C_Startbit_Timeout(2)))
		return ;
	
	//Write command to sensor
	I2C2->DR = Write_Address;

	if(!(I2C_Address_Transmission_Timeout(2)))
		 return ;
	
	if(!(I2C_Write_Timeout(2)))
		return ;
	//command for continuous measurement mode
	I2C2->DR = Command1;

	if(!(I2C_Byte_transfer_Timeout(2)))
		return ;
	
	if(!(I2C_Write_Timeout(2)))
		return ;
	//command for continuous measurement mode
	I2C2->DR = Command0;

	if(!(I2C_Byte_transfer_Timeout(2)))
		return ;
	
	//Stop
	I2C2->CR1 |= (1<<9);

	TIMER_DELAY_ms(10);
	
//	//Start generation 
//	I2C2->CR1 |= (1<<8);
//	if(!(I2C_Startbit_Timeout(2)))
//		return ;
//	
//	//READ ADDRESS
//	I2C2->DR = Read_Address;		
//	
//	if(!(I2C_Address_Transmission_Timeout(2)))
//		return ;
//	
//	for(uint8_t Count = 0; Count<9; Count++)
//		{
//		if(!(I2C_Read_Timeout(2)))
//			return ;
//			I2C2_Data_array[Count] = I2C2->DR;
//		}
//		
//	//Stop
//	I2C2->CR1 |= (1<<9);

//	TIMER_DELAY_ms(950);
//		
//	Diff_Pressure = (I2C2_Data_array[0]<<8 | I2C2_Data_array[1]) ;
//	Diff_Pressure = Diff_Pressure / Scale_Factor_SDP31_Pascal ;
//		
//	Error = Sq_Coeff * (Diff_Pressure * Compliance) * (Diff_Pressure * Compliance) + Var_Coeff * (Diff_Pressure * Compliance) + Constant;
//	
//	Flow = (Diff_Pressure * Compliance) - Error ;
//		
//	Temperature = (I2C2_Data_array[3]<<8 | I2C2_Data_array[4]) ;
//	Temperature = Temperature / Scale_Factor_SDP31_DegC ;
//		
//	EXP_SDP31_Data[0] = Flow;
//	EXP_SDP31_Data[1] = Temperature;

}

void SDP31_I2C3_Init(void)																					//Proximal diff pressure	
{												
	uint8_t Command1 = 0x36;																						//SDP81	addresses						
	uint8_t Command0 = 0x08;
//	uint8_t Read_Address = 0x4B;
	uint8_t Write_Address = 0x4A;
	uint8_t Reset_Address = 0x00;
	uint8_t Reset_Command = 0x06;
//	int Scale_Factor_SDP31_Pascal = 60;
//	int Scale_Factor_SDP31_DegC = 200;
//	uint8_t I2C3_Data_array[9];
//	float Diff_Pressure = 0;
//	float Temperature = 0;
//	const float Compliance = 0.134247851;
//	const float Sq_Coeff = 0.00076976;
//	const float Var_Coeff = 0.553016781;
//	const float Constant = -0.267837698;
//	float Error = 0;
//	float Flow = 0;
	
	//ENABLE ACK
	I2C3->CR1 |= (1<<10);
	
	if(!(I2C_Bus_Busy_Timeout(3)))
		return ;
	
	//Start generation 																									//Reset routine
	I2C3->CR1 |= (1<<8);
	
	if(!(I2C_Startbit_Timeout(3)))
		return ;
	
	//Reset sensor
	I2C3->DR = Reset_Address;

	if(!(I2C_Address_Transmission_Timeout(3)))
		 return ;
	
	if(!(I2C_Write_Timeout(3)))
		return ;
	//command for reset mode
	I2C3->DR = Reset_Command;

	if(!(I2C_Byte_transfer_Timeout(3)))
		return ;
	
	//Stop
	I2C3->CR1 |= (1<<9);

	TIMER_DELAY_ms(25);																									//!!!!!!!!!DANGER!!!!!!!!!!! DO NOT CHANGE //default value 25ms
	
	//Start generation 
	I2C3->CR1 |= (1<<8);
	
	if(!(I2C_Startbit_Timeout(3)))
		return ;
	
	//Write command to sensor
	I2C3->DR = Write_Address;

	if(!(I2C_Address_Transmission_Timeout(3)))
		 return ;
	
	if(!(I2C_Write_Timeout(3)))
		return ;
	//command for continuous measurement mode
	I2C3->DR = Command1;

	if(!(I2C_Byte_transfer_Timeout(3)))
		return ;
	
	if(!(I2C_Write_Timeout(3)))
		return ;
	//command for continuous measurement mode
	I2C3->DR = Command0;

	if(!(I2C_Byte_transfer_Timeout(3)))
		return ;
	
	//Stop
	I2C3->CR1 |= (1<<9);

	TIMER_DELAY_ms(10);
	
//	//Start generation 
//	I2C3->CR1 |= (1<<8);
//	if(!(I2C_Startbit_Timeout(3)))
//		return ;
//	
//	//READ ADDRESS
//	I2C3->DR = Read_Address;		
//	
//	if(!(I2C_Address_Transmission_Timeout(3)))
//		return ;
//	
//	for(uint8_t Count = 0; Count<9; Count++)
//		{
//		if(!(I2C_Read_Timeout(3)))
//			return ;
//			I2C3_Data_array[Count] = I2C3->DR;
//		}
//		
//	//Stop
//	I2C3->CR1 |= (1<<9);

//	TIMER_DELAY_ms(950);
//		
//	Diff_Pressure = (I2C3_Data_array[0]<<8 | I2C3_Data_array[1]) ;
//	Diff_Pressure = Diff_Pressure/Scale_Factor_SDP31_Pascal ;
//		
//	Error = Sq_Coeff * (Diff_Pressure * Compliance) * (Diff_Pressure * Compliance) + Var_Coeff * (Diff_Pressure * Compliance) + Constant;
//	
//	Flow = (Diff_Pressure * Compliance) - Error ;
//		
//	Temperature = (I2C3_Data_array[3]<<8 | I2C3_Data_array[4]) ;
//	Temperature = Temperature/Scale_Factor_SDP31_DegC ;
//		
//	PROX_SDP31_Data[0] = Flow;
//	PROX_SDP31_Data[1] = Temperature;
}

void DLHR_I2C1_sensor_data(void)																		//Proximal pressure sensor
{
	uint8_t Read_Address = 0x53;
	uint8_t Write_Address = 0x52;
	uint8_t Command = 0xAA;
	uint8_t Data_array[7];
	int Pres = 0;
	int Temp = 0;
	const float Bit_Factor = 16777216.0;
	int Offset_Pressure = 1677722;																		//0.5*2^24 for differential , 1677722 for gauge
	const float Conversion_Factor = 2.53746;
	const float Conversion_constant = 75.0/16777216.0;
	
	//ENABLE ACK
	I2C1->CR1 |= (1<<10);
	
	if(!(I2C_Bus_Busy_Timeout(1)))
		return ;
	
	//Start generation 
	I2C1->CR1 |= (1<<8);
	
	//while(!(I2C1->SR1 & (1<<0)));
	if(!(I2C_Startbit_Timeout(1)))
		return ;
	
	//Write command to sensor
	I2C1->DR = Write_Address;
	
	//while(!(I2C1->SR1 & (1<<1)));
	if(!(I2C_Address_Transmission_Timeout(1)))
		 return ;
	
	//while(!(I2C1->SR1 & (1<<7)));
	if(!(I2C_Write_Timeout(1)))
		return ;
	
	//command for single start measurement mode
	I2C1->DR = Command;
	
	//while(!(I2C1->SR1 & (1<<2)));
	if(!(I2C_Byte_transfer_Timeout(1)))
		return ;
	
	//Stop
	I2C1->CR1 |= (1<<9);
	
	//Start generation
	I2C1->CR1 |= (1<<8);
	
	//while(!(I2C1->SR1 & (1<<0)));
	if(!(I2C_Startbit_Timeout(1)))
		return ;
	
	//read data from sensor
	I2C1->DR = Read_Address;
	
	//while(!(I2C1->SR1 & (1<<1)));
	if(!(I2C_Address_Transmission_Timeout(1)))
		return ;
	
	//Read Data
	for(uint8_t Count = 0; Count<7; Count++)
	{
		if(!(I2C_Read_Timeout(1)))
			return ;
		//while(!(I2C1->SR1 & (1<<6)));
		Data_array[Count] = I2C1->DR;
		if(Count==6)
		{
			I2C1->CR1 &= ~(1<<10);																			//disable ACK to receive NACK for command
		}
	}
	
	//Stop
	I2C1->CR1 |= (1<<9);
	
	//Calculate Pressure
	Pres = (Data_array[1]<<16 | Data_array[2]<<8 | Data_array[3]);
	Pres = Conversion_constant * (Pres - Offset_Pressure) * Conversion_Factor / Bit_Factor; 						
	
	Temp = (Data_array[4]<<16 | Data_array[5]<<8 | Data_array[6]);
	Temp = (Temp * 125 / Bit_Factor) - 40;																//Transfer function applied as per datasheet 
	
	PROX_Pressure_Data[0] = Pres;
	PROX_Pressure_Data[1] = Temp;
}

void EXP_Flow_I2C2_Data(void)
{
	uint8_t Read_Address = 0x4B;
	uint8_t I2C2_Data_array[9];
	int Scale_Factor_SDP31_Pascal = 60;
	int Scale_Factor_SDP31_DegC = 200;
	float Diff_Pressure = 0;
	float Temperature = 0;
	const float Compliance = 0.134247851;
	const float Sq_Coeff = 0.00076976;
	const float Var_Coeff = 0.553016781;
	const float Constant = -0.267837698;
	float Error = 0;
	float Flow = 0;
	
	//Start generation 
	I2C2->CR1 |= (1<<8);
	if(!(I2C_Startbit_Timeout(2)))
		return ;
	
	//READ ADDRESS
	I2C2->DR = Read_Address;		
	
	if(!(I2C_Address_Transmission_Timeout(2)))
		return ;
	
	for(uint8_t Count = 0; Count<9; Count++)
		{
		if(!(I2C_Read_Timeout(2)))
			return ;
			I2C2_Data_array[Count] = I2C2->DR;
		}
		
	//Stop
	//I2C2->CR1 |= (1<<9);
		
	Diff_Pressure = (I2C2_Data_array[0]<<8 | I2C2_Data_array[1]) ;
	Diff_Pressure = Diff_Pressure / Scale_Factor_SDP31_Pascal ;
		
	Error = Sq_Coeff * (Diff_Pressure * Compliance) * (Diff_Pressure * Compliance) + Var_Coeff * (Diff_Pressure * Compliance) + Constant;
	
	Flow = (Diff_Pressure * Compliance) - Error ;
		
	Temperature = (I2C2_Data_array[3]<<8 | I2C2_Data_array[4]) ;
	Temperature = Temperature / Scale_Factor_SDP31_DegC ;
		
	EXP_SDP31_Data[0] = Flow;
	EXP_SDP31_Data[1] = Temperature;
}

void PROX_Flow_I2C3_Data(void)
{
	uint8_t Read_Address = 0x4B;
	uint8_t I2C3_Data_array[9];
	float Diff_Pressure = 0;
	float Temperature = 0;
	int Scale_Factor_SDP31_Pascal = 60;
	int Scale_Factor_SDP31_DegC = 200;
	const float Compliance = 0.134247851;
	const float Sq_Coeff = 0.00076976;
	const float Var_Coeff = 0.553016781;
	const float Constant = -0.267837698;
	float Error = 0;
	float Flow = 0;
	
	//Start generation 
	I2C3->CR1 |= (1<<8);
	if(!(I2C_Startbit_Timeout(3)))
		return ;
	
	//READ ADDRESS
	I2C3->DR = Read_Address;		
	
	if(!(I2C_Address_Transmission_Timeout(3)))
		return ;
	
	for(uint8_t Count = 0; Count<9; Count++)
		{
		if(!(I2C_Read_Timeout(3)))
			return ;
			I2C3_Data_array[Count] = I2C3->DR;
		}
		
	//Stop
	//I2C3->CR1 |= (1<<9);
		
	Diff_Pressure = (I2C3_Data_array[0]<<8 | I2C3_Data_array[1]) ;
	Diff_Pressure = Diff_Pressure/Scale_Factor_SDP31_Pascal ;
		
	Error = Sq_Coeff * (Diff_Pressure * Compliance) * (Diff_Pressure * Compliance) + Var_Coeff * (Diff_Pressure * Compliance) + Constant;
	
	Flow = (Diff_Pressure * Compliance) - Error ;
		
	Temperature = (I2C3_Data_array[3]<<8 | I2C3_Data_array[4]) ;
	Temperature = Temperature/Scale_Factor_SDP31_DegC ;
		
	PROX_SDP31_Data[0] = Flow;
	PROX_SDP31_Data[1] = Temperature;
}

int main(void)
{
	///CLOCK, TIMERS INITIALIZE////////////////////////////
	Clock_Config();
	Timer2_Init();
	
	///GPIO INITIALIZE/////////////////////////////////////
	UART_GPIO_Init();
	
	///I2C INITIALIZE//////////////////////////////////////
	I2C1_Config();
	I2C2_Config();
	I2C3_Config();
	
	///I2C INITIALIZE//////////////////////////////////////
	SDP31_I2C2_Init();
	SDP31_I2C3_Init();
	
	///UART INITIALIZE/////////////////////////////////////
	UART_4_init();
	UART_6_init();
	UART_Packet_Init();

	TIMER_DELAY_ms(100);
	while(1)
	{
		///SENSOR DATA//////////////////////////////////////
		EXP_Flow_I2C2_Data();
		PROX_Flow_I2C3_Data();
		DLHR_I2C1_sensor_data();
		
		
		///MASTER-SLAVE COMMUNICATION///////////////////////
		//Make_Packet(SLAVE_MASTER, PROX_FLOW, PROX_SDP31_Data[0]);
		Make_Packet(SLAVE_MASTER, EXP_FLOW, EXP_SDP31_Data[0]);
		//Make_Packet(SLAVE_MASTER, PROX_PRESSURE, PROX_Pressure_Data[0]);
		
		
		///PARSE INCOMING DATA//////////////////////////////
		Packet_Parser(UDMA6_BUFF);
		Packet_Parser(UDMA4_BUFF);
	}	
}

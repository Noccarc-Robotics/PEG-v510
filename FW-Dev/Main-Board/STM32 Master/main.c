/*
Peripherals on main board
3 I2C 			3 UART					2 DAC					4 ADC				
														A4,A5					A7,C0,C1,C2

//SOM BOARD//				//POWER BOARD//			//MASTER-SLAVE//
UART 1							UART 4					 		UART 6
(B6=Tx)	A9=Tx				C10=Tx							C6=Tx
(B7=Rx)	A10=Rx			C11=Rx							C7=Rx

DMA 2 							DMA	1								DMA 2
CH4--STREAM7=Tx			CH4--STREAM4=Tx			CH5--STREAM6 =Tx					
CH4--STREAM5=Rx			CH4--STREAM2=Rx			CH5--STREAM1 =Rx	


Master board I2C
Sensors:
1. Inlet air flow sensor - SFM3019 (B10=SCL , B11=SDA, I2C2)
2. Inlet O2 flow sensor - SFM3019 (C9=SDA , A8=SCL, I2C3)
3. Respiratory pressure sensor - DLHR-L60G (B9=SDA , B8=SCL, I2C1) (B6=SCL, B7=SDA, I2C1)


6 global varibles for sensors
2 variables of proportional valves
5 valve signals
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
#define PLLP 0 													//PLLP=2
#define PLLQ 7

//ADC VARIABLES//////////////////////////////////////////
uint16_t ADC_DMA_BUFFER[4];
float HPS_O2_Data = 0;
float HPS_Air_Data = 0;
float O2_Sensor_Data = 0;

//I2C VARIABLES/////////////////////////////////////////
float Respiratory_Pressure[2] ;
float Air_Flow_Data[2];
float O2_Flow_Data[2];

//I2C VARIABLES FROM SLAVE//////////////////////////////
float PROX_Pressure_Data[2];
float EXP_SDP31_Data[2];
float PROX_SDP31_Data[2];

//FLOW COMPENSATION VARIABLES///////////////////////////
const int Pabs = 101325;
const float Dv_SFM = 900.639;
const float Ph20_SFM = 415.616;
const float Ch20_SFM = 0.002;

//UART VARIABLES////////////////////////////////////////
uart_handle_t uart_handle;

uint8_t UDMA1_SOM_RX_BUFF[100];
uint8_t UDMA1_SOM_TX_BUFF[11];

uint8_t UDMA4_POWER_RX_BUFF[100];
uint8_t UDMA4_POWER_TX_BUFF[11];

uint8_t UDMA6_Master_Slave_RX_BUFF[100];
uint8_t UDMA6_Master_Slave_TX_BUFF[11];

uint8_t Power_MCU_Send[11];
uint8_t SOM_Board_Send[11];
uint8_t Master_Slave_Send[11];

//PARAMETER DATA ARRAY//////////////////////////////////
uint32_t Parameter_Data_Array[150];

///////////
int Uart1_Tx_Flag = 1;
int Packet_Sent = 1;
int dummy = 0;
float AIR = 0;
float O2 = 0;
//float Air_Valve_Current = 0.00000;
//float O2_Valve_Current = 0.0000;
uint16_t Air_Valve = 0;
uint16_t O2_Valve = 0;

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

void delay (uint32_t time)
{
while(time--);
}

void Valves_GPIO_Init(void)
{
	//ENABLE GPIO CLOCKS
	RCC->AHB1ENR |= (1<<1);
	
	//CONFIGURE GPIO 
	GPIOB->MODER |= (2<<2);		///B1--SAFETY VALVE
	GPIOB->MODER |= (2<<10);	///B5--NEBULISER NC VALVE
	GPIOB->MODER |= (2<<18);	///B9--EXPIRATORY ACTUATOR
	
	GPIOB->OTYPER = 0;
	
	GPIOB->OSPEEDR |= (2<<2) | (2<<10) | (2<<18);
	
	GPIOB->PUPDR = 0;
	
	GPIOB->AFR[0] &= ~(15<<4);
	GPIOB->AFR[0] &= ~(15<<20);
	GPIOB->AFR[1] &= ~(15<<4);
}

void I2C3_O2_Flow_Config(void)
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

void I2C2_Air_Flow_Config(void)
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

void I2C1_Resp_Pressure_Config(void)
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
		TIMER_DELAY_us(10);
		//delay(100);
		I2C1->CR1 &= ~(1<<15);
		I2C1_Resp_Pressure_Config();
	}
	
	if(n==2)
	{
		I2C2->CR1 |= (1<<15);
		TIMER_DELAY_us(10);
		//delay(100);
		I2C2->CR1 &= ~(1<<15);
		I2C2_Air_Flow_Config();
	}
	
	if(n==3)
	{
		I2C3->CR1 |= (1<<15);
		TIMER_DELAY_us(10);
		//delay(100);
		I2C3->CR1 &= ~(1<<15);
		I2C3_O2_Flow_Config();
	}
		/*int Reset_Count = 0;
	
	//Reset peipheral
	I2C1->CR1 |= (1<<15);
	//I2C1->CR1 &= ~(1<<15);
	
	//Disable i2c CLOCK and GPIOB clock
	RCC->APB1ENR &= ~(1<<21);
	RCC->AHB1ENR &= ~(1<<1);	
	
	//Reset I2c peipheral
	I2C1->CR1 |= (1<<15);
	//Enable GPIO Clock
	RCC->AHB1ENR |= (1<<1);	
	
	//Convert AF to GPIO
	GPIOB->AFR[1] &= ~(4<<0);										//AF4 IN B8
	GPIOB->AFR[1] &= ~(4<<4);										//AF4 IN B9
	
	//configure GPIO as output
	GPIOB->MODER &= ~(2<<16);  									//Reset 
	GPIOB->MODER &= ~(2<<18);
	GPIOB->MODER |= (1<<16) | (1<<18);  				//Set
	
	//Enable push pull
	GPIOB->OTYPER  &= ~(1<<8);
	GPIOB->OTYPER  &= ~(1<<9);
	
	//Configure GPIO SDA HIGH
	GPIOB->BSRR |= (1<<9);
	for(int i=0; i<20; i++)
	{
		if(i%2==0)
		{	
			GPIOB->BSRR |= (1<<8);
		}
		else
		{
			GPIOB->BSRR |= ((1<<8) << 16);
		}
		if((GPIOB->ODR & (1<<9)) && (GPIOB->ODR & (1<<8)))
		{
			Reset_Count++;
		}
		delay(190);
	}
	if(Reset_Count >= 9)
	{
		GPIOB->BSRR |= (1<<8) ;
	}
	else
	{
		//Failed
	}*/
	//delay(10000);
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
	uint8_t I2C_Dummy_Read_Temp = 0;
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
			I2C_Dummy_Read_Temp = I2C1->SR1 | I2C1->SR2;
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
			I2C_Dummy_Read_Temp = I2C2->SR1 | I2C2->SR2;
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
			I2C_Dummy_Read_Temp = I2C3->SR1 | I2C3->SR2;
	}
			
	return status;
}

void UART_GPIO_Init(void)
{
	gpio_pin_conf_t uart_pin_conf;
	
  /*enable the clock for the GPIO port A */
	 _HAL_RCC_GPIOA_CLK_ENABLE();  
	 _HAL_RCC_GPIOC_CLK_ENABLE();
	 //_HAL_RCC_GPIOB_CLK_ENABLE();
	
	//##################----UART1----################
  /*configure the GPIO_PORT_A_PIN_9 as TX */
	uart_pin_conf.pin = USART1_TX_PIN;
	uart_pin_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	uart_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	uart_pin_conf.speed =  GPIO_PIN_SPEED_HIGH;
	uart_pin_conf.pull  =  GPIO_PIN_NO_PULL_PUSH;
	hal_gpio_set_alt_function(GPIOA,USART1_TX_PIN,USART1_TX_AF);
	hal_gpio_init(GPIOA ,&uart_pin_conf);
	
	/*configure the GPIO_PORT_B_PIN_6 as TX */
	//uart_pin_conf.pin = USART1_TX_PIN;
	//uart_pin_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	//uart_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	//uart_pin_conf.speed =  GPIO_PIN_SPEED_HIGH;
	//uart_pin_conf.pull  =  GPIO_PIN_NO_PULL_PUSH;
	//hal_gpio_set_alt_function(GPIOB,USART1_TX_PIN,USART1_TX_AF);
	//hal_gpio_init(GPIOB ,&uart_pin_conf);

	/*configure the GPIO_PORT_A_PIN_10 as RX */
	uart_pin_conf.pin = USART1_RX_PIN;
	hal_gpio_set_alt_function(GPIOA,USART1_RX_PIN,USART1_RX_AF);
	hal_gpio_init(GPIOA ,&uart_pin_conf);
	
	/*configure the GPIO_PORT_B_PIN_7 as RX */
	//uart_pin_conf.pin = USART1_RX_PIN;
	//hal_gpio_set_alt_function(GPIOB,USART1_RX_PIN,USART1_RX_AF);
	//hal_gpio_init(GPIOB ,&uart_pin_conf);
  
	//##################----UART4----################
	/*configure the GPIO_PORT_C_PIN_10 as TX */
	uart_pin_conf.pin = USART4_TX_PIN;
	hal_gpio_set_alt_function(GPIOC,USART4_TX_PIN,USART4_TX_AF);
	hal_gpio_init(GPIOC ,&uart_pin_conf);
	
	/*configure the GPIO_PORT_C_PIN_11 as RX */
	uart_pin_conf.pin = USART4_RX_PIN;
	hal_gpio_set_alt_function(GPIOC,USART4_RX_PIN,USART4_RX_AF);
	hal_gpio_init(GPIOC ,&uart_pin_conf);
	
	//##################----UART6----################
	/*configure the GPIO_PORT_C_PIN_6 as TX */
	uart_pin_conf.pin = USART6_TX_PIN;
	hal_gpio_set_alt_function(GPIOC,USART6_TX_PIN,USART6_TX_AF);
	hal_gpio_init(GPIOC ,&uart_pin_conf);
	
	/*configure the GPIO_PORT_C_PIN_7 as RX */
	uart_pin_conf.pin = USART6_RX_PIN;
	hal_gpio_set_alt_function(GPIOC,USART6_RX_PIN,USART6_RX_AF);
	hal_gpio_init(GPIOC ,&uart_pin_conf);	
}
void SFM3019_I2C2_Init(void)																				//Inlet air flow sensor
{												
	uint8_t Command1 = 0x36;															
	uint8_t Command0 = 0x08;
	uint8_t Write_Address = 0x5C;
	uint8_t Reset_Address = 0x00;
	uint8_t Reset_Command = 0x06;
	
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

	TIMER_DELAY_ms(10);																									//!!!!!!!!!DANGER!!!!!!!!!!! DO NOT CHANGE //default value 10ms
	
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

	TIMER_DELAY_ms(15);																										//!!!!!!!!!DANGER!!!!!!!!!!! DO NOT CHANGE //default value 15ms
	
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

//	//TIMER_DELAY_ms(950);

//	Flow = (I2C2_Data_array[0]<<8 | I2C2_Data_array[1]);
//	Flow = ((Flow - Offset_SFM2019)/Scale_Factor_SFM2019_Flow) - 96;
////Flow = (Flow/(1+Ch20_SFM*(Dv_SFM*exp(17.62*Temperature/(243.12+Temperature)))/(273.15+Temperature)))*(101325/(Pabs-Ph20_SFM*exp(17.62*Temperature/(243.12+Temperature))))*(Temperature+273.15)/293.15 ;


//	Temperature = (I2C2_Data_array[3]<<8 | I2C2_Data_array[4]);
//	Temperature = Temperature/Scale_Factor_SFM2019_Temp ;
//	
//	Air_Flow_Data[0] = Flow;
//	Air_Flow_Data[1] = Temperature;
		
}

void SFM3019_I2C3_Init(void)																				//Inlet O2 flow sensor	
{												
	uint8_t Command1 = 0x36;																						//SFM3019	addresses						
	uint8_t Command0 = 0x08;
	uint8_t Write_Address = 0x5C;
	uint8_t Reset_Address = 0x00;
	uint8_t Reset_Command = 0x06;
	
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

	TIMER_DELAY_ms(10);																									//!!!!!!!!!DANGER!!!!!!!!!!! DO NOT CHANGE //default value 10ms
	
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

	TIMER_DELAY_ms(15);																									//!!!!!!!!!DANGER!!!!!!!!!!! DO NOT CHANGE //default value 15ms
}

void Resp_Pressure_I2C1_sensor_data(void)													  //Respiratory pressure sensor
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
	const float Conversion_constant = 75.0;
	
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
	
	Respiratory_Pressure[0] = Pres;
	Respiratory_Pressure[1] = Temp;
}

void UART_1_SOM_Init(void)
{
  _HAL_RCC_USART1_CLK_ENABLE();
	//_HAL_RCC_DMA1_CLK_ENABLE() ;
	_HAL_RCC_DMA2_CLK_ENABLE();
		
	
	uart_handle.Instance          = USART_1;

	uart_handle.Init.BaudRate     = USART_BAUD_115200 ;
	uart_handle.Init.WordLength   = USART_WL_1S8B;
	uart_handle.Init.StopBits     = USART_STOP_BITS_1;
	uart_handle.Init.Parity       = UART_PARITY_NONE;
	uart_handle.Init.Mode         = UART_MODE_TX_RX;
	uart_handle.Init.OverSampling = USART_OVER16_ENABLE;
	
	//##########################---UART_DMA_RX---###################
	
   	
		DMA2_Stream5->M0AR =(uint32_t)UDMA1_SOM_RX_BUFF; //memary address
		DMA2_Stream5->PAR =(uint32_t)&USART1->DR;//peri address
		DMA2_Stream5->NDTR = 100;
		
		DMA2_Stream5->FCR |= (1<<2);		//direct mode off
		DMA2_Stream5->FCR =0x3<<0;			//fifo mode on
		
	  DMA2_Stream5->CR &= ~(0x4<<25);
	  DMA2_Stream5->CR |=(0x4<<25);  	//CHANNEL SELECTION 4 BY 100
	 
	  DMA2_Stream5->CR &= ~(1<<10);
	  DMA2_Stream5->CR |=(1<<10);			//MEM INCREMENT ENABLE
	  
		DMA2_Stream5->CR |= (0x1<<16);	//PRIORITY MODERATE
		
		DMA2_Stream5->CR &= ~(1<<4);		//TC COMPLETE INTRRUPT ENABLE
		DMA2_Stream5->CR |= (1<<4);
	
		DMA2_Stream5->CR &= ~(1<<0);		//STREAM ENABLE IN THE END
		DMA2_Stream5->CR |=(1<<0);
	 
	//#################---DMA-RX-END--###############################
	//###################---DMA-TX-ST--################################  
	  DMA2_Stream7->M0AR =(uint32_t) UDMA1_SOM_TX_BUFF; //memary address
		DMA2_Stream7->PAR =(uint32_t)&USART1->DR;//peri address
		DMA2_Stream7->NDTR = 11;
	
	  DMA2_Stream7->FCR |= (1<<2);		//direct mode off
		DMA2_Stream7->FCR =0x3<<0;			//fifo mode on
		
		DMA2_Stream7->CR &= ~(0x4<<25);
	  DMA2_Stream7->CR |=(0x4<<25);  	//CHANNEL SELECTION 4 BY 100
	 
	  
		DMA2_Stream7->CR &= ~(1<<10);
	  DMA2_Stream7->CR |=(1<<10);			//MEM INCREMENT ENABLE
		
		DMA2_Stream7->CR |= (0x01<<6);	//DATA TRANFAR MEM TO PERI 
		
		DMA2_Stream7->CR &= ~(1<<4);		//TC COMPLETE INTRRUPT ENABLE
		DMA2_Stream7->CR |= (1<<4);
	
		DMA2_Stream7->CR &= ~(1<<0);		//STREAM ENABLE IN THE END
		DMA2_Stream7->CR |=(1<<0);
	
	//#################---DMA-TX-END--###############################
	
	 hal_uart_init(&uart_handle);
	// USART_2->SR |=(1<<5);
	 USART_1->CR1 &=~(1<<4);
	
   NVIC_SetPriority(USART1_IRQn_EN, 2);  

	 NVIC_EnableIRQ(USART1_IRQn_EN);
	 
	 NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	 NVIC_EnableIRQ(DMA2_Stream7_IRQn);
 }

void UART_4_Power_Init(void)
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
	
   	
		DMA1_Stream2->M0AR =(uint32_t)UDMA4_POWER_RX_BUFF; //memary address
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
	  DMA1_Stream4->M0AR =(uint32_t) UDMA4_POWER_TX_BUFF; //memary address
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

void UART_6_Master_Slave_Init(void)
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
		
			
			DMA2_Stream1->M0AR =(uint32_t)UDMA6_Master_Slave_RX_BUFF; //memary address
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
			DMA2_Stream6->M0AR =(uint32_t) UDMA6_Master_Slave_TX_BUFF; //memary address
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
	USART_1->CR3 |= (1<<6);//ENABLE DMA BIT FOR TX
	USART_1->CR3 |= (1<<7);//ENABLE DMA BIT FOR RX
	
	USART_4->CR3 |= (1<<6);//ENABLE DMA BIT FOR TX
	USART_4->CR3 |= (1<<7);//ENABLE DMA BIT FOR RX
	
	USART_6->CR3 |= (1<<6);//ENABLE DMA BIT FOR TX
	USART_6->CR3 |= (1<<7);//ENABLE DMA BIT FOR RX
	
	///SOM BOARD PACKET INITIALIZE///////////////////////////////
	SOM_Board_Send[0] = 0x23;		
	SOM_Board_Send[1] = 0xF5;		
	SOM_Board_Send[2] =	0xF5;		
	SOM_Board_Send[3] =	0xF5;		
	SOM_Board_Send[4] =	0xF5;		
	SOM_Board_Send[5] = 0xF5;			
	SOM_Board_Send[6] =	0xF5;		
	SOM_Board_Send[7] =	0xF5;		
	SOM_Board_Send[8] =	0xF5;		
	SOM_Board_Send[9] =	0xF5;
	SOM_Board_Send[10] = 0x40;
	
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
	memcpy(UDMA1_SOM_TX_BUFF,SOM_Board_Send,11);	
	memcpy(UDMA4_POWER_TX_BUFF,Power_MCU_Send,11);
	memcpy(UDMA6_Master_Slave_TX_BUFF,Master_Slave_Send,11);	
}

void Make_Packet(uint32_t mode, uint32_t para_id, uint32_t data)
{
	uint8_t Sign_byte;
	uint8_t Byte_Zero;
	uint8_t Byte_One;
	uint8_t Byte_Two;
	int Mantisa;
	int Abs_Mantisa;
	
	if(Uart1_Tx_Flag==1)
	{
		if(para_id!=POWER_SEND)
		{
			SOM_Board_Send[1] = (uint8_t)(mode>>16);		
			SOM_Board_Send[2] =	(uint8_t)(mode>>8);		
			SOM_Board_Send[3] =	(uint8_t)(mode);		
			SOM_Board_Send[4] =	(uint8_t)(para_id>>8);		
			SOM_Board_Send[5] = (uint8_t)(para_id);			
			SOM_Board_Send[6] =	(uint8_t)(data>>24);		
			SOM_Board_Send[7] =	(uint8_t)(data>>16);		
			SOM_Board_Send[8] =	(uint8_t)(data>>8);		
			SOM_Board_Send[9] =	(uint8_t)(data);
			
		}
		else
		{
			Power_MCU_Send[1] = mode;
			Power_MCU_Send[2] = para_id;
			Power_MCU_Send[3] = Sign_byte;
			Power_MCU_Send[4] = Byte_Two;
			Power_MCU_Send[5] = Byte_One;
			Power_MCU_Send[6] = (uint8_t)(data);
		}
	}
	Uart1_Tx_Flag = 0;
}

void Packet_Parser(uint8_t data[])
{
	//PARSING VARIABLES/////////////////////////////////////
	uint32_t Mode;
	uint32_t Para_ID;
	uint32_t Payload ;
	//uint8_t Sign_Byte;
	//int Exponent = 1000;
	
	for(int i=0;i<100;i++)	
		{
			/*if(data[i]==0x23 && data[i+1]==0x32 && data[i+10]==0x40)
			{
				Para_ID = (data[i+2]);
				Sign_Byte = (data[i+3]);
				Payload = (data[i+4]<<16)+(data[i+5]<<8)+(data[i+6]);
				if(Sign_Byte!=0x01)
					Payload = (Payload / Exponent);
				else
					Payload = (-1) * (Payload / Exponent);
				if(Para_ID==EXP_FLOW)
					EXP_SDP31_Data[0] = Payload;
				if(Para_ID==PROX_FLOW)
					PROX_SDP31_Data[0] = Payload;
				if(Para_ID==PROX_PRESSURE)
					PROX_Pressure_Data[0] = Payload;
			}*/
			
			if(data[i]==0x23 && data[i+10]==0x40)
			{
				Mode = (data[i+1]<<16)+(data[i+2]<<8)+(data[i+3]);
				Para_ID =(data[i+4]<<8)+(data[i+5]);
				Payload = (data[i+6]<<24)+(data[i+7]<<16)+(data[i+8]<<8)+(data[i+9]);
				if(Mode==MCU_GUI)
				{
				Parameter_Data_Array[(uint8_t)(data[i+5])] = Payload;
				}
			}
		}
}
 void ADC_GPIO_Conf(void)
{
	//GPIO CLOCK
	RCC->AHB1ENR |= (1<<0);		//PORT A
	RCC->AHB1ENR |= (1<<2);		//PORT C
	
	//GPIO MODER
	//GPIOA->MODER |= (3<<0);		//A0
	GPIOA->MODER |= (3<<14);		//A7--O2 HPS
	GPIOC->MODER |= (3<<0);			//C0--MPXV5010
	GPIOC->MODER |= (3<<2);		  //C1--O2 sensor
	GPIOC->MODER |= (3<<4);		  //C2--Air HPS
	
}

void ADC_Init(void)
{
	//ENABLE ADC CLOCK
	RCC->APB2ENR |= (1<<8);
	
	//DISABLE ADC
	ADC1->CR2 &= ~(1<<0);
	ADC1->CR2 &= ~(1<<30);
	
	//PRESCALER IN CCR
	ADC->CCR |= (1<<16);
	
	//SCAN MODE ENABLE AND RESOLUTION 12 BIT
	ADC1->CR1 |= (1<<8);
	ADC1->CR1 &= ~(1<<24);
	
	//CONTINUOUS CONVERSION, EOC, DATA ALIGNMENT
	ADC1->CR2 |= (1<<1);
	ADC1->CR2 |= (1<<10);
	ADC1->CR2 &= ~(1<<11);
	
	//SAMPLING TIME and CHANNEL SEQUENCE (CHANNEL 0)
	ADC1->SQR1 |= (3<<20);			//4 CONVERSIONS
	ADC1->SQR3 |= (11<<0);			//C1--O2 sensor
	ADC1->SQR3 |= (10<<5);			//C0--MPXV5010
	ADC1->SQR3 |= (12<<10);			//C2--Air HPS
	ADC1->SQR3 |= (7<<15);			//A7--O2 HPS
	
	//SAMPLING TIME (84 CYCLES)
	ADC1->SMPR2 |= (4<<21);		//CHANNEL 7
	ADC1->SMPR1 |= (4<<0);		//CHANNEL 10
	ADC1->SMPR1 |= (4<<6);		//CHANNEL 12
	ADC1->SMPR1 |= (4<<3);		//CHANNEL 11
}

void ADC_DMA_Init(void)
{
	//ENABLE DMA2 CLOCK
	RCC->AHB1ENR |= (1<<22);
	
	//DISABLE DMA STREAM
	DMA2_Stream0->CR &= ~(1<<0);
	
	//DATA DIRECTION
	DMA2_Stream0->CR &= ~(3<<6);
	
	//CIRCULAR MODE
	DMA2_Stream0->CR |= (1<<8);
	
	//DISABLE FIFO
	DMA2_Stream0->FCR &= ~(1<<2);

	//MEMORY INCREMENT
	DMA2_Stream0->CR |= (1<<10);
	
	//16BIT FOR DATA SIZE
	DMA2_Stream0->CR |= (1<<11) | (1<<13);
	
	//DMA CHANNEL 0
	DMA2_Stream0->CR &= ~(7<<25);
	
	//MEMORY ADDRESS
	DMA2_Stream0->M0AR = (uint32_t)ADC_DMA_BUFFER;
	
	//PERIPHERAL ADDRESS
	DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
	
	//NDTR
	DMA2_Stream0->NDTR = 0x4;
}

void ADC_Start_DMA(void)
{
	//ENABLE DMA
	ADC1->CR2 |= (1<<8);
	
	//ENABLE CONTINUOUS REQUEST
	ADC1->CR2 |= (1<<9);
	
  //ADC ENABLE
	ADC1->CR2 |= (1<<0);
	
	//TIMER_DELAY_us(50);
	
	//ENABLE DMA STREAM
	DMA2_Stream0->CR |= (1<<0);

	//CLEAR STATUS REGISTER
	ADC1->SR = 0;
	
	//START CONVERSION
	ADC1->CR2 |= (1<<30);
}

void ADC_Sensor_Data(void)
{
	///HIGH PRESSURE SENSOR TRANSFER FUNCTION VARIABLES///////////////////////
	float Volt_factor_HPS = 3.0/4096.0;
	float Offset_Factor_HPS = 0.3;
	float Fullscale_Volt_HPS = 2.4;
	float Fullscale_Pressure_HPS = 150;
	float Psi_to_Bar = 0.0689476;
	
	///O2 SENSOR TRANSFER FUNCTION VARIABLES/////////////////////////////////
	float Slope_O2_Sensor = 0.9405;
	float Intercept_O2_Sensor =  46.71;
		
	///ADC DMA FLAGS CLEAR///////////////
	if(DMA2->LISR & (1<<5))
	{
		DMA2->LIFCR |= (1<<5) | (1<<4);
	}
	
	///HIGH PRESSURE SENSOR TRANSFER FUNCTIONS///////////////////////////////
	HPS_O2_Data = (((Volt_factor_HPS * ADC_DMA_BUFFER[3])- Offset_Factor_HPS) * Fullscale_Pressure_HPS / Fullscale_Volt_HPS) * (Psi_to_Bar);	
	HPS_Air_Data = (((Volt_factor_HPS * ADC_DMA_BUFFER[2]) - Offset_Factor_HPS) * Fullscale_Pressure_HPS / Fullscale_Volt_HPS) * (Psi_to_Bar);
	
	///O2 SENSOR TRANSFER FUNCTION///////////////////////////////////////////
	O2_Sensor_Data = (Slope_O2_Sensor * ADC_DMA_BUFFER[0]) - (Intercept_O2_Sensor);
}

void DAC_GPIO_Config(void)
{
	//ENABLE GPIO CLOCK
	RCC->AHB1ENR |= (1<<0);  //PORT A
	
	//MODER AS ANALOG
	GPIOA->MODER |= (3<<8) | (3<<10);  //A4 AND A5
	
}

void DAC_Init(void)
{
	//ENABLE DAC CLOCK 
	RCC->APB1ENR |= (1<<29);
	
	//CLEAR CR REGISTER
	DAC->CR = 0;
	
	//ENABLE CHANNEL 1 AND CHANNEL 2
	DAC->CR |= (1<<0) | (1<<16);
}

void Proportional_Valve_Drive(double Air_Flow, double O2_Flow)
{
	///DAC CALCULATION VARIABLES//////////////////////////////////////////
	const double Slope = 12952.0;
	const double Intercept = 98.272;
	const double F_I_Lin_Coeff = 0.0033;
	const double F_I_Quad_Coeff = 0.00005;
	const double F_I_Quib_Coeff = 0.0000004;
	const double F_I_Quin_Coeff = 0.0000000008;
	const double F_I_Const1 = 0.0528;
	const double I_D_Const1 = 0.1571;
	const double I_D_Const2 = 0.02;
	const double I_D_Const3 = 0.0071;
	double Air_Valve_Current;
	double O2_Valve_Current;
//	uint16_t Air_Valve;
//	uint16_t O2_Valve;
	
	///VALVE CURRENT CALCULATION//////////////////////////////////////////
	Air_Valve_Current = (F_I_Const1 + F_I_Lin_Coeff * Air_Flow - F_I_Quad_Coeff * Air_Flow*Air_Flow + F_I_Quib_Coeff * Air_Flow*Air_Flow*Air_Flow - F_I_Quin_Coeff * Air_Flow*Air_Flow*Air_Flow*Air_Flow);
	O2_Valve_Current = (F_I_Const1 + F_I_Lin_Coeff * O2_Flow - F_I_Quad_Coeff * O2_Flow*O2_Flow + F_I_Quib_Coeff * O2_Flow*O2_Flow*O2_Flow - F_I_Quin_Coeff * O2_Flow*O2_Flow*O2_Flow*O2_Flow);
	
	///DAC VALUE CALCULATION/////////////////////////////////////////////	
	Air_Valve = (Slope * (Air_Valve_Current + I_D_Const1 * (Air_Valve_Current + I_D_Const2) + I_D_Const3) - Intercept);
	O2_Valve = (Slope * (O2_Valve_Current + I_D_Const1 * (O2_Valve_Current + I_D_Const2) + I_D_Const3) - Intercept);
	
	///ASSIGN DAC VALUE TO 12 BIT RIGHT ALIGNED REGISTER/////////////////
	DAC->DHR12R1 = Air_Valve;
	DAC->DHR12R2 = O2_Valve;
}
void DMA2_Stream5_SOM_Rx_callback(void)
{
	  USART_1->CR3 &= ~(1<<6);//DISBLE DMA BIT FOR RX
    DMA2_Stream5->NDTR = 100;//LENGTH OF RX
		DMA2_Stream5->CR &= ~(1<<0);//STREAM ENABLE IN THE END
		DMA2_Stream5->CR |=(1<<0);
   
	  USART_1->CR3 |= (1<<6);//ENABLE DMA BIT FOR RX
}
void DMA2_Stream7_SOM_Tx_callback(void)
{
	  USART_1->CR3 &= ~(1<<7);//DISBLE DMA BIT FOR TX
   
		DMA2_Stream7->NDTR = 11; //LENGTH OF TX
		DMA2_Stream7->CR &= ~(1<<0);//STREAM ENABLE IN THE END
		DMA2_Stream7->CR |=(1<<0);
		
		Uart1_Tx_Flag = 1;
		Packet_Sent++;
	if(Packet_Sent>9)
			Packet_Sent=0;
		USART_1->CR3 |= (1<<7);//ENABLE DMA BIT FOR TX
		memcpy(UDMA1_SOM_TX_BUFF,SOM_Board_Send,11);
		
}

void DMA1_Stream2_Power_Rx_callback(void)
{
	  USART_4->CR3 &= ~(1<<6);//DISBLE DMA BIT FOR RX
    DMA1_Stream2->NDTR = 100;//LENGTH OF RX
		DMA1_Stream2->CR &= ~(1<<0);//STREAM ENABLE IN THE END
		DMA1_Stream2->CR |=(1<<0);//
   
	  USART_4->CR3 |= (1<<6);//ENABLE DMA BIT FOR RX
}
void DMA1_Stream4_Power_Tx_callback(void)
{
	  USART_4->CR3 &= ~(1<<7);//DISBLE DMA BIT FOR TX
   
		DMA1_Stream4->NDTR = 11; //LENGTH OF TX
		DMA1_Stream4->CR &= ~(1<<0);//STREAM ENABLE IN THE END
		DMA1_Stream4->CR |=(1<<0);
   
    USART_4->CR3 |= (1<<7);//ENABLE DMA BIT FOR TX
    memcpy(UDMA4_POWER_TX_BUFF,Power_MCU_Send,11);
}

void DMA2_Stream1_Master_Slave_Rx_callback(void)
{
	  USART_6->CR3 &= ~(1<<6);//DISBLE DMA BIT FOR RX
    
		DMA2_Stream1->NDTR = 100;//LENGTH OF RX
		DMA2_Stream1->CR &= ~(1<<0);//STREAM ENABLE IN THE END
		DMA2_Stream1->CR |=(1<<0);//
   
	  USART_6->CR3 |= (1<<6);//ENABLE DMA BIT FOR RX
}
void DMA2_Stream6_Master_Slave_Tx_callback(void)
{
	  USART_6->CR3 &= ~(1<<7);//DISBLE DMA BIT FOR TX
   
		DMA2_Stream6->NDTR = 11; //LENGTH OF TX
		DMA2_Stream6->CR &= ~(1<<0);//STREAM ENABLE IN THE END
		DMA2_Stream6->CR |=(1<<0);
   
    USART_6->CR3 |= (1<<7);//ENABLE DMA BIT FOR TX
    memcpy(UDMA6_Master_Slave_TX_BUFF,Master_Slave_Send,11);
}

 
void DMA2_Stream5_SOM_Rx_IRQHandler (void)
{
 if(DMA2->HISR&(1<<11))
 {
 
	DMA2->HIFCR |= (1<<11);
	DMA2->HIFCR |= (1<<10);	 
	DMA2_Stream5_SOM_Rx_callback();
 }
}

void DMA2_Stream7_SOM_Tx_IRQHandler (void)
{
 if(DMA2->HISR&(1<<27))
 {
	DMA2->HIFCR |= (1<<27);
	DMA2->HIFCR |= (1<<26);	 
	DMA2_Stream7_SOM_Tx_callback();
 }
}

void DMA1_Stream2_Power_Rx_IRQHandler (void)
{
 if(DMA1->LISR&(1<<21))
 {
	DMA1->LIFCR |= (1<<21);
	DMA1->LIFCR |= (1<<20);	 
	DMA1_Stream2_Power_Rx_callback();
 }
}

void DMA1_Stream4_Power_Tx_IRQHandler (void)
{
 if(DMA1->HISR&(1<<5))
 {
 
	DMA1->HIFCR |= (1<<5);
	DMA1->HIFCR |= (1<<4);	 
	DMA1_Stream4_Power_Tx_callback();
 }
}

void DMA2_Stream1_Master_Slave_Rx_IRQHandler (void)
{
 if(DMA2->LISR&(1<<11))
 {
	DMA2->LIFCR |= (1<<11);
	DMA2->LIFCR |= (1<<10);	 
	DMA2_Stream1_Master_Slave_Rx_callback();
 }
}

void DMA2_Stream6_Master_Slave_Tx_IRQHandler (void)
{
 if(DMA2->HISR&(1<<21))
 {
 
	DMA2->HIFCR |= (1<<21);
	DMA2->HIFCR |= (1<<20);	 
	DMA2_Stream6_Master_Slave_Tx_callback();
 }
}
void Inlet_Air_Flow_I2C2_Read(void)
{
	uint8_t Read_Address = 0x5D;
	const uint16_t Offset_SFM2019 = 24576;
	const uint16_t Scale_Factor_SFM2019_Flow = 170;
	const uint16_t Scale_Factor_SFM2019_Temp = 200;
	uint8_t I2C2_Data_array[9];
	float Flow = 0;
	float Temperature = 0;
	
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

	//TIMER_DELAY_ms(950);

	Flow = (I2C2_Data_array[0]<<8 | I2C2_Data_array[1]);
	Flow = ((Flow - Offset_SFM2019)/Scale_Factor_SFM2019_Flow) - 96;
//Flow = (Flow/(1+Ch20_SFM*(Dv_SFM*exp(17.62*Temperature/(243.12+Temperature)))/(273.15+Temperature)))*(101325/(Pabs-Ph20_SFM*exp(17.62*Temperature/(243.12+Temperature))))*(Temperature+273.15)/293.15 ;

	Temperature = (I2C2_Data_array[3]<<8 | I2C2_Data_array[4]);
	Temperature = Temperature/Scale_Factor_SFM2019_Temp ;
	
	Air_Flow_Data[0] = Flow;
	Air_Flow_Data[1] = Temperature;
}

void Inlet_O2_Flow_I2C3_Read(void)
{
	uint8_t Read_Address = 0x5D;
	const uint16_t Offset_SFM2019 = 24576;
	const uint16_t Scale_Factor_SFM2019_Flow = 170;
	const uint16_t Scale_Factor_SFM2019_Temp = 200;
	uint8_t I2C3_Data_array[9];
	float Flow = 0;
	float Temperature = 0;
	
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
		
	//TIMER_DELAY_ms(950);
	
	Flow = (I2C3_Data_array[0]<<8 | I2C3_Data_array[1]);
	Flow = ((Flow - Offset_SFM2019)/Scale_Factor_SFM2019_Flow) - 96;
	//Flow = (Flow/(1+Ch20_SFM*(Dv_SFM*exp(17.62*Temperature/(243.12+Temperature)))/(273.15+Temperature)))*(101325/(Pabs-Ph20_SFM*exp(17.62*Temperature/(243.12+Temperature))))*(Temperature+273.15)/293.15 ;

	Temperature = (I2C3_Data_array[3]<<8 | I2C3_Data_array[4]);
	Temperature = Temperature/Scale_Factor_SFM2019_Temp ;
	
	O2_Flow_Data[0] = Flow;
	O2_Flow_Data[1] = Temperature;
}

int main(void)
{	
	///CLOCK, TIMERS INITIALIZE////////////////////////////
	Clock_Config ();
	Timer2_Init();
	
	///GPIO INITIALIZE/////////////////////////////////////
	UART_GPIO_Init();
	ADC_GPIO_Conf();
	
	///I2C INITIALIZE//////////////////////////////////////
	I2C1_Resp_Pressure_Config();
	I2C2_Air_Flow_Config();
	I2C3_O2_Flow_Config();
	
	///ADC INITIALIZE//////////////////////////////////////
	ADC_Init();
	ADC_DMA_Init();
	ADC_Start_DMA();

	///DAC CHANNEL INITIALZE///////////////////////////////
	DAC_Init();
	
	///I2C INITIALIZE//////////////////////////////////////
	SFM3019_I2C2_Init();
	SFM3019_I2C3_Init();
	
	///UART INITIALIZE/////////////////////////////////////
	UART_1_SOM_Init(); 
	UART_4_Power_Init();
	UART_6_Master_Slave_Init();
	UART_Packet_Init();
	
	///////////////
	for(int k = 33;k<=40;k++)
		{
			Parameter_Data_Array[k] = k;
		}
	Parameter_Data_Array[42] = 42;
		
	//SAFETY_ACTUATOR_ON();
	//EXPIRATORY_ACTUATOR_ON();
				
	//TIMER_DELAY_ms(100);
	while(1)
	{
		///SENSOR DATA FUNCTIONS/////////////////////////////
		ADC_Sensor_Data();
		Inlet_Air_Flow_I2C2_Read();
		Inlet_O2_Flow_I2C3_Read();
		Resp_Pressure_I2C1_sensor_data();
		
		///PROPORTIONAL VALVE DRIVE FUNCTION/////////////////
		Proportional_Valve_Drive(AIR, O2);			
		
		///CHECK FOR DATA FROM SLAVE/////////////////////////
		Packet_Parser(UDMA4_POWER_RX_BUFF);
		
		
		///SEND DATA TO POWER BOARD//////////////////////////
		Make_Packet(SLAVE_MASTER, POWER_SEND, dummy);
		
		///SEND DATA TO SOM BOARD////////////////////////////
		Make_Packet(MCU_GUI, Resp_Volume_MCU_GUI, Parameter_Data_Array[33]);
		TIMER_DELAY_ms(100);
		Make_Packet(MCU_GUI, Resp_Pressure_MCU_GUI, Parameter_Data_Array[34]);
		TIMER_DELAY_ms(100);
		Make_Packet(MCU_GUI, Resp_Flow_MCU_GUI, Parameter_Data_Array[35]);
		TIMER_DELAY_ms(100);
		Make_Packet(MCU_GUI, Resp_Fio2_MCU_GUI, Parameter_Data_Array[36]);
		TIMER_DELAY_ms(100);
		
		///CHECK FOR DATA FROM SOM/////////////////////////
		Packet_Parser(UDMA1_SOM_RX_BUFF);
	}
}

	


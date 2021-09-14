#include <stdint.h>
#include "stm32f4xx.h"
#include "hal_spi_driver.h"
#include "hal_gpio_driver.h"
#include "spi_main.h"
#include "led.h"
#include "uart_drv_al.h"
#include "uart_debug.h"

#include <math.h>
/* SPI handle for our SPI device */

#define PI 3.14159265358979323846
const float alpha = 0.5;
 volatile double msticks=0;
double Xg = 0;
double Yg = 0;
double Zg = 0;
double Pg = 0;
double Cg = 0;
 double  rp=0.004;
spi_handle_t SpiHandle;

int TestReady = 0;
float X,Y,Z,Roll,Pitch=0;
int x,y,z=0;
uint16_t sx,sy,sz;
uint8_t on[] = "C000";

uint8_t master_read_buffer[6];
uint8_t rx_buf;
 uint8_t xbee_rx[2];
 uart_handle_t uart_handle;
//#define GPIO_AF7_USART1        ((uint8_t)0x07)  /* USART1 Alternate Function mapping     */
#define GPIO_AF7_USART2        ((uint8_t)0x07)  /* USART2 Alternate Function mapping     */
#define GPIO_AF7_USART3        ((uint8_t)0x07)  /* USART3 Alternate Function mapping     */

#define GPIO_PIN_2_SEL                       2
#define GPIO_PIN_3_SEL                       3    
#define USARTx_TX_PIN                    GPIO_PIN_2_SEL
#define USARTx_TX_GPIO_PORT              GPIOA  
#define USARTx_TX_AF                     GPIO_AF7_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3_SEL
#define USARTx_RX_GPIO_PORT              GPIOA 
#define USARTx_RX_AF                     GPIO_AF7_USART2
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler




void SystemCoreClockConfigure(void) {

 RCC->CR |= (1<<16);                     /* Enable HSE */
  while (!(RCC->CR & (1<<17)));                  /* Wait for HSE Ready */

		
  RCC->CR&=~(1<<24);/*KEEP PLL OFF*/
	
	/*MALE HSE AS INPUT TO PLL*/
	RCC->PLLCFGR|=(1<<22);

	/* CONFIG PLLM VALUES*/
RCC->PLLCFGR&=~(0x3F);
RCC->PLLCFGR|=(0x08);	

	/*CONFIG PLLN VALUES*/
RCC->PLLCFGR&=~(0x1FF<<6);
RCC->PLLCFGR|=(0x150<<6);		
	
	/*CONFIG PLLP VALUES*/
RCC->PLLCFGR&=~(0x03<<16);
//RCC->PLLCFGR|=(0x01<<16);

/*NOW ENABLE PLL*/
RCC->CR|=(1<<24);
 
 
 /*WAIT TO PLL GET LOCKED*/
  while(!(RCC->CR&(1<<25)));
	
	/*Flash latancy*/
	FLASH->ACR|=(0x5);
	
/* MAKEPLL AS SYSTEM CLOCK*/
	RCC->CFGR&=~(0x03);
	RCC->CFGR|=0x02;
	  while(!(RCC->CFGR&(0x2<<2)));
	
   RCC->CFGR &= ~(0xF<<4);
	
	/*MAKE APB1=42MHZ*/
	 RCC->CFGR &= ~(0x7<<10);
	 RCC->CFGR |= (0x5<<10);
		
		
		/*MAKE APB2=84MHZ*/
	 RCC->CFGR &= ~(0x7<<13);
   RCC->CFGR |= (0x4<<13);


}  
/* configure gpio for spi functionality */


void spi_gpio_init(void)
{
	
	 _HAL_RCC_GPIOA_CLK_ENABLE();
		_HAL_RCC_GPIOB_CLK_ENABLE();
	 	_HAL_RCC_GPIOC_CLK_ENABLE(); 
    _HAL_RCC_GPIOD_CLK_ENABLE();
    _HAL_RCC_GPIOE_CLK_ENABLE();
     
	gpio_pin_conf_t spi_conf;
	 	_HAL_RCC_GPIOE_CLK_ENABLE();
	
	_HAL_RCC_GPIOA_CLK_ENABLE();
	
	/* configure GPIOB_PIN_13 for SPI CLK functionality */ 
	spi_conf.pin = SPI_CLK_PIN;
	spi_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	spi_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	spi_conf.pull = GPIO_PIN_NO_PULL_PUSH ;
	spi_conf.speed = GPIO_PIN_SPEED_MEDIUM;
	
	hal_gpio_set_alt_function(GPIOB,SPI_CLK_PIN,GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB, &spi_conf);

		
//	hal_gpio_set_alt_function(GPIOA,SPI1_CLK_PIN,GPIO_PIN_AF5_SPI1);
//	hal_gpio_init(GPIOA, &spi_conf);

	/* configure GPIOB_PIN_14 for SPI MISO functionality */ 
	spi_conf.pin = SPI_MISO_PIN;
	spi_conf.pull = GPIO_PIN_NO_PULL_PUSH ;
	hal_gpio_set_alt_function(GPIOB,SPI_MISO_PIN,GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB, &spi_conf);
	
	/* configure GPIOB_PIN_15 for SPI MOSI functionality */ 
	spi_conf.pin = SPI_MOSI_PIN;
	spi_conf.pull = GPIO_PIN_NO_PULL_PUSH ;
	hal_gpio_set_alt_function(GPIOB,SPI_MOSI_PIN,GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB, &spi_conf);
	
	/* configure GPIOE_PIN_10 for SPI MOSI functionality */ 
	spi_conf.pin = SPI_CS_PIN;
	spi_conf.mode=GPIO_PIN_OUTPUT_MODE;
  spi_conf.op_type=GPIO_PIN_OP_TYPE_PUSHPULL;
	spi_conf.pull = GPIO_PIN_NO_PULL_PUSH ;
	hal_gpio_init(GPIOE, &spi_conf);
	 
	 
}
void app_tx_cmp_callback(void *size)
{

	
}

/*This callback will be called by the driver when the application receives the command */
void app_rx_cmp_callback(void *size)
{
	//we got a command,  parse it 
	//parse_cmd(rx_buffer);
// hal_uart_tx(&uart_handle,rx_buffer, 1 ); 
	

}


void uart_gpio_init(void)
{
	gpio_pin_conf_t uart_pin_conf;
	
	gpio_pin_conf_t uart3_pin_conf;
	
/*enable the clock for the GPIO port A */
	 _HAL_RCC_GPIOA_CLK_ENABLE();  
	 _HAL_RCC_GPIOD_CLK_ENABLE() ;
	
/*configure the GPIO_PORT_A_PIN_2 as TX */
	uart_pin_conf.pin = USARTx_TX_PIN;
	uart_pin_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	uart_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	uart_pin_conf.speed =  GPIO_PIN_SPEED_HIGH;
	uart_pin_conf.pull  =  GPIO_PIN_NO_PULL_PUSH;
	hal_gpio_set_alt_function(GPIOA,USARTx_TX_PIN,USARTx_TX_AF);
	hal_gpio_init(GPIOA ,&uart_pin_conf);

	/*configure the GPIO_PORT_A_PIN_3 as RX */
	uart_pin_conf.pin = USARTx_RX_PIN;
	hal_gpio_set_alt_function(GPIOA,USARTx_RX_PIN,USARTx_TX_AF);
	hal_gpio_init(GPIOA ,&uart_pin_conf);
  
	
	
	
}
void UART_2_Xbee_init()
{
_HAL_RCC_USART2_CLK_ENABLE();
	 _HAL_RCC_GPIOA_CLK_ENABLE();  
	 _HAL_RCC_GPIOD_CLK_ENABLE() ;
		
	
	uart_handle.Instance          = USART_2;

	uart_handle.Init.BaudRate     =USART_BAUD_115200 ;
	uart_handle.Init.WordLength   = USART_WL_1S8B;
	uart_handle.Init.StopBits     = UART_STOPBITS_1;
	uart_handle.Init.Parity       = UART_PARITY_NONE;
	uart_handle.Init.Mode         = UART_MODE_TX_RX;
	uart_handle.Init.OverSampling = USART_OVER16_ENABLE;

/*fill out the application callbacks */
	uart_handle.tx_cmp_cb = app_tx_cmp_callback;
	uart_handle.rx_cmp_cb = app_rx_cmp_callback;
	
	 hal_uart_init(&uart_handle);
	
	//uart_handle.tx_state = HAL_UART_STATE_READY;
	// USART_2->SR |=(1<<5);
	 USART_2->CR1 &=~(1<<4);
	
   NVIC_SetPriority(USARTx_IRQn, 2);  

	 NVIC_EnableIRQ(USARTx_IRQn);
 }

/* some delay generation */
void delay_gen( )
{
	uint32_t cnt = 580;
	while(cnt--);
}
void delay_long( )
{
	uint32_t cnt = 100000000;
	while(cnt--);
}


//hang on here, if applicaton can not proceed due to error
	
void assert_error(void)
{
	
}

/* function used to compare two buffers */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }
	return 0;
}
	
int main(void)
{
	SystemCoreClockConfigure();
      SysTick_Config(167999);	
	uint32_t i=0;
	uint8_t addrcmd[2];
	uint8_t ack_buf[2];
	_HAL_RCC_GPIOE_CLK_ENABLE();
		_HAL_RCC_GPIOA_CLK_ENABLE();
	
	spi_gpio_init();
	uart_gpio_init();
	UART_2_Xbee_init();
	
	/* To use LED */
	led_init();
	
	/* Configure USER Button interrupt*/
	//_HAL_RCC_GPIOA_CLK_ENABLE();
//	_HAL_RCC_GPIOE_CLK_ENABLE();
//		_HAL_RCC_GPIOA_CLK_ENABLE();
	//hal_gpio_configure_interrupt(GPIO_BUTTON_PIN, INT_FALLING_EDGE);
//	hal_gpio_enable_interrupt(GPIO_BUTTON_PIN,EXTI0_IRQn);
	
	/* enable the clock for the SPI2 */
	_HAL_RCC_SPI2_CLK_ENABLE() ;
	
	/*fill up the handle structure */
	SpiHandle.Instance               = SPI_2;
	SpiHandle.Init.BaudRatePrescaler = SPI_REG_CR1_BR_PCLK_DIV_64;
	SpiHandle.Init.Direction         = SPI_ENABLE_2_LINE_UNI_DIR    ;
	SpiHandle.Init.CLKPhase          = SPI_SECOND_CLOCK_TRANS  ;
	SpiHandle.Init.CLKPolarity       = SPI_CPOL_HIGH;
	SpiHandle.Init.DataSize          = SPI_8BIT_DF_ENABLE;
	SpiHandle.Init.FirstBit          = SPI_TX_MSB_FIRST;
	SpiHandle.Init.NSS               = SPI_SSM_ENABLE  ;
	SpiHandle.Init.Mode              = SPI_MASTER_MODE_SEL;
	
	SpiHandle.State = HAL_SPI_STATE_READY;
	
	
	/* Call driver API to initialize the SPI device */
	hal_spi_init(&SpiHandle);
	
		/* Enable the IRQs in the NVIC */
  NVIC_EnableIRQ(SPI2_IRQn);

/* First initilaize the Debug UART */
 // hal_debug_uart_init( DEBUG_USART_BAUD_9600);
	
	//uart_printf("SPI master Application Running ... \n");
	
	


/******************************************************************************/
/*                                                                            */
/*                        Master command sending code                         */
/*                         Write and read commands                            */
/******************************************************************************/
int ii=0;
    
  
		
		
		 
		    addrcmd[0] =0xFC ;   //this is for st-acclro
	      addrcmd[1] =0xB8;
	             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);

				        
				         hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	               //addrcmd[0] =(0x32|0x0C0) ;
	               hal_spi_master_tx(&SpiHandle, addrcmd,2);
		             
	               
		             
	 
    //                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	              //hal_spi_master_rx(&SpiHandle,master_read_buffer,4);
      //            hal_spi_master_rx(&SpiHandle,master_read_buffer,2);		
	
	/// wait untill ACK reception finishes 
	               while(SpiHandle.State != HAL_SPI_STATE_READY );
						 delay_gen();
						 delay_gen();
								 
		             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
	 
		  delay_gen();
		  delay_gen();
			
			
			  addrcmd[0] =0x38 ;   //this is for starting parameters like hold time and hold torque
	      addrcmd[1] =0xDF;
	             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);

				        
				         hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	               //addrcmd[0] =(0x32|0x0C0) ;
	               hal_spi_master_tx(&SpiHandle, addrcmd,2);
		             
	               
		             
	 
    //                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	              //hal_spi_master_rx(&SpiHandle,master_read_buffer,4);
      //            hal_spi_master_rx(&SpiHandle,master_read_buffer,2);		
	
	/// wait untill ACK reception finishes 
	               while(SpiHandle.State != HAL_SPI_STATE_READY );
						 delay_gen();
						 delay_gen();
								 
		             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
     
   
	 
		 delay_gen();
		  delay_gen();
			
			
			  addrcmd[0] =0x95 ;   //this is for starting speed
	      addrcmd[1] =0x4B;
	             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);

				        
				         hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	               //addrcmd[0] =(0x32|0x0C0) ;
	               hal_spi_master_tx(&SpiHandle, addrcmd,2);
		             
	               
		             
	 
    //                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	              //hal_spi_master_rx(&SpiHandle,master_read_buffer,4);
      //            hal_spi_master_rx(&SpiHandle,master_read_buffer,2);		
	
	/// wait untill ACK reception finishes 
	               while(SpiHandle.State != HAL_SPI_STATE_READY );
						 delay_gen();
						 delay_gen();
								 
		             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
     
   
	 
		 delay_gen();
		  delay_gen();
			

		    addrcmd[0] =0x77 ;   //this is for st-acclro
	      addrcmd[1] =0x32;
	             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);

				        
				         hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	               //addrcmd[0] =(0x32|0x0C0) ;
	               hal_spi_master_tx(&SpiHandle, addrcmd,2);
		             
	               
		             
	 
    //                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	              //hal_spi_master_rx(&SpiHandle,master_read_buffer,4);
      //            hal_spi_master_rx(&SpiHandle,master_read_buffer,2);		
	
	/// wait untill ACK reception finishes 
	               while(SpiHandle.State != HAL_SPI_STATE_READY );
						 delay_gen();
						 delay_gen();
								 
		             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
     
   
	 
		 delay_gen();
		  delay_gen();
			
			
			
			 // addrcmd[0] =0xB5 ;   //this is for st-acclro
	     // addrcmd[1] =0x30;
	       //      hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);

				        
				   //      hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
           //      while(SpiHandle.State != HAL_SPI_STATE_READY );
	               //addrcmd[0] =(0x32|0x0C0) ;
	        //       hal_spi_master_tx(&SpiHandle, addrcmd,2);
		             
	               
		             
	 
    //                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	              //hal_spi_master_rx(&SpiHandle,master_read_buffer,4);
      //            hal_spi_master_rx(&SpiHandle,master_read_buffer,2);		
	
	/// wait untill ACK reception finishes 
	          //     while(SpiHandle.State != HAL_SPI_STATE_READY );
						// delay_gen();
						// delay_gen();
								 
		        //     hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
	 
		  //delay_gen();
		  //delay_gen();
			
			
        
		    addrcmd[0] =0xFC ;   //this is for st-acclro
	      addrcmd[1] =0xB8;
	             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);

				        
				         hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	               //addrcmd[0] =(0x32|0x0C0) ;
	               hal_spi_master_tx(&SpiHandle, addrcmd,2);
		             
	               
		             
	 
                       //  while(SpiHandle.State != HAL_SPI_STATE_READY );
	             // hal_spi_master_rx(&SpiHandle,master_read_buffer,4);
                    //  hal_spi_master_rx(&SpiHandle,master_read_buffer,2);		
	
	/// wait untill ACK reception finishes 
	               while(SpiHandle.State != HAL_SPI_STATE_READY );
						 delay_gen();
						 delay_gen();
								 
		             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
     
   
	 	         delay_gen();
						 delay_gen();
		 
		 while(1)
		 {
			  Xg= hal_gpio_read_from_pin(GPIOB,7);
	      Yg=	hal_gpio_read_from_pin(GPIOB,8);
			
				 if(Xg==1&&Yg==0&&Zg==0)
				 {
			      Zg=1;
					  Pg=0;
					  Cg=0;
	         addrcmd[0] =0xFC ;   //this is for st-acclro
	         addrcmd[1] =0xB8;
	             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);

				        
				         hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	               //addrcmd[0] =(0x32|0x0C0) ;
	               hal_spi_master_tx(&SpiHandle, addrcmd,2);
		             
	               
		             
	 
    //                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	              //hal_spi_master_rx(&SpiHandle,master_read_buffer,4);
      //            hal_spi_master_rx(&SpiHandle,master_read_buffer,2);		
	
	/// wait untill ACK reception finishes 
	               while(SpiHandle.State != HAL_SPI_STATE_READY );
						 delay_gen();
						 delay_gen();
								 
		             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
     
             delay_gen();
						 delay_gen();
	 
	      	 delay_long();				  
					  
			         addrcmd[0] =0xFC ;   //this is for st-acclro
	             addrcmd[1] =0xBB;
	             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);

				        
				         hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	               //addrcmd[0] =(0x32|0x0C0) ;
	               hal_spi_master_tx(&SpiHandle, addrcmd,2);
		             
	               
		             
	 
    //                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	              //hal_spi_master_rx(&SpiHandle,master_read_buffer,4);
      //            hal_spi_master_rx(&SpiHandle,master_read_buffer,2);		
	
	/// wait untill ACK reception finishes 
	               while(SpiHandle.State != HAL_SPI_STATE_READY );
						 delay_gen();
						 delay_gen();
								 
		             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
             delay_gen();
						 delay_gen();
	        }
	 
		 
		 
	//##########################-allegro spi-read--################	 
		/*  addrcmd[0] =0xEC ;   //this is for st-acclro
	      addrcmd[1] =0x00;
	             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);

				        
				         hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	               //addrcmd[0] =(0x32|0x0C0) ;
	               hal_spi_master_tx(&SpiHandle, addrcmd,2);
		             
	               
		             
	 
                         while(SpiHandle.State != HAL_SPI_STATE_READY );
	             // hal_spi_master_rx(&SpiHandle,master_read_buffer,4);
                      hal_spi_master_rx(&SpiHandle,master_read_buffer,2);		
	
	/// wait untill ACK reception finishes 
	               while(SpiHandle.State != HAL_SPI_STATE_READY );
						 delay_gen();
						 delay_gen();
						  delay_gen();
						 delay_gen();
						  delay_gen();
								 
		             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
     
   
	 
		 
		  delay_long();
			
			//#########################--allegro-spi-read--end--#################
			*/
				 if(Xg==0&&Yg==0&&Cg==0)
				 {
					  Pg=0;
           	Zg=0;
					  Cg=1;
			     addrcmd[0] =0xFC ;   //this is for st-acclro
	         addrcmd[1] =0xB8;
	             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);

				        
				         hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	               //addrcmd[0] =(0x32|0x0C0) ;
	               hal_spi_master_tx(&SpiHandle, addrcmd,2);
		             
	               
		             
	 
    //                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	              //hal_spi_master_rx(&SpiHandle,master_read_buffer,4);
      //            hal_spi_master_rx(&SpiHandle,master_read_buffer,2);		
	
	/// wait untill ACK reception finishes 
	               while(SpiHandle.State != HAL_SPI_STATE_READY );
						 delay_gen();
						 delay_gen();
								 
		             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
     
             delay_gen();
						 delay_gen();
	 
	      	 delay_long();
		  
		     }
			
  			if(Yg==1&&Xg==0&&Pg==0)
	       		{		
               Pg=1;
               Cg=0;
                 addrcmd[0] =0xFC ;   //this is for st-acclro
	         addrcmd[1] =0xB8;
	             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);

				        
				         hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	               //addrcmd[0] =(0x32|0x0C0) ;
	               hal_spi_master_tx(&SpiHandle, addrcmd,2);
		             
	               
		             
	 
    //                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	              //hal_spi_master_rx(&SpiHandle,master_read_buffer,4);
      //            hal_spi_master_rx(&SpiHandle,master_read_buffer,2);		
	
	/// wait untill ACK reception finishes 
	               while(SpiHandle.State != HAL_SPI_STATE_READY );
						 delay_gen();
						 delay_gen();
								 
		             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
     
             delay_gen();
						 delay_gen();
	 
  	      	 delay_long();
					 
      				 addrcmd[0] =0xFC ;   //this is for st-acclro
	             addrcmd[1] =0xB9;
	             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);

				        
				         hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	               //addrcmd[0] =(0x32|0x0C0) ;
	               hal_spi_master_tx(&SpiHandle, addrcmd,2);
		             
	               
		             
	 
    //                 while(SpiHandle.State != HAL_SPI_STATE_READY );
	              //hal_spi_master_rx(&SpiHandle,master_read_buffer,4);
      //            hal_spi_master_rx(&SpiHandle,master_read_buffer,2);		
	
	/// wait untill ACK reception finishes 
	               while(SpiHandle.State != HAL_SPI_STATE_READY );
						 delay_gen();
						 delay_gen();
								 
		             hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
     
             delay_gen();
						 delay_gen();
	 
             }
		 
				 
    

      }
	
	
  hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
	delay_gen();
	
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* Master write command data format */
	//addrcmd[0] =0x31 ;  this is for adxl-345
	//addrcmd[1] =0x0B;
	addrcmd[0] =0x20 ;   //this is for st-acclro
	addrcmd[1] =0x67;
	
	/* first send the master write cmd to slave */
	hal_spi_master_tx(&SpiHandle, addrcmd,2);
	
	/* application can block here, or can do other task untill above tx finishes */
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* this dealy helps for the slave to be ready with the ACK bytes */
	delay_gen();
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
	
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
	delay_gen();
	
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* Master write command data format */
	//addrcmd[0] =0x31 ;  this is for adxl-345
	//addrcmd[1] =0x0B;
	addrcmd[0] =0x24 ;   //this is for st-acclro
	addrcmd[1] =0x20;
	
	/* first send the master write cmd to slave */
	hal_spi_master_tx(&SpiHandle, addrcmd,2);
	
	/* application can block here, or can do other task untill above tx finishes */
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* this dealy helps for the slave to be ready with the ACK bytes */
	delay_gen();
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
	
//	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
//	delay_gen();
	
//	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* Master write command data format */
	//addrcmd[0] =0x31 ;  this is for adxl-345
	//addrcmd[1] =0x0B;
//	addrcmd[0] =0x24 ;   //this is for st-acclro
//	addrcmd[1] =0x48;
	
	/* first send the master write cmd to slave */
//	hal_spi_master_tx(&SpiHandle, addrcmd,2);
	
	/* application can block here, or can do other task untill above tx finishes */
	//while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* this dealy helps for the slave to be ready with the ACK bytes */
//	delay_gen();
//	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
	
	while(1)
	{
		
		hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	addrcmd[0] =(0x28|0x80) ;
	hal_spi_master_tx(&SpiHandle, addrcmd,1);
	delay_gen();
  //while(SpiHandle.State != HAL_SPI_STATE_READY );
	hal_spi_master_rx(&SpiHandle,master_read_buffer,1);	
	
	/* wait untill ACK reception finishes */
	 while(SpiHandle.State != HAL_SPI_STATE_READY );
		hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
    	delay_gen();
		delay_gen();delay_gen();delay_gen();delay_gen();
		
	}
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
	delay_gen();
	
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* Master write command data format */
	addrcmd[0] =0x1E ;
	addrcmd[1] =-0x03 ;
	
	/* first send the master write cmd to slave */
	hal_spi_master_tx(&SpiHandle, addrcmd,2);
	
	/* application can block here, or can do other task untill above tx finishes */
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* this dealy helps for the slave to be ready with the ACK bytes */
	delay_gen();
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
	
	
	
	
	
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* Master write command */
	addrcmd[0] =0x38 ;
	addrcmd[1] =0x00 ;
	
	/* first send the master write cmd to slave **/
	hal_spi_master_tx(&SpiHandle, addrcmd,2);
	
	/* application can block here, or can do other task untill above tx finishes */
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* this dealy helps for the slave to be ready with the ACK bytes */
	delay_gen();
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
	
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* Master write command */
	addrcmd[0] =0x2D ;
	addrcmd[1] =0x00 ;
	
	/* first send the master write cmd to slave */
	hal_spi_master_tx(&SpiHandle, addrcmd,2);
	
	/* application can block here, or can do other task untill above tx finishes */
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* this dealy helps for the slave to be ready with the ACK bytes */
	delay_gen();
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
	
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* Master write command */
	addrcmd[0] =0x2D ;
	addrcmd[1] =0x08 ;
	
	/* first send the master write cmd to slave */
	hal_spi_master_tx(&SpiHandle, addrcmd,2);
	
	/* application can block here, or can do other task untill above tx finishes */
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* this dealy helps for the slave to be ready with the ACK bytes */
	delay_gen();
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
while(1)
{
	
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* Master write command */
	
	
	addrcmd[0] =0x2D ;
	addrcmd[1] =0x08 ;
	
	/* first send the master write cmd to slave */
	hal_spi_master_tx(&SpiHandle, addrcmd,2);
	
	/* application can block here, or can do other task untill above tx finishes */
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* this dealy helps for the slave to be ready with the ACK bytes */
	//delay_gen();
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);

	
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	addrcmd[0] =(0x32|0x0C0) ;
	hal_spi_master_tx(&SpiHandle, addrcmd,1);
	//delay_gen();
  //while(SpiHandle.State != HAL_SPI_STATE_READY );
	hal_spi_master_rx(&SpiHandle,master_read_buffer,6);	
	
	/* wait untill ACK reception finishes */
	 while(SpiHandle.State != HAL_SPI_STATE_READY );
		hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);

	   // delay_gen();
	  //master_read_buffer[2] = (((master_read_buffer[1]) << 8) | master_read_buffer[0]);   
    //master_read_buffer[2]= (((master_read_buffer[3]) << 8) | master_read_buffer[2]);   
    //master_read_buffer[4]= (((master_read_buffer[5]) << 8) | master_read_buffer[4]); 
    
	 X=((master_read_buffer[1]));
     //x= 0.004*9.81*((((int)(master_read_buffer[1]|(master_read_buffer[0]<<8)))));	
		 rp=0.004;
		// rx_buf=master_read_buffer[0];
	  // if(rx_buf==(0xFF))
   Y=master_read_buffer[3];
	 Z=master_read_buffer[5];
		// {
		 // rx_buf=0x00;
		//	 rp=-rp;
			 
		// }
		// if(rx_buf==(0xFE))
   /// {
      //rx_buf=0x00;
		//	rp=-rp;
		//}
		 x= ((((int)(master_read_buffer[1])|(master_read_buffer[0]<<8))));	
	    
  /*	 Yg=rp*9.81*(((int)(master_read_buffer[3])));
		
		
		 
     Zg= rp*9.81*(((int)(master_read_buffer[5])));
     
		 		 
	    X = Xg * alpha + (x * (1.0 - alpha));
      Y = Yg * alpha + (y* (1.0 - alpha));
      Z = Zg * alpha + (x * (1.0 - alpha));
 x=(int)Xg;
 y=(int)Yg;
 z=(int)Zg;
	    */
	// Roll = atan2(y, z) * 180/PI;
   //Pitch = atan2(-x, sqrt(y*y + z*z)) * 180/PI;
	/*
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
	
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	addrcmd[0] =(0x33|0x080) ;
	hal_spi_master_tx(&SpiHandle, addrcmd,1);
	//delay_gen();
  //while(SpiHandle.State != HAL_SPI_STATE_READY );
	hal_spi_master_rx(&SpiHandle,&master_read_buffer[1],1);	
	
	// wait untill ACK reception finishes 
	 while(SpiHandle.State != HAL_SPI_STATE_READY );
		hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
 
	delay_gen();
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	addrcmd[0] =(0x34|0x080) ;
	hal_spi_master_tx(&SpiHandle, addrcmd,1);
	//delay_gen();
  //while(SpiHandle.State != HAL_SPI_STATE_READY );
	hal_spi_master_rx(&SpiHandle,&master_read_buffer[2],1);	
	
	// wait untill ACK reception finishes /
	 while(SpiHandle.State != HAL_SPI_STATE_READY );
		hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
 
	delay_gen();
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	addrcmd[0] =(0x35|0x080) ;
	hal_spi_master_tx(&SpiHandle, addrcmd,1);
	//delay_gen();
  //while(SpiHandle.State != HAL_SPI_STATE_READY );
	hal_spi_master_rx(&SpiHandle,&master_read_buffer[3],1);	
	
	/// wait untill ACK reception finishes /
	 while(SpiHandle.State != HAL_SPI_STATE_READY );
		hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
 
	delay_gen();
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	addrcmd[0] =(0x36|0x080) ;
	hal_spi_master_tx(&SpiHandle, addrcmd,1);
	//delay_gen();
  //while(SpiHandle.State != HAL_SPI_STATE_READY );
	hal_spi_master_rx(&SpiHandle,&master_read_buffer[4],1);	
	
	// wait untill ACK reception finishes /
	 while(SpiHandle.State != HAL_SPI_STATE_READY );
		hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
 
	delay_gen();
	hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,RESET);
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	addrcmd[0] =(0x37|0x080) ;
	hal_spi_master_tx(&SpiHandle, addrcmd,1);
	//delay_gen();
  //while(SpiHandle.State != HAL_SPI_STATE_READY );
	hal_spi_master_rx(&SpiHandle,&master_read_buffer[5],1);	
	
	/// wait untill ACK reception finishes 
	 while(SpiHandle.State != HAL_SPI_STATE_READY );
		hal_gpio_write_to_pin(GPIOE,SPI_CS_PIN,SET);
 
	*/
	 delay_gen();
   delay_gen();
   delay_gen();
	
}
	return 0;
}



/**
  * @brief  This function handles SPI2 interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_IRQHandler(void)
{
	/* call the driver api to process this interrupt */
  hal_spi_irq_handler(&SpiHandle);
}


/**
  * @brief  This function handles EXTI0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
	/* In the ISR, first clear out the sticky interrupt pending bit for this interrupt */
  hal_gpio_clear_interrupt(GPIO_BUTTON_PIN);
	/* Do your task here */
	TestReady = SET;
}
void USARTx_IRQHandler(void)
{
	
hal_uart_handle_interrupt(&uart_handle);
	
	
}


void SysTick_Handler(void)
{
	
	msticks++;
     
}

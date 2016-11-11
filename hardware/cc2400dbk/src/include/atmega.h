/*****************************************************************************
 *                                                                           *
 *        **********                                                         *
 *       ************                                                        *
 *      ***        ***                                                       *
 *      ***   +++   ***                                                      *
 *      ***   + +   ***                                                      *
 *      ***   +                            CHIPCON CC2400                    *
 *      ***   + +   ***                  ATMEGA INCLUDE FILE                 *
 *      ***   +++   ***                                                      *
 *      ***       ***                                                        *
 *       ***********                                                         *
 *        *********                                                          *
 *                                                                           *
 *****************************************************************************
 * A number of helpful constants and macros are also included which can      *
 * be used to increase the legibility of your code.                          *                                                                           *
 *****************************************************************************
 * Device:      CC2400                                                       *
 * File:		atmega.h					                                 *
 * Author:		MBR						                                     *
 * Target:		ATmega8						                                 *
 * Created:		2004-01-05					                                 *
 *				    			                                             *
 *****************************************************************************
 * Revision history:                                                         *
 *                                                                           *
 * $Log: atmega.h,v $
 * Revision 1.3  2004/01/28 09:17:28  mbr
 * Changed: UART macros, UART0_WAIT() & UART_TX() & ucTimer2Init()
 * Added: Timer 2 options macros
 *
 * Revision 1.2  2004/01/20 14:51:25  mbr
 * Cosmetic Changes
 * Changes - ENABLE_PKT_INT() and ENABLE_FIFO_INT() macros
 *
 * Revision 1.1  2004/01/07 13:06:01  tos
 * Initial version in CVS.
 *                                                        *
 *                                                                           *  
 *									                                         *	
 *                                                                           *
 ****************************************************************************/

#ifndef ATMEGA_H
#define ATMEGA_H                   

#ifndef C8_MHZ
	#define C8_MHZ         // Main clock is 8MHz 
#endif

#define CLK_FREQ    4000000

#ifndef FALSE
	#define FALSE		0
#endif

#ifndef TRUE
	#define TRUE		1
#endif

#ifndef NULL
	#define NULL		0
#endif

// Buffer Defines 
#define RX_BUFFER_SIZE 16     // 2,4,8,16,32,64,128 or 256 bytes 
#define TX_BUFFER_SIZE 16     // 2,4,8,16,32,64,128 or 256 bytes 
											
// Atmel AVR SPI port definitions 

#define SS    PB2 // Port B pin 2
#define MOSI  PB3 // Port B pin 3
#define MISO  PB4 // Port B pin 4
#define SCK   PB5 // Port B pin 5


#ifdef C8_MHZ
	#define	T10_MS	78  // Set the timer tick to 10 ms using internal 8MHz RC oscillator 
	#define	T1_MS	8	// Set the timer tick to 1 ms using internal 8MHz RC oscillator   
#endif                  // Use timer0 with CLK/1024 

// Registro macro defines for Atmega8
#define UART_STATUS_REG	        UCSRA
#define RECEIVE_COMPLETE_BIT	RXC
#define TRANSMIT_COMPLETE_BIT   UDRE
#define TXC_COMPLETE_BIT   	TXC

#define TIMER2_COUNT_REG        TCNT2
#define UART_DATA_REG	        UDR
#define SPI_STATUS_REG          SPSR
#define SPI_COMPLETE_BIT        SPIF

// Voltage reference for ADC
#define V_REF           		3.300  

 		
/***************************************************************************************************
 * 			COMPILER INDEPENDENT DATA TYPES                                            *
 ***************************************************************************************************
*/ 
typedef unsigned char BOOL;
typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned char uint8;
typedef signed char sint8;
typedef char int8;
typedef unsigned int uint16;
typedef signed int sint16;
typedef unsigned long uint32;
typedef signed long sint32;

extern char dummy;

/***************************************************************************
 * 			MACROS		                                    			   *
 ***************************************************************************
*/ 
// Switch macro 
#define SWITCH(a,b) do { b^=a; a^=b; b^=a; } while (0)
											
// SPI Port bit settings
#define SPI_ENABLE()    (PORTB &= ~(1<<SS))
#define SPI_DISABLE()   (PORTB |= (1<<SS))
#define CLR_SPI_IRQ()   SPI_STATUS_REG &= ~(1<<SPI_COMPLETE_BIT)
//----------------------------------------------------------------------------
// Timer 2 Macros 
//----------------------------------------------------------------------------
// Option defines Timer 2
#define NO_CLK_SOURCE	0x00	// No clock source
#define T2_DIV_0		0x01	// No Prescaling
#define T2_DIV_8		0x02	// Prescaling clk/8
#define T2_DIV_32		0x03	// Prescaling clk/32
#define T2_DIV_64		0x04	// Prescaling clk/64
#define T2_DIV_128		0x05	// Prescaling clk/128	
#define T2_DIV_256		0x06	// Prescaling clk/256
#define T2_DIV_1024		0x07	// Prescaling clk/1024
#define CLR_T2_CMP		0x08	// Clear Timer on Compare (CTC)

// Macro to stop timer 2
#define T2_DISABLE() 	CLRBIT(TCCR2,((1<<CS22)|(1<<CS21)|(1<<CS20))) // 
#define T2_ENABLE() 	SETBIT(TCCR2,((1<<CS22)|(1<<CS20)))
//----------------------------------------------------------------------------
// Atmega8 SPI Macros 
//----------------------------------------------------------------------------
#define FASTSPI_DISABLE() do { SPI_DISABLE(); dummy++; } while (0)
//----------------------------------------------------------------------------
// Macro used for communication data polling and wait on the SPI bus 
//----------------------------------------------------------------------------
#define FASTSPI_WAIT() \
	do { \
		while (!(SPSR & (1<<SPIF))); \
		CLR_SPI_IRQ(); \
	} while (0) 
//----------------------------------------------------------------------------
// Macro to write a byte on the SPI bus 
//----------------------------------------------------------------------------
#define FASTSPI_TX(x) \
	do { \
		SPDR = x; \
		FASTSPI_WAIT(); \
	} while (0)
//----------------------------------------------------------------------------
// Macro to read a byte on the SPI bus 
//----------------------------------------------------------------------------
#define FASTSPI_RX(x) \
	do { \
		SPDR = 0; \
		FASTSPI_WAIT(); \
		x = SPDR; \
	} while (0)
//----------------------------------------------------------------------------
// Macro to read a word on the SPI bus 
//----------------------------------------------------------------------------
#define FASTSPI_RX_WORD(x) \
	do { \
		SPDR = 0; \
		FASTSPI_WAIT(); \
		x = SPDR << 8; \
		SPDR = 0; \
		FASTSPI_WAIT(); \
		x |= SPDR; \
	} while (0)
//----------------------------------------------------------------------------
// Macro to write an address to CC2400 and updating the global status byte
//----------------------------------------------------------------------------
#define FASTSPI_TX_ADDR(a) \
	do { \
		SPDR = a; \
		FASTSPI_WAIT(); \
		CC2400_Status = SPDR; \
	} while (0)
//----------------------------------------------------------------------------
// Macro to read an address in CC2400 and updating the global status byte
//----------------------------------------------------------------------------
#define FASTSPI_RX_ADDR(a) \
	do { \
		SPDR = (a) | 0x80; \
		FASTSPI_WAIT(); \
		CC2400_Status = SPDR; \
	} while (0)
//----------------------------------------------------------------------------
// Macro to access the command strobe register in CC2400 
//----------------------------------------------------------------------------
#define FASTSPI_STROBE(a) \
	do { \
		SPI_ENABLE(); \
		FASTSPI_TX_ADDR(a); \
		FASTSPI_DISABLE(); \
	} while (0)
//----------------------------------------------------------------------------
// Macro to write to a register in CC2400 
//----------------------------------------------------------------------------
#define FASTSPI_SETREG(a,v) \
	do { \
		SPI_ENABLE(); \
		FASTSPI_TX_ADDR(a); \
		FASTSPI_TX((BYTE)((v) >> 8)); \
		FASTSPI_TX((BYTE)(v)); \
		FASTSPI_DISABLE();\
	} while (0)
//----------------------------------------------------------------------------
// Macro to read a register and store the value from CC2400 
//----------------------------------------------------------------------------
#define FASTSPI_GETREG(a,v) \
	do { \
		SPI_ENABLE(); \
		FASTSPI_RX_ADDR(a); \
		FASTSPI_RX_WORD(v); \
		FASTSPI_DISABLE(); \
	} while (0)
//----------------------------------------------------------------------------
// Macro to read and update the global Status Byte from
// by addressing the CC2400 MAIN register 
//----------------------------------------------------------------------------
#define FASTSPI_UPD_STATUS() \
	do { \
		SPI_ENABLE(); \
		FASTSPI_RX_ADDR(CC2400_MAIN); \
		SPI_DISABLE(); \
	} while (0)
//----------------------------------------------------------------------------
// Macro to fill the CC2400 FIFO 
//----------------------------------------------------------------------------
// Usage example:
//     uint8 n 		// Local counter variable
//	   uint8 length // The number of bytes to transmit.	
//	   uint8 *pData	// A pointer to the actual data to transmit.
//
//     		FASTSPI_WRITE_FIFO(pData,length,n);
//
#define FASTSPI_WRITE_FIFO(p,c,n) \
	do { \
		SPI_ENABLE(); \
		FASTSPI_TX_ADDR(CC2400_FIFOREG); \
		for (n = 0; n < (c); n++) { \
			FASTSPI_TX((p)[n]); \
		} \
		FASTSPI_DISABLE(); \
	} while (0)
//----------------------------------------------------------------------------
// Macro to read and empty the content of the CC2400 FIFO 
//----------------------------------------------------------------------------	
#define FASTSPI_READ_FIFO(p,c,n) \
	do { \
		SPI_ENABLE(); \
		FASTSPI_RX_ADDR(CC2400_FIFOREG); \
		for (n = 0; n < (c); n++) { \
			FASTSPI_RX((p)[n]); \
		} \
		FASTSPI_DISABLE(); \
	} while (0)
//----------------------------------------------------------------------------
// Macro to fill the CC2400 FIFO by using a given buffer size   
//----------------------------------------------------------------------------	
#define FASTSPI_WRITE_FIFO_BUF(p,c,n) \
	do { \
		SPI_ENABLE(); \
		FASTSPI_TX_ADDR(CC2400_FIFOREG); \
		for (n = 0; n < (c); \
		n=(n+1)%TX_BUFFER_SIZE) { \
			FASTSPI_TX((p)[n]); \
		} \
		FASTSPI_DISABLE(); \
	} while (0)
//----------------------------------------------------------------------------
// Macro to read and empty the content of the CC2400 FIFO 
//----------------------------------------------------------------------------	
#define FASTSPI_READ_FIFO_BUF(p,c,n) \
	do { \
		SPI_ENABLE(); \
		FASTSPI_RX_ADDR(CC2400_FIFOREG); \
		for (n = 0; n < (c);  n=(n+1)%RX_BUFFER_SIZE) { \
			FASTSPI_RX((p)[n]); \
		} \
		FASTSPI_DISABLE(); \
	} while (0)
//----------------------------------------------------------------------------
// Macro to fill a byte in the CC2400 FIFO 
//----------------------------------------------------------------------------		
#define FASTSPI_WRITE_FIFO_BYTE(x) \
	do { \
		SPI_ENABLE(); \
		FASTSPI_TX_ADDR(CC2400_FIFOREG); \
		FASTSPI_TX(x); \
		FASTSPI_DISABLE(); \
	} while(0)
//----------------------------------------------------------------------------
// Macro to read a byte from the CC2400 FIFO 
//----------------------------------------------------------------------------	
#define FASTSPI_READ_FIFO_BYTE(x) \
	do { \
		SPI_ENABLE(); \
		FASTSPI_RX_ADDR(CC2400_FIFOREG); \
		FASTSPI_RX(x); \
		FASTSPI_DISABLE(); \
	} while(0)
	
// Fixed EEPROM location to store OSCCAL Calibration byte
#define CAL_BYTE_ADDR 0x01FF

// EEPROM macros
#define EEPROM_WAIT() \
	do { \
		while(EECR & (1<<EEWE)); \
	} while (0) 
	
// Enable EEPROM read
#define EEPROM_RD_ENABLE() (EECR |= (1<<EERE));	

//----------------------------------------------------------------------------
// Macro to read AVR EEPROM		
//----------------------------------------------------------------------------
#define EEPROM_READ(a,v) \
	do { \
		EEPROM_WAIT(); \
		EEAR = (a); \
		EEPROM_RD_ENABLE(); \
		(BYTE)(v) = EEDR; \
	} while (0) 	
//----------------------------------------------------------------------------
// SERIAL PORT FUNCTIONS AND MACROS	
//----------------------------------------------------------------------------
// Macro to disable and enable UART for Atmega8
#define UART0_DISABLE() 		(UCSRB &= ~(( 1 << RXEN )|(1 << TXEN )))
#define UART0_ENABLE()			(UCSRB |=  (( 1 << RXEN )|(1 << TXEN ))) 

// UART macro to enable disable the UART RX and TX interrupt
#define UART0_INT_DISABLE()		(UCSRB &= ~((1<<UDRIE)|(1<<RXCIE))) 
#define UART0_INT_ENABLE()		(UCSRB |= ((1<<UDRIE)|(1<<RXCIE)))

// UART macro to clear the TX and RX interrupt 
#define UART0_RX_CLR_IRQ()       (UART_STATUS_REG &= ~(1<<RECEIVE_COMPLETE_BIT)) 
#define UART0_TX_CLR_IRQ()       (UART_STATUS_REG &= ~(1<<TRANSMIT_COMPLETE_BIT))

// UART interrupts macros
// UART TX interrupt 
#define UART0_INT_TX_DISABLE()	(UCSRB &= ~(1<<UDRIE)) 
#define UART0_INT_TX_ENABLE()	(UCSRB |= (1<<UDRIE))

// UART RX interrupt
#define UART0_INT_RX_DISABLE()	(UCSRB &= ~(1<<RXCIE)) 
#define UART0_INT_RX_ENABLE()	(UCSRB |= (1<<RXCIE))

// Enable double speed on the UART 
#define UART0_DOUBLE_SPEED()	(UCSRA |= (1 << U2X))

//----------------------------------------------------------------------------
//  UART0_SETUP(baudRate,options, BOOL) 
//
//  Description:
//      Macro which does all the initialization necessary to establish a 
//      simple serial link on serial port. 
//      The UART is configured according to _options_, the different values 
//      which are listed below.
//      The first byte is transmitted simply by using the UART0_SEND(...) macros.
//
//  Arguments:
//      word baudRate
//          Baudrate (, 19200, 57600, ...)
//      byte options
//          One or more of the below constants. The value 0 gives the common 
//          setting UART_TWO_STOP_BIT | UART_EIGHT_CHAR_BIT | REGISTER_SEL.
//      BOOL
//          FALSE = 8-bit 
//			TRUE  = 9-bit 
//----------------------------------------------------------------------------
#define UART0_SETUP(baudRate,options,BOOL)do{UBRRH=(baudRate)>>8; UBRRL=(baudRate);UCSRC=options; if(BOOL){UCSRB|=0x40;}}while(0)

// options is one or more of these constants:
#define UART_TWO_STOP_BIT		0x08	// Two stop bit enabled 
#define UART_EVEN_POLARITY		0x20	// Enable Even polarity
#define UART_ODD_POLARITY 		0x30	// Enable Odd polarity
#define UART_SIX_CHAR_BIT		0x02	// Enable 6-bit Character size
#define UART_SEVEN_CHAR_BIT		0x04	// Enable 7-bit Character size
#define UART_EIGHT_CHAR_BIT		0x06	// Enable 8-bit Character size
#define REGISTER_SEL			0x80	// Enable Register Select 
#define BAUDRATE_19K2_U2X		0x33	// Set baudrate to 19.2 kbps at double speed (Only Asynch Mode)
#define BAUDRATE_28K8_U2X		0x22	// Set baudrate to 28.8 kbps at double speed (Only Asynch Mode)
#define BAUDRATE_38K4_U2X		0x19	// Set baudrate to 38.4 kbps at double speed (Only Asynch Mode)
#define BAUDRATE_57K6_U2X		0x10	// Set baudrate to 57.6 kbps at double speed (Only Asynch Mode)
#define BAUDRATE_76K8_U2X		0x0C	// Set baudrate to 76.8 kbps at double speed (Only Asynch Mode)
#define BAUDRATE_115K2_U2X		0x08	// Set baudrate to 115.2 kbps at double speed (Only Asynch Mode)
#define BAUDRATE_19200			0x19	// Set baudrate to 19.2 kbps 	
#define BAUDRATE_1M_U2X			0x00	// Set baudrate to 1 Mbps 
                   
///////////////////////////////////////////////////////////////////////////////
// A number of macros have been defined which allow easy use of the serial
// ports in the ATMega:
//   UART0_SETUP(...)           - Used to configure the serial ports (baudrate,
//   					          8-bit / 8-bit  
//   UART0_TX(...)	            - Used to send a single byte with the relevant
//   				              UART in 8-bit mode. A check is made to see 
//                                if the data register is empty.
//   UART0_RX(...)  	        - Used to receive a single byte with the
//  				              relevant UART in 8-bit mode. A check is made
//                                whether a new byte has been received.
//   UART0_WAIT_AND_SEND(x)     - Waits until the transmit buffer has been 
//  					          emptied (and thus sent) and then sends x 
//                                using the 8-bit mode.
//   UART0_WAIT_AND_RECEIVE(x)  - Waits until a byte is received and then
//  						      stores it in x. The 8-bit mode is used.
//
/////////////////////////////////////////////////////////////////////////////// 

//-----------------------------------------------------------------------------
// Macros which are helful when transmitting data over a serial interface.
// Usage example:
//      UART0_TX(data);
//      for (i=0; i<len; i++)
//          UART0_WAIT_AND_SEND(data[i]);
//-----------------------------------------------------------------------------
#define UART0_WAIT() do {while (!(UCSRA & (1<<UDRE))); UART0_TX_CLR_IRQ();} while (0) 
#define UART0_TX(x) do {UDR =(x);} while (0)
#define UART0_WAIT_AND_SEND(x) do {UART0_WAIT(); UART0_TX(x);} while (0)
#define FASTUART_WRITE(p,c) do {char n; UART0_ENABLE(); for(n=0;n<(c);n++){UART0_TX((p)[n]);}UART0_DISABLE();}while(0)
#define FASTUART_PUTPACKET(ch) do {char c[]=ch; uint8 n; if(c[0]!= '\0'){for(n=0;c[n]!= 0; n++){UART0_WAIT_AND_SEND(c[n]);}}}while(0)

//-----------------------------------------------------------------------------
// Macros which are helful when receiving data over a serial interface.
// Usage example:
//      UART0_RX(data);
//      while (len-- > 0) {
//          UART0_WAIT_AND_RECEIVE(data[i++]);
//      }
#define UART0_RX(x) do {UDR = 0; (x) = UDR;} while (0)
#define UART0_WAIT_AND_RECEIVE(x) do {UDR=0; UART0_WAIT(); UART0_RX(x); } while (0)

// Macro to enable global interrupt for the AVR processor	
#define ENABLE_GLOBAL_INT() do { SREG |= 0x80; } while (0)   	

// Macros for external interrupt 1 
#define CLR_INT1_IRQ() do {GIFR = 0x80;} while (0)
#define ENABLE_PKT_INT() do { GICR |= 0x80; CLR_INT1_IRQ(); } while (0)
#define GET_INT1_IRQ() (GIFR & (1<< INTF1)) 
#define DISABLE_PKT_INT() do { GICR &= ~0x80; } while (0)

//Macros for external interrupt 0
#define CLR_INT0_IRQ() do {GIFR = 0x40;} while (0)
#define ENABLE_FIFO_INT() do {GICR |= 0x40; CLR_INT0_IRQ(); } while (0)
#define DISABLE_FIFO_INT() do {GICR &= ~0x40; } while (0)

// Macros for interrupt 1 & 2 
#define INT_EXTERNAL_TRIGGER_EDGE() do {MCUCR |= 0x0A;} while (0)

/***************************************************************************************************
 * 			LOCAL FUNCTION PROTOYPES
 ***************************************************************************************************
*/ 
// Prototypes
void ucInit(BYTE options);
// Microprocessor wait/delay function
void ucWait(uint8 timeOut, uint32 clkFreq);
// Initialize the ADC
void ADCInit(void);
// ADC conversion
WORD ADC_conversion(void);
// Temperature calculation
WORD Temp_calculation(WORD ADC_value);           
// Timer 0 intialize function
void ucTimer0Init(void);
// Timer 1 initialize function
void ucTimer1Init(uint8 timeout);
// Timer 2 initialize function
void ucTimer2Init(uint8 timeout, uint8 options);
//void ucTimer2Init(uint8 timeout);
// EEPROM write 
void EEPROM_write(uint16 address, uint8 data);
// EEPROM read
uint8 EEPROM_read(uint16 address);
// Internal RC Oscillator Calibration
void CalibrateInternalRc(void);

#endif

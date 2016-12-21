/*****************************************************************************
 *                                                                           *
 *        **********                                                         *
 *       ************                                                        *
 *      ***        ***                                                       *
 *      ***   +++   ***                                                      *
 *      ***   + +   ***                                                      *
 *      ***   +                            CHIPCON CC2400                    *
 *      ***   + +   ***                ATMEGA 8 LIBRARY FILE	         	 *
 *      ***   +++   ***                                                      *
 *      ***       ***                                                        *
 *       ***********                                                         *
 *        *********                                                          *
 *                                                                           *
 *****************************************************************************
 * Device:      CC2400                                                       *
 * File:		atmega.c					                                 *
 * Author:		MBR 					                                 	 *
 * Target:		ATmega8 					                                 *
 * Tools:		WINAVRgcc                                        			 *
 * Created:		2004-01-05					                                 *
 * Description:	Low level function for CC2400                                * 
 *		                                                                     *
 *****************************************************************************
 * Revision history:                                                         *
 *                                                                           *
 * $Log: atmega.c,v $
 * Revision 1.3  2004/01/28 09:24:49  mbr
 * Removed #ifdef from ucInit() function
 * Added argument BYTE options to the ucInit() function
 *
 * Revision 1.2  2004/01/20 15:17:53  mbr
 * Cosmetic Changes
 * Removed Function:  ADC_conversion() in ADCInit
 *
 * Revision 1.1  2004/01/07 13:09:43  tos
 * Initial version in CVS.
 *
 * 
 *									                                         *	
 * 									                                         *
 *									                                         *	
 *                                                                           *
 ****************************************************************************/
// Includes 
#include "include.h" // Master Include File

// Global variables
char dummy;
float Temperature;				// float to calculate the temperature
uint8 Cal_value = 0;    		// Temp storage of calibration byte
//----------------------------------------------------------------------------
//  void USART_Init(...)
//
//  Description:
//      A USART initialization function for ATmega8 on CC2400DB
//
//  Arguments:
//      None
//  Return value:
//      None
//----------------------------------------------------------------------------
void USART_Init(void) {
		
	// Read Calibration value from EEPROM and write to 	
	EEPROM_READ(0x01FF,OSCCAL);		
	
	// Enable UART double speed 	  
	UART0_DOUBLE_SPEED();
	   	   
	// Configure UART 
	// 19.2 kBaud, Frame format: 8 data bit,  2 stop bit 
	UART0_SETUP(BAUDRATE_19K2_U2X,(REGISTER_SEL | UART_EIGHT_CHAR_BIT | UART_TWO_STOP_BIT),FALSE);
	    	   
	// Enable UART transmit and receive
	UART0_ENABLE();
		
	// Enable UART interrupts
	UART0_INT_ENABLE();		
} // USART_Init
//----------------------------------------------------------------------------
//  void ADCInit(...)
//
//  Description:
//      A ADC initialization function for ATmega8 on CC2400DB  
//
//  Arguments:
//      None
//  Return value:
//      void
//----------------------------------------------------------------------------
void ADCInit(void) {
	
	//AVCC with external capacitor at VREF, single ended input 
    ADMUX = 0x47;            
    // Delay 5 ms in order for channel selection to settle         
    ucWait(50,CLK_FREQ);           

    // ADC Enable
    // Set ADC prescaler to  8 
    ADCSRA = (1<<ADEN) | (1<<ADPS2);    
    
} // ADCInit
//----------------------------------------------------------------------------
//  WORD ADC_conversion(...)
//
//  Description:
//      This function perfroms eight ADC conversions and establishes an
//      average ADC value found by the internal Atmega8 ADC.
//
//  Arguments:
//     None
//     
//  Return value:
//      The ADC value found for the ADC conversion preformed
//----------------------------------------------------------------------------
WORD ADC_conversion(void) {
	uint16 ADC_temp;
	uint16 ADCresult = 0;
	uint8 i;
	uint16 temperature;
    
    //Do the ADC conversion 8 times for better accuracy    
	for(i = 0; i < 8; i++) {							 
		ADCSRA |= (1 << ADSC);			 	//Start conversion
        	while(!(ADCSRA & 0x10));		//Wait for conversion done, ADIF flag active

		ADC_temp = ADCL;            		//Read out ADCL register
		ADC_temp += (ADCH << 8);    		//Read out ADCH register        

		ADCresult += ADC_temp;      		//Accumulate result (8 samples) for later averaging
	}
    
    	ADCresult = ADCresult >> 3;     		//Average the 8 samples

	temperature = Temp_calculation(ADCresult);	//Call the temperature calculation function
	return(temperature);
} //ADC_conversion
//----------------------------------------------------------------------------
//  WORD Temp_calculation(...)
//
//  Description:
//      This function calculates the temperature base on the
//      ADC value found by the internal Atmega8 ADC in degrees
//		Celsius.
//
//  Arguments:
//      WORD ADC_value
//          The ADC value found for the ADC conversion preformed
//  Return value:
//      Temperature in degrees Celsius
//----------------------------------------------------------------------------
WORD Temp_calculation(WORD ADC_value) {
	float temp;
	WORD temperature;
    
	temp = ((((float)ADC_value * V_REF) / 1024.0) - 0.6) / 0.01;      		
	
	if(((float)temp - (WORD)temp) < 0.6)
		temperature = (WORD)temp;
	else
		temperature = (WORD)temp + 1;
	
	return(temperature);
} // Temp_calculation
//----------------------------------------------------------------------------
//  void ucInit(...)
//
//  Description:
//      The main initialization function for ATmega8 on CC2400DB  
//
//  Arguments:
//      None
//  Return value:
//      void
//----------------------------------------------------------------------------
void ucInit(BYTE options) {	
	//----------------------------------------------------------------------------
	// Set Port B pins    
	// PB.0 - Input: Push Button  
	// PB.1 - Output: CTS - Clear to Send
	// PB.2 - Output: SPI Chip Select (nCS) 
	// PB.3 - Output: MOSI - Master Output Slave Input
	// PB.4 - Input: MISO - Master Input Slave output
	// PB.5 - Output: SCk - SPI Serial Clock
	// PB.6 - Input: GIO1 from CC2400
	// PB.7 - Input: GIO6 from CC2400
	// PB7 PB6 PB5 PB4 PB3 PB2 PB1 PB0
	//  0   0   1   0   1   1   1   0
	//
	// 0b00101110 -> 0x2E
	//-----------------------------------------------------------------------------
	outp(0x2E,DDRB);
	//----------------------------------------------------------------------------
	// Set Port B pins    
	// PB.0 - Pull Up Enable   
	// PB.1 - Pull Up Enable 
	// PB.2 - Pull Up Disable, Tri-state (Hi-Z) 
	// PB.3 - Pull Up Disable, Tri-state (Hi-Z)  
	// PB.4 - Pull Up Disable, Tri-state (Hi-Z)  
	// PB.5 - Pull Up Disable, Tri-state (Hi-Z)  
	// PB.6 - Pull Up Enable
	// PB.7 - Pull Up Enable
	// PB7 PB6 PB5 PB4 PB3 PB2 PB1 PB0
	//  1   1   0   0   0   0   1   1
	//
	// 0b11000011 -> 0xC3
	//-----------------------------------------------------------------------------	
	outp(0xC3,PORTB);		
	//----------------------------------------------------------------------------
	// Set Port C pins    
	// PC.0 - Input: Joystick Up   
	// PC.1 - Input: Joystick Right
	// PC.2 - Output: Strobe TX CC2400 
	// PC.3 - Output: Strobe RX CC2400 
	// PC.4 - Output: Strobe FORCE_ON (UART transceiver ON/OFF)
	// PC.5 - Output: Green LED
	// PC.6 - RESET
	// PC.7 - 
	// PC7 PC6 PC5 PC4 PC3 PC2 PC1 PC0
	//  0   0   1   1   1   1   0   0
	//
	// 0b00101110 -> 0x2E
	//-----------------------------------------------------------------------------
	outp(0x3C,DDRC);
	//----------------------------------------------------------------------------
	// Set Port C pins    
	// PC.0 - Pull Up Disable, Tri-state (Hi-Z)   
	// PC.1 - Pull Up Enable 
	// PC.2 - Pull Up Disable, Tri-state (Hi-Z)  
	// PC.3 - Pull Up Disable, Tri-state (Hi-Z)  
	// PC.4 - Pull Up Disable, Tri-state (Hi-Z)  
	// PC.5 - Pull Up Disable, Tri-state (Hi-Z)  
	// PC.6 - RESET w/external Pull Up
	// PC.7 - 
	// PC7 PC6 PC5 PC4 PC3 PC2 PC1 PC0
	//  0   0   1   1   0   0   0   0
	//
	// 0b00110000 -> 0x30
	//-----------------------------------------------------------------------------	
	outp(0x30,PORTC);	
	//----------------------------------------------------------------------------
	// Set Port D pins    
	// PD.0 - Input: RXD0    
	// PD.1 - Output: TXD0 
	// PD.2 - Input: External Interrupt signal FIFO from CC2400 
	// PD.3 - Input: External Interrupt signal PKT from CC2400 
	// PD.4 - Output: Yellow LED  
	// PD.5 - Input: Joystick Down  
	// PD.6 - Input: Joystick Left 
	// PD.7 - Input: Joystick Push Button 
	// PD7 PD6 PD5 PD4 PD3 PD2 PD1 PD0
	//  0   0   0   1   0   0   1   0
	//
	// 0b00010010 -> 0x12
	//-----------------------------------------------------------------------------	 
	outp(0x12,DDRD);
	//----------------------------------------------------------------------------
	// Set Port D pins    
	// PD.0 - Pull Up Disable, Tri-state (Hi-Z)   
	// PD.1 - Pull Up Disable, Tri-state (Hi-Z)
	// PD.2 - Pull Up Disable, Tri-state (Hi-Z)  
	// PD.3 - Pull Up Disable, Tri-state (Hi-Z)  
	// PD.4 - Pull Up Enable   
	// PD.5 - Pull Up Enable 
	// PD.6 - Pull Up Enable 
	// PD.7 - Pull Up Enable 
	// PD7 PD6 PD5 PD4 PD3 PD2 PD1 PD0
	//  1   1   1   1   0   0   0   0
	//
	// 0b11110000 -> 0xF0
	//-----------------------------------------------------------------------------	 
	outp(0xF0,PORTD);
	
	// Disable SPI interrupt
	SPI_DISABLE();
	
	// Enable SPI, Master, set clock rate fck/2, SPI mode 0 
	SPCR |= (1<<SPE)|(1<<MSTR);
	SPSR |= (1<<SPI2X);

	// Enable all interrupts
	ENABLE_GLOBAL_INT(); 	
	// Trigger level for external interrupts INT0 & INT1, falling edge	
	INT_EXTERNAL_TRIGGER_EDGE();

//------------------------------------------------------------------------------
// Important Note: Options indicating what to configure are set in the include 
// file devboard.h of the CC2400DBK library.
//------------------------------------------------------------------------------
// If read Calibration value 
  if(options & CAL_INIT) {
  	// Read Cal value from EEPROM and write it to the OSCCAL register   	
  	EEPROM_READ(0x01FF,OSCCAL);	  
  }	
//if PKT interrupt is set
  if(options & PKT_INT) {
  	ENABLE_PKT_INT();  
  }	
// If UART is desired	
  if(options & UART_INIT) {
  	   USART_Init();
  }
// If ADC is desired   
  if(options & ADC_INIT) {
  		ADCInit();
  }   
} // ucInit
//----------------------------------------------------------------------------
//  void ucWait(...)
//
//  Description:
//      A wait functions which performs a number of iterations of a simple
//      wait loop, so that at least _timeOut_ ms goes by before the
//      function returns.
//
//  Arguments:
//      byte timeOut
//          The time to wait in ms.
//      word clkFreq
//          The XOSC clock frequency in MHz.
//
//  Return value:
//      void
//----------------------------------------------------------------------------
void ucWait(uint8 timeOut, uint32 clkFreq) {
    uint32 wait;
    
    wait = (uint32)timeOut*(uint32)clkFreq/1024;
    while (wait--);
} // ucWait
//----------------------------------------------------------------------------
//  void ucTimer0Init(...)
//
//  Description:
//      A timer 0 initialization function, that configure 
//      the timer to run at system clock/256. This means the
//		clock tick is 32 microseconds.
//
//  Arguments:
//      None
//          
//  Return value:
//      void
//----------------------------------------------------------------------------
void ucTimer0Init(void) {	
	TIMSK |= (1<<TOIE0);        // Enable output compare interrupt 
	outp(0,TCNT0);              // Start value of T/C 0    
	TIFR |= (1<<TOV0);          // Clear interrupt flag   
	TCCR0 |= (1<<CS02);   		//Prescale CLK/256, start time
} // ucTimer0Init
//----------------------------------------------------------------------------
//  void ucTimer1Init(...)
//
//  Description:
//      A timer 1 initialization function, that configure
//      the timer to run at system clock/256. This means the
//		clock tick is 32 microseconds.
//
//  Arguments:
//      uint8 timeout
//          The time to wait in in steps of 32us
//
//  Return value:
//      void
//----------------------------------------------------------------------------
void ucTimer1Init(uint8 timeout) {
	OCR1A	|= timeout;			// Set timer 1 interval
	TIMSK 	|= (1<<OCIE1A);       // Enable output compare interrupt 
	TIFR 	|= (1<<OCF1A);         // Clear interrupt flag   
	TCCR1B	|= (1<<WGM12)|(1<<CS12);   //Clear timer on compare match, CLK/256, start time	
} // ucTimer1Init
//----------------------------------------------------------------------------
//  void ucTimer2Init(...)
//
//  Description:
//      A timer 2 initialization function, that configure 
//      the timer to run at system clock/256. This means the 
//		clock tick is 32 microseconds.
//
//  Arguments:
//      uint8 timeout
//          The time to wait in in steps of 32us
//		uint8 options
//			The timer prescaling divider setting  		
//  Return value:
//      void
//----------------------------------------------------------------------------
void ucTimer2Init(uint8 timeout, uint8 options) {
	OCR2 = timeout;			// Set timer 2 interval
	TIMSK  |= (1<<OCIE2);   // Enable output compare interrupt 
	TIFR   |= (1<<OCF2);	// Clear interrupt flag   
	TCCR2  |= options;		//Clear timer on compare match 
}// ucTimer2Init
//----------------------------------------------------------------------------
//  void EEPROM_write(...)
//
//  Description:
//      Used to write to an address in the internal ATmega8 EEPROM
//      
//  Arguments:
//      uint16 address
//          The address to be written to in the EEPROM
//		uint8 data
//			The data to be written to selected address
//  Return value:
//      void
//----------------------------------------------------------------------------
void EEPROM_write(uint16 address, uint8 data) {
	
	// Wait for completion of previous write
	while(EECR & (1<<EEWE));
	
	// Set up address and data register
	EEAR = address;
	EEDR = data;
	
	// Write logical one to EEMWE
	EECR |= (1<<EEMWE);
	
	// Start eeprom  write by setting EEWE 
	EECR |= (1<<EEWE);
		
} // EEPROM_write
//----------------------------------------------------------------------------
//  uint8 EEPROM_read(...)
//
//  Description: Used to read an address from internal ATmega8 EEPROM
//      
//  Arguments:
//      uint16 address
//          The address to be read from in the EEPROM
//  Return value:
//		uint8
//      Data at selected read address
//----------------------------------------------------------------------------
uint8 EEPROM_read(uint16 address) {
	
	// Wait for completion of previous write
	while(EECR & (1<<EEWE));
	
	// Set up address and data register
	EEAR = address;
		
	// Start eeprom read by setting EERE  
	EECR |= (1<<EERE);
    	
	// Return data from EEPROM data register	
 	return EEDR; 
} // EEPROM_read
//----------------------------------------------------------------------------
//  void CalibrateInternalRc(...)
//
//  Description:
//		A function that performs calibration of the internal RC of the Atmega8 on
//		CC2400DB.
//		FREQUENCY_XMHz = (10/32768)			10: external ticks     
// 						----------			6:  loop cycles
//						(6/X MHz)			X: RC oscillator frequency 
//
//		FREQUENCY_XMHz = (10/32768) = 407	
//						 ----------		
//						 (6/8e6))
//  Arguments:
//      None
//  Return value:
//      void
//----------------------------------------------------------------------------
// Frequency counter value 8 MHz
#define FREQUENCY_8MHZ 407

void CalibrateInternalRc(void) {
	
	uint8 cycles = 0xFF;
	uint16 count;

	do {
		count = 0;						//Reset counter
		ASSR |= (1<<AS2);				//Set TC2 to be asynchronous from 
										//the CPU clock with a second external 
										//clock(32,768kHz)driving it (PB6)
		TCNT2 = 0x00;					//Reset TC2
		TCCR2 = 0x01; 					//Prescale the timer to be undivided 
										//from clock source
		while (ASSR & 0x07);			//Wait until TC2 is updated
		
		do
			count++; 					//Increment counter for 305 us
		while(TCNT2 < 10);
		
		_NOP();							//Make sure the loop count is 6 in all 
										//iterations
		if(count > FREQUENCY_8MHZ)
			OSCCAL--; 					//If count > 407 - decrease speed
			
		if(count < FREQUENCY_8MHZ) 		
			OSCCAL++; 					//If count < 407 - increase speed
	
	} while (--cycles); 				//Calibrate for 255 cycles

	EEPROM_write(0x01FF, OSCCAL);		//Write calibration value to EEPROM
} // CalibrationInternalRC

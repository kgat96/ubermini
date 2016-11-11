/*****************************************************************************
 *                                                                           *
 *        **********                                                         *
 *       ************                                                        *
 *      ***        ***                                                       *
 *      ***   +++   ***                                                      *
 *      ***   + +   ***                                                      *
 *      ***   +                            CHIPCON CC2400                    *
 *      ***   + +   ***            DEMONSTRATION BOARD INCLUDE FILE    	     *
 *      ***   +++   ***                                                      *
 *      ***       ***                                                        *
 *       ***********                                                         *
 *        *********                                                          *
 *                                                                           *
 *****************************************************************************
 * A number of helpful constants and macros are also included which can      *
 * be used to increase the legibility of your code.  						 *
 * CC2400 demonstration board include file                         		     *                                                                           *
 *****************************************************************************
 * Device:      CC2400                                                       *
 * File:		devboard.h					                                 *
 * Author:		MBR						                                     *
 * Target:		ATmega8						                                 *
 * Created:		2004-01-05					                                 *
 *				    			                                             *
 *****************************************************************************
 * Revision history:                                                         *
 *                                                                           *
 * $Log: devboard.h,v $
 * Revision 1.3  2004/01/28 09:22:10  mbr
 * Added ucInit options macros for CC2400DBK
 *
 * Revision 1.2  2004/01/20 14:49:14  mbr
 * Cosmetic changes
 * Removed - SET_LED(n)
 *
 * Revision 1.1  2004/01/07 13:06:40  tos
 * Initial version in CVS.
 *                                                      *
 *                                                                           *  
 *									                                         *	
 *                                                                           *
 ****************************************************************************/
#ifndef DEVBOARD_H
#define DEVBOARD_H

//-------------------------------------------------------------------------------------------------------

// CC2400DB LEDs (connected to PORT B and PORT C)
#define TOGGLE_GLED   (PORTC = PINC ^ 0x20)
#define TOGGLE_YLED   (PORTD = PIND ^ 0x10)
#define TOGGLE_RLED   (PORTB = PINB ^ 0x02)

// Macros used for the CC2400DB LEDS
#define SET_GLED		(PORTC = PINC & ~0x20)
#define SET_YLED 		(PORTD = PIND & ~0x10)
#define SET_RLED		(PORTB = PINB & ~0x02)

#define CLR_GLED		(PORTC = PINC |  0x20)
#define CLR_YLED 		(PORTD = PIND |  0x10)
#define CLR_RLED		(PORTB = PINB |  0x02)

//---------------------------------------------------------------------------------
// Macros to set Port pins and registers
//---------------------------------------------------------------------------------  
#define SETBIT(x,y)		(x |= y )  // Set bit y in byte x
#define CLRBIT(x,y)		(x &=(~y)) // Clear bit y in byte x
#define CHECKBIT(x,y)	(x & (y))  // Check bit y in byte x
//---------------------------------------------------------------------------------
// Buttons and switches
//---------------------------------------------------------------------------------
#define	SW_PRESSED		PB0		// Push Button

#define JOYSTICK_UP		PB6	    // Up -> Port B
#define JOYSTICK_DOWN	PD5		// Down right -> Port B
#define JOYSTICK_LEFT	PD6		// Left -> Port D
#define JOYSTICK_RIGHT	PB7		// Right -> Port B
#define JOYSTICK_SW		PD7		// Right -> Port D
//---------------------------------------------------------------------------------
// LED
//---------------------------------------------------------------------------------
#define GLED	PB1	// Green LED
#define RLED	PD4	// Red LED
#define YLED	PC5	// Yellow LED

#define TXLED	YLED	     // Transmit LED
#define RXLED	GLED	     // Receieve LED	

// SW UART Flow Control
#define UART0_FLOW_CTRL_CTS		PB1 // Clear to Send, HW UART handsaking
#define UART0_FLOW_CTRL_RTS		PB0 // Ready to Receive, HW UART handsaking
#define UART0_FORCE_ON			PC4  // Turn on and off UART driver

// CC2400 SW RF Strobe Control
#define RF_STROBE_RX			PC3 // RX Flow control signal for CC2400
#define RF_STROBE_TX			PC2 // TX Flow control signal for CC2400

// CC2400 Handsake signals and general purpose I/O
#define FIFO  					PD2 // FIFO signal for CC2400
#define PKT						PD3 // PKT packet signal from CC2400 
#define GIO1					PC1 // GIO1 pin from CC2400
#define GIO6					PC0 // GIO6 pin from CC2400 

// When using 0 MHz offset TX must be 1 MHz higher than RX
#define TX_FREQ		0x0981
#define RX_FREQ		0x0980

// Set RF data rate at compile time
#define RANGE_1MBPS		1
#define RANGE_250KBPS	0
#define RANGE_10KBPS	0
// Enable PKT signal interrupt at compile time

// Ooptions for CC2400DBK
#define CAL_INIT		0 
#define PKT_INT			1	
#define UART_INIT		2
#define ADC_INIT		4
			
#endif

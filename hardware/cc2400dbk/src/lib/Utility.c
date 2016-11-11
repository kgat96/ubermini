/*****************************************************************************
 *                                                                           *
 *        **********                                                         *
 *       ************                                                        *
 *      ***        ***                                                       *
 *      ***   +++   ***                                                      *
 *      ***   + +   ***                                                      *
 *      ***   +                            CHIPCON CC2400                    *
 *      ***   + +   ***                  UTILITY FUNCTION LIB FILE           *
 *      ***   +++   ***                                                      *
 *      ***       ***                                                        *
 *       ***********                                                         *
 *        *********                                                          *
 *                                                                           *
 *****************************************************************************
 * A number of helpful utility function which can be used to increase the	 * 
 * legibility of your code.  						 						 *
 * 				                        				 					 *                                                                           *
 *****************************************************************************
 * Device:      CC2400                                                       *
 * File:		Utility.c					                                 *
 * Author:		MBR						                                     *
 * Target:		ATmega8						                                 *
 * Created:		2004-01-05					                                 *
 *				    			                                             *
 *****************************************************************************
 * Revision history:                                                         *
 *                                                                           *
 * $Log: Utility.c,v $
 * Revision 1.2  2004/01/20 15:15:48  mbr
 * Cosmetic Changes
 * Removed Function char hexdigit2init(char val)
 *
 * Revision 1.1  2004/01/07 13:09:43  tos
 * Initial version in CVS.
 *                                                       *
 *                                                                           *  
 *									                                         *	
 *                                                                           *
 *****************************************************************************/ 
// Includes 
#include "include.h" // Master Include File
//----------------------------------------------------------------------------
//  BYTE uart_putchar(...)
//
//  Description:
//      Simple implementation of the ANSI putchar() routine that
//		writes to UART.
//
//  Arguments:
//      BYTE ch
//			The data to be written UART 
//  Return value:
//		The data send to the UART
//----------------------------------------------------------------------------
BYTE uart_putchar (BYTE ch) {
	/* Wait for empty transmit buffer */
	UART0_WAIT();      
	/* Put data into buffer, sends the data */
	UART0_TX(ch);
	return ch;
} // uart_putchar
//----------------------------------------------------------------------------
//  BYTE uart_getchar(...)
//
//  Description:
//      Simple implementation of the ANSI getchar() routine that
//		reads from UART.
//
//  Arguments:
//      None			
//  Return value:
//		The data read from the UART
//----------------------------------------------------------------------------
BYTE uart_getchar(void) {
   /* Wait for data to be received */
   while ( !(UCSRA & (1<<RXC)) )
      ;
   /* Get and return received data from buffer */
   return UDR;
} // uart_getchar
//----------------------------------------------------------------------------
//  void writestr(...)
//
//  Description:
//      This routine outputs a string 
//
//  Arguments:
//      The string to be written to the UART			
//  Return value:
//		void
//----------------------------------------------------------------------------
void writestr(char *str) {
	
  while (*str != 0) {
    uart_putchar(*str++);
  }
} // writestr
//----------------------------------------------------------------------------
//  void writeln(...)
//
//  Description:
//      This routine outputs a string, and also outputs trailing new-line 
//		and carrage return characters
//
//  Arguments:
//      The string to be written to the UART			
//  Return value:
//		void
//----------------------------------------------------------------------------
void writeln(char *str) {
  
  while (*str != 0) {
    uart_putchar(*str++);
  }
  uart_putchar('\n');
  uart_putchar('\r');
} // writeln
//----------------------------------------------------------------------------
//  void newline(...)
//
//  Description: Used only to output trailing new-line and  
//				 carrage return characters
//						
//  Arguments:
//		None
//  Return value:
//      None
//----------------------------------------------------------------------------	
void newline(void) {
   uart_putchar('\n');
   uart_putchar('\r');
} // newline
//----------------------------------------------------------------------------
//  void writestr_p(...)
//
//  Description:
//      This routine outputs a string in Flash 
//
//  Arguments:
//      The string to be written from Flash			
//  Return value:
//		void
//----------------------------------------------------------------------------
void writestr_p(char *str) {
   
   char c;   
   while ((c = pgm_read_byte_near((WORD)str++)))
      uart_putchar(c);
} // writestr_p
//----------------------------------------------------------------------------
//  void writeln_p(...)
//
//  Description:
//      This routine outputs a string from Flash, and also outputs trailing 
//		new-line and carrage return characters.  
//  Arguments:
//      The string to be written from Flash			
//  Return value:
//		void
//----------------------------------------------------------------------------
void writeln_p(const char *str) {
   
   char c;

   while ((c = pgm_read_byte_near((WORD)str++)))
      uart_putchar(c);
      uart_putchar('\n');
      uart_putchar('\r');
} // writeln_p
//----------------------------------------------------------------------------
//  void writelong(...)
//
//  Description:
//      This routine outputs a long value in decimal.   
//		  
//  Arguments:
//		long longval
//      	The long value to be written to the UART			
//  Return value:
//		void
//----------------------------------------------------------------------------
void writelong(long longval) {
  
  char i;
  long divider;

  divider=1000000000;

  for (i=10;i>0;i--) {
    uart_putchar((longval/divider)+0x30);
    divider/=10;
  }
} // writelong
//----------------------------------------------------------------------------
//  void writehex(...)
//
//  Description:
//      This routine outputs an int (in hexadecimal).   
//		  
//  Arguments:
//		int hexval
//      	The int value to be written to the UART			
//  Return value:
//		void
//----------------------------------------------------------------------------
void writehex(int hexval) {
  char i;

  int temp;

  char val;

  uart_putchar('0');
  uart_putchar('x');

  temp=hexval;

  for(i=0;i<4;i++) {
    val=(temp&0xF000)>>12;
    if (val<=9) {
      uart_putchar(val+0x30);
    } else {
      uart_putchar(val+0x41-0xA);
    }
    temp=(temp<<4);
  }
} // writehex
//----------------------------------------------------------------------------
//  void writeint(...)
//
//  Description:
//      This routine outputs an integer value in decimal, without 
//		preceeding zero's.   
//  Arguments:
//		int num
//      	The int value to be written to the UART			
//  Return value:
//		void
//----------------------------------------------------------------------------
void writeint(int num){
	char i;
	char digit_start = 0;
	int digit = 0;
	int denom = 10000;

	if (num==0) {
		uart_putchar('0');
	} else {
		for(i = 5; i > 0; i--) {
			digit = num/denom;
      			if((digit_start == 1) || (digit != 0)) {
         			digit_start = 1;
         			num %= denom;
         			uart_putchar(digit+'0');
      			}
      			denom/=10;
   		}
	}
} // writeint

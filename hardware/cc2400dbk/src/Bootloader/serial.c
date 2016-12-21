/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***                                                                                *
 *      ***   +                                   CHIPCON CC2400DBK BOOTLOADER                         *
 *      ***   + +   ***                          Demonstration board bootloader                        *
 *      ***   +++   ***                                                                                *
 *      ***        ***                                                                                 *
 *       ************                                                                                  *
 *        **********                                                                                   *
 *                                                                                                     *
 *******************************************************************************************************
 * This program allows an AVR with bootloader capabilities to read and write to its own Flash/EEPROM.  * 
 * It contains the AVR C and EC++ startup routine and must be tailored to suit customers's hardware.   * 
 *                                                                                                     *
 * Please that this code is based on Atmel Corporation source code from their application note AVR109, *
 * Self Programming                                                                                    *
 *                                                                                                     *
 *******************************************************************************************************
 * Compiler: IAR EW                                                                                  *
 * Target platform: CC2420DB (can easily be ported to other platforms)                                 *
 *******************************************************************************************************
 * Revision history:                                                                                   *
 * $Log: serial.c,v $
 * Revision 1.2  2004/07/20 12:22:20  mbr
 * Bootloader according application note AVR109,
 * Self Programming with only C - code
 *
 * Revision 1.2  2004/03/30 13:31:02  mbr
 * Release for web
 *                                                                                  *
 *                                                                                                     *
 *******************************************************************************************************/
#include "defines.h"


//-------------------------------------------------------------------------------------------------------
//  void avrCalibrate(void)
//
//  DESCRIPTION:
//      This function reads the calibration value from EEPROM and writes the  value to OSCCAL register.  
//
//  ARGUMENTS:
//	None
//	      	
//  RETURN VALUE:
//      None
//			
//-------------------------------------------------------------------------------------------------------
void avrCalibrate(void) {
	// Read Calibration value from EEPROM and write to 	
	EEPROM_READ(0x01FF,OSCCAL);					
} // avrCalibrate



//-------------------------------------------------------------------------------------------------------
//  void initBootUart(void)
//
//  DESCRIPTION:
//      This function initializes the UART for use with the bootloader. For details regarding this program,  
//      please study the AVR109 application note, Self-programming
//
//  ARGUMENTS:
//	None
//	      	
//  RETURN VALUE:
//      None
//			
//-------------------------------------------------------------------------------------------------------
void initBootUart(void){
  // Write calibration value 
   avrCalibrate();  
  // Turn on UART transceiver
  UART_FORCE_ON();                         
  // Enable pull-up on UARTCTRL line on UARTPORT
  UARTPORT |= UARTCTRL();                   
  // 19200bps @ 8 MHz
  BAUD_RATE_LOW_REG = 0x19;                 
  // Enable receive and transmit 
  UART_CONTROL_REG = ((1 << ENABLE_RECEIVER_BIT) | (1 << ENABLE_TRANSMITTER_BIT)); 

} // initBootUart




//-------------------------------------------------------------------------------------------------------
//  void sendChar(char c)
//
//  DESCRIPTION:
//      This function transmit a byte on the UART. For details regarding this program,  
//      please study the AVR109 application note, Self-programming
//
//  ARGUMENTS:
//		char c
//          The byte to be transmitted by the UART
//	      	
//  RETURN VALUE:
//      None
//			
//-------------------------------------------------------------------------------------------------------
void sendChar(char c) {
  // Prepare transmission
  UART_DATA_REG = c;
  // Wait until byte sent                                   
  while (!(UART_STATUS_REG & (1 << TRANSMIT_COMPLETE_BIT)));
  // Delete TXC flag
  UART_STATUS_REG |= (1 << TRANSMIT_COMPLETE_BIT);          
} // sendChar




//-------------------------------------------------------------------------------------------------------
//  void recChar(char c)
//
//  DESCRIPTION:
//      This function receives a byte on the UART. For details regarding this program,  
//      please study the AVR109 application note, Self-programming
//
//  ARGUMENTS:
//		char c
//          The byte to be transmitted by the UART
//	      	
//  RETURN VALUE:
//      None
//			
//-------------------------------------------------------------------------------------------------------
char recChar(void) {
  while(!(UART_STATUS_REG & (1 << RECEIVE_COMPLETE_BIT)));  // wait for data
  return UART_DATA_REG;
} // recChar

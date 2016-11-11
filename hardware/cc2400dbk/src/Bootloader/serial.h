/*******************************************************************************************************
*                                                                                                     *
*        **********                                                                                   *
*       ************                                                                                  *
*      ***        ***                                                                                 *
*      ***   +++   ***                                                                                *
*      ***   + +   ***                                                                                *
*      ***   +                                   CHIPCON CC2400DBK BOOTLOADER                         *
*      ***   + +   ***                           Bootloader serial include file                       *
*      ***   +++   ***                                                                                *
*      ***        ***                                                                                 *
*       ************                                                                                  *
*        **********                                                                                   *
*                                                                                                     *
*******************************************************************************************************
* This program allows an AVR with bootloader capabilities to read and write to its own Flash/EEPROM.  * 
* It contains the AVR C and EC++ startup routine and must be tailored to suit customers's hardware.   *
*                                                                                                     *
*                                                                                                     *
* Please that this code is based on Atmel Corporation source code from their application note AVR109, *
* Self Programming                                                                                    *
*                                                                                                     *
*******************************************************************************************************
* Compiler: AVR-GCC                                                                                   *
* Target platform: CC2400DB (can easily be ported to other platforms)                                 *
*******************************************************************************************************
* Revision history:                                                                                   *
* $Log: serial.h,v $
* Revision 1.2  2004/07/20 12:22:20  mbr
* Bootloader according application note AVR109,
* Self Programming with only C - code
*
* Revision 1.2  2004/03/30 13:31:02  mbr
* Release for web
*                                                                                  *
* Revision 1.1.1.1  2004/02/10 10:59:01  jol
* Created archive
*                                                                                                     *
*******************************************************************************************************/

//Prototypes

//-------------------------------------------------------------------------------------------------------
//  void initBootUart(void)
//
//  DESCRIPTION:
//      This function initializes the UART for use with the bootloader. For details regarding this program,  
//      please study the AVR109 application note, Self-programming
//
//  ARGUMENTS:
//		None
//	      	
//  RETURN VALUE:
//      None
//			
//-------------------------------------------------------------------------------------------------------
void initBootUart(void);




//-------------------------------------------------------------------------------------------------------
//  void sendChar(char c)
//
//  DESCRIPTION:
//      This function transmit a byte on the UART. For details regarding this program,  
//      please study the AVR109 application note, Self-programming
//
//  ARGUMENTS:
//	char c
//      The byte to be transmitted by the UART
//	      	
//  RETURN VALUE:
//      None
//			
//-------------------------------------------------------------------------------------------------------
void sendChar(char);




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
char recChar(void );

/*****************************************************************************
 *                                                                           *
 *        **********                                                         *
 *       ************                                                        *
 *      ***        ***                                                       *
 *      ***   +++   ***                                                      *
 *      ***   + +   ***                                                      *
 *      ***   +                            CHIPCON CC2400                    *
 *      ***   + +   ***                  UTILITY INCLUDE FILE                *
 *      ***   +++   ***                                                      *
 *      ***       ***                                                        *
 *       ***********                                                         *
 *        *********                                                          *
 *                                                                           *
 *****************************************************************************
 * 						 													 *
 * Utility include file for CC2400DB                        				 *                                                                           *
 *****************************************************************************
 * Device:      CC2400                                                       *
 * File:		Utility.h					                                 *
 * Author:		MBR						                                     *
 * Target:		ATmega8						                                 *
 * Created:		2004-01-05					                                 *
 *				    			                                             *
 *****************************************************************************
 * Revision history:                                                         *
 *                                                                           *
 * $Log: Utility.h,v $
 * Revision 1.3  2004/01/28 09:22:56  mbr
 * Added newline() prototype
 *
 * Revision 1.2  2004/01/20 14:49:52  mbr
 * Cosmetic changes
 *
 * Revision 1.1  2004/01/07 13:06:40  tos
 * Initial version in CVS.
 *                                                       					 *
 *                                                                           *  
 *									                                         *	
 *                                                                           *
 ****************************************************************************/

#ifndef UTILITY_H
#define UTILITY_H

//Prototypes
BYTE uart_putchar (BYTE ch);
BYTE uart_getchar(void);
void writestr(char *str);
void writeln(char *str);
void writestr_p(char *str);
void writeln_p(const char *str);
void writeint(int intval);
void newline(void);

#endif

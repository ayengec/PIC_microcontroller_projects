/*
 * File:   main.c
 * Author: ayengec
 */

#include <xc.h>
#include "LCD_write.h"
#include "stdio.h"

#define _XTAL_FREQ  4000000
/*
To set up an Asynchronous Reception with Address
Detect Enable:
	1. Initialize the SPBRGH:SPBRG registers for the
	appropriate baud rate. Set or clear the BRGH
	and BRG16 bits, as required, to achieve the
	desired baud rate.
	2. Enable the asynchronous serial port by clearing
	the SYNC bit and setting the SPEN bit.
	3. If interrupts are required, set the RCEN bit and
	select the desired priority level with the RCIP bit.
	4. Set the RX9 bit to enable 9-bit reception.
	5. Set the ADDEN bit to enable address detect.
	6. Enable reception by setting the CREN bit.
	7. The RCIF bit will be set when reception is
	complete. The interrupt will be Acknowledged if
	the RCIE and GIE bits are set.
	8. Read the RCSTA register to determine if any
	error occurred during reception, as well as read
	bit 9 of data (if applicable).
	9. Read RCREG to determine if the device is being
	addressed.
	10. If any error occurred, clear the CREN bit.
	11. If the device has been addressed, clear the
	ADDEN bit to allow all received data
*/
void send_char(unsigned char m_data)
{
	TXREG=m_data;		// UART Transmit Register
	while(TXIF!=1);		// EUSART Transmit Interrupt Flag bit
	/*
		1 = The EUSART transmit buffer, TXREG, is empty (cleared when TXREG is written)
		0 = The EUSART transmit buffer is full
	*/
	TXIF=0;			// clear the transmit interrupt flag
}

unsigned char receive_Data(void)
{
	while (1)
	{
		while(RCIF!=1);		// while receive interrupt flag is clear
		/*
			RCIF: EUSART Receive Interrupt Flag bit
				1 = The EUSART receive buffer, RCREG, is full (cleared when RCREG is read)
				0 = The EUSART receive buffer is empty
		*/
			RCIF=0;			// clear the receive interrupt flag
			if((RCSTA&0x06)!=0)	// if not (Overrun Error bit and Framing Error bit)
			{
				CREN=0;		// Continuous Receive Enable bit => disable receiver
				CREN=1;		// enable receiver
			}
		else break;
	}
	return RCREG; 		// UART Receive Register
}

void main()
{
	unsigned char    m_data;
	TRISC6=0;		// portC6 to be use as UART TX
	TRISC7=1;		// portC7 to be use as UART RX
	SPBRG=25;		// Baud Rate Generator Register : to set 9600
	TXSTA=0x24;		// TRANSMIT STATUS AND CONTROL REGISTER = > BRGH: High Baud Rate Select bit = High Speed	and	TXEN:Transmit Enable bit  	
	RCSTA=0x90;		// RECEIVE STATUS AND CONTROL REGISTER => Ninth bit of Received Data + Address Detect Enable bit
	
	for(;;)
	{
		initialize_LCD();
		m_data=receive_Data();
		send_char(m_data);
		write_char(m_data);
        __delay_ms(3000);
	}
}
	                                                                                    

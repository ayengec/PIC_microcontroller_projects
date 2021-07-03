/*
 * File:   uart_control.c
 * Author: ayengec
 *
 */

#include <xc.h>

void send_char(char m_data)
{
	TXREG=m_data;		// UART Transmit Register
	while(TXIF!=1);		// EUSART Transmit Interrupt Flag bit
	/*
		1 = The EUSART transmit buffer, TXREG, is empty (cleared when TXREG is written)
		0 = The EUSART transmit buffer is full
	*/
	TXIF=0;			// clear the transmit interrupt flag
}

 void send_string(char *s)
 {
     while(*s != 0)
     {
         send_char(*s++);
         //s++;
     }
 }

unsigned char receive_Data(void)
{
	while (1)
	{
		while(!RCIF)
        {
        }		// while receive interrupt flag is clear
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

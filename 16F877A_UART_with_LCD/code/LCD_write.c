/*
 * File:   LCD_write.c
 * Author: ayengec
 */


#include <xc.h>
#include "stdio.h"
#define _XTAL_FREQ  4000000

void write_char(unsigned char num)
{
	PORTB = num;
	PORTD=5;
	__delay_us(2000);
	PORTD=4;
}
void initialize_LCD()
{
	TRISB=0;
	TRISD=0;
	PORTB=1;
	PORTD=1;
	__delay_us(2000);
	PORTD=0;
	PORTB=15;
	PORTD=1;
	__delay_us(2000);
	PORTD=0;
	PORTB=48;   
	PORTD=1;
	__delay_us(2000);
	PORTD=0;
	PORTD=4;
}
void write_string(char *text)
{
	int i=0;
	while(text[i]!='\n')
    {
        write_char((text[i]));
        i++;
    }
}

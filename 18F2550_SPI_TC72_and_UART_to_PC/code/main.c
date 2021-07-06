/*
 * File:   main.c
 * Author: ayengec
 *
 */


// PIC18F2550 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = XT_XT     // Oscillator Selection bits (XT oscillator (XT))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = ON        // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 0         // Brown-out Reset Voltage bits (Maximum setting 4.59V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 1        // Watchdog Timer Postscale Select bits (1:1)

// CONFIG3H
#pragma config CCP2MX = OFF     // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (RE3 input pin enabled; MCLR pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = ON         // Code Protection bit (Block 0 (000800-001FFFh) is code-protected)
#pragma config CP1 = ON         // Code Protection bit (Block 1 (002000-003FFFh) is code-protected)
#pragma config CP2 = ON         // Code Protection bit (Block 2 (004000-005FFFh) is code-protected)
#pragma config CP3 = ON         // Code Protection bit (Block 3 (006000-007FFFh) is code-protected)

// CONFIG5H
#pragma config CPB = ON         // Boot Block Code Protection bit (Boot block (000000-0007FFh) is code-protected)
#pragma config CPD = ON         // Data EEPROM Code Protection bit (Data EEPROM is code-protected)

// CONFIG6L
#pragma config WRT0 = ON        // Write Protection bit (Block 0 (000800-001FFFh) is write-protected)
#pragma config WRT1 = ON        // Write Protection bit (Block 1 (002000-003FFFh) is write-protected)
#pragma config WRT2 = ON        // Write Protection bit (Block 2 (004000-005FFFh) is write-protected)
#pragma config WRT3 = ON        // Write Protection bit (Block 3 (006000-007FFFh) is write-protected)

// CONFIG6H
#pragma config WRTC = ON        // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are write-protected)
#pragma config WRTB = ON        // Boot Block Write Protection bit (Boot block (000000-0007FFh) is write-protected)
#pragma config WRTD = ON        // Data EEPROM Write Protection bit (Data EEPROM is write-protected)

// CONFIG7L
#pragma config EBTR0 = ON       // Table Read Protection bit (Block 0 (000800-001FFFh) is protected from table reads executed in other blocks)
#pragma config EBTR1 = ON       // Table Read Protection bit (Block 1 (002000-003FFFh) is protected from table reads executed in other blocks)
#pragma config EBTR2 = ON       // Table Read Protection bit (Block 2 (004000-005FFFh) is protected from table reads executed in other blocks)
#pragma config EBTR3 = ON       // Table Read Protection bit (Block 3 (006000-007FFFh) is protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = ON       // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>

#include "uart_control.h"

#define _XTAL_FREQ  20000000

#define SCLK  PORTBbits.RB1
#define MISO  PORTBbits.RB0
#define CS    PORTCbits.RC0
#define MOSI  PORTCbits.RC7


void spi_init(void)
{
   
    TRISCbits.TRISC0 = 0;   // CS pin direction
    TRISBbits.TRISB1 = 0;   // SCLK pin direction
    TRISBbits.TRISB0 = 1;   // Master input slave output pin is set as input
    TRISCbits.TRISC7 = 0;   // Master output is set as output
    
    CS = 0;               // CS set to high    
    
    ADCON1bits.PCFG = 0xF;
    
    SSPSTATbits.CKE = 0;
    SSPCON1bits.CKP = 1;
    
    SSPCON1bits.SSPM  = 0b0001;  // Synchronous Serial Port Mode Select bits: SPI Master mode, clock = FOSC/16 
    
    SSPCON1bits.SSPEN = 1;  //  SSPEN: Synchronous Serial Port Enable bit
                                    //1 = Enables serial port and configures SCK, SDO, SDI and SS as serial port pins
                                    //0 = Disables serial port and configures these pins as I/O port pins
    PIR1bits.SSPIF = 0;
    
}

void spi_write_char(char w_data)
{
    SSPBUF = w_data;
    while(!PIR1bits.SSPIF);
    PIR1bits.SSPIF = 0;
    char dummy_read = SSPBUF;  // to clear buffer
}

char spi_read()
{
    SSPBUF = 0;
    
    while(!SSPSTATbits.BF);
    while(!PIR1bits.SSPIF);    
    PIR1bits.SSPIF = 0;
    
    return SSPBUF;
}

void main(void) {
    
    char msb_temp, lsb_temp;
    char buf[80];
            
    float result;
    
    TRISCbits.RC6 = 0;  // uart tx pin set as output
    
    // uart settings
    SPBRG = 31;
    TXSTA = 0x20;
    RCSTA = 0x90;
    
    send_string("******* ayengec SPI PIC MCUs *************\r\n");
    
    spi_init();

    while (1) {
        
    __delay_ms(500);
    
    
    CS =1;
    spi_write_char(0x80);           // Control Register
    spi_write_char(0x10);           // continous temperature conversion setting
    CS = 0;
    
    __delay_ms(150);
    
    CS =1;
    spi_write_char(0x02);           // temp read address
    msb_temp = spi_read();          // first 8 bit of data
    lsb_temp = spi_read();          // least 8 bit of data
    CS = 0;
    
    if(msb_temp > 127)
    {
            result = -(float)(256-(msb_temp - lsb_temp/16));       // if minus degree
    }
    else
    {
            result = (float)(msb_temp + lsb_temp/16);              // for LSB/16 => if 0x80 = 0.5; else if 0x40 = 0.25; else 0x00
    }


    sprintf(buf, "TEMP SPI SENSOR: %.2f C\r\n", result);
    send_string(buf);
    __delay_ms(500);
    
    }

}

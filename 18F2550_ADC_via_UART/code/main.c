/*
 * File:   main.c
 * Author: ayengec
 */
/*The following steps should be followed to perform an A/D conversion:
    1. Configure the A/D module:
    ? Configure analog pins, voltage reference and digital I/O (ADCON1)
    ? Select A/D input channel (ADCON0)
    ? Select A/D acquisition time (ADCON2)
    ? Select A/D conversion clock (ADCON2)
    ? Turn on A/D module (ADCON0)
    2. Configure A/D interrupt (if desired):
    ? Clear ADIF bit
    ? Set ADIE bit
    ? Set GIE bit
    3. Wait the required acquisition time (if required).
    4. Start conversion:
    ? Set GO/DONE bit (ADCON0 register)
    5. Wait for A/D conversion to complete, by either:
    ? Polling for the GO/DONE bit to be cleared
    OR
    ? Waiting for the A/D interrupt
    6. Read A/D Result registers (ADRESH:ADRESL);
    clear bit ADIF, if required.
    7. For next conversion, go to step 1 or step 2, as
    required. The A/D conversion time per bit is
    defined as TAD. A minimum wait of 3 TAD is
    required before the next acquisition starts.
 * 
 */

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = HS     // Oscillator Selection bits (XT oscillator (XT))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdio.h>
#include "string.h"
#include "uart_control.h"
#define _XTAL_FREQ 20000000

void init_adc_port(void)
{
    
    // 1. Configure the A/D module
    ADCON1bits.PCFG = 0xD;   // 1110 : Only AN0 is selected as ANALOG MODE, pthers DIGITAL
    ADCON0bits.CHS  = 0x0;   // Channel 0 (AN0) is selected
    ADCON2bits.ACQT = 0b110; // A/D Conversion Clock Select bits : 16 TAD
    ADCON2bits.ADCS = 0b010; // A/D Conversion Clock Select bits : FOSC/32
    ADCON0bits.ADON = 1;     // A/D is enabled
}
unsigned int ADC_Read(unsigned int channel_no)
{
    if(channel_no>13)
        return 0;
    ADCON1bits.PCFG = 0xD-channel_no;
    ADCON0bits.CHS  = channel_no;
    ADCON2bits.ACQT = 0b110; // A/D Conversion Clock Select bits : 16 TAD
    ADCON2bits.ADCS = 0b010; // A/D Conversion Clock Select bits : FOSC/32
   
    PIR1bits.ADIF = 0;  // A/D Converter Interrupt Flag bit 
                        // 1 = An A/D conversion completed (must be cleared in software)
    PIE1bits.ADIE  = 1; // A/D Converter Interrupt Enable bit
    INTCONbits.GIE = 1; // Global Interrupt enable
    
    __delay_us(25);               // 3. Wait the required acquisition time (if required)
    ADCON0bits.GO_nDONE = 1;        // 4. Start conversion
    while(ADCON0bits.GO_nDONE);     // 5. wait until conversion is completed
    while(!PIR1bits.ADIF);           // 5. Waiting for the A/D interrupt
    PIR1bits.ADIF = 0;              // Clearing interrupt
    return ((ADRESH*4) + (ADRESL/64));   // 
}

void main(void) {
    unsigned int adc_data;
    char buf[80];
    float result_data;
    TRISCbits.RC6 = 0;		// portC6 to be use as UART TX
	TRISCbits.RC7 = 1;		// portC7 to be use as UART RX
    TRISAbits.RA0 = 1;      // AO pin set as input
    SPBRG=31;		// Baud Rate Generator Register : to set 9600
	TXSTA=0x20;		// TRANSMIT STATUS AND CONTROL REGISTER = > BRGH: High Baud Rate Select bit = High Speed	and	TXEN:Transmit Enable bit  	
	RCSTA=0x90;		// RECEIVE STATUS AND CONTROL REGISTER => Ninth bit of Received Data + Address Detect Enable bit
    send_string("********************** AYENGEC *********************\r\n");
    send_string("POT ANALOG READ AND SEND TO PC VIA UART\r\n");
    TRISAbits.RA0 = 1;
    init_adc_port();
    
    TRISAbits.RA0 = 1;
    
        while(1)
    {
        send_string("Reading From AN0 ");
        __delay_ms(100);
        
        adc_data = ADC_Read(0);
        result_data = 5*(float)(adc_data/1024.000);   // divide to 10bit and map to 5V
            
        sprintf(buf, "ADC Value=%.3f\r\n", result_data);
        send_string(buf);
        __delay_ms(1000);
    }
    
    
}

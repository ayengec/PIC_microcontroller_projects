
// ayengec 
// PIC I2C DS1621 Temperature Sensor Read Data and Send via UART to PC

// main.c

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
#include <string.h>
#include <stdio.h>
#include "i2c_control.h"
#include "uart_control.h"
#define _XTAL_FREQ 20000000

double get_sensor_val()
{
    unsigned char r_data_msb;
    unsigned char r_data_lsb;
    
    initialize_i2c();
    __delay_ms(1000);
    start_i2c();
    __delay_us(8);
    
    i2c_write_byte(0x90);
    i2c_write_byte(0x22);
    i2c_stop();
    start_i2c();
    i2c_write_byte(0x90);
    i2c_write_byte(0xEE);
    i2c_stop();
    
    start_i2c();
    i2c_write_byte(0x90);
    i2c_write_byte(0xAA);
  
    i2c_repeated_start();
 
    i2c_write_byte(0x91);
    r_data_msb = i2c_read_byte();
    send_master_ack();
    r_data_lsb = i2c_read_byte();
    send_master_nack();
    i2c_stop();
    __delay_us(8);
    if(r_data_msb >127)
    {
        return r_data_msb-256+r_data_lsb*0.004;
    }
    else
    {
        return r_data_msb+r_data_lsb*0.004;
    }
}

void main(void) {
        
    double m_data;
    char   buf[80];
    TRISCbits.RC6=0;		// portC6 to be use as UART TX
    TRISCbits.RC7=1;		// portC7 to be use as UART RX
    SPBRG=31;		// Baud Rate Generator Register : to set 9600
    TXSTA=0x20;		// TRANSMIT STATUS AND CONTROL REGISTER = > BRGH: High Baud Rate Select bit = High Speed	and	TXEN:Transmit Enable bit  	
    RCSTA=0x90;		// RECEIVE STATUS AND CONTROL REGISTER => Ninth bit of Received Data + Address Detect Enable bit
    
    send_string("********************** AYENGEC *********************\r\n");
    send_string("READ TEMPERATURE FROM DS1621 AND SEND TO PC VIA UART\r\n");
    
    while(1)
    {
        
        m_data = get_sensor_val();
        sprintf(buf, "TEMP = %.1f C\r\n", m_data);
        send_string(buf);
        __delay_ms(1000);
    }
}

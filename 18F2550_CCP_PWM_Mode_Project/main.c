// ayengec: PIC18f2550 CCP PWM Mode Usage Project
// PIC18F2550 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = XT_XT     // Oscillator Selection bits (EC oscillator, CLKO function on RA6 (EC))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
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



#define _XTAL_FREQ 20000000

int main()// @0x100
{

    TRISC=0;
    PORTC=0;
    
    /* ------------DATASHEET-----------------------------------------
     * 15.4.4 SETUP FOR PWM OPERATION
        The following steps should be taken when configuring
        the CCP module for PWM operation:
        1. Set the PWM period by writing to the PR2
        register.
        2. Set the PWM duty cycle by writing to the
        CCPRxL register and CCPxCON<5:4> bits.
        3. Make the CCPx pin an output by clearing the
        appropriate TRIS bit.
        4. Set the TMR2 prescale value, then enable
        Timer2 by writing to T2CON.
        5. Configure the CCPx module for PWM operation.
     */

    // ECCP CONTROL REGISTER: Field=CCP1M3:CCP1M0 (Enhanced CCP Mode Select bits)
    // means to write 4'b1100 on CCP1M field  = PWM mode: P1A, P1C active-high; P1B, P1D active-high
    CCP1M3 = 1;             // 3.bit
    CCP1M2 = 1;             // 2.bit
    
    // T2CON register's field => T2CKPS1:T2CKPS0: Timer2 Clock Prescale Select bits. 01 equals to "Prescaler is 4"
    T2CONbits.T2CKPS0 = 1;
    T2CONbits.T2CKPS1 = 0;
    
    PR2 = 124;              // Timer2 Period Register
    // PWM Period = [(PR2) + 1] ? 4 ? TOSC ?(TMR2 Prescale Value)
    // The PWM duty cycle is copied from CCPR1L into CCPR1H
    // to 10 us period;
    // (PR2+1)*4*4*1/(20*10e6)ca = 100*10e-3
    // tduty = 100 us
    // PR2+1 = 125
    
    // PWM Duty Cycle = (CCPRXL:CCPXCON<5:4>) ? TOSC ? (TMR2 Prescale Value)
    // (CCPRXL&CCPXCON[5:4]) = 10bit data => 
    // to evaluate 40us duty high => %40 (40us/100us) duty cycle
    // 10bit data must be equal => decimal 200 => 10'b0011001000
    // CCPRXL = These bits are the two LSbs (bit 1 and bit 0) of the 10-bit PWM duty cycle. The eight MSbs(DCx9:DCx2) of the duty cycle are found in CCPRxL
    CCP1CONbits.DC1B1 = 0;  // CCPXCON5
    CCP1CONbits.DC1B0 = 0;  // CCPXCON4
    // first 8 bits of 10 bit data => 00110010 = 0x32
    CCPR1L = 0x32; // decimal 50

    
    // T2CON register TMR2ON bit is set.
    T2CONbits.TMR2ON = 1;
    
    while(1)
    {
        PORTCbits.RC0 = 1;
        PORTCbits.RC1 = 0;
        __delay_ms(500);
        
        PORTCbits.RC0 = 0;
        PORTCbits.RC1 = 1;
        __delay_ms(500);
    }
    
    return 0;
}

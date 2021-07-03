/*
 * File:   i2c_control.c
 * Author: ayengec
 *
 */


#include <xc.h>
#include "stdio.h"
#include "stdbool.h"

/*
 * Once Master mode is enabled, the user has six options:
    1. Assert a Start condition on SDA and SCL.
    2. Assert a Repeated Start condition on SDA and
    SCL.
    3. Write to the SSPBUF register initiating
    transmission of data/address.
    4. Configure the I2C port to receive data.
    5. Generate an Acknowledge condition at the end
    of a received byte of data.
    6. Generate a Stop condition on SDA and SCL.
 */
#define I2C_FREQ    200000          // 100 kbps
#define _XTAL_FREQ  20000000

#define SCL         PORTBbits.RB1   // SCL pin of PIC 18F2550
#define SDA         PORTBbits.RB0   // SDA pin of PIC 18F2550

#define SCL_DIR     TRISBbits.RB1   // SCL direction 1=in, 0=out
#define SDA_DIR     TRISBbits.RB0   // SDA direction




void initialize_i2c(void)
{
    SDA_DIR = 1;    // 1: OUT DIG I2C? data output (MSSP module); takes priority over port data. I2C data input (MSSP module); input type depends on module setting.
                    // So, SDA is bidirectional when set to 1
    SCL_DIR = 1;    // 0: I2C clock output (MSSP module); takes priority over port data.

    SSPSTAT = 0x80;
    SSPCON1 = 0x28;
    SSPCON2 = 0x60;
 
    SSPADD = ((_XTAL_FREQ/4)/I2C_FREQ )-1; // To find SSPADD value of previous line, calculate it with dedicated i2c clock as 200000; 
                                           //SSP Baud Rate Reload Register in I2C Master Mode. Otherwise, SSP Address Register in I2C Slave Mode.
    
    /*SSPSTATbits.SMP   = 1; //MSSP STATUS REGISTER (I2C MODE); Slew Rate Control bit is disabled
        
    
    SSPCON1bits.SSPM  = 8; // 4'b1000 = I2C Master mode, clock = FOSC/(4 * (SSPADD + 1))
    SSPCON1bits.SSPEN = 1; // Synchronous Serial Port Enable bit*/
                


    
    PIE1bits.SSPIE = 1;
    SSPIF = 0;
}

void i2c_wait(void)
{
    while((SSPCON2 & 0x1F) | (SSPSTATbits.R_W) );
}

void start_i2c(void) 
{
    //i2c_wait();
    SSPCON2bits.SEN = 1 ; // Send the Start bit of i2c frame
    /*  SSPIF: Master Synchronous Serial Port Interrupt Flag bit
            1 = The transmission/reception is complete (must be cleared in software)
            0 = Waiting to transmit/receive
     */
    while(SSPCON2bits.SEN);
    PIR1bits.SSPIF = 0;         // it is cleared by software
    while(!SSPSTATbits.S);
}

void i2c_repeated_start(void)
{
    /*  RSEN: Repeated Start Condition Enabled bit (Master mode only)(1)
            1 = Initiate Repeated Start condition on SDA and SCL pins. Automatically cleared by hardware.
            0 = Repeated Start condition Idle
     */
    i2c_wait();
   // SSPCON2bits.PEN = 1;
    SSPCON2bits.RSEN = 1;       // Repeated Start initiated
    while(SSPCON2bits.RSEN); // wait until the transmission is complete
    PIR1bits.SSPIF = 0;         // it is cleared by software
}

void i2c_stop(void)
{
    /*  PEN: Stop Condition Enable bit (Master mode only)(1)
            1 = Initiate Stop condition on SDA and SCL pins. Automatically cleared by hardware.
            0 = Stop condition Idle
     */
   // while(!SSPIF); // wait until the transmission is complete
  //  PIR1bits.SSPIF = 0;         // it is cleared by software
    i2c_wait();
    
    SSPCON2bits.PEN = 1;
    
    while(SSPCON2bits.PEN);
    while(!SSPSTATbits.P);
}
char i2c_write_byte(char m_data)
{
    //i2c_wait();
    SSPBUF = m_data;            // Write requested data to buffer: Synchronous Serial Port Receive Buffer/Transmit Register
    while(!SSPIF); // wait until the transmission is complete
    PIR1bits.SSPIF = 0;         // it is cleared by software
    //while(SSPSTATbits.BF);
    //i2c_wait();
    /*  ACKSTAT: Acknowledge Status bit (Master Transmit mode only)
            1 = Acknowledge was not received from slave
            0 = Acknowledge was received from slave
     */
    return (SSPCON2bits.ACKSTAT);  // It returns Ack/nAck from slave
}
void send_master_ack(void)
{
    /*  ACKEN: Acknowledge Sequence Enable bit (Master Receive mode only)(1)
            1 = Initiate Acknowledge sequence on SDA and SCL pins and transmit ACKDT data bit.Automatically cleared by hardware.
            0 = Acknowledge sequence Idle
     */
    SSPCON2bits.ACKDT = 0;
    SSPCON2bits.ACKEN = 1;
    while(!PIR1bits.SSPIF); // wait until the transmission is complete
    PIR1bits.SSPIF = 0;         // it is cleared by software
    
}

void send_master_nack(void)
{
    /*  ACKEN: Acknowledge Sequence Enable bit (Master Receive mode only)(1)
            1 = Initiate Acknowledge sequence on SDA and SCL pins and transmit ACKDT data bit.Automatically cleared by hardware.
            0 = Acknowledge sequence Idle
     */
    SSPCON2bits.ACKDT = 1;
    SSPCON2bits.ACKEN = 1;
    while(SSPCON2bits.ACKEN); // wait until the transmission is complete
    
}
unsigned char i2c_read_byte(void)
{
    /*  RCEN: Receive Enable bit (Master mode only)(1)
        1 = Enables Receive mode for I2C
        0 = Receive Idle
     */
    i2c_wait();
    SSPCON2bits.RCEN = 1;
    
    while(!SSPSTATbits.BF);
    while(!PIR1bits.SSPIF); // wait until the receiving is complete
    PIR1bits.SSPIF = 0;         // it is cleared by software
    
    return SSPBUF;              // It returns the received byte: Synchronous Serial Port Receive Buffer/Transmit Register
}

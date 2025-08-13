#ifndef CAMIIT_PIC32_H
#define CAMIIT_PIC32_H

// PIC32MM0256GPM064 

#include <xc.h>
#include <stdint.h>

// (TMR) Timer 1 
#define FOSC 8000000
#define PRESCALER 8
#define INCPERSEC FOSC/2/PRESCALER

// (PWM) CCPx 
#define CCP1_ON CCP1CON1bits.ON 
#define CCP2_ON CCP2CON1bits.ON 
#define CCP3_ON CCP3CON1bits.ON 

typedef enum {CCP_1, CCP_2, CCP_3} CCPx ;

#define OCM1A CCP1CON2bits.OCAEN 
#define OCM1B CCP1CON2bits.OCBEN 
#define OCM1C CCP1CON2bits.OCCEN
#define OCM1D CCP1CON2bits.OCDEN
#define OCM1E CCP1CON2bits.OCEEN
#define OCM1F CCP1CON2bits.OCFEN

#define OCM2A CCP2CON2bits.OCAEN 
#define OCM2B CCP2CON2bits.OCBEN 
#define OCM2C CCP2CON2bits.OCCEN
#define OCM2D CCP2CON2bits.OCDEN
#define OCM2E CCP2CON2bits.OCEEN
#define OCM2F CCP2CON2bits.OCFEN

#define OCM3A CCP3CON2bits.OCAEN 
#define OCM3B CCP3CON2bits.OCBEN 
#define OCM3C CCP3CON2bits.OCCEN
#define OCM3D CCP3CON2bits.OCDEN
#define OCM3E CCP3CON2bits.OCEEN
#define OCM3F CCP3CON2bits.OCFEN

// (I2C) I2C3
#define I2CSPEED 100000
#define PBCLK    8000000
#define ADDREAD  0x0F
#define ADDWRITE 0x0E


// Timer Init's
void TMR1_init();
// void TMR2_init();
// void TMR3_init();

// Delay's  
void Delay_ms(int);
void Delay_us(int);
void Delay_s(int);

// Analog to Digital Converter Init
void ADC_init();

// Analog Read
uint16_t read_ADC();

// UART Init
void UART_init();

// UART Transmit (Tx)
void UART_send(char c);

// UART Receive (Rx)
char UART_receive();

// Compare / Capture Mode Init
void CCP_init(CCPx);

// Compare / Capture Mode Timer Init
void CCPTMR_init(CCPx);

// I2C3 Init
void I2C_init();

// I2C3 Commands
void I2C_wait();
void I2C_start();
void I2C_stop();
void I2C_restart();
void I2C_ack();
void I2C_nack();
void I2C_write(unsigned char data, char ack);
void I2C_read(unsigned char *value, char ack_nack);

// I2C3 Communication 
void sensor_send(unsigned char address, unsigned char value);
void sensor_receive(unsigned char address, unsigned char *value);



#endif
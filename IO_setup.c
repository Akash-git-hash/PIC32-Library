#include <xc.h>

void GPIO_init(){

    // All GPIO's set as Digital Pins 
    ANSELA = 0; //Set PORTA Pins as Digital
    ANSELB = 0; //Set PORTB Pins as Digital
    ANSELC = 0; //Set PORTC Pins as Digital
    ANSELD = 0; //Set PORTD Pins as Digital


    //**
    // 
    // GPIO Configuration: PIC32MM0256GPM064 
    //  
    // */

    // UART Module (UART)

    // UART1
    // TRISCbits.TRISC12 = 0;   // U1TX
    // TRISAbits.TRISA6 = 1;    // U1RX
     
    // PWM/Output Compare Module (CCPx)

    // CCP1 
    // TRISAbits.TRISA12 = 0;       // OCM1A //
    // TRISDbits.TRISD1 = 0;        // OCM1B //
    // TRISAbits.TRISA2 = 0;        // OCM1C 
    // TRISCbits.TRISC5 = 0;        // OCM1D //
    // TRISCbits.TRISC4 = 0;        // OCM1E 
    // TRISAbits.TRISA1 = 0;        // OCM1F  

    // CCP2
    // TRISCbits.TRISC2 = 0;        // OCM2A //
    // TRISCbits.TRISC3 = 0;        // OCM2B //
    // TRISBbits.TRISB0 = 0;        // OCM2C 
    // TRISBbits.TRISB1 = 0;        // OCM2D 
    // TRISBbits.TRISB2 = 0;        // OCM2E  
    // TRISBbits.TRISB3 = 0;        // OCM2F  

    // CCP3
    // TRISAbits.TRISA5 = 0;        // OCM3A //
    // TRISDbits.TRISD3 = 0;        // OCM3B //
    // TRISAbits.TRISA14 = 0;       // OCM3C //
    // TRISCbits.TRISC14 = 0;       // OCM3D //
    // TRISCbits.TRISC15 = 0;       // OCM3E //
    // TRISCbits.TRISC10 = 0;       // OCM3F //

    // I2C Clock and Data (I2Cx)    

    // I2C1                         
    // // TRISBbits.TRISB9 = 0;     // SDA1
    // // TRISBbits.TRISB8 = 0;     // SCL1 

    // I2C2                         
    // TRISBbits.TRISB2 = 0;        // SDA2 //
    // TRISBbits.TRISB3 = 0;        // SCL2 //
    
    // I2C3
    // TRISBbits.TRISB7 = 0;        // SCL3 //
    // TRISBbits.TRISB13 = 0;       // SCL3 //

    // ADC / Analog-to-Digital Pins (AN0 - 19)

    // ANSELAbits.ANSA0 = 1;        // Analog Select 
    // TRISAbits.TRISA0 = 1;        // AN0

    // ANSELAbits.ANSA1 = 1;        // Analog Select
    // TRISAbits.TRISA1 = 1;        // AN1

    // ANSELBbits.ANSB0 = 1;        // Analog Select
    // TRISBbits.TRISB0 = 1;        // AN2

    // ANSELBbits.ANSB1 = 1;        // Analog Select
    // TRISBbits.TRISB1 = 1;        // AN3

    // ANSELBbits.ANSB2 = 1;        // Analog Select
    // TRISBbits.TRISB2 = 1;        // AN4

    // ANSELAbits.ANSA2 = 1;        // Analog Select
    // TRISAbits.TRISA2 = 1;        // AN5

    // ANSELAbits.ANSA3 = 1;        // Analog Select
    // TRISAbits.TRISA3 = 1;        // AN6

    // ANSELBbits.ANSB4 = 1;        // Analog Select
    // TRISBbits.TRISB4 = 1;        // AN7

    // ANSELBbits.ANSB13 = 1;       // Analog Select
    // TRISBbits.TRISB13 = 1;       // AN8

    // ANSELBbits.ANSB14 = 1;       // Analog Select
    // TRISBbits.TRISB14 = 1;       // AN9

    // ANSELBbits.ANSB15 = 1;       // Analog Select
    // TRISBbits.TRISB15 = 1;       // AN10

    // ANSELBbits.ANSB3 = 1;        // Analog Select
    // TRISBbits.TRISB3 = 1;        // AN11

    // ANSELCbits.ANSC0 = 1;        // Analog Select
    // TRISCbits.TRISC0 = 1;        // AN12

    // ANSELCbits.ANSC1 = 1;        // Analog Select
    // TRISCbits.TRISC1 = 1;        // AN13

    // ANSELCbits.ANSC8 = 1;        // Analog Select
    // TRISCbits.TRISC8 = 1;        // AN14

    // ANSELCbits.ANSC5 = 1;        // Analog Select
    // TRISCbits.TRISC5 = 1;        // AN15

    // ANSELAbits.ANSA13 = 1;       // Analog Select
    // TRISAbits.TRISA13 = 1;       // AN16

    // ANSELAbits.ANSA12 = 1;       // Analog Select
    // TRISAbits.TRISA12 = 1;       // AN17

    // ANSELAbits.ANSA11 = 1;       // Analog Select
    // TRISAbits.TRISA11 = 1;       // AN18

    // ANSELAbits.ANSA6 = 1;        // Analog Select
    // TRISAbits.TRISA6 = 1;        // AN19    


};

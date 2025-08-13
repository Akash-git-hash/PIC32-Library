#include "PIC32.h"

// Timer 1 Init
void TMR1_init(){

    TMR1 = 0x00;    // Timer Reset 
    PR1 = 0xFFFF;   // Period
    T1CON = 0x8010; // 1us per tick
}

// Timer 2 Init
// void TMR2_init(){

//     TMR2 = 0x00;
//     PR2 = 0xFFFF;
//     T2CON = 0x8010; // 1us per tick
// }

// // Timer 3 Init
// void TMR3_init(){

//     TMR3 = 0x00;
//     PR3 = 0xFFFF;
//     T3CON = 0x8010; // 1us per tick
// }

// Delay Seconds
void Delay_s(int s){

    TMR1 = 0x00;
    while(TMR1 < 1000000){} 
}

// Delay Milli - Seconds
void Delay_ms(int ms){

    for(int i = 0; i < ms; i++)
    {
        TMR1 = 0x00;
        while(TMR1<1000){}
    }
}

// Delay Micro - Seconds
void Delay_us(int us){

    TMR1 = 0x00;
    while(TMR1 < us){}
}

// ADC1 Init
void ADC_init(){

     AD1CHS = 14u; // Positive Input (AN14) Select Bit (CHOSA<4:0>) 

    AD1CON2bits.VCFG = 0x0;
    AD1CON3bits.ADCS = 0xFF;
    AD1CON1bits.SSRC = 0x0;
    AD1CON3bits.SAMC = 0b10000;
    AD1CON1bits.FORM = 0b00;
    AD1CON2bits.SMPI = 0x0;
    AD1CON1bits.ON = 1;
}

// Analog Read Value
uint16_t read_ADC(){

    AD1CON1bits.SAMP = 1;
    Delay_ms(50);
    AD1CON1bits.SAMP = 1;
    Delay_ms(50);
    while(!AD1CON1bits.DONE);
    return ADC1BUF0;
}

// UART 1 Init
void UART_init(){

    U1BRG = 51;
    U1STA = 0;
    U1MODE = 0x8080;
    U1STA = 0x1400;
}

// UART Receive (Rx)
char UART_receive(){

    while(!U1STAbits.URXDA){}
    return U1RXREG;
}

// UART Transmit (Tx)
void UART_send(char c){

    while(U1STAbits.UTXBF){}
    U1TXREG = c;
}

// CCPx Init's
void CCP_init(CCPx CCPx){

    switch(CCPx){

        case CCP_1:
        
            // CCP1 Module Configuration
            CCP1CON1bits.CCSEL = 0;     // Output Compare
            CCP1CON1bits.MOD = 0b0101;  // Mode
            CCP1CON3bits.OUTM = 0b000;  // Output Mode
            CCP1CON3bits.POLACE = 0;    // Output Polarity Not Inverted
            
            // CCP1CON2bits.OCAEN = 1;
            // CCP1CON2bits.OCBEN = 1; 
            // CCP1CON2bits.OCCEN = 1;
            // CCP1CON2bits.OCDEN = 1;
            // CCP1CON2bits.OCEEN = 1;
            // CCP1CON2bits.OCFEN = 1;

            CCP1TMR = 0x00;         // Timer Register
            
            CCP1PR = 1023;        // Period Register
            CCP1RA = 0;             // Duty Cycle Rising Edge Trigger
            CCP1RB = 800;            // Duty Cycle Falling Edge Trigger

            break;

        case CCP_2:

            // CCP2 Module Configuration
            CCP2CON1bits.CCSEL = 0;     // Output Compare
            CCP2CON1bits.MOD = 0b0101;  // Mode - Variable Frequency 
            CCP2CON3bits.OUTM = 0b000;  // Output Mode
            CCP2CON3bits.POLACE = 0;    // Output Polarity Not Inverted
            
            // CCP2CON2bits.OCAEN = 1;
            // CCP2CON2bits.OCBEN = 1; 
            CCP2CON2bits.OCCEN = 1;
            // CCP2CON2bits.OCDEN = 1;
            // CCP2CON2bits.OCEEN = 1;
            // CCP2CON2bits.OCFEN = 1;

            CCP2TMR = 0x00;         // Timer Register
            
            CCP2PR = 1000;        // Period Register
            CCP2RA = 0x00;             // Duty Cycle Rising Edge Trigger
            CCP2RB = 100;            // Duty Cycle Falling Edge Trigger

            break;

        case CCP_3:

            // CCP3 Module Configuration
            CCP3CON1bits.CCSEL = 0;     // Output Compare                     
            CCP3CON1bits.MOD = 0b0101;  // Mode                    
            CCP3CON3bits.OUTM = 0b000;  // Output Mode                   
            
            // CCP3CON2bits.OCAEN = 1;
            // CCP3CON2bits.OCBEN = 1;
            // CCP3CON2bits.OCCEN = 1;
            // CCP3CON2bits.OCDEN = 1;
            // CCP3CON2bits.OCEEN = 1;
            // CCP3CON2bits.OCFEN = 1;
            
            CCP3TMR = 0x00;         // Timer Register                            
            
            CCP3PR = 0xFFFF;        // Period Register                            
            CCP3RA = 1;             // Duty Cycle Rising Edge Trigger                                
            CCP3RB = 10;            // Duty Cycle Falling Edge Trigger                                
    
            break;

    }
}

// CCPx Timebase Configuration // 

// CCPxTMR Init's
void CCPTMR_init(CCPx CCPxTMR){

    switch(CCPxTMR){

        case CCP_1:

            //CCP1 Timer Configuration 
            CCP1CON1bits.ON = 0;            // CCP2 Module Off
            CCP1CON1bits.T32 = 0;           // 16-bit Timer
            CCP1CON1bits.TMRPS = 0b01;      // Pre Scaler
            CCP1CON1bits.CLKSEL = 0b000;    // Clock Select
            CCP1CON1bits.TMRSYNC = 0;       // No Synchronization 
            CCP1CON1bits.SYNC = 0b00000;    //
            CCP1CON1bits.TRIGEN = 0;        // Trigger Disabled
            
            break;

        case CCP_2:

            // CCP2 Timer Configuration
            CCP2CON1bits.ON = 0;            // CCP2 Module Off
            CCP2CON1bits.T32 = 0;           // 16-bit Timer
            CCP2CON1bits.TMRPS = 0b10;      // Pre Scaler - 1:4
            CCP2CON1bits.CLKSEL = 0b000;    // Clock Select
            CCP2CON1bits.TMRSYNC = 0;       // No Synchronization
            CCP2CON1bits.SYNC = 0b00000;    // Ignores the Period Register
            CCP2CON1bits.TRIGEN = 0;        // Trigger Disabled
            CCP2CON1bits.OPS = 0b0000;      // Post Scaler - 1:1
            
            break;

        case CCP_3:
            
            // CCP3 Timer Configuration 
            CCP3CON1bits.ON = 0;
            CCP3CON1bits.T32 = 0;           // 16-bit Timer                        
            CCP3CON1bits.TMRPS = 0b00;      // Pre Scaler                    
            CCP3CON1bits.CLKSEL = 0b000;    // Clock Select                 
            CCP3CON1bits.TMRSYNC = 0;       // No Synchronization                      
            CCP3CON1bits.SYNC = 0b00000;    //                  
            CCP3CON1bits.TRIGEN = 0;        // Trigger Disabled  

            break;
        }   
}

// I2C Protocol Configuration //

// Initialize I2C
void I2C_init(void) {
    double BRG = 0.5 * PBCLK / I2CSPEED - 1 - 0.5 * PBCLK * 130e-9;
    I2C3CON = 0; // Clear I2C3 control register //
    I2C3CONbits.DISSLW = 0; // Enable Slew Rate Control // 
    I2C3BRG = (int)BRG; // Set the I2C3 Baud Rate //
    I2C3CONbits.ON = 1; // Enable I2C3 module // 
}

// Wait for I2C to become idle 
void I2C_wait(void) {
    while (I2C3CON & 0x1F) {} // Waits I2C Control Register to be clear // 
    while (I2C3STATbits.TRSTAT) {} // Waits for Transmit Status Flat to be clear //
}

// I2C Start Condition 
void I2C_start(void) {
    I2C_wait();
    I2C3CONbits.SEN = 1; // Initiate Start Condition 
    while (I2C3CONbits.SEN == 1) {} // Wait for Start Condition to complete 
}

// I2C Stop Condition
void I2C_stop(void) {
    I2C_wait();
    I2C3CONbits.PEN = 1; // I2C Stop Condition
    while (I2C3CONbits.PEN == 1) {} // Wait for Stop Condition to complete 
}

// I2C Restart Condition 
void I2C_restart(void) {
    I2C_wait();
    I2C3CONbits.RSEN = 1; // I2C Restart Condition 
    while (I2C3CONbits.RSEN == 1) {} // Wait for Restart Condition to complete 
}

// Generate Acknowledge Condition 
void I2C_ack(void) {
    I2C_wait();
    I2C3CONbits.ACKDT = 0; // Set acknowledge bit to 0 
    I2C3CONbits.ACKEN = 1; // Generate Acknowledge Condition 
    while (I2C3CONbits.ACKEN == 1) {} // Wait for Acknowledge Condition to complete  
}

// Generate Non-Acknowledge Condition 
void I2C_nack(void) {
    I2C_wait();
    I2C3CONbits.ACKDT = 1; // Set acknowledge bit to 1
    I2C3CONbits.ACKEN = 1; // Generate Acknowledge Condition
    while (I2C3CONbits.ACKEN == 1) {} // Wait for Acknowledge Condition to complete
}

// Write a byte to the I2C Bus
void I2C_write(unsigned char data, char ack) {
    I2C_wait(); 
    I2C3TRN = data; // Load data into the transmit register 
    while (I2C3STATbits.TBF == 1) {} // Wait for data transmission to complete 
    I2C_wait();
    if (ack) {
        while (I2C3STATbits.ACKSTAT == 1) {} // Wait for Acknowledge Status bits to be clear // 
    }
}

// Read a byte from the I2C Bus
void I2C_read(unsigned char *value, char ack_nack) {
    I2C3CONbits.RCEN = 1; // Enable Master for reception
    while (I2C3CONbits.RCEN == 1) {} // Wait for reception to complete
    while (I2C3STATbits.RBF == 0) {} // Wait for buffer to be full 
    *value = I2C3RCV; // Read received data 
    if (!ack_nack) { 
        I2C_ack(); // Generate Acknowledge 
    } else {
        I2C_nack(); // Generate Non-Acknowledge 
    }
}

// Send a command and a value to the sensor 
void sensor_send(uint8_t address, uint8_t value) {
    I2C_start(); // Start I2C 
    I2C_write(ADDWRITE, 1); // Send Sensor's Write Address
    I2C_write(address, 1); // Send the register address to write to 
    I2C_write(value, 1); // Send the value to write to the register 
    I2C_stop(); // Stop I2C 
}

// Receive a value from the sensor
void sensor_receive(uint8_t address, uint8_t *value) {
    I2C_start(); // Start I2C 
    I2C_write(ADDWRITE, 1); // Send the Sensor's Write Address
    I2C_write(address, 1); // Send the register address to read from 
    I2C_restart(); // Restart the I2C communication 
    I2C_write(ADDREAD, 1); // Send the Sensor's read address 
    I2C_read(value, 1); // Read the value from the sensor 
    I2C_stop(); // Stop I2C
}

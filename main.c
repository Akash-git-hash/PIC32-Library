#include "PIC32.h"
#include "IO_setup.h"

//#include "Motor_Control.h"

#define IN1 LATAbits.LATA12
#define IN2 LATCbits.LATC14
#define SLEEP LATCbits.LATC11

typedef enum
{
    FORWARD = 1,
    REVERSE = 0,
    STOP = 2
}direction;

void set_Movement(direction);
void set_Speed(uint16_t speed);

int main() {

    // Hello
    TMR1_init();
    
    GPIO_init();
    
    CCPTMR_init(CCP_2);
    CCP_init(CCP_2);
    
    while(1){
    
        set_Movement(REVERSE);
        set_Speed(850);
        
    }
}

void set_Movement(direction direction){

    if(direction == FORWARD){
        SLEEP = 1;
        IN1 = 1;
        IN2 = 0;
        CCP2_ON = 1;
    }

    if(direction == REVERSE){
        SLEEP = 1;
        IN1 = 0;
        IN2 = 1;
        CCP2_ON = 1; 
    }
    
    else if(direction == STOP){
    
        SLEEP = 0;
        CCP2_ON = 0;
    }
    
}

void set_Speed(uint16_t speed){

    if(speed>1000)
        speed = 1000;
    
    if(speed<100)
        speed = 100;
    
    CCP2RB = speed;

}
#include <stdint.h>
#include "msp.h"
#include "../inc/SysTick.h"
#include "../inc/Bump.h"
//**************RSLK1.1***************************
// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)

// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)
// *******Lab 12 solution*******

void Motor_InitSimple(void){
// Initializes the 6 GPIO lines and puts driver to sleep
// Returns right away
// initialize P5.4 and P5.5 and make them outputs

    // Configure Port 2
    P2->SEL0 &= ~0xC0;
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;
    P2->OUT &= ~0xC0;

    // Configure Port 3
    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0;
    P3->DIR |= 0xC0;
    P3->OUT &= ~0xC0;

    // Configure Port 5
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR |= 0x30;
    P5->OUT &= ~0x30;

}

void Motor_StopSimple(void){
// Stops both motors, puts driver to sleep
// Returns right away
  P2->OUT &= ~0xC0;   // Turn off motor
  P3->OUT &= ~0xC0;   // Set sleep
}

void Motor_ForwardSimple(uint16_t duty, uint32_t time){
// Drives both motors forward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Stop the motors and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit

    P5->OUT &= ~0x30;       // Forward direction
    P3->OUT |= 0xC0;        // Enable both motors

    uint32_t i;
    for(i = 0; i < time; i++){
        P2->OUT |= 0xC0;
        SysTick_Wait(48 * duty);              // ON
        P2->OUT &= ~0xC0;
        SysTick_Wait(48 * (10000 - duty));    // OFF for remainder
    }
    Motor_StopSimple();
}

void Motor_BackwardSimple(uint16_t duty, uint32_t time){
// Drives both motors backward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Stop the motors and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit

    P5->OUT |= 0x30;       // Backward direction
    P3->OUT |= 0xC0;        // Enable both motors

    uint32_t i;
    for(i = 0; i < time; i++){
        P2->OUT |= 0xC0;
        SysTick_Wait(48 * duty);              // ON
        P2->OUT &= ~0xC0;
        SysTick_Wait(48 * (10000 - duty));    // OFF for remainder
    }
    Motor_StopSimple();
}

void Motor_LeftSimple(uint16_t duty, uint32_t time){
// Drives just the left motor forward at duty (100 to 9900)
// Right motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit

    P5->OUT &= ~0x30;       // Forward direction
    P3->OUT &= ~0x40;       // Disable right motor
    P3->OUT |= 0x80;        // Enable left motor

    uint32_t i;
    for(i = 0; i < time; i++){
        P2->OUT |= 0xC0;
        SysTick_Wait(48 * duty);              // ON
        P2->OUT &= ~0xC0;
        SysTick_Wait(48 * (10000 - duty));    // OFF for remainder
    }
    Motor_StopSimple();
}
void Motor_RightSimple(uint16_t duty, uint32_t time){
// Drives just the right motor forward at duty (100 to 9900)
// Left motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit

    P5->OUT &= ~0x30;       // Forward direction
    P3->OUT &= ~0x80;       // Disable left motor
    P3->OUT |= 0x40;        // Enable right motor

    uint32_t i;
    for(i = 0; i < time; i++){
        P2->OUT |= 0xC0;
        SysTick_Wait(48 * duty);              // ON
        P2->OUT &= ~0xC0;
        SysTick_Wait(48 * (10000 - duty));    // OFF for remainder
    }
    Motor_StopSimple();
}

// Motor.c
// Runs on MSP432
// Provide mid-level functions that initialize ports and
// set motor speeds to move the robot. Lab 13 solution
// Daniel Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

#include <stdint.h>
#include "msp.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"

// *******Lab 13 solution*******


#define PWM_PERIOD 15000 // Define a Safe Period for PWM


// ------------Motor_Init------------
// Initialize GPIO pins for output, which will be
// used to control the direction of the motors and
// to enable or disable the drivers.
// The motors are initially stopped, the drivers
// are initially powered down, and the PWM speed
// control is uninitialized.
// Input: none
// Output: none
void Motor_Init(void){
    // Initialize Direction pins (P5.4 Left, P5.5 Right)
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR |= 0x30;       // Set as outputs
    P5->OUT &= ~0x30;      // Default to forward (0)

    // Initialize Enable pins (P3.7 Left, P3.6 Right)
    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0;
    P3->DIR |= 0xC0;       // Set as outputs
    P3->OUT &= ~0xC0;      // Default to sleep/disabled (0)

    // Initialize PWM for P2.6 (Right) and P2.7 (Left) to 0 duty cycle
    PWM_Init34(PWM_PERIOD, 0, 0);
}

// ------------Motor_Stop------------
// Stop the motors, power down the drivers, and
// set the PWM speed control to 0% duty cycle.
// Input: none
// Output: none
void Motor_Stop(void){
    P3->OUT &= ~0xC0;      // Put motor drivers to sleep (Disable)
    PWM_Duty4(0);          // Stop Left PWM
    PWM_Duty3(0);          // Stop Right PWM
}

// ------------Motor_Forward------------
// Drive the robot forward by running left and
// right wheels forward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Forward(uint16_t leftDuty, uint16_t rightDuty){ 
    P5->OUT &= ~0x30;      // Both directions forward (P5.4=0, P5.5=0)
    PWM_Duty4(leftDuty);   // Set left speed
    PWM_Duty3(rightDuty);  // Set right speed
    P3->OUT |= 0xC0;       // Wake up motor drivers (Enable)
}

// ------------Motor_Right------------
// Turn the robot to the right by running the
// left wheel forward and the right wheel
// backward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Right(uint16_t leftDuty, uint16_t rightDuty){ 
    P5->OUT &= ~0x10;      // Left forward (P5.4=0)
    P5->OUT |= 0x20;       // Right backward (P5.5=1)
    PWM_Duty4(leftDuty);
    PWM_Duty3(rightDuty);
    P3->OUT |= 0xC0;
}

// ------------Motor_Left------------
// Turn the robot to the left by running the
// left wheel backward and the right wheel
// forward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Left(uint16_t leftDuty, uint16_t rightDuty){ 
    P5->OUT |= 0x10;       // Left backward (P5.4=1)
    P5->OUT &= ~0x20;      // Right forward (P5.5=0)
    PWM_Duty4(leftDuty);
    PWM_Duty3(rightDuty);
    P3->OUT |= 0xC0;
}

// ------------Motor_Backward------------
// Drive the robot backward by running left and
// right wheels backward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Backward(uint16_t leftDuty, uint16_t rightDuty){ 
    P5->OUT |= 0x30;       // Both directions backward (P5.4=1, P5.5=1)
    PWM_Duty4(leftDuty);
    PWM_Duty3(rightDuty);
    P3->OUT |= 0xC0;
}

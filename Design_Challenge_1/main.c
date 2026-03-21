#include "msp.h"
#include "..\inc\Bump.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\FlashProgram.h"
#include "..\inc\Motor.h"

// Declare global variables
volatile uint8_t reflectance_val;
volatile uint8_t bump_val;
volatile uint8_t counter;

// Declare state struct
/*
 *
 */
struct State{
    uint32_t out;                                   // 2-bit directional output
//    uint32_t delay_ms;                              // time to delay
    uint16_t leftDuty;                              // Left Motor PWM
    uint16_t rightDuty;                             // Right Motor PWM
    const struct State *next[4];                    // Next state
};
typedef const struct State State_t;

// Define FSM
//#define Center  &fsm[0]
//#define
//#define Left    &fsm[1]
//#define Right   &fsm[2]
//State_t fsm[3] = {
//    {0x03, }
//    {0x02, }
//    {0x01, }
//};

// Declare function prototypes
void initialize_robot(void);


// Systick Interrupt Handler
void SysTick_Handler(void){                         // every 1ms
    counter += 1;                                   // Increment Counter

}

// Main function
void main(void){
    uint32_t heart = 0;                             // heartbeat
    initialize_robot();                             // initialize the robot
	SysTick_Init(48000, 2);                         // Interrupt @ 1000Hz
	EnableInterrupts();                             // Enable Interrupts

	while(1){
	    heart = heart^1;                            // toggle heartbeat
	    bump_val = Bump_Read();
	    bump_val = Bump_Read();
	    if(bump_val){                   // any nonzero = something pressed
	        Motor_Forward(3000, 3000);
	    } else {
	        Motor_Stop();
	    }
	}
}


// Helper function Implementation
void initialize_robot(void){
    Clock_Init48MHz();                              // Initialize Clock
    LaunchPad_Init();                               // Initialize LED/Buttons
    Bump_Init();                                    // Initialize bump sensor
    Reflectance_Init();                             // Initialize reflectance
    Motor_Init();                                   // Initialize motors
}

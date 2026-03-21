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
volatile uint8_t data_ready;

// Declare state struct
/*
 *
 */
struct State{
    uint32_t dir_out;                                   // 2-bit directional output
    uint16_t left_duty_percent;                              // Left Motor PWM
    uint16_t right_duty_percent;                             // Right Motor PWM
    const struct State *next[5];                    // Next state
};
typedef const struct State State_t;

// Define FSM
#define FWD &fsm[0]
#define SL  &fsm[1]
#define SR  &fsm[2]
#define HL  &fsm[3]
#define HR  &fsm[4]

State_t fsm[5] = {
    {0x03, 100, 100, {SL,SL,FWD,SR,SR}}               // FWD
    {0x03, 60, 100, {HL, SL, FWD, FWD, FWD}}          // SL
    {0x03, 100, 60, {FWD, FWD, FWD, SR, HR}}          // SR
    {0x01, 100, 100, {HL, SL, SL, SL, SL}}            // HL
    {0x02, 100, 100, {SR, SR, SR, SR, HR}}            // HR
};

// Declare function prototypes
void initialize_robot(void);


// Systick Interrupt Handler
void SysTick_Handler(void){                         // every 1ms
    counter += 1;                                   // Increment Counter
    if (count == 0){
        Reflectance_Start();
    } else if (count == 1){
        reflectance_val = Reflectance_End();
        data_ready = 1;
    }
}

// Main function
void main(void){
    uint32_t heart = 0;                             // heartbeat
    initialize_robot();                             // initialize the robot
	SysTick_Init(48000, 2);                         // Interrupt @ 1000Hz
	EnableInterrupts();                             // Enable Interrupts

	while(1){
	    heart = heart^1;                            // toggle heartbeat
	    if (data_ready){

	        data_ready = 0;
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

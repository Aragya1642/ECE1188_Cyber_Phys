#include "msp.h"
#include "..\inc\Bump.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\FlashProgram.h"
#include "..\inc\Motor.h"
#include <stdlib.h>

// Declare global variables
const uint16_t MAX_PWM = 5000;
volatile uint8_t reflectance_val;
volatile uint8_t bump_val;
volatile uint8_t data_ready;

// Declare state struct
/*
 *
 */
struct State{
    void (*drive)(uint16_t, uint16_t);                                   // Pointer to the motor function
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
    {&Motor_Forward, 100, 100, {SL,SL,FWD,SR,SR}},                // FWD
    {&Motor_Forward, 60, 100, {HL, SL, FWD, FWD, FWD}},           // SL
    {&Motor_Forward, 100, 60, {FWD, FWD, FWD, SR, HR}},           // SR
    {&Motor_Left, 100, 100, {HL, SL, SL, SL, SL}},             // HL
    {&Motor_Right, 100, 100, {SR, SR, SR, SR, HR}}              // HR
};

// Declare function prototypes
void initialize_robot(void);
void Pause(void){
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}

// Systick Interrupt Handler
void SysTick_Handler(void){                         // every 1ms
    static uint8_t count = 0;
    if (count == 0){
        Reflectance_Start();
    } else if (count == 1){
        reflectance_val = Reflectance_End();
        data_ready = 1;
    }
    count = (count + 1) % 10;
}

// Main function
void main(void){
    uint32_t heart = 0;                             // heartbeat
    initialize_robot();                             // initialize the robot
	SysTick_Init(48000, 2);                         // Interrupt @ 1000Hz
	EnableInterrupts();                             // Enable Interrupts
	State_t *current = FWD;                         // Initialize in FWD state

	while(1){
	    heart = heart^1;                            // toggle heartbeat
	    if (data_ready){
	        // Get high level position values
	        int32_t reflectance_pos = Reflectance_Position(reflectance_val);
	        int32_t reflectance_pos_abs = abs(reflectance_pos);

	        uint8_t index = 2;
	        // Categorize into degree of turn
	        if (reflectance_pos_abs <= 4800){
	            // Forward
	            index = 2;
	        } else if (reflectance_pos_abs <= 23800){
	            // Slight Turn
	            if (reflectance_pos > 0){
	                index = 3;
	            } else{
	                index = 1;
	            }
	        } else if (reflectance_pos_abs <= 33400){
	            // Hard Turn
	            if (reflectance_pos > 0){
	                index = 4;
	            } else{
	                index = 0;
	            }
	        } // TODO: Add else statements for lost state

	        // Set next state
	        current = current->next[index];

	        // Set motor outputs
	        current->drive((current->left_duty_percent * MAX_PWM) / 100, (current->right_duty_percent * MAX_PWM) / 100);

	        // Set flag low
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

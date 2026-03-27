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
const uint16_t MAX_PWM = 6000;
volatile uint8_t reflectance_val;
volatile uint8_t bump_val;
volatile uint8_t data_ready;


// Declare state struct
struct State{
    void (*drive)(uint16_t, uint16_t);                                   // Pointer to the motor function
    uint16_t left_duty_percent;                              // Left Motor PWM
    uint16_t right_duty_percent;                             // Right Motor PWM
    const struct State *next[6];                    // Next state
};
typedef const struct State State_t;

// Define FSM
#define FWD &fsm[0]
#define SL  &fsm[1]
#define SR  &fsm[2]
#define ML  &fsm[3]
#define MR  &fsm[4]
#define HL  &fsm[5]
#define HR  &fsm[6]


State_t fsm[7] = {
    //  drive,          L%,  R%,    {0:FL, 1:SL, 2:C,   3:SR, 4:FR, 5:LOST}
    {&Motor_Forward, 100, 100,   { ML,   SL,   FWD,  SR,   MR,   FWD }}, // FWD
    {&Motor_Forward,  60, 100,   { HL,   SL,   FWD,  FWD,  SR,   SL  }}, // SL 
    {&Motor_Forward, 100,  60,   { SL,   FWD,  FWD,  SR,   HR,   SR  }}, // SR  
    {&Motor_Forward,   0, 100,   { HL,   ML,   FWD,  SR,   MR,   HL  }}, // ML 
    {&Motor_Forward, 100,   0,   { ML,   SL,   FWD,  MR,   HR,   HR  }}, // MR  
    {&Motor_Left,    100, 100,   { HL,   ML,   FWD,  SR,   HR,   HL  }}, // HL  
    {&Motor_Right,   100, 100,   { HL,   SL,   FWD,  MR,   HR,   HR  }}  // HR  
};

volatile State_t *current = FWD; // Initialize in FWD state

// Declare function prototypes
void initialize_robot(void);

// Systick Interrupt Handler
void SysTick_Handler(void){                         // every 1ms
    static uint8_t count = 0;
    if (count == 0){
        Reflectance_Start();
    } else if (count == 1){
        reflectance_val = Reflectance_End();
        data_ready = 1;
    }
    count = (count + 1) % 3;
}

void PORT4_IRQHandler(void){

    Motor_Stop();
    Clock_Delay1ms(250);
    bump_val = Bump_Read();

    //uint8_t is_left = bump_val & 0b00110000;

    uint8_t is_center = bump_val & 0b00001100;
    uint8_t is_right = bump_val & 0b00000011;

    if (is_center > 0) {
       Motor_Backward((MAX_PWM * 70) / 100, (MAX_PWM * 70) / 100);
       Clock_Delay1ms(250);
    } else if (is_right > 0) {
       Motor_Backward((MAX_PWM * 70) / 100, (MAX_PWM * 20) / 100);
       Clock_Delay1ms(500);
       Motor_Forward((MAX_PWM * 70) / 100, (MAX_PWM * 70) / 100);
    } else {
       Motor_Backward((MAX_PWM * 20) / 100, (MAX_PWM * 70) / 100);
       Clock_Delay1ms(500);
       Motor_Forward((MAX_PWM * 70) / 100, (MAX_PWM * 70) / 100);
    }

    Clock_Delay1ms(500);

    Motor_Stop();

    Clock_Delay1ms(250);
    current = FWD;
    bump_val = 0;

    P4->IFG &= ~0xED;   // clear bump interrupt flags
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
	        // Get high level position values
	        int32_t reflectance_pos = Reflectance_Position(reflectance_val);
	        int32_t reflectance_pos_abs = abs(reflectance_pos);

	        static uint8_t index = 2;


	        // Categorize into degree of turn
	        if (reflectance_val == 0x00){
	            index = 5;


	        }
	        else if (reflectance_pos_abs <= 6000){
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

	        }

	        // transition state
	        current = current->next[index];
	        // Set motor outputs
	        current->drive((current->left_duty_percent * MAX_PWM) / 100, (current->right_duty_percent * MAX_PWM) / 100);

	        // Set flag low=
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

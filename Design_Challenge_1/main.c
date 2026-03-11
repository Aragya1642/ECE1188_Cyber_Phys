#include "msp.h"
#include "..\inc\Bump.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\FlashProgram.h"

// Declare global variables
volatile uint8_t reflectance_val;
volatile uint8_t bump_val;
volatile uint8_t counter;

// Declare function prototypes
void initialize_robot(void);


// Systick Interrupt Handler
void SysTick_Handler(void){                         // every 1ms
    counter += 1;                                   // Increment Counter

}

// Main function
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	initialize_robot();                             // initialize the robot
	SysTick_Init(48000, 2);                         // Interrupt @ 1000Hz

	while(1){

	}
}


// Helper function Implementation
void initialize_robot(void){
    Clock_Init48MHz();                              // Initialize Clock
    LaunchPad_Init();                               // Initialize LED/Buttons
    Bump_Init();                                    // Initialize bump sensor
    Reflectance_Init();                             // Initialize reflectance
}

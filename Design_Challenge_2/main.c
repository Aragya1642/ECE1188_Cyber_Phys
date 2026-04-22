#include <stdint.h>
#include "msp.h"
#include "..\inc\Clock.h"
#include "..\inc\Motor.h"
#include "..\inc\Tachometer.h"
#include "..\inc\odometry.h"
#include "..\inc\Bump.h"
#include "..\inc\CortexM.h"
#include "..\inc\TimerA1.h"
#include "..\inc\opt3101.h"
#include "..\inc\I2CB1.h"
#include "..\inc\UART0.h"

// 0 = STOPPED, 1 = RUNNING
int systemState = 0;

// --- Hardware Constraints ---
#define DESIRED_DISTANCE 250   // mm from the right wall
#define GOAL_X_COORD 2000000   // 2 meters (units of 0.0001cm)

// --- Background Task ---
// This runs exactly every 20ms to accurately integrate tachometer counts
void Background_OdometryTask(void){
    UpdatePosition(); // From odometry.c: reads tachs, calculates math, updates X/Y/Theta
}

int main(void){
    // 1. Initialize System and Peripherals
    Clock_Init48MHz();
    Motor_Init();
    Tachometer_Init();
    Bump_Init();
    UART0_Init();

    I2CB1_Init(30); // Initialize I2C with prescaler 30 (400 kHz baud rate)

    OPT3101_Init();
    OPT3101_Setup(); // Configure sensor registers and monoshot mode
    OPT3101_CalibrateInternalCrosstalk(); // Calibrate phase offsets

    // 2. Initialize Odometry State
    // Start at coordinate (0,0), facing East (0 radians)
    Odometry_Init(0, 0, 0);
    Odometry_SetPower(3000, 1500);

    // 3. Set up Timer A1 for 20ms periodic interrupts (48MHz / 50Hz = 960,000 cycles)
    // This allows Odometry_Update to run constantly in the background
    TimerA1_Init(&Background_OdometryTask, 960000);

    EnableInterrupts();

    // Variables to hold sensor data and position
    uint32_t distLeft, distCenter, distRight;
    int32_t myX, myY, myTheta;

    char command;

    while(1){

        // NON-BLOCKING CHECK: Only attempt to read if a character is available in the buffer
        if((EUSCI_A0->IFG & 0x01) != 0) {
            command = UART0_InChar();

            if (command == 'f' || command == 'F') {
                systemState = 1;
            }
            else if (command == 's' || command == 'S') {
                systemState = 0;
                Motor_Stop(); // Hard stop immediately
            }
        }

        // --- Execute Robot Logic Based on State ---
        if(systemState == 1){
            // Get distances to walls
            distLeft = OPT3101_GetLeft();
            distCenter = OPT3101_GetCenter();
            distRight = OPT3101_GetRight();
            Odometry_Get(&myX, &myY, &myTheta);

            // Goal Check
            if(myX >= GOAL_X_COORD) {
                Motor_Stop();
                systemState = 0; // Change state to STOPPED
            }
            // Wall Following Logic
            else if(distCenter < 200) {
                Motor_Left(2000, 2000);
            }
            else if(distRight < (DESIRED_DISTANCE - 50)) {
                Motor_Forward(2500, 1500); // Right wheel faster, steer left
            }
            else if(distRight > (DESIRED_DISTANCE + 50)) {
                Motor_Forward(1500, 2500); // Left wheel faster, steer right
            }
            else {
                Motor_Forward(2000, 2000); // Drive straight
            }

            // Bump Sensors
            if (Bump_Read() != 0) {
                Motor_Stop();
                systemState = 0; // Stop the system if a collision occurs
            }
        }
        else {
            // If systemState is 0, ensure motors stay off
            Motor_Stop();
        }

        // Brief delay before reading I2C sensors again (e.g., 20ms)
        Clock_Delay1ms(20);
    }
}

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// --- Primary Hardware Includes ---
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
#include "..\inc\Reflectance.h" // Added for DC1 Line Sensor

// --- Wi-Fi / MQTT Includes ---
#include "driverlib.h"
#include "simplelink.h"
#include "sl_common.h"
#include "MQTTClient.h"

// ============================================================================
// MACROS & DEFINITIONS
// ============================================================================

// --- Hardware Constraints ---
#define DESIRED_DISTANCE 350   // mm from the right wall
#define GOAL_X_COORD 2147483647   // 2 meters (units of 0.0001cm)

// --- Wi-Fi Settings ---
#define SSID_NAME       "GoyalPhone"
#define SEC_TYPE        SL_SEC_TYPE_WPA_WPA2
#define PASSKEY         "13572468"
#define PASSKEY_LEN     pal_Strlen(PASSKEY)

// --- MQTT Settings ---
#define MQTT_BROKER_SERVER  "mqtt-dashboard.com"
#define MQTT_BROKER_PORT    1883
#define PUBLISH_TOPIC       "testingtesting12345"
#define BUFF_SIZE 64
#define APPLICATION_VERSION "1.0.0"

// --- SimpleLink Macros ---
#define SL_STOP_TIMEOUT        0xFF
#define SMALL_BUF           32
#define MAX_SEND_BUF_SIZE   512
#define MAX_SEND_RCV_SIZE   1024
#define min(X,Y) ((X) < (Y) ? (X) : (Y))

typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,
    HTTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    HTTP_RECV_ERROR = HTTP_SEND_ERROR - 1,
    HTTP_INVALID_RESPONSE = HTTP_RECV_ERROR -1,
    STATUS_CODE_MAX = -0xBB8
} e_AppStatusCodes;

// ============================================================================
// GLOBAL VARIABLES & TELEMETRY
// ============================================================================

// 0 = STOPPED, 1 = RUNNING
int systemState = 0;

// --- Telemetry Trackers ---
volatile uint32_t systemTimeMs = 0;
uint32_t startTimeMs = 0;
uint32_t crashCount = 0;
uint32_t maxSpeed = 4500;

// --- Line Sensor Trackers ---
volatile uint8_t reflectance_val = 0;
volatile uint8_t finish_line_crossed = 0; // Replaced data_ready with a persistent flag

// --- MQTT / SimpleLink Globals ---
volatile int publishID = 0;
unsigned char macAddressVal[SL_MAC_ADDR_LEN];
unsigned char macAddressLen = SL_MAC_ADDR_LEN;
char macStr[18];
char uniqueID[9];

Network n;
Client hMQTTClient;

_u32  g_Status = 0;
struct{
    _u8 Recvbuff[MAX_SEND_RCV_SIZE];
    _u8 SendBuff[MAX_SEND_BUF_SIZE];
    _u8 HostName[SMALL_BUF];
    _u8 CityName[SMALL_BUF];
    _u32 DestinationIP;
    _i16 SockID;
} g_AppData;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
static _i32 establishConnectionWithAP();
static _i32 configureSimpleLinkToDefaultState();
static _i32 initializeAppVariables();
static void displayBanner();
static void messageSend();
static void generateUniqueID();

// ============================================================================
// ASYNCHRONOUS EVENT HANDLERS (Required by SimpleLink)
// ============================================================================
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
    if(pWlanEvent == NULL) ;
    switch(pWlanEvent->Event) {
        case SL_WLAN_CONNECT_EVENT:
            SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            break;
        case SL_WLAN_DISCONNECT_EVENT:
            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);
            break;
        default: break;
    }
}

void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
    if(pNetAppEvent == NULL) ;
    switch(pNetAppEvent->Event) {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);
            break;
        default: break;
    }
}

void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse) {}
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent) {}
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {}

// ============================================================================
// BACKGROUND TASKS & CALLBACKS (Timer Multiplexing)
// ============================================================================

// This single task fires every 1ms to handle BOTH line sensing and odometry
void Background_1ms_Task(void){
    // --- 1. Reflectance Sensor Logic (Runs every 1ms) ---
    static uint8_t ref_count = 0;
    if (ref_count == 0){
        Reflectance_Start();
    } else if (ref_count == 1){
        reflectance_val = Reflectance_End();

        // NEW: Check for the finish line HERE at high speed (333Hz).
        // 0x7E is 0b01111110. This ensures the 6 center sensors see black,
        // ignoring the 2 outer edges which are prone to ambient light bleed.
        if ((reflectance_val & 0x7E) == 0x7E) {
            finish_line_crossed = 1;
        }
    }
    ref_count = (ref_count + 1) % 3;

    // --- 2. Odometry Logic (Runs every 20ms) ---
    static uint8_t odom_count = 0;
    odom_count++;
    if(odom_count >= 20) {
        UpdatePosition();
        systemTimeMs += 20; // Increment global time tracker
        odom_count = 0;     // Reset the 20ms counter
    }
}

// Packages the 3 global variables and broadcasts them to Node-RED
static void messageSend(void) {
    char payload[BUFF_SIZE];
    uint32_t activeRunTime = systemTimeMs - startTimeMs;

    // Format telemetry as a JSON string
    snprintf(payload, BUFF_SIZE, "{\"speed\":%d,\"time\":%d,\"crashes\":%d}",
             maxSpeed, activeRunTime, crashCount);

    MQTTMessage msg;
    msg.dup = 0;
    msg.id = 0;
    msg.payload = payload;
    msg.payloadlen = strlen(payload);
    msg.qos = QOS0;
    msg.retained = 0;

    // Publish to the MQTT Broker
    int rc = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC, &msg);
}

// ============================================================================
// HARDWARE INTERRUPT HANDLERS
// ============================================================================

// Hardware Interrupt for Bump Sensors
void PORT4_IRQHandler(void) {
    uint8_t bump_val;

    // 1. Immediately stop motors and delay 500ms
    Motor_Stop();
    Clock_Delay1ms(500);

    // 2. Read bumper state
    bump_val = Bump_Read();

    // 3. Log the crash for telemetry
    crashCount++;

    // 4. Isolate sensor zones
    uint8_t is_center = bump_val & 0b00001100;
    uint8_t is_right  = bump_val & 0b00000011;

    // 5. Execute Evasive Maneuver
    if (is_center > 0) {
        Motor_Backward((maxSpeed * 70) / 100, (maxSpeed * 70) / 100);
        Clock_Delay1ms(250);
    }
    else if (is_right > 0) {
        Motor_Backward((maxSpeed * 70) / 100, (maxSpeed * 20) / 100);
        Clock_Delay1ms(500);
        Motor_Forward((maxSpeed * 70) / 100, (maxSpeed * 70) / 100);
    }
    else {
        // Left side or default
        Motor_Backward((maxSpeed * 20) / 100, (maxSpeed * 70) / 100);
        Clock_Delay1ms(500);
        Motor_Forward((maxSpeed * 70) / 100, (maxSpeed * 70) / 100);
    }

    Clock_Delay1ms(500);

    // 6. Stabilize before returning to state machine
    Motor_Stop();
    Clock_Delay1ms(250);

    // 7. Clear bump interrupt flags to prevent Default_Handler crash
    P4->IFG &= ~0xED;
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================
int main(void){
    _i32 retVal = -1;
    int rc = 0;
    unsigned char buf[100];
    unsigned char readbuf[100];

    // 1. Initialize Base Clock
    Clock_Init48MHz();

    // 2. Clear any lingering ghost interrupts on Port 2
    P2->IFG = 0;

    // 3. Enable Global Interrupts
    EnableInterrupts();
    Interrupt_enableMaster();

    // 4. Give Wi-Fi chip time to boot
    Clock_Delay1ms(100);

    // --- WI-FI & MQTT INITIALIZATION BLOCK ---
    retVal = initializeAppVariables();
    UART0_Init();

    retVal = configureSimpleLinkToDefaultState();
    if(retVal < 0) LOOP_FOREVER();

    retVal = sl_Start(0, 0, 0);
    if ((retVal < 0) || (ROLE_STA != retVal) ) LOOP_FOREVER();

    retVal = establishConnectionWithAP();
    if(retVal < 0) LOOP_FOREVER();

    sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&macAddressLen,(unsigned char *)macAddressVal);
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            macAddressVal[0], macAddressVal[1], macAddressVal[2], macAddressVal[3], macAddressVal[4], macAddressVal[5]);
    generateUniqueID();

    NewNetwork(&n);

    rc = ConnectNetwork(&n, MQTT_BROKER_SERVER, MQTT_BROKER_PORT);
    if (rc != 0) LOOP_FOREVER();

    MQTTClient(&hMQTTClient, &n, 5000, buf, 100, readbuf, 100);
    MQTTPacket_connectData cdata = MQTTPacket_connectData_initializer;

    cdata.MQTTVersion = 4;
    cdata.clientID.cstring = uniqueID;

    rc = MQTTConnect(&hMQTTClient, &cdata);
    if (rc != 0) {
        LOOP_FOREVER();
    }
    // -----------------------------------------

    // 2. Initialize Robot Hardware Peripherals
    Motor_Init();
    Tachometer_Init();
    Bump_Init();
    I2CB1_Init(30);

    OPT3101_Init();
    OPT3101_Setup();
    OPT3101_CalibrateInternalCrosstalk();

    Reflectance_Init(); // Initialize line sensor pins

    // 3. Initialize Odometry State
    Odometry_Init(0, 0, 0);
    Odometry_SetPower(3000, 1500);

    // 4. Start Timer Multiplexing
    TimerA1_Init(&Background_1ms_Task, 48000);

    uint32_t distLeft, distCenter, distRight;
    int32_t myX, myY, myTheta;
    char command;

    // ========================================================================
    // MAIN LOOP
    // ========================================================================
    while(1){

        // 1. Service MQTT Client (Keepalive ping)
        rc = MQTTYield(&hMQTTClient, 10);
        if (rc != 0) LOOP_FOREVER(); // Connection lost

        // 2. Service UART Bluetooth Commands
        if((EUSCI_A0->IFG & 0x01) != 0) {
            command = UART0_InChar();

            if (command == 'f' || command == 'F') {
                systemState = 1;
                startTimeMs = systemTimeMs; // Mark the start time
                crashCount = 0;             // Reset crash counter for the new run
                finish_line_crossed = 0;    // Reset finish line flag
            }
            else if (command == 's' || command == 'S') {
                systemState = 0;
                Motor_Stop(); // Hard stop immediately
                messageSend(); // Transmit telemetry data to Node-RED
            }
        }

        // --- Tuning Parameters for P-Control ---
        int32_t baseSpeed = 5000; // Normal driving speed
        int32_t Kp = 10;          // Proportional Gain
        int32_t minSpeed = 1000;  // Minimum PWM to prevent stalling

        // --- Execute Robot Logic Based on State ---
        if(systemState == 1){

            // --- Black Line Finish Detection ---
            if (finish_line_crossed == 1) {
                finish_line_crossed = 0; // Clear the flag
                Motor_Stop();
                systemState = 0;
                messageSend(); // Transmit final telemetry data to Node-RED
                continue;      // Skip the rest of the movement loop
            }

            distLeft = OPT3101_GetLeft();
            distCenter = OPT3101_GetCenter();
            distRight = OPT3101_GetRight();
            Odometry_Get(&myX, &myY, &myTheta);

            if(myX >= GOAL_X_COORD) {
                Motor_Stop();
                systemState = 0;
            }
            // Front Collision Avoidance (Overrides wall following)
            else if(distCenter < 200) {
                Motor_Left(2000, 2000);
            }
            // Proportional Wall Following
            else {
                int32_t error = DESIRED_DISTANCE - distRight;
                int32_t turnAdjustment = Kp * error;

                int32_t leftPWM = baseSpeed - turnAdjustment;
                int32_t rightPWM = baseSpeed + turnAdjustment;

                if (leftPWM > maxSpeed) leftPWM = maxSpeed;
                if (leftPWM < minSpeed) leftPWM = minSpeed;
                if (rightPWM > maxSpeed) rightPWM = maxSpeed;
                if (rightPWM < minSpeed) rightPWM = minSpeed;

                Motor_Forward(leftPWM, rightPWM);
            }
        }
        else {
            Motor_Stop();
        }

        Clock_Delay1ms(10);
    }
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static _i32 configureSimpleLinkToDefaultState() {
    SlVersionFull ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    _u8 val = 1;
    _u8 configOpt = 0;
    _u8 configLen = 0;
    _u8 power = 0;
    _i32 retVal = -1;
    _i32 mode = -1;

    mode = sl_Start(0, 0, 0);
    if (ROLE_STA != mode) {
        if (ROLE_AP == mode) {
            while(!IS_IP_ACQUIRED(g_Status)) { _SlNonOsMainLoopTask(); }
        }
        retVal = sl_WlanSetMode(ROLE_STA);
        retVal = sl_Stop(SL_STOP_TIMEOUT);
        retVal = sl_Start(0, 0, 0);
    }

    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));

    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    retVal = sl_WlanProfileDel(0xFF);

    retVal = sl_WlanDisconnect();
    if(0 == retVal) {
        while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }
    }

    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);

    power = 0;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    retVal = sl_NetAppMDNSUnRegisterService(0, 0);

    pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));

    retVal = sl_Stop(SL_STOP_TIMEOUT);
    retVal = initializeAppVariables();

    return retVal;
}

static _i32 establishConnectionWithAP() {
    SlSecParams_t secParams = {0};
    _i32 retVal = 0;

    secParams.Key = PASSKEY;
    secParams.KeyLen = PASSKEY_LEN;
    secParams.Type = SEC_TYPE;

    retVal = sl_WlanConnect(SSID_NAME, pal_Strlen(SSID_NAME), 0, &secParams, 0);
    while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status))) { _SlNonOsMainLoopTask(); }

    return SUCCESS;
}

static _i32 initializeAppVariables() {
    g_Status = 0;
    pal_Memset(&g_AppData, 0, sizeof(g_AppData));
    return SUCCESS;
}

static void generateUniqueID() {
    CRC32_setSeed(TLV->RANDOM_NUM_1, CRC32_MODE);
    CRC32_set32BitData(TLV->RANDOM_NUM_2);
    CRC32_set32BitData(TLV->RANDOM_NUM_3);
    CRC32_set32BitData(TLV->RANDOM_NUM_4);
    int i;
    for (i = 0; i < 6; i++)
        CRC32_set8BitData(macAddressVal[i], CRC32_MODE);

    uint32_t crcResult = CRC32_getResult(CRC32_MODE);
    sprintf(uniqueID, "%06X", crcResult);
}

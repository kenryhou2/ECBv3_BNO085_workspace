/* ========================================
 *
 * Copyright CMU BIOROBOTICS LAB,2019
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF CMU BIORROBOTICS LAB
 *
 * ========================================
*/

#include "project.h"
#include "include/IMU_BNO085.h"
#include <stdio.h> 

/**********CONSTANT VALUES AND FLAGS*********/
#define USBUART_MODE                         //Debugging does not work with USBUART enabled. Disable for debugging and performance
#define BUZZER_TIMER_CLK 8000000
#define USBFS_DEVICE    (0u)

/* The buffer size is equal to the maximum packet size of the IN and OUT bulk
* endpoints.
*/
#define USBUART_BUFFER_SIZE (64u)
#define LINE_STR_LENGTH     (20u)

//IMU Data modes
#define DATA_GAME_ROT_VEC   0 //Game Rotation Vector uses a variety of raw sensors to generate a quaternion.
#define ACCELEROMETER       1 
#define DATA_TRANSM2        3

#define INVALID_INPUT -1
#define NO_CMD_FOUND  -2
#define OUT_OF_CHARS  -3
#define CMD_SIZE 3

//GLOBALS
uint16 buzzer_freq = 20;
uint16 buzzer_clock_div = 2;
uint16 buzzer_timer_peorid = 1;
uint16 buzzer_timer_cmp1 = 0;
uint16 buzzer_chirp_dir = 1;
uint8_t count;                     //Used in DATATRANSM2
uint8 buffer[USBUART_BUFFER_SIZE]; //used for USBUART in DATATRANSM2
sh2_Hal_t *pSh2Hal;
sh2_ProductIds_t prodIds;           //Used for getting product ID in Sh2_getProdID().           

//Debugging constants
int interruptval = 1;
volatile int status;                //Every sh2 function returns an int. Values specified in Sh2_err.h
static char str [64];
volatile uint32_t time1 = 0;
volatile uint32_t time2 = 0;
float transmitBuf[10];   

//FLAGS
uint8_t IMU_READY = 0; //Startup IMU not ready
uint8_t state = DATA_GAME_ROT_VEC;

//STATIC VARS 
//Def: Static variables retains value even when declared out of scope... essentially a global variable with a few caveats.
//Def: Statif functions are functions reserved for that particular c file. Non static functions can be called outside the file 
//it was declared in to be used in other files.

static sh2_AsyncEvent_t async_event; //for helper function eventHandler()
static uint8_t send_time = 0u;
static uint8_t new_imu_data = 0;        //Used only in DATATRANSM2
static sh2_SensorEvent_t sensor_event;  //Used in start_reports()

/****HELPER FUNCTIONS*****/

//Utility function for debugging
void printOut(char s[64])           
{
    /*
    #ifdef USBUART_MODE
    //Print Statement. Each time you print, use both CDCIsReady() and PutString(). 
    while(0 == USBUART_CDCIsReady())
    {
        //This function returns a nonzero value if the Component is ready to send more data to the PC; 
        //otherwise, it returns zero.
    }
    USBUART_PutString(s);
    CyDelay(1); //delay so statement can fully print
    //End Print Statement
    #endif //USBUART_MODE
    */
}


//Utility function for debugging
char * f2cstring(char s[32],float f) //Converts floats to a string for printOut
{
    char *tmpSign = (f < 0) ? "-" : "";
    float tmpVal = (f < 0) ? -f : f;            
                                           //Example: 678.0123
    int tmpInt1 = tmpVal;                  // Get the integer (678).
    float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
    int tmpInt2 = trunc(tmpFrac * 1000000);  // Turn into integer (123). match zeroes with precision 6
    // Print as parts, note that you need 0-padding for fractional bit.
    sprintf (s, "%s%d.%06d", tmpSign, tmpInt1, tmpInt2); //6 places after the decimal precision. Will just append zeroes if float is not specified to have that precision
    return s;
}

//Utility function for debugging
void testPrintFloats()
{
    char s[32];
    char s2[32];
    float f = 3.14159;
    float f2 = -3.1415999;
    sprintf(str,"pi: %s, neg pi: %s\r\n",f2cstring(s,f),f2cstring(s2,f2));
    printOut(str);
}

//Utility function for debugging
void printDataToStr(float i, float j, float k)
{
    char s0[32];
    char s1[32];
    char s2[32];
    //char s3[32];
    sprintf(str,"x:%s y:%s z:%s\r\n",f2cstring(s0,i),f2cstring(s1,j),f2cstring(s2,k));
    printOut(str);
}

void printGameRotVec(float i,float j,float k,float r)
{
    char s0[32];
    char s1[32];
    char s2[32];
    char s3[32];
    sprintf(str,"i:%s j:%s k:%s r:%s\r\n",f2cstring(s0,i),f2cstring(s1,j),f2cstring(s2,k),f2cstring(s3,r));
    printOut(str);
}

void sendFloatArr(float i, float j, float k, float r)
{
    transmitBuf[0] = i;
    transmitBuf[1] = j;
    transmitBuf[2] = k;
    transmitBuf[3] = r;
    USBUART_PutData((void *)transmitBuf, 16);
}

static int start_reports()
{
    static sh2_SensorConfig_t config;
    int status;
    int sensorID;
    
    static const int enabledSensors[] = {SH2_GAME_ROTATION_VECTOR, SH2_ACCELEROMETER, SH2_CAL_GYRO};// SH2_ACCELEROMETER
    
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;

    // Select a report interval.
    //config.reportInterval_us = 10000;  // microseconds (100Hz)
    config.reportInterval_us = 2500;   // microseconds (400Hz)
    // config.reportInterval_us = 1000;   // microseconds (1000Hz)

    for (unsigned int n = 0; n < ARRAY_LEN(enabledSensors); n++)
    {
        // Configure the sensor hub to produce these reports
        sensorID = enabledSensors[n];
        status = sh2_setSensorConfig(sensorID, &config);
        if (status != 0) 
        {
            return status;
        }
    }
    //return status;
}

static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent){
    sensor_event = *pEvent;
    return;
}

static void eventHandler(void *cookie, sh2_AsyncEvent_t *pEvent){
    for(int i = 0; i < 5; i++){}
    async_event = *pEvent;
    return;
}

int process_commands(uint8_t *buffer, uint8_t count)
{
    //Avoid undefined behavior with checks for improper inputs
    if(buffer == NULL | count == 0 | count >= USBUART_BUFFER_SIZE) return INVALID_INPUT;
    
    //Search for the start of a command, marked by the character 'C'
    uint8_t ind, num_ex = 0;
    while(ind < count){
        //Search the buffer for the start character
        uint8_t cmd_start = 0;
        while(ind < count && !cmd_start){
            cmd_start = buffer[ind] == 'C';
            ind += 1;
        }
        if(!cmd_start) return NO_CMD_FOUND;
        
        //If we find the start character, ensure that we do not run out of characters
        if(ind + CMD_SIZE > count) return OUT_OF_CHARS;
        num_ex++; //If valid length, add to counter of commands executed
        char cmd = buffer[ind];
        switch(cmd){
            //Set indicator LEDs command
            //case 'S': set_LEDs(buffer[ind+2]); break;
            //case 'G': send_Time(); break;
            default: 
                num_ex--; //If invalid command, don't count it
        }
        ind += CMD_SIZE;
    }
    return num_ex;
}

// Read product ids with version info from sensor hub and print them
static int reportProdIds(void)
{
    int status;
    
    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    
    if (status < 0) {
        sprintf(str,"Error from sh2_getProdIds: %d\r\n",status);
        printOut(str);
        return status;
    }

    // Report the results
    for (int n = 0; n < prodIds.numEntries; n++) {
        sprintf(str,"Part %d : Version %d.%d.%d Build %d\r\n",
               prodIds.entry[n].swPartNumber,
               prodIds.entry[n].swVersionMajor, prodIds.entry[n].swVersionMinor, 
               prodIds.entry[n].swVersionPatch, prodIds.entry[n].swBuildNumber);
        printOut(str);
        // Wait a bit so we don't overflow the console output.
        CyDelayUs(10000);
    }
    return status;
}

//Phase functions:
/* Order of Operations for IMU to work
    1. If USBUART enabled, start our USBUART
    2. Initialize non IMU associated hardware (LEDs, Buzzer)
    3. Initialize HAL object and functions
    4. Open our Sh2 object which sets up parameters from HAL to Sh2, and SHTP. 
    5. Enable our sensors and configure them for communication through reports. (with setSensorCallback, and start_reports)
       Now we can use SH2 functions to communicate with IMU. (functs such as Sh2_getProdID())
    6. Repeatedly call Sh2_service and decode_Sensor_Event to repeatedly obtain data from IMU.
       Embedded system unable to print floats so need to convert them into s strings.
*/

void USBUART_setup()
{
    #ifdef USBUART_MODE
    USBUART_Start(USBFS_DEVICE, USBUART_5V_OPERATION);
    while(0 == USBUART_GetConfiguration())  //wait for enumeration
    {
        //If stuck here, USB has not enumerated properly. 
        //Check Hardware connection, USBUART CDC settings: (clock, driver update,endpoints)
    } 
    USBUART_CDC_Init(); //Initialize PC to receive data from USBUART
    while (0u == USBUART_GetConfiguration())
    {
        //This is to see if device has been enumerated properly
    }
    CyDelay(2000); //Henry: Need a delay here to delay the main loop for any init debug msgs
    //end USBUART Start process
    #endif //USBUART_MODE
}

void IMU_setup()
{
    //Note: Initialization process involves sh2_hal_init AND sh2_open functions. These setup firmware for reading from IMU.
    pSh2Hal = sh2_hal_init(); //Within here we setup all functions dealing with IMU including open(), close(), read(), write().
    //Those IMU functions will set up IMU Interrupt, IMU I2C comms, and timer for timestamps.
    status = sh2_open(pSh2Hal, eventHandler, NULL); //sh2_open calls sh2_i2c_hal_open() in sh2_hal_psoc5.c
    sprintf(str,"sh2_open returned with status: %d\r\n",status);
    //printOut(str);
    status = sh2_setSensorCallback(sensorHandler, NULL); 
}

int main(void)
{
    CyGlobalIntEnable; // Enable global interrupts.
   
    /****SYSTEM INITS****/
    
    USBUART_setup();             //Start USBUART start process
    PWM_LED_Start();             //Init debug LEDs and Buzzer... no audio though
    PWM_EN_Start();
    PWM_BUZZER_Start();   
    
    printOut("****INITIALIZATION START****\r\n");
    IMU_setup();                 //Initialize IMU using SH2 HAL
    start_reports();
    //status = reportProdIds();  //Simple Sh2 function to get a product ID. Used in debugging to verify HAL read and write.
	
    /****LOOP****/
    bool got_accel = 0, got_gyro = 0, got_rot = 0;
    for(;;)
    {
        sh2_service();
        sh2_SensorValue_t value;
        status = sh2_decodeSensorEvent(&value, &sensor_event); //Sensors output events as reports everytime the IMU reads from the sensor.
        
        switch(sensor_event.reportId)
        {
            case SH2_GAME_ROTATION_VECTOR:
            {
                got_rot = 1;
                transmitBuf[0] = value.un.gameRotationVector.i;
                transmitBuf[1] = value.un.gameRotationVector.j;
                transmitBuf[2] = value.un.gameRotationVector.k;
                transmitBuf[3] = value.un.gameRotationVector.real;
                break;
            }
            case SH2_ACCELEROMETER:
            {
                got_accel = 1;
                transmitBuf[4] = value.un.accelerometer.x; 
                transmitBuf[5] = value.un.accelerometer.y; 
                transmitBuf[6] = value.un.accelerometer.z;
                break;
            }
            case SH2_CAL_GYRO:
            {
                got_gyro = 1;
                transmitBuf[7] = value.un.gyroscope.x; 
                transmitBuf[8] = value.un.gyroscope.y; 
                transmitBuf[9] = value.un.gyroscope.z;
                break;
            }
            default:
              continue;
        }
        
        if (got_accel && got_gyro && got_rot)
        {
            printOut("");
            #ifdef  USBUART_MODE
            USBUART_PutData(( void *)transmitBuf,40);
            #endif
        }
        
    } //End For loop
} //end Main

    
/* [] END OF FILE */

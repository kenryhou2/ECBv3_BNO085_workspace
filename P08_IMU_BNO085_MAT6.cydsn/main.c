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
#define DATA_OUTPUT_MODE                     //In order to debug, comment out DATA_OUTPUT_MODE to be able to print strings. But for procedure use we just want serial output of the float data.


#define BUZZER_TIMER_CLK 8000000
#define USBFS_DEVICE    (0u)

/* The buffer size is equal to the maximum packet size of the IN and OUT bulk
* endpoints.
*/
#define USBUART_BUFFER_SIZE (64u)
#define LINE_STR_LENGTH     (20u)

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
sh2_SensorValue_t value;
//Debugging constants
int interruptval = 1;
volatile int status;                //Every sh2 function returns an int. Values specified in Sh2_err.h
static char str [128];              //C-String used in printOut()
volatile uint32_t time1 = 0;
volatile uint32_t time2 = 0;
float transmitBuf[10];              //Used to hold the float data from IMU when broadcasting in Data output mode.
//float transmitBuf[7]; //quaternion, linear acceleration
int16_t transmitIntBuf[6];  //int16_t is 2 byte signed integer  //Used for raw IMU data when broadcasting in Data Output mode.

bool got_accel = 0, got_linAccel = 0, got_gyro = 0, got_rot = 0, got_rot_stable = 0, got_mag = 0, got_rawGyro = 0, got_rawAccel = 0, got_gyroRV = 0; //Flags to detect when sensor_event activates a certain sensor to receive data from
uint8_t gameAccuracy = 0, gyroAccuracy = 0, accelAccuracy = 0, magAccuracy = 0; //Reports from each sensor field have a status bit indicating how accurate the data is from [0-3] 3 being most accurate.
uint32_t accCount = 0;      //accuracy count: Used to record how many times the enabled sensors are the accuracy threshold.
volatile uint32_t tdf;      //time difference debug variable
bool cal = false;           //calibrated boolean. Becomes true when accCount reaches a certain number.
bool firing_pin_state = false;  //Debug pin boolean used in toggling the state of the firing pin.

//FLAGS
uint8_t IMU_READY = 0; //Startup IMU not ready

//STATIC VARS 
//Def: Static variables retains value even when declared out of scope... essentially a global variable with a few caveats.
//Def: Statif functions are functions reserved for that particular c file. Non static functions can be called outside the file 
//it was declared in to be used in other files.

static sh2_AsyncEvent_t async_event; //for helper function eventHandler()
static uint8_t send_time = 0u;
static sh2_SensorEvent_t sensor_event;  //Used in start_reports()

/****HELPER FUNCTIONS*****/

//Utility function for debugging
void printOut(char s[64])           
{ 
    #ifdef USBUART_MODE
    #ifndef DATA_OUTPUT_MODE
    //Print Statement. Each time you print, use both CDCIsReady() and PutString(). 
    while(0 == USBUART_CDCIsReady())
    {
        //This function returns a nonzero value if the Component is ready to send more data to the PC; 
        //otherwise, it returns zero.
    }
    USBUART_PutString(s);
    CyDelay(1); //delay so statement can fully print
    //End Print Statement
    #endif
    #endif //USBUART_MODE
}


//Utility function
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

//Returns time difference between two calls of this function.
uint32_t getTdiff()
{
    uint32_t tdiff = 0;
    time2 = get_timestamp();
    
    tdiff = time2 - time1;
    time1 = time2;
    return tdiff;
}

//Utility function
void toggleFiringPin()
{
    if(firing_pin_state)
        TIMER_FIRING_PIN_Write(0);
    else
        TIMER_FIRING_PIN_Write(1);
        
    firing_pin_state = !firing_pin_state;
}

//Utility function for printing out string form of the IMU data. This is our printout during non Data output mode.


void print10(float fa[10])//, int16_t ia[3], uint8_t gameAcc, uint8_t accelAcc, uint8_t gyroAcc)
{
    float a,b,c,d,e,f,g,h,i,j;
    char s0[32];
    char s1[32];
    char s2[32];
    char s3[32];
    char s4[32];
    char s5[32];
    char s6[32];
    char s7[32];
    char s8[32];
    char s9[32];

    //Printing for Arduino serial plotter

//    sprintf(str,"%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\r\n",
//    f2cstring(s0,fa[0]),f2cstring(s1,fa[1]),
//    f2cstring(s2,fa[2]),f2cstring(s3,fa[3]),
//    f2cstring(s4,fa[4]),f2cstring(s5,fa[5]),
//    f2cstring(s6,fa[6]),f2cstring(s7,fa[7]),
//    f2cstring(s8,fa[8]),f2cstring(s9,fa[9])
//    );
//    printOut(str);
    
    sprintf(str,"%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\r\n",
    f2cstring(s0,fa[0]*5),f2cstring(s1,fa[1]*5),
    f2cstring(s2,fa[2]*5),f2cstring(s3,fa[3]*5),
    f2cstring(s4,fa[4] + 20),f2cstring(s5,fa[5] + 20),
    f2cstring(s6,fa[6] + 20),f2cstring(s7,fa[7] - 20),
    f2cstring(s8,fa[8] - 20),f2cstring(s9,fa[9] - 20)
    );
    printOut(str);

    //quaternion only 
//    sprintf(str,"%s\t%s\t%s\t%s\r\n",
//    f2cstring(s0,fa[0]),f2cstring(s1,fa[1]),
//    f2cstring(s2,fa[2]),f2cstring(s3,fa[3]));
//    printOut(str);
    
    //gyro only
//    sprintf(str,"%s\t%s\t%s\r\n",
//    f2cstring(s0,fa[7]),f2cstring(s1,fa[8]),
//    f2cstring(s2,fa[9]));
//    printOut(str);
    
    //accelerometer only
//    sprintf(str,"%s\t%s\t%s\r\n",
//    f2cstring(s0,fa[4]),f2cstring(s1,fa[5]),
//    f2cstring(s2,fa[6]));
//    printOut(str);
    
    //End printing for Arduino serial plotter.
    
//    sprintf(str,"i:%s\tj:%s\tk:%s\tr:%s\tx:%s\ty:%s\tz:%s\twx:%d\twy:%d\twz:%d\r\n",
//    f2cstring(s0,fa[0]),f2cstring(s1,fa[1]),
//    f2cstring(s2,fa[2]),f2cstring(s3,fa[3]), 
//    f2cstring(s4,fa[4]),f2cstring(s5,fa[5]),
//    f2cstring(s6,fa[6]),
//    ia[0],ia[1],ia[2]
//    );
//    printOut(str);
    
}

//Used to enable and configure sensor fields before reading from them.
static int start_reports()
{
    static sh2_SensorConfig_t config;
    static sh2_SensorConfig_t accelConfig;
    int status;
    int sensorID;
    

    static const int enabledSensors[] = {SH2_GYRO_INTEGRATED_RV, SH2_ACCELEROMETER};
//    static const int enabledSensors[] = 
//    {SH2_RAW_GYROSCOPE, 
//     SH2_GYROSCOPE_CALIBRATED,
//     //SH2_GAME_ROTATION_VECTOR,
//     SH2_RAW_ACCELEROMETER,
//     SH2_ACCELEROMETER,
//     SH2_GYRO_INTEGRATED_RV,
//    };    
    config.changeSensitivityEnabled = false;
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;
    
    
    accelConfig.changeSensitivityEnabled = false;
    accelConfig.changeSensitivityEnabled = false;
    accelConfig.wakeupEnabled = false;
    accelConfig.changeSensitivityRelative = false;
    accelConfig.alwaysOnEnabled = false;
    accelConfig.changeSensitivity = 0;
    accelConfig.batchInterval_us = 0;
    accelConfig.sensorSpecific = 0;
        
    // Select a report interval.
    //config.reportInterval_us = 10000;  // microseconds (100Hz)
    //config.reportInterval_us = 2500;   // microseconds (400Hz)
    //config.reportInterval_us = 1000;  // microseconds (1000 Hz)
    config.reportInterval_us = 5000;   // microseconds ( 200 Hz)
    accelConfig.reportInterval_us = 1000; //Separate accel config variable  so we can change the report rate.
    
    //Note for implementing multiple sensors try to keep report rates different so that IMU processor doesn't get overwhelmed and decrease output frequency.

    for (unsigned int n = 0; n < ARRAY_LEN(enabledSensors); n++)
    {
        // Configure the sensor hub to produce these reports
        sensorID = enabledSensors[n];
        status = sh2_setSensorConfig(sensorID, &config);
        if(sensorID == SH2_ACCELEROMETER)
        {
            status = sh2_setSensorConfig(sensorID, &accelConfig);
        }
        if (status != 0) 
        {
            return status;
        }
    }
    //return status;
}

static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent){
   
    sensor_event = *pEvent;
    status = sh2_decodeSensorEvent(&value, &sensor_event); //sensor_event fluctuates type of data it is outputting randomly. Use flags to detect when certain sensor is selected.
    switch(sensor_event.reportId)
    {
        case SH2_GAME_ROTATION_VECTOR:                  //detection of the event being for a certain sensor.
        {
            got_rot = 1;
            transmitBuf[0] = value.un.gameRotationVector.i;
            transmitBuf[1] = value.un.gameRotationVector.j;
            transmitBuf[2] = value.un.gameRotationVector.k;
            transmitBuf[3] = value.un.gameRotationVector.real;
            gameAccuracy = (sensor_event.report[2] & 0x03);
            break;
        }
        
        case SH2_LINEAR_ACCELERATION:                  //detection of the event being for a certain sensor.
        {
            got_linAccel = 1;
            transmitBuf[4] = value.un.linearAcceleration.x;
            transmitBuf[5] = value.un.linearAcceleration.y;
            transmitBuf[6] = value.un.linearAcceleration.z;
            break;
        }
        case SH2_GYRO_INTEGRATED_RV:
        { //reports at ~300 Hz
            got_gyroRV = 1;
            transmitBuf[0] = value.un.gyroIntegratedRV.i;
            transmitBuf[1] = value.un.gyroIntegratedRV.j;
            transmitBuf[2] = value.un.gyroIntegratedRV.k;
            transmitBuf[3] = value.un.gyroIntegratedRV.real;
            transmitBuf[7] = value.un.gyroIntegratedRV.angVelX;
            transmitBuf[8] = value.un.gyroIntegratedRV.angVelY;
            transmitBuf[9] = value.un.gyroIntegratedRV.angVelZ;
            break;
        }
        
        case SH2_ACCELEROMETER:
        {
            got_accel = 1;
            transmitBuf[4] = value.un.accelerometer.x; 
            transmitBuf[5] = value.un.accelerometer.y; 
            transmitBuf[6] = value.un.accelerometer.z;
            accelAccuracy = (sensor_event.report[2] & 0x03);     
            
            break;
        }
        case SH2_GYROSCOPE_CALIBRATED:
        {
            got_gyro = 1;
            transmitBuf[7] = value.un.gyroscope.x; 
            transmitBuf[8] = value.un.gyroscope.y; 
            transmitBuf[9] = value.un.gyroscope.z;
            gyroAccuracy = (sensor_event.report[2] & 0x03);   
         
            break;
        }
        
        case SH2_RAW_GYROSCOPE:
        {
            got_rawGyro = 1;
            transmitIntBuf[0] = value.un.rawGyroscope.x; 
            transmitIntBuf[1] = value.un.rawGyroscope.y;  
            transmitIntBuf[2] = value.un.rawGyroscope.z;             
            break;
        }
        
        case SH2_RAW_ACCELEROMETER:
        {
            got_rawAccel = 1;
            transmitIntBuf[3] = value.un.rawAccelerometer.x; 
            transmitIntBuf[4] = value.un.rawAccelerometer.y;  
            transmitIntBuf[5] = value.un.rawAccelerometer.z;
            break;
        }      
        case SH2_MAGNETIC_FIELD_CALIBRATED:
        {
            got_mag = 1;
            magAccuracy = (sensor_event.report[2] & 0x03);
            break;
        }
    }
        
     //if(got_gyroRV || got_accel)
    if(got_gyroRV && got_accel)
    {         
        //Outputting transmit Buf
        #ifdef  USBUART_MODE
        #ifdef DATA_OUTPUT_MODE    
        USBUART_PutData(( void *)transmitBuf,40); //transmit orientation, acceleration, angular velocity.
        //USBUART_PutData(( void *)transmitIntBuf,12); //transmit raw gyro
        #endif
        print10(transmitBuf);//,transmitIntBuf,gameAccuracy, accelAccuracy, gyroAccuracy);
        
        //Check accuracy of each enabled sensor
        if(accelAccuracy >= 2) //note: gyroIntegratedRV  doesn't have an accuracy bit
        {
            accCount ++;
        }
        else
        {
            cal = false;        //If the accuracy of enabled sensors is not higher than 2, accCount will reset and calibration starts again.
            accCount = 0;       //Red LED indicates that calibration is finished.
            LED_R_Write(0);
        }
        #endif
        
        //Reset flags
        got_accel = false;      //Reset flags every time we output. So we will obtain new values from each sensor.
        got_gyroRV = false;
//        got_gyro = false;     //Uncomment if corresponding sensor is enabled. And revise if statement conditionals for output.
//        got_rot = false;
//        got_rawGyro = false;
//        got_rawAccel = false;
        
//        got_linAccel = false;
        toggleFiringPin();
    }
    return;
}

//Not sure what this is for, not used.
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

//During the initialization process, enable calibrations for these sensor fields.
static int enableCal(bool calAccel, bool calGyro, bool calMag)
{
    volatile int status;
    volatile int statusReturn;
    uint8_t enabled = 0x0;
    uint8_t a = 0, g = 0, m = 0;
    if (calAccel)
        a = SH2_CAL_ACCEL;
    if (calGyro)
        g = SH2_CAL_GYRO;
    if(calMag)
        m = SH2_CAL_MAG;    
    status = sh2_setCalConfig(a | g | m);
    statusReturn = sh2_getCalConfig(&enabled);
    return status & statusReturn;
}

static void EcheckCal()  //Once all accuracy bits of each sensor field is satisfactory, for 1000 cycles, save the configuration and dynamic calibration is finished.
{
    if(accCount >= 1000 && cal != true) //accCount is generated by checking if the accuracy bits of the corresponding enabled sensors are an optimal value.
        {
            cal = true;
            LED_R_Write(1);
            printOut("IMU calibrated\r\n");
            sh2_saveDcdNow();  
        }
}

//Phase functions:
/* Order of Operations for IMU to work
    1. If USBUART enabled, start our USBUART
    2. Initialize non IMU associated hardware (LEDs, Buzzer)
    3. Initialize HAL object and functions
    4. Open our Sh2 object which sets up parameters from HAL to Sh2, and SHTP. 
    5. Enable our sensors and configure them for communication through reports. (with setSensorCallback, and start_reports)
       Now we can use SH2 functions to communicate with IMU. (functs such as Sh2_getProdID())
    6. Repeatedly call Sh2_service which calls the HAL read function to repeatedly obtain data from IMU.
       Embedded system unable to print floats so need to convert them into s strings.
    7. After enough HAL Reads, we have a sensor callback function that runs to output the IMU data. sensor callback is called within the service function.
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
    ENABLE_3V_PIN_Write(1);     //Physically pull the hardware pin port 0, pin 4 high to enable the 3.3V voltage regulator.
    
    LED_R_Write(1);             //Initialization LED_R signals IMU initialization in process. (Also if calibration is done)
    
    printOut("****IMU INITIALIZATION START****\r\n");
    IMU_setup();                 //Initialize IMU using SH2 HAL
    start_reports();             //Enable our sensors and configure their output reports.
    //status = reportProdIds();  //Simple Sh2 function to get a product ID. Used in debugging to verify HAL read and write.
    bool calAccel = 1, calGyro = 1, calMag = 1; //choose which axes to calibrate. We want all 9 axes calibrated
    status = enableCal(calAccel, calGyro, calMag);  //enable calibration for the corresponding axes. Calibration is done
                                                    //via a physical orientation of the IMU.
    while(status != 0)                              //If status does not return zero, there is an additional problem with the calibration.
    {}   
    /****LOOP****/
    LED_R_Write(0);             //This LED ensures that our IMU Initialization is done, otherwise there may be an issue with a serial connection or another thing.           
    
    //IMU initialization is finished at this point.
    for(;;)
    {
        
        checkCal(); //Check if calibration of enabled sensors has occurred.
        sh2_service(); //periodic call to read in new IMU data
        
    } //End For loop
        
} //end Main

    
/* [] END OF FILE */

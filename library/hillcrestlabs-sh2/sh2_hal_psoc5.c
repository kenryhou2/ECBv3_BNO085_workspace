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
#include "sh2_hal.h"
#include "sh2_err.h"

#define ADDR_SH2_0          0x4A    //I2C IMU slave device address
#define STARTUP_DELAY_MS    2000
#define READ_LEN            2       //Number of bytes to read to determine length

//Debugging HAL read versions
#define HAL_READ_V2_MODE

//Debugging Constants
uint16 interruptval;                //Polling interrupt input pin value
                                    //Note: Currently two ways of detecting if interrupt pin is Low from IMU: Polling and Interrupt.
static char str [64];

//Enumerated Constants for State of I2C Bus
enum BusState_e {
    BUS_INIT,
    BUS_IDLE,
    BUS_READING_LEN,
    BUS_GOT_LEN,
    BUS_READING_TRANSFER,
    BUS_GOT_TRANSFER,
    BUS_WRITING_MSG,
    BUS_READING_DFU,
    BUS_WRITING_DFU,
    BUS_ERR,
};

enum BusState_e I2C_bus_state;
static uint8_t attention_needed = 0;

// Receive Buffer
static uint8_t rx_buf[SH2_HAL_MAX_TRANSFER_IN];      // data receive buffer from IMU, max length 384
static uint32_t rx_buf_len = 0;                      // valid bytes stored in rxBuf (0 when buf empty)
static uint16_t payload_len = 0;                         //Stores the end result length of a transfer between IMU and PSoC

// Transmit buffer
static uint8_t tx_buf[SH2_HAL_MAX_TRANSFER_OUT];     //data transmit buffer to IMU, max length 256
static uint32_t discards = 0;
static uint8_t IS_OPEN = 0;
static uint8_t IMU_READY = 0;

//Timer constants
static uint32_t current_time = 0;
volatile uint32_t rx_timestamp_us;            // timestamp of INTN event

extern int printOut(char str[64]);

/* Time Keeping Code 
   The timer keeps track of time with respect to microseconds between its start and stop.
   How the timer works:
   - Linked to the PSoC Bus Clock, the timer has a counter and an interrupt.
   - The counter decrements at a specified rate from a value called a period. (Hence the time for the counter to reach from start to 0 is a period)
   - Then, when the value of the counter reaches 0, it activates an interrupt, tc, and resets the counter to the period value and decrements again.
   - Within our code, we keep track of the total time from the Timer start by adding to the current_time the value of the period whenever the interrupt fires.
   - We also can get a timestamp in microseconds by outputting the value of the current time and the value in the counter.
*/
CY_ISR(TIMER_ISR_HANDLER){
    //Every time the timer completes, add to the time counter
    current_time += TIMER_US_ReadPeriod();
}

uint32_t get_timestamp()
{
    uint32_t raw_timer_total_time = current_time + (uint32_t)TIMER_US_ReadCounter();
    return (raw_timer_total_time/24); //Dependent on the BUS CLK speed.
    // the raw_timer_total_time value is based on clock speed of the Timer. Conversion: 24 raw clocks roughly equal to one us.
}

CY_ISR(IMU_ISR_HANDLER)
{
    IMU_READY = 1;
    //printOut("IMU_Interrupt_Triggered!!!\r\n"); //For debug purposes only
    attention_needed = 1;
    IMU_INT_ClearInterrupt();
}

static void imu_i2c_stop(void){
    IMU_I2C_Stop();
}

//For ECBv3, we don't need to set the BOOT pin to I2C bc it is hardwired.
//static void imu_set_boot(uint8_t val){
//    IMU_BOOT_Write(val);
//}

static void imu_disable_ints(void){
    //IMU_ISR_Disable();
    IMU_ISR_INT_Disable();
}

static void imu_enable_ints(void){
    //IMU_ISR_Enable();
    IMU_ISR_INT_Enable();
}

uint16_t imu_reset(){ //Henry: custom function to initialize the hardware
    printOut("IMU Hardware Resetting // Pin pulled LOW\r\n");
//    int val = IMU_RST_Read();
//    sprintf(str,"IMU_RST before reset: %d\r\n",val);
//    printOut(str);
    IMU_RST_Write(0); 
    CyDelay(50);
//    val = IMU_RST_Read();
//    sprintf(str,"IMU_RST during reset: %d\r\n",val);
//    printOut(str);
    IMU_RST_Write(1);
    CyDelay(50);
//    val = IMU_RST_Read();
//    sprintf(str,"IMU_RST after: %d\r\n",val);
//    printOut(str);
    uint16_t count = 0;
    //When the IMU is not asking for 
    //attention and one sec hasn't passed, increment count.
    //Exits while loop when IMU asks for attention or count has passed 1000.
    //We can replace attention_needed with IMU_INT_Read() == 0.
    while((IMU_INT_Read() == 1) && count < 1000) //Set a timeout of one second, record amt of time 
    {
        count++;
        CyDelay(1);
    }
    //sprintf(str,"reset count: %d\r\n",count);
    //printOut(str);
    return count;
}

void imu_handshake() //for debugging
{
    //Handshake??
    
    //imu_read(...) is just a hard read from the I2C bus.
    //imu_get_message is SHTP based, with length considered.handshake_buffer_len is the buffer read length by this imu read...
        
    //SH2 get advertisement
    /*Operation:
    * MCU writes reset command along with SHTP header from the int array 'data' to the IMU
    * MCU reads reset response along with SHTP header from the int array 'buff' from the IMU
    * Result is the error status of the read/write. 
    * If result is equal to error, while perform the write/read again.
    */
    uint8_t data[6] = {0x06,0x00,0x02,0x00,0xF9,0x00};
    int buffSize = 292;
    uint8_t buff[buffSize]; //buff[4];
    for(int i = 0; i < buffSize; i++)
    {
        //data[i] = 1;
        buff[i] = 0;
    }
    
    uint32_t result = 0;
            	
    CyDelay(500);
    //writing software reset command to the IMU.
    do{
        result = IMU_I2C_MasterWriteBuf(0x4A, data, 6, IMU_I2C_MODE_COMPLETE_XFER); //not sure why we only write the first four bytes... the header only..
        while (0u == (IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_WR_CMPLT))
        {
            
        }
    }while(result != IMU_I2C_MSTR_NO_ERROR);
    
    //reading in an ad packet
	do
    {
	IMU_I2C_MasterReadBuf(0x4A, buff, buffSize, IMU_I2C_MODE_COMPLETE_XFER);
		while (0u == (IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT)) //bitwise &.. triggers means master_status is 0. 0 & 1 = 0
		{
			//if stuck, means masterStatus is 0... probably some error.           
		}
	}
    while(result != IMU_I2C_MSTR_NO_ERROR);
    
    interruptval = IMU_INT_Read();
    sprintf(str,"interrupt value after read: %d\r\n",interruptval);
    printOut(str);
    //reads one... therefore the interrupt deasserts after the write and read.

    printOut("buff after read: \r\n");
    for(int i = 0; i < buffSize; i++)
    {
        sprintf(str, "byte %d: %02x\r\n",i,buff[i]);
        printOut(str);
    }
    interruptval = IMU_INT_Read();
    sprintf(str,"interrupt value after a while: %d\r\n",interruptval);
    printOut(str);
    
    printOut("IMU get ad done\r\n");
    
}

int poll_IMU_INT() //For debugging
{
    int intval = IMU_INT_Read();  
    sprintf(str,"interrupt value: %d\r\n",interruptval);
    printOut(str);
    return intval;
}

void test_TimerUs() //For debugging
{
    printOut("Test Timer\r\n");
    volatile uint32_t time1 = 0;
    volatile uint32_t time2 = 0;
    CyDelay(10);
    time1 = get_timestamp();    
    CyDelayUs(1000); //~24,000 clocks for 1 millisecond ~ 1000 microsecond (us). 
                     //~23.8, 24 clocks for one microsecond.
    time2 = get_timestamp(); 
    sprintf(str,"time1: %u time2: %u\r\n",time1,time2);
    printOut(str);
}

static int sh2_i2c_hal_open(sh2_Hal_t *self){
    
    //Init flags 
    IMU_READY = 0; //Indicates IMU is not ready to communicate. Tricky: Not a true status of the interrupt pin. We abstract it so that we will read IMU when we are ready
    attention_needed = 0;
    I2C_bus_state = BUS_INIT;
        
    //Initialize and Enable Comms to IMU with I2C bus
    IMU_I2C_Start();
    printOut("IMU I2C Initialized\r\n");
    
    //Init IMU Interrupts
    IMU_ISR_INT_StartEx(IMU_ISR_HANDLER); //Install the IMU interrupt handler.  
    printOut("IMU Interrupt Initialized\r\n");
    
    //Initialize Timer 
    TIMER_US_Start();
    printOut("Timer Initialized\r\n");
  
    //Initialize Timer tc interrupt
    STAMP_ISR_StartEx(TIMER_ISR_HANDLER);
    printOut("Timer TC Interrupt Initialized\r\n");
    
    CyDelay(STARTUP_DELAY_MS); //Wait for components to be ready for STARTUP_DELAY_MS
   
    //imu_handshake();
    
    //Hardware reset the IMU RST pin
    uint16_t time_to_response = imu_reset();
    
    
    //If we haven't timed out, return SH2_OK               
    if(time_to_response < 1000)
    {
        I2C_bus_state = BUS_IDLE;
        IMU_READY = 1;
        printOut("***INITIALIZATION SUCCESSFUL***\r\n");
        return SH2_OK;
    }
    else
    {
        printOut("***INITIALIZATION ERROR***\r\n");
        return SH2_ERR;
    }
}

static void sh2_i2c_hal_close(sh2_Hal_t *self){
    //Reset the IMU
    imu_reset();
    
    I2C_bus_state = BUS_INIT;
    
    //Stop the API
    imu_i2c_stop();
    
    //Mark that the port is closed
    IS_OPEN = 0;
}

#ifdef HAL_READ_V1_MODE
static int sh2_i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t)
{
    int retval = 0;
    
    imu_disable_ints();
    
    if(rx_buf_len > 0){
        if(rx_buf_len > len){
            retval = 0;
            return retval;
        }
        
        memcpy(pBuffer, rx_buf, payload_len);
        retval = rx_buf_len;
        rx_buf_len = 0;
    } else if (RX_DATA_READY){
        retval = imu_get_message();
        
        
        if(rx_buf_len > len){
            retval = 0;
            return retval;
        }
        
        memcpy(pBuffer, rx_buf, rx_buf_len);
        rx_buf_len = 0;
        RX_DATA_READY = 0;
    } else {
        retval = 0;
    }
    
    imu_enable_ints();
    *t = get_timestamp();
    
    return retval;
}
#endif

 //Read function STM inspired. Took out returns so entire read function runs and added delays after the i2c reads.
#ifdef HAL_READ_VSTM_MODE
static int sh2_i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t){
    if(I2C_bus_state == BUS_IDLE && attention_needed)
    {
        uint32_t result = IMU_I2C_MasterReadBuf(ADDR_SH2_0, rx_buf, READ_LEN,IMU_I2C_MODE_COMPLETE_XFER);
        CyDelayUs(1000); //Delay Needs to be here so I2C can successfully finish transferring
        if(result == IMU_I2C_MSTR_NO_ERROR)
        {
            I2C_bus_state = BUS_READING_LEN;
            printOut("BUS_READING_LEN\r\n");
        }
        else
            I2C_bus_state = BUS_ERR;
        //return 0;
    }
    
    if(I2C_bus_state == BUS_READING_LEN)
    {
        
        if((IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT)) // originally  bitwise &
        {
            I2C_bus_state = BUS_GOT_LEN;
            printOut("BUS_GOT_LEN\r\n");
            uint16_t len = (rx_buf[0] + (rx_buf[1] << 8)) & ~0x8000;    //Converts the SHTP length field from bytes to an int
                                                                        //Then compares the length in two cases
            if(len > SH2_HAL_MAX_TRANSFER_IN)                           //Case 1: Length of cargo is larger than max transfer size
            {                                                               //if so: create payload length max transfer size
                payload_len = SH2_HAL_MAX_TRANSFER_IN;                  
            } 
            else                                                        //Case 2: Length cargo is smaller or equal to max transfer size
            {                                                               //if so: payload length is equal to that length
                payload_len = len;
            }
            
            sprintf(str,"Payload length: %u\r\n",payload_len);
            printOut(str);
            
            uint32_t result = IMU_I2C_MasterReadBuf(ADDR_SH2_0, rx_buf, payload_len, IMU_I2C_MODE_COMPLETE_XFER);    //read in the total message now.
            CyDelayUs(2000); //Delay Needs to be here so I2C can successfully finish transferring
            //uint32_t result = IMU_I2C_MasterReadBuf(ADDR_SH2_0, pBuffer, payload_len, IMU_I2C_MODE_COMPLETE_XFER);
            if(result == IMU_I2C_MSTR_NO_ERROR)
            {
                I2C_bus_state = BUS_READING_TRANSFER;
                printOut("BUS_READING_TRANSFER\r\n");
            }
            else
                I2C_bus_state = BUS_ERR;
        }
        //return 0;
    }    
    if (I2C_bus_state == BUS_READING_TRANSFER) 
    {
        //If the read is successful, copy the memory into the buffer
        CyDelayUs(2000); //this delay needs to be here.
        if((IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT))
        {
            I2C_bus_state = BUS_GOT_TRANSFER;
            
            memcpy(pBuffer, rx_buf, payload_len);
            CyDelay(100);
            *t = get_timestamp();
            //sprintf(str,"time: %u\r\n",*t);
            //printOut(str);
            I2C_bus_state = BUS_IDLE;
            printOut("TRANSFER_COMPLETE\r\n");
        }
    } 
    int seq = pBuffer[3];
    sprintf(str,"sequence read in: %d\r\n",seq);
    printOut(str);
    return payload_len;
}
#endif

/*
    do{
        result = IMU_I2C_MasterReadBuf(ADDR_SH2_0,rx_buf, SH2_HAL_MAX_TRANSFER_IN , IMU_I2C_MODE_COMPLETE_XFER);
        while (0u == (IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT))
        {
            CyDelayUs(1);
        }
    }while(result != IMU_I2C_MSTR_NO_ERROR);


*/

 //Original Read Function
#ifdef HAL_READ_V2_MODE
static int sh2_i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t){
    if(I2C_bus_state == BUS_IDLE && attention_needed)
    {
        uint32_t result;
        result = IMU_I2C_MasterReadBuf(ADDR_SH2_0, rx_buf, READ_LEN,IMU_I2C_MODE_COMPLETE_XFER);
        CyDelay(10);
//        do{
//            
//            while (0u == (IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT))
//            {
//                CyDelayUs(1);
//            }
//        }while(result != IMU_I2C_MSTR_NO_ERROR);
        
        if(result == IMU_I2C_MSTR_NO_ERROR)
        {
            I2C_bus_state = BUS_READING_LEN;
            //printOut("BUS_READING_LEN\r\n");
        }
        else
            I2C_bus_state = BUS_ERR;
        return 0;
    }
    
    else if(I2C_bus_state == BUS_READING_LEN)
    {
        if((IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT))
        {
            I2C_bus_state = BUS_GOT_LEN;
            //printOut("BUS_GOT_LEN\r\n");
            volatile uint16_t len = (rx_buf[0] + (rx_buf[1] << 8)) & ~0x8000;    //Converts the SHTP length field from bytes to an int
                                                                        //Then compares the length in two cases
//            if(len > SH2_HAL_MAX_TRANSFER_IN)                           //Case 1: Length of cargo is larger than max transfer size
//            {                                                               //if so: create payload length max transfer size
//                payload_len = SH2_HAL_MAX_TRANSFER_IN;                  
//            } 
//            else                                                        //Case 2: Length cargo is smaller or equal to max transfer size
//            {                                                               //if so: payload length is equal to that length
//                payload_len = len;
//            }
//            
//            sprintf(str,"Payload length: %u\r\n",payload_len);
//            printOut(str);
//            
//            uint32_t result; 
//            
//            result = IMU_I2C_MasterReadBuf(ADDR_SH2_0, rx_buf, payload_len, IMU_I2C_MODE_COMPLETE_XFER); 
//            CyDelay(100); //Delay needs to be in here to let I2C finish transfer.
//            do{
//                result = IMU_I2C_MasterReadBuf(ADDR_SH2_0, rx_buf, payload_len, IMU_I2C_MODE_COMPLETE_XFER);    //read in the total message now.
//                while (0u == (IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT))
//                {
//                    CyDelayUs(1);
//                }
//            }while(result != IMU_I2C_MSTR_NO_ERROR);
            
            //len checks
            volatile uint32_t result;
            if(len <= 0xFF) //CASE 1: Length of cargo is less than max HAL transfer size and less than the I2C max transfer size. 
            {
                do
                {
                    result = IMU_I2C_MasterReadBuf(ADDR_SH2_0,rx_buf, (uint8_t)len, IMU_I2C_MODE_COMPLETE_XFER);    //read in the total message now.
                    while (0u == (IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT))
                    {
                        CyDelayUs(1);
                    }
                }while(result != IMU_I2C_MSTR_NO_ERROR);
                payload_len = len;
            }
            
            else        //CASE 2: Length of cargo is greater than I2C max transfer size.  Has two sub cases
                        //(Subcase 1: where length is greater than both I2C max and HAL max transfer sizes)
                        //(Subcase 2: where length is greater than I2C max but less than HAL max transfer size)
            {
                if(len > SH2_HAL_MAX_TRANSFER_IN)       //If the length of message is more than max HAL transfer, limit to max HAL transfer
                    len = SH2_HAL_MAX_TRANSFER_IN;      //At this point, worst case message would be a msg size 384 bytes.
                uint16_t r = len - 0xFF;                //if the length of message is less than max HAL transfer, but more than max I2C transfer, make a rem_len to read in the remaining bytes after the initial max I2C read.
                do
                {
                    result = IMU_I2C_MasterReadBuf(ADDR_SH2_0,rx_buf, 0xFF, IMU_I2C_MODE_COMPLETE_XFER);    //read in the total message now.
                    while (0u == (IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT))
                    {
                        CyDelayUs(1);
                    }
                }while(result != IMU_I2C_MSTR_NO_ERROR);
                
                uint8_t rem_len = (uint8_t)r;           //This is the raw remaining values we want. Will add to it 4 bytes for the header in the I2C read
                uint8_t temp_buf[256] = {0};            //Store the remaining bytes of the message into temp_buf, but recall that each I2C read from the IMU is prefixed with 4 byte SHTP header.
                
                do
                {
                    result = IMU_I2C_MasterReadBuf(ADDR_SH2_0,temp_buf, rem_len+0x04,IMU_I2C_MODE_COMPLETE_XFER);    //read in the rest of the message now and place into tempbuf. Watch out for that header! Put in a 4 byte offset.
                    while (0u == (IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT))
                    {
                        CyDelayUs(1);
                    }
                }while(result != IMU_I2C_MSTR_NO_ERROR);

                memcpy(rx_buf + 0xFF,temp_buf + 0x04, rem_len); //copying the remainder transfer from temp_buf without the 4 byte header into rx_buf
                payload_len = 0xFF + rem_len; //Reset the payload_len to the length entirety of the two I2C reads
            }//end else
            
            if(result == IMU_I2C_MSTR_NO_ERROR)
            {
                I2C_bus_state = BUS_READING_TRANSFER;
                //printOut("BUS_READING_TRANSFER\r\n");
            }
            else
                I2C_bus_state = BUS_ERR;
        }
        return 0;
    }    
    else if (I2C_bus_state == BUS_READING_TRANSFER) 
    {
        //If the read is successful, copy the memory into the buffer
        if((IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT))
        {
            I2C_bus_state = BUS_GOT_TRANSFER;
            
            memcpy(pBuffer, rx_buf, payload_len);
            *t = get_timestamp();
            //sprintf(str,"time: %u\r\n",*t);
            //printOut(str);
            I2C_bus_state = BUS_IDLE;
            //printOut("TRANSFER_COMPLETE\r\n");
            //int seq = pBuffer[3];
            //sprintf(str,"sequence read in: %d\r\n",seq);
            //printOut(str);
            return payload_len;
        }
    } 
    
    return 0;
}
#endif

static int sh2_i2c_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    printOut("Writing msg\r\n");
    if ((pBuffer == 0) || (len == 0) || (len > SH2_HAL_MAX_TRANSFER_OUT)) //checking for invalid write lengths.
    {
        return SH2_ERR_BAD_PARAM;
    }
    
    if(I2C_bus_state == BUS_IDLE)
    {
        I2C_bus_state = BUS_WRITING_MSG;
        
        memcpy(tx_buf, pBuffer, len);
        IMU_I2C_MasterWriteBuf(ADDR_SH2_0, tx_buf, len, IMU_I2C_MODE_COMPLETE_XFER);
        CyDelayUs(1000);
        uint16_t ct = 0;
        while (0u == (IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_WR_CMPLT))
        {
            CyDelay(1);
            ct++;
            if(ct > 1000) break;
        }
        
        I2C_bus_state = BUS_IDLE;
        
        if(ct > 1000) 
            return 0;        
    }
    return len;
}

static sh2_Hal_t sh2Hal;

sh2_Hal_t *sh2_hal_init(void)
{
    //Initializing all the member functions of the SH2 struct that will deal with all things IMU.
    sh2Hal.open = sh2_i2c_hal_open;     
    sh2Hal.close = sh2_i2c_hal_close;
    sh2Hal.read = sh2_i2c_hal_read;  //This has to complete under ADVERT_TIMEOUT_US in order to obtain a successful advertisemment packet read.
    sh2Hal.write = sh2_i2c_hal_write;
    sh2Hal.getTimeUs = get_timestamp; //has to be in units microseconds (us)
    return &sh2Hal;
}


/* [] END OF FILE */

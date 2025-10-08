/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright ...
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   main.c
 *
 * Project:
 * --------
 *   OpenCPU
 *
 * Description:
 * ------------
 *   This app demonstrates how to send AT command with RIL API, and transparently
 *   transfer the response through MAIN UART. And how to use UART port.
 *   Developer can program the application based on this example.
 * 
 ****************************************************************************/
#ifdef __CUSTOMER_CODE__


#include "custom_feature_def.h"
#include "ril.h"
#include "ril_util.h"
#include "ril_telephony.h"
#include "ql_stdlib.h"
#include "ql_error.h"
#include "ql_trace.h"
#include "ql_uart.h"
#include "ql_system.h"
#include "ql_iic.h"
#include "ql_type.h"
#include "ql_gpio.h"
#include "ql_spi.h"
#include "ql_eint.h"
#include "ql_fs.h"
#include "ril_network.h"
#include "ql_timer.h"
#include "ril_gps.h"

#include "oled.h"
#include "icm42670.h"
#include "sht40.h"
#include "spi_flash_driver.h"
#include <stdint.h>
#include <stdbool.h>

//--------------------------------------------------------------------------------------------------------------------------------//
#define DEBUG_ENABLE 1
#if DEBUG_ENABLE > 0
#define DEBUG_PORT  UART_PORT1
#define DBG_BUF_LEN   512
static char DBG_BUFFER[DBG_BUF_LEN];
#define APP_DEBUG(FORMAT,...) do {\
    Ql_memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    Ql_sprintf(DBG_BUFFER,FORMAT,##__VA_ARGS__); \
    Ql_UART_Write((Enum_SerialPort)(DEBUG_PORT), (u8*)(DBG_BUFFER), Ql_strlen((const char *)(DBG_BUFFER)));\
} while(0)
#else
#define APP_DEBUG(FORMAT,...) 
#endif

#define SERIAL_RX_BUFFER_LEN  2048
static u8 m_RxBuf_Uart1[SERIAL_RX_BUFFER_LEN]; // *** FIX: moved from local to global
static Enum_SerialPort uart_debug  = UART_PORT1;

// Pre‑allocated buffers for OLED handlers
static char bufOLED[64];       // *** FIX: global buffer
static char simStatusStr[32];  // *** FIX: global buffer

static int semICM  = -1;
static int semSHT  = -1;
static int semSIM = -1;
static int semGNSS = -1;
static u32 I2C_MUTEX=0;
static char gnssResponse[256];
//------------------------------------------------------------------------------------------------------------------------------------//

static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara);
static s32 ATResponse_Handler(char* line, u32 len, void* userData);
static void Callback_Timer(u32 timerId, void* param);
static void Timer_Init(void);
static int parse_rmc_coordinate(const char *rmc,
                                long *lonInt, long *latInt,
                                long *lonFrac, long *latFrac);
static long parse_simple_coord(const char *field);
static const char* skipComma(const char *p, unsigned commas);

//------------------------------------------------------------------------------------------------------------------------------------//
void proc_main_task(s32 taskId)
{
    s32 ret;
    ST_MSG msg;
    Enum_SIMState simSate = 0;
    u32 rssi = 0;
    
    I2C_MUTEX =  Ql_OS_CreateMutex("I2CMUTEX");
    semICM    =  Ql_OS_CreateSemaphore("semICM", 0);
    semSHT    =  Ql_OS_CreateSemaphore("semSHT", 0);
    semSIM    =  Ql_OS_CreateSemaphore("semSIM", 0);
    semGNSS   =  Ql_OS_CreateSemaphore("semGNSS", 0);
    
    Timer_Init();
    Ql_UART_Register(uart_debug, CallBack_UART_Hdlr, NULL);
    Ql_UART_Open(uart_debug, 115200, FC_NONE);

    Ql_IIC_Init(0, PINNAME_RI, PINNAME_DCD, 0);
    Ql_SPI_Init(1, PINNAME_PCM_IN, PINNAME_PCM_SYNC, PINNAME_PCM_OUT, PINNAME_PCM_CLK, 1);
    Ql_SPI_Config(1, TRUE, 0, 0, 1000);

    APP_DEBUG("OpenCPU: EiCut Demo App\r\n");


    // Start message loop of this task
    while (TRUE)
    {
        // Retrieve a message from the message queue of this task. 
        // This task will pend here if no message in the message queue, till a new message arrives.
        Ql_OS_GetMessage(&msg);

        // Handle the received message
        switch (msg.message)
        {
        case MSG_ID_RIL_READY:
            APP_DEBUG("<-- RIL is ready -->\r\n");
            Ql_RIL_Initialize();
            // After RIL init, RIL APIs from SDK\ril\inc can be used
            break;

        case MSG_ID_URC_INDICATION:
            // URC messages include init state, CFUN, SIM state, GSM net, GPRS net, calls, SMS, voltage, etc.
            switch (msg.param1)
            {
            case URC_SYS_INIT_STATE_IND:
                APP_DEBUG("<-- Sys Init Status %d -->\r\n", msg.param2);
                if (SYS_STATE_SMSOK == msg.param2)
                    APP_DEBUG("<-- Application can program SMS -->\r\n");
                break;

            case URC_SIM_CARD_STATE_IND:
                APP_DEBUG("<-- SIM Card Status:%d -->\r\n", msg.param2);
                if (SIM_STAT_READY == msg.param2)
                    APP_DEBUG("<-- SIM card is ready -->\r\n");
                else
                    APP_DEBUG("<-- SIM card is not available, cause:%d -->\r\n", msg.param2);
                break;

            case URC_GSM_NW_STATE_IND:
                APP_DEBUG("<-- GSM Network Status:%d -->\r\n", msg.param2);
                if (NW_STAT_REGISTERED == msg.param2 || NW_STAT_REGISTERED_ROAMING == msg.param2)
                    APP_DEBUG("<-- Module has registered to GSM network -->\r\n");
                break;

            case URC_GPRS_NW_STATE_IND:
                APP_DEBUG("<-- GPRS Network Status:%d -->\r\n", msg.param2);
                if (NW_STAT_REGISTERED == msg.param2 || NW_STAT_REGISTERED_ROAMING == msg.param2)
                {
                    APP_DEBUG("<-- Module has registered to GPRS network -->\r\n");
                }
                else
                {
                    if (NW_STAT_NOT_REGISTERED == msg.param2)
                    {// GPRS drops down
                        u32 rssi;
                        u32 ber;
                        s32 nRet = RIL_NW_GetSignalQuality(&rssi, &ber);
                        APP_DEBUG("<-- Signal strength:%d, BER:%d -->\r\n", rssi, ber);
                        
                    }
                }
                break;

            case URC_CFUN_STATE_IND:
                APP_DEBUG("<-- CFUN Status:%d -->\r\n", msg.param2);
                break;

            case URC_COMING_CALL_IND:
            {
                ST_ComingCall* pComingCall = (ST_ComingCall*)msg.param2;
                APP_DEBUG("<-- Coming call, number:%s, type:%d -->\r\n",
                        pComingCall->phoneNumber, pComingCall->type);
            }
            break;

            case URC_CALL_STATE_IND:
                APP_DEBUG("<-- Call state:%d -->\r\n", msg.param2);
                break;

            case URC_NEW_SMS_IND:
                APP_DEBUG("<-- New SMS Arrives: index=%d\r\n", msg.param2);
                break;

            case URC_MODULE_VOLTAGE_IND:
                APP_DEBUG("<-- VBatt Voltage Ind: type=%d -->\r\n", msg.param2);
                break;

            default:
                APP_DEBUG("<-- Other URC: type=%d -->\r\n", msg.param1);
                break;
            }
            break;

        default:
            break;
        }
    }
}

static s32 ReadSerialPort(Enum_SerialPort port, u8* pBuffer, u32 bufLen)
{
    s32 rdLen = 0, rdTotalLen = 0;
    if (!pBuffer || bufLen==0) return -1;
    Ql_memset(pBuffer, 0, bufLen);
    while (1)
    {
        rdLen = Ql_UART_Read(port, pBuffer + rdTotalLen, bufLen - rdTotalLen);
        if (rdLen <= 0) break;
        rdTotalLen += rdLen;
    }
    if (rdLen < 0) return -99;
    return rdTotalLen;
}

static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara)
{
    switch (msg)
    {
    case EVENT_UART_READY_TO_READ:
        if (uart_debug == port)
        {
            s32 totalBytes = ReadSerialPort(port, m_RxBuf_Uart1, sizeof(m_RxBuf_Uart1));
            if (totalBytes > 0)
            {
                char* pCh = Ql_strstr((char*)m_RxBuf_Uart1, "\r\n");
                if (pCh) { *pCh = '\0'; *(pCh + 1) = '\0'; }
                if (Ql_strlen((char*)m_RxBuf_Uart1) > 0)
                {
                    Ql_RIL_SendATCmd((char*)m_RxBuf_Uart1, totalBytes, ATResponse_Handler, NULL, 0);
                }
            }
        }
        break;
    default:
        break;
    }
}

static s32 ATResponse_Handler(char* line, u32 len, void* userData)
{
    char* responseBuf = (char*)userData;
    Ql_UART_Write(uart_debug, (u8*)line, len);
    if (Ql_RIL_FindLine(line, len, "OK")) return RIL_ATRSP_SUCCESS;
    if (Ql_RIL_FindLine(line, len, "ERROR")) return RIL_ATRSP_FAILED;
    if (Ql_RIL_FindString(line, len, "+CME ERROR")) return RIL_ATRSP_FAILED;
    if (Ql_RIL_FindString(line, len, "+CMS ERROR:")) return RIL_ATRSP_FAILED;
    if (Ql_RIL_FindString(line, len, "+QGNSSRD:")) {
        char *start = strstr(line, "$");
        if (start && responseBuf) {
            strncpy(responseBuf, start, 255);
            responseBuf[255] = '\0';
        }
    }
    return RIL_ATRSP_CONTINUE;
}

//-------------------------------------------------------OLED-Display-----------------------------------------------------------------//
static void oled_init_hw(void) {
    s32 ret = Ql_IIC_Config(OLED_CHANNEL, TRUE, OLED_I2C_ADDR, OLED_I2C_TIMEOUT);
    oledInit();
}

static void handle_sim(u32 sig, Enum_SIMState simStatus) {
    switch (simStatus) {
        case SIM_STAT_NOT_INSERTED:  Ql_sprintf(simStatusStr, "No SIM");         break;
        case SIM_STAT_READY:         Ql_sprintf(simStatusStr, "Ready");          break;
        case SIM_STAT_PIN_REQ:       Ql_sprintf(simStatusStr, "PIN");            break; //Need PIN
        case SIM_STAT_PUK_REQ:       Ql_sprintf(simStatusStr, "PUK");            break; //Need PUK
        case SIM_STAT_PH_PIN_REQ:    Ql_sprintf(simStatusStr, "PIN Req");        break; //Phone PIN Req
        case SIM_STAT_PH_PUK_REQ:    Ql_sprintf(simStatusStr, "PUK Req");        break; //Phone PUK Req
        case SIM_STAT_PIN2_REQ:      Ql_sprintf(simStatusStr, "Need PIN2");      break;
        case SIM_STAT_PUK2_REQ:      Ql_sprintf(simStatusStr, "Need PUK2");      break;
        case SIM_STAT_BUSY:          Ql_sprintf(simStatusStr, "Busy");           break;
        case SIM_STAT_NOT_READY:     Ql_sprintf(simStatusStr, "NotReady");       break;
        case SIM_STAT_UNSPECIFIED:   Ql_sprintf(simStatusStr, "Unspecified");    break;
        default:                     Ql_sprintf(simStatusStr, "Unknown");        break;
    }
    setCursor(0, 0);
    oledPrint("EICUT.COM");
    setCursor(0, 2);
    Ql_sprintf(bufOLED, "Signal:%lu     \nSIM:\n%s     ", sig, simStatusStr);
    oledPrint(bufOLED);
}

static void dispatch_message(const ST_MSG *msg) {
    switch (msg->message) {
        case 1: {
            setCursor(0, 0);
            oledPrint("Temp&Hum:");
            setCursor(0, 2);
            Ql_sprintf(bufOLED,"%luC \n%lu%%  ", msg->param1,msg->param2);
            oledPrint(bufOLED);
            Ql_OS_GiveSemaphore(semSHT);
            } break;
        case 2: {
            s32 ax = (s16)(msg->param1 >> 16);
            s32 ay = (s16)(msg->param1 & 0xFFFF);
            setCursor(0, 0);
            oledPrint("ACC:");
            setCursor(0, 2);
            Ql_sprintf(bufOLED,"X:%d   \nY:%d  \nZ:%lu    ", ax, ay, msg->param2);
            oledPrint(bufOLED);
            Ql_OS_GiveSemaphore(semICM);
        } break;
        case 3: {
            s32 gx = (s16)(msg->param1 >> 16);
            s32 gy = (s16)(msg->param1 & 0xFFFF);
            setCursor(0, 0);
            oledPrint("GYRO:");
            setCursor(0, 2);
            Ql_sprintf(bufOLED,"X:%d   \nY:%d  \nZ:%lu    ", gx, gy, msg->param2);
            oledPrint(bufOLED);
            Ql_OS_GiveSemaphore(semICM);
        } break;
        case 4: {
            handle_sim(msg->param1, msg->param2); 
        }
        break;
        case 5:{
            long lonInt = (msg->param1 >> 16) & 0xFFFF;
            long lonFrac = msg->param1 & 0xFFFF;
            long latInt = (msg->param2 >> 16) & 0xFFFF;
            long latFrac = msg->param2 & 0xFFFF;
            Ql_sprintf(bufOLED,"longitude:\n%ld.%04ld   \nlatitude:\n%ld.%04ld   ",
                    lonInt, lonFrac, latInt, latFrac);
            oledPrint(bufOLED);
        }break;
        default: oledPrint("No Data To Show"); break;
    }
}

void proc_oled(s32 taskId) {
    ST_MSG msg;
    uint8_t page = 0; // 1=Temp, 2=Acc+Gyro, 3=SIM

    Ql_OS_TakeMutex(I2C_MUTEX);
    oled_init_hw();
    oledFill();
    oledClear();
    setFont(Adafruit5x7);
    set2X();
    oledPrint("EICUT.COM");
    
    Ql_OS_GiveMutex(I2C_MUTEX);

    while (1) {
        Ql_OS_GetMessage(&msg);

        if (msg.message == 100) { // page change
            page = (page + 1) % 5; // now 0..4
            APP_DEBUG("Page number:%d\r\n",page);
            oledClear();
            switch (page) {
                case 0: Ql_OS_GiveSemaphore(semSHT); break; // Temp/Hum
                case 1: Ql_OS_GiveSemaphore(semICM); break; // Gyro
                case 2: Ql_OS_GiveSemaphore(semICM); break; // Acc
                case 3: Ql_OS_GiveSemaphore(semSIM); break; //Simcard
                case 4: Ql_OS_GiveSemaphore(semGNSS); break; //GNSS
            }
            continue;
        }

        bool pageMatch = TRUE;
        switch (page) {
            case 0: pageMatch = (msg.message == 1);  break; // Temp/Hum
            case 1: pageMatch = (msg.message == 2);  break; // Acc
            case 2: pageMatch = (msg.message == 3);  break; // Gyro
            case 3: pageMatch = (msg.message == 4);  break; // SIM
            case 4: pageMatch = (msg.message == 5);  break; 
        }

        if (pageMatch) {
            Ql_OS_TakeMutex(I2C_MUTEX);
            setFont(Adafruit5x7);
            set2X();
            dispatch_message(&msg);
            Ql_OS_GiveMutex(I2C_MUTEX);
        }
        Ql_Sleep(50);
    }
}

//--------------------------------------------------------Gyro & ACC----------------------------------------------------------------//
// Each sensor task: take mutex only during sensor read, release before sleep.
void proc_ICM42670(s32 taskId) {
    if (icm_init() != QL_RET_OK) return;
    while (1) {
        Ql_OS_TakeSemaphore(semICM, TRUE);

        Ql_OS_TakeMutex(I2C_MUTEX);
        uint8_t whoami; icm_read(REG_WHO_AM_I, &whoami, 1);
        int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
        icm_read_accel(&ax, &ay, &az);
        icm_read_gyro(&gx, &gy, &gz);
        Ql_OS_GiveMutex(I2C_MUTEX);

        uint32_t packedA = ((uint32_t)(uint16_t)ax << 16) | (uint16_t)ay;
        Ql_OS_SendMessage(OLED_id, 2, packedA, (uint32_t)(uint16_t)az);
        uint32_t packedB = ((uint32_t)(uint16_t)gx << 16) | (uint16_t)gy;
        Ql_OS_SendMessage(OLED_id, 3, packedB, (uint32_t)(uint16_t)gz);

        Ql_Sleep(10);
    }
}
//--------------------------------------------------------Temp & Hum----------------------------------------------------------------//
void proc_SHT4x(s32 taskId) {
    if (sht4x_init(SHT4X_CHNNL_NO, SHT4X_SLAVE_ADDR) != QL_RET_OK) return;

    static u32 last_temp = 0;
    static u32 last_hum  = 0;

    while (1) {
        Ql_OS_TakeSemaphore(semSHT, TRUE);
        Ql_OS_TakeMutex(I2C_MUTEX);

        #define NUM_SAMPLES 2
        sht4x_data_t samples[NUM_SAMPLES];
        u32 final_temp =0;
        u32 final_hum=0;

        // Read several samples
        for (int i = 0; i < NUM_SAMPLES; i++) {
            sht4x_measure(SHT4X_CHNNL_NO, SHT4X_SLAVE_ADDR, &samples[i]);
            Ql_Sleep(5);
        }

        Ql_OS_GiveMutex(I2C_MUTEX);

        // Calculate average temperature/humidity
        double avg_temp = 0, avg_hum = 0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            avg_temp += samples[i].temperature_c;
            avg_hum  += samples[i].humidity_pct;
        }
        avg_temp /= NUM_SAMPLES;
        avg_hum  /= NUM_SAMPLES;

        // Simple outlier rejection
        int valid_count = 0;
        double temp_sum = 0, hum_sum = 0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            if (fabs(samples[i].temperature_c - avg_temp) <= 2.0 &&
                fabs(samples[i].humidity_pct - avg_hum) <= 5.0) {
                temp_sum += samples[i].temperature_c;
                hum_sum  += samples[i].humidity_pct;
                valid_count++;
            }
        }

        if (valid_count > 0) {
            final_temp = (u32)(temp_sum / valid_count);
            final_hum  = (u32)(hum_sum  / valid_count);

            // update cache
            last_temp = final_temp;
            last_hum  = final_hum;

            Ql_OS_SendMessage(OLED_id, 1, final_temp, final_hum);
        }
        else {
            // send last known valid reading to prevent blocking
            Ql_OS_SendMessage(OLED_id, 1, last_temp, last_hum);
        }

        Ql_Sleep(50);
    }
}

//--------------------------------------------------------Simcard--------------------------------------------------------------//
void proc_simcard(s32 taskId) {
    u32 rssi;
    u32 ber;
    char strImsi[30]= {0};
    s32 simState=0;
    while (1) {
        Ql_OS_TakeSemaphore(semSIM, TRUE);
        RIL_NW_GetSignalQuality(&rssi, &ber);
        RIL_SIM_GetSimState(&simState);
        RIL_SIM_GetIMSI(strImsi);
        APP_DEBUG("<-- Signal strength:%d, Simcard:%d, IMSI:%s -->\r\n", rssi, simState,strImsi);
        Ql_OS_SendMessage(OLED_id, 4, rssi, simState);
        Ql_Sleep(150);
    }
}

//--------------------------------------------------------GNSS--------------------------------------------------------------//
void proc_gnss(s32 taskId) {
    s32 ret;
    u16 atLength=0;
    u8  readBuff[200];
    u32 lon=0;
    u32 lat=0;
    u32 lonFraction=0;
    u32 latFraction=0;
    char strAT[50] = {"\0"};
    while (1) {
        Ql_OS_TakeSemaphore(semGNSS, TRUE);
        atLength = Ql_sprintf(strAT, "AT+QGNSSC=1");
	    ret = Ql_RIL_SendATCmd(strAT, atLength, ATResponse_Handler, readBuff, 0);
        atLength = Ql_sprintf(strAT, "AT+QGNSSRD=\"NMEA/RMC\"");
	    ret = Ql_RIL_SendATCmd(strAT, atLength, ATResponse_Handler, readBuff, 0);
        APP_DEBUG("GNSSTask:%s\r\n",readBuff);
        // Parse coordinates from RMC data
        if (parse_rmc_coordinate(readBuff, &lon, &lat, &lonFraction, &latFraction ) == 0) {
        } else {
            APP_DEBUG("Failed to parse coordinates\r\n");
            lat = 0;
            lon = 0;
            lonFraction=0;
            latFraction=0;
        }
        Ql_OS_SendMessage(OLED_id, 5, ((lon << 16) | lonFraction),
                                    ((lat << 16) | latFraction));
        APP_DEBUG("Parsed - Lat:%ld.%ld, Lon:%ld.%ld\r\n",
                        lat, latFraction, lon, lonFraction);
        Ql_Sleep(150);
    }
}

/**
 * GPRMC message fields
1	UTC of position fix
2	Status A=active or V=void
3	Latitude
4	Longitude
5	Speed over the ground in knots
6	Track angle in degrees (True)
7	Date
8	Magnetic variation, in degrees
9	The checksum data, always begins with *
 */
static const char* skipComma(const char *p, unsigned commas)
{
    unsigned count = 0;
    while (*p) {
        if (*p == ',') {
            count++;
            if (count == commas) {
                return p + 1; // return pointer after desired comma
            }
        }
        p++;
    }
    return NULL; // not enough commas found
}


static int parse_coord_parts(const char *field, long *intPart, long *fracPart)
{
    *intPart = 0;
    *fracPart = 0;
    int after_dot = 0;

    while (*field && *field != ',' && *field != '\r' && *field != '\n') {
        if (*field == '.') {
            after_dot = 1;
            field++;
            continue;
        }
        if (*field >= '0' && *field <= '9') {
            if (!after_dot) {
                *intPart = (*intPart) * 10 + (*field - '0');
            } else {
                *fracPart = (*fracPart) * 10 + (*field - '0');
            }
        }
        field++;
    }
    return 0;
}
// Parse coordinate into integer and fractional parts separately
// Returns 0 on success, -1 on error
// Minimal parse with integer + fractional return
static int parse_rmc_coordinate(const char *rmc,
                                long *lonInt, long *latInt,
                                long *lonFrac, long *latFrac)
{
    const char *latStr = skipComma(rmc, 3);
    const char *lonStr = skipComma(rmc, 5);
    if (!latStr || !lonStr) return -1;

    // Latitude parts
    if (parse_coord_parts(latStr, latInt, latFrac) != 0) return -1;
    // Longitude parts
    if (parse_coord_parts(lonStr, lonInt, lonFrac) != 0) return -1;

    return 0;
}
//--------------------------------------------------------FLASH----------------------------------------------------------------//
/**
 * @brief: This function is used for writing in flash IC and read back to check it is working
 */
void proc_flash(s32 taskId) {
    const u8 flashCh = 1;   // SPI channel
    static u8 txBuf[FLASH_TX_BUF_SIZE];
    static u8 rxBuf[FLASH_RX_BUF_SIZE];
    u8 id[3];
    u8 readBuf[6] = {0};    // extra byte for debug-friendly null termination

    // --- Read chip ID ---
    spi_flash_read_jedec_id(flashCh, id, txBuf, rxBuf);
    APP_DEBUG("JEDEC ID: %02X %02X %02X\r\n", id[0], id[1], id[2]);

    // --- Read existing 5 bytes ---
    spi_flash_read_data(flashCh, 0x000000, readBuf, 5, txBuf, rxBuf);
    APP_DEBUG("Flash content: %.*s\r\n", 5, readBuf);

    // --- Compare and update if needed ---
    if (Ql_memcmp(readBuf, "EiCut", 5) != 0) {
        APP_DEBUG("Mismatch — erasing and programming...\r\n");

        // Enable write, erase
        spi_flash_write_enable(flashCh, txBuf);
        spi_flash_sector_erase(flashCh, 0x000000, txBuf, rxBuf);

        // Enable write, program
        spi_flash_write_enable(flashCh, txBuf);
        spi_flash_page_program(flashCh, 0x000000, (const u8 *)"EiCut", 5, txBuf, rxBuf);

        // Verify
        Ql_memset(readBuf, 0, sizeof(readBuf));
        spi_flash_read_data(flashCh, 0x000000, readBuf, 5, txBuf, rxBuf);
        APP_DEBUG("Updated flash content: %.*s\r\n", 5, readBuf);
    } else {
        APP_DEBUG("No update needed! flash already contains EiCut\r\n");
    }

    while(1){
      Ql_Sleep(450);  
    }
}

//-------------------------------------------------------Other Tasks & Interrupts------------------------------------------------//

//Callback function.
static void Callback_Timer(u32 timerId, void* param)
{
     Ql_OS_SendMessage(OLED_id, 100, 0, 0); // signal OLED to change page
}

static void Timer_Init(){
    s32 ret_local;
    static u32 timerId=999; //Timer ID is 999
    u32 interval=3 * 1000; //3 seconds
    bool autoRepeat=TRUE;
    u32 param=555;
    //Register the timer.
    ret_local=Ql_Timer_Register(timerId, Callback_Timer, &param);
    Ql_Debug_Trace("\r\n<--Register: timerId=%d, param=%d,ret=%d -->\r\n", timerId ,param,ret_local);
    //Start the timer.
    ret_local=Ql_Timer_Start(timerId, interval, autoRepeat);
    Ql_Debug_Trace("\r\n<--Start: timerId=%d,repeat=%d,ret=%d -->\r\n", timerId , autoRepeat,ret_local);
}

#endif // __CUSTOMER_CODE__


#include <string.h>
#include <stdio.h>
#include <time.h>
#include <api_os.h>
#include <api_gps.h>
#include <api_event.h>
#include <api_hal_uart.h>
#include <api_debug.h>
#include "buffer.h"
#include "gps_parse.h"
#include "math.h"
#include "gps.h"
#include "api_hal_pm.h"
#include "time.h"
#include "api_info.h"
#include "assert.h"
#include "api_socket.h"
#include "api_network.h"

/**
 *  GPS_TRACKER на базе платы разрабочика A9/A9G development board без чипа LIS3DHx (Акселерометр 3-осевой).
 * Использует open source сервер Traccar.org, реализован протокол обмена данных t55.
 * ДОЛЖНО БЫТЬ ИЗМЕНЕНО: 
 *      #define SERVER_IP   "ss.neucrack.com" - имя/IP адрес Вашего сервера.
 *      #define SERVER_PORT  5005             - порт сервера для протокола обмена данных t55.
 *      #define TRACKER_ID   234707           - индентификатор трекера.
 *          
 * @attention Данный код демонстрирует как можно использовать плату A9/A9G, автор не несет ответственности 
 * за возможные проблемы. This code demonstrates how you can use the Board A9/A9G, the author is not 
 * responsible for possible problems.
 * 
 * Andrey Emelyanov
 * aemelyanov757@gmail.com
 */
#define SERVER_IP    "XX.XX.XX.XX"  // IP адрес сервера или доменное имя
#define SERVER_PORT  5005           // Порт сервера для данных с трекера
#define TRACKER_ID   234707         // ID номер трекера

//#define GPS_NMEA_LOG_FILE_PATH "/t/gps_nmea.log"



#define MAIN_TASK_STACK_SIZE    (2048 * 2)
#define MAIN_TASK_PRIORITY      0
#define MAIN_TASK_NAME          "GPS Test Task"

static HANDLE gpsTaskHandle = NULL;
bool isGpsOn = true;
bool networkFlag = false;

uint8_t buffer[1024];   // общий буфер для UART1, Trace, LOG_FILE
uint8_t buffer2[512];   // буфер для HTTP_request

int loop_counter = -1;  // счетчик циклов определения координат GPS, для отправки отчета, начальное состояние -1!

// const uint8_t nmea[]="$GNGGA,000021.263,2228.7216,N,11345.5625,E,0,0,,153.3,M,-3.3,M,,*4E\r\n$GPGSA,A,1,,,,,,,,,,,,,,,*1E\r\n$BDGSA,A,1,,,,,,,,,,,,,,,*0F\r\n$GPGSV,1,1,00*79\r\n$BDGSV,1,1,00*68\r\n$GNRMC,000021.263,V,2228.7216,N,11345.5625,E,0.000,0.00,060180,,,N*5D\r\n$GNVTG,0.00,T,,M,0.000,N,0.000,K,N*2C\r\n";

void EventDispatch(API_Event_t* pEvent)
{
    switch(pEvent->id)
    {
        case API_EVENT_ID_NO_SIMCARD:
            Trace(10,"!!NO SIM CARD%d!!!!",pEvent->param1);
            networkFlag = false;
            break;
        case API_EVENT_ID_NETWORK_REGISTER_SEARCHING:
            Trace(2,"network register searching");
            networkFlag = false;
            break;
        case API_EVENT_ID_NETWORK_REGISTER_DENIED:
            Trace(2,"network register denied");
        case API_EVENT_ID_NETWORK_REGISTER_NO:
            Trace(2,"network register no");
            break;
        case API_EVENT_ID_GPS_UART_RECEIVED:
            // Trace(1,"received GPS data,length:%d, data:%s,flag:%d",pEvent->param1,pEvent->pParam1,flag);
            GPS_Update(pEvent->pParam1,pEvent->param1);
            break;
        case API_EVENT_ID_NETWORK_REGISTERED_HOME:
        case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
        {
            uint8_t status;
            Trace(2,"network register success");
            bool ret = Network_GetAttachStatus(&status);
            if(!ret)
                Trace(1,"get attach staus fail");
            Trace(1,"attach status:%d",status);
            if(status == 0)
            {
                ret = Network_StartAttach();
                if(!ret)
                {
                    Trace(1,"network attach fail");
                }
            }
            else
            {
                Network_PDP_Context_t context = {
                    .apn        ="cmnet",
                    .userName   = ""    ,
                    .userPasswd = ""
                };
                Network_StartActive(context);
            }
            break;
        }
        case API_EVENT_ID_NETWORK_ATTACHED:
            Trace(2,"network attach success");
            Network_PDP_Context_t context = {
                .apn        ="cmnet",
                .userName   = ""    ,
                .userPasswd = ""
            };
            Network_StartActive(context);
            break;

        case API_EVENT_ID_NETWORK_ACTIVATED:
            Trace(2,"network activate success");
            networkFlag = true;
            break;
        
        case API_EVENT_ID_UART_RECEIVED:
            if(pEvent->param1 == UART1)
            {
                uint8_t data[pEvent->param2+1];
                data[pEvent->param2] = 0;
                memcpy(data,pEvent->pParam1,pEvent->param2);
                Trace(1,"uart received data,length:%d,data:%s",pEvent->param2,data);
                if(strcmp(data,"close") == 0)
                {
                    Trace(1,"close gps");
                    GPS_Close();
                    isGpsOn = false;
                }
                else if(strcmp(data,"open") == 0)
                {
                    Trace(1,"open gps");
                    GPS_Open(NULL);
                    isGpsOn = true;
                }
            }
            break;
        default:
            break;
    }
}

//http post with no header
int Http_Post(const char* domain, int port,const char* path,uint8_t* body, uint16_t bodyLen, char* retBuffer, int bufferLen)
{
    uint8_t ip[16];
    bool flag = false;
    uint16_t recvLen = 0;

    //connect server
    memset(ip,0,sizeof(ip));
    if(DNS_GetHostByName2(domain,ip) != 0)
    {
        Trace(2,"get ip error");
        return -1;
    }
    // Trace(2,"get ip success:%s -> %s",domain,ip);
    char* servInetAddr = ip;
    char* temp = OS_Malloc(2048);
    if(!temp)
    {
        Trace(2,"malloc fail");
        return -1;
    }
    snprintf(temp,2048,"POST %s HTTP/1.1\r\nContent-Type: application/x-www-form-urlencoded\r\nConnection: Keep-Alive\r\nHost: %s\r\nContent-Length: %d\r\n\r\n",
                            path,domain,bodyLen);
    char* pData = temp;
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(fd < 0){
        Trace(2,"socket fail");
        OS_Free(temp);
        return -1;
    }
    // Trace(2,"fd:%d",fd);

    struct sockaddr_in sockaddr;
    memset(&sockaddr,0,sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(port);
    inet_pton(AF_INET,servInetAddr,&sockaddr.sin_addr);

    int ret = connect(fd, (struct sockaddr*)&sockaddr, sizeof(struct sockaddr_in));
    if(ret < 0){
        Trace(2,"socket connect fail");
        OS_Free(temp);
        return -1;
    }
    // Trace(2,"socket connect success");
    Trace(2,"send request:%s",pData);
    ret = send(fd, pData, strlen(pData), 0);
    if(ret < 0){
        Trace(2,"socket send fail");
        OS_Free(temp);
        return -1;
    }
    ret = send(fd, body, bodyLen, 0);
    if(ret < 0){
        Trace(2,"socket send fail");
        OS_Free(temp);
        return -1;
    }
    // Trace(2,"socket send success");

    struct fd_set fds;
    struct timeval timeout={12,0};
    FD_ZERO(&fds);
    FD_SET(fd,&fds);
    while(!flag)
    {
        ret = select(fd+1,&fds,NULL,NULL,&timeout);
        // Trace(2,"select return:%d",ret);
        switch(ret)
        {
            case -1:
                Trace(2,"select error");
                flag = true;
                break;
            case 0:
                Trace(2,"select timeout");
                flag = true;
                break;
            default:
                if(FD_ISSET(fd,&fds))
                {
                    memset(retBuffer,0,bufferLen);
                    ret = recv(fd,retBuffer,bufferLen,0);
                    recvLen += ret;
                    if(ret < 0)
                    {
                        Trace(2,"recv error");
                        flag = true;
                        break;
                    }
                    else if(ret == 0)
                    {
                        Trace(2,"ret == 0");
                        break;
                    }
                    else if(ret < 1352)
                    {
                        GPS_DEBUG_I("recv len:%d,data:%s",recvLen,retBuffer);
                        close(fd);
                        OS_Free(temp);
                        return recvLen;
                    }
                }
                break;
        }
    }
    close(fd);
    OS_Free(temp);
    return -1;
}

void gps_testTask(void *pData)
{
    GPS_Info_t* gpsInfo = Gps_GetInfo();
    

    //wait for gprs register complete
    //The process of GPRS registration network may cause the power supply voltage of GPS to drop,
    //which resulting in GPS restart.
    while(!networkFlag)
    {
        Trace(1,"wait for gprs regiter complete");
        OS_Sleep(2000);
    }

    //open GPS hardware(UART2 open either)
    GPS_Init();
    //GPS_SaveLog(true,GPS_NMEA_LOG_FILE_PATH);
    // if(!GPS_ClearLog())
    //     Trace(1,"open file error, please check tf card");
    GPS_Open(NULL);

    //wait for gps start up, or gps will not response command
    while(gpsInfo->rmc.latitude.value == 0)
        OS_Sleep(1000);
    

    // set gps nmea output interval
    for(uint8_t i = 0;i<5;++i)
    {
        bool ret = GPS_SetOutputInterval(10000);
        Trace(1,"set gps ret:%d",ret);
        if(ret)
            break;
        OS_Sleep(1000);
    }

    // if(!GPS_ClearInfoInFlash())
    //     Trace(1,"erase gps fail");
    
    // if(!GPS_SetQzssOutput(false))
    //     Trace(1,"enable qzss nmea output fail");

    // if(!GPS_SetSearchMode(true,false,true,false))
    //     Trace(1,"set search mode fail");

    // if(!GPS_SetSBASEnable(true))
    //     Trace(1,"enable sbas fail");
    
    if(!GPS_GetVersion(buffer,150))
        Trace(1,"get gps firmware version fail");
    else
        Trace(1,"gps firmware version:%s",buffer);

    // if(!GPS_SetFixMode(GPS_FIX_MODE_LOW_SPEED))
        // Trace(1,"set fix mode fail");

    if(!GPS_SetOutputInterval(1000))
        Trace(1,"set nmea output interval fail");
    
    Trace(1,"init ok");

    while(1)
    {
        if(isGpsOn)
        {
            //???=анализ текущих координат=???
            uint8_t isFixed = gpsInfo->gsa[0].fix_type > gpsInfo->gsa[1].fix_type ?gpsInfo->gsa[0].fix_type:gpsInfo->gsa[1].fix_type;
            char* isFixedStr;            
            if(isFixed == 2)
                isFixedStr = "2D fix";
            else if(isFixed == 3)
            {
                if(gpsInfo->gga.fix_quality == 1)
                    isFixedStr = "3D fix";
                else if(gpsInfo->gga.fix_quality == 2)
                    isFixedStr = "3D/DGPS fix";
            }
            else
                isFixedStr = "no fix";
            //???=анализ текущих координат=???

            // = СБОР ДАННЫХ С ТРЕКЕРА =
            // преобразование координат из unit [ddmm.mmmm] в градусы [degree(°)] 
            int temp = (int)(gpsInfo->rmc.latitude.value/gpsInfo->rmc.latitude.scale/100);
            double latitude = temp+(double)(gpsInfo->rmc.latitude.value - temp*gpsInfo->rmc.latitude.scale*100)/gpsInfo->rmc.latitude.scale/60.0;
            temp = (int)(gpsInfo->rmc.longitude.value/gpsInfo->rmc.longitude.scale/100);
            double longitude = temp+(double)(gpsInfo->rmc.longitude.value - temp*gpsInfo->rmc.longitude.scale*100)/gpsInfo->rmc.longitude.scale/60.0;
            // получение скорости и курса
            double speed = (double)(gpsInfo->rmc.speed.value/gpsInfo->rmc.speed.scale);
            double course = (double)(gpsInfo->rmc.course.value/gpsInfo->rmc.course.scale);
            double altitude = (double)(gpsInfo->gga.altitude.value/gpsInfo->gga.altitude.scale); 
            // получение даты по GPS
            int day = (int)(gpsInfo->rmc.date.day);
            int month = (int)(gpsInfo->rmc.date.month);
            int year = (int)(gpsInfo->rmc.date.year);
            int hours = (int)(gpsInfo->rmc.time.hours);
            int minutes = (int)(gpsInfo->rmc.time.minutes);
            int seconds = (int)(gpsInfo->rmc.time.seconds);
            // получение данных об энергообеспечении
            uint8_t batteryLevel; // batteryLevel - проценты!
            uint16_t milliVolt = PM_Voltage(&batteryLevel);// milliVolt - миливольты! 
            Trace(1,"power volt = %f, procent = %1.f",milliVolt/1000.0,batteryLevel*1.0);

            // формирование строки отчета для UART1
            //you can copy ` latitude,longitude ` to http://www.gpsspg.com/maps.htm check location on map
            snprintf(buffer,sizeof(buffer),"GPS fix mode:%d, BDS fix mode:%d, fix quality:%d, satellites tracked:%d, gps sates total:%d, is fixed:%s, coordinate:WGS84, Latitude:%f, Longitude:%f, unit:degree,altitude:%f",
                     gpsInfo->gsa[0].fix_type, gpsInfo->gsa[1].fix_type,gpsInfo->gga.fix_quality,gpsInfo->gga.satellites_tracked, 
                     gpsInfo->gsv[0].total_sats, isFixedStr, latitude,longitude,gpsInfo->gga.altitude);

            // показываем буфер в отладчике кода
            Trace(1,buffer);

            // отправляем буфер в UART1
            UART_Write(UART1,buffer,strlen(buffer));
            UART_Write(UART1,"\r\n\r\n",4);
            memset(buffer,0,sizeof(buffer)); // очистка буфера

            // подготовка данных для TCP/IP запроса
            char* requestPath = buffer2;

            //if(!INFO_GetIMEI(buffer)) // !!!=заменил на TRACKER_ID=!!!
            //    Assert(false,"NO IMEI");
            //Trace(1,"device name:%s",buffer);

            // Начало формирования строки с координатами по протоколу t55 ("old Traccar Client")
            // Формат запроса состоит из трех строк(предложений):
            // 1 строка запроса:
            // $PGID,123456789012345*0F\r\n (где: 123456789012345 - ID код трекера)
            // 2 строка запроса (стандартное предложение NMEA GPRMC):
            // $GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68\r\n
            // !!!=P/S: не могу найти сырые данные от модуля A9G. Должны быть в UART2, или void EventDispatch(API_Event_t* pEvent)=!!!
            // 3 строка запроса (не обязательная, дополнена Anton Tananaev(Traccar.org)):
            // $TRCCR,20140111000000.000,A,60.000000,60.000000,0.00,0.00,0.00,5.0,*3a\r\n
            // где:
            //      * 20140111000000.000 - дата и время 2014-01-11 00:00:00.000 UTC
            //          * 2014           - год, 4 знака
            //          * 01             - месяц, 2 знака
            //          * 11             - день, 2 знака
            //          * 00             - час, 2 знака
            //          * 00             - минута, 2 знака
            //          * 00             - секунда, 2 знака
            //          * .000           - микросекунды, 3 знака (!!!=в этой версии .000=!!!)
            //      * A или V - навигационное предупреждение GPS (!!!=в этой версии все данные валидны=!!!)
            //          * A              - All OK, данные валидны 
            //          * V              - предупреждение, данные не валидны
            //      * 60.000000 - широта в градусах (отрицательное для Юга), 6-знаков после запятой;
            //      * 60.000000 - долгота в градусах (отрицательное для Запада), 6-знаков после запятой;
            //      * 0.00 - скорость отностительно Земли в узлах, 2-знака после запятой;
            //      * 0.00 - курс в градусах, 2-знака после запятой;
            //      * 0.00 - высота над морем в метрах, 2-знака после запятой;
            //      * 5.0  - заряд батареи в процентах, 1-знак после запятой;
            //!!! Реализована передача данных вида:
            // $PGID,123456789012345*0F\r\n$TRCCR,20140111000000.000,A,60.000000,60.000000,0.00,0.00,0.00,5.0,*3a\r\n
            snprintf(requestPath,sizeof(buffer2),
                     "$PGID,%d*0F\r\n$TRCCR,%.4d%.2d%.2d%.2d%.2d%.2d.000,A,%.6f,%.6f,%.2f,%.2f,%.2f,%.1f,*3a\r\n",
                     TRACKER_ID,year,month,day,hours,minutes,seconds,latitude,longitude,speed,course,
                     altitude,batteryLevel*1.0);
            
            // А при такой реализации запроса, происходит ошибка: на сервер приходит неверное ВРЕМЯ
            //snprintf(requestPath,sizeof(buffer2),"/?id=%s&timestamp=%d&lat=%f&lon=%f&speed=%f&bearing=%.1f&altitude=%f&accuracy=%.1f&batt=%.1f",
            //         buffer,clock(NULL),latitude,longitude,isFixed*1.0,0.0,gpsInfo->gga.altitude,0.0,batteryLevel*1.0);
            
            // Цикл передачи сообщения:
            bool isDataSend = false; // Флаг отправки сообщения (false - "флаг опущен")
            // ЕСЛИ: начальное состояние счетчика циклов -1 или 120 - отчет оправить, обнулить счетчик циклов.
            if((loop_counter == -1)||(loop_counter > 119)){
                // отправка...
                if(Http_Post(SERVER_IP,SERVER_PORT,requestPath,NULL,0,buffer,sizeof(buffer)) < 0){
                    Trace(1,"send location to server fail");
                    loop_counter = -2; // "первый раз" - делаем безконечное кол-во попыток 
                    isDataSend = false;
                }else{
                    Trace(1,"send location to server success");
                    Trace(1,"response:%s",buffer);
                    isDataSend   = true;
                    loop_counter =  0;  // счетчик обнулен
                }
            }
            // ЕСЛИ: скорость более 3 узлов, и прошло 4 цикла, и "флаг опущен" - отчет оправить, обнулить счетчик циклов;
            if((speed > 3.0)&&(loop_counter > 3)&&(!isDataSend)){
                // отправка...
                if(Http_Post(SERVER_IP,SERVER_PORT,requestPath,NULL,0,buffer,sizeof(buffer)) < 0){
                    Trace(1,"send location to server fail");
                    isDataSend = false;
                }else{
                    Trace(1,"send location to server success");
                    Trace(1,"response:%s",buffer);
                    isDataSend   = true;
                    loop_counter =  0;  // счетчик обнулен
                }
            }
            // ЕСЛИ: скорость более 25 узлов, и прошло 2 цикла, и "флаг опущен" - отчет оправить, обнулить счетчик циклов;
            if((speed > 25.0)&&(loop_counter > 1)&&(!isDataSend)){
                // отправка...
                if(Http_Post(SERVER_IP,SERVER_PORT,requestPath,NULL,0,buffer,sizeof(buffer)) < 0){
                    Trace(1,"send location to server fail");
                    isDataSend = false;
                }else{
                    Trace(1,"send location to server success");
                    Trace(1,"response:%s",buffer);
                    isDataSend   = true;
                    loop_counter =  0;  // счетчик обнулен
                }
            }
            // ЕСЛИ: скорость более 50 узлов и "флаг опущен" - отчет оправить, обнулить счетчик циклов;
            if((speed > 50.0)&&(!isDataSend)){
                // отправка...
                if(Http_Post(SERVER_IP,SERVER_PORT,requestPath,NULL,0,buffer,sizeof(buffer)) < 0){
                    Trace(1,"send location to server fail");
                    isDataSend = false;
                }else{
                    Trace(1,"send location to server success");
                    Trace(1,"response:%s",buffer);
                    isDataSend   = true;
                    loop_counter =  0;  // счетчик обнулен
                }
            }
            // ЕСЛИ: не было отправки отчета - увеличиваем счетчик 
            if(!isDataSend) loop_counter++;
        }

        OS_Sleep(10000);    // СОН ЗАДАЧИ 10 секунд, но A9G в работе!
    }
}


void gps_MainTask(void *pData)
{
    API_Event_t* event=NULL;
    
    TIME_SetIsAutoUpdateRtcTime(true);
    
    //open UART1 to print NMEA infomation
    UART_Config_t config = {
        .baudRate = UART_BAUD_RATE_115200,
        .dataBits = UART_DATA_BITS_8,
        .stopBits = UART_STOP_BITS_1,
        .parity   = UART_PARITY_NONE,
        .rxCallback = NULL,
        .useEvent   = true
    };
    UART_Init(UART1,config);

    //Create UART1 send task and location print task
    OS_CreateTask(gps_testTask,
            NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);

    //Wait event
    while(1)
    {
        if(OS_WaitEvent(gpsTaskHandle, (void**)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            EventDispatch(event);
            OS_Free(event->pParam1);
            OS_Free(event->pParam2);
            OS_Free(event);
        }
    }
}


void gps_tracker_Main(void)
{
    gpsTaskHandle = OS_CreateTask(gps_MainTask,
        NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
    OS_SetUserMainHandle(&gpsTaskHandle);
}

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/I2CB1.h"
#include "../inc/CortexM.h"
#include "../inc/LPF.h"
#include "../inc/opt3101.h"
#include "../inc/LaunchPad.h"
#include "../inc/BumpInt.h"
#include "../inc/Motor.h"
#include "UART0.h"
#include "../inc/SSD1306.h"
#include "../inc/FFT.h"
#include "MQTTClient.h"
#include "simplelink.h"
#include "driverlib.h"
#include "board.h"
#include "sl_common.h"
#include "stdio.h"
#include "string.h"
#include "../inc/odometry.h"
#include "../inc/blinker.h"
#include "../inc/Tachometer.h"
#include "../inc/TimerA1.h"
#include "PID.h"
#include "string.h"
#include "stdio.h"

// Prototype functions
void collision(uint8_t);
void recover();
void publish_string(char*,char*);

//#define USEOLED 0
#define USEUART

#ifdef USENOKIA
// this batch configures for LCD
#include "../inc/Nokia5110.h"
#define Init Nokia5110_Init
#define Clear Nokia5110_Clear
#define SetCursor Nokia5110_SetCursor
#define OutString Nokia5110_OutString
#define OutChar Nokia5110_OutChar
#define OutUDec Nokia5110_OutUDec
#define OutSDec Nokia5110_OutSDec
#endif

#ifdef USEOLED
// this batch configures for OLED

void OLEDinit(void){SSD1306_Init(SSD1306_SWITCHCAPVCC);}
#define Init OLEDinit
#define Clear SSD1306_Clear
#define SetCursor SSD1306_SetCursor
#define OutChar SSD1306_OutChar
#define OutString SSD1306_OutString
#define OutUDec SSD1306_OutUDec
#define OutSDec SSD1306_OutSDec
#endif

#ifdef USEUART
// this batch configures for UART link to PC
#include "../inc/UART0.h"
void UartSetCur(uint8_t newX, uint8_t newY){
  if(newX == 6){
    UART0_OutString("\n\rTxChannel= ");
    UART0_OutUDec(newY-1);
    UART0_OutString(" Distance= ");
  }else{
    UART0_OutString("\n\r");
  }
}
void UartClear(void){UART0_OutString("\n\r");};
#define Init UART0_Init
#define Clear UartClear
#define SetCursor UartSetCur
#define OutString UART0_OutString
#define OutChar UART0_OutChar
#define OutUDec UART0_OutUDec
#define OutSDec UART0_OutSDec
#endif

/*
 * Values for below macros shall be modified per the access-point's (AP) properties
 * SimpleLink device will connect to following AP when the application is executed
 */
#define SSID_NAME       "ECE DESIGN LAB 2.4"       /* Access point name to connect to. */
#define SEC_TYPE        SL_SEC_TYPE_WPA_WPA2     /* Security type of the Access piont */
#define PASSKEY         "ecedesignlab12345"   /* Password in case of secure AP */
#define PASSKEY_LEN     pal_Strlen(PASSKEY)  /* Password length in case of secure AP */

/*
 * MQTT server and topic properties that shall be modified per application
 */
#define MQTT_BROKER_SERVER  "broker.hivemq.com"
#define PUBLISH_TOPIC_STATUS "status"
#define PUBLISH_TOPIC_RPM_Left "motor/left"
#define PUBLISH_TOPIC_RPM_Right "motor/right"
#define PUBLISH_TOPIC_X "odom/x"
#define PUBLISH_TOPIC_Y "odom/y"
#define PUBLISH_TOPIC_Angle "odom/angle"

// MQTT message buffer size
#define BUFF_SIZE 32

Network n;
Client hMQTTClient;     // MQTT Client

uint8_t isCrash = 0;

unsigned char macAddressVal[SL_MAC_ADDR_LEN];
unsigned char macAddressLen = SL_MAC_ADDR_LEN;

char macStr[18];        // Formatted MAC Address String
char uniqueID[9];       // Unique ID generated from TLV RAND NUM and MAC Address

#define SL_STOP_TIMEOUT        0xFF

#define SMALL_BUF           32
#define MAX_SEND_BUF_SIZE   512
#define MAX_SEND_RCV_SIZE   1024

/*
 * STATIC FUNCTION DEFINITIONS
 */
static _i32 establishConnectionWithAP();
static _i32 configureSimpleLinkToDefaultState();
static _i32 initializeAppVariables();
static void generateUniqueID();

/* Application specific status/error codes */
typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,        /* Choosing this number to avoid overlap with host-driver's error codes */
    HTTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    HTTP_RECV_ERROR = HTTP_SEND_ERROR - 1,
    HTTP_INVALID_RESPONSE = HTTP_RECV_ERROR -1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

_u32  g_Status = 0;
struct{
    _u8 Recvbuff[MAX_SEND_RCV_SIZE];
    _u8 SendBuff[MAX_SEND_BUF_SIZE];

    _u8 HostName[SMALL_BUF];
    _u8 CityName[SMALL_BUF];

    _u32 DestinationIP;
    _i16 SockID;
}g_AppData;

/*!
    \brief This function handles WLAN events

    \param[in]      pWlanEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
        CLI_Write(" [WLAN EVENT] NULL Pointer Error \n\r");

    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);

            /*
             * Information about the connected AP (like name, MAC etc) will be
             * available in 'slWlanConnectAsyncResponse_t' - Applications
             * can use it if required
             *
             * slWlanConnectAsyncResponse_t *pEventData = NULL;
             * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
             *
             */
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request, 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                CLI_Write(" Device disconnected from the AP on application's request \n\r");
            }
            else
            {
                CLI_Write(" Device disconnected from the AP on an ERROR..!! \n\r");
            }
        }
        break;

        default:
        {
            CLI_Write(" [WLAN EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles events for IP address acquisition via DHCP
           indication

    \param[in]      pNetAppEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
        CLI_Write(" [NETAPP EVENT] NULL Pointer Error \n\r");

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            /*
             * Information about the connected AP's IP, gateway, DNS etc
             * will be available in 'SlIpV4AcquiredAsync_t' - Applications
             * can use it if required
             *
             * SlIpV4AcquiredAsync_t *pEventData = NULL;
             * pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
             * <gateway_ip> = pEventData->gateway;
             *
             */
        }
        break;

        default:
        {
            CLI_Write(" [NETAPP EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles callback for the HTTP server events

    \param[in]      pHttpEvent - Contains the relevant event information
    \param[in]      pHttpResponse - Should be filled by the user with the
                    relevant response information

    \return         None

    \note

    \warning
*/
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    /*
     * This application doesn't work with HTTP server - Hence these
     * events are not handled here
     */
    CLI_Write(" [HTTP EVENT] Unexpected event \n\r");
}

/*!
    \brief This function handles general error events indication

    \param[in]      pDevEvent is the event passed to the handler

    \return         None
*/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /*
     * Most of the general errors are not FATAL are are to be handled
     * appropriately by the application
     */
    CLI_Write(" [GENERAL EVENT] \n\r");
}

/*!
    \brief This function handles socket events indication

    \param[in]      pSock is the event passed to the handler

    \return         None
*/
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
        CLI_Write(" [SOCK EVENT] NULL Pointer Error \n\r");

    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
        {
            /*
            * TX Failed
            *
            * Information about the socket descriptor and status will be
            * available in 'SlSockEventData_t' - Applications can use it if
            * required
            *
            * SlSockEventData_t *pEventData = NULL;
            * pEventData = & pSock->EventData;
            */
            switch( pSock->EventData.status )
            {
                case SL_ECLOSE:
                    CLI_Write(" [SOCK EVENT] Close socket operation failed to transmit all queued packets\n\r");
                break;


                default:
                    CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
                break;
            }
        }
        break;

        default:
            CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
        break;
    }
}

uint32_t Distances[3];
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t Noises[3];
uint32_t TxChannel;
uint32_t StartTime;
uint32_t TimeToConvert; // in msec
bool pollDistanceSensor(void){
  if(OPT3101_CheckDistanceSensor()){
    TxChannel = OPT3101_GetMeasurement(Distances,Amplitudes);
    return true;
  }
  return false;
}

// calibrated for 500mm track
// right is raw sensor data from right sensor
// return calibrated distance from center of Robot to right wall
int32_t Right(int32_t right){
  return  (right*(59*right + 7305) + 2348974)/32768;
}
// left is raw sensor data from left sensor
// return calibrated distance from center of Robot to left wall
int32_t Left(int32_t left){
  return (left*(59*left + 7305) + 2348974)/32768;
}

int32_t Mode=0; // 0 stop, 1 run
int32_t Error;
int32_t Ki=1000;  // integral controller gain
int32_t Kp=5;  // proportional controller gain //was 4

int32_t UR, UL;  // PWM duty 0 to 14,998
char command;


#define TOOCLOSE 200
#define DESIRED 250
int32_t SetPoint = 250;
int32_t LeftDistance,CenterDistance,RightDistance; // mm
#define TOOFAR 400 // was 400

#define PWMNOMINAL 7000 // was 2500
#define SWING 500 //was 1000
#define PWMMIN 1000
#define PWMMAX 7500
#define AVOIDSETPOINT 500

void Controller(void){ // runs at 100 Hz
  if(Mode){
    if((LeftDistance>DESIRED)&&(RightDistance>DESIRED)){
      SetPoint = (LeftDistance+RightDistance)/2;
    }
    else
        SetPoint = DESIRED;

    if(LeftDistance < RightDistance ){
      Error = LeftDistance-SetPoint;
    }
    else{
      Error = SetPoint-RightDistance;
    }

    UR = PWMNOMINAL+Kp*Error; // proportional control
    UR = UR + Ki*Error;      // adjust right motor
    UL = PWMNOMINAL-Kp*Error; // proportional control

    if(UR < (PWMNOMINAL-SWING)) UR = PWMMIN; // 3,000 to 7,000
    if(UR > (PWMNOMINAL+SWING)) UR = PWMMAX;
    if(UL < (PWMNOMINAL-SWING)) UL = PWMMIN; // 3,000 to 7,000
    if(UL > (PWMNOMINAL+SWING)) UL = PWMMAX;

    if((RightDistance < AVOIDSETPOINT) && (CenterDistance < AVOIDSETPOINT)){
        UL = 0;
        UR = PWMNOMINAL;
    }

    else if((LeftDistance < AVOIDSETPOINT) && (CenterDistance < AVOIDSETPOINT)){
        UR = 0;
        UL = PWMNOMINAL;
    }

    Motor_Forward(UL,UR);

  }
}
extern enum RobotState Action;

void main(void){
  int i = 0;
  uint32_t channel = 1;
  DisableInterrupts();
  Clock_Init48MHz();
  Motor_Init();
  BumpInt_Init(&collision);
  LaunchPad_Init(); // built-in switches and LEDs
  Motor_Stop(); // initialize and stop
  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
  UART0_Initprintf();
  Action = ISSTOPPED;
  Blinker_Init();
  Odometry_Init(0,0, NORTH);
  Tachometer_Init();
  TimerA1_Init(&UpdatePosition,20000); // every 40ms
  printf("\nHallway Racer\n\r");
  OPT3101_Init();
  OPT3101_Setup();
  OPT3101_CalibrateInternalCrosstalk();
  OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);
  TxChannel = 3;
  OPT3101_StartMeasurementChannel(channel);
  LPF_Init(100,8);
  LPF_Init2(100,8);
  LPF_Init3(100,8);
  UR = UL = PWMNOMINAL; //initial power
  EnableInterrupts();

  // ================ CONNECTION SETUP ================
  _i32 retVal = -1;

  retVal = configureSimpleLinkToDefaultState();

  if(retVal < 0)
  {
      if (DEVICE_NOT_IN_STATION_MODE == retVal)
          CLI_Write(" Failed to configure the device in its default state \n\r");

      LOOP_FOREVER();
  }

  CLI_Write(" Device is configured in default state \n\r");

  /*
   * Assumption is that the device is configured in station mode already
   * and it is in its default state
   */
  retVal = sl_Start(0, 0, 0);
  if ((retVal < 0) ||
      (ROLE_STA != retVal) )
  {
      CLI_Write(" Failed to start the device \n\r");
      LOOP_FOREVER();
  }

  CLI_Write(" Device started as STATION \n\r");

  /* Connecting to WLAN AP */
  retVal = establishConnectionWithAP();
  if(retVal < 0)
  {
      CLI_Write(" Failed to establish connection w/ an AP \n\r");
      LOOP_FOREVER();
  }

  CLI_Write(" Connection established w/ AP and IP is acquired \n\r");

  // ================ CONNECTION SETUP ================

  // ================ MQTT SETUP ======================
  // Obtain MAC Address
  sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&macAddressLen,(unsigned char *)macAddressVal);

  // Print MAC Addres to be formatted string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
          macAddressVal[0], macAddressVal[1], macAddressVal[2], macAddressVal[3], macAddressVal[4], macAddressVal[5]);

  // Generate 32bit unique ID from TLV Random Number and MAC Address
  generateUniqueID();

  int rc = 0;
  unsigned char buf[100];
  unsigned char readbuf[100];

  NewNetwork(&n);
  rc = ConnectNetwork(&n, MQTT_BROKER_SERVER, 1883);

  if (rc != 0) {
      CLI_Write(" Failed to connect to MQTT broker \n\r");
      LOOP_FOREVER();
  }
  CLI_Write(" Connected to MQTT broker \n\r");

  MQTTClient(&hMQTTClient, &n, 1000, buf, 100, readbuf, 100);
  MQTTPacket_connectData cdata = MQTTPacket_connectData_initializer;
  cdata.MQTTVersion = 3;
  cdata.clientID.cstring = uniqueID;
  rc = MQTTConnect(&hMQTTClient, &cdata);

  if (rc != 0) {
      printf(" Failed to start MQTT client \n\r");
      LOOP_FOREVER();
  }
  printf(" Started MQTT client successfully \n\r");
  // ================ MQTT SETUP ======================

  Mode = 1;

  while(1){



    command = UART0_InChar(); //calling UART0_InChar function

    Odometry_Init(0,0,NORTH);

    while(command == 'G') //if the input via Bluetooth is "G"
    {
        if(isCrash){
              isCrash = 0;
              publish_string(PUBLISH_TOPIC_STATUS, "Crashed");
              Clock_Delay1ms(1000);
          }

        publish_string(PUBLISH_TOPIC_STATUS, "Running");

        if (rc != 0) {
          printf(" Failed to publish collision to MQTT broker \n\r");
          LOOP_FOREVER();
        }
        printf(" Published collision successfully \n\r");
        command = UART0_InCharNoWait();
        if(command == 'S'){
            Motor_Stop();
            break;
        }
        command = 'G';

        if(TxChannel <= 2){ // 0,1,2 means new data
          if(TxChannel==0){
            if(Amplitudes[0] > 1000){
              LeftDistance = FilteredDistances[0] = Left(LPF_Calc(Distances[0]));
            }else{
              LeftDistance = FilteredDistances[0] = 500;
            }
          }else if(TxChannel==1){
            if(Amplitudes[1] > 1000){
              CenterDistance = FilteredDistances[1] = LPF_Calc2(Distances[1]);
            }else{
              CenterDistance = FilteredDistances[1] = 500;
            }
          }else {
            if(Amplitudes[2] > 1000){
              RightDistance = FilteredDistances[2] = Right(LPF_Calc3(Distances[2]));
            }else{
              RightDistance = FilteredDistances[2] = 500;
            }
          }
          printf("L Distance %d   R Distance %d\n\r", LeftDistance, RightDistance);
          printf("Center Distance %d\n\r", CenterDistance);

          TxChannel = 3; // 3 means no data
          channel = (channel+1)%3;
          OPT3101_StartMeasurementChannel(channel);
          i = i + 1;
        }
      else {
        if(Amplitudes[2] > 1000){
          RightDistance = FilteredDistances[2] = LPF_Calc3(Distances[2]);
        }else{
          RightDistance = FilteredDistances[2] = 500;
        }
      }
      //printf("L Distance %d   R Distance %d\n\r", LeftDistance, RightDistance);
      //printf("Center Distance %d\n\r", CenterDistance);

      TxChannel = 3; // 3 means no data
      channel = (channel+1)%3;
      OPT3101_StartMeasurementChannel(channel);
      i = i + 1;
    }

    Controller();

    //getting RPM
    uint16_t *Actual_RPM;
    int32_t Get_X, Get_Y, Get_Angle;
    char Actual_Left[5], Actual_Right[5], X[5], Y[5], Angle[5];
    getActual_Start();
    Actual_RPM = getActual_End();
    sprintf(Actual_Left, "%d", Actual_RPM[0]);
    sprintf(Actual_Right, "%d", Actual_RPM[1]);
    int rc0 = 0;
    int rc1 = 0;
    MQTTMessage msg0, msg1, msg_X, msg_Y, msg_Angle;
    msg0.dup = 0;
    msg0.id = 0;
    msg0.payload = Actual_Left;
    msg0.payloadlen = 4;
    msg0.qos = QOS0;
    msg0.retained = 0;
    rc0 = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_RPM_Left, &msg0);

    msg1.dup = 0;
    msg1.id = 0;
    msg1.payload = Actual_Right;
    msg1.payloadlen = 4;
    msg1.qos = QOS0;
    msg1.retained = 0;
    rc1 = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_RPM_Right, &msg1);

    int rc_X = 0;
    int rc_Y = 0;
    int rc_Angle = 0;

    Get_X = Odometry_GetX();
    Get_Y = Odometry_GetY();
    Get_Angle = Odometry_GetAngle();
    sprintf(X, "%d", Get_X);
    sprintf(Y, "%d", Get_Y);
    sprintf(Angle, "%d", Get_Angle);

    msg_X.dup = 0;
    msg_X.id = 0;
    msg_X.payload = X;
    msg_X.payloadlen = 4;
    msg_X.qos = QOS0;
    msg_X.retained = 0;
    rc_X = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_X, &msg_X);

    msg_Y.dup = 0;
    msg_Y.id = 0;
    msg_Y.payload = Y;
    msg_Y.payloadlen = 4;
    msg_Y.qos = QOS0;
    msg_Y.retained = 0;
    rc_Y = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_Y, &msg_Y);

    msg_Angle.dup = 0;
    msg_Angle.id = 0;
    msg_Angle.payload = Angle;
    msg_Angle.payloadlen = 4;
    msg_Angle.qos = QOS0;
    msg_Angle.retained = 0;
    rc_Angle = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_Angle, &msg_Angle);


    if(i >= 100)
        i = 0;

        WaitForInterrupt();
    }
  }
}

void publish_string(char* topic, char* payload){
    int rc = 0;
    MQTTMessage msg;
    msg.dup = 0;
    msg.id = 0;
    msg.payload = payload;
    msg.payloadlen = 8;
    msg.qos = QOS0;
    msg.retained = 0;
    rc = MQTTPublish(&hMQTTClient, topic, &msg);

    if (rc != 0) {
        printf(" Failed to publish collision to MQTT broker \n\r");
        LOOP_FOREVER();
    }
    printf(" Published collision successfully \n\r");
}

void recover(){
    Motor_Backward(3000,3000);
    Clock_Delay1ms(2000);
    Motor_Stop();
    Clock_Delay1ms(500);
}

void collision(uint8_t bump){
    switch(bump){
    case BIT0:
    case BIT2:
    case BIT3:
    case BIT5:
    case BIT6:
    case BIT7:
        isCrash = 1;
        Motor_Stop();
        Clock_Delay1ms(1000);
        recover();
        break;
    default:
        break;
    }
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

/*!
    \brief This function configure the SimpleLink device in its default state. It:
           - Sets the mode to STATION
           - Configures connection policy to Auto and AutoSmartConfig
           - Deletes all the stored profiles
           - Enables DHCP
           - Disables Scan policy
           - Sets Tx power to maximum
           - Sets power policy to normal
           - Unregisters mDNS services
           - Remove all filters

    \param[in]      none

    \return         On success, zero is returned. On error, negative is returned
*/
static _i32 configureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    _u8           val = 1;
    _u8           configOpt = 0;
    _u8           configLen = 0;
    _u8           power = 0;

    _i32          retVal = -1;
    _i32          mode = -1;

    mode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(mode);

    /* If the device is not in station-mode, try configuring it in station-mode */
    if (ROLE_STA != mode)
    {
        if (ROLE_AP == mode)
        {
            /* If the device is in AP mode, we need to wait for this event before doing anything */
            while(!IS_IP_ACQUIRED(g_Status)) { _SlNonOsMainLoopTask(); }
        }

        /* Switch to STA role and restart */
        retVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(retVal);

        /* Check if the device is in station again */
        if (ROLE_STA != retVal)
        {
            /* We don't want to proceed if the device is not coming up in station-mode */
            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
        }
    }

    /* Get the device's version-information */
    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));
    ASSERT_ON_ERROR(retVal);

    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove all profiles */
    retVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(retVal);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = sl_WlanDisconnect();
    if(0 == retVal)
    {
        /* Wait */
        while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }
    }

    /* Enable DHCP client*/
    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
    ASSERT_ON_ERROR(retVal);

    /* Disable scan */
    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
    power = 0;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
    ASSERT_ON_ERROR(retVal);

    /* Set PM policy to normal */
    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Unregister mDNS services */
    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove  all 64 filters (8*8) */
    pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(retVal);

    retVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(retVal);

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    return retVal; /* Success */
}

/*!
    \brief Connecting to a WLAN Access point

    This function connects to the required AP (SSID_NAME).
    The function will return once we are connected and have acquired IP address

    \param[in]  None

    \return     0 on success, negative error-code on error

    \note

    \warning    If the WLAN connection fails or we don't acquire an IP address,
                We will be stuck in this function forever.
*/
static _i32 establishConnectionWithAP()
{
    SlSecParams_t secParams = {0};
    _i32 retVal = 0;

    secParams.Key = PASSKEY;
    secParams.KeyLen = PASSKEY_LEN;
    secParams.Type = SEC_TYPE;

    retVal = sl_WlanConnect(SSID_NAME, pal_Strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(retVal);

    /* Wait */
    while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status))) { _SlNonOsMainLoopTask(); }

    return SUCCESS;
}

/*!
    \brief This function initializes the application variables

    \param[in]  None

    \return     0 on success, negative error-code on error
*/
static _i32 initializeAppVariables()
{
    g_Status = 0;
    pal_Memset(&g_AppData, 0, sizeof(g_AppData));

    return SUCCESS;
}


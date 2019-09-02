/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

//#define xdc_runtime_Log_DISABLE_ALL 1  // Add to disable logs from this file

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/drivers/PIN.h>
#include <ti/mw/display/Display.h>

#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>

// Stack headers
#include <hci_tl.h>
#include <gap.h>
#include <gatt.h>
#include <gapgattserver.h>
#include <gattservapp.h>
#include <osal_snv.h>
#include <gapbondmgr.h>
#include <peripheral.h>
#include <icall_apimsg.h>

#include <devinfoservice.h>

#include "util.h"

#include "Board.h"
#include "project_zero.h"

// Custom services
#include "bluetoothService.h"
#include "userInterface.h"

#include "scif.h"
#define BV(x) (1 << (x))

// Pin mapping for MUX Pin
#define MUX_EN                                IOID_5
#define MUX_A0                                IOID_6
#define MUX_A1                                IOID_7
#define MUX_A2                                IOID_23

/*********************************************************************
 * CONSTANTS
 */
// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Default pass-code used for pairing.
#define DEFAULT_PASSCODE                      000000

// Task configuration
#define PRZ_TASK_PRIORITY                     1

#ifndef PRZ_TASK_STACK_SIZE
#define PRZ_TASK_STACK_SIZE                   800
#endif

// Internal Events for RTOS application
#define PRZ_STATE_CHANGE_EVT                  0x0001
#define PRZ_CHAR_CHANGE_EVT                   0x0002
#define PRZ_PERIODIC_EVT                      0x0004
#define PRZ_CONN_EVT_END_EVT                  0x0008

// Unique IDs for SC tasks
#define RUN_CV_TASK                           0
#define RUN_CA_TASK                           1

#define WE0                                   0
#define WE1                                   1
#define WE2                                   2
#define WE3                                   3


/*********************************************************************
 * TYPEDEFS
 */
// Types of messages that can be sent to the user application task from other
// tasks or interrupts. Note: Messages from BLE Stack are sent differently.
typedef enum
{
  APP_MSG_GAP_STATE_CHANGE = 0,   /* A characteristic value has been written     */
  APP_MSG_SEND_PASSCODE,          /* A pass-code/PIN is requested during pairing */
  APP_MSG_SC_CTRL_READY,          /* SC Task initialization complete             */
  APP_MSG_SC_TASK_ALERT           /* SC Task generated an interrupt              */  
} app_msg_types_t;

// Struct for messages sent to the application task
typedef struct
{
  Queue_Elem       _elem;
  app_msg_types_t  type;
  uint8_t          pdu[];
} app_msg_t;

// Struct for messages about characteristic data
typedef struct
{
  uint16_t svcUUID; // UUID of the service
  uint16_t dataLen; //
  uint8_t  paramID; // Index of the characteristic
  uint8_t  data[];  // Flexible array member, extended to malloc - sizeof(.)
} char_data_t;

// Struct for messages from a service
typedef struct
{
  Queue_Elem _elem;
  uint16_t svcUUID;
  uint16_t dataLen;
  uint8_t paramID;
  uint8_t data[]; // Flexible array member, extended to malloc - sizeof(.)
} server_char_write_t;

// Struct for message about sending/requesting passcode from peer.
typedef struct
{
  uint16_t connHandle;
  uint8_t  uiInputs;
  uint8_t  uiOutputs;
  uint32   numComparison;
} passcode_req_t;

// Struct for message about button state
typedef struct
{
  PIN_Id   pinId;
  uint8_t  state;
} button_state_t;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Queue object used for application messages.
static Queue_Struct applicationMsgQ;
static Queue_Handle hApplicationMsgQ;

// Queue object used for service messages.
static Queue_Struct serviceMsgQ;        // SOLUTION
static Queue_Handle hServiceMsgQ;       // SOLUTION

// Task configuration
Task_Struct przTask;
Char przTaskStack[PRZ_TASK_STACK_SIZE];

// Pin driver handles
static PIN_Handle muxPinHandle;

// Global memory storage for a PIN_Config table
static PIN_State muxPinState;

// Initial MUX pin config table
PIN_Config muxPinTable[] = {
  MUX_EN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  MUX_A0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  MUX_A1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  MUX_A2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // No scan response data provided.
  0x00 // Placeholder to keep the compiler happy.
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) or general
  // discoverable mode (advertises indefinitely), depending
  // on the DEFAULT_DISCOVERY_MODE define.
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // complete name
  13,
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'P', 'r', 'o', 'j', 'e', 'c', 't', ' ', 'Z', 'e', 'r', 'o',

};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Project Zero";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Global display handle
Display_Handle dispHandle;

// Variables for SC configuration
uint16_t lmpMode = 0;
uint16_t measuring = 0;
uint16_t tiaGain = 0;
uint16_t weSelect = 0;
uint32_t rtc_Hz = 20;  // 20Hz RTC
uint16_t cvPeriod = 1;  // 1 step per second for CV
uint16_t caPulseLen = 1; // pulse length of 1 second

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void ProjectZero_init( void );
static void ProjectZero_taskFxn(UArg a0, UArg a1);

static void user_processApplicationMessage(app_msg_t *pMsg);
static uint8_t ProjectZero_processStackMsg(ICall_Hdr *pMsg);
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg);

static void ProjectZero_sendAttRsp(void);
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg);
static void ProjectZero_freeAttRsp(uint8_t status);

static void user_processGapStateChangeEvt(gaprole_States_t newState);
static void user_gapStateChangeCB(gaprole_States_t newState);
static void user_gapBondMgr_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                       uint8_t uiInputs, uint8_t uiOutputs, uint32 numComparison);
static void user_gapBondMgr_pairStateCB(uint16_t connHandle, uint8_t state,
                                        uint8_t status);


// Callback handlers for value changes in services.
static void user_userInterfaceValueChangeCB(uint8_t paramID); // Callback from the service. // SOLUTION

// Task context handlers for generated services.
static void user_userInterface_ValueChangeDispatchHandler(server_char_write_t* pWrite); // Local handler called from the Task context of this task. SOLUTION

// Task handler for sending notifications.

// Utility functions
static void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData, uint16_t len );

static char *Util_convertArrayToHexString(uint8_t const *src, uint8_t src_len,
                                          uint8_t *dst, uint8_t dst_len);
static char *Util_getLocalNameStr(const uint8_t *data);

// Sensor controller functions
static void scCtrlReadyCallback(void);
static void scTaskAlertCallback(void);
static void processTaskAlert(void);
static void processSampleADC(void);
static void processCV(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t user_gapRoleCBs =
{
  user_gapStateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t user_bondMgrCBs =
{
  user_gapBondMgr_passcodeCB, // Passcode callback
  user_gapBondMgr_pairStateCB // Pairing / Bonding state Callback
};

/*
 * Callbacks in the user application for events originating from BLE services.
 */
// userInterface Service callback function implementation
// userInterfaceService callback handler. The type userInterfaceCBs_t is defined in userInterface.h
static userInterfaceCBs_t user_userInterfaceCBs =
{
  user_userInterfaceValueChangeCB // Characteristic value change callback handler
};

// Sensor Controller callbacks
static void scCtrlReadyCallback(void)
{
  user_enqueueRawAppMsg(APP_MSG_SC_CTRL_READY, NULL, 0);
} // scCtrlReadyCallback

static void scTaskAlertCallback(void)
{
  user_enqueueRawAppMsg(APP_MSG_SC_TASK_ALERT, NULL, 0);
} // scTaskAlertCallback

static void processTaskAlert(void) {
  // Clear the ALERT interrupt source
  scifClearAlertIntSource();

  // decode interrupt source
    // Get the alert events
  uint32_t bvAlertEvents = scifGetAlertEvents();

  // Check which task called and do process
  if (bvAlertEvents & BV(SCIF_SAMPLE_ADC_TASK_ID)) {
      processSampleADC();
  }
  if (bvAlertEvents & BV(SCIF_CV_TASK_TASK_ID)) {
      processCV();
  }


      
  // Acknowledge the ALERT event
  scifAckAlertEvents();
} // processTaskAlert


//processCV()
static void processCV(void)
{
  //initialize all the state variable
  scifTaskData.cvTask.state.biasStep = 0;
  scifTaskData.cvTask.state.biasStepFlag = 0;
  scifTaskData.cvTask.state.shotDownCVtask = 0;
  //Kill all the task and then reconstruct the task
  uint32_t activeTasks = scifGetActiveTaskIds();
  scifStopTasksNbl(activeTasks);
  scifResetTaskStructs(activeTasks, BV(SCIF_STRUCT_INPUT) | BV(SCIF_STRUCT_OUTPUT) | BV(SCIF_STRUCT_CFG) | BV(SCIF_STRUCT_STATE));
  //turn measuring to 0 
  measuring = 0;
  //update the measuring value to bluetoothService
  UserInterface_SetParameter(USERINTERFACE_MEASURING, USERINTERFACE_MEASURING_LEN, &measuring);
  return;
}


//processSampleADC()
static void processSampleADC(void)
{
  // Set the dawnStr to the String characteristic in Data Service
  switch(lmpMode) {
    case RUN_CV_TASK:
      {
        uint16_t biasValue = scifTaskData.cvTask.state.refcnRegValue;
        uint16_t lmpValue = scifTaskData.sampleAdc.output.adcValue;
        switch(weSelect){
          case WE0:
          {// Get state.biasStep to find the bias Step
            BluetoothService_SetParameter(BLUETOOTHSERVICE_WE0_I, BLUETOOTHSERVICE_WE0_LEN_I, &biasValue);
            BluetoothService_SetParameter(BLUETOOTHSERVICE_WE0_O, BLUETOOTHSERVICE_WE0_LEN_O, &lmpValue);
            break;
          }
          case WE1:
          {// Get state.biasStep to find the bias Step
            BluetoothService_SetParameter(BLUETOOTHSERVICE_WE1_I, BLUETOOTHSERVICE_WE1_LEN_I, &biasValue);
            BluetoothService_SetParameter(BLUETOOTHSERVICE_WE1_O, BLUETOOTHSERVICE_WE1_LEN_O, &lmpValue);
            break;
          }
          case WE2:
          {// Get state.biasStep to find the bias Step
            BluetoothService_SetParameter(BLUETOOTHSERVICE_WE2_I, BLUETOOTHSERVICE_WE2_LEN_I, &biasValue);
            BluetoothService_SetParameter(BLUETOOTHSERVICE_WE2_O, BLUETOOTHSERVICE_WE2_LEN_O, &lmpValue);
            break;
          }
          case WE3:
          {// Get state.biasStep to find the bias Step
            BluetoothService_SetParameter(BLUETOOTHSERVICE_WE3_I, BLUETOOTHSERVICE_WE3_LEN_I, &biasValue);
            BluetoothService_SetParameter(BLUETOOTHSERVICE_WE3_O, BLUETOOTHSERVICE_WE3_LEN_O, &lmpValue);
            break;
          }

        }
      }
    case RUN_CA_TASK:
      {
        break;
      }
    default:
        break;
 
  }
  return;
}






/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
 * @brief   Task creation function for the user task.
 *
 * @param   None.
 *
 * @return  None.
 */////////////
void ProjectZero_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = przTaskStack;
  taskParams.stackSize = PRZ_TASK_STACK_SIZE;
  taskParams.priority = PRZ_TASK_PRIORITY;

  Task_construct(&przTask, ProjectZero_taskFxn, &taskParams, NULL);
}
/////////////////
/*
 * @brief   Called before the task loop and contains application-specific
 *          initialization of the BLE stack, hardware setup, power-state
 *          notification if used, and BLE profile/service initialization.
 *
 * @param   None.
 *
 * @return  None.
 */
static void ProjectZero_init(void)
{
  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages via ICall to Stack.
  ICall_registerApp(&selfEntity, &sem);

  Log_info0("Initializing the user task, hardware, BLE stack and services.");

  // Open display. By default this is disabled via the predefined symbol Display_DISABLE_ALL.
  dispHandle = Display_open(Display_Type_LCD, NULL);

  // Initialize queue for application messages.
  // Note: Used to transfer control to application thread from e.g. interrupts.
  Queue_construct(&applicationMsgQ, NULL);
  hApplicationMsgQ = Queue_handle(&applicationMsgQ);

  // Initialize queue for service messages.
  // Note: Used to transfer control to application thread
  Queue_construct(&serviceMsgQ, NULL);        // SOLUTION
  hServiceMsgQ = Queue_handle(&serviceMsgQ);  // SOLUTION

  // ******************************************************************
  // Hardware initialization
  // ******************************************************************

  // Open MUX pins
  muxPinHandle = PIN_open(&muxPinState, muxPinTable);
  if(!muxPinHandle) {
    Log_error0("Error initializing board LED pins");
    Task_exit();
  }
  PIN_setOutputEnable(muxPinHandle, MUX_EN, 1);
  PIN_setOutputEnable(muxPinHandle, MUX_A0, 1);
  PIN_setOutputEnable(muxPinHandle, MUX_A1, 1);
  PIN_setOutputEnable(muxPinHandle, MUX_A2, 1);
  PIN_setOutputValue(muxPinHandle, MUX_EN, 0);
  PIN_setOutputValue(muxPinHandle, MUX_A0, 0);
  PIN_setOutputValue(muxPinHandle, MUX_A1, 0);
  PIN_setOutputValue(muxPinHandle, MUX_A2, 0);

  // ******************************************************************
  // BLE Stack initialization
  // ******************************************************************

  // Setup the GAP Peripheral Role Profile
  uint8_t initialAdvertEnable = TRUE;  // Advertise on power-up

  // By setting this to zero, the device will go into the waiting state after
  // being discoverable. Otherwise wait this long [ms] before advertising again.
  uint16_t advertOffTime = 0; // miliseconds

  // Set advertisement enabled.
  GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                       &initialAdvertEnable);

  // Configure the wait-time before restarting advertisement automatically
  GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                       &advertOffTime);

  // Initialize Scan Response data
  GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);

  // Initialize Advertisement data
  GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

  Log_info1("Name in advertData array: \x1b[33m%s\x1b[0m",
            (IArg)Util_getLocalNameStr(advertData));

  // Set advertising interval
  uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

  GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
  GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
  GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
  GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);

  // Set duration of advertisement before stopping in Limited adv mode.
  GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, 30); // Seconds

  // ******************************************************************
  // BLE Bond Manager initialization
  // ******************************************************************
  uint32_t passkey = 0; // passkey "000000"
  uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
  uint8_t mitm = TRUE;
  uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
  uint8_t bonding = TRUE;

  GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                          &passkey);
  GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
  GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
  GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
  GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);

  // ******************************************************************
  // BLE Service initialization
  // ******************************************************************

  // Add services to GATT server
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

  // Set the device name characteristic in the GAP Profile
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Add Bluetooth Service
  BluetoothService_AddService();

  // Initialize Parameters for Bluetooth Service

  // Add userInterface Service
  UserInterface_AddService();
  UserInterface_RegisterAppCBs( &user_userInterfaceCBs );

  // Initialize Parameters for userInterface Service

  // Start the stack in Peripheral mode.
  VOID GAPRole_StartDevice(&user_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&user_bondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // ******************************************************************
  // Sensor Controller Initialization
  // ******************************************************************

  // Initialize the Sensor Controller
  scifOsalInit();
  scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
  scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
  scifInit(&scifDriverSetup);

  // Set the Sensor Controller task tick interval
  scifStartRtcTicksNow(0x00010000 / rtc_Hz);

  // Configure Sensor Controller tasks
  scifTaskData.cvTask.cfg.weSelect = 0;
  scifTaskData.cvTask.state.biasStep = 0;
  scifTaskData.cvTask.cfg.rtcPeriod = cvPeriod * rtc_Hz;

  // Sensor controller tasks started on user input
}


/*
 * @brief   Application task entry point.
 *
 *          Invoked by TI-RTOS when BIOS_start is called. Calls an init function
 *          and enters an infinite loop waiting for messages.
 *
 *          Messages can be either directly from the BLE stack or from user code
 *          like Hardware Interrupt (Hwi) or a callback function.
 *
 *          The reason for sending messages to this task from e.g. Hwi's is that
 *          some RTOS and Stack APIs are not available in callbacks and so the
 *          actions that may need to be taken is dispatched to this Task.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void ProjectZero_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  ProjectZero_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Check if we got a signal because of a stack message
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for event flags received (event signature 0xffff)
          if (pEvt->signature == 0xffff)
          {
            // Event received when a connection event is completed
            if (pEvt->event_flag & PRZ_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              ProjectZero_sendAttRsp();
            }
          }
          else // It's a message from the stack and not an event.
          {
            // Process inter-task message
            safeToDealloc = ProjectZero_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // Process messages sent from another task or another context.
      while (!Queue_empty(hApplicationMsgQ))
      {
        app_msg_t *pMsg = Queue_dequeue(hApplicationMsgQ);

        // Process application-layer message probably sent from ourselves.
        user_processApplicationMessage(pMsg);

        // Free the received message.
        ICall_free(pMsg);
      }

      // Process messages sent from another task or another context. // SOLUTION
      while (!Queue_empty(hServiceMsgQ))
      {
        server_char_write_t* pWrite = Queue_dequeue(hServiceMsgQ);

        // Process service message.
        switch (pWrite->svcUUID) {
        case USERINTERFACE_SERV_UUID:
          user_userInterface_ValueChangeDispatchHandler(pWrite);
          break;
        }

        // Free the message received from the service callback.
        ICall_free(pWrite);
      }
    }
  }
}


/*
 * @brief   Handle application messages
 *
 *          These are messages not from the BLE stack, but from the
 *          application itself.
 *
 *          For example, in a Software Interrupt (Swi) it is not possible to
 *          call any BLE APIs, so instead the Swi function must send a message
 *          to the application Task for processing in Task context.
 *
 * @param   pMsg  Pointer to the message of type app_msg_t.
 *
 * @return  None.
 */
static void user_processApplicationMessage(app_msg_t *pMsg)
{
  switch (pMsg->type)
  {
    case APP_MSG_GAP_STATE_CHANGE: /* Message that GAP state changed  */
      user_processGapStateChangeEvt( *(gaprole_States_t *)pMsg->pdu );
      break;

    case APP_MSG_SEND_PASSCODE: /* Message about pairing PIN request */
      {
        passcode_req_t *pReq = (passcode_req_t *)pMsg->pdu;
        Log_info2("BondMgr Requested passcode. We are %s passcode %06d",
                  (IArg)(pReq->uiInputs?"Sending":"Displaying"),
                  DEFAULT_PASSCODE);
        // Send passcode response.
        GAPBondMgr_PasscodeRsp(pReq->connHandle, SUCCESS, DEFAULT_PASSCODE);
      }
      break;

    case APP_MSG_SC_TASK_ALERT: // Message from Sensor Controller
      {
        processTaskAlert();
        break;
      }
  }
}


/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of system/application events deferred to the user Task context.
 *  Invoked from the application Task function above.
 *
 *  Further down you can find the callback handler section containing the
 *  functions that defer their actions via messages to the application task.
 *
 ****************************************************************************
 *****************************************************************************/

/*
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void user_processGapStateChangeEvt(gaprole_States_t newState)
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
        char *cstr_ownAddress = Util_convertBdAddr2Str(ownAddress);
        Log_info1("GAP is started. Our address: \x1b[32m%s\x1b[0m", (IArg)cstr_ownAddress);
      }
      break;

    case GAPROLE_ADVERTISING:
      Log_info0("Advertising");
      break;

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        char *cstr_peerAddress = Util_convertBdAddr2Str(peerAddress);
        Log_info1("Connected. Peer address: \x1b[32m%s\x1b[0m", (IArg)cstr_peerAddress);
       }
      break;

    case GAPROLE_CONNECTED_ADV:
      Log_info0("Connected and advertising");
      break;

    case GAPROLE_WAITING:
      Log_info0("Disconnected / Idle");
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      Log_info0("Connection timed out");
      break;

    case GAPROLE_ERROR:
      Log_info0("Error");
      break;

    default:
      break;
  }
}

/*
 * @brief   Process an incoming BLE stack message.
 *
 *          This could be a GATT message from a peer device like acknowledgement
 *          of an Indication we sent, or it could be a response from the stack
 *          to an HCI message that the user application sent.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ProjectZero_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = ProjectZero_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            Log_info0("HCI Command Complete Event received");
            break;

          default:
            break;
        }
      }
      break;

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}


/*
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    Log_warning1("Outgoing RF FIFO full. Re-schedule transmission of msg with opcode 0x%02x",
      pMsg->method);

    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   PRZ_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      ProjectZero_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Log the opcode of the message that caused the violation.
    Log_error1("Flow control violated. Opcode of offending ATT msg: 0x%02x",
      pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Log_info1("MTU Size change: %d bytes", pMsg->msg.mtuEvt.MTU);
  }
  else
  {
    // Got an expected GATT message from a peer.
    Log_info1("Recevied GATT Message. Opcode: 0x%02x", pMsg->method);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}




/*
 *  Application error handling functions
 *****************************************************************************/

/*
 * @brief   Send a pending ATT response message.
 *
 *          The message is one that the stack was trying to send based on a
 *          peer request, but the response couldn't be sent because the
 *          user application had filled the TX queue with other data.
 *
 * @param   none
 *
 * @return  none
 */
static void ProjectZero_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      ProjectZero_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Log_warning2("Retrying message with opcode 0x%02x. Attempt %d",
        pAttRsp->method, rspTxRetry);
    }
  }
}

/*
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void ProjectZero_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Log_info2("Sent message with opcode 0x%02x. Attempt %d",
        pAttRsp->method, rspTxRetry);
    }
    else
    {
      Log_error2("Gave up message with opcode 0x%02x. Status: %d",
        pAttRsp->method, status);

      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}


/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of direct system callbacks.
 *
 *  Typically enqueue the information or request as a message for the
 *  application Task for handling.
 *
 ****************************************************************************
 *****************************************************************************/


/*
 *  Callbacks from the Stack Task context (GAP or Service changes)
 *****************************************************************************/

/**
 * Callback from GAP Role indicating a role state change.
 */
static void user_gapStateChangeCB(gaprole_States_t newState)
{
  Log_info1("(CB) GAP State change: %d, Sending msg to app.", (IArg)newState);
  user_enqueueRawAppMsg( APP_MSG_GAP_STATE_CHANGE, (uint8_t *)&newState, sizeof(newState) );
}

/*
 * @brief   Passcode callback.
 *
 * @param   connHandle - connection handle
 * @param   uiInputs   - input passcode?
 * @param   uiOutputs  - display passcode?
 * @param   numComparison - numeric comparison value
 *
 * @return  none
 */
static void user_gapBondMgr_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                       uint8_t uiInputs, uint8_t uiOutputs, uint32 numComparison)
{
  passcode_req_t req =
  {
    .connHandle = connHandle,
    .uiInputs = uiInputs,
    .uiOutputs = uiOutputs,
    .numComparison = numComparison
  };

  // Defer handling of the passcode request to the application, in case
  // user input is required, and because a BLE API must be used from Task.
  user_enqueueRawAppMsg(APP_MSG_SEND_PASSCODE, (uint8_t *)&req, sizeof(req));
}

/*
 * @brief   Pairing state callback.
 *
 * @param   connHandle - connection handle
 * @param   state      - pairing state
 * @param   status     - pairing status
 *
 * @return  none
 */
static void user_gapBondMgr_pairStateCB(uint16_t connHandle, uint8_t state,
                                        uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Log_info0("Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      Log_info0("Pairing completed successfully.");
    }
    else
    {
      Log_error1("Pairing failed. Error: %02x", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
     Log_info0("Re-established pairing from stored bond info.");
    }
  }
}

/******************************************************************************
 *****************************************************************************
 *
 *  Utility functions
 *
 ****************************************************************************
 *****************************************************************************/

/*
 * @brief  Generic message constructor for application messages.
 *
 *         Sends a message to the application for handling in Task context.
 *
 * @param  appMsgType    Enumerated type of message being sent.
 * @oaram  *pValue       Pointer to characteristic value
 * @param  len           Length of characteristic data
 */
static void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData,
                                  uint16_t len)
{
  // Allocate memory for the message.
  app_msg_t *pMsg = ICall_malloc( sizeof(app_msg_t) + len );

  if (pMsg != NULL)
  {
    pMsg->type = appMsgType;

    // Copy data into message
    memcpy(pMsg->pdu, pData, len);

    // Enqueue the message using pointer to queue node element.
    Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
    // Let application know there's a message.
    Semaphore_post(sem);
  }
}

/*
 * @brief   Convert {0x01, 0x02} to "01:02"
 *
 * @param   src - source byte-array
 * @param   src_len - length of array
 * @param   dst - destination string-array
 * @param   dst_len - length of array
 *
 * @return  array as string
 */
static char *Util_convertArrayToHexString(uint8_t const *src, uint8_t src_len,
                                          uint8_t *dst, uint8_t dst_len)
{
  char        hex[] = "0123456789ABCDEF";
  uint8_t     *pStr = dst;
  uint8_t     avail = dst_len-1;

  memset(dst, 0, avail);

  while (src_len && avail > 3)
  {
    if (avail < dst_len-1) { *pStr++ = ':'; avail -= 1; };
    *pStr++ = hex[*src >> 4];
    *pStr++ = hex[*src++ & 0x0F];
    avail -= 2;
    src_len--;
  }

  if (src_len && avail)
    *pStr++ = ':'; // Indicate not all data fit on line.

  return (char *)dst;
}

/*
 * @brief   Extract the LOCALNAME from Scan/AdvData
 *
 * @param   data - Pointer to the advertisement or scan response data
 *
 * @return  Pointer to null-terminated string with the adv local name.
 */
static char *Util_getLocalNameStr(const uint8_t *data) {
  uint8_t nuggetLen = 0;
  uint8_t nuggetType = 0;
  uint8_t advIdx = 0;

  static char localNameStr[32] = { 0 };
  memset(localNameStr, 0, sizeof(localNameStr));

  for (advIdx = 0; advIdx < 32;) {
    nuggetLen = data[advIdx++];
    nuggetType = data[advIdx];
    if ( (nuggetType == GAP_ADTYPE_LOCAL_NAME_COMPLETE ||
          nuggetType == GAP_ADTYPE_LOCAL_NAME_SHORT) && nuggetLen < 31) {
      memcpy(localNameStr, &data[advIdx + 1], nuggetLen - 1);
      break;
    } else {
      advIdx += nuggetLen;
    }
  }

  return localNameStr;
}

/*
  * @brief   Process message from userInterface service value change
  *
  *          These are messages not from the BLE stack, but from the
  *          application itself.
  *
  *
  * @param   pWrite - Pointer to struct with new values
  */
void user_userInterface_ValueChangeDispatchHandler(server_char_write_t* pWrite)
{

  switch (pWrite->paramID) {
    
    case USERINTERFACE_WESELECT:
      {
        uint16_t new_weSelect = 0;
        memcpy(&new_weSelect, pWrite->data, USERINTERFACE_WESELECT_LEN);

        // boundary checks
        if ( measuring == 1  || new_weSelect > 7) {
          UserInterface_SetParameter(USERINTERFACE_WESELECT, USERINTERFACE_WESELECT_LEN, &weSelect);
        } else {
          weSelect = new_weSelect;
          switch (weSelect) {
            case 0:
              PIN_setOutputValue(muxPinHandle, MUX_A0, 0);
              PIN_setOutputValue(muxPinHandle, MUX_A1, 0);
              PIN_setOutputValue(muxPinHandle, MUX_A2, 0);
              break;
            case 1:
              PIN_setOutputValue(muxPinHandle, MUX_A0, 1);
              PIN_setOutputValue(muxPinHandle, MUX_A1, 0);
              PIN_setOutputValue(muxPinHandle, MUX_A2, 0);
              break;
            case 2:
              PIN_setOutputValue(muxPinHandle, MUX_A0, 0);
              PIN_setOutputValue(muxPinHandle, MUX_A1, 1);
              PIN_setOutputValue(muxPinHandle, MUX_A2, 0);
              break;
            case 3:
              PIN_setOutputValue(muxPinHandle, MUX_A0, 1);
              PIN_setOutputValue(muxPinHandle, MUX_A1, 1);
              PIN_setOutputValue(muxPinHandle, MUX_A2, 0);
              break;
            case 4:
              PIN_setOutputValue(muxPinHandle, MUX_A0, 0);
              PIN_setOutputValue(muxPinHandle, MUX_A1, 0);
              PIN_setOutputValue(muxPinHandle, MUX_A2, 1);
              break;
            case 5:
              PIN_setOutputValue(muxPinHandle, MUX_A0, 1);
              PIN_setOutputValue(muxPinHandle, MUX_A1, 0);
              PIN_setOutputValue(muxPinHandle, MUX_A2, 1);
              break;
            case 6:
              PIN_setOutputValue(muxPinHandle, MUX_A0, 0);
              PIN_setOutputValue(muxPinHandle, MUX_A1, 1);
              PIN_setOutputValue(muxPinHandle, MUX_A2, 1);
              break;
            case 7:
              PIN_setOutputValue(muxPinHandle, MUX_A0, 1);
              PIN_setOutputValue(muxPinHandle, MUX_A1, 1);
              PIN_setOutputValue(muxPinHandle, MUX_A2, 1);
              break;
            default:
              break;
          }
        }
        break;
      }

    case USERINTERFACE_GAIN:
      {
        uint16_t new_tiaGain = 0;
        memcpy(&new_tiaGain, pWrite->data, USERINTERFACE_GAIN_LEN);

        // boundary checks
        if (measuring || new_tiaGain > 7) {
          uint16_t old_tiaGain = ( tiaGain-3 ) >> 2;
          UserInterface_SetParameter(USERINTERFACE_GAIN, USERINTERFACE_WESELECT_LEN, &old_tiaGain);
        } else {
          tiaGain = (new_tiaGain << 2) | 3;
        }

        break;
      }

    case USERINTERFACE_INPUT:
      {
        uint16_t new_lmpMode = 0;
        memcpy(&new_lmpMode, pWrite->data, USERINTERFACE_INPUT_LEN);

        // boundary checks
        if (measuring == 1 || new_lmpMode > 3) {
          UserInterface_SetParameter(USERINTERFACE_INPUT, USERINTERFACE_INPUT_LEN, &lmpMode);
        } else {
          lmpMode = new_lmpMode; 
        }

        break;
      }
    
    case USERINTERFACE_MEASURING:
      {
        uint16_t new_measuring = 0;
        memcpy(&new_measuring, pWrite->data, USERINTERFACE_MEASURING_LEN);

        // boundary checks
        if (new_measuring > 1) {
          UserInterface_SetParameter(USERINTERFACE_MEASURING, USERINTERFACE_MEASURING_LEN, &measuring);
          break;
        }

        measuring = new_measuring;
        
        // When user writes measuring to 1, they are STARTING a new measurement
        if (measuring) {
          // Switch over the mode to start the selected analysis technique
          switch (lmpMode) {
            case RUN_CV_TASK:
              // Enable MUX
              PIN_setOutputValue(muxPinHandle, MUX_EN, 1);

              // Set configuration parameters
              scifTaskData.cvTask.cfg.internalGain = tiaGain;
              scifTaskData.cvTask.cfg.weSelect = weSelect;
              scifTaskData.cvTask.cfg.rtcPeriod = cvPeriod * rtc_Hz;
              scifTaskData.cvTask.cfg.TaskSelection = 0;

              // Start Tasks
              scifStartTasksNbl(BV(SCIF_CV_TASK_TASK_ID) | BV(SCIF_SAMPLE_ADC_TASK_ID));
              break;
            case RUN_CA_TASK:
              // Enable MUX
              PIN_setOutputValue(muxPinHandle, MUX_EN, 1);

              // Set configuration parameters
              scifTaskData.cvTask.cfg.internalGain = tiaGain;
              scifTaskData.cvTask.cfg.weSelect = weSelect;
              scifTaskData.cvTask.cfg.rtcPeriodCA = caPulseLen * rtc_Hz;
              scifTaskData.cvTask.cfg.TaskSelection = 1;

              // Start Tasks
              scifStartTasksNbl(BV(SCIF_CV_TASK_TASK_ID) | BV(SCIF_SAMPLE_ADC_TASK_ID));
              break;
            default:
              break;
          }

        // When user writes measuring to 0, they are STOPPING the measurement 
        } else {
          // Kill all SC tasks
          uint32_t activeTasks = scifGetActiveTaskIds();
          scifStopTasksNbl(activeTasks);
          scifResetTaskStructs(activeTasks, BV(SCIF_STRUCT_INPUT) | BV(SCIF_STRUCT_OUTPUT) | BV(SCIF_STRUCT_CFG) | BV(SCIF_STRUCT_STATE));
        }
        break;
      }
    default:
      return;
  }
}

/*
@brief   Callback from sunlightService indicating a characteristic value change
*
* This function asks the service what the received data was, and sends
* this in a message to the user Task for processing.
*
* @param   paramID - parameter ID of the value that was changed
*/


static void user_userInterfaceValueChangeCB(uint8_t paramID)
{
  // See sunlightService.h to compare paramID with characteristic value attribute.
  // Called in Stack Task context, so can't do processing here.

  // Send message to application message queue about received data.
  uint16_t readLen = 0; // How much to read via service API

  switch (paramID) {

    case USERINTERFACE_WESELECT:
      readLen = USERINTERFACE_WESELECT_LEN;
      break;

    case USERINTERFACE_GAIN:
      readLen = USERINTERFACE_GAIN_LEN;
      break;
    
    case USERINTERFACE_INPUT:
      readLen = USERINTERFACE_INPUT_LEN;
      break;
    
    case USERINTERFACE_MEASURING:
      readLen = USERINTERFACE_MEASURING_LEN;
      break;
    
    default:
      return;
  }

  // Allocate memory for the message.
  // Note: The message doesn't have to contain the data itself, as that's stored in
  //       a variable in the service. However, to prevent data loss if a new value is received
  //       before GetParameter is called, we call GetParameter now.
  server_char_write_t* pWrite = ICall_malloc(sizeof(server_char_write_t) + readLen);

  if (pWrite != NULL)
  {
    pWrite->svcUUID = USERINTERFACE_SERV_UUID;
    pWrite->dataLen = readLen;
    pWrite->paramID = paramID;
    // Get the data from the service API.
    // Note: Fixed length is used here, but changing the GetParameter signature is not
    //       a problem, in case variable length is needed.
    // Note: It could be just as well to send dataLen and a pointer to the received data directly to this callback, avoiding GetParameter alltogether.
    UserInterface_GetParameter(paramID, pWrite->data);

    // Enqueue the message using pointer to queue node element.
    Queue_enqueue(hServiceMsgQ, &pWrite->_elem);
    // Let application know there's a message
    Semaphore_post(sem);
  }
}
/*********************************************************************
*********************************************************************/

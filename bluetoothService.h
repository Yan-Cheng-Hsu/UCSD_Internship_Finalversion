/**********************************************************************************************
 * Filename:       bluetoothService.h
 *
 * Description:    This file contains the bluetoothService service definitions and
 *                 prototypes.
 *
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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
 *
 *************************************************************************************************/


#ifndef _BLUETOOTHSERVICE_H_
#define _BLUETOOTHSERVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
* CONSTANTS
*/
// Service UUID
#define BLUETOOTHSERVICE_SERV_UUID 0xBA55


 /*@@@For our own bluetooth service*/

// WE0 INPUT
#define BLUETOOTHSERVICE_WE0_I 0
#define BLUETOOTHSERVICE_WE0_UUID_I 0x2131
#define BLUETOOTHSERVICE_WE0_LEN_I 2
// WE0 OUTPUT
#define BLUETOOTHSERVICE_WE0_O 1
#define BLUETOOTHSERVICE_WE0_UUID_O 0x2241
#define BLUETOOTHSERVICE_WE0_LEN_O 2


//WE1 INPUT
#define BLUETOOTHSERVICE_WE1_I 2
#define BLUETOOTHSERVICE_WE1_UUID_I 0x2351
#define BLUETOOTHSERVICE_WE1_LEN_I 2
//WE1 OUTPUT
#define BLUETOOTHSERVICE_WE1_O 3
#define BLUETOOTHSERVICE_WE1_UUID_O 0x2461
#define BLUETOOTHSERVICE_WE1_LEN_O 2


//WE2 INPUT
#define BLUETOOTHSERVICE_WE2_I 4
#define BLUETOOTHSERVICE_WE2_UUID_I 0x2571
#define BLUETOOTHSERVICE_WE2_LEN_I 2
//WE2 OUTPUT
#define BLUETOOTHSERVICE_WE2_O 5
#define BLUETOOTHSERVICE_WE2_UUID_O 0x2681
#define BLUETOOTHSERVICE_WE2_LEN_O 2

//WE3 INPUT
#define BLUETOOTHSERVICE_WE3_I 6
#define BLUETOOTHSERVICE_WE3_UUID_I 0x2791
#define BLUETOOTHSERVICE_WE3_LEN_I 2

//WE3 INPUT
#define BLUETOOTHSERVICE_WE3_O 7
#define BLUETOOTHSERVICE_WE3_UUID_O 0x28A1
#define BLUETOOTHSERVICE_WE3_LEN_O 2


 /*@@@For our own bluetooth service*/

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*bluetoothServiceChange_t)( uint8 paramID );

typedef struct
{
  bluetoothServiceChange_t        pfnChangeCb;  // Called when characteristic value changes
} bluetoothServiceCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * BluetoothService_AddService- Initializes the BluetoothService service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t BluetoothService_AddService( void );

/*
 * BluetoothService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t BluetoothService_RegisterAppCBs( bluetoothServiceCBs_t *appCallbacks );

/*
 * BluetoothService_SetParameter - Set a BluetoothService parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t BluetoothService_SetParameter( uint8 param, uint8 len, void *value );

/*
 * BluetoothService_GetParameter - Get a BluetoothService parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t BluetoothService_GetParameter( uint8 param, void *value );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _BLUETOOTHSERVICE_H_ */

/**********************************************************************************************
 * Filename:       bluetoothService.c
 *
 * Description:    This file contains the implementation of the service.
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


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "bluetoothService.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// bluetoothService Service UUID
CONST uint8_t bluetoothServiceUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BLUETOOTHSERVICE_SERV_UUID), HI_UINT16(BLUETOOTHSERVICE_SERV_UUID)
};
// Service declaration
static CONST gattAttrType_t bluetoothServiceDecl = { ATT_BT_UUID_SIZE, bluetoothServiceUUID };


/*@@@For our own bluetooth service*/

//WE0 INPUT
CONST uint8_t bluetoothService_WE0_UUID_I[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLUETOOTHSERVICE_WE0_UUID_I)
};

// Characteristic "bluetoothService_WE0" Properties (for declaration)
static uint8_t bluetoothService_WE0_props_I = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "bluetoothService_WE0" Value variable
static uint8_t bluetoothService_WE0_val_I[BLUETOOTHSERVICE_WE0_LEN_I] = { 0 };

// Characteristic "bluetoothService_WE0" CCCD
static gattCharCfg_t* bluetoothService_WE0_CCCD_I;

//  WE0 User Description
static uint8 bluetoothService_WE0_UserDis_I[15] = "Input of WE0: ";

//WE0 OUTPUT
CONST uint8_t bluetoothService_WE0_UUID_O[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLUETOOTHSERVICE_WE0_UUID_O)
};
// Characteristic "bluetoothService_WE0" Properties (for declaration)
static uint8_t bluetoothService_WE0_props_O = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "bluetoothService_WE0" Value variable
static uint8_t bluetoothService_WE0_val_O[BLUETOOTHSERVICE_WE0_LEN_O] = { 1 };

// Characteristic "bluetoothService_WE0" CCCD
static gattCharCfg_t* bluetoothService_WE0_CCCD_O;

//  WE0 User Description
static uint8 bluetoothService_WE0_UserDis_O[16] = "Output of WE0: ";





//WE1 INPUT
CONST uint8_t bluetoothService_WE1_UUID_I[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLUETOOTHSERVICE_WE1_UUID_I)
};
// Characteristic "bluetoothService_WE1" Properties (for declaration)
static uint8_t bluetoothService_WE1_props_I = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "bluetoothService_WE1" Value variable
static uint8_t bluetoothService_WE1_val_I[BLUETOOTHSERVICE_WE1_LEN_I] = { 2 };

// Characteristic "bluetoothService_WE1" CCCD
static gattCharCfg_t* bluetoothService_WE1_CCCD_I;

//  WE1 User Description
static uint8 bluetoothService_WE1_UserDis_I[15] = "Input of WE1: ";

//WE1 OUTPUT
CONST uint8_t bluetoothService_WE1_UUID_O[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLUETOOTHSERVICE_WE1_UUID_O)
};
// Characteristic "bluetoothService_WE1" Properties (for declaration)
static uint8_t bluetoothService_WE1_props_O = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "bluetoothService_WE1" Value variable
static uint8_t bluetoothService_WE1_val_O[BLUETOOTHSERVICE_WE1_LEN_O] = { 3 };

// Characteristic "bluetoothService_WE1" CCCD
static gattCharCfg_t* bluetoothService_WE1_CCCD_O;

//  WE1 User Description
static uint8 bluetoothService_WE1_UserDis_O[16] = "Output of WE1: ";



//WE2 INPUT
CONST uint8_t bluetoothService_WE2_UUID_I[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLUETOOTHSERVICE_WE2_UUID_I)
};
// Characteristic "bluetoothService_WE2" Properties (for declaration)
static uint8_t bluetoothService_WE2_props_I = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "bluetoothService_WE2" Value variable
static uint8_t bluetoothService_WE2_val_I[BLUETOOTHSERVICE_WE2_LEN_I] = { 4 };

// Characteristic "bluetoothService_WE2" CCCD
static gattCharCfg_t* bluetoothService_WE2_CCCD_I;

//  WE1 User Description
static uint8 bluetoothService_WE2_UserDis_I[15] = "Input of WE2: ";

//WE2 OUTPUT
CONST uint8_t bluetoothService_WE2_UUID_O[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLUETOOTHSERVICE_WE2_UUID_O)
};
// Characteristic "bluetoothService_WE2" Properties (for declaration)
static uint8_t bluetoothService_WE2_props_O = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "bluetoothService_WE2" Value variable
static uint8_t bluetoothService_WE2_val_O[BLUETOOTHSERVICE_WE2_LEN_O] = { 5 };

// Characteristic "bluetoothService_WE2" CCCD
static gattCharCfg_t* bluetoothService_WE2_CCCD_O;

//  WE1 User Description
static uint8 bluetoothService_WE2_UserDis_O[16] = "Output of WE2: ";



//WE3 INPUT
CONST uint8_t bluetoothService_WE3_UUID_I[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLUETOOTHSERVICE_WE3_UUID_I)
};
// Characteristic "bluetoothService_WE3" Properties (for declaration)
static uint8_t bluetoothService_WE3_props_I = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "bluetoothService_WE3" Value variable
static uint8_t bluetoothService_WE3_val_I[BLUETOOTHSERVICE_WE2_LEN_I] = { 6 };

// Characteristic "bluetoothService_WE3" CCCD
static gattCharCfg_t* bluetoothService_WE3_CCCD_I;

//  WE3 User Description
static uint8 bluetoothService_WE3_UserDis_I[15] = "Input of WE3: ";

//WE3 OUTPUT
CONST uint8_t bluetoothService_WE3_UUID_O[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLUETOOTHSERVICE_WE3_UUID_O)
};
// Characteristic "bluetoothService_WE3" Properties (for declaration)
static uint8_t bluetoothService_WE3_props_O = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "bluetoothService_WE3" Value variable
static uint8_t bluetoothService_WE3_val_O[BLUETOOTHSERVICE_WE2_LEN_O] = { 6 };

// Characteristic "bluetoothService_WE3" CCCD
static gattCharCfg_t* bluetoothService_WE3_CCCD_O;

//  WE3 User Description
static uint8 bluetoothService_WE3_UserDis_O[16] = "Output of WE3: ";





/*@@@For our own bluetooth service*/




/*********************************************************************
 * LOCAL VARIABLES
 */

static bluetoothServiceCBs_t *pAppCBs = NULL;





/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t bluetoothServiceAttrTbl[] =
{
  // bluetoothService Service Declaration
	{
		{ ATT_BT_UUID_SIZE, primaryServiceUUID },
		GATT_PERMIT_READ,
		0,
		(uint8_t *)&bluetoothServiceDecl
	},  
	/*@@@For our own bluetooth service*/

	// WE0 INPUT
	// Declaration
	{
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ,
		0,
		& bluetoothService_WE0_props_I
	},

	// Characteristic Value
	{
		{ ATT_UUID_SIZE, bluetoothService_WE0_UUID_I },
		GATT_PERMIT_READ,
		0,
		bluetoothService_WE0_val_I
	},

	// CCCD
	{
		{ ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		0,
		(uint8*)& bluetoothService_WE0_CCCD_I
	},

	// User Description
	{
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ,
		0,
		bluetoothService_WE0_UserDis_I
	},

	// WE0 OUTPUT
	// Declaration
	{
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ ,
		0,
		&bluetoothService_WE0_props_O
	},

	// Characteristic Value
	{
		{ ATT_UUID_SIZE, bluetoothService_WE0_UUID_O },
		GATT_PERMIT_READ,
		0,
		bluetoothService_WE0_val_O
	},

	// CCCD
	{
		{ ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		0,
		(uint8*)& bluetoothService_WE0_CCCD_O
	},

	// User Description
	{
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ,
		0,
		bluetoothService_WE0_UserDis_O
	},

	// WE1 INPUT
	// Declaration
	{
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ,
		0,
		&bluetoothService_WE1_props_I
	},

	// Characteristic Value
	{
		{ ATT_UUID_SIZE, bluetoothService_WE1_UUID_I },
		GATT_PERMIT_READ,
		0,
		bluetoothService_WE1_val_I
	},

	// CCCD
	{
		{ ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		0,
		(uint8*)& bluetoothService_WE1_CCCD_I
	},

	// User Description
	{
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ,
		0,
		bluetoothService_WE1_UserDis_I
	},

	// WE1 OUTPUT
	// Declaration
	{
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ,
		0,
		&bluetoothService_WE1_props_O
	},

			// Characteristic Value
	{
		{ ATT_UUID_SIZE, bluetoothService_WE1_UUID_O },
		GATT_PERMIT_READ,
		0,
		bluetoothService_WE1_val_O
	},

			// CCCD
	{
		{ ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		0,
		(uint8*)& bluetoothService_WE1_CCCD_O
	},

			// User Description
	{
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ,
		0,
		bluetoothService_WE1_UserDis_O
	},

	// WE2 INPUT
	// Declaration
	{
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ,
		0,
		&bluetoothService_WE2_props_I
	},

		// Characteristic Value
	{
		{ ATT_UUID_SIZE, bluetoothService_WE2_UUID_I },
		GATT_PERMIT_READ,
		0,
		bluetoothService_WE2_val_I
	},

		// CCCD
	{
		{ ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		0,
		(uint8*)& bluetoothService_WE2_CCCD_I
	},

		// User Description
	{
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ,
		0,
		bluetoothService_WE2_UserDis_I
	},

	// WE2 OUTPUT
	// Declaration
	{
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ,
		0,
		&bluetoothService_WE2_props_O
	},

			// Characteristic Value
	{
		{ ATT_UUID_SIZE, bluetoothService_WE2_UUID_O },
		GATT_PERMIT_READ,
		0,
		bluetoothService_WE2_val_O
	},

			// CCCD
	{
		{ ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		0,
		(uint8*)& bluetoothService_WE2_CCCD_O
	},

			// User Description
	{
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ,
		0,
		bluetoothService_WE2_UserDis_O
	},


	// WE3 INPUT
	// Declaration
	{
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ,
		0,
		&bluetoothService_WE3_props_I
	},

			// Characteristic Value
	{
		{ ATT_UUID_SIZE, bluetoothService_WE3_UUID_I },
		GATT_PERMIT_READ,
		0,
		bluetoothService_WE3_val_I
	},

			// CCCD
	{
		{ ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		0,
		(uint8*)& bluetoothService_WE3_CCCD_I
	},

			// User Description
	{
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ,
		0,
		bluetoothService_WE3_UserDis_I
	},


	// WE3 OUTPUT
	// Declaration
	{
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ,
		0,
		&bluetoothService_WE3_props_O
	},

			// Characteristic Value
	{
		{ ATT_UUID_SIZE, bluetoothService_WE3_UUID_O },
		GATT_PERMIT_READ,
		0,
		bluetoothService_WE3_val_O
	},

			// CCCD
	{
		{ ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		0,
		(uint8*)& bluetoothService_WE3_CCCD_O
	},

			// User Description
	{
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ,
		0,
		bluetoothService_WE3_UserDis_O
	},

	/*@@@For our own bluetooth service*/


};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t bluetoothService_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 *pLen, uint16 offset,
                                           uint16 maxLen, uint8 method );
static bStatus_t bluetoothService_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint16 len, uint16 offset,
                                            uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t bluetoothServiceCBs =
{
  bluetoothService_ReadAttrCB,  // Read callback function pointer
  bluetoothService_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * BluetoothService_AddService- Initializes the BluetoothService service by registering
 *          GATT attributes with the GATT server.
 *
 */
bStatus_t BluetoothService_AddService( void )
{
  uint8_t status;

  /*@@@For our own bluetooth service*/
  //WE0 INPUT
  bluetoothService_WE0_CCCD_I = (gattCharCfg_t*)ICall_malloc(sizeof(gattCharCfg_t) *
	  linkDBNumConns);
  if (bluetoothService_WE0_CCCD_I == NULL)
  {
	  return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, bluetoothService_WE0_CCCD_I);

  //WE0 OUTPUT
  bluetoothService_WE0_CCCD_O = (gattCharCfg_t*)ICall_malloc(sizeof(gattCharCfg_t) *
	  linkDBNumConns);
  if (bluetoothService_WE0_CCCD_O == NULL)
  {
	  return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, bluetoothService_WE0_CCCD_O);

  //WE1 INPUT
  bluetoothService_WE1_CCCD_I = (gattCharCfg_t*)ICall_malloc(sizeof(gattCharCfg_t) *
	  linkDBNumConns);
  if (bluetoothService_WE1_CCCD_I == NULL)
  {
	  return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, bluetoothService_WE1_CCCD_I);


  //WE1 OUTPUT
  bluetoothService_WE1_CCCD_O = (gattCharCfg_t*)ICall_malloc(sizeof(gattCharCfg_t) *
	  linkDBNumConns);
  if (bluetoothService_WE1_CCCD_O == NULL)
  {
	  return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, bluetoothService_WE1_CCCD_O);


  //WE2 INPUT
  bluetoothService_WE2_CCCD_I = (gattCharCfg_t*)ICall_malloc(sizeof(gattCharCfg_t) *
	  linkDBNumConns);
  if (bluetoothService_WE2_CCCD_I == NULL)
  {
	  return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, bluetoothService_WE2_CCCD_I);


  //WE2 OUTPUT
  bluetoothService_WE2_CCCD_O = (gattCharCfg_t*)ICall_malloc(sizeof(gattCharCfg_t) *
	  linkDBNumConns);
  if (bluetoothService_WE2_CCCD_O == NULL)
  {
	  return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, bluetoothService_WE2_CCCD_O);


  //WE3 INPUT
  bluetoothService_WE3_CCCD_I = (gattCharCfg_t*)ICall_malloc(sizeof(gattCharCfg_t) *
	  linkDBNumConns);
  if (bluetoothService_WE3_CCCD_I == NULL)
  {
	  return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, bluetoothService_WE3_CCCD_I);


  //WE3 OUTPUT
  bluetoothService_WE3_CCCD_O = (gattCharCfg_t*)ICall_malloc(sizeof(gattCharCfg_t) *
	  linkDBNumConns);
  if (bluetoothService_WE3_CCCD_O == NULL)
  {
	  return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, bluetoothService_WE3_CCCD_O);

  /*@@@For our own bluetooth service*/

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( bluetoothServiceAttrTbl,
                                        GATT_NUM_ATTRS( bluetoothServiceAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &bluetoothServiceCBs );

  return ( status );
}

/*
 * BluetoothService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */



bStatus_t BluetoothService_RegisterAppCBs( bluetoothServiceCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

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
bStatus_t BluetoothService_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case BLUETOOTHSERVICE_WE0_I:
	  if (len == BLUETOOTHSERVICE_WE0_LEN_I)
	  {
		  memcpy(bluetoothService_WE0_val_I, value, len);

		  // Try to send notification.
		  GATTServApp_ProcessCharCfg(bluetoothService_WE0_CCCD_I, (uint8_t*)& bluetoothService_WE0_val_I, FALSE,
			  bluetoothServiceAttrTbl, GATT_NUM_ATTRS(bluetoothServiceAttrTbl),
			  INVALID_TASK_ID, bluetoothService_ReadAttrCB);
	  }
	  else
	  {
		  ret = bleInvalidRange;
	  }
	  break;
	case BLUETOOTHSERVICE_WE0_O:
		if (len == BLUETOOTHSERVICE_WE0_LEN_O)
		{
			memcpy(bluetoothService_WE0_val_O, value, len);

			// Try to send notification.
			GATTServApp_ProcessCharCfg(bluetoothService_WE0_CCCD_O, (uint8_t*)& bluetoothService_WE0_val_O, FALSE,
				bluetoothServiceAttrTbl, GATT_NUM_ATTRS(bluetoothServiceAttrTbl),
				INVALID_TASK_ID, bluetoothService_ReadAttrCB);
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;
	case BLUETOOTHSERVICE_WE1_I:
		if (len == BLUETOOTHSERVICE_WE1_LEN_I)
		{
			memcpy(bluetoothService_WE1_val_I, value, len);

			// Try to send notification.
			GATTServApp_ProcessCharCfg(bluetoothService_WE1_CCCD_I, (uint8_t*)& bluetoothService_WE1_val_I, FALSE,
				bluetoothServiceAttrTbl, GATT_NUM_ATTRS(bluetoothServiceAttrTbl),
				INVALID_TASK_ID, bluetoothService_ReadAttrCB);
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;
	case BLUETOOTHSERVICE_WE1_O:
		if (len == BLUETOOTHSERVICE_WE1_LEN_O)
		{
			memcpy(bluetoothService_WE1_val_O, value, len);

			// Try to send notification.
			GATTServApp_ProcessCharCfg(bluetoothService_WE1_CCCD_O, (uint8_t*)& bluetoothService_WE1_val_O, FALSE,
				bluetoothServiceAttrTbl, GATT_NUM_ATTRS(bluetoothServiceAttrTbl),
				INVALID_TASK_ID, bluetoothService_ReadAttrCB);
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;
	case BLUETOOTHSERVICE_WE2_I:
		if (len == BLUETOOTHSERVICE_WE2_LEN_I)
		{
			memcpy(bluetoothService_WE2_val_I, value, len);

			// Try to send notification.
			GATTServApp_ProcessCharCfg(bluetoothService_WE2_CCCD_I, (uint8_t*)& bluetoothService_WE2_val_I, FALSE,
				bluetoothServiceAttrTbl, GATT_NUM_ATTRS(bluetoothServiceAttrTbl),
				INVALID_TASK_ID, bluetoothService_ReadAttrCB);
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;
	case BLUETOOTHSERVICE_WE2_O:
		if (len == BLUETOOTHSERVICE_WE2_LEN_O)
		{
			memcpy(bluetoothService_WE2_val_O, value, len);

			// Try to send notification.
			GATTServApp_ProcessCharCfg(bluetoothService_WE2_CCCD_O, (uint8_t*)& bluetoothService_WE2_val_O, FALSE,
				bluetoothServiceAttrTbl, GATT_NUM_ATTRS(bluetoothServiceAttrTbl),
				INVALID_TASK_ID, bluetoothService_ReadAttrCB);
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;
	case BLUETOOTHSERVICE_WE3_I:
		if (len == BLUETOOTHSERVICE_WE3_LEN_I)
		{
			memcpy(bluetoothService_WE3_val_I, value, len);

			// Try to send notification.
			GATTServApp_ProcessCharCfg(bluetoothService_WE3_CCCD_I, (uint8_t*)& bluetoothService_WE3_val_I, FALSE,
				bluetoothServiceAttrTbl, GATT_NUM_ATTRS(bluetoothServiceAttrTbl),
				INVALID_TASK_ID, bluetoothService_ReadAttrCB);
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;
	case BLUETOOTHSERVICE_WE3_O:
		if (len == BLUETOOTHSERVICE_WE3_LEN_O)
		{
			memcpy(bluetoothService_WE3_val_O, value, len);

			// Try to send notification.
			GATTServApp_ProcessCharCfg(bluetoothService_WE3_CCCD_O, (uint8_t*)& bluetoothService_WE3_val_O, FALSE,
				bluetoothServiceAttrTbl, GATT_NUM_ATTRS(bluetoothServiceAttrTbl),
				INVALID_TASK_ID, bluetoothService_ReadAttrCB);
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;
	default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*
 * BluetoothService_GetParameter - Get a BluetoothService parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t BluetoothService_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case BLUETOOTHSERVICE_WE0_I:
	  memcpy(value, bluetoothService_WE0_val_I, BLUETOOTHSERVICE_WE0_LEN_I);
	  break;
	case BLUETOOTHSERVICE_WE0_O:
	  memcpy(value, bluetoothService_WE0_val_O, BLUETOOTHSERVICE_WE0_LEN_O);
	  break;
	case BLUETOOTHSERVICE_WE1_I:
		memcpy(value, bluetoothService_WE1_val_I, BLUETOOTHSERVICE_WE1_LEN_I);
		break;
	case BLUETOOTHSERVICE_WE1_O:
		memcpy(value, bluetoothService_WE1_val_O, BLUETOOTHSERVICE_WE1_LEN_O);
		break;
	case BLUETOOTHSERVICE_WE2_I:
		memcpy(value, bluetoothService_WE2_val_I, BLUETOOTHSERVICE_WE2_LEN_I);
		break;
	case BLUETOOTHSERVICE_WE2_O:
		memcpy(value, bluetoothService_WE2_val_O, BLUETOOTHSERVICE_WE2_LEN_O);
		break;
	case BLUETOOTHSERVICE_WE3_I:
		memcpy(value, bluetoothService_WE3_val_I, BLUETOOTHSERVICE_WE3_LEN_I);
		break;
	case BLUETOOTHSERVICE_WE3_O:
		memcpy(value, bluetoothService_WE3_val_O, BLUETOOTHSERVICE_WE3_LEN_O);
		break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          bluetoothService_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t bluetoothService_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                       uint8 *pValue, uint16 *pLen, uint16 offset,
                                       uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  if (!memcmp(pAttr->type.uuid, bluetoothService_WE0_UUID_I, pAttr->type.len))
  {
	  if (offset > BLUETOOTHSERVICE_WE0_LEN_I)  // Prevent malicious ATT ReadBlob offsets.
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  *pLen = MIN(maxLen, BLUETOOTHSERVICE_WE0_LEN_I - offset);  // Transmit as much as possible
		  memcpy(pValue, pAttr->pValue + offset, *pLen);
	  }
  }
  else if (!memcmp(pAttr->type.uuid, bluetoothService_WE0_UUID_O, pAttr->type.len))
  {
	  if (offset > BLUETOOTHSERVICE_WE0_LEN_O)  // Prevent malicious ATT ReadBlob offsets.
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  *pLen = MIN(maxLen, BLUETOOTHSERVICE_WE0_LEN_O - offset);  // Transmit as much as possible
		  memcpy(pValue, pAttr->pValue + offset, *pLen);
	  }
  }
  else if (!memcmp(pAttr->type.uuid, bluetoothService_WE1_UUID_I, pAttr->type.len))
  {
	  if (offset > BLUETOOTHSERVICE_WE1_LEN_I)  // Prevent malicious ATT ReadBlob offsets.
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  *pLen = MIN(maxLen, BLUETOOTHSERVICE_WE1_LEN_I - offset);  // Transmit as much as possible
		  memcpy(pValue, pAttr->pValue + offset, *pLen);
	  }
  }
  else if (!memcmp(pAttr->type.uuid, bluetoothService_WE1_UUID_O, pAttr->type.len))
  {
	  if (offset > BLUETOOTHSERVICE_WE1_LEN_O)  // Prevent malicious ATT ReadBlob offsets.
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  *pLen = MIN(maxLen, BLUETOOTHSERVICE_WE1_LEN_O - offset);  // Transmit as much as possible
		  memcpy(pValue, pAttr->pValue + offset, *pLen);
	  }
  }
  else if (!memcmp(pAttr->type.uuid, bluetoothService_WE2_UUID_I, pAttr->type.len))
  {
	  if (offset > BLUETOOTHSERVICE_WE2_LEN_I)  // Prevent malicious ATT ReadBlob offsets.
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  *pLen = MIN(maxLen, BLUETOOTHSERVICE_WE2_LEN_I - offset);  // Transmit as much as possible
		  memcpy(pValue, pAttr->pValue + offset, *pLen);
	  }
  }
  else if (!memcmp(pAttr->type.uuid, bluetoothService_WE2_UUID_O, pAttr->type.len))
  {
	  if (offset > BLUETOOTHSERVICE_WE2_LEN_O)  // Prevent malicious ATT ReadBlob offsets.
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  *pLen = MIN(maxLen, BLUETOOTHSERVICE_WE2_LEN_O - offset);  // Transmit as much as possible
		  memcpy(pValue, pAttr->pValue + offset, *pLen);
	  }
  }
  else if (!memcmp(pAttr->type.uuid, bluetoothService_WE3_UUID_I, pAttr->type.len))
  {
	  if (offset > BLUETOOTHSERVICE_WE3_LEN_I)  // Prevent malicious ATT ReadBlob offsets.
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  *pLen = MIN(maxLen, BLUETOOTHSERVICE_WE3_LEN_I - offset);  // Transmit as much as possible
		  memcpy(pValue, pAttr->pValue + offset, *pLen);
	  }
  }
  else if (!memcmp(pAttr->type.uuid, bluetoothService_WE3_UUID_O, pAttr->type.len))
  {
	  if (offset > BLUETOOTHSERVICE_WE3_LEN_O)  // Prevent malicious ATT ReadBlob offsets.
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  *pLen = MIN(maxLen, BLUETOOTHSERVICE_WE3_LEN_O - offset);  // Transmit as much as possible
		  memcpy(pValue, pAttr->pValue + offset, *pLen);
	  }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}


/*********************************************************************
 * @fn      bluetoothService_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t bluetoothService_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                        uint8 *pValue, uint16 len, uint16 offset,
                                        uint8 method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (paramID != 0xFF)
    if ( pAppCBs && pAppCBs->pfnChangeCb )
      pAppCBs->pfnChangeCb( paramID ); // Call app function from stack task context.

  return status;
}

/**********************************************************************************************
 * Filename:       userInterface.c
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

#include "userInterface.h"

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

// userInterface Service UUID
CONST uint8_t userInterfaceUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(USERINTERFACE_SERV_UUID), HI_UINT16(USERINTERFACE_SERV_UUID)
};




/*@@@userInterface@@@*/

//weSelect(W/R)

CONST uint8_t userInterface_weSelect_UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(USERINTERFACE_WESELECT_UUID)
};

// Characteristic "userInterface_weSelect" Properties (for declaration)
static uint8_t userInterface_weSelect_props = GATT_PROP_READ | GATT_PROP_WRITE| GATT_PROP_NOTIFY;

// Characteristic "userInterface_weSelect" Value variable
static uint8_t userInterface_weSelect_val[USERINTERFACE_WESELECT_LEN] = { 0 };

// Characteristic "userInterface_weSelect" CCCD
static gattCharCfg_t* userInterface_weSelect_CCCD;

// weSelect User Description
static uint8 userInterface_weSelect_UserDis[20] = "Working Electrode: ";



//Gain(W/R)

CONST uint8_t userInterface_Gain_UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(USERINTERFACE_GAIN_UUID)
};

// Characteristic "userInterface_Gain" Properties (for declaration)
static uint8_t userInterface_Gain_props = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// Characteristic "userInterface_Gain" Value variable
static uint8_t userInterface_Gain_val[USERINTERFACE_GAIN_LEN] = { 0 };

// Characteristic "userInterface_Gain" CCCD
static gattCharCfg_t* userInterface_Gain_CCCD;

//  Gain User Description
static uint8 userInterface_Gain_UserDis[16] = "Internal Gain: ";



/*@@@@2019/08/06@@@@@*/

//INPUT SELECTION(W/R)

CONST uint8_t userInterface_Input_UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(USERINTERFACE_INPUT_UUID)
};

// Characteristic "userInterface_Input" Properties (for declaration)
static uint8_t userInterface_Input_props = GATT_PROP_READ | GATT_PROP_WRITE; //| GATT_PROP_NOTIFY;

// Characteristic "userInterface_Input" Value variable
static uint8_t userInterface_Input_val[USERINTERFACE_INPUT_LEN] = { 0 };

// Characteristic "userInterface_Input" CCCD
static gattCharCfg_t* userInterface_Input_CCCD;

//  Input User Description
static uint8 userInterface_Input_UserDis[54] = "Please select input signal ( 0:CV 1:CA 2:DPV 3:pH ): ";

/*@@@@2019/08/06@@@@@*/




/*@@@@2019/08/14@@@@@*/

//Measuring(W/R)

CONST uint8_t userInterface_Measuring_UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(USERINTERFACE_MEASURING_UUID)
};

// Characteristic "userInterface_Measuring" Properties (for declaration)
static uint8_t userInterface_Measuring_props = GATT_PROP_READ | GATT_PROP_NOTIFY | GATT_PROP_WRITE;

// Characteristic "userInterface_Measuring" Value variable
static uint8_t userInterface_Measuring_val[USERINTERFACE_MEASURING_LEN] = { 0 };

// Characteristic "userInterface_Measuring" CCCD
static gattCharCfg_t* userInterface_Measuring_CCCD;

//  Measuring User Description
static uint8 userInterface_Measuring_UserDis[25] = "Measuring or not (1/0): ";


/*@@@@2019/08/14@@@@@*/

//Modecn(R)
CONST uint8_t userInterface_Modecn_UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(USERINTERFACE_MODECN_UUID)
};

// Characteristic "userInterface_Modecn" Properties (for declaration)
static uint8_t userInterface_Modecn_props = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "userInterface_Modecn" Value variable
static uint8_t userInterface_Modecn_val[USERINTERFACE_MODECN_LEN] = { 0 };

// Characteristic "userInterface_Modecn" CCCD
static gattCharCfg_t* userInterface_Modecn_CCCD;

//  Modecn User Description
static uint8 userInterface_Modecn_UserDis[11] = "LMP MODE: ";



/*@@@userInterface@@@*/



/*********************************************************************
 * LOCAL VARIABLES
 */

static userInterfaceCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t userInterfaceDecl = { ATT_BT_UUID_SIZE, userInterfaceUUID };


/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t userInterfaceAttrTbl[] =
{
  // userInterface Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&userInterfaceDecl
  },

    //weSelect(W/R)
	//  Declaration
	{
	  { ATT_BT_UUID_SIZE, characterUUID },
	  GATT_PERMIT_READ,
	  0,
	  &userInterface_weSelect_props
	},

	//   Value 
	{
	  { ATT_UUID_SIZE, userInterface_weSelect_UUID },
	  GATT_PERMIT_READ | GATT_PERMIT_WRITE,
	  0,
	  userInterface_weSelect_val
	},

	// CCCD
	{
	  { ATT_BT_UUID_SIZE, clientCharCfgUUID },
	  GATT_PERMIT_READ | GATT_PERMIT_WRITE,
	  0,
	  (uint8*)& userInterface_weSelect_CCCD
	},

	//  User Description
	{
	  { ATT_BT_UUID_SIZE, charUserDescUUID },
	  GATT_PERMIT_READ,
	  0,
	  userInterface_weSelect_UserDis
	},


	//Internal Gain(W/R)
	//  Declaration
	{
	  { ATT_BT_UUID_SIZE, characterUUID },
	  GATT_PERMIT_READ,
	  0,
	  &userInterface_Gain_props
	},

	//   Value 
	{
	  { ATT_UUID_SIZE, userInterface_Gain_UUID },
	  GATT_PERMIT_READ | GATT_PERMIT_WRITE,
	  0,
	  userInterface_Gain_val
	},

	// CCCD
	{
	  { ATT_BT_UUID_SIZE, clientCharCfgUUID },
	  GATT_PERMIT_READ | GATT_PERMIT_WRITE,
	  0,
	  (uint8*)& userInterface_Gain_CCCD
	},

	//  User Description
	{
	  { ATT_BT_UUID_SIZE, charUserDescUUID },
	  GATT_PERMIT_READ,
	  0,
	  userInterface_Gain_UserDis
	},



	/*@@@@2019/08/06@@@@@*/

	//INPUT SELECTION(W/R)
	//  Declaration
	{
	  { ATT_BT_UUID_SIZE, characterUUID },
	  GATT_PERMIT_READ,
	  0,
	  &userInterface_Input_props
	},

	//   Value 
	{
	  { ATT_UUID_SIZE, userInterface_Input_UUID },
	  GATT_PERMIT_READ | GATT_PERMIT_WRITE,
	  0,
	  userInterface_Input_val
	},

	// CCCD
	{
	  { ATT_BT_UUID_SIZE, clientCharCfgUUID },
	  GATT_PERMIT_READ | GATT_PERMIT_WRITE,
	  0,
	  (uint8*)& userInterface_Input_CCCD
	},

	//  User Description
	{
	  { ATT_BT_UUID_SIZE, charUserDescUUID },
	  GATT_PERMIT_READ,
	  0,
	  userInterface_Input_UserDis
	},


	/*@@@@2019/08/06@@@@@*/


	/*@@@@2019/08/14@@@@@*/

	//Measuring(R)
	//  Declaration
	{
	  { ATT_BT_UUID_SIZE, characterUUID },
	  GATT_PERMIT_READ,
	  0,
	  &userInterface_Measuring_props
	},

	//   Value 
	{
	  { ATT_UUID_SIZE, userInterface_Measuring_UUID },
	  GATT_PERMIT_READ | GATT_PERMIT_WRITE,
	  0,
	  userInterface_Measuring_val
	},

	// CCCD
	{
	  { ATT_BT_UUID_SIZE, clientCharCfgUUID },
	  GATT_PERMIT_READ | GATT_PERMIT_WRITE,
	  0,
	  (uint8*)& userInterface_Measuring_CCCD
	},

	//  User Description
	{
	  { ATT_BT_UUID_SIZE, charUserDescUUID },
	  GATT_PERMIT_READ,
	  0,
	  userInterface_Measuring_UserDis
	},

	/*@@@@2019/08/14@@@@@*/


	//Modecn(R)
	//  Declaration
	{
	  { ATT_BT_UUID_SIZE, characterUUID },
	  GATT_PERMIT_READ,
	  0,
	  &userInterface_Modecn_props
	},

		  //   Value 
	{
	  { ATT_UUID_SIZE, userInterface_Modecn_UUID },
	  GATT_PERMIT_READ,
	  0,
	  userInterface_Modecn_val
	},

		  // CCCD
	{
	  { ATT_BT_UUID_SIZE, clientCharCfgUUID },
	  GATT_PERMIT_READ | GATT_PERMIT_WRITE,
	  0,
	  (uint8*)& userInterface_Modecn_CCCD
	},

		  //  User Description
	{
	  { ATT_BT_UUID_SIZE, charUserDescUUID },
	  GATT_PERMIT_READ,
	  0,
	  userInterface_Modecn_UserDis
	},


};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t userInterface_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 *pLen, uint16 offset,
                                           uint16 maxLen, uint8 method );
static bStatus_t userInterface_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint16 len, uint16 offset,
                                            uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t userInterfaceCBs =
{
  userInterface_ReadAttrCB,  // Read callback function pointer
  userInterface_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * UserInterface_AddService- Initializes the UserInterface service by registering
 *          GATT attributes with the GATT server.
 *
 */
bStatus_t UserInterface_AddService( void )
{
  uint8_t status;

  //userInterface_weSelect
  userInterface_weSelect_CCCD = (gattCharCfg_t*)ICall_malloc(sizeof(gattCharCfg_t) *
	  linkDBNumConns);
  if (userInterface_weSelect_CCCD == NULL)
  {
	  return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, userInterface_weSelect_CCCD);



  //userInterface_Gain
  userInterface_Gain_CCCD = (gattCharCfg_t*)ICall_malloc(sizeof(gattCharCfg_t) *
	  linkDBNumConns);
  if (userInterface_Gain_CCCD == NULL)
  {
	  return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, userInterface_Gain_CCCD);



  /*@@@@2019/08/06@@@@@*/

  //userInterface_Input
  userInterface_Input_CCCD = (gattCharCfg_t*)ICall_malloc(sizeof(gattCharCfg_t) *
	  linkDBNumConns);
  if (userInterface_Input_CCCD == NULL)
  {
	  return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, userInterface_Input_CCCD);

  /*@@@@2019/08/06@@@@@*/


  //userInterface_Measuring
  userInterface_Measuring_CCCD = (gattCharCfg_t*)ICall_malloc(sizeof(gattCharCfg_t) *
	  linkDBNumConns);
  if (userInterface_Measuring_CCCD == NULL)
  {
	  return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, userInterface_Measuring_CCCD);




  //userInterface_Modecn
  userInterface_Modecn_CCCD = (gattCharCfg_t*)ICall_malloc(sizeof(gattCharCfg_t) *
	  linkDBNumConns);
  if (userInterface_Modecn_CCCD == NULL)
  {
	  return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, userInterface_Modecn_CCCD);



  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( userInterfaceAttrTbl,
                                        GATT_NUM_ATTRS( userInterfaceAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &userInterfaceCBs );

  return ( status );
}

/*
 * UserInterface_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t UserInterface_RegisterAppCBs( userInterfaceCBs_t *appCallbacks )
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
 * UserInterface_SetParameter - Set a UserInterface parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t UserInterface_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case USERINTERFACE_WESELECT:
	  if (len == USERINTERFACE_WESELECT_LEN)
	  {
		  memcpy(userInterface_weSelect_val, value, len);

		  // Try to send notification.
		  GATTServApp_ProcessCharCfg(userInterface_weSelect_CCCD, (uint8_t*)& userInterface_weSelect_val, FALSE,
			  userInterfaceAttrTbl, GATT_NUM_ATTRS(userInterfaceAttrTbl),
			  INVALID_TASK_ID, userInterface_ReadAttrCB);
	  }
	  else
	  {
		  ret = bleInvalidRange;
	  }
	  break;
	case USERINTERFACE_GAIN:
		if (len == USERINTERFACE_GAIN_LEN)
		{
			memcpy(userInterface_Gain_val, value, len);

			// Try to send notification.
			GATTServApp_ProcessCharCfg(userInterface_Gain_CCCD, (uint8_t*)& userInterface_Gain_val, FALSE,
				userInterfaceAttrTbl, GATT_NUM_ATTRS(userInterfaceAttrTbl),
				INVALID_TASK_ID, userInterface_ReadAttrCB);
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;
	/*@@@@2019/08/06@@@@@*/
	case USERINTERFACE_INPUT:
		if (len == USERINTERFACE_INPUT_LEN)
		{
			memcpy(userInterface_Input_val, value, len);

			// Try to send notification.
			GATTServApp_ProcessCharCfg(userInterface_Input_CCCD, (uint8_t*)& userInterface_Input_val, FALSE,
				userInterfaceAttrTbl, GATT_NUM_ATTRS(userInterfaceAttrTbl),
				INVALID_TASK_ID, userInterface_ReadAttrCB);
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;
	/*@@@@2019/08/06@@@@@*/
	case USERINTERFACE_MEASURING:
		if (len == USERINTERFACE_MEASURING_LEN)
		{
			memcpy(userInterface_Measuring_val, value, len);

			// Try to send notification.
			GATTServApp_ProcessCharCfg(userInterface_Measuring_CCCD, (uint8_t*)& userInterface_Measuring_val, FALSE,
				userInterfaceAttrTbl, GATT_NUM_ATTRS(userInterfaceAttrTbl),
				INVALID_TASK_ID, userInterface_ReadAttrCB);
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;
	case USERINTERFACE_MODECN:
		if (len == USERINTERFACE_MODECN_LEN)
		{
			memcpy(userInterface_Modecn_val, value, len);

			// Try to send notification.
			GATTServApp_ProcessCharCfg(userInterface_Modecn_CCCD, (uint8_t*)& userInterface_Modecn_val, FALSE,
				userInterfaceAttrTbl, GATT_NUM_ATTRS(userInterfaceAttrTbl),
				INVALID_TASK_ID, userInterface_ReadAttrCB);
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
 * UserInterface_GetParameter - Get a UserInterface parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t UserInterface_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
  case USERINTERFACE_WESELECT:
	  memcpy(value, userInterface_weSelect_val, USERINTERFACE_WESELECT_LEN);
	  break;
  case USERINTERFACE_GAIN:
	  memcpy(value, userInterface_Gain_val, USERINTERFACE_GAIN_LEN);
	  break;
	  /*@@@@2019/08/06@@@@@*/
  case USERINTERFACE_INPUT:
	  memcpy(value, userInterface_Input_val, USERINTERFACE_INPUT_LEN);
	  break;
	  /*@@@@2019/08/06@@@@@*/
  case USERINTERFACE_MEASURING:
	  memcpy(value, userInterface_Measuring_val, USERINTERFACE_MEASURING_LEN);
	  break;
  case USERINTERFACE_MODECN:
	  memcpy(value, userInterface_Modecn_val, USERINTERFACE_MODECN_LEN);
	  break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          userInterface_ReadAttrCB
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
static bStatus_t userInterface_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                       uint8 *pValue, uint16 *pLen, uint16 offset,
                                       uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;
  if (!memcmp(pAttr->type.uuid, userInterface_weSelect_UUID, pAttr->type.len))
  {
	  if (offset > USERINTERFACE_WESELECT_LEN)  // Prevent malicious ATT ReadBlob offsets.
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  *pLen = MIN(maxLen, USERINTERFACE_WESELECT_LEN - offset);  // Transmit as much as possible
		  memcpy(pValue, pAttr->pValue + offset, *pLen);
	  }
  }
  else if (!memcmp(pAttr->type.uuid, userInterface_Gain_UUID, pAttr->type.len))
  {
	  if (offset > USERINTERFACE_GAIN_LEN)  // Prevent malicious ATT ReadBlob offsets.
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  *pLen = MIN(maxLen, USERINTERFACE_GAIN_LEN - offset);  // Transmit as much as possible
		  memcpy(pValue, pAttr->pValue + offset, *pLen);
	  }
  }
  /*@@@@2019/08/06@@@@@*/
  else if (!memcmp(pAttr->type.uuid, userInterface_Input_UUID, pAttr->type.len))
  {
	  if (offset > USERINTERFACE_INPUT_LEN)  // Prevent malicious ATT ReadBlob offsets.
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  *pLen = MIN(maxLen, USERINTERFACE_INPUT_LEN - offset);  // Transmit as much as possible
		  memcpy(pValue, pAttr->pValue + offset, *pLen);
	  }
  }


  /*@@@@2019/08/06@@@@@*/
  else if (!memcmp(pAttr->type.uuid, userInterface_Measuring_UUID, pAttr->type.len))
  {
	  if (offset > USERINTERFACE_MEASURING_LEN)  // Prevent malicious ATT ReadBlob offsets.
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  *pLen = MIN(maxLen, USERINTERFACE_MEASURING_LEN - offset);  // Transmit as much as possible
		  memcpy(pValue, pAttr->pValue + offset, *pLen);
	  }
  }
  else if (!memcmp(pAttr->type.uuid, userInterface_Modecn_UUID, pAttr->type.len))
  {
	  if (offset > USERINTERFACE_MODECN_LEN)  // Prevent malicious ATT ReadBlob offsets.
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  *pLen = MIN(maxLen, USERINTERFACE_MODECN_LEN - offset);  // Transmit as much as possible
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
 * @fn      userInterface_WriteAttrCB
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
static bStatus_t userInterface_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
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
  else if (!memcmp(pAttr->type.uuid, userInterface_weSelect_UUID, pAttr->type.len))
  {
	  if (offset + len > USERINTERFACE_WESELECT_LEN)
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  // Copy pValue into the variable we point to from the attribute table.
		  memcpy(pAttr->pValue + offset, pValue, len);

		  // Only notify application if entire expected value is written
		  //if (offset + len == USERINTERFACE_WESELECT_LEN)
			  paramID = USERINTERFACE_WESELECT;
	  }
  }
  else if (!memcmp(pAttr->type.uuid, userInterface_Gain_UUID, pAttr->type.len))
  {
	  if (offset + len > USERINTERFACE_GAIN_LEN)
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  // Copy pValue into the variable we point to from the attribute table.
		  memcpy(pAttr->pValue + offset, pValue, len);

		  // Only notify application if entire expected value is written
		  //if (offset + len == USERINTERFACE_GAIN_LEN)
			  paramID = USERINTERFACE_GAIN;
	  }
  }
  /*@@@@2019/08/06@@@@@*/
  else if (!memcmp(pAttr->type.uuid, userInterface_Input_UUID, pAttr->type.len))
  {
	  if (offset + len > USERINTERFACE_INPUT_LEN)
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  // Copy pValue into the variable we point to from the attribute table.
		  memcpy(pAttr->pValue + offset, pValue, len);

		  // Only notify application if entire expected value is written
		  //if (offset + len == USERINTERFACE_GAIN_LEN)
		  paramID = USERINTERFACE_INPUT;
	  }
  }
	/*@@@@2019/08/06@@@@@*/


	/*@@@@2019/08/14@@@@@*/
  else if (!memcmp(pAttr->type.uuid, userInterface_Measuring_UUID, pAttr->type.len))
  {
	  if (offset + len > USERINTERFACE_MEASURING_LEN)
	  {
		  status = ATT_ERR_INVALID_OFFSET;
	  }
	  else
	  {
		  // Copy pValue into the variable we point to from the attribute table.
		  memcpy(pAttr->pValue + offset, pValue, len);

		  // Only notify application if entire expected value is written
		  //if (offset + len == USERINTERFACE_GAIN_LEN)
		  paramID = USERINTERFACE_MEASURING;
	  }
  }
	/*@@@@2019/08/14@@@@@*/
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

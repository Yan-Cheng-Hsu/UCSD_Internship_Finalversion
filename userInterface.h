/**********************************************************************************************
 * Filename:       userInterface.h
 *
 * Description:    This file contains the userInterface service definitions and
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


#ifndef _USERINTERFACE_H_
#define _USERINTERFACE_H_

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
#define USERINTERFACE_SERV_UUID 0xBABE


//WESELECT
#define USERINTERFACE_WESELECT 0
#define USERINTERFACE_WESELECT_UUID 0x3131
#define USERINTERFACE_WESELECT_LEN 2

//MEASURING 
#define USERINTERFACE_MEASURING 1
#define USERINTERFACE_MEASURING_UUID 0x3231
#define USERINTERFACE_MEASURING_LEN 2

//GAIN
#define USERINTERFACE_GAIN 2
#define USERINTERFACE_GAIN_UUID 0x3331
#define USERINTERFACE_GAIN_LEN 2

//MODECN OF LMP
#define USERINTERFACE_MODECN 3
#define USERINTERFACE_MODECN_UUID 0x3431
#define USERINTERFACE_MODECN_LEN 2


/*@@@@2019/08/06@@@@@*/
//INPUT SELECTION
#define USERINTERFACE_INPUT 4
#define USERINTERFACE_INPUT_UUID 0x3531
#define USERINTERFACE_INPUT_LEN 2
/*@@@@2019/08/06@@@@@*/







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
typedef void (*userInterfaceChange_t)( uint8 paramID );

typedef struct
{
  userInterfaceChange_t        pfnChangeCb;  // Called when characteristic value changes
} userInterfaceCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * UserInterface_AddService- Initializes the UserInterface service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t UserInterface_AddService( void );

/*
 * UserInterface_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t UserInterface_RegisterAppCBs( userInterfaceCBs_t *appCallbacks );

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
extern bStatus_t UserInterface_SetParameter( uint8 param, uint8 len, void *value );

/*
 * UserInterface_GetParameter - Get a UserInterface parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t UserInterface_GetParameter( uint8 param, void *value );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _USERINTERFACE_H_ */

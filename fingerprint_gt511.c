/******************************************************************************
 *
 * fingerprint_gt511.c - Driver API for GT-511C fingerprint sensor.
 *
 * Copyright (c) 2015, Joseph Kroesche (kroesche.org)
 * All rights reserved.
 *
 * This software is released under the FreeBSD license, found in the
 * accompanying file LICENSE.txt and at the following URL:
 *      http://www.freebsd.org/copyright/freebsd-license.html
 *
 * This software is provided as-is and without warranty.
 *
 *****************************************************************************/

// Library headers
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Module headers
#include "fingerprint_gt511.h"

/**
 * @addtogroup gt511_driver Driver for GT-511C Fingerprint Sensor
 *
 * This is a driver for communicating with and controlling a GT-511C
 * fingerprint scanner.  It is specifically for a GT-511C1R which is
 * the version that holds only 20 fingerprints.  However it should also
 * work for the GT-511C3 although it has not been tested.  These fingerprint
 * sensors are relatively inexpensive and available from places like
 * (Sparkfun)[http://www.sparkfun.com/].
 *
 * This particular driver was been written for a Silicon Labs EFM32 Wonder
 * Gecko.
 * Link: http://www.silabs.com/products/mcu/lowpower/Pages/efm32wg-wonder-gecko.aspx
 * However it was designed to be hardware and application indepedent so it
 * should be usable on any modern microcontroller, especially any based on
 * Cortex-M.
 *
 * ## Features ##
 *
 * - direct functions to map to each GT-511C command
 * - macro functions for performing enrollment and identification
 * - abstracted hardware interface using application-provided read and
 *   write functions
 * - callback functions to inform application of progress and status
 *
 * Please refer to the GT-511C sensor data sheet to understand how
 * the functions of this driver should be used.
 *
 * ## Hardware Abstraction ##
 *
 * To avoid the need to code to any particular kind of hardware, this
 * driver uses application-provided functions to write and read messages
 * to the GT-511C hardware.  Normally this is a serial port of the
 * host microcontroller.  The application must implement the following
 * functions:
 *
 * - GT511_SendMessage()
 * - GT511_ReceiveMessage()
 *
 * These can be as simple as polling-loop functions to copy bytes to and
 * from the serial port, or more sophisticated that utilize interrupts
 * and buffered IO with timeouts.
 *
 * The driver also needs a way to check for a timeout.  So the applciation
 * must also implement the following functions to support a timeout:
 *
 * - GT511_SetTimeout()
 * - GT511_CheckTimeout()
 *
 * The application design can use these in any way needed.  It can create
 * different timeout periods based on the context, or just use one timeout
 * for all cases, or not use any timeouts.  The driver will call these
 * functions to setup a timeout when it is about to do something that
 * requires a wait (such as waiting for a person to touch the sensor).  This
 * allows the application to control how long the driver will wait for such
 * events to occur.
 *
 * ## Application Callback ##
 *
 * The driver will call an application-provided callback function to supply
 * status.  The function is named GT511_UserCallback().  This will be called
 * at various points during driver operation and can be used by the
 * application to know at what points it is appropriate to prompt the user
 * to do some action such as touch the sensor.
 *
 * ## Typical API Usage ##
 *
 * All of the functions are written using the same style and all have the
 * same return type GT511_Error_t to indicate the success or error code
 * for an operation.  When any operation is successful the return code will
 * be GT511_ERR_NONE.  The remaining error codes map to codes returned
 * from the GT-511C, or GT511_ERR_OTHER_ERROR for all other error conditions
 * (such as a communication error).  If the function needs to return a
 * value, that is done through pointer function parameters.
 *
 * __Example__
 *
 * ~~~~~~~~.c
 * // example to open and init the fingerprint sensor
 *
 * // Structure to hold returned sensor info
 * GT511_Info_t sensorInfo;
 *
 * // call function to open the sensor
 * GT511_Error_t err = GT511_Open(&sensorInfo);
 *
 * // check for errors
 * if (err != GT511_ERR_NONE)
 * {
 *     // open was not successful
 *     // so handle error
 * }
 * else
 * {
 *     // open was successful and sensor info is now contained
 *     // in sensorInfo structure
 *     ...
 * }
 * ~~~~~~~~
 * @{
 */

/**
 * Set the number of supported fingerprint slots.
 * This should match the capability of your sensor hardware.
 */
#ifndef GT511_NUM_SLOTS
#define GT511_NUM_SLOTS 20
#endif

/******************************************************************************
 * Private/local data and functions
 *****************************************************************************/

// enable debug print messages depending on DEBUG build
// This will try to use printf as console output if DEBUG is defined
#ifdef DEBUG
#define ConsolePrintf(...) printf(__VA_ARGS__)
#else
#define ConsolePrintf(...)
#endif

// Define the GT-511C command set.
// See the data sheet for descriptions.  Not all commands are still
// supported by the sensor.
typedef enum
{
    GT511_CMD_OPEN                  = 0x01,
    GT511_CMD_CLOSE                 = 0x02,
    GT511_CMD_USB_INTERNAL_CHECK    = 0x03,
    GT511_CMD_CHANGE_BAUDRATE       = 0x04,
    GT511_CMD_SET_IAP_MODE          = 0x05,
    GT511_CMD_CMOS_LED              = 0x12,
    GT511_CMD_GET_ENROLL_COUNT      = 0x20,
    GT511_CMD_CHECK_ENROLLED        = 0x21,
    GT511_CMD_ENROLL_START          = 0x22,
    GT511_CMD_ENROLL1               = 0x23,
    GT511_CMD_ENROLL2               = 0x24,
    GT511_CMD_ENROLL3               = 0x25,
    GT511_CMD_IS_PRESS_FINGER       = 0x26,
    GT511_CMD_DELETE_ID             = 0x40,
    GT511_CMD_DELETE_ALL            = 0x41,
    GT511_CMD_VERIFY                = 0x50,
    GT511_CMD_IDENTIFY              = 0x51,
    GT511_CMD_VERIFY_TEMPLATE       = 0x52,
    GT511_CMD_IDENTIFY_TEMPLATE     = 0x53,
    GT511_CMD_CAPTURE_FINGER        = 0x60,
    GT511_CMD_MAKE_TEMPLATE         = 0x61,
    GT511_CMD_GET_IMAGE             = 0x62,
    GT511_CMD_GET_RAW_IMAGE         = 0x63,
    GT511_CMD_GET_TEMPLATE          = 0x70,
    GT511_CMD_SET_TEMPLATE          = 0x71,
    GT511_CMD_UPGRADE_FIRMWARE      = 0x80,
} GT511_Command_t;

// ACK/NACK codes for response packets.
typedef enum
{
    GT511_RESP_ACK = 0x30,
    GT511_RESP_NACK = 0x31
} GT511_Response_t;

// GT-511C packet format.  This is for command packets and
// response packets.
typedef struct
{
    uint8_t start1;
    uint8_t start2;
    uint16_t id;
    uint32_t parameter;
    uint16_t command;
    uint16_t checksum;
} GT511_Packet_t;

// GT-511C data packet format.  The payload is variable length so
// the actual location of the checksum cannot be coded into the structure.
typedef struct
{
    uint8_t start1;
    uint8_t start2;
    uint16_t id;
    uint8_t payload[2];
    // uint16_t checksum;
} GT511_DataPacket_t;

/*
 * Memory pool for packets used by this module.
 * This memory is shared by the different types of packets.
 * This is enough memory to hold the command packet and response
 * packets (but not at the same time), or the data packet returned
 * from an "open" command (the info packet).  Any other data such as
 * templates will need a separate memory allocation.
 */
static uint8_t mempool[sizeof(GT511_DataPacket_t) + sizeof(GT511_Info_t)];

/*
 * Compute the checksum of a buffer.
 *
 * @param pBuf points at the buffer for checksumming
 * @param length is the number of bytes to checksum
 *
 * @return the sum of the bytes as a 16-bit checksum
 */
static uint16_t
Checksum(uint8_t *pBuf, uint32_t length)
{
    uint32_t sum = 0;
    while (length--)
    {
        sum += *pBuf;
        ++pBuf;
    }

    return sum;
}

/*
 * Test a response packet for validity.
 *
 * @param pResp packet containing a response from the GT511
 *
 * Checks various fields of the response packet for valid values,
 * including the checksum calculation.
 *
 * @return **true** if all fields look okay, **false** if any problems
 * are found.
 */
static bool
IsValidResponse(GT511_Packet_t *pResp)
{
    // Test the checksum
    uint16_t computedChecksum = Checksum((uint8_t *)pResp, sizeof(GT511_Packet_t) - 2);
    if (computedChecksum != pResp->checksum)
    {
        return false;
    }

    // Check the packet start bytes
    if ((pResp->start1 != 0x55) || (pResp->start2 != 0xAA))
    {
        return false;
    }

    // Check the ID which is always 1
    if (pResp->id != 1)
    {
        return false;
    }

    // Check the response field to make sure it is one of two possible values
    if ((pResp->command != GT511_RESP_ACK) && (pResp->command != GT511_RESP_NACK))
    {
        return false;
    }

    // No trouble found, so return valid indication
    return true;
}

/*
 * Issue a command and check response.
 *
 * @param command specific GT511 command code to send to reader
 * @param pParameter points at storage for command and response parameter
 *
 * Prepares a command packet and sends it to the GT511 reader.  It then
 * receives a response packet and validates it.  If everything is valid
 * and there is an ACK response, then the parameter can be returned.
 * The argument _pParameter_ is used for both passing in a parameter to be
 * used for the command, and to return a parameter from the response, if
 * any.  This argument can be NULL in which case 0 will be used for the
 * command parameter and no response parameter can be returned.
 *
 * @return **GT511_ERR_NONE** if the command is sent successfully and a
 * correct response is received with an ACK.  If the response contains a
 * NACK, then the response error code is returned.  If any other error
 * occurs (bad communication, invalid packet, etc) then
 * **GT511_ERR_OTHER_ERROR** is returned.
 */
static GT511_Error_t
IssueCommand(uint16_t command, uint32_t *pParameter)
{
    // Prepare a command packet
    GT511_Packet_t *pCmd = (GT511_Packet_t *)mempool;
    pCmd->start1 = 0x55;
    pCmd->start2 = 0xAA;
    pCmd->id = 1;
    pCmd->parameter = (pParameter != NULL) ? *pParameter : 0;
    pCmd->command = command;
    pCmd->checksum = Checksum((uint8_t *)pCmd, sizeof(GT511_Packet_t) - 2);

    // Send the prepared command and check for send error
    bool ok = GT511_SendMessage((uint8_t *)pCmd, sizeof(GT511_Packet_t));
    if (!ok)
    {
        return GT511_ERR_OTHER_ERROR;
    }

    // Try to receive a response packet
    GT511_Packet_t *pResp = (GT511_Packet_t *)mempool;
    uint32_t respCount = GT511_ReceiveMessage((uint8_t *)pResp, sizeof(GT511_Packet_t));
    if (respCount != sizeof(GT511_Packet_t))
    {
        return GT511_ERR_OTHER_ERROR;
    }

    // We got a response, so now validate it
    ok = IsValidResponse(pResp);
    if (!ok)
    {
        return GT511_ERR_OTHER_ERROR;
    }

    // Response if valid, check for NACK
    if (pResp->command == GT511_RESP_NACK)
    {
        // Return the error code for the NACK
        return (GT511_Error_t)pResp->parameter;
    }

    // otherwise there was an ACK
    else
    {
        // return response parameter to caller, if needed
        if (pParameter != NULL)
        {
            *pParameter = pResp->parameter;
        }

        return GT511_ERR_NONE;
    }
}

/*
 * Wait for user to touch finger to sensor.
 *
 * @param mode the current mode of the driver (identify, enroll, etc)
 *
 * This function will use the GT511_UserCallback() to prompt the user
 * to press the sensor, and to then wait for the user to press the
 * sensor.  At the start GT511_SetTimeout() will be called to allow the
 * application set a timeout if needed.  While waiting then
 * GT511_CheckTimeout() will be repeatedly called to see if a timeout
 * occurs.  This function will wait until either it correctly detects a
 * finger touch, or the timeout occurs, or some other error happens.
 *
 * @return **GT511_ERR_NONE** if a touch was detected.  If the timeout
 * happens then it will return **GT511_ERR_OTHER_ERROR**.  If any other
 * error occurs then that code will be returned.  The function was not
 * successful at detecting a touch if anything other than GT511_ERR_NONE
 * is returned.
 */
static GT511_Error_t
WaitFingerPress(GT511_Mode_t mode)
{
    GT511_Error_t err;
    // wait for a finger press
    GT511_UserCallback(mode, GT511_UI_PRESS);
    ConsolePrintf("waiting for touch\n");
    bool isPressed = false;
    GT511_SetTimeout(mode);
    do
    {
        // check for timeout from app
        bool timeout = GT511_CheckTimeout(mode);
        if (timeout)
        {
            ConsolePrintf("touch wait timeout\n");
            GT511_UserCallback(mode, GT511_UI_TIMEOUT);
            return GT511_ERR_OTHER_ERROR;
        }

        // check for finger press
        err = GT511_IsPressFinger(&isPressed);
        if (err != GT511_ERR_NONE)
        {
            GT511_CmosLed(false);
            ConsolePrintf("error checking for finger press: %s\n",
                          GT511_ErrorString(err));
            GT511_UserCallback(mode, GT511_UI_ERROR);
            return err;
        }
    } while (!isPressed);

    ConsolePrintf("touch detected\n");
    return err;
}

/*
 * Wait for user to release sensor touch.
 *
 * @param mode the current mode of the driver (identify, enroll, etc)
 *
 * This function will use the GT511_UserCallback() to prompt the user
 * to release the sensor, and to then wait for the user to release the
 * sensor.  At the start GT511_SetTimeout() will be called to allow the
 * application set a timeout if needed.  While waiting then
 * GT511_CheckTimeout() will be repeatedly called to see if a timeout
 * occurs.  This function will wait until either it correctly detects the
 * touch has been released, or the timeout occurs, or some other error happens.
 *
 * @return **GT511_ERR_NONE** if the touch was released.  If the timeout
 * happens then it will return **GT511_ERR_OTHER_ERROR**.  If any other
 * error occurs then that code will be returned.  The function was not
 * successful at detecting a release if anything other than GT511_ERR_NONE
 * is returned.
 */
static GT511_Error_t
WaitFingerRelease(GT511_Mode_t mode)
{
    GT511_Error_t err;
    // wait for a finger release
    GT511_UserCallback(mode, GT511_UI_RELEASE);
    ConsolePrintf("waiting for release\n");
    bool isPressed = true;
    GT511_SetTimeout(mode);
    do
    {
        // check for timeout from app
        bool timeout = GT511_CheckTimeout(mode);
        if (timeout)
        {
            ConsolePrintf("release wait timeout\n");
            GT511_UserCallback(mode, GT511_UI_TIMEOUT);
            return GT511_ERR_OTHER_ERROR;
        }

        // check for finger press
        err = GT511_IsPressFinger(&isPressed);
        if (err != GT511_ERR_NONE)
        {
            GT511_CmosLed(false);
            ConsolePrintf("error checking for finger press: %s\n",
                          GT511_ErrorString(err));
            GT511_UserCallback(mode, GT511_UI_ERROR);
            return err;
        }
    } while (isPressed);

    ConsolePrintf("release detected\n");
    return err;
}

// Define a table to map error codes to human readable strings
typedef struct
{
    GT511_Error_t errCode;
    const char *errString;
} ErrorStringTable_t;

static const ErrorStringTable_t errorStringTable[] =
{
    { GT511_ERR_NONE, "NONE" },
    { GT511_ERR_TIMEOUT, "TIMEOUT" },
    { GT511_ERR_INVALID_BAUDRATE, "INVALID_BAUDRATE" },
    { GT511_ERR_INVALID_POS, "INVALID_POS" },
    { GT511_ERR_IS_NOT_USED, "IS_NOT_USED" },
    { GT511_ERR_IS_ALREADY_USED, "IS_ALREADY_USED" },
    { GT511_ERR_COMM_ERR, "COMM_ERR" },
    { GT511_ERR_VERIFY_FAILED, "VERIFY_FAILED" },
    { GT511_ERR_IDENTIFY_FAILED, "IDENTIFY_FAILED" },
    { GT511_ERR_DB_IS_FULL, "DB_IS_FULL" },
    { GT511_ERR_DB_IS_EMPTY, "DB_IS_EMPTY" },
    { GT511_ERR_TURN_ERR, "TURN_ERR" },
    { GT511_ERR_BAD_FINGER, "BAD_FINGER" },
    { GT511_ERR_ENROLL_FAILED, "ENROLL_FAILED" },
    { GT511_ERR_IS_NOT_SUPPORTED, "IS_NOT_SUPPORTED" },
    { GT511_ERR_DEV_ERR, "DEV_ERR" },
    { GT511_ERR_CAPTURE_CANCELED, "CAPTURE_CANCELED" },
    { GT511_ERR_INVALID_PARAM, "INVALID_PARAM" },
    { GT511_ERR_FINGER_IS_NOT_PRESSED, "FINGER_IS_NOT_PRESSED" },
    { GT511_ERR_OTHER_ERROR, "OTHER_ERROR" },
};
#define NUM_ERR_STRINGS (sizeof(errorStringTable) / sizeof(ErrorStringTable_t))

/******************************************************************************
 * Public API
 *****************************************************************************/

/**
 * Return string representation of error code.
 *
 * @param err the error code to check
 *
 * This can be used for debugging to print a string name of the error code.
 *
 * @return A string representation of the error code.  If the error code
 * does not match any known value then "UNKNOWN" is returned.
 */
const char *
GT511_ErrorString(GT511_Error_t err)
{
    for (uint32_t i = 0; i < NUM_ERR_STRINGS; i++)
    {
        if (errorStringTable[i].errCode == err)
        {
            return errorStringTable[i].errString;
        }
    }
    return "UNKNOWN";
}

/**
 * Open the fingerprint reader for operation.
 *
 * @param pInfo optional pointer for retrieving sensor info
 *
 * The _pInfo_ parameter is optional and can be NULL.  If not NULL, then
 * it will be populated with the information returned from the sensor.
 *
 * @return **GT511_ERR_NONE** if no errors occurred.
 */
GT511_Error_t
GT511_Open(GT511_Info_t *pInfo)
{
    uint32_t parm = pInfo ? 1 : 0;

    // Send the command and check response
    GT511_Error_t err = IssueCommand(GT511_CMD_OPEN, &parm);
    if (err != GT511_ERR_NONE)
    {
        return err;
    }

    // If user asked for extra info, collect the info data which should
    // be forthcoming
    if (pInfo)
    {
        // Receive a data packet containing the extra info
        GT511_DataPacket_t *pData = (GT511_DataPacket_t *)mempool;
        uint32_t dataLength = sizeof(GT511_DataPacket_t) + sizeof(GT511_Info_t);
        uint32_t dataCount = GT511_ReceiveMessage((uint8_t *)pData, dataLength);
        if (dataCount != dataLength)
        {
            return GT511_ERR_OTHER_ERROR;
        }

        // Copy the extra info fields to the callers storage
        GT511_Info_t *pThisInfo = (GT511_Info_t *)&pData->payload[0];
        pInfo->firmwareVersion = pThisInfo->firmwareVersion;
        pInfo->isoAreaMaxSize = pThisInfo->isoAreaMaxSize;
        memcpy(&pInfo->serialNumber[0], &pThisInfo->serialNumber[0], 16);
    }

    // If we got this far then there are no errors.
    return GT511_ERR_NONE;
}

/**
 * Close the fingerprint reader for operation.
 *
 * @return **GT511_ERR_NONE** if no errors occurred.
 */
GT511_Error_t
GT511_Close(void)
{
    GT511_Error_t err = IssueCommand(GT511_CMD_CLOSE, NULL);
    return err;
}

/**
 * Control the LED backlight.
 *
 * @param on set to **true** to turn on the backlight, or **false** for off
 *
 * @return **GT511_ERR_NONE** if no errors occurred.
 */
GT511_Error_t
GT511_CmosLed(bool on)
{
    uint32_t parm = on ? 1 : 0;
    GT511_Error_t err = IssueCommand(GT511_CMD_CMOS_LED, &parm);
    return err;
}

/**
 * Check to see if a finger is pressed
 *
 * @param pIsPressed storage for returned value
 *
 * The parameter *pIsPressed* points at a flag to contain the returned
 * value.  It will be set to **true** if a finger press is detected, or
 * **false** if no finger press is detected.
 *
 * @return **GT511_ERR_NONE** if no errors occurred.
 */
GT511_Error_t
GT511_IsPressFinger(bool *pIsPressed)
{
    uint32_t parm = 0;
    GT511_Error_t err = IssueCommand(GT511_CMD_IS_PRESS_FINGER, &parm);

    // return requested flag
    // this will have no meaning if err != _NONE
    if (pIsPressed != NULL)
    {
        // return parameter is 0 if pressed, so we need to NOT it
        *pIsPressed = !parm;
    }

    return err;
}

/**
 * Capture a finger print.
 *
 * @param highQuality **true** to use a high quality capture
 *
 * According to the sensor data sheet, a high quality capture should be
 * used for enrollment while a normal capture used for identification.
 *
 * @return **GT511_ERR_NONE** if no errors occurred.
 */
GT511_Error_t
GT511_CaptureFinger(bool highQuality)
{
    uint32_t parm = highQuality ? 1 : 0;
    GT511_Error_t err = IssueCommand(GT511_CMD_CAPTURE_FINGER, &parm);
    return err;
}

/**
 * Identify a fingerprint
 *
 * @param pId pointer to the ID value of the identified fingerprint
 *
 * If a captured fingerprint is successfully identified, then the index ID
 * of the fingerprint will be stored at _*pId_.  You must check the function
 * return value to make sure that the returned ID value is valid.
 *
 * @return **GT511_ERR_NONE** if no errors occurred.
 */
GT511_Error_t
GT511_Identify(uint32_t *pId)
{
    uint32_t parm = 0;
    GT511_Error_t err = IssueCommand(GT511_CMD_IDENTIFY, &parm);

    // Read id from response parameter.  Not meaningful if err != _NONE
    if (pId != NULL)
    {
        *pId = parm;
    }

    return err;
}

/**
 * Start enrollment
 *
 * @param id index ID to use for fingerprint enrollment
 *
 * This will start the enrollment process, using the specified ID index
 * to store the enrolled fingerprint.  This will not work if there is
 * already a fingerprint enrolled at that index.  You can use
 * GT511_CheckEnrolled() to make sure the index is available.
 *
 * @note Please see the sensor data sheet to understand the enrollment process.
 * There are several steps.
 *
 * @return **GT511_ERR_NONE** if no errors occurred.
 */
GT511_Error_t
GT511_EnrollStart(uint32_t id)
{
    GT511_Error_t err = IssueCommand(GT511_CMD_ENROLL_START, &id);
    return err;
}

/**
 * Enroll step 1
 *
 * Enroll a captured fingerprint, first step.
 *
 * @note Please see the sensor data sheet to understand the enrollment process.
 * There are several steps.
 *
 * @return **GT511_ERR_NONE** if no errors occurred.
 */
GT511_Error_t
GT511_Enroll1(void)
{
    GT511_Error_t err = IssueCommand(GT511_CMD_ENROLL1, NULL);
    return err;
}

/**
 * Enroll step 2
 *
 * Enroll a captured fingerprint, second step.
 *
 * @note Please see the sensor data sheet to understand the enrollment process.
 * There are several steps.
 *
 * @return **GT511_ERR_NONE** if no errors occurred.
 */
GT511_Error_t
GT511_Enroll2(void)
{
    GT511_Error_t err = IssueCommand(GT511_CMD_ENROLL2, NULL);
    return err;
}

/**
 * Enroll step 3
 *
 * Enroll a captured fingerprint, third and final step.
 *
 * @note Please see the sensor data sheet to understand the enrollment process.
 * There are several steps.
 *
 * @return **GT511_ERR_NONE** if no errors occurred.
 */
GT511_Error_t
GT511_Enroll3(void)
{
    GT511_Error_t err = IssueCommand(GT511_CMD_ENROLL3, NULL);
    return err;
}

/**
 * Delete all enrolled IDs
 *
 * @return **GT511_ERR_NONE** if no errors occurred.
 */
GT511_Error_t
GT511_DeleteAll(void)
{
    GT511_Error_t err = IssueCommand(GT511_CMD_DELETE_ALL, NULL);
    return err;
}

/**
 * Delete specific enrolled ID
 *
 * @param id index of ID to delete
 *
 * @return **GT511_ERR_NONE** if no errors occurred.
 */
GT511_Error_t
GT511_DeleteID(uint32_t id)
{
    GT511_Error_t err = IssueCommand(GT511_CMD_DELETE_ID, &id);
    return err;
}

/**
 * Get the count of enrolled IDs
 *
 * @param pEnrolledCount pointer to storage for the enrolled count
 *
 * This function will query the fingerprint reader for the total number
 * of enrollments and return the count through the *pEnrolledCount*
 * argument.  The value is only meaningful if the function return code
 * is *GT511_ERR_NONE*.
 *
 * @return **GT511_ERR_NONE** if no errors occurred.
 */
GT511_Error_t
GT511_GetEnrollCount(uint32_t *pEnrolledCount)
{
    uint32_t parm = 0;
    GT511_Error_t err = IssueCommand(GT511_CMD_GET_ENROLL_COUNT, &parm);

    // Read id from response parameter.  Not meaningful if err != _NONE
    if (pEnrolledCount != NULL)
    {
        *pEnrolledCount = parm;
    }

    return err;
}

/**
 * Check if a specific ID is enrolled
 *
 * @param id index of ID to check for enrollment
 *
 * This function will query the fingerprint reader to find out if the
 * the specific ID is enrolled.
 *
 * @return **GT511_ERR_NONE** if the specified index *IS* enrolled.
 * **GT511_ERR_IS_NOT_USED** if the specified index *IS NOT* enrolled.
 * Any other return value indicates an error.
 */
GT511_Error_t
GT511_CheckEnrolled(uint32_t id)
{
    GT511_Error_t err = IssueCommand(GT511_CMD_CHECK_ENROLLED, &id);
    return err;
}

/**
 * Verify fingerprint for specific ID
 *
 * @param id index of ID to verify
 *
 * This function will check a fingerprint match of a specific ID.
 *
 * @return **GT511_ERR_NONE** if the specified index has a fingerprint
 * match.  **GT511_ERR_VERIFY_FAILED** if the specified index does not
 * match the fingerprint.  Any other return value indicates an error.
 */
GT511_Error_t
GT511_Verify(uint32_t id)
{
    GT511_Error_t err = IssueCommand(GT511_CMD_VERIFY, &id);
    return err;
}

/**
 * Find an unused ID index.
 *
 * @param pId pointer to available ID index
 *
 * Will check each ID index (slot) in sequence until an unused one is found.
 * It will return the available index number through the pointer *pId*.
 *
 * @return **GT511_ERR_NONE** if an empty index was found.  If no empty
 * slot is found then **GT511_ERR_INVALID_POS** is returned.
 */
GT511_Error_t
GT511_FindAvailable(uint32_t *pId)
{
    // validate argument
    if (!pId)
    {
        return GT511_ERR_OTHER_ERROR;
    }

    ConsolePrintf("FindAvailable()\n");

    // iterate over all possible ID slots
    for (uint32_t i = 0; i < GT511_NUM_SLOTS; i++)
    {
        ConsolePrintf("slot %u: ", (unsigned int)i);

        // check to see if this slot has an enrollment
        uint32_t parm = i;
        GT511_Error_t err = IssueCommand(GT511_CMD_CHECK_ENROLLED, &parm);

        // if this slot is not used then return it as available
        if (err == GT511_ERR_IS_NOT_USED)
        {
            ConsolePrintf("OK\n");
            *pId = i;
            return GT511_ERR_NONE;
        }

        // check for any other unexpected return error code
        else if (err != GT511_ERR_NONE)
        {
            ConsolePrintf("ERROR: %s\n", GT511_ErrorString(err));
            return err;
        }
        else
        {
            ConsolePrintf("IN USE\n");
        }
    }

    // if we get here then no empty slots were found
    return GT511_ERR_INVALID_POS;
}

/**
 * Run the identification process.
 *
 * @param pId points to the ID index match, if any
 *
 * This function runs through all of the steps needed for fingerprint
 * identification, and returns the ID index of the match if one is found.
 * The user/app will be notified of progress as needed by calling
 * GT511_UserCallback().  For example, the callback function will be called to
 * inform the app/user when the finger should be pressed to the sensor.
 * It will be up to the application how this prompt is manifested to the
 * user.
 *
 * The following table shows how the callback is used for the various
 * steps.  In all cases, the *mode* parameter of the callback function will
 * be **GT511_MODE_IDENTIFY**.
 *
 * |callback parameter| event                                                |
 * |------------------|------------------------------------------------------|
 * | GT511_UI_PRESS   | user should press the sensor                         |
 * | GT511_UI_RELEASE | user should release the sensor                       |
 * | GT511_UI_TIMEOUT | timed out waiting for press or release               |
 * | GT511_UI_ACCEPT  | the fingerprint was identified                       |
 * | GT511_UI_REJECT  | no fingerprint match was found                       |
 * | GT511_UI_ERROR   | some error occurred (see this function return value) |
 *
 * @return **GT511_ERR_NONE** if a fingerprint match was found, in which case
 * the ID index value will be stored at *pId.  If the fingerprint was read
 * but no match was found, then **GT511_ERR_IDENTIFY_FAILED is returned.
 * Any other return value means that no match was found and may indicate
 * another kind of error.
 */
GT511_Error_t
GT511_RunIdentify(uint32_t *pId)
{
    GT511_Error_t err;

    ConsolePrintf("RunIdentify()\n");

    // turn on the LED backlight
    err = GT511_CmosLed(true);
    if (err != GT511_ERR_NONE)
    {
        ConsolePrintf("error turning on backlight: %s\n", GT511_ErrorString(err));
        // attempt to turn it off anyway
        GT511_CmosLed(false);
        return err;
    }

    // wait for a finger press
    // this function will prompt user and do UI callbacks
    err = WaitFingerPress(GT511_MODE_IDENTIFY);
    if (err != GT511_ERR_NONE)
    {
        GT511_CmosLed(false);
        return err;
    }

    // Capture the fingerprint
    err = GT511_CaptureFinger(false);
    if (err != GT511_ERR_NONE)
    {
        GT511_CmosLed(false);
        ConsolePrintf("error capture finger: %s\n", GT511_ErrorString(err));
        GT511_UserCallback(GT511_MODE_IDENTIFY, GT511_UI_ERROR);
        return err;
    }

    // Ask reader for identification
    ConsolePrintf("identifying ...\n");
    uint32_t id;
    err = GT511_Identify(&id);
    if (err != GT511_ERR_NONE)
    {
        GT511_CmosLed(false);
        ConsolePrintf("error identify: %s\n", GT511_ErrorString(err));
        GT511_UserCallback(GT511_MODE_IDENTIFY, GT511_UI_REJECT);
        return err;
    }

    // return the matched ID
    if (pId)
    {
        *pId =  id;
    }

    // wait for user to release the touch
    err = WaitFingerRelease(GT511_MODE_IDENTIFY);
    if (err != GT511_ERR_NONE)
    {
        GT511_CmosLed(false);
        return err;
    }

    // we can turn off the backlight now
    // dont bother to check errors because we are not going to do
    // anything about it anyway
    GT511_CmosLed(false);

    // At this point the ID was successful
    ConsolePrintf("identify ok: %u\n", (unsigned int)id);
    GT511_UserCallback(GT511_MODE_IDENTIFY, GT511_UI_ACCEPT);

    return err;
}

/**
 * Run the identification process.
 *
 * @param pId points to the ID index match, if any
 *
 * This function runs through all of the steps needed for fingerprint
 * identification, and returns the ID index of the match if one is found.
 * The user/app will be notified of progress as needed by calling
 * GT511_UserCallback().  For example, the callback function will be called to
 * inform the app/user when the finger should be pressed to the sensor.
 * It will be up to the application how this prompt is manifested to the
 * user.
 *
 * The following table shows how the callback is used for the various
 * steps.  In all cases, the *mode* parameter of the callback function will
 * be **GT511_MODE_IDENTIFY**.
 *
 * |callback parameter| event                                                |
 * |------------------|------------------------------------------------------|
 * | GT511_UI_PRESS   | user should press the sensor                         |
 * | GT511_UI_RELEASE | user should release the sensor                       |
 * | GT511_UI_TIMEOUT | timed out waiting for press or release               |
 * | GT511_UI_ACCEPT  | the fingerprint was identified                       |
 * | GT511_UI_REJECT  | no fingerprint match was found                       |
 * | GT511_UI_ERROR   | some error occurred (see this function return value) |
 *
 * @return **GT511_ERR_NONE** if a fingerprint match was found, in which case
 * the ID index value will be stored at *pId.  If the fingerprint was read
 * but no match was found, then **GT511_ERR_IDENTIFY_FAILED is returned.
 * Any other return value means that no match was found and may indicate
 * another kind of error.
 */
GT511_Error_t
GT511_RunVerify(uint32_t id)
{
    GT511_Error_t err;

    ConsolePrintf("RunVerify()\n");

    // turn on the LED backlight
    err = GT511_CmosLed(true);
    if (err != GT511_ERR_NONE)
    {
        ConsolePrintf("error turning on backlight: %s\n", GT511_ErrorString(err));
        // attempt to turn it off anyway
        GT511_CmosLed(false);
        return err;
    }

    // wait for a finger press
    // this function will prompt user and do UI callbacks
    err = WaitFingerPress(GT511_MODE_VERIFY);
    if (err != GT511_ERR_NONE)
    {
        GT511_CmosLed(false);
        return err;
    }

    // Capture the fingerprint
    err = GT511_CaptureFinger(false);
    if (err != GT511_ERR_NONE)
    {
        GT511_CmosLed(false);
        ConsolePrintf("error capture finger: %s\n", GT511_ErrorString(err));
        GT511_UserCallback(GT511_MODE_VERIFY, GT511_UI_ERROR);
        return err;
    }

    // Ask reader for identification
    ConsolePrintf("verifying ...\n");
    err = GT511_Verify(id);
    if (err != GT511_ERR_NONE)
    {
        GT511_CmosLed(false);
        ConsolePrintf("error verify: %s\n", GT511_ErrorString(err));
        GT511_UserCallback(GT511_MODE_VERIFY, GT511_UI_REJECT);
        return err;
    }

    // wait for user to release the touch
    err = WaitFingerRelease(GT511_MODE_VERIFY);
    if (err != GT511_ERR_NONE)
    {
        GT511_CmosLed(false);
        return err;
    }

    // we can turn off the backlight now
    // dont bother to check errors because we are not going to do
    // anything about it anyway
    GT511_CmosLed(false);

    // At this point the ID was successful
    ConsolePrintf("verify ok: %u\n", (unsigned int)id);
    GT511_UserCallback(GT511_MODE_VERIFY, GT511_UI_ACCEPT);

    return err;
}

/**
 * Run the enrollment process.
 *
 * @param pId points to the ID index to use for enrollment
 *
 * This function runs through all of the steps needed for fingerprint
 * enrollment using the ID index specified through *pId*.
 * The user/app will be notified of progress as needed by calling
 * GT511_UserCallback().  For example, the callback function will be called to
 * inform the app/user when the finger should be pressed to the sensor.
 * It will be up to the application how this prompt is manifested to the
 * user.
 *
 * The following table shows how the callback is used for the various
 * steps.  In all cases, the *mode* parameter of the callback function will
 * be **GT511_MODE_ENROLL**.
 *
 * |callback parameter| event                                                |
 * |------------------|------------------------------------------------------|
 * | GT511_UI_PRESS   | user should press the sensor                         |
 * | GT511_UI_RELEASE | user should release the sensor                       |
 * | GT511_UI_TIMEOUT | timed out waiting for press or release               |
 * | GT511_UI_ACCEPT  | the fingerprint was enrolled                         |
 * | GT511_UI_REJECT  | the fingerprint was not enrolled                     |
 * | GT511_UI_ERROR   | some error occurred (see this function return value) |
 *
 * @return **GT511_ERR_NONE** if the fingerprint was enrolled.  If the
 * fingerprint was not enrolled for some reason, then **GT511_ERR_ENROLL_FAILED
 * is returned.  Any other return value means that there was no enrollment
 * due to another kind of error.
 */
GT511_Error_t
GT511_RunEnroll(uint32_t *pId)
{
    if (!pId)
    {
        return GT511_ERR_OTHER_ERROR;
    }

    ConsolePrintf("RunEnroll()\n");

    // Find available slot for a new enrollment
    GT511_Error_t err = GT511_FindAvailable(pId);
    if (err != GT511_ERR_NONE)
    {
        GT511_UserCallback(GT511_MODE_ENROLL, GT511_UI_ERROR);
        ConsolePrintf("no available slots for enrollment\n");
        return err;
    }

    // start the enrollment process
    err = GT511_EnrollStart(*pId);
    if (err != GT511_ERR_NONE)
    {
        return err;
    }

    for (uint32_t step = 0 ; step < 3; step++)
    {
        ConsolePrintf("enroll step %u\n", (unsigned int)step);

        // turn on the LED backlight
        err = GT511_CmosLed(true);
        if (err != GT511_ERR_NONE)
        {
            ConsolePrintf("error turning on backlight: %s\n", GT511_ErrorString(err));
            // attempt to turn it off anyway
            GT511_CmosLed(false);
            return err;
        }

        // wait for a finger press
        // this function will prompt user and do UI callbacks
        err = WaitFingerPress(GT511_MODE_ENROLL);
        if (err != GT511_ERR_NONE)
        {
            GT511_CmosLed(false);
            return err;
        }

        // Capture the fingerprint
        err = GT511_CaptureFinger(true);
        if (err != GT511_ERR_NONE)
        {
            GT511_CmosLed(false);
            ConsolePrintf("error capture finger: %s\n", GT511_ErrorString(err));
            GT511_UserCallback(GT511_MODE_ENROLL, GT511_UI_ERROR);
            return err;
        }

        // Issue the enrollment command based on the step in the sequence.
        // There are 3 enrollment steps.
        if (step == 0)
        {
            err = GT511_Enroll1();
        }
        else if (step ==1)
        {
            err = GT511_Enroll2();
        }
        else
        {
            err = GT511_Enroll3();
        }
        if (err != GT511_ERR_NONE)
        {
            GT511_UserCallback(GT511_MODE_ENROLL, GT511_UI_REJECT);
            ConsolePrintf("enrollment failed at step %u, err=%s\n",
                          (unsigned int)step, GT511_ErrorString(err));
            GT511_CmosLed(false);
            return err;
        }

        // wait for user to release the touch
        err = WaitFingerRelease(GT511_MODE_ENROLL);
        if (err != GT511_ERR_NONE)
        {
            GT511_CmosLed(false);
            return err;
        }

        // we can turn off the backlight now
        // dont bother to check errors because we are not going to do
        // anything about it anyway
        GT511_CmosLed(false);
    }

    // At this point the enroll was successful
    ConsolePrintf("enroll ok: %u\n", (unsigned int)*pId);
    GT511_UserCallback(GT511_MODE_ENROLL, GT511_UI_ACCEPT);

    return GT511_ERR_NONE;
}

/** @} */


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

#ifndef __FINGERPRINT_GT511_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup gt511_driver
 * @{
 */

/**
 * Possible error codes that can be returned by the GT-511C driver API
 * functions.  Most of these map directly to errors produced by the hardware
 * sensor.  Refer to the Gt-511C data sheet for more details.
 */
typedef enum
{
    GT511_ERR_NONE              = 0,        ///< no error; success
    GT511_ERR_TIMEOUT           = 0x1001,   ///< obsolete; timeout during capture
    GT511_ERR_INVALID_BAUDRATE  = 0x1002,   ///< obsolete; invalid baud rate
    GT511_ERR_INVALID_POS       = 0x1003,   ///< specified ID is not between 0~19
    GT511_ERR_IS_NOT_USED       = 0x1004,   ///< specified ID is not in use
    GT511_ERR_IS_ALREADY_USED   = 0x1005,   ///< specified ID is already in use
    GT511_ERR_COMM_ERR          = 0x1006,   ///< communication error
    GT511_ERR_VERIFY_FAILED     = 0x1007,   ///< verification failed
    GT511_ERR_IDENTIFY_FAILED   = 0x1008,   ///< identification failed
    GT511_ERR_DB_IS_FULL        = 0x1009,   ///< database is full
    GT511_ERR_DB_IS_EMPTY       = 0x100A,   ///< database is empty
    GT511_ERR_TURN_ERR          = 0x100B,   ///< obsolete; bad enrollment order
    GT511_ERR_BAD_FINGER        = 0x100C,   ///< bad fingerprint
    GT511_ERR_ENROLL_FAILED     = 0x100D,   ///< enrollment failed
    GT511_ERR_IS_NOT_SUPPORTED  = 0x100E,   ///< unsupported command was used
    GT511_ERR_DEV_ERR           = 0x100F,   ///< hardware device error
    GT511_ERR_CAPTURE_CANCELED  = 0x1010,   ///< obsolete; capture is cancelled
    GT511_ERR_INVALID_PARAM     = 0x1011,   ///< invalid parameter
    GT511_ERR_FINGER_IS_NOT_PRESSED  = 0x1012, ///< finger not pressed
    GT511_ERR_OTHER_ERROR = 0xFFFF          ///< non-hardware driver error
} GT511_Error_t;

/**
 * Structure that holds the fingerprint sensor info data that is returned
 * during "open".  Refer to the datasheet for more details.
 */
typedef struct
{
    uint32_t firmwareVersion;
    uint32_t isoAreaMaxSize;
    uint8_t serialNumber[16];
} GT511_Info_t;

/**
 * Send message to sensor (implemented by application).
 *
 * @param pMessage points at packet to be sent to sensor
 * @param length number of bytes in the packet
 *
 * This function is used by the GT-511C fingerprint sensor driver to send
 * command packets to the sensor.  This is normally done via a serial port.
 * This function must be implemented by the application.  This can be as
 * simple or sophisticated as needed.  For example it may just use
 * poll-waiting to copy bytes to the serial output.  Or it may be interrupt
 * driven with buffering and timeouts.
 *
 * @note This function is implemented by the application.
 *
 * @return **true** if the message was sent and **false** if there was any
 * error sending the message.
 */
extern bool GT511_SendMessage(uint8_t *pMessage, uint32_t length);

/**
 * Receive message from the sensor (implemented by application).
 *
 * @param pMessage points at storage for the incoming message packet
 * @param length number of expected bytes in the packet
 *
 * This function is used by the GT-511C fingerprint sensor driver to read
 * an incoming message from the sensor.  This is normally done via a serial
 * port.  This function must be implemented by the application and can be as
 * simple or sophisticated as needed.  For example it may just use
 * poll-waiting to copy bytes from the serial input.  Or it may be interrupt
 * driven with buffering and timeouts.
 *
 * @note This function is implemented by the application.
 *
 * @return The number of bytes that were actually read and stored at
 * pMessage.  Anything other than the requested length is considered an
 * error.
 */
extern uint32_t GT511_ReceiveMessage(uint8_t *pMessage, uint32_t length);

/**
 * Identifies the different possible modes of the driver when the callback
 * function GT511_UserCallback() is called.
 */
typedef enum
{
    GT511_MODE_IDLE,        ///< driver is idle
    GT511_MODE_IDENTIFY,    ///< driver is performing identification process
    GT511_MODE_VERIFY,      ///< driver is performing verification process
    GT511_MODE_CAPTURE,     ///< driver is performing capture process
    GT511_MODE_ENROLL,      ///< driver is performing enrollment process
} GT511_Mode_t;

/**
 * Identifies an event when the callback function GT511_UserCallback() is
 * called.
 */
typedef enum
{
    GT511_UI_PRESS,     ///< user should press the sensor
    GT511_UI_RELEASE,   ///< user should release the sensor
    GT511_UI_TIMEOUT,   ///< timeout occurred waiting for press or release
    GT511_UI_ACCEPT,    ///< fingerprint was accepted (identify or enrollment)
    GT511_UI_REJECT,    ///< fingerprint was rejected (identify or enrollment)
    GT511_UI_ERROR,     ///< processing error occurred
} GT511_UserInfo_t;

/**
 * Notify the application/user of a driver event (implemented by application)
 *
 * @param mode the current processing mode of the driver
 * @param ui the user event or notification information
 *
 * This function is used to notify the user or application of a driver
 * event during a process such as enrollment or identification.  The main
 * purpose is to provide a way for the driver to request an action such
 * as to press or release the sensor.
 *
 * @note This function is implemented by the application.
 */
extern void GT511_UserCallback(GT511_Mode_t mode, GT511_UserInfo_t ui);

/**
 * Ask application to start a timeout (implemented by application)
 *
 * @param mode the current processing mode of the driver
 *
 * This function is called by the driver before it enters a process where
 * it must wait for something to occur (such as waiting for a touch).
 * The application must use it to set an appropriate timeout period
 * according to the mode.  The same timeout value can be used in all cases,
 * or even no timeout used.
 *
 * @note This function is implemented by the application.
 */
extern void GT511_SetTimeout(GT511_Mode_t mode);

/**
 * Check for timeout expiration (implemented by application).
 *
 * @param mode the current processing mode of the driver
 *
 * This function is called repeatedly by the driver when it is performing
 * a process that requires waiting for an external event (such as waiting
 * for a touch).  The application can return **true** to indicate that a
 * timeout occurred thus canceling the wait.  The application can use a
 * different timeout for each mode, or the same value for all modes, or
 * not even use timeouts if not needed.
 *
 * @note This function is implemented by the application.
 */
extern bool GT511_CheckTimeout(GT511_Mode_t mode);

/** @} */

// Remaining function prototypes.  These are documented in the .c file.
extern const char *GT511_ErrorString(GT511_Error_t err);
extern GT511_Error_t GT511_Open(GT511_Info_t *pInfo);
extern GT511_Error_t GT511_Close(void);
extern GT511_Error_t GT511_CmosLed(bool on);
extern GT511_Error_t GT511_IsPressFinger(bool *pIsPressed);
extern GT511_Error_t GT511_CaptureFinger(bool highQuality);
extern GT511_Error_t GT511_Identify(uint32_t *pId);
extern GT511_Error_t GT511_Verify(uint32_t id);
extern GT511_Error_t GT511_EnrollStart(uint32_t id);
extern GT511_Error_t GT511_Enroll1(void);
extern GT511_Error_t GT511_Enroll2(void);
extern GT511_Error_t GT511_Enroll3(void);
extern GT511_Error_t GT511_DeleteID(uint32_t id);
extern GT511_Error_t GT511_DeleteAll(void);
extern GT511_Error_t GT511_GetEnrollCount(uint32_t *pEnrolledCount);
extern GT511_Error_t GT511_CheckEnrolled(uint32_t id);
extern GT511_Error_t GT511_FindAvailable(uint32_t *pId);
extern GT511_Error_t GT511_RunEnroll(uint32_t *pId);
extern GT511_Error_t GT511_RunIdentify(uint32_t *pId);
extern GT511_Error_t GT511_RunVerify(uint32_t id);

// These are only stubs.  To be implemented some day.
extern GT511_Error_t GT511_ChangeBaudrate(uint32_t baudrate);
extern GT511_Error_t GT511_VerifyTemplate(uint32_t id, uint8_t *pTemplate, uint32_t size);
extern GT511_Error_t GT511_IdentifyTemplate(uint8_t *pTemplate, uint32_t size);
extern GT511_Error_t GT511_MakeTemplate(uint8_t *pTemplate, uint32_t size);
extern GT511_Error_t GT511_GetImage(uint8_t *pImage, uint32_t size);
extern GT511_Error_t GT511_GetRawImage(uint8_t *pImage, uint32_t size);
extern GT511_Error_t GT511_GetTemplate(uint32_t id, uint8_t *pTemplate, uint32_t size);
extern GT511_Error_t GT511_SetTemplate(uint32_t id, bool checkDuplicate, uint8_t *pTemplate, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif

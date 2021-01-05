/**
  ******************************************************************************
  * File Name          : aws_pkcs11_pal.h
  * Description        : This file provides code that interfaces AWS PKCS11
  *                      layer to the STM32H743ZI.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/**
 * @file aws_pkcs11_pal.c
 * @brief Device specific helpers for PKCS11 Interface.
 */

/* Amazon FreeRTOS Includes. */
#include "aws_pkcs11.h"
#include "FreeRTOS.h"
#include "FreeRTOSIPConfig.h"
#include "task.h"
#include "aws_pkcs11_config.h"

/* C runtime includes. */
#include <stdio.h>
#include <string.h>
#include "rng.h"
#include "aws_flash.h"

#define pkcs11OBJECT_MAX_SIZE         2048
#define pkcs11OBJECT_PRESENT_MAGIC    ( 0xABCD0000uL )
#define pkcs11OBJECT_LENGTH_MASK      ( 0x0000FFFFuL )
#define pkcs11OBJECT_PRESENT_MASK     ( 0xFFFF0000uL )

enum eObjectHandles
{
    eInvalidHandle = 0, /* From PKCS #11 spec: 0 is never a valid object handle.*/
    eAwsDevicePrivateKey = 1,
    eAwsDevicePublicKey,
    eAwsDeviceCertificate,
    eAwsCodeSigningKey
};

/**
 * @brief Structure for certificates/key storage.
 */
typedef struct
{
    CK_CHAR cDeviceCertificate[ pkcs11OBJECT_MAX_SIZE ];
    uint32_t ulDeviceCertificateMark;
    uint32_t ulDeviceCertificateSize;
    uint32_t ulDeviceCertificateMarkPadding[6];

    CK_CHAR cDeviceKey[ pkcs11OBJECT_MAX_SIZE ];
    uint32_t ulDeviceKeyMark;
    uint32_t ulDeviceKeySize;
    uint32_t ulDeviceKeyMarkPadding[7];
	
    CK_CHAR cCodeSignKey[ pkcs11OBJECT_MAX_SIZE ];
	uint32_t ulCodeSignKeyMark;
    uint32_t ulCodeSignKeySize;
    uint32_t ulCodeSignKeyMarkPadding[7];
	
} P11KeyConfig_t;


/**
 * @brief Certificates/key storage in flash.
 */

volatile P11KeyConfig_t * P11KeyConfig = ( P11KeyConfig_t * ) 0x08100000uL;
P11KeyConfig_t P11KeyConfigSave;

#define PKCS11_SECTOR             ( FLASH_SECTOR_12 )

uint32_t FLASH_UpdateCert( uint8_t * data, uint32_t size )
{
    BaseType_t xReturn = pdFAIL;
    memset(&P11KeyConfigSave, 0, sizeof(P11KeyConfigSave));

    /* Check if a certificate is present. */
    if( pkcs11OBJECT_PRESENT_MAGIC == P11KeyConfig->ulDeviceCertificateMark )
    {
        /* If the certificate is not the same, replace it. */
        if( memcmp( ( char * ) data,
                    ( char * ) P11KeyConfig->cDeviceCertificate,
                    size ) != 0 )
        {
            /* Check if a key is present. */
            if( pkcs11OBJECT_PRESENT_MAGIC == P11KeyConfig->ulDeviceKeyMark )
            {
                memcpy( P11KeyConfigSave.cDeviceKey, (CK_CHAR *) P11KeyConfig->cDeviceKey, P11KeyConfig->ulDeviceKeySize );
                P11KeyConfigSave.ulDeviceKeyMark = pkcs11OBJECT_PRESENT_MAGIC;
                P11KeyConfigSave.ulDeviceKeySize = P11KeyConfig->ulDeviceKeySize;
            }

            /* Check if a code sign key is present. */
            if( pkcs11OBJECT_PRESENT_MAGIC == P11KeyConfig->ulCodeSignKeyMark )
            {
                memcpy( P11KeyConfigSave.cCodeSignKey, (CK_CHAR *) P11KeyConfig->cCodeSignKey, P11KeyConfig->ulCodeSignKeySize );
                P11KeyConfigSave.ulCodeSignKeyMark = pkcs11OBJECT_PRESENT_MAGIC;
                P11KeyConfigSave.ulCodeSignKeySize = P11KeyConfig->ulCodeSignKeySize;
            }

            memcpy( P11KeyConfigSave.cDeviceCertificate, data, size );
            P11KeyConfigSave.ulDeviceCertificateMark = pkcs11OBJECT_PRESENT_MAGIC;
            P11KeyConfigSave.ulDeviceCertificateSize = size;
            
            //  Blank out PKCS sector
            xReturn = AWS_FlashEraseSector( FLASH_BANK_2, PKCS11_SECTOR );
            
            if (xReturn == pdPASS)
            {
            	xReturn = AWS_FlashProgramBlock( (CK_CHAR *) P11KeyConfig->cDeviceCertificate, P11KeyConfigSave.cDeviceCertificate, pkcs11OBJECT_MAX_SIZE + 32 );
            }

            if( pkcs11OBJECT_PRESENT_MAGIC == P11KeyConfigSave.ulDeviceKeyMark )
            {
                if (xReturn == pdPASS)
                {
                    //  Recopy Private Key
                    xReturn = AWS_FlashProgramBlock( (CK_CHAR *) P11KeyConfig->cDeviceKey, P11KeyConfigSave.cDeviceKey, pkcs11OBJECT_MAX_SIZE + 32 );
                }
			}

            if( pkcs11OBJECT_PRESENT_MAGIC == P11KeyConfigSave.ulCodeSignKeyMark )
            {
                if (xReturn == pdPASS)
                {
                    //  Recopy Code Sign Key
                    xReturn = AWS_FlashProgramBlock( (CK_CHAR *) P11KeyConfig->cCodeSignKey, P11KeyConfigSave.cCodeSignKey, pkcs11OBJECT_MAX_SIZE + 32 );
                }
			}
        }
        else
        {
            xReturn = pdPASS;
        }
    }
    /* No certificate is present so just write it. */
    else
    {
        memcpy( P11KeyConfigSave.cDeviceCertificate, data, size );
        P11KeyConfigSave.ulDeviceCertificateMark = pkcs11OBJECT_PRESENT_MAGIC;
        P11KeyConfigSave.ulDeviceCertificateSize = size;

        xReturn = AWS_FlashProgramBlock( (CK_CHAR *) P11KeyConfig->cDeviceCertificate, P11KeyConfigSave.cDeviceCertificate, pkcs11OBJECT_MAX_SIZE + 32 );
    }
    
    if (xReturn != pdPASS)
    {
    	size = 0;
    }

    return size;
}

uint32_t FLASH_UpdateKey( uint8_t * data, uint32_t size )
{
    BaseType_t xReturn = pdFAIL;
    memset(&P11KeyConfigSave, 0, sizeof(P11KeyConfigSave));

    /* Check if a key is present. */
    if( pkcs11OBJECT_PRESENT_MAGIC == P11KeyConfig->ulDeviceKeyMark )
    {
        /* If the key is not the same, replace it. */
        if ( memcmp( ( char * ) data,
                     ( char * ) P11KeyConfig->cDeviceKey,
                     size ) != 0 )
        {
            /* Check if a certificate is present. */
            if( pkcs11OBJECT_PRESENT_MAGIC == P11KeyConfig->ulDeviceCertificateMark )
            {
                memcpy( P11KeyConfigSave.cDeviceCertificate, (CK_CHAR *) P11KeyConfig->cDeviceCertificate, P11KeyConfig->ulDeviceCertificateSize );
                P11KeyConfigSave.ulDeviceCertificateMark = pkcs11OBJECT_PRESENT_MAGIC;
                P11KeyConfigSave.ulDeviceCertificateSize = P11KeyConfig->ulDeviceCertificateSize;
            }

            /* Check if a code sign key is present. */
            if( pkcs11OBJECT_PRESENT_MAGIC == P11KeyConfig->ulCodeSignKeyMark )
            {
                memcpy( P11KeyConfigSave.cCodeSignKey, (CK_CHAR *) P11KeyConfig->cCodeSignKey, P11KeyConfig->ulCodeSignKeySize );
                P11KeyConfigSave.ulCodeSignKeyMark = pkcs11OBJECT_PRESENT_MAGIC;
                P11KeyConfigSave.ulCodeSignKeySize = P11KeyConfig->ulCodeSignKeySize;
            }

            memcpy( P11KeyConfigSave.cDeviceKey, data, size );
            P11KeyConfigSave.ulDeviceKeyMark = pkcs11OBJECT_PRESENT_MAGIC;
            P11KeyConfigSave.ulDeviceKeySize = size;
            
            //  Blank out PKCS sector
            xReturn = AWS_FlashEraseSector( FLASH_BANK_2, PKCS11_SECTOR );

            if (xReturn == pdPASS)
            {
                xReturn = AWS_FlashProgramBlock( (CK_CHAR *) P11KeyConfig->cDeviceKey, P11KeyConfigSave.cDeviceKey, pkcs11OBJECT_MAX_SIZE + 32 );
            }

            if( pkcs11OBJECT_PRESENT_MAGIC == P11KeyConfigSave.ulDeviceCertificateMark )
            {
                if (xReturn == pdPASS)
                {
                    //  Recopy Private Key
                    xReturn = AWS_FlashProgramBlock( (CK_CHAR *) P11KeyConfig->cDeviceCertificate, P11KeyConfigSave.cDeviceCertificate, pkcs11OBJECT_MAX_SIZE + 32 );
                }
            }
            if( pkcs11OBJECT_PRESENT_MAGIC == P11KeyConfigSave.ulCodeSignKeyMark )
            {
                if (xReturn == pdPASS)
                {
                    //  Recopy Code Sign Key
                    xReturn = AWS_FlashProgramBlock( (CK_CHAR *) P11KeyConfig->cCodeSignKey, P11KeyConfigSave.cCodeSignKey, pkcs11OBJECT_MAX_SIZE + 32 );
                }
			}
        }
        else
        {
            xReturn = pdPASS;
        }
    }
    /* No device private key is present so just write it. */
    else
    {
        memcpy( P11KeyConfigSave.cDeviceKey, data, size );
        P11KeyConfigSave.ulDeviceKeyMark = pkcs11OBJECT_PRESENT_MAGIC;
        P11KeyConfigSave.ulDeviceKeySize = size;

        xReturn = AWS_FlashProgramBlock( (CK_CHAR *) P11KeyConfig->cDeviceKey, P11KeyConfigSave.cDeviceKey, pkcs11OBJECT_MAX_SIZE + 32 );
    }

    if (xReturn != pdPASS)
    {
        size = 0;
    }
    
    return size;
}

uint32_t FLASH_UpdateCodeSignKey( uint8_t * data, uint32_t size )
{
    BaseType_t xReturn = pdFAIL;
    memset(&P11KeyConfigSave, 0, sizeof(P11KeyConfigSave));

    /* Check if a code sign key is present. */
    if( pkcs11OBJECT_PRESENT_MAGIC == P11KeyConfig->ulCodeSignKeyMark )
    {
        /* If the certificate is not the same, replace it. */
        if( memcmp( ( char * ) data,
                    ( char * ) P11KeyConfig->cCodeSignKey,
                    size ) != 0 )
        {
            /* Check if a certificate is present. */
            if( pkcs11OBJECT_PRESENT_MAGIC == P11KeyConfig->ulDeviceCertificateMark )
            {
                memcpy( P11KeyConfigSave.cDeviceCertificate, (CK_CHAR *) P11KeyConfig->cDeviceCertificate, P11KeyConfig->ulDeviceCertificateSize );
                P11KeyConfigSave.ulDeviceCertificateMark = pkcs11OBJECT_PRESENT_MAGIC;
                P11KeyConfigSave.ulDeviceCertificateSize = P11KeyConfig->ulDeviceCertificateSize;
            }

            /* Check if a key is present. */
            if( pkcs11OBJECT_PRESENT_MAGIC == P11KeyConfig->ulDeviceKeyMark )
            {
                memcpy( P11KeyConfigSave.cDeviceKey, (CK_CHAR *) P11KeyConfig->cDeviceKey, P11KeyConfig->ulDeviceKeySize );
                P11KeyConfigSave.ulDeviceKeyMark = pkcs11OBJECT_PRESENT_MAGIC;
                P11KeyConfigSave.ulDeviceKeySize = P11KeyConfig->ulDeviceKeySize;
            }

            memcpy( P11KeyConfigSave.cCodeSignKey, data, size );
            P11KeyConfigSave.ulCodeSignKeyMark = pkcs11OBJECT_PRESENT_MAGIC;
            P11KeyConfigSave.ulCodeSignKeySize = size;

            //  Blank out PKCS sector
            xReturn = AWS_FlashEraseSector( FLASH_BANK_2, PKCS11_SECTOR );

            if (xReturn == pdPASS)
            {
            	xReturn = AWS_FlashProgramBlock( (CK_CHAR *) P11KeyConfig->cCodeSignKey, P11KeyConfigSave.cCodeSignKey, pkcs11OBJECT_MAX_SIZE + 32 );
            }
            if( pkcs11OBJECT_PRESENT_MAGIC == P11KeyConfigSave.ulDeviceCertificateMark )
            {
                if (xReturn == pdPASS)
                {
                    //  Recopy Private Key
                    xReturn = AWS_FlashProgramBlock( (CK_CHAR *) P11KeyConfig->cDeviceCertificate, P11KeyConfigSave.cDeviceCertificate, pkcs11OBJECT_MAX_SIZE + 32 );
                }
            }
            if( pkcs11OBJECT_PRESENT_MAGIC == P11KeyConfigSave.ulDeviceKeyMark )
            {
                if (xReturn == pdPASS)
                {
                    //  Recopy Private Key
                    xReturn = AWS_FlashProgramBlock( (CK_CHAR *) P11KeyConfig->cDeviceKey, P11KeyConfigSave.cDeviceKey, pkcs11OBJECT_MAX_SIZE + 32 );
                }
			}
        }
        else
        {
            xReturn = pdPASS;
        }
    }
    /* No code signing key is present so just write it. */
    else
    {
        memcpy( P11KeyConfigSave.cCodeSignKey, data, size );
        P11KeyConfigSave.ulCodeSignKeyMark = pkcs11OBJECT_PRESENT_MAGIC;
        P11KeyConfigSave.ulCodeSignKeySize = size;

        xReturn = AWS_FlashProgramBlock( (CK_CHAR *) P11KeyConfig->cCodeSignKey, P11KeyConfigSave.cCodeSignKey, pkcs11OBJECT_MAX_SIZE + 32 );
    }

    if (xReturn != pdPASS)
    {
    	size = 0;
    }

    return size;
}

/**
* @brief Writes a file to local storage.
*
* Port-specific file write for crytographic information.
*
* @param[in] pxLabel       Label of the object to be saved.
* @param[in] pucData       Data buffer to be written to file
* @param[in] ulDataSize    Size (in bytes) of data to be saved.
*
* @return The file handle of the object that was stored.
*/
CK_OBJECT_HANDLE PKCS11_PAL_SaveObject( CK_ATTRIBUTE_PTR pxLabel,
    uint8_t * pucData,
    uint32_t ulDataSize )
{
    CK_OBJECT_HANDLE xHandle = eInvalidHandle;
    CK_RV xBytesWritten = 0;

    if( ulDataSize <= pkcs11OBJECT_MAX_SIZE )
    {
        /*
         * write client certificate.
         */
        if( strcmp( pxLabel->pValue,
                    pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS ) == 0 )
        {
            xBytesWritten = FLASH_UpdateCert( pucData,
                                          ( ulDataSize ) );

            if( xBytesWritten == ( ulDataSize ) )
            {
                xHandle = eAwsDeviceCertificate;
            }
        }

        /*
         * write client key.
         */

        else if( strcmp( pxLabel->pValue,
                         pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS ) == 0 )
        {
            xBytesWritten = FLASH_UpdateKey( pucData,
                                          ulDataSize );

            if( xBytesWritten == ( ulDataSize ) )
            {
                xHandle = eAwsDevicePrivateKey;
            }
        }

        else if( strcmp( pxLabel->pValue,
                         pkcs11configLABEL_CODE_VERIFICATION_KEY ) == 0 )
        {
            xBytesWritten = FLASH_UpdateCodeSignKey( pucData,
                                          ulDataSize );

            if( xBytesWritten == ( ulDataSize ) )
            {
                xHandle = eAwsCodeSigningKey;
            }
        }
    }

    return xHandle;
}

/**
* @brief Translates a PKCS #11 label into an object handle.
*
* Port-specific object handle retrieval.
*
*
* @param[in] pLabel         Pointer to the label of the object
*                           who's handle should be found.
* @param[in] usLength       The length of the label, in bytes.
*
* @return The object handle if operation was successful.
* Returns eInvalidHandle if unsuccessful.
*/
CK_OBJECT_HANDLE PKCS11_PAL_FindObject( uint8_t * pLabel,
    uint8_t usLength )
{
    CK_OBJECT_HANDLE xHandle = eInvalidHandle;

    if( ( 0 == memcmp( pLabel, pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS, usLength ) ) &&
        ( ( P11KeyConfig->ulDeviceCertificateMark & pkcs11OBJECT_PRESENT_MASK ) == pkcs11OBJECT_PRESENT_MAGIC ) )
    {
        xHandle = eAwsDeviceCertificate;
    }
    else if( ( 0 == memcmp( pLabel, pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS, usLength ) ) &&
             ( ( P11KeyConfig->ulDeviceKeyMark & pkcs11OBJECT_PRESENT_MASK ) == pkcs11OBJECT_PRESENT_MAGIC ) )
    {
        xHandle = eAwsDevicePrivateKey;
    }
    else if( ( 0 == memcmp( pLabel, pkcs11configLABEL_CODE_VERIFICATION_KEY, usLength ) ) &&
             ( ( P11KeyConfig->ulCodeSignKeyMark & pkcs11OBJECT_PRESENT_MASK ) == pkcs11OBJECT_PRESENT_MAGIC ) )
    {
        xHandle = eAwsCodeSigningKey;
    }

    return xHandle;
}

/**
* @brief Gets the value of an object in storage, by handle.
*
* Port-specific file access for cryptographic information.
*
* This call dynamically allocates the buffer which object value
* data is copied into.  PKCS11_PAL_GetObjectValueCleanup()
* should be called after each use to free the dynamically allocated
* buffer.
*
* @sa PKCS11_PAL_GetObjectValueCleanup
*
* @param[in] pcFileName    The name of the file to be read.
* @param[out] ppucData     Pointer to buffer for file data.
* @param[out] pulDataSize  Size (in bytes) of data located in file.
* @param[out] pIsPrivate   Boolean indicating if value is private (CK_TRUE)
*                          or exportable (CK_FALSE)
*
* @return CKR_OK if operation was successful.  CKR_KEY_HANDLE_INVALID if
* no such object handle was found, CKR_DEVICE_MEMORY if memory for
* buffer could not be allocated, CKR_FUNCTION_FAILED for device driver
* error.
*/
CK_RV PKCS11_PAL_GetObjectValue( CK_OBJECT_HANDLE xHandle,
    uint8_t ** ppucData,
    uint32_t * pulDataSize,
    CK_BBOOL * pIsPrivate )
{
    CK_RV ulReturn = CKR_OBJECT_HANDLE_INVALID;

    /*
     * Read client certificate.
     */

    if( xHandle == eAwsDeviceCertificate )
    {
        /*
         * return reference and size only if certificates are present in flash
         */
        if( ( P11KeyConfig->ulDeviceCertificateMark & pkcs11OBJECT_PRESENT_MASK ) == pkcs11OBJECT_PRESENT_MAGIC )
        {
            *ppucData = (CK_CHAR *) P11KeyConfig->cDeviceCertificate;
            *pulDataSize = P11KeyConfig->ulDeviceCertificateSize;
            *pIsPrivate = CK_FALSE;
            ulReturn = CKR_OK;
        }
    }

    /*
     * Read client key.
     */

    else if( xHandle == eAwsDevicePrivateKey )
    {
        /*
         * return reference and size only if certificates are present in flash
         */
        if( ( P11KeyConfig->ulDeviceKeyMark & pkcs11OBJECT_PRESENT_MASK ) == pkcs11OBJECT_PRESENT_MAGIC )
        {
            *ppucData = (CK_CHAR *) P11KeyConfig->cDeviceKey;
            *pulDataSize = P11KeyConfig->ulDeviceKeySize;
            *pIsPrivate = CK_TRUE;
            ulReturn = CKR_OK;
        }
    }

    else if( xHandle == eAwsDevicePublicKey )
    {
        /*
         * return reference and size only if certificates are present in flash
         */
        if( ( P11KeyConfig->ulDeviceKeyMark & pkcs11OBJECT_PRESENT_MASK ) == pkcs11OBJECT_PRESENT_MAGIC )
        {
            *ppucData = (CK_CHAR *) P11KeyConfig->cDeviceKey;
            *pulDataSize = P11KeyConfig->ulDeviceKeySize;
            *pIsPrivate = CK_FALSE;
            ulReturn = CKR_OK;
        }
    }

    else if( xHandle == eAwsCodeSigningKey )
    {
        if( ( P11KeyConfig->ulCodeSignKeyMark & pkcs11OBJECT_PRESENT_MASK ) == pkcs11OBJECT_PRESENT_MAGIC )
        {
            *ppucData = (CK_CHAR *) P11KeyConfig->cCodeSignKey;
            *pulDataSize = P11KeyConfig->ulCodeSignKeySize;
            *pIsPrivate = CK_FALSE;
            ulReturn = CKR_OK;
        }
    }

    return ulReturn;
}


/**
* @brief Cleanup after PKCS11_GetObjectValue().
*
* @param[in] pucData       The buffer to free.
*                          (*ppucData from PKCS11_PAL_GetObjectValue())
* @param[in] ulDataSize    The length of the buffer to free.
*                          (*pulDataSize from PKCS11_PAL_GetObjectValue())
*/
void PKCS11_PAL_GetObjectValueCleanup( uint8_t * pucData,
    uint32_t ulDataSize )
{
    /* Unused parameters. */
    ( void ) pucData;
    ( void ) ulDataSize;

    /* Since no buffer was allocated on heap, there is no cleanup
     * to be done. */
}

/*-----------------------------------------------------------*/

extern RNG_HandleTypeDef hrng;

int mbedtls_hardware_poll( void * data,
                           unsigned char * output,
                           size_t len,
                           size_t * olen )
{
  uint32_t index;
  uint32_t randomValue;

  for( index = 0; index < len / 4; index++ )
  {
    if( HAL_RNG_GenerateRandomNumber( &hrng, &randomValue ) == HAL_OK )
    {
      *olen += 4;
      memset( &( output[index * 4] ), ( int ) randomValue, 4 );
    }
    else
    {
      Error_Handler();
    }
  }

  return 0;
}

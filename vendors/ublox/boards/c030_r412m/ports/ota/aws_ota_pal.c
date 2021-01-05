/**
  ******************************************************************************
  * File Name          : aws_ota_pal.c
  * Description        : This file provides code that interfaces AWS OTA layer
  *                      to the STM32F767ZI.
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

/* C Runtime includes. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Amazon FreeRTOS include. */
#include "FreeRTOS.h"
#include "aws_ota_pal.h"
#include "aws_ota_agent_internal.h"

#include "aws_pkcs11.h"
#include "aws_crypto.h"
#include "aws_ota_codesigner_certificate.h"

#include "stm32f7xx_hal.h"
#include "aws_flash.h"

/* Specify the OTA signature algorithm we support on this platform. */
const char cOTA_JSON_FileSignatureKey[ OTA_FILE_SIG_KEY_STR_MAX_LENGTH ] = "sig-sha256-ecdsa";

#define OTA_HALF_SECOND_DELAY            pdMS_TO_TICKS( 500UL )

/* definitions shared with the resident bootloader. */
#define AWS_BOOT_IMAGE_SIGNATURE         "@AFRTOS"
#define AWS_BOOT_IMAGE_SIGNATURE_SIZE    ( 7U )

/* ST PAL error codes. */

#define ST_ERR_NONE                         0     /* No error. */
#define ST_ERR_INVALID_CONTEXT              -1    /* The context valiation failed. */
#define ST_ERR_ADDR_OUT_OF_RANGE            -2    /* The block write address was out of range. */
#define ST_ERR_FLASH_WRITE_FAIL             -3    /* We failed to write data to flash. */
#define ST_ERR_FLASH_ERASE_FAIL             -4    /* The flash erase operation failed. */
#define ST_ERR_NOT_PENDING_COMMIT           -5    /* Image isn't in the Pending Commit state. */

#define AWS_BOOT_FLAG_IMG_NEW               0xffU /* 11111111b A new image that hasn't yet been run. */
#define AWS_BOOT_FLAG_IMG_PENDING_COMMIT    0xfeU /* 11111110b Image is pending commit and is ready for self test. */
#define AWS_BOOT_FLAG_IMG_VALID             0xfcU /* 11111100b The image was accepted as valid by the self test code. */
#define AWS_BOOT_FLAG_IMG_INVALID           0xf8U /* 11111000b The image was NOT accepted by the self test code. */

/*
 * Image Header.
 */
typedef union
{
    uint32_t ulAlign[ 104 ];
    struct
    {
      char      cImgSignature[AWS_BOOT_IMAGE_SIGNATURE_SIZE];       /* Signature identifying a valid application: AWS_BOOT_IMAGE_SIGNATURE. */
      uint8_t   Reserved0;
      uint32_t  Reserved1[6];
      uint32_t  ucImageNewFlag;
      uint32_t  Reserved2[7];
      uint32_t  ucImageCommitPendingFlag;
      uint32_t  Reserved3[7];
      uint32_t  ucImageValidFlag;
      uint32_t  Reserved4[7];
      uint32_t  ucImageInvalidFlag;
      uint32_t  Reserved5[7];
    };
} BootImageHeader_t;

/* Boot application image descriptor.
 * Total size is 512 bytes
 * This is the descriptor used by the bootloader
 * to maintain the application images.
 */
typedef struct
{
    BootImageHeader_t   xImgHeader;     /* Application image header (416 bytes). */
    uint32_t            ulSequenceNum;  /* OTA sequence number. Higher is newer. */
    /* Use byte pointers for image addresses so pointer math doesn't use incorrect scalars. */
    const uint8_t*      pvStartAddr;    /* Image start address. */
    const uint8_t*      pvEndAddr;      /* Image end address. */
    const uint8_t*      pvExecAddr;     /* Execution start address. */
    uint32_t ulHardwareID;              /* Unique Hardware ID. */
    uint32_t ulReserved[19];                /* Reserved. *//*lint -e754 -e830 intentionally unreferenced alignment word. */
} BootImageDescriptor_t;

/*
 * Image Trailer.
 */
typedef struct
{
    uint8_t     aucSignatureType[OTA_FILE_SIG_KEY_STR_MAX_LENGTH]; /* Signature Type. */
    uint32_t    ulSignatureSize;                                   /* Signature size. */
    uint8_t     aucSignature[kOTA_MaxSignatureSize];               /* Signature */
} BootImageTrailer_t;

/**
 * @brief Application image flags.
 * These are the flags used by bootloader to maintain the application
 * images.
 * New (11111111b) - A new image that hasn't yet been run.
 * Commit Pending (11111110b) - Image is pending commit and is ready for self test.
 * Valid (11111100b) - The image was accepted as valid by the self test code.
 * Invalid (11111000b) - The image was NOT accepted by the self test code.
 */
typedef enum
{
    eBootImageFlagNew = 0xff,
    eBootImageFlagCommitPending = 0xfe,
    eBootImageFlagValid = 0xfc,
    eBootImageFlagInvalid = 0xf8,
} BOOTImageFlag_t;

/* adopted for H7 -> F7 porting */
#define FLASH_SECTOR_SIZE  0x00020000UL        /* 128 KB */

/* The new image is always programmed in the upper flash bank. */
#define AWS_FLASH_IMAGE_START               ( 0x08120000UL )
static const uint32_t ulFlashImageMaxSize = ( uint32_t ) ( 6 * FLASH_SECTOR_SIZE ) - sizeof( BootImageDescriptor_t )
    - sizeof( BootImageTrailer_t ); /*lint !e9075 !e9029 Please see comment header block above. */

typedef struct
{
    const OTA_FileContext_t * pxCurOTAFile; /* Current OTA file to be processed. */
    uint32_t ulLowImageOffset;              /* Lowest offset/address in the application image. */
    uint32_t ulHighImageOffset;             /* Highest offset/address in the application image. */
} OTA_OperationDescriptor_t;

/* NOTE that this implementation supports only one OTA at a time since it uses a single static instance. */
static OTA_OperationDescriptor_t xCurOTAOpDesc;         /* current OTA operation in progress. */
static OTA_OperationDescriptor_t * pxCurOTADesc = NULL; /* pointer to current OTA operation. */

/* The static functions below (prvPAL_CheckFileSignature and prvPAL_ReadAndAssumeCertificate) 
 * are optionally implemented. If these functions are implemented then please set the following macros in 
 * aws_test_ota_config.h to 1:
 * otatestpalCHECK_FILE_SIGNATURE_SUPPORTED
 * otatestpalREAD_AND_ASSUME_CERTIFICATE_SUPPORTED
 */

/**
 * @brief Verify the signature of the specified file.
 * 
 * This function should be implemented if signature verification is not offloaded
 * to non-volatile memory io functions.
 * 
 * This function is called from prvPAL_Close(). 
 * 
 * @param[in] C OTA file context information.
 * 
 * @return Below are the valid return values for this function.
 * kOTA_Err_None if the signature verification passes.
 * kOTA_Err_SignatureCheckFailed if the signature verification fails.
 * kOTA_Err_BadSignerCert if the if the signature verification certificate cannot be read.
 * 
 */
static OTA_Err_t prvPAL_CheckFileSignature( OTA_FileContext_t * const C );

/**
 * @brief Read the specified signer certificate from the filesystem into a local buffer.
 * 
 * The allocated memory returned becomes the property of the caller who is responsible for freeing it.
 * 
 * This function is called from prvPAL_CheckFileSignature(). It should be implemented if signature
 * verification is not offloaded to non-volatile memory io function.
 * 
 * @param[in] pucCertName The file path of the certificate file.
 * @param[out] ulSignerCertSize The size of the certificate file read.
 * 
 * @return A pointer to the signer certificate in the file system. NULL if the certificate cannot be read.
 * This returned pointer is the responsibility of the caller; if the memory is allocated the caller must free it.
 */
static uint8_t * prvPAL_ReadAndAssumeCertificate( const uint8_t * const pucCertName,
                                                  uint32_t * const ulSignerCertSize );
static CK_RV prvGetCertificateHandle( CK_FUNCTION_LIST_PTR pxFunctionList,
                                      CK_SESSION_HANDLE xSession,
                                      const char * pcLabelName,
                                      CK_OBJECT_HANDLE_PTR pxCertHandle );
static CK_RV prvGetCertificate( const char * pcLabelName,
                                uint8_t ** ppucData,
                                uint32_t * pulDataSize );
								
/*-----------------------------------------------------------*/

BaseType_t AWS_FlashEraseUpdateBank( void )
{
    BaseType_t xReturn = pdFAIL;

    for( BaseType_t cFlashSector = FLASH_SECTOR_17; cFlashSector <= FLASH_SECTOR_23; ++cFlashSector )
    {
        xReturn = AWS_FlashEraseSector( FLASH_BANK_2, cFlashSector );

        if( pdFAIL == xReturn )
        {
            break;
        }
    }

    return xReturn;
}

static inline bool_t prvContextValidate( OTA_FileContext_t *C )
{
    return( ( pxCurOTADesc != NULL ) && ( C != NULL ) &&
            ( pxCurOTADesc->pxCurOTAFile == C ) &&
            ( C->pucFile == ( uint8_t * ) pxCurOTADesc ) ); /*lint !e9034 This preserves the abstraction layer. */
}

static inline void prvContextClose( OTA_FileContext_t *C )
{
    if( NULL != C )
    {
        C->pucFile = NULL;
    }

    xCurOTAOpDesc.pxCurOTAFile = NULL;
    pxCurOTADesc = NULL;
}

static BootImageHeader_t xImgHeader;

static BootImageTrailer_t xImgTrailer;

static bool_t prvContextUpdateImageHeaderAndTrailer( OTA_FileContext_t * C )
{
    DEFINE_OTA_METHOD_NAME( "prvContextUpdateImageHeaderAndTrailer" );

    //BootImageHeader_t xImgHeader;
    BootImageDescriptor_t * pxImgDesc;
    //BootImageTrailer_t xImgTrailer;

    memcpy( xImgHeader.cImgSignature,
            AWS_BOOT_IMAGE_SIGNATURE,
            sizeof( xImgHeader.cImgSignature ) );

    xImgHeader.ucImageNewFlag = 0;

    /**
     * Regarding MISRA 2012 requirements and this function, the implementation
     * of the PAL is such that there are two OTA flash banks and each starts with
     * a descriptor structure.
     */

    /* Pointer to the app descriptor in the flash upper page. */
    pxImgDesc = ( BootImageDescriptor_t * ) AWS_FLASH_IMAGE_START; /*lint !e9078 !e923 !e9027 !e9029 !e9033 !e9079 Please see the comment header block above. */

    /* Write header to flash. */
    BaseType_t xProgResult = AWS_FlashProgramBlock( ( uint8_t * ) AWS_FLASH_IMAGE_START,
                                                    ( const uint8_t * ) &xImgHeader,
                                                    64 );

    OTA_LOG_L1( "[%s] OTA Sequence Number: %d\r\n", OTA_METHOD_NAME, pxImgDesc->ulSequenceNum );
    OTA_LOG_L1( "[%s] Image - Start: 0x%08x, End: 0x%08x\r\n", OTA_METHOD_NAME,
                pxImgDesc->pvStartAddr, pxImgDesc->pvEndAddr );

    /* If header write is successful write trailer. */
    if( pdPASS == xProgResult )
    {
        /* Create image trailer. */
        memcpy( xImgTrailer.aucSignatureType, cOTA_JSON_FileSignatureKey, sizeof( cOTA_JSON_FileSignatureKey ) );
        xImgTrailer.ulSignatureSize = C->pxSignature->usSize;
        memcpy( xImgTrailer.aucSignature, C->pxSignature->ucData, C->pxSignature->usSize );

        /* Pointer to the trailer in the flash upper page. */
        const uint8_t * pxAppImgTrailerPtr = ( const uint8_t * ) AWS_FLASH_IMAGE_START + sizeof( BootImageHeader_t ) + pxCurOTADesc->ulHighImageOffset;
        
/*         Align it to FLASHWORD_SIZE.
        if( ( ( uint32_t ) pxAppImgTrailerPtr % FLASHWORD_SIZE ) != 0 )
        {
            pxAppImgTrailerPtr += FLASHWORD_SIZE - ( ( uint32_t ) pxAppImgTrailerPtr % FLASHWORD_SIZE );
        }*/


        xProgResult = AWS_FlashProgramBlock( pxAppImgTrailerPtr,
                                             ( const uint8_t * ) &xImgTrailer,
                                             sizeof( xImgTrailer ) );

        OTA_LOG_L1( "[%s] Writing Trailer at: 0x%08x\n", OTA_METHOD_NAME, pxAppImgTrailerPtr );
    }

    return xProgResult;
}

/*
 * Refreshes the watchdog to give time for the application to set the image
 * state correctly.
 */
static void prvPAL_WatchdogRefresh( void )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_WatchdogRefresh" );

    OTA_LOG_L1( "[%s] Refresh the watchdog timer. \n", OTA_METHOD_NAME );

/*    IWDG_HandleTypeDef hiwdg1;
    
    hiwdg1.Instance = IWDG1;
    HAL_IWDG_Refresh( &hiwdg1 );*/

/* adopted for H7 -> F7 porting */
    IWDG_HandleTypeDef hiwdg;

    hiwdg.Instance = IWDG;
    HAL_IWDG_Refresh( &hiwdg );
}

OTA_Err_t prvPAL_CreateFileForRx( OTA_FileContext_t * const C )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_CreateFileForRx" );

    int32_t lErr = ST_ERR_NONE;
    OTA_Err_t xReturnCode = kOTA_Err_Uninitialized;

    /* Check parameters. The filepath is unused on this platform so ignore it. */
    if( NULL == C )
    {
        OTA_LOG_L1( "[%s] Error: context pointer is null.\r\n", OTA_METHOD_NAME );
        lErr = ST_ERR_INVALID_CONTEXT;
    }
    else
    {
        if( AWS_FlashEraseUpdateBank() == pdFALSE )
        {
            OTA_LOG_L1( "[%s] Error: Failed to erase the flash!\r\n", OTA_METHOD_NAME );
            lErr = ST_ERR_FLASH_ERASE_FAIL;
        }
        else
        {
            pxCurOTADesc = &xCurOTAOpDesc;
            pxCurOTADesc->pxCurOTAFile = C;
            pxCurOTADesc->ulLowImageOffset = ulFlashImageMaxSize;
            pxCurOTADesc->ulHighImageOffset = 0;

            OTA_LOG_L1( "[%s] Receive file created.\r\n", OTA_METHOD_NAME );
            C->pucFile = ( uint8_t * ) pxCurOTADesc;
        }
    }

    if( ST_ERR_NONE == lErr )
    {
        xReturnCode = kOTA_Err_None;
    }
    else
    {
        xReturnCode = ( uint32_t ) kOTA_Err_RxFileCreateFailed | ( ( ( uint32_t ) lErr ) & ( uint32_t ) kOTA_PAL_ErrMask ); /*lint !e571 intentionally cast lErr to larger composite error code. */
    }
    return xReturnCode;
}
/*-----------------------------------------------------------*/

OTA_Err_t prvPAL_Abort( OTA_FileContext_t * const C )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_Abort" );

    /* Check for null file handle since we may call this before a file is actually opened. */
    prvContextClose( C );
    OTA_LOG_L1( "[%s] Abort - OK\r\n", OTA_METHOD_NAME );

    return kOTA_Err_None;
}
/*-----------------------------------------------------------*/

/* Write a block of data to the specified file. */
int16_t prvPAL_WriteBlock( OTA_FileContext_t * const C,
                           uint32_t ulOffset,
                           uint8_t * const pacData,
                           uint32_t ulBlockSize )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_WriteBlock" );
	
    int16_t sReturnVal = 0;
    uint8_t * pucWriteData = pacData;
    uint32_t ulWriteBlockSize = ulBlockSize;

    if( prvContextValidate( C ) == ( bool_t ) pdFALSE )
    {
        sReturnVal = ST_ERR_INVALID_CONTEXT;
    }
    else if( ( ulOffset + ulBlockSize ) > ulFlashImageMaxSize )
    {   /* invalid address. */
        sReturnVal = ST_ERR_ADDR_OUT_OF_RANGE;
    }
    else /* Update the image offsets. */
    {
        if( ulOffset < pxCurOTADesc->ulLowImageOffset )
        {
            pxCurOTADesc->ulLowImageOffset = ulOffset;
        }

        if( ( ulOffset + ulBlockSize ) > pxCurOTADesc->ulHighImageOffset )
        {
            pxCurOTADesc->ulHighImageOffset = ulOffset + ulBlockSize;
        }
        
        const uint8_t * pucFlashAddr = ( const uint8_t * ) ( AWS_FLASH_IMAGE_START + sizeof( BootImageHeader_t ) + ulOffset );

        if( AWS_FlashProgramBlock( pucFlashAddr, pucWriteData, ulWriteBlockSize ) == pdFALSE )
        {   /* Failed to program block to flash. */
            sReturnVal = ST_ERR_FLASH_WRITE_FAIL;
        }
        else
        {   /* Success. */
            sReturnVal = ( int16_t ) ulBlockSize;
        }
    }

    return sReturnVal;
}
/*-----------------------------------------------------------*/

OTA_Err_t prvPAL_CloseFile( OTA_FileContext_t * const C )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_CloseFile" );

    OTA_Err_t eResult = kOTA_Err_None;

    if( prvContextValidate( C ) == ( bool_t ) pdFALSE )
    {
        eResult = kOTA_Err_FileClose;
    }

    if( kOTA_Err_None == eResult )
    {
        /* Verify that a block has actually been written by checking that the high image offset
         *  is greater than the low image offset. If that is not the case, then an invalid memory location
         *  may get passed to CRYPTO_SignatureVerificationUpdate, resulting in a data bus error. */
        if( ( C->pxSignature != NULL ) &&
            ( pxCurOTADesc->ulHighImageOffset > pxCurOTADesc->ulLowImageOffset ) )
        {
            OTA_LOG_L1( "[%s] Authenticating and closing file.\r\n", OTA_METHOD_NAME );

            /* Verify the file signature, close the file and return the signature verification result. */
            eResult = prvPAL_CheckFileSignature( C );
        }
        else
        {
            eResult = kOTA_Err_SignatureCheckFailed;
        }
    }

    if( kOTA_Err_None == eResult )
    {
        /* Update the image header. */
        OTA_LOG_L1( "[%s] %s signature verification passed.\r\n", OTA_METHOD_NAME, cOTA_JSON_FileSignatureKey );

        if( prvContextUpdateImageHeaderAndTrailer( C ) == ( bool_t ) pdTRUE )
        {
            OTA_LOG_L1( "[%s] Image header updated.\r\n", OTA_METHOD_NAME );
        }
        else
        {
            OTA_LOG_L1( "[%s] ERROR: Failed to update the image header.\r\n", OTA_METHOD_NAME );
            eResult = kOTA_Err_FileClose;
        }
    }
    else
    {
        OTA_LOG_L1( "[%s] ERROR: Failed to pass %s signature verification: %d.\r\n", OTA_METHOD_NAME,
                    cOTA_JSON_FileSignatureKey, eResult );
    }

    prvContextClose( C );
    return eResult;
}
/*-----------------------------------------------------------*/


static OTA_Err_t prvPAL_CheckFileSignature( OTA_FileContext_t * const C )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_CheckFileSignature" );

    OTA_Err_t eResult;
    uint32_t ulSignerCertSize;
    void * pvSigVerifyContext;
    uint8_t * pucSignerCert = NULL;

    /* Verify an ECDSA-SHA256 signature. */
    if( CRYPTO_SignatureVerificationStart( &pvSigVerifyContext, cryptoASYMMETRIC_ALGORITHM_ECDSA,
                                           cryptoHASH_ALGORITHM_SHA256 ) == pdFALSE )
    {
        eResult = kOTA_Err_SignatureCheckFailed;
    }
    else
    {
        OTA_LOG_L1( "[%s] Started %s signature verification, file: %s\r\n", OTA_METHOD_NAME,
                    cOTA_JSON_FileSignatureKey, ( const char * ) C->pucCertFilepath );
        pucSignerCert = prvPAL_ReadAndAssumeCertificate( ( const uint8_t * const ) C->pucCertFilepath, &ulSignerCertSize );

        if( pucSignerCert == NULL )
        {
            eResult = kOTA_Err_BadSignerCert;
        }
        else
        {
            uint8_t * pucFlashAddr = ( uint8_t * ) ( AWS_FLASH_IMAGE_START + sizeof( BootImageHeader_t ) + pxCurOTADesc->ulLowImageOffset );
            CRYPTO_SignatureVerificationUpdate( pvSigVerifyContext, pucFlashAddr,
                                                pxCurOTADesc->ulHighImageOffset - pxCurOTADesc->ulLowImageOffset );

            if( CRYPTO_SignatureVerificationFinal( pvSigVerifyContext, ( char * ) pucSignerCert, ulSignerCertSize,
                                                   C->pxSignature->ucData, C->pxSignature->usSize ) == pdFALSE )
            {
                eResult = kOTA_Err_SignatureCheckFailed;

                /* Erase the image as signature verification failed.*/
               if( AWS_FlashEraseUpdateBank() == pdFALSE )
                {
                    OTA_LOG_L1( "[%s] Error: Failed to erase the flash !\r\n", OTA_METHOD_NAME );
                }
            }
            else
            {
                eResult = kOTA_Err_None;
            }
        }
    }

    /* Free the signer certificate that we now own after prvPAL_ReadAndAssumeCertificate(). */
    if( pucSignerCert != NULL )
    {
        vPortFree( pucSignerCert );
    }

    return eResult;
}
/*-----------------------------------------------------------*/

static CK_RV prvGetCertificateHandle( CK_FUNCTION_LIST_PTR pxFunctionList,
                                      CK_SESSION_HANDLE xSession,
                                      const char * pcLabelName,
                                      CK_OBJECT_HANDLE_PTR pxCertHandle )
{
    CK_ATTRIBUTE xTemplate;
    CK_RV xResult = CKR_OK;
    CK_ULONG ulCount = 0;
    CK_BBOOL xFindInit = CK_FALSE;

    /* Get the certificate handle. */
    if( 0 == xResult )
    {
        xTemplate.type = CKA_LABEL;
        xTemplate.ulValueLen = strlen( pcLabelName ) + 1;
        xTemplate.pValue = ( char * ) pcLabelName;
        xResult = pxFunctionList->C_FindObjectsInit( xSession, &xTemplate, 1 );
    }

    if( 0 == xResult )
    {
        xFindInit = CK_TRUE;
        xResult = pxFunctionList->C_FindObjects( xSession,
                                                 ( CK_OBJECT_HANDLE_PTR ) pxCertHandle,
                                                 1,
                                                 &ulCount );
    }

    if( CK_TRUE == xFindInit )
    {
        xResult = pxFunctionList->C_FindObjectsFinal( xSession );
    }

    return xResult;
}

/* Note that this function mallocs a buffer for the certificate to reside in,
 * and it is the responsibility of the caller to free the buffer. */
static CK_RV prvGetCertificate( const char * pcLabelName,
                                uint8_t ** ppucData,
                                uint32_t * pulDataSize )
{
    /* Find the certificate */
    CK_OBJECT_HANDLE xHandle;
    CK_RV xResult;
    CK_FUNCTION_LIST_PTR xFunctionList;
    CK_SLOT_ID xSlotId;
    CK_ULONG xCount = 1;
    CK_SESSION_HANDLE xSession;
    CK_ATTRIBUTE xTemplate = { 0 };
    uint8_t * pucCert = NULL;
    CK_BBOOL xSessionOpen = CK_FALSE;

    xResult = C_GetFunctionList( &xFunctionList );

    if( CKR_OK == xResult )
    {
        xResult = xFunctionList->C_Initialize( NULL );
    }

    if( ( CKR_OK == xResult ) || ( CKR_CRYPTOKI_ALREADY_INITIALIZED == xResult ) )
    {
        xResult = xFunctionList->C_GetSlotList( CK_TRUE, &xSlotId, &xCount );
    }

    if( CKR_OK == xResult )
    {
        xResult = xFunctionList->C_OpenSession( xSlotId, CKF_SERIAL_SESSION, NULL, NULL, &xSession );
    }

    if( CKR_OK == xResult )
    {
        xSessionOpen = CK_TRUE;
        xResult = prvGetCertificateHandle( xFunctionList, xSession, pcLabelName, &xHandle );
    }

    if( ( xHandle != 0 ) && ( xResult == CKR_OK ) ) /* 0 is an invalid handle */
    {
        /* Get the length of the certificate */
        xTemplate.type = CKA_VALUE;
        xTemplate.pValue = NULL;
        xResult = xFunctionList->C_GetAttributeValue( xSession, xHandle, &xTemplate, xCount );

        if( xResult == CKR_OK )
        {
            pucCert = pvPortMalloc( xTemplate.ulValueLen );
        }

        if( ( xResult == CKR_OK ) && ( pucCert == NULL ) )
        {
            xResult = CKR_HOST_MEMORY;
        }

        if( xResult == CKR_OK )
        {
            xTemplate.pValue = pucCert;
            xResult = xFunctionList->C_GetAttributeValue( xSession, xHandle, &xTemplate, xCount );

            if( xResult == CKR_OK )
            {
                *ppucData = pucCert;
                *pulDataSize = xTemplate.ulValueLen;
            }
            else
            {
                vPortFree( pucCert );
            }
        }
    }
    else /* Certificate was not found. */
    {
        *ppucData = NULL;
        *pulDataSize = 0;
    }

    if( xSessionOpen == CK_TRUE )
    {
        ( void ) xFunctionList->C_CloseSession( xSession );
    }

    return xResult;
}

/* Read the specified signer certificate from the filesystem into a local buffer. The
 * allocated memory becomes the property of the caller who is responsible for freeing it.
 */
static uint8_t * prvPAL_ReadAndAssumeCertificate( const uint8_t * const pucCertName,
                                                  uint32_t * const ulSignerCertSize )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_ReadAndAssumeCertificate" );

    uint8_t * pucCertData;
    uint32_t ulCertSize;
    uint8_t * pucSignerCert = NULL;
    CK_RV xResult;

    xResult = prvGetCertificate( ( const char * ) pucCertName, &pucSignerCert, ulSignerCertSize );

    if( ( xResult == CKR_OK ) && ( pucSignerCert != NULL ) )
    {
        OTA_LOG_L1( "[%s] Using cert with label: %s OK\r\n", OTA_METHOD_NAME, ( const char * ) pucCertName );
    }
    else
    {
        OTA_LOG_L1( "[%s] No such certificate file: %s. Using aws_ota_codesigner_certificate.h.\r\n", OTA_METHOD_NAME,
                    ( const char * ) pucCertName );

        /* Allocate memory for the signer certificate plus a terminating zero so we can copy it and return to the caller. */
        ulCertSize = sizeof( signingcredentialSIGNING_CERTIFICATE_PEM );
        pucSignerCert = pvPortMalloc( ulCertSize + 1 );                       /*lint !e9029 !e9079 !e838 malloc proto requires void*. */
        pucCertData = ( uint8_t * ) signingcredentialSIGNING_CERTIFICATE_PEM; /*lint !e9005 we don't modify the cert but it could be set by PKCS11 so it's not const. */

        if( pucSignerCert != NULL )
        {
            memcpy( pucSignerCert, pucCertData, ulCertSize );
            /* The crypto code requires the terminating zero to be part of the length so add 1 to the size. */
            pucSignerCert[ ulCertSize ] = 0U;
            *ulSignerCertSize = ulCertSize + 1U;
        }
        else
        {
            OTA_LOG_L1( "[%s] Error: No memory for certificate of size %d!\r\n", OTA_METHOD_NAME, ulCertSize );
        }
    }

    return pucSignerCert;
}
/*-----------------------------------------------------------*/

OTA_Err_t prvPAL_ResetDevice( void )
{
    DEFINE_OTA_METHOD_NAME("prvPAL_ResetDevice");

    OTA_LOG_L1( "[%s] Resetting the device.\r\n", OTA_METHOD_NAME );

    /* Short delay for debug log output before reset. */
    vTaskDelay( OTA_HALF_SECOND_DELAY );
    HAL_NVIC_SystemReset( );
    
    /* We shouldn't actually get here if the board supports the auto reset.
     * But, it doesn't hurt anything if we do although someone will need to
     * reset the device for the new image to boot. */
    return kOTA_Err_None;
}
/*-----------------------------------------------------------*/

OTA_Err_t prvPAL_ActivateNewImage( void )
{
    DEFINE_OTA_METHOD_NAME("prvPAL_ActivateNewImage");

    OTA_LOG_L1( "[%s] Activating the new MCU image.\r\n", OTA_METHOD_NAME );
    return prvPAL_ResetDevice();
}
/*-----------------------------------------------------------*/

static BootImageDescriptor_t xDescCopy;

OTA_Err_t prvPAL_SetPlatformImageState( OTA_ImageState_t eState )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_SetPlatformImageState" );

    //BootImageDescriptor_t xDescCopy;
    OTA_Err_t eResult = kOTA_Err_Uninitialized;

    /* Descriptor handle for the image being executed, which is always the lower bank. */
    const BootImageDescriptor_t * pxAppImgDesc;
    pxAppImgDesc = ( const BootImageDescriptor_t * ) ( 0x08040000UL ); /*lint !e923 !e9027 !e9029 !e9033 !e9079 !e9078 !e9087 Please see earlier lint comment header. */
    xDescCopy = *pxAppImgDesc;  /* Copy image descriptor from flash into RAM struct. */

    /* This should be an image launched in self test mode! */
    uint8_t ucImageFlags = eBootImageFlagInvalid;
    
    if( xDescCopy.xImgHeader.ucImageInvalidFlag == 0 )
    {
      ucImageFlags = eBootImageFlagInvalid;
    }
    else
    {
      if( xDescCopy.xImgHeader.ucImageValidFlag == 0 )
      {
        ucImageFlags = eBootImageFlagValid;
      }
      else if( xDescCopy.xImgHeader.ucImageCommitPendingFlag == 0 )
      {
        ucImageFlags = eBootImageFlagCommitPending;
      }
      else if( xDescCopy.xImgHeader.ucImageNewFlag == 0 )
      {
        ucImageFlags = eBootImageFlagNew;
      }
    }

    if( ucImageFlags == AWS_BOOT_FLAG_IMG_PENDING_COMMIT )
    {
        if(eState == eOTA_ImageState_Accepted)
        {
            /* Mark the image as valid */
            xDescCopy.xImgHeader.ucImageValidFlag = 0;

            if( AWS_FlashProgramBlock( (uint8_t *) &pxAppImgDesc->xImgHeader.ulAlign[24],
                                       (uint8_t *) &xDescCopy.xImgHeader.ucImageValidFlag,
                                       32 ) == pdTRUE )
            {
                OTA_LOG_L1( "[%s] Accepted and committed final image.\r\n", OTA_METHOD_NAME );

                /* Refreshes the watchdog timer. */
                prvPAL_WatchdogRefresh();

                /* We always execute from the lower bank and self-test is good, we should erase the older
                 * version of the firmware by doing bank erase on upper bank. */
                if( AWS_FlashEraseUpdateBank() == pdFALSE )
                {
                    OTA_LOG_L1( "[%s] Warning: Failed to erase the other image!\r\n", OTA_METHOD_NAME );
                }

                eResult = kOTA_Err_None;
            }
            else
            {
                OTA_LOG_L1( "[%s] Accepted final image but commit failed (%d).\r\n", OTA_METHOD_NAME,
                            ST_ERR_FLASH_WRITE_FAIL );
                eResult = ( uint32_t ) kOTA_Err_CommitFailed | ( ( ( uint32_t ) ST_ERR_FLASH_WRITE_FAIL ) & ( uint32_t ) kOTA_PAL_ErrMask );
            }
        }
        else if( eState == eOTA_ImageState_Rejected )
        {
            /* Mark the image as invalid */
            xDescCopy.xImgHeader.ucImageInvalidFlag = 0;

            if( AWS_FlashProgramBlock( ( uint8_t * ) &pxAppImgDesc->xImgHeader.ulAlign[32],
                                       ( uint8_t * ) &xDescCopy.xImgHeader.ucImageInvalidFlag,
                                       32 ) == pdTRUE )
            {
                OTA_LOG_L1( "[%s] Rejected image.\r\n", OTA_METHOD_NAME );

                eResult = kOTA_Err_None;
            }
            else
            {
                OTA_LOG_L1( "[%s] Failed updating the flags.(%d).\r\n", OTA_METHOD_NAME,
                            ST_ERR_FLASH_WRITE_FAIL );
                eResult = ( uint32_t ) kOTA_Err_RejectFailed | ( ( ( uint32_t ) ST_ERR_FLASH_WRITE_FAIL ) & ( uint32_t ) kOTA_PAL_ErrMask );
            }
        }
        else if( eState == eOTA_ImageState_Aborted )
        {
            /* Mark the image as invalid */
            xDescCopy.xImgHeader.ucImageInvalidFlag = 0;

            if( AWS_FlashProgramBlock( ( uint8_t * ) &pxAppImgDesc->xImgHeader.ulAlign[32],
                                       ( uint8_t * ) &xDescCopy.xImgHeader.ucImageInvalidFlag,
                                       32 ) == pdTRUE )
            {
                OTA_LOG_L1( "[%s] Aborted image.\r\n", OTA_METHOD_NAME );

                eResult = kOTA_Err_None;
            }
            else
            {
                OTA_LOG_L1( "[%s] Failed updating the flags.(%d).\r\n", OTA_METHOD_NAME,
                            ST_ERR_FLASH_WRITE_FAIL );
                eResult = ( uint32_t ) kOTA_Err_AbortFailed | ( ( ( uint32_t ) ST_ERR_FLASH_WRITE_FAIL ) & ( uint32_t ) kOTA_PAL_ErrMask );
            }
        }
        else if( eState == eOTA_ImageState_Testing )
        {
            eResult = kOTA_Err_None;
        }
        else
        {
            OTA_LOG_L1( "[%s] Unknown state received %d.\r\n", OTA_METHOD_NAME, ( int32_t ) eState );
            eResult = kOTA_Err_BadImageState;
        }
    }
    else
    {
        /* Not in self-test mode so get the descriptor for image in upper bank. */
        pxAppImgDesc = ( const BootImageDescriptor_t * ) AWS_FLASH_IMAGE_START;
        xDescCopy = *pxAppImgDesc;

        if( eState == eOTA_ImageState_Accepted )
        {
            /* We are not in self-test mode so can not set the image in upper bank as valid.  */
            OTA_LOG_L1( "[%s] Not in commit pending so can not mark image valid (%d).\r\n", OTA_METHOD_NAME,
                                ST_ERR_NOT_PENDING_COMMIT );
            eResult = ( uint32_t ) kOTA_Err_CommitFailed | ( ( ( uint32_t ) ST_ERR_NOT_PENDING_COMMIT ) & ( uint32_t ) kOTA_PAL_ErrMask );
        }
        else if( eState == eOTA_ImageState_Rejected )
        {
            OTA_LOG_L1( "[%s] Rejected image.\r\n", OTA_METHOD_NAME );

            /* The OTA on program image bank (upper bank) is rejected so erase the bank.  */
            if( AWS_FlashEraseUpdateBank() == pdFALSE )
            {
                OTA_LOG_L1( "[%s] Error: Failed to erase the flash! (%d).\r\n", OTA_METHOD_NAME,
                             ST_ERR_FLASH_ERASE_FAIL );
                eResult = ( uint32_t ) kOTA_Err_RejectFailed | ( ( ( uint32_t ) ST_ERR_FLASH_ERASE_FAIL ) & ( uint32_t ) kOTA_PAL_ErrMask );
            }
            else
            {
                eResult = kOTA_Err_None;
            }
        }
        else if( eState == eOTA_ImageState_Aborted )
        {
            OTA_LOG_L1( "[%s] Aborted image.\r\n", OTA_METHOD_NAME );

            /* The OTA on program image bank (upper bank) is aborted so erase the bank.  */
            if( AWS_FlashEraseUpdateBank() == pdFALSE )
            {
                OTA_LOG_L1( "[%s] Error: Failed to erase the flash! (%d).\r\n", OTA_METHOD_NAME,
                             ST_ERR_FLASH_ERASE_FAIL );
                eResult = ( uint32_t ) kOTA_Err_AbortFailed | ( ( ( uint32_t ) ST_ERR_FLASH_ERASE_FAIL ) & ( uint32_t ) kOTA_PAL_ErrMask );
            }
            else
            {
                eResult = kOTA_Err_None;
            }
        }
        else if( eState == eOTA_ImageState_Testing )
        {
            eResult = kOTA_Err_None;
        }
        else
        {
            eResult = kOTA_Err_BadImageState;
        }
    }

    return eResult;
}
/*-----------------------------------------------------------*/

OTA_PAL_ImageState_t prvPAL_GetPlatformImageState( void )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_GetPlatformImageState" );
    
    //BootImageDescriptor_t xDescCopy;
    OTA_PAL_ImageState_t eImageState = eOTA_PAL_ImageState_Invalid;
    const BootImageDescriptor_t * pxAppImgDesc;

    pxAppImgDesc = ( const BootImageDescriptor_t * ) 0x08040000UL; /*lint !e923 !e9027 !e9029 !e9033 !e9079 !e9078 !e9087 Please see earlier lint comment header. */
    xDescCopy = *pxAppImgDesc;

    uint8_t ucImageFlags = eBootImageFlagInvalid;
    
    if( xDescCopy.xImgHeader.ucImageInvalidFlag == 0 )
    {
      ucImageFlags = eBootImageFlagInvalid;
    }
    else
    {
      if( xDescCopy.xImgHeader.ucImageValidFlag == 0 )
      {
        ucImageFlags = eBootImageFlagValid;
      }
      else if( xDescCopy.xImgHeader.ucImageCommitPendingFlag == 0 )
      {
        ucImageFlags = eBootImageFlagCommitPending;
      }
      else if( xDescCopy.xImgHeader.ucImageNewFlag == 0 )
      {
        ucImageFlags = eBootImageFlagNew;
      }
    }

    /**
     *  Check if valid magic code is present for the application image in lower bank.
     */
    if( memcmp( pxAppImgDesc->xImgHeader.cImgSignature,
                AWS_BOOT_IMAGE_SIGNATURE,
                AWS_BOOT_IMAGE_SIGNATURE_SIZE ) == 0 )
    {

        if( ucImageFlags == AWS_BOOT_FLAG_IMG_PENDING_COMMIT )
        {
            /* Pending Commit means we're in the Self Test phase. */
            eImageState = eOTA_PAL_ImageState_PendingCommit;
        }
        else
        {
            /* The commit pending flag for application in lower bank is not set so we are not in self-test phase
             so use the header flags from program bank(upper bank). */
            pxAppImgDesc = ( const BootImageDescriptor_t * ) AWS_FLASH_IMAGE_START;
            xDescCopy = *pxAppImgDesc;
            
            if( xDescCopy.xImgHeader.ucImageInvalidFlag == 0 )
            {
              ucImageFlags = eBootImageFlagInvalid;
            }
            else
            {
              if( xDescCopy.xImgHeader.ucImageValidFlag == 0 )
              {
                ucImageFlags = eBootImageFlagValid;
              }
              else if( xDescCopy.xImgHeader.ucImageCommitPendingFlag == 0 )
              {
                ucImageFlags = eBootImageFlagCommitPending;
              }
              else if( xDescCopy.xImgHeader.ucImageNewFlag == 0 )
              {
                ucImageFlags = eBootImageFlagNew;
              }
            }
            
            /**
             *  Check if valid magic code is present for the application image in upper bank.
             */
            if( memcmp( pxAppImgDesc->xImgHeader.cImgSignature,
                AWS_BOOT_IMAGE_SIGNATURE,
                AWS_BOOT_IMAGE_SIGNATURE_SIZE ) == 0 )
            {

                switch( ucImageFlags )
                {
                    case AWS_BOOT_FLAG_IMG_PENDING_COMMIT:
                    {
                        eImageState = eOTA_PAL_ImageState_PendingCommit;
                        break;
                    }
                    case AWS_BOOT_FLAG_IMG_VALID:
                    case AWS_BOOT_FLAG_IMG_NEW:
                    {
                        eImageState = eOTA_PAL_ImageState_Valid;
                        break;
                    }
                    default:
                    {
                        eImageState = eOTA_PAL_ImageState_Invalid;
                        break;
                    }
                }
            }

        }
    }

    return eImageState;
}
/*-----------------------------------------------------------*/

/* Provide access to private members for testing. */
#ifdef AMAZON_FREERTOS_ENABLE_UNIT_TESTS
    #include "aws_ota_pal_test_access_define.h"
#endif

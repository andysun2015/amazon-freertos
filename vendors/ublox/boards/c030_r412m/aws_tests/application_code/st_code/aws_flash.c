/**
 ******************************************************************************
 * File Name          : aws_flash.c
 * Description        : This file provides code for writing and erasing
 * 					 the FLASH.
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

#include "string.h"
#include "stm32f7xx_hal.h"
#include "FreeRTOS.h"

#include "aws_flash.h"

BaseType_t AWS_FlashProgramBlock( const uint8_t * pucAddress,
                                  const uint8_t * pucData,
                                  int32_t lSize )
{
    uint32_t ulAddress 	= (uint32_t) pucAddress;
    uint32_t ullData 	= (uint32_t) pucData;
    //uint8_t xWriteBuffer[ FLASHWORD_SIZE ];

    HAL_FLASH_Unlock();

    /*
     *  If the address is not aligned to a 256-bit boundary, the write must be
     *  offset by preserving the preceding bits.
     */
/*    if( ulAddress % FLASHWORD_SIZE != 0 )
    {
        uint32_t ulBaseAddress = ( ulAddress / FLASHWORD_SIZE ) * FLASHWORD_SIZE;
        uint32_t ulOffset = ulAddress - ulBaseAddress;
        uint32_t ulWriteLength = ( lSize <= ( FLASHWORD_SIZE - ulOffset ) )? lSize : ( FLASHWORD_SIZE - ulOffset );


         *  Fill the write buffer with 0xFF (empty) bytes

        memset( xWriteBuffer, 0xFF, FLASHWORD_SIZE );


         *  Copy the original bytes from FLASH

        memcpy( xWriteBuffer, ( uint8_t * ) ulBaseAddress, FLASHWORD_SIZE - ulOffset );


         *  Copy the new bytes from the buffer

        memcpy( xWriteBuffer + ulOffset, ( uint8_t * ) ullData, ulWriteLength );

        //if ( HAL_OK == HAL_FLASH_Program( FLASH_TYPEPROGRAM_FLASHWORD, ulBaseAddress, (uint64_t) (uint32_t) xWriteBuffer ) )
        // adapt from h7 --> f7
        if ( HAL_OK == HAL_FLASH_Program( FLASH_TYPEPROGRAM_DOUBLEWORD, ulBaseAddress, (uint64_t) (uint32_t) xWriteBuffer ) )
        {
            ulAddress = ulBaseAddress + FLASHWORD_SIZE;
            ullData += ulWriteLength;
            lSize -= ulWriteLength;
        }
        else
        {
            HAL_FLASH_Lock();

            return pdFAIL;
        }
    }*/

    while( lSize > 0 )
    {
/*        if( lSize < FLASHWORD_SIZE )
        {
            memset( xWriteBuffer, 0xFF, FLASHWORD_SIZE );
            memcpy( xWriteBuffer, ( uint8_t * ) ullData, lSize );
            //ullData = ( uint64_t ) ( uint32_t ) xWriteBuffer;
            ullData = ( uint32_t ) xWriteBuffer;
        }*/
        //if ( HAL_OK == HAL_FLASH_Program( FLASH_TYPEPROGRAM_FLASHWORD, ulAddress, ullData ) )
        // adapt from h7 --> f7
        if ( HAL_OK == HAL_FLASH_Program( FLASH_TYPEPROGRAM_BYTE, ulAddress, (uint64_t)(*(uint8_t *)ullData) ) )
        {
            //ulAddress += FLASHWORD_SIZE;
            //ullData += FLASHWORD_SIZE;
            //lSize -= FLASHWORD_SIZE;

            ulAddress ++;
            ullData ++;
            lSize --;
        }
        else
        {
            HAL_FLASH_Lock();

            return pdFAIL;
        }
    }

    HAL_FLASH_Lock();

    return pdPASS;
}

BaseType_t AWS_FlashEraseSector( BaseType_t cBank, BaseType_t cSector )
{
    BaseType_t xReturn = pdFAIL;

    uint32_t SectorError = 0xFFFFFFFFUL;

    /* Program this new file in the upper flash bank. */
    FLASH_EraseInitTypeDef xEraseInit =
    {
        .TypeErase      = FLASH_TYPEERASE_SECTORS,
        .Banks          = cBank,
        .Sector         = cSector,
        .NbSectors      = 1,
        //.VoltageRange   = FLASH_VOLTAGE_RANGE_1
        // adapt from h7 --> f7
		.VoltageRange   = FLASH_VOLTAGE_RANGE_3
    };

    HAL_FLASH_Unlock();

/*    if( FLASH_BANK_1 == cBank )
    {
        __HAL_FLASH_CLEAR_FLAG_BANK1( FLASH_FLAG_ALL_BANK1 );
    }
    else
    {
        __HAL_FLASH_CLEAR_FLAG_BANK2( FLASH_FLAG_ALL_BANK2 );
    }*/

    // adapt from h7 --> f7
    if(( cBank == FLASH_BANK_1 ) || ( cBank == FLASH_BANK_2 ))
    {
        __HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_ALL_ERRORS );
    }

    if( HAL_OK == HAL_FLASHEx_Erase( &xEraseInit, &SectorError ) )
    {
        xReturn = pdPASS;
    }

    return xReturn;
}

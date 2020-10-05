/* Implementation of safe malloc which returns NULL if the requested
 * size is 0.  Warning: The behavior of malloc(0) is platform
 * dependent.  It is possible for malloc(0) to return an address
 * without allocating memory.
 */
void *safeMalloc(size_t xWantedSize) 
{
    return nondet_bool() ? malloc(xWantedSize) : NULL;
}

void * pvPortMalloc( size_t xWantedSize )
{
    return safeMalloc(xWantedSize);
}

void vPortFree( void * pv )
{
    ( void ) pv;
    free( pv );
}

void allocateSocket( CellularHandle_t pCellularHandle )
{
    CellularContext_t * pContext = ( CellularContext_t * ) pCellularHandle;
    CellularSocketContext_t * pSocketData = NULL;
    uint8_t socketId = 0;

    for( socketId = 0; socketId < CELLULAR_NUM_SOCKET_MAX; socketId++ )
    {
        if( pContext->pSocketData[ socketId ] == NULL )
        {
            pSocketData = ( CellularSocketContext_t * ) safeMalloc( sizeof( CellularSocketContext_t ) );
            if( pSocketData != NULL )
            {
                pSocketData->socketId = socketId;
                pContext->pSocketData[ socketId ] = pSocketData;
            }
        }
    }
}


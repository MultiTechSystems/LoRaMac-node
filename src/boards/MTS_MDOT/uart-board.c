/*!
 * \file      uart-board.c
 *
 * \brief     Target board UART driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include "stm32f4xx.h"
#include "utilities.h"
#include "board.h"
#include "uart-board.h"

/*!
 * Number of times the UartPutBuffer will try to send the buffer before
 * returning ERROR
 */
#define TX_BUFFER_RETRY_COUNT                       10

static UART_HandleTypeDef Uart1Handle;
static UART_HandleTypeDef Uart2Handle;
uint8_t RxData = 0;
uint8_t TxData = 0;

extern Uart_t Uart1;
extern Uart_t Uart2;

void UartMcuInit( Uart_t *obj, UartId_t uartId, PinNames tx, PinNames rx )
{
    obj->UartId = uartId;

    if( uartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        UartUsbInit( obj, uartId, NC, NC );
#endif
    }
    else if (uartId == UART_1)
    {
        __HAL_RCC_USART1_FORCE_RESET( );
        __HAL_RCC_USART1_RELEASE_RESET( );
        __HAL_RCC_USART1_CLK_ENABLE( );

        GpioInit( &obj->Tx, tx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF7_USART1 );
        GpioInit( &obj->Rx, rx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF7_USART1 );
    }
    else if (uartId == UART_2)
    {
        __HAL_RCC_USART2_FORCE_RESET( );
        __HAL_RCC_USART2_RELEASE_RESET( );
        __HAL_RCC_USART2_CLK_ENABLE( );

        GpioInit( &obj->Tx, tx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF7_USART2 );
        GpioInit( &obj->Rx, rx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF7_USART2 );
    }
}

void UartMcuConfig( Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl )
{
    if( obj->UartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        UartUsbConfig( obj, mode, baudrate, wordLength, stopBits, parity, flowCtrl );
#endif
    }
    else
    {
        UART_HandleTypeDef* handle;
        IRQn_Type irqn;
        if ( obj->UartId == UART_1 )
        {
            handle = &Uart1Handle;
            irqn = USART1_IRQn;
            handle->Instance = USART1;
        }
        else if ( obj-> UartId == UART_2 )
        {
            handle = &Uart2Handle;
            irqn = USART2_IRQn;
            handle->Instance = USART2;
        }
        else
        {
            return;
        }
        
        handle->Init.BaudRate = baudrate;

        if( mode == TX_ONLY )
        {
            if( obj->FifoTx.Data == NULL )
            {
                assert_param( FAIL );
            }
            handle->Init.Mode = UART_MODE_TX;
        }
        else if( mode == RX_ONLY )
        {
            if( obj->FifoRx.Data == NULL )
            {
                assert_param( FAIL );
            }
            handle->Init.Mode = UART_MODE_RX;
        }
        else if( mode == RX_TX )
        {
            if( ( obj->FifoTx.Data == NULL ) || ( obj->FifoRx.Data == NULL ) )
            {
                assert_param( FAIL );
            }
            handle->Init.Mode = UART_MODE_TX_RX;
        }
        else
        {
            assert_param( FAIL );
        }

        if( wordLength == UART_8_BIT )
        {
            handle->Init.WordLength = UART_WORDLENGTH_8B;
        }
        else if( wordLength == UART_9_BIT )
        {
            handle->Init.WordLength = UART_WORDLENGTH_9B;
        }

        switch( stopBits )
        {
        case UART_2_STOP_BIT:
            handle->Init.StopBits = UART_STOPBITS_2;
            break;
        case UART_1_STOP_BIT:
        default:
            handle->Init.StopBits = UART_STOPBITS_1;
            break;
        }

        if( parity == NO_PARITY )
        {
            handle->Init.Parity = UART_PARITY_NONE;
        }
        else if( parity == EVEN_PARITY )
        {
            handle->Init.Parity = UART_PARITY_EVEN;
        }
        else
        {
            handle->Init.Parity = UART_PARITY_ODD;
        }

        if( flowCtrl == NO_FLOW_CTRL )
        {
            handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
        }
        else if( flowCtrl == RTS_FLOW_CTRL )
        {
            handle->Init.HwFlowCtl = UART_HWCONTROL_RTS;
        }
        else if( flowCtrl == CTS_FLOW_CTRL )
        {
            handle->Init.HwFlowCtl = UART_HWCONTROL_CTS;
        }
        else if( flowCtrl == RTS_CTS_FLOW_CTRL )
        {
            handle->Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
        }

        handle->Init.OverSampling = UART_OVERSAMPLING_16;

        if( HAL_UART_Init( handle ) != HAL_OK )
        {
            assert_param( FAIL );
        }

        HAL_NVIC_SetPriority( irqn, 1, 0 );
        HAL_NVIC_EnableIRQ( irqn );

        /* Enable the UART Data Register not empty Interrupt */
        HAL_UART_Receive_IT( handle, &RxData, 1 );
    }
}

void UartMcuDeInit( Uart_t *obj )
{
    if( obj->UartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        UartUsbDeInit( obj );
#endif
    }
    else
    {    
        if( obj->UartId == UART_1 )
        {
            __HAL_RCC_USART1_FORCE_RESET( );
            __HAL_RCC_USART1_RELEASE_RESET( );
            __HAL_RCC_USART1_CLK_DISABLE( );

        }
        else if( obj->UartId == UART_2 )
        {
            __HAL_RCC_USART2_FORCE_RESET( );
            __HAL_RCC_USART2_RELEASE_RESET( );
            __HAL_RCC_USART2_CLK_DISABLE( );
        }

        GpioInit( &obj->Tx, obj->Tx.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &obj->Rx, obj->Rx.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    }
}

uint8_t UartMcuPutChar( Uart_t *obj, uint8_t data )
{
    if( obj->UartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        return UartUsbPutChar( obj, data );
#else
        return 255; // Not supported
#endif
    }
    else
    {
        UART_HandleTypeDef* handle;
        if ( obj->UartId == UART_1 )
        {
            handle = &Uart1Handle;
        }
        else if ( obj-> UartId == UART_2 )
        {
            handle = &Uart2Handle;
        }
        else
        {
            return 255;
        }

        CRITICAL_SECTION_BEGIN( );
        TxData = data;

        if( IsFifoFull( &obj->FifoTx ) == false )
        {
            FifoPush( &obj->FifoTx, TxData );

            
            // Trig UART Tx interrupt to start sending the FIFO contents.
            __HAL_UART_ENABLE_IT( handle, UART_IT_TC );

            CRITICAL_SECTION_END( );
            return 0; // OK
        }
        CRITICAL_SECTION_END( );
        return 1; // Busy
    }
}

uint8_t UartMcuGetChar( Uart_t *obj, uint8_t *data )
{
    if( obj->UartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        return UartUsbGetChar( obj, data );
#else
        return 255; // Not supported
#endif
    }
    else
    {
        CRITICAL_SECTION_BEGIN( );

        if( IsFifoEmpty( &obj->FifoRx ) == false )
        {
            *data = FifoPop( &obj->FifoRx );
            CRITICAL_SECTION_END( );
            return 0;
        }
        CRITICAL_SECTION_END( );
        return 1;
    }
}

uint8_t UartMcuPutBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size )
{
    if( obj->UartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        return UartUsbPutBuffer( obj, buffer, size );
#else
        return 255; // Not supported
#endif
    }
    else
    {
        uint8_t retryCount;
        uint16_t i;

        for( i = 0; i < size; i++ )
        {
            retryCount = 0;
            while( UartMcuPutChar( obj, buffer[i] ) != 0 )
            {
                retryCount++;

                // Exit if something goes terribly wrong
                if( retryCount > TX_BUFFER_RETRY_COUNT )
                {
                    return 1; // Error
                }
            }
        }
        return 0; // OK
    }
}

uint8_t UartMcuGetBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size, uint16_t *nbReadBytes )
{
    uint16_t localSize = 0;

    while( localSize < size )
    {
        if( UartGetChar( obj, buffer + localSize ) == 0 )
        {
            localSize++;
        }
        else
        {
            break;
        }
    }

    *nbReadBytes = localSize;

    if( localSize == 0 )
    {
        return 1; // Empty
    }
    return 0; // OK
}

static inline void UartHandleToInstance( UART_HandleTypeDef *handle, Uart_t** inst )
{
    if (handle->Instance == USART2)
    {
        *inst = &Uart2;
    }
    else if (handle->Instance == USART1)
    {
        *inst = &Uart1;
    }
    else
    {
        *inst = NULL;
    }
}

void HAL_UART_TxCpltCallback( UART_HandleTypeDef *handle )
{
    Uart_t* inst;
    UartHandleToInstance(handle, &inst);

    if (inst != NULL)
    {
        if( IsFifoEmpty( &inst->FifoTx ) == false )
        {
            TxData = FifoPop( &inst->FifoTx );
            //  Write one byte to the transmit data register
            HAL_UART_Transmit_IT( handle, &TxData, 1 );
        }

        if( inst->IrqNotify != NULL )
        {
            inst->IrqNotify( UART_NOTIFY_TX );
        }
    }
}

void HAL_UART_RxCpltCallback( UART_HandleTypeDef *handle )
{
    Uart_t* inst;
    UartHandleToInstance(handle, &inst);

    if( IsFifoFull( &inst->FifoRx ) == false )
    {
        // Read one byte from the receive data register
        FifoPush( &inst->FifoRx, RxData );
    }

    if( inst->IrqNotify != NULL )
    {
        inst->IrqNotify( UART_NOTIFY_RX );
    }

    HAL_UART_Receive_IT( handle, &RxData, 1 );
}

void HAL_UART_ErrorCallback( UART_HandleTypeDef *handle )
{
    HAL_UART_Receive_IT( handle, &RxData, 1 );
}

static inline void IRQHandler(UART_HandleTypeDef* handle)
{
    // [BEGIN] Workaround to solve an issue with the HAL drivers not managing the uart state correctly.
    uint32_t tmpFlag = 0, tmpItSource = 0;

    tmpFlag = __HAL_UART_GET_FLAG( handle, UART_FLAG_TC );
    tmpItSource = __HAL_UART_GET_IT_SOURCE( handle, UART_IT_TC );
    // UART in mode Transmitter end
    if( ( tmpFlag != RESET ) && ( tmpItSource != RESET ) )
    {
        if( ( handle->gState == HAL_UART_STATE_BUSY_RX ) || handle->gState == HAL_UART_STATE_BUSY_TX_RX )
        {
            handle->gState = HAL_UART_STATE_BUSY_TX_RX;
        }
    }
    // [END] Workaround to solve an issue with the HAL drivers not managing the uart state correctly.

    HAL_UART_IRQHandler( handle );
}

void USART1_IRQHandler( void )
{
    IRQHandler( &Uart1Handle );
}

void USART2_IRQHandler( void )
{
    IRQHandler( &Uart2Handle );
}

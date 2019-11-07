/*!
 * \file      board-config.h
 *
 * \brief     Board configuration
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
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 */
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#if defined( SX1262MBXDAS )
#define BOARD_TCXO_WAKEUP_TIME                      5
#else
#define BOARD_TCXO_WAKEUP_TIME                      0
#endif

/*!
 * Board MCU pins definitions
 */
#define RADIO_RESET                                 PC_0

#define RADIO_MOSI                                  PB_15
#define RADIO_MISO                                  PB_14
#define RADIO_SCLK                                  PB_13

#if defined( SX1261MBXBAS ) || defined( SX1262MBXCAS ) || defined( SX1262MBXDAS )

#define RADIO_NSS                                   PA_8
#define RADIO_BUSY                                  PB_3
#define RADIO_DIO_1                                 PB_4

#define RADIO_ANT_SWITCH_POWER                      PA_9
#define RADIO_FREQ_SEL                              PA_1
#define RADIO_XTAL_SEL                              PB_0
#define RADIO_DEVICE_SEL                            PA_4

#define LED_1                                       PC_1
#define LED_2                                       PC_0

// Debug pins definition.
#define RADIO_DBG_PIN_TX                            PB_6
#define RADIO_DBG_PIN_RX                            PC_7

#elif defined( SX1272MB2DAS) || defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )

#define RADIO_NSS                                   PB_12

#define RADIO_DIO_0                                 PB_5
#define RADIO_DIO_1                                 PB_6
#define RADIO_DIO_2                                 PB_7
#define RADIO_DIO_3                                 PB_8
#define RADIO_DIO_4                                 PB_9
#define RADIO_DIO_5                                 PB_10

#define RADIO_ANT_SWITCH                            NC

// #define LED_1                                       NC
// #define LED_2                                       NC
// #define LED_3                                       NC
// #define LED_4                                       NC

#define LED_1                                       PC_13   /* D2 */
#define LED_2                                       PA_0    /* D3 */
#define LED_3                                       PA_1    /* D5 */
#define LED_4                                       PA_11   /* D6 */

// Debug pins definition.
#define RADIO_DBG_PIN_TX                            PA_9
#define RADIO_DBG_PIN_RX                            PA_10

#endif

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

#define SWCLK                                       PA_14
#define SWDAT                                       PA_13

#define I2C_SCL                                     PA_8
#define I2C_SDA                                     PC_9

#define UART_TX                                     PA_2
#define UART_RX                                     PA_3

#endif // __BOARD_CONFIG_H__

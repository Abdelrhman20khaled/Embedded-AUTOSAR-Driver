/******************************************************************************
 *                                                                            *
 * Module: Port                                                               *
 *                                                                            *
 * File Name: Port_cfg.h (Auto Generated)                                     *
 *                                                                            *
 * Description: port configuration for TM4C123GH6PM Microcontroller           *
 *              Port Driver.                                                  *
 *                                                                            *
 * Author: Abdelrhmna Khaled                                                  *
 *                                                                            *
 ******************************************************************************/

#ifndef PORT_CFG_H_
#define PORT_CFG_H_

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/*
 * Switches the development error detection and notification on or off.
 • true: detection and notification is enabled.
 • false: detection and notification is disabled
 */
#define PORT_DEV_ERROR_DETECT               (STD_ON)

/*
 * Pre-processor switch to enable / disable the API to read out the modules version
 * information. true: Version info API enabled. false: Version info API disabled.
 *
 */
#define PORT_VERSION_INFO_API               (STD_ON)

/*
 * Pre-processor switch to enable / disable the use of the function Port_SetPinDirection().
 * TRUE:  Enabled  - Function Port_SetPinDirection() is available.
 * FALSE: Disabled - Function Port_SetPinDirection() is not available
 *
 */
#define PORT_SET_PIN_DIRECTION_API          (STD_ON)

/*
 * Pre-processor switch to enable / disable the use of the function Port_SetPinMode().
 * true:  Enabled  - Function Port_SetPinMode() is available.
 * false: Disabled - Function Port_SetPinMode() is not available.
 *
 */
#define PORT_SET_PIN_MODE_API               (STD_ON)

/*
 * Define number of pins were configured
 *
 */
#define PORT_CONFIGURED_CHANNLES            (43U)

/*
 * Define number of pins were configured
 *
 */
#define NUMBER_OF_MODES_IN_TIVAC            (15U)
/*******************************************************************************
 *                                                                             *
 *       >> TM4C123GH6PM I/O Ports Modes (From the Manual PG 1351) <<          *
 *                                                                             *
 *           Digital Function (GPIOPCTL PMCx Bit Field Encoding)               *
 *                                                                             *
 ******************************************************************************/
#define PIN_MODE_DIO                (0U)
#define PIN_MODE_UART               (1U)
#define PIN_MODE_SSI                (2U)
#define PIN_MODE_TWI                (3U)
#define PIN_MODE_M0PWM              (4U)
#define PIN_MODE_M0FAULT            (4U)
#define PIN_MODE_M1PWM              (5U)
#define PIN_MODE_M1FAULT            (5U)
#define PIN_MODE_IDX                (6U)
#define PIN_MODE_PHASE              (6U)
#define PIN_MODE_TIMER              (7U)
#define PIN_MODE_CAN                (8U)
#define PIN_MODE_USB                (8U)
#define PIN_MODE_NMI                (8U)
#define PIN_MODE_ANALOG_COMP        (9U)
#define PIN_MODE_ADC                (9U)
#define PIN_MODE_TRACE_CLOCK        (14U)
#define PIN_MODE_NO_ACTION          (15U)

/*******************************************************************************
 *                                                                             *
 *           >> TM4C123GH6PM I/O Ports Registers (From the Manual) <<          *
 *                                                                             *
 *  All Pins direction were configured as default as input pins  in manual     *
 *                                                                             *
 *  except the pins were  chose in AUTOSAR tool.                               *
 *                                                                             *
 ******************************************************************************/

/*PORT A*/
#define PortConfig_PA0_DIR  PORT_PIN_IN
#define PortConfig_PA1_DIR  PORT_PIN_IN
#define PortConfig_PA2_DIR  PORT_PIN_IN
#define PortConfig_PA3_DIR  PORT_PIN_IN
#define PortConfig_PA4_DIR  PORT_PIN_IN
#define PortConfig_PA5_DIR  PORT_PIN_IN
#define PortConfig_PA6_DIR  PORT_PIN_IN
#define PortConfig_PA7_DIR  PORT_PIN_IN

/*PORT B*/
#define PortConfig_PB0_DIR  PORT_PIN_IN
#define PortConfig_PB1_DIR  PORT_PIN_IN
#define PortConfig_PB2_DIR  PORT_PIN_IN
#define PortConfig_PB3_DIR  PORT_PIN_IN
#define PortConfig_PB4_DIR  PORT_PIN_IN
#define PortConfig_PB5_DIR  PORT_PIN_IN
#define PortConfig_PB6_DIR  PORT_PIN_IN
#define PortConfig_PB7_DIR  PORT_PIN_IN

/*PORT C*/
#define PortConfig_PC0_DIR  PORT_PIN_IN /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC1_DIR  PORT_PIN_IN /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC2_DIR  PORT_PIN_IN /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC3_DIR  PORT_PIN_IN /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC4_DIR  PORT_PIN_IN
#define PortConfig_PC5_DIR  PORT_PIN_IN
#define PortConfig_PC6_DIR  PORT_PIN_IN
#define PortConfig_PC7_DIR  PORT_PIN_IN

/*PORT D*/
#define PortConfig_PD0_DIR  PORT_PIN_IN
#define PortConfig_PD1_DIR  PORT_PIN_IN
#define PortConfig_PD2_DIR  PORT_PIN_IN
#define PortConfig_PD3_DIR  PORT_PIN_IN
#define PortConfig_PD4_DIR  PORT_PIN_IN
#define PortConfig_PD5_DIR  PORT_PIN_IN
#define PortConfig_PD6_DIR  PORT_PIN_IN
#define PortConfig_PD7_DIR  PORT_PIN_IN

/*PORT E*/
#define PortConfig_PE0_DIR  PORT_PIN_IN
#define PortConfig_PE1_DIR  PORT_PIN_IN
#define PortConfig_PE2_DIR  PORT_PIN_IN
#define PortConfig_PE3_DIR  PORT_PIN_IN
#define PortConfig_PE4_DIR  PORT_PIN_IN
#define PortConfig_PE5_DIR  PORT_PIN_IN

/*PORT F*/
#define PortConfig_PF0_DIR  PORT_PIN_IN
#define PortConfig_PF1_DIR  PORT_PIN_OUT /*Led Channel*/
#define PortConfig_PF2_DIR  PORT_PIN_IN
#define PortConfig_PF3_DIR  PORT_PIN_IN
#define PortConfig_PF4_DIR  PORT_PIN_IN /*Switch Channel*/

/*******************************************************************************
 *                                                                             *
 *           >> TM4C123GH6PM I/O Ports Registers (From the Manual) <<          *
 *                                                                             *
 *  All Pins Mode were configured as default as DIO pins in manual             *
 *                                                                             *
 *  except the pins were  chose in AUTOSAR tool.                               *
 *                                                                             *
 ******************************************************************************/
/*PORT A*/
#define PortConfig_PA0_MODE  PIN_MODE_DIO
#define PortConfig_PA1_MODE  PIN_MODE_DIO
#define PortConfig_PA2_MODE  PIN_MODE_DIO
#define PortConfig_PA3_MODE  PIN_MODE_DIO
#define PortConfig_PA4_MODE  PIN_MODE_DIO
#define PortConfig_PA5_MODE  PIN_MODE_DIO
#define PortConfig_PA6_MODE  PIN_MODE_DIO
#define PortConfig_PA7_MODE  PIN_MODE_DIO

/*PORT B*/
#define PortConfig_PB0_MODE  PIN_MODE_DIO
#define PortConfig_PB1_MODE  PIN_MODE_DIO
#define PortConfig_PB2_MODE  PIN_MODE_DIO
#define PortConfig_PB3_MODE  PIN_MODE_DIO
#define PortConfig_PB4_MODE  PIN_MODE_DIO
#define PortConfig_PB5_MODE  PIN_MODE_DIO
#define PortConfig_PB6_MODE  PIN_MODE_DIO
#define PortConfig_PB7_MODE  PIN_MODE_DIO

/*PORT C*/
#define PortConfig_PC0_MODE  PIN_MODE_DIO /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC1_MODE  PIN_MODE_DIO /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC2_MODE  PIN_MODE_DIO /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC3_MODE  PIN_MODE_DIO /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC4_MODE  PIN_MODE_DIO
#define PortConfig_PC5_MODE  PIN_MODE_DIO
#define PortConfig_PC6_MODE  PIN_MODE_DIO
#define PortConfig_PC7_MODE  PIN_MODE_DIO

/*PORT D*/
#define PortConfig_PD0_MODE  PIN_MODE_DIO
#define PortConfig_PD1_MODE  PIN_MODE_DIO
#define PortConfig_PD2_MODE  PIN_MODE_DIO
#define PortConfig_PD3_MODE  PIN_MODE_DIO
#define PortConfig_PD4_MODE  PIN_MODE_DIO
#define PortConfig_PD5_MODE  PIN_MODE_DIO
#define PortConfig_PD6_MODE  PIN_MODE_DIO
#define PortConfig_PD7_MODE  PIN_MODE_DIO

/*PORT E*/
#define PortConfig_PE0_MODE  PIN_MODE_DIO
#define PortConfig_PE1_MODE  PIN_MODE_DIO
#define PortConfig_PE2_MODE  PIN_MODE_DIO
#define PortConfig_PE3_MODE  PIN_MODE_DIO
#define PortConfig_PE4_MODE  PIN_MODE_DIO
#define PortConfig_PE5_MODE  PIN_MODE_DIO

/*PORT F*/
#define PortConfig_PF0_MODE  PIN_MODE_DIO
#define PortConfig_PF1_MODE  PIN_MODE_DIO /*Led Channel*/
#define PortConfig_PF2_MODE  PIN_MODE_DIO
#define PortConfig_PF3_MODE  PIN_MODE_DIO
#define PortConfig_PF4_MODE  PIN_MODE_DIO /*Switch Channel*/

/*******************************************************************************
 *                                                                             *
 *           >> TM4C123GH6PM I/O Ports Registers (From the Manual) <<          *
 *                                                                             *
 *  All Pins Internal Resistors  were configured as default as OFF in          *
 *                                                                             *
 *  manual except the pins were  chose in AUTOSAR tool.                        *
 *                                                                             *
 ******************************************************************************/

/*PORT A*/
#define PortConfig_PA0_INTERNAL_RES  OFF
#define PortConfig_PA1_INTERNAL_RES  OFF
#define PortConfig_PA2_INTERNAL_RES  OFF
#define PortConfig_PA3_INTERNAL_RES  OFF
#define PortConfig_PA4_INTERNAL_RES  OFF
#define PortConfig_PA5_INTERNAL_RES  OFF
#define PortConfig_PA6_INTERNAL_RES  OFF
#define PortConfig_PA7_INTERNAL_RES  OFF

/*PORT B*/
#define PortConfig_PB0_INTERNAL_RES  OFF
#define PortConfig_PB1_INTERNAL_RES  OFF
#define PortConfig_PB2_INTERNAL_RES  OFF
#define PortConfig_PB3_INTERNAL_RES  OFF
#define PortConfig_PB4_INTERNAL_RES  OFF
#define PortConfig_PB5_INTERNAL_RES  OFF
#define PortConfig_PB6_INTERNAL_RES  OFF
#define PortConfig_PB7_INTERNAL_RES  OFF

/*PORT C*/
#define PortConfig_PC0_INTERNAL_RES  OFF /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC1_INTERNAL_RES  OFF /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC2_INTERNAL_RES  OFF /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC3_INTERNAL_RES  OFF /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC4_INTERNAL_RES  OFF
#define PortConfig_PC5_INTERNAL_RES  OFF
#define PortConfig_PC6_INTERNAL_RES  OFF
#define PortConfig_PC7_INTERNAL_RES  OFF

/*PORT D*/
#define PortConfig_PD0_INTERNAL_RES  OFF
#define PortConfig_PD1_INTERNAL_RES  OFF
#define PortConfig_PD2_INTERNAL_RES  OFF
#define PortConfig_PD3_INTERNAL_RES  OFF
#define PortConfig_PD4_INTERNAL_RES  OFF
#define PortConfig_PD5_INTERNAL_RES  OFF
#define PortConfig_PD6_INTERNAL_RES  OFF
#define PortConfig_PD7_INTERNAL_RES  OFF

/*PORT E*/
#define PortConfig_PE0_INTERNAL_RES  OFF
#define PortConfig_PE1_INTERNAL_RES  OFF
#define PortConfig_PE2_INTERNAL_RES  OFF
#define PortConfig_PE3_INTERNAL_RES  OFF
#define PortConfig_PE4_INTERNAL_RES  OFF
#define PortConfig_PE5_INTERNAL_RES  OFF

/*PORT F*/
#define PortConfig_PF0_INTERNAL_RES  OFF
#define PortConfig_PF1_INTERNAL_RES  OFF /*Led Channel*/
#define PortConfig_PF2_INTERNAL_RES  OFF
#define PortConfig_PF3_INTERNAL_RES  OFF
#define PortConfig_PF4_INTERNAL_RES  PULL_UP /*Switch Channel*/

/*******************************************************************************
 *                                                                             *
 *           >> TM4C123GH6PM I/O Ports Registers (From the Manual) <<          *
 *                                                                             *
 *  All Pins ability to change direction were configured as default as         *
 *                                                                             *
 *  STD_OFF                                                                    *
 *                                                                             *
 ******************************************************************************/

/*PORT A*/
#define PortConfig_PA0_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PA1_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PA2_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PA3_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PA4_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PA5_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PA6_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PA7_CHANGEABLE_DIR  STD_OFF

/*PORT B*/
#define PortConfig_PB0_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PB1_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PB2_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PB3_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PB4_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PB5_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PB6_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PB7_CHANGEABLE_DIR  STD_OFF

/*PORT C*/
#define PortConfig_PC0_CHANGEABLE_DIR  STD_OFF /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC1_CHANGEABLE_DIR  STD_OFF /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC2_CHANGEABLE_DIR  STD_OFF /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC3_CHANGEABLE_DIR  STD_OFF /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC4_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PC5_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PC6_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PC7_CHANGEABLE_DIR  STD_OFF

/*PORT D*/
#define PortConfig_PD0_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PD1_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PD2_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PD3_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PD4_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PD5_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PD6_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PD7_CHANGEABLE_DIR  STD_OFF

/*PORT E*/
#define PortConfig_PE0_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PE1_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PE2_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PE3_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PE4_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PE5_CHANGEABLE_DIR  STD_OFF

/*PORT F*/
#define PortConfig_PF0_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PF1_CHANGEABLE_DIR  STD_OFF /*Led Channel*/
#define PortConfig_PF2_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PF3_CHANGEABLE_DIR  STD_OFF
#define PortConfig_PF4_CHANGEABLE_DIR  STD_OFF /*Switch Channel*/

/*******************************************************************************
 *                                                                             *
 *           >> TM4C123GH6PM I/O Ports Registers (From the Manual) <<          *
 *                                                                             *
 *  All Pins ability to change Mode were configured as default as              *
 *                                                                             *
 *  STD_OFF                                                                    *
 *                                                                             *
 ******************************************************************************/

/*PORT A*/
#define PortConfig_PA0_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PA1_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PA2_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PA3_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PA4_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PA5_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PA6_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PA7_CHANGEABLE_MODE   STD_OFF

/*PORT B*/
#define PortConfig_PB0_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PB1_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PB2_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PB3_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PB4_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PB5_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PB6_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PB7_CHANGEABLE_MODE   STD_OFF

/*PORT C*/
#define PortConfig_PC0_CHANGEABLE_MODE   STD_OFF /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC1_CHANGEABLE_MODE   STD_OFF /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC2_CHANGEABLE_MODE   STD_OFF /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC3_CHANGEABLE_MODE   STD_OFF /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC4_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PC5_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PC6_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PC7_CHANGEABLE_MODE   STD_OFF

/*PORT D*/
#define PortConfig_PD0_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PD1_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PD2_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PD3_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PD4_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PD5_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PD6_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PD7_CHANGEABLE_MODE   STD_OFF

/*PORT E*/
#define PortConfig_PE0_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PE1_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PE2_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PE3_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PE4_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PE5_CHANGEABLE_MODE   STD_OFF

/*PORT F*/
#define PortConfig_PF0_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PF1_CHANGEABLE_MODE   STD_OFF /*Led Channel*/
#define PortConfig_PF2_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PF3_CHANGEABLE_MODE   STD_OFF
#define PortConfig_PF4_CHANGEABLE_MODE   STD_OFF /*Switch Channel*/

/*******************************************************************************
 *                                                                             *
 *           >> TM4C123GH6PM I/O Ports Registers (From the Manual) <<          *
 *                                                                             *
 *  All Pins initial values were configured as default as STD_LOW in           *
 *                                                                             *
 *  manual except the pins were chose in AUTOSAR tool.                         *
 *                                                                             *
 ******************************************************************************/

/*PORT A*/
#define PortConfig_PA0_INITIAL_VALUE     STD_LOW
#define PortConfig_PA1_INITIAL_VALUE     STD_LOW
#define PortConfig_PA2_INITIAL_VALUE     STD_LOW
#define PortConfig_PA3_INITIAL_VALUE     STD_LOW
#define PortConfig_PA4_INITIAL_VALUE     STD_LOW
#define PortConfig_PA5_INITIAL_VALUE     STD_LOW
#define PortConfig_PA6_INITIAL_VALUE     STD_LOW
#define PortConfig_PA7_INITIAL_VALUE     STD_LOW

/*PORT B*/
#define PortConfig_PB0_INITIAL_VALUE     STD_LOW
#define PortConfig_PB1_INITIAL_VALUE     STD_LOW
#define PortConfig_PB2_INITIAL_VALUE     STD_LOW
#define PortConfig_PB3_INITIAL_VALUE     STD_LOW
#define PortConfig_PB4_INITIAL_VALUE     STD_LOW
#define PortConfig_PB5_INITIAL_VALUE     STD_LOW
#define PortConfig_PB6_INITIAL_VALUE     STD_LOW
#define PortConfig_PB7_INITIAL_VALUE     STD_LOW

/*PORT C*/
#define PortConfig_PC0_INITIAL_VALUE     STD_LOW /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC1_INITIAL_VALUE     STD_LOW /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC2_INITIAL_VALUE     STD_LOW /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC3_INITIAL_VALUE     STD_LOW /*No Action Needed . . .  JTAG Pins*/
#define PortConfig_PC4_INITIAL_VALUE     STD_LOW
#define PortConfig_PC5_INITIAL_VALUE     STD_LOW
#define PortConfig_PC6_INITIAL_VALUE     STD_LOW
#define PortConfig_PC7_INITIAL_VALUE     STD_LOW

/*PORT D*/
#define PortConfig_PD0_INITIAL_VALUE     STD_LOW
#define PortConfig_PD1_INITIAL_VALUE     STD_LOW
#define PortConfig_PD2_INITIAL_VALUE     STD_LOW
#define PortConfig_PD3_INITIAL_VALUE     STD_LOW
#define PortConfig_PD4_INITIAL_VALUE     STD_LOW
#define PortConfig_PD5_INITIAL_VALUE     STD_LOW
#define PortConfig_PD6_INITIAL_VALUE     STD_LOW
#define PortConfig_PD7_INITIAL_VALUE     STD_LOW

/*PORT E*/
#define PortConfig_PE0_INITIAL_VALUE     STD_LOW
#define PortConfig_PE1_INITIAL_VALUE     STD_LOW
#define PortConfig_PE2_INITIAL_VALUE     STD_LOW
#define PortConfig_PE3_INITIAL_VALUE     STD_LOW
#define PortConfig_PE4_INITIAL_VALUE     STD_LOW
#define PortConfig_PE5_INITIAL_VALUE     STD_LOW

/*PORT F*/
#define PortConfig_PF0_INITIAL_VALUE     STD_LOW
#define PortConfig_PF1_INITIAL_VALUE     STD_LOW  /*Led channel*/
#define PortConfig_PF2_INITIAL_VALUE     STD_LOW
#define PortConfig_PF3_INITIAL_VALUE     STD_LOW
#define PortConfig_PF4_INITIAL_VALUE     STD_HIGH /*Switch Channel*/

#endif /* PORT_CFG_H_ */

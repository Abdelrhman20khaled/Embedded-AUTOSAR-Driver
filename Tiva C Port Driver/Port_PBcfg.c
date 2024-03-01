/******************************************************************************
 *                                                                            *
 * Module: Port                                                               *
 *                                                                            *
 * File Name: Port_PBcfg.h                                                    *
 *                                                                            *
 * Description: Post Build Configuration Source file for TM4C123GH6PM         *
 *              Microcontroller - PORT Driver                                 *
 *                                                                            *
 * Author: Abdelrhmna Khaled                                                  *
 *                                                                            *
 ******************************************************************************/

#include "Port.h"

/*
 * Module Version 1.0.0
 */
#define PORT_PBCFG_SW_MAJOR_VERSION              (1U)
#define PORT_PBCFG_SW_MINOR_VERSION              (0U)
#define PORT_PBCFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_PBCFG_AR_RELEASE_MAJOR_VERSION      (4U)
#define PORT_PBCFG_AR_RELEASE_MINOR_VERSION      (0U)
#define PORT_PBCFG_AR_RELEASE_PATCH_VERSION      (3U)


/* AUTOSAR Version checking between PORT_PBcfg.c and PORT.h files */
#if ((PORT_PBCFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
        ||  (PORT_PBCFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
        ||  (PORT_PBCFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of PBcfg.c does not match the expected version"
#endif

/* Software Version checking between PORT_PBcfg.c and PORT.h files */
#if ((PORT_PBCFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
        ||  (PORT_PBCFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
        ||  (PORT_PBCFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
#error "The SW version of PBcfg.c does not match the expected version"
#endif

/*
 * The Port_Configuration consists of array of structures each index is a structure consists of:
 * 1) Port of Channels
 * 2) Channel Number
 * 3) Direction of Channel
 * 4) Channel Mode
 * 5) Internal Resistor
 * 6) Channel Changeable Direction
 * 7) Channel Changeable Mode
 * 8) Channel Initial Value
 * the port structure should initialize all channels in each port (43 channel) (auto generated)
 *
 */
const Port_ConfigType Port_Configuration = {
                                            /*Index 0*/
                                            PORTA_ID,PIN0_ID,
                                            PortConfig_PA0_DIR,
                                            PortConfig_PA0_MODE,
                                            PortConfig_PA0_INTERNAL_RES,
                                            PortConfig_PA0_CHANGEABLE_DIR,
                                            PortConfig_PA0_CHANGEABLE_MODE,
                                            PortConfig_PA0_INITIAL_VALUE,
                                            /*Index 1*/
                                            PORTA_ID,PIN1_ID,
                                            PortConfig_PA1_DIR,
                                            PortConfig_PA1_MODE,
                                            PortConfig_PA1_INTERNAL_RES,
                                            PortConfig_PA1_CHANGEABLE_DIR,
                                            PortConfig_PA1_CHANGEABLE_MODE,
                                            PortConfig_PA1_INITIAL_VALUE,
                                            /*Index 2*/
                                            PORTA_ID,PIN2_ID,
                                            PortConfig_PA2_DIR,
                                            PortConfig_PA2_MODE,
                                            PortConfig_PA2_INTERNAL_RES,
                                            PortConfig_PA2_CHANGEABLE_DIR,
                                            PortConfig_PA2_CHANGEABLE_MODE,
                                            PortConfig_PA2_INITIAL_VALUE,
                                            /*Index 3*/
                                            PORTA_ID,PIN3_ID,
                                            PortConfig_PA3_DIR,
                                            PortConfig_PA3_MODE,
                                            PortConfig_PA3_INTERNAL_RES,
                                            PortConfig_PA3_CHANGEABLE_DIR,
                                            PortConfig_PA3_CHANGEABLE_MODE,
                                            PortConfig_PA3_INITIAL_VALUE,
                                            /*Index 4*/
                                            PORTA_ID,PIN4_ID,
                                            PortConfig_PA4_DIR,
                                            PortConfig_PA4_MODE,
                                            PortConfig_PA4_INTERNAL_RES,
                                            PortConfig_PA4_CHANGEABLE_DIR,
                                            PortConfig_PA4_CHANGEABLE_MODE,
                                            PortConfig_PA4_INITIAL_VALUE,
                                            /*Index 5*/
                                            PORTA_ID,PIN5_ID,
                                            PortConfig_PA5_DIR,
                                            PortConfig_PA5_MODE,
                                            PortConfig_PA5_INTERNAL_RES,
                                            PortConfig_PA5_CHANGEABLE_DIR,
                                            PortConfig_PA5_CHANGEABLE_MODE,
                                            PortConfig_PA5_INITIAL_VALUE,
                                            /*Index 6*/
                                            PORTA_ID,PIN6_ID,
                                            PortConfig_PA6_DIR,
                                            PortConfig_PA6_MODE,
                                            PortConfig_PA6_INTERNAL_RES,
                                            PortConfig_PA6_CHANGEABLE_DIR,
                                            PortConfig_PA6_CHANGEABLE_MODE,
                                            PortConfig_PA6_INITIAL_VALUE,
                                            /*Index 7*/
                                            PORTA_ID,PIN7_ID,
                                            PortConfig_PA7_DIR,
                                            PortConfig_PA7_MODE,
                                            PortConfig_PA7_INTERNAL_RES,
                                            PortConfig_PA7_CHANGEABLE_DIR,
                                            PortConfig_PA7_CHANGEABLE_MODE,
                                            PortConfig_PA7_INITIAL_VALUE,

                                            /*Index 8*/
                                            PORTB_ID,PIN0_ID,
                                            PortConfig_PB0_DIR,
                                            PortConfig_PB0_MODE,
                                            PortConfig_PB0_INTERNAL_RES,
                                            PortConfig_PB0_CHANGEABLE_DIR,
                                            PortConfig_PB0_CHANGEABLE_MODE,
                                            PortConfig_PB0_INITIAL_VALUE,
                                            /*Index 9*/
                                            PORTB_ID,PIN1_ID,
                                            PortConfig_PB1_DIR,
                                            PortConfig_PB1_MODE,
                                            PortConfig_PB1_INTERNAL_RES,
                                            PortConfig_PB1_CHANGEABLE_DIR,
                                            PortConfig_PB1_CHANGEABLE_MODE,
                                            PortConfig_PB1_INITIAL_VALUE,
                                            /*Index 10*/
                                            PORTB_ID,PIN2_ID,
                                            PortConfig_PB2_DIR,
                                            PortConfig_PB2_MODE,
                                            PortConfig_PB2_INTERNAL_RES,
                                            PortConfig_PB2_CHANGEABLE_DIR,
                                            PortConfig_PB2_CHANGEABLE_MODE,
                                            PortConfig_PB2_INITIAL_VALUE,
                                            /*Index 11*/
                                            PORTB_ID,PIN3_ID,
                                            PortConfig_PB3_DIR,
                                            PortConfig_PB3_MODE,
                                            PortConfig_PB3_INTERNAL_RES,
                                            PortConfig_PB3_CHANGEABLE_DIR,
                                            PortConfig_PB3_CHANGEABLE_MODE,
                                            PortConfig_PB3_INITIAL_VALUE,
                                            /*Index 12*/
                                            PORTB_ID,PIN4_ID,
                                            PortConfig_PB4_DIR,
                                            PortConfig_PB4_MODE,
                                            PortConfig_PB4_INTERNAL_RES,
                                            PortConfig_PB4_CHANGEABLE_DIR,
                                            PortConfig_PB4_CHANGEABLE_MODE,
                                            PortConfig_PB4_INITIAL_VALUE,
                                            /*Index 13*/
                                            PORTB_ID,PIN5_ID,
                                            PortConfig_PB5_DIR,
                                            PortConfig_PB5_MODE,
                                            PortConfig_PB5_INTERNAL_RES,
                                            PortConfig_PB5_CHANGEABLE_DIR,
                                            PortConfig_PB5_CHANGEABLE_MODE,
                                            PortConfig_PB5_INITIAL_VALUE,
                                            /*Index 14*/
                                            PORTB_ID,PIN6_ID,
                                            PortConfig_PB6_DIR,
                                            PortConfig_PB6_MODE,
                                            PortConfig_PB6_INTERNAL_RES,
                                            PortConfig_PB6_CHANGEABLE_DIR,
                                            PortConfig_PB6_CHANGEABLE_MODE,
                                            PortConfig_PB6_INITIAL_VALUE,
                                            /*Index 15*/
                                            PORTB_ID,PIN7_ID,
                                            PortConfig_PB7_DIR,
                                            PortConfig_PB7_MODE,
                                            PortConfig_PB7_INTERNAL_RES,
                                            PortConfig_PB7_CHANGEABLE_DIR,
                                            PortConfig_PB7_CHANGEABLE_MODE,
                                            PortConfig_PB7_INITIAL_VALUE,

                                            /*Index 16*/
                                            PORTC_ID,PIN0_ID,
                                            PortConfig_PC0_DIR,
                                            PortConfig_PC0_MODE,
                                            PortConfig_PC0_INTERNAL_RES,
                                            PortConfig_PC0_CHANGEABLE_DIR,
                                            PortConfig_PC0_CHANGEABLE_MODE,
                                            PortConfig_PC0_INITIAL_VALUE,
                                            /*Index 17*/
                                            PORTC_ID,PIN1_ID,
                                            PortConfig_PC1_DIR,
                                            PortConfig_PC1_MODE,
                                            PortConfig_PC1_INTERNAL_RES,
                                            PortConfig_PC1_CHANGEABLE_DIR,
                                            PortConfig_PC1_CHANGEABLE_MODE,
                                            PortConfig_PC1_INITIAL_VALUE,
                                            /*Index 18*/
                                            PORTC_ID,PIN2_ID,
                                            PortConfig_PC2_DIR,
                                            PortConfig_PC2_MODE,
                                            PortConfig_PC2_INTERNAL_RES,
                                            PortConfig_PC2_CHANGEABLE_DIR,
                                            PortConfig_PC2_CHANGEABLE_MODE,
                                            PortConfig_PC2_INITIAL_VALUE,
                                            /*Index 19*/
                                            PORTC_ID,PIN3_ID,
                                            PortConfig_PC3_DIR,
                                            PortConfig_PC3_MODE,
                                            PortConfig_PC3_INTERNAL_RES,
                                            PortConfig_PC3_CHANGEABLE_DIR,
                                            PortConfig_PC3_CHANGEABLE_MODE,
                                            PortConfig_PC3_INITIAL_VALUE,
                                            /*Index 20*/
                                            PORTC_ID,PIN4_ID,
                                            PortConfig_PC4_DIR,
                                            PortConfig_PC4_MODE,
                                            PortConfig_PC4_INTERNAL_RES,
                                            PortConfig_PC4_CHANGEABLE_DIR,
                                            PortConfig_PC4_CHANGEABLE_MODE,
                                            PortConfig_PC4_INITIAL_VALUE,
                                            /*Index 21*/
                                            PORTC_ID,PIN5_ID,
                                            PortConfig_PC5_DIR,
                                            PortConfig_PC5_MODE,
                                            PortConfig_PC5_INTERNAL_RES,
                                            PortConfig_PC5_CHANGEABLE_DIR,
                                            PortConfig_PC5_CHANGEABLE_MODE,
                                            PortConfig_PC5_INITIAL_VALUE,
                                            /*Index 22*/
                                            PORTC_ID,PIN6_ID,
                                            PortConfig_PC6_DIR,
                                            PortConfig_PC6_MODE,
                                            PortConfig_PC6_INTERNAL_RES,
                                            PortConfig_PC6_CHANGEABLE_DIR,
                                            PortConfig_PC6_CHANGEABLE_MODE,
                                            PortConfig_PC6_INITIAL_VALUE,
                                            /*Index 23*/
                                            PORTC_ID,PIN7_ID,
                                            PortConfig_PC7_DIR,
                                            PortConfig_PC7_MODE,
                                            PortConfig_PC7_INTERNAL_RES,
                                            PortConfig_PC7_CHANGEABLE_DIR,
                                            PortConfig_PC7_CHANGEABLE_MODE,
                                            PortConfig_PC7_INITIAL_VALUE,

                                            /*Index 24*/
                                            PORTD_ID,PIN0_ID,
                                            PortConfig_PD0_DIR,
                                            PortConfig_PD0_MODE,
                                            PortConfig_PD0_INTERNAL_RES,
                                            PortConfig_PD0_CHANGEABLE_DIR,
                                            PortConfig_PD0_CHANGEABLE_MODE,
                                            PortConfig_PD0_INITIAL_VALUE,
                                            /*Index 25*/
                                            PORTD_ID,PIN1_ID,
                                            PortConfig_PD1_DIR,
                                            PortConfig_PD1_MODE,
                                            PortConfig_PD1_INTERNAL_RES,
                                            PortConfig_PD1_CHANGEABLE_DIR,
                                            PortConfig_PD1_CHANGEABLE_MODE,
                                            PortConfig_PD1_INITIAL_VALUE,
                                            /*Index 26*/
                                            PORTD_ID,PIN2_ID,
                                            PortConfig_PD2_DIR,
                                            PortConfig_PD2_MODE,
                                            PortConfig_PD2_INTERNAL_RES,
                                            PortConfig_PD2_CHANGEABLE_DIR,
                                            PortConfig_PD2_CHANGEABLE_MODE,
                                            PortConfig_PD2_INITIAL_VALUE,
                                            /*Index 27*/
                                            PORTD_ID,PIN3_ID,
                                            PortConfig_PD3_DIR,
                                            PortConfig_PD3_MODE,
                                            PortConfig_PD3_INTERNAL_RES,
                                            PortConfig_PD3_CHANGEABLE_DIR,
                                            PortConfig_PD3_CHANGEABLE_MODE,
                                            PortConfig_PD3_INITIAL_VALUE,
                                            /*Index 28*/
                                            PORTD_ID,PIN4_ID,
                                            PortConfig_PD4_DIR,
                                            PortConfig_PD4_MODE,
                                            PortConfig_PD4_INTERNAL_RES,
                                            PortConfig_PD4_CHANGEABLE_DIR,
                                            PortConfig_PD4_CHANGEABLE_MODE,
                                            PortConfig_PD4_INITIAL_VALUE,
                                            /*Index 29*/
                                            PORTD_ID,PIN5_ID,
                                            PortConfig_PD5_DIR,
                                            PortConfig_PD5_MODE,
                                            PortConfig_PD5_INTERNAL_RES,
                                            PortConfig_PD5_CHANGEABLE_DIR,
                                            PortConfig_PD5_CHANGEABLE_MODE,
                                            PortConfig_PD5_INITIAL_VALUE,
                                            /*Index 30*/
                                            PORTD_ID,PIN6_ID,
                                            PortConfig_PD6_DIR,
                                            PortConfig_PD6_MODE,
                                            PortConfig_PD6_INTERNAL_RES,
                                            PortConfig_PD6_CHANGEABLE_DIR,
                                            PortConfig_PD6_CHANGEABLE_MODE,
                                            PortConfig_PD6_INITIAL_VALUE,
                                            /*Index 31*/
                                            PORTD_ID,PIN7_ID,
                                            PortConfig_PD7_DIR,
                                            PortConfig_PD7_MODE,
                                            PortConfig_PD7_INTERNAL_RES,
                                            PortConfig_PD7_CHANGEABLE_DIR,
                                            PortConfig_PD7_CHANGEABLE_MODE,
                                            PortConfig_PD7_INITIAL_VALUE,

                                            /*Index 32*/
                                            PORTE_ID,PIN0_ID,
                                            PortConfig_PE0_DIR,
                                            PortConfig_PE0_MODE,
                                            PortConfig_PE0_INTERNAL_RES,
                                            PortConfig_PE0_CHANGEABLE_DIR,
                                            PortConfig_PE0_CHANGEABLE_MODE,
                                            PortConfig_PE0_INITIAL_VALUE,
                                            /*Index 33*/
                                            PORTE_ID,PIN1_ID,
                                            PortConfig_PE1_DIR,
                                            PortConfig_PE1_MODE,
                                            PortConfig_PE1_INTERNAL_RES,
                                            PortConfig_PE1_CHANGEABLE_DIR,
                                            PortConfig_PE1_CHANGEABLE_MODE,
                                            PortConfig_PE1_INITIAL_VALUE,
                                            /*Index 34*/
                                            PORTE_ID,PIN2_ID,
                                            PortConfig_PE2_DIR,
                                            PortConfig_PE2_MODE,
                                            PortConfig_PE2_INTERNAL_RES,
                                            PortConfig_PE2_CHANGEABLE_DIR,
                                            PortConfig_PE2_CHANGEABLE_MODE,
                                            PortConfig_PE2_INITIAL_VALUE,
                                            /*Index 35*/
                                            PORTE_ID,PIN3_ID,
                                            PortConfig_PE3_DIR,
                                            PortConfig_PE3_MODE,
                                            PortConfig_PE3_INTERNAL_RES,
                                            PortConfig_PE3_CHANGEABLE_DIR,
                                            PortConfig_PE3_CHANGEABLE_MODE,
                                            PortConfig_PE3_INITIAL_VALUE,
                                            /*Index 36*/
                                            PORTE_ID,PIN4_ID,
                                            PortConfig_PE4_DIR,
                                            PortConfig_PE4_MODE,
                                            PortConfig_PE4_INTERNAL_RES,
                                            PortConfig_PE4_CHANGEABLE_DIR,
                                            PortConfig_PE4_CHANGEABLE_MODE,
                                            PortConfig_PE4_INITIAL_VALUE,
                                            /*Index 37*/
                                            PORTE_ID,PIN5_ID,
                                            PortConfig_PE5_DIR,
                                            PortConfig_PE5_MODE,
                                            PortConfig_PE5_INTERNAL_RES,
                                            PortConfig_PE5_CHANGEABLE_DIR,
                                            PortConfig_PE5_CHANGEABLE_MODE,
                                            PortConfig_PE5_INITIAL_VALUE,

                                            /*Index 38*/
                                            PORTF_ID,PIN0_ID,
                                            PortConfig_PF0_DIR,
                                            PortConfig_PF0_MODE,
                                            PortConfig_PF0_INTERNAL_RES,
                                            PortConfig_PF0_CHANGEABLE_DIR,
                                            PortConfig_PF0_CHANGEABLE_MODE,
                                            PortConfig_PF0_INITIAL_VALUE,
                                            /*Index 39*/
                                            PORTF_ID,PIN1_ID,
                                            PortConfig_PF1_DIR,
                                            PortConfig_PF1_MODE,
                                            PortConfig_PF1_INTERNAL_RES,
                                            PortConfig_PF1_CHANGEABLE_DIR,
                                            PortConfig_PF1_CHANGEABLE_MODE,
                                            PortConfig_PF1_INITIAL_VALUE,
                                            /*Index 40*/
                                            PORTF_ID,PIN2_ID,
                                            PortConfig_PF2_DIR,
                                            PortConfig_PF2_MODE,
                                            PortConfig_PF2_INTERNAL_RES,
                                            PortConfig_PF2_CHANGEABLE_DIR,
                                            PortConfig_PF2_CHANGEABLE_MODE,
                                            PortConfig_PF2_INITIAL_VALUE,
                                            /*Index 41*/
                                            PORTF_ID,PIN3_ID,
                                            PortConfig_PF3_DIR,
                                            PortConfig_PF3_MODE,
                                            PortConfig_PF3_INTERNAL_RES,
                                            PortConfig_PF3_CHANGEABLE_DIR,
                                            PortConfig_PF3_CHANGEABLE_MODE,
                                            PortConfig_PF3_INITIAL_VALUE,
                                            /*Index 42*/
                                            PORTF_ID,PIN4_ID,
                                            PortConfig_PF4_DIR,
                                            PortConfig_PF4_MODE,
                                            PortConfig_PF4_INTERNAL_RES,
                                            PortConfig_PF4_CHANGEABLE_DIR,
                                            PortConfig_PF4_CHANGEABLE_MODE,
                                            PortConfig_PF4_INITIAL_VALUE,
};

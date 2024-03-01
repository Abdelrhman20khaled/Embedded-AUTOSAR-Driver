/******************************************************************************
 *                                                                            *
 * Module: Port                                                               *
 *                                                                            *
 * File Name: Port.h                                                          *
 *                                                                            *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.   *
 *                                                                            *
 * Author: Abdelrhmna Khaled                                                  *
 *                                                                            *
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

#include "Common_Macros.h"
/*******************************************************************************
 *                              Module Informations                            *
 *******************************************************************************/

/*Each Module Should has a Vendor ID and it is module and instance module*/
#define PORT_VENDOR_ID      (1000U)

#define PORT_MODULE_ID      (120U)

#define PORT_INSTANCE_ID    (0U)

/*The Version of the PORT driver */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for PORT Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

/*******************************************************************************
 *                             Check Module Matching                           *
 *******************************************************************************/
#include "Std_Types.h"
/*Check the matching between PORT AUIOSAR Version and STD_Types AUTOSAR Version */
#if((PORT_AR_RELEASE_MAJOR_VERSION != STD_TYPES_AR_RELEASE_MAJOR_VERSION)\
       ||(PORT_AR_RELEASE_MINOR_VERSION != STD_TYPES_AR_RELEASE_MINOR_VERSION)\
       ||(PORT_AR_RELEASE_PATCH_VERSION != STD_TYPES_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Std_Types.h does not match the expected version"

#endif

#include "Port_cfg.h"
#if((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
     || (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
     || (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Port_cfg.h does not match the expected version"
#endif

#if((PORT_SW_MAJOR_VERSION != PORT_CFG_SW_MAJOR_VERSION)\
    || (PORT_SW_MINOR_VERSION != PORT_CFG_SW_MINOR_VERSION)\
    || (PORT_SW_PATCH_VERSION != PORT_CFG_SW_PATCH_VERSION))
#error "The SW version of Port_cfg.h does not match the expected version"

#endif
/*******************************************************************************
 *                              Module Definitions                             *
 *******************************************************************************/

/* GPIO Registers base addresses */
#define GPIO_PORTA_BASE_ADDRESS           0x40004000
#define GPIO_PORTB_BASE_ADDRESS           0x40005000
#define GPIO_PORTC_BASE_ADDRESS           0x40006000
#define GPIO_PORTD_BASE_ADDRESS           0x40007000
#define GPIO_PORTE_BASE_ADDRESS           0x40024000
#define GPIO_PORTF_BASE_ADDRESS           0x40025000

/* GPIO Registers offset addresses */
#define PORT_DATA_REG_OFFSET              0x3FC
#define PORT_DIR_REG_OFFSET               0x400
#define PORT_ALT_FUNC_REG_OFFSET          0x420
#define PORT_PULL_UP_REG_OFFSET           0x510
#define PORT_PULL_DOWN_REG_OFFSET         0x514
#define PORT_DIGITAL_ENABLE_REG_OFFSET    0x51C
#define PORT_LOCK_REG_OFFSET              0x520
#define PORT_COMMIT_REG_OFFSET            0x524
#define PORT_ANALOG_MODE_SEL_REG_OFFSET   0x528
#define PORT_CTL_REG_OFFSET               0x52C

/*
 * Define Ports ID
 */
#define PORTA_ID    0
#define PORTB_ID    1
#define PORTC_ID    2
#define PORTD_ID    3
#define PORTE_ID    4
#define PORTF_ID    5

/*
 * Define Pins ID
 */
#define PIN0_ID     0
#define PIN1_ID     1
#define PIN2_ID     2
#define PIN3_ID     3
#define PIN4_ID     4
#define PIN5_ID     5
#define PIN6_ID     6
#define PIN7_ID     7

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/

#define PORT_INIT_SID               (uint8)0x00

#define Port_SET_PIN_DIRECTION_SID  (uint8)0x01

#define PORT_REFRESH_PORT_DIRECTION_SID (uint8)0x02

#if(PORT_VERSION_INFO_API == STD_ON)
#define PORT_GET_VERSION_INFO_SID   (uint8)0x03
#endif

/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/

#define PORT_E_PARAM_PIN                (uint8)0x0A

#define PORT_E_DIRECTION_UNCHANGEABLE   (uint8)0x0B

#define PORT_E_INIT_FAILED              (uint8)0x0C

#define PORT_E_PARAM_INVALID_MODE       (uint8)0x0D

#define PORT_E_MODE_UNCHANGEABLE        (uint8)0x0E

#define PORT_E_UNINIT                   (uint8)0x0F

#define PORT_E_PARAM_POINTER            (uint8)0x10

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/*create a data type of a pin type  from unsigned char type*/
typedef uint8 Port_PinType;

/*create a data type of a pin mode from unsigned char type*/
typedef uint8 Port_PinModeType;

/*
 * Parameter to indicate if the direction is changeable on a port pin during runtime:-
 * true (STD_ON):Port Pin direction changeable enabled.
 * false (STD_Off): Port Pin direction changeable disabled.
 *
 */
typedef uint8 Port_PinDirectionChangaeble;

/*
 * Parameter to indicate if the mode is changeable on a port pin during runtime:-
 * True (STD_ON) : Port Pin mode changeable allowed.
 * False (STD_OFF) : Port Pin mode changeable not permitted.
 *
 */
typedef uint8 Port_PinModeChangeable;

/* Description: Enum to hold PIN direction */
typedef enum
{
    PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirectionType;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
     OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;

/* Description: Structure to configure each individual PIN:
*	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
*	2. the number of the pin in the PORT.
*   3. the direction of pin --> INPUT or OUTPUT
*   4. the internal resistor --> Disable, Pull up or Pull down
* note that: each index will include 5 elements that all is information about the PIN
*/
typedef struct
{
    uint8 port_num;
    Port_PinType pin_num;
    Port_PinDirectionType direction;
    Port_PinModeType mode;
    Port_InternalResistor resistor;
    Port_PinDirectionChangaeble changeable_direction;
    Port_PinModeChangeable changeable_mode;
    uint8 initial_value;
}Port_ConfigChannel;

/*structure for all pins configured in port*/
typedef struct
{
    Port_ConfigChannel PORT_Channels[PORT_CONFIGURED_CHANNLES];
}Port_ConfigType;


/*******************************************************************************
 *                           Function Prototypes                               *
 *******************************************************************************/
/************************************************************************************
* Service Name: Port_Init                                                           *
* Sync/Async: Synchronous                                                           *
* Reentrancy: Non Reentrant                                                         *
* Parameters (in): ConfigPtr - Pointer to post-build configuration data             *
* Parameters (inout): None                                                          *
* Parameters (out): None                                                            *
* Return value: None                                                                *
* Description: Initializes the Port Driver module.                                  *
************************************************************************************/
void Port_Init(
        const Port_ConfigType *ConfigPtr
);

/************************************************************************************
* Service Name: Port_SetPinDirection                                                *
* Sync/Async: Synchronous                                                           *
* Reentrancy: Reentrant                                                             *
* Parameters (in): - Port Pin ID number                                             *
*                  - Port Pin Direction                                             *
* Parameters (inout): None                                                          *
* Parameters (out): None                                                            *
* Return value: None                                                                *
* Description: Sets the port pin direction                                          *
************************************************************************************/
void Port_SetPinDirection (
        Port_PinType Pin,
        Port_PinDirectionType Direction
);

/************************************************************************************
* Service Name: Port_RefreshPortDirection                                           *
* Sync/Async: Synchronous                                                           *
* Reentrancy: Non Reentrant                                                         *
* Parameters (in): None                                                             *
* Parameters (inout): None                                                          *
* Parameters (out): None                                                            *
* Return value: None                                                                *
* Description: Refreshes port direction.                                            *
************************************************************************************/
void Port_RefreshPortDirection (
       void
);

/************************************************************************************
* Service Name: Port_GetVersionInfo                                                 *
* Sync/Async: Synchronous                                                           *
* Reentrancy: Reentrant                                                             *
* Parameters (in): Pointer to where to store the version information of this module.*
* Parameters (inout): None                                                          *
* Parameters (out): None                                                            *
* Return value: None                                                                *
* Description: Returns the version information of this module.                      *
************************************************************************************/
void Port_GetVersionInfo (
        Std_VersionInfoType* versioninfo
);

/************************************************************************************
* Service Name: Port_SetPinMode                                                     *
* Sync/Async: Synchronous                                                           *
* Reentrancy: Reentrant                                                             *
* Parameters (in): - Port Pin ID number                                             *
*                  - New Port Pin mode to be set on port pin.                       *
* Parameters (inout): None                                                          *
* Parameters (out): None                                                            *
* Return value: None                                                                *
* Description: New Port Pin mode to be set on port pin.                             *
************************************************************************************/
void Port_SetPinMode (
       Port_PinType Pin,
       Port_PinModeType Mode
);


/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/
/* Extern PB structures to be used by PORT and other modules */
extern const Port_ConfigType Port_Configuration;

#endif /* PORT_H */

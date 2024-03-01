/*******************************************************************************
 *                                                                             *
 * Module: Port                                                                *
 *                                                                             *
 * File Name: Port.c                                                           *
 *                                                                             *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.    *
 *                                                                             *
 * Author:  Abdelrhman Khaled Sobhi                                            *
 *                                                                             *
 *******************************************************************************/

#include "Port.h"
#include "Port_Regs.h"

/*******************************************************************************
 *                             Check AUTOSAR Matching                          *
 *******************************************************************************/

/*First check if the DET Error activated or NOt */
#if(PORT_DEV_ERROR_DETECT == STD_ON)
#include "Det.h"
#if((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
        ||(DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
        ||(DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Det.h does not match the expected version"
#endif
#endif

/*Pointer to structure of configured channels*/
STATIC const Port_ConfigChannel * Port_Configuration_Channel = NULL_PTR;
/*Condition of the Port Module*/
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;

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
)
{
    /*Used For Pointing to the PORT*/
    volatile uint32 * PortGpio_Ptr = NULL_PTR;
    /*Local Variable for Checking Error*/
    uint8 error = FALSE;
    /*local variable used as an iteration count in loop on array of structures*/
    uint8 Channel_ID = 0;

    /*Check that DEV activated or not*/
#if(PORT_DEV_ERROR_DETECT == STD_ON)
    if(ConfigPtr == NULL_PTR)
    {
        Det_ReportError(
                PORT_MODULE_ID,
                PORT_INSTANCE_ID,
                PORT_INIT_SID,
                PORT_E_PARAM_POINTER);

        error =TRUE;
    }
    else
#endif
    {
        Port_Status = PORT_INITIALIZED;
        Port_Configuration_Channel = ConfigPtr->PORT_Channels;
        error =FALSE;
    }
    /*
     * Loop on array of structure to initialize each pin in each
     * index by the data passed in configuration structure.
     *
     */
    if(error == FALSE)
    {
        for(Channel_ID =0; Channel_ID < PORT_CONFIGURED_CHANNLES; Channel_ID++)
        {
            switch(Port_Configuration_Channel[Channel_ID].port_num)
            {
            case 0:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;
                break;
            case 1:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;
                break;
            case 2:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS;
                break;
            case 3:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS;
                break;
            case 4:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS;
                break;
            case 5:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS;
                break;
            }/*End of Switch Case*/

            /*
             * if pin is PD7 or PF0 , this pins used for NMI the lock and commit
             * should be set for this pins, any thing else you do not need to set
             * lock or commit.
             *
             */
            if( ((Port_Configuration_Channel[Channel_ID].port_num == PORTD_ID)
                    && (Port_Configuration_Channel[Channel_ID].pin_num == PIN7_ID))
                    || ((Port_Configuration_Channel[Channel_ID].port_num == PORTF_ID)
                            && (Port_Configuration_Channel[Channel_ID].pin_num == PIN0_ID)))
            {
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_Configuration_Channel[Channel_ID].port_num );
            }
            else
            {
                /*No Action Required*/
            }

            /*
             * the last check about specific pins is that check about JTAG pins
             * if the pin on of JATG pins (port c : pin 0 to pin 3) do no thing
             * on this pins, any pin else start the code and start the initialize.
             *
             */
            if( (Port_Configuration_Channel[Channel_ID].port_num == PORTC_ID)
                    && (Port_Configuration_Channel[Channel_ID].pin_num <= PIN3_ID))
            {
                /* Do Nothing ...  this is the JTAG pins */
            }
            else
            {
                /*This portion of code ensure that the pin was not one of JTAG pins*/
                /*
                 * 1) the Digital Enable >> enable
                 * 2) the Analog Mode >> disable
                 * 3) the Alternate Function and port control >> disable
                 * 4) At first clear the PMCX field on GPIOPCTRL register for the corresponding pin
                 */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) &= ~(0x0000000F<<(Port_Configuration_Channel[Channel_ID].pin_num*4));

                /*
                 * Check the direction of the pin is:
                 * 1) output pin then put the initial value passed in data structure to the pin.
                 * 2) input pin then check the pin use pull up or pull down resistor.
                 *
                 */
                if(Port_Configuration_Channel[Channel_ID].direction == PORT_PIN_OUT)
                {
                    /*Set the corresponding bit in the GPIO direction register*/
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET),Port_Configuration_Channel[Channel_ID].pin_num);

                    if(Port_Configuration_Channel[Channel_ID].initial_value == STD_HIGH)
                    {
                        /*Set the corresponding bit in the GPIO data register*/
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_Configuration_Channel[Channel_ID].pin_num);
                    }
                    else if(Port_Configuration_Channel[Channel_ID].initial_value == STD_LOW)
                    {
                        /*Clear the corresponding bit in the GPIO data register*/
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_Configuration_Channel[Channel_ID].pin_num);
                    }
                    else
                    {
                        /*No Action Required*/
                    }

                }
                else if(Port_Configuration_Channel[Channel_ID].direction == PORT_PIN_IN)
                {
                    /*Clear the corresponding bit in the GPIO direction register*/
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET),Port_Configuration_Channel[Channel_ID].pin_num);

                    if(Port_Configuration_Channel[Channel_ID].resistor == PULL_UP)
                    {
                        /*Set the corresponding bit in the GPIO pull up resistor register*/
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                    }
                    else if(Port_Configuration_Channel[Channel_ID].resistor == PULL_DOWN)
                    {
                        /*Set the corresponding bit in the GPIO pull down resistor register*/
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                    }
                    else if(Port_Configuration_Channel[Channel_ID].resistor == OFF)
                    {
                        /*
                         * Clear the both bits in both GPIO pull up and pull down registers,
                         * if neither pull up nor pull down resistors were chosen.
                         */
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                    }
                    else
                    {
                        /*No Action Required*/
                    }

                }
                else //end of if of pull up or pull down
                {
                    /*No Action Required*/
                }

                /*
                 * The corresponding pin in the specific port was initialized and then
                 * setup the mode for each pin
                 *
                 */
                if(Port_Configuration_Channel[Channel_ID].mode == PIN_MODE_DIO)
                {
                    /* Enable the Digital mode before define the pin as DIO pin */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                    /* Deactivate the alternative function */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                }
                else
                {
                    /* Enable the Digital mode before define the pin as DIO pin */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                    /* Anything except DIO, the alternate function should be activated */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                }


                /*
                 * set up the ADC mode individually because it need to set the analog mode
                 * and clear the digital mode.
                 *
                 */
                if(Port_Configuration_Channel[Channel_ID].mode == PIN_MODE_ADC)
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                }
                else
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                }

                /*
                 * Switch on the alternative function that defined in data structure
                 * and then set the value of the corresponding bits in GPIOPCTRL
                 * register.
                 *
                 * Note: this switch configure the register CTRL, configure the 4 pins so
                 * the common macros can not used here.
                 *
                 */
                switch(Port_Configuration_Channel[Channel_ID].mode)
                {

                case PIN_MODE_DIO:
                    /* DIO mode do not need to alternative function */
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) &= ~(0x0000000F<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    break;

                case PIN_MODE_UART:

                    /*
                     * Check if the pin of not pin 0 to pin pin 3 in port D because this pins
                     * used in SSI communication.
                     *
                     */
                    if( (Port_Configuration_Channel[Channel_ID].port_num == PORTD_ID)
                            &&(Port_Configuration_Channel[Channel_ID].pin_num <= PIN3_ID))
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32)PIN_MODE_SSI)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }
                    else
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32)PIN_MODE_UART)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }

                    break;

                case PIN_MODE_SSI:

                    if( (Port_Configuration_Channel[Channel_ID].port_num == PORTC_ID)
                            && ((Port_Configuration_Channel[Channel_ID].pin_num == PIN4_ID)
                                    || (Port_Configuration_Channel[Channel_ID].pin_num == PIN5_ID)))
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32)PIN_MODE_UART)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }
                    else
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32)PIN_MODE_SSI)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }

                    break;

                case PIN_MODE_TWI:
                    /*
                     * Check if the pin not pin 0 and pin 3 in port F because they used for CAN
                     * for avoiding any conflicting.
                     *
                     */
                    if( (Port_Configuration_Channel[Channel_ID].port_num == PORTF_ID)
                            && ((Port_Configuration_Channel[Channel_ID].pin_num == PIN0_ID)
                                    || (Port_Configuration_Channel[Channel_ID].pin_num == PIN3_ID)))
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_CAN)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }
                    else
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_TWI)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }

                    break;

                case PIN_MODE_M0PWM:
                    /*
                     * Check if the pin not pin 2 and pin 6 in port D and not pin 2 in port F
                     * because they pins used for M0FUALT not for M0PWM, for avoiding any conflicting.
                     *
                     */
                    if( ((Port_Configuration_Channel[Channel_ID].port_num == PORTD_ID)
                            && ((Port_Configuration_Channel[Channel_ID].pin_num == PIN2_ID)
                                    || (Port_Configuration_Channel[Channel_ID].pin_num == PIN6_ID)))
                            || ((Port_Configuration_Channel[Channel_ID].port_num == PORTF_ID)
                                    && (Port_Configuration_Channel[Channel_ID].pin_num == PIN2_ID)))
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_M0FAULT)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }
                    else
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_M0PWM)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }
                    break;

                case PIN_MODE_M1PWM:
                    /*
                     * Check if the pin not pin 4 port F because this pins used for
                     *  M0FUALT not for M0PWM, for avoiding any conflicting.
                     *
                     */
                    if( (Port_Configuration_Channel[Channel_ID].port_num == PORTF_ID)
                            && (Port_Configuration_Channel[Channel_ID].pin_num == PIN4_ID))
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_M1FAULT)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }
                    else
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_M1PWM)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }
                    break;

                case PIN_MODE_PHASE:
                    /*
                     * Check that pin not pin 4 in port C and not pin 3 in port D and not
                     * pin 4 in port F, because this pins used for IDX mode not Phase mode.
                     *
                     */
                    if( ((Port_Configuration_Channel[Channel_ID].port_num == PORTC_ID)
                            && (Port_Configuration_Channel[Channel_ID].pin_num == PIN4_ID))
                            || ((Port_Configuration_Channel[Channel_ID].port_num == PORTD_ID)
                                    && (Port_Configuration_Channel[Channel_ID].pin_num == PIN3_ID))
                                    || (Port_Configuration_Channel[Channel_ID].port_num == PORTF_ID)
                                    && (Port_Configuration_Channel[Channel_ID].pin_num == PIN4_ID))
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_IDX)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }
                    else
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_PHASE)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }
                    break;

                case PIN_MODE_TIMER:
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_TIMER)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    break;

                case PIN_MODE_CAN:

                    /*
                     * The condition for checking that if :
                     * 1) the pin 4 and pin 5 in port C those pins used for UART.
                     * 2) the pin 6 and pin 7 in port C and pin 2 and pin 3 in port D and pin 4 in port F those pins used in
                     *    USB communication.
                     * 3) the pin 7 in port D and pin 0 in port F those pins used in NON MASKABLE INTERRUPT.
                     * 4) anything except the 3 points mentioned is used in CAN communication.
                     */
                    if((Port_Configuration_Channel[Channel_ID].port_num == PORTC_ID)
                            && ((Port_Configuration_Channel[Channel_ID].pin_num == PIN4_ID)
                                    || (Port_Configuration_Channel[Channel_ID].pin_num == PIN5_ID)))
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_UART)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }
                    else if( ((Port_Configuration_Channel[Channel_ID].port_num == PORTC_ID)
                            &&((Port_Configuration_Channel[Channel_ID].pin_num == PIN6_ID)
                                    ||(Port_Configuration_Channel[Channel_ID].pin_num == PIN7_ID)))
                            ||((Port_Configuration_Channel[Channel_ID].port_num == PORTD_ID)
                                    &&((Port_Configuration_Channel[Channel_ID].pin_num == PIN2_ID)
                                            || (Port_Configuration_Channel[Channel_ID].pin_num == PIN3_ID)))
                                            ||((Port_Configuration_Channel[Channel_ID].port_num == PORTF_ID)
                                                    && (Port_Configuration_Channel[Channel_ID].pin_num == PIN4_ID)))
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_USB)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }
                    else if( ((Port_Configuration_Channel[Channel_ID].port_num == PORTD_ID)
                            && (Port_Configuration_Channel[Channel_ID].pin_num == PIN7_ID))
                            || ((Port_Configuration_Channel[Channel_ID].port_num == PORTF_ID)
                                    && (Port_Configuration_Channel[Channel_ID].pin_num == PIN0_ID)))
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_NMI)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }
                    else
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_CAN)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }

                    break;

                case PIN_MODE_ADC:

                    if( (Port_Configuration_Channel[Channel_ID].port_num == PORTF_ID)
                            && ((Port_Configuration_Channel[Channel_ID].pin_num == PIN0_ID)
                                    ||   (Port_Configuration_Channel[Channel_ID].pin_num == PIN1_ID)))
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_ANALOG_COMP)<<((Port_Configuration_Channel[Channel_ID].pin_num)*4));
                    }
                    else
                    {
                        /*This is ADC mode and it was configured before*/
                    }

                    break;

                case PIN_MODE_NO_ACTION:

                    /*No Action Required*/

                    break;

                default:

                    /*
                     * the default case report a DET error that indicates
                     * the mode entered is invalid mode.
                     *
                     */
                    if (PORT_DEV_ERROR_DETECT == STD_ON){
                        Det_ReportError(
                                PORT_MODULE_ID,
                                PORT_INSTANCE_ID,
                                PORT_INIT_SID,
                                PORT_E_PARAM_INVALID_MODE);
                    }
                    else
                    {
                        /* No Action Required */
                    }
                    break;

                }/*End of Switch Case*/
            }/*End of condition that check the pin not JTAG Pin*/


        }/*End of For Loop on Array of configured Structures*/
    }/*End of If Condition*/
    else
    {
        /* No Action Required */
    }
}

/*************************************************************************************
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
#if(PORT_SET_PIN_DIRECTION_API == STD_ON)

void Port_SetPinDirection (
        Port_PinType Pin,
        Port_PinDirectionType Direction
)
{
    volatile uint32 * PortGpio_Ptr = NULL_PTR;
    boolean error = FALSE;

#if(PORT_DEV_ERROR_DETECT == STD_ON)

    if(Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(
                PORT_MODULE_ID,
                PORT_INSTANCE_ID,
                Port_SET_PIN_DIRECTION_SID,
                PORT_E_UNINIT);

        error = TRUE;
    }
    else
    {
        error = FALSE;
    }

    if( Pin >=  PORT_CONFIGURED_CHANNLES )
    {
        Det_ReportError(
                PORT_MODULE_ID,
                PORT_INSTANCE_ID,
                Port_SET_PIN_DIRECTION_SID,
                PORT_E_PARAM_PIN);

        error = TRUE;
    }
    else
    {
        error = FALSE;
    }

    if(Port_Configuration_Channel[Pin].changeable_direction == STD_OFF)
    {
        Det_ReportError(
                PORT_MODULE_ID,
                PORT_INSTANCE_ID,
                Port_SET_PIN_DIRECTION_SID,
                PORT_E_DIRECTION_UNCHANGEABLE
        );

        error = TRUE;
    }
    else
    {
        error = FALSE;
    }

#endif

    if(error == FALSE)
    {
        switch(Port_Configuration_Channel[Pin].port_num)
        {
        case 0:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;
            break;
        case 1:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;
            break;
        case 2:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS;
            break;
        case 3:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS;
            break;
        case 4:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS;
            break;
        case 5:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS;
            break;
        }/*End of Switch Case*/

        /*
         * if pin is PD7 or PF0 , this pins used for NMI the lock and commit
         * should be set for this pins, any thing else you do not need to set
         * lock or commit.
         *
         */
        if( ((Port_Configuration_Channel[Pin].port_num == PORTD_ID)
                && (Port_Configuration_Channel[Pin].pin_num == PIN7_ID))
                || ((Port_Configuration_Channel[Pin].port_num == PORTF_ID)
                        && (Port_Configuration_Channel[Pin].pin_num == PIN0_ID)))
        {
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_Configuration_Channel[Pin].port_num );
        }
        else
        {
            /*No Action Required*/
        }

        /*
         * the last check about specific pins is that check about JTAG pins
         * if the pin on of JATG pins (port c : pin 0 to pin 3) do no thing
         * on this pins, any pin else start the code and start the initialize.
         *
         */
        if( (Port_Configuration_Channel[Pin].port_num == PORTC_ID)
                && (Port_Configuration_Channel[Pin].pin_num <= PIN3_ID))
        {
            /* Do Nothing ...  this is the JTAG pins */
        }
        else
        {
            /*
             * Check the direction of the pin is:
             * 1) output pin then put the initial value passed in data structure to the pin.
             * 2) input pin then check the pin use pull up or pull down resistor.
             *
             */
            if(Port_Configuration_Channel[Pin].direction == PORT_PIN_OUT)
            {
                /*Set the corresponding bit in the GPIO direction register*/
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET),Port_Configuration_Channel[Pin].pin_num);

                if(Port_Configuration_Channel[Pin].initial_value == STD_HIGH)
                {
                    /*Set the corresponding bit in the GPIO data register*/
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_Configuration_Channel[Pin].pin_num);
                }
                else if(Port_Configuration_Channel[Pin].initial_value == STD_LOW)
                {
                    /*Clear the corresponding bit in the GPIO data register*/
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_Configuration_Channel[Pin].pin_num);
                }
                else
                {
                    /*No Action Required*/
                }

            }
            else if(Port_Configuration_Channel[Pin].direction == PORT_PIN_IN)
            {
                /*Clear the corresponding bit in the GPIO direction register*/
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET),Port_Configuration_Channel[Pin].pin_num);

                if(Port_Configuration_Channel[Pin].resistor == PULL_UP)
                {
                    /*Set the corresponding bit in the GPIO pull up resistor register*/
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET ),Port_Configuration_Channel[Pin].pin_num);
                }
                else if(Port_Configuration_Channel[Pin].resistor == PULL_DOWN)
                {
                    /*Set the corresponding bit in the GPIO pull down resistor register*/
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET ),Port_Configuration_Channel[Pin].pin_num);
                }
                else if(Port_Configuration_Channel[Pin].resistor == OFF)
                {
                    /*
                     * Clear the both bits in both GPIO pull up and pull down registers,
                     * if neither pull up nor pull down resistors were chosen.
                     */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET ),Port_Configuration_Channel[Pin].pin_num);
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET ),Port_Configuration_Channel[Pin].pin_num);
                }
                else
                {
                    /*No Action Required*/
                }

            }
            else //end of if of pull up or pull down
            {
                /*No Action Required*/
            }
        }
    }
}

#endif

/*************************************************************************************
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
)
{
    volatile uint32 * PortGpio_Ptr = NULL_PTR;
    boolean error = FALSE;
    uint8 Channel_ID = 0;

#if(PORT_DEV_ERROR_DETECT == STD_ON)

    if( Port_Status == PORT_NOT_INITIALIZED )
    {
        Det_ReportError(
                PORT_MODULE_ID,
                PORT_INSTANCE_ID,
                PORT_REFRESH_PORT_DIRECTION_SID,
                PORT_E_UNINIT);

        error = TRUE;
    }
    else
    {
        error = FALSE;
    }

#endif
    /*
     * Check first that no error happen and then loop on all channels to refresh the
     * direction.
     *
     */
    if( error == FALSE )
    {
        for(Channel_ID = 0; Channel_ID <= PORT_CONFIGURED_CHANNLES; Channel_ID++)
        {
            switch(Port_Configuration_Channel[Channel_ID].port_num)
            {
            case 0:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;
                break;
            case 1:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;
                break;
            case 2:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS;
                break;
            case 3:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS;
                break;
            case 4:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS;
                break;
            case 5:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS;
                break;
            }/*End of Switch Case*/

            if( ((Port_Configuration_Channel[Channel_ID].port_num == PORTD_ID)
                    && (Port_Configuration_Channel[Channel_ID].pin_num == PIN7_ID))
                    || ((Port_Configuration_Channel[Channel_ID].port_num == PORTF_ID)
                            && (Port_Configuration_Channel[Channel_ID].pin_num == PIN0_ID)))
            {
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_Configuration_Channel[Channel_ID].pin_num);
            }
            else if( (Port_Configuration_Channel[Channel_ID].port_num == PORTC_ID)
                    && (Port_Configuration_Channel[Channel_ID].pin_num <= PIN3_ID))
            {
                /* Do Nothing ...  this is the JTAG pins */
            }
            else
            {
                /*
                 * check if this channel supports ability for changing direction do not
                 * refresh the direction of this pin, change all pins except the pins
                 * support ability to change.
                 *
                 */
                if(Port_Configuration_Channel[Channel_ID].changeable_direction == STD_ON)
                {

                    /* No Action Required, this pins should be excluded from refreshing. */

                }
                else
                {
                    if(Port_Configuration_Channel[Channel_ID].direction == PORT_PIN_OUT)
                    {
                        /*Set the corresponding bit in the GPIO direction register*/
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET),Port_Configuration_Channel[Channel_ID].pin_num);

                        if(Port_Configuration_Channel[Channel_ID].initial_value == STD_HIGH)
                        {
                            /*Set the corresponding bit in the GPIO data register*/
                            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_Configuration_Channel[Channel_ID].pin_num);
                        }
                        else if(Port_Configuration_Channel[Channel_ID].initial_value == STD_LOW)
                        {
                            /*Clear the corresponding bit in the GPIO data register*/
                            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_Configuration_Channel[Channel_ID].pin_num);
                        }
                        else
                        {
                            /*No Action Required*/
                        }
                    }
                    else if(Port_Configuration_Channel[Channel_ID].direction == PORT_PIN_IN)
                    {
                        /*Clear the corresponding bit in the GPIO direction register*/
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET),Port_Configuration_Channel[Channel_ID].pin_num);

                        if(Port_Configuration_Channel[Channel_ID].resistor == PULL_UP)
                        {
                            /*Set the corresponding bit in the GPIO pull up resistor register*/
                            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                        }
                        else if(Port_Configuration_Channel[Channel_ID].resistor == PULL_DOWN)
                        {
                            /*Set the corresponding bit in the GPIO pull down resistor register*/
                            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                        }
                        else if(Port_Configuration_Channel[Channel_ID].resistor == OFF)
                        {
                            /*
                             * Clear the both bits in both GPIO pull up and pull down registers,
                             * if neither pull up nor pull down resistors were chosen.
                             */
                            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET ),Port_Configuration_Channel[Channel_ID].pin_num);
                        }
                        else
                        {
                            /*No Action Required*/
                        }

                    }
                    else //end of if of pull up or pull down
                    {
                        /*No Action Required*/
                    }

                }/*End of condition that check if pin able to change direction during run time or not*/
            }/*End of check that the pin not one of JTAG pins*/
        }/*End of for loop*/
    }/* End of if condition that check on no error */
    else
    {
        /* No Action Required */
    }
}

/*************************************************************************************
 * Service Name: Port_GetVersionInfo                                                 *
 * Sync/Async: Synchronous                                                           *
 * Reentrancy: Reentrant                                                             *
 * Parameters (in): Pointer to where to store the version information of this module.*
 * Parameters (inout): None                                                          *
 * Parameters (out): None                                                            *
 * Return value: None                                                                *
 * Description: Returns the version information of this module.                      *
 ************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo (
        Std_VersionInfoType* versioninfo
)
{

#if(PORT_DEV_ERROR_DETECT == STD_ON)

    if(versioninfo == NULL_PTR)
    {
        Det_ReportError(
                PORT_MODULE_ID,
                PORT_INSTANCE_ID,
                PORT_GET_VERSION_INFO_SID,
                PORT_E_PARAM_POINTER
        );
    }
    else
    {
        /* No Action Required */
    }

    if(Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(
                PORT_MODULE_ID,
                PORT_INSTANCE_ID,
                PORT_GET_VERSION_INFO_SID,
                PORT_E_UNINIT
        );
    }
    else
    {
        /* No Action Required */
    }

#endif
    /*return Vendor ID*/
    versioninfo -> vendorID         = (uint16) PORT_VENDOR_ID;
    /*return Module ID*/
    versioninfo -> moduleID         = (uint16) PORT_MODULE_ID;
    /*return Software Major Version*/
    versioninfo -> sw_major_version = (uint8)  PORT_SW_MAJOR_VERSION;
    /*return Software Minor Version*/
    versioninfo -> sw_minor_version = (uint8)  PORT_SW_MINOR_VERSION;
    /*return Software Patch Version*/
    versioninfo -> sw_patch_version = (uint8)  PORT_SW_PATCH_VERSION;

}
#endif

/*************************************************************************************
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

#if(PORT_SET_PIN_MODE_API == STD_ON)

void Port_SetPinMode (
        Port_PinType Pin,
        Port_PinModeType Mode
)
{
    /*Used For Pointing to the PORT*/
    volatile uint32 * PortGpio_Ptr = NULL_PTR;

    /*Local Variable for Checking Error*/
    uint8 error = FALSE;

#if(PORT_DEV_ERROR_DETECT == STD_ON)

    if(Port_Configuration_Channel[Pin].changeable_mode == STD_OFF)
    {
        Det_ReportError(
                PORT_MODULE_ID,
                PORT_INSTANCE_ID,
                PORT_SET_PIN_MODE_API,
                PORT_E_MODE_UNCHANGEABLE
        );

        error = TRUE;

    }
    else
    {
        /* No Action Required*/
    }

    if(Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(
                PORT_MODULE_ID,
                PORT_INSTANCE_ID,
                PORT_SET_PIN_MODE_API,
                PORT_E_UNINIT
        );

        error = TRUE;

    }
    else
    {
        /* No Action Required */
    }

    if( Pin >= PORT_CONFIGURED_CHANNLES )
    {
        Det_ReportError(
                PORT_MODULE_ID,
                PORT_INSTANCE_ID,
                PORT_SET_PIN_MODE_API,
                PORT_E_PARAM_PIN
        );

        error = TRUE;

    }
    else
    {
        /* No Action Required */
    }

    if( Mode > NUMBER_OF_MODES_IN_TIVAC )
    {
        Det_ReportError(
                PORT_MODULE_ID,
                PORT_INSTANCE_ID,
                PORT_SET_PIN_MODE_API,
                PORT_E_PARAM_INVALID_MODE
        );

        error = TRUE;

    }
    else
    {
        /* No Action Required */
    }

#endif

    if(error == FALSE)
    {
        switch(Port_Configuration_Channel[Pin].port_num)
        {
        case 0:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;
            break;
        case 1:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;
            break;
        case 2:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS;
            break;
        case 3:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS;
            break;
        case 4:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS;
            break;
        case 5:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS;
            break;
        }/*End of Switch Case*/

        /*
         * if pin is PD7 or PF0 , this pins used for NMI the lock and commit
         * should be set for this pins, any thing else you do not need to set
         * lock or commit.
         *
         */
        if( ((Port_Configuration_Channel[Pin].port_num == PORTD_ID)
                && (Port_Configuration_Channel[Pin].pin_num == PIN7_ID))
                || ((Port_Configuration_Channel[Pin].port_num == PORTF_ID)
                        && (Port_Configuration_Channel[Pin].pin_num == PIN0_ID)))
        {
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_Configuration_Channel[Pin].port_num );
        }
        else
        {
            /*No Action Required*/
        }

        /*
         * the last check about specific pins is that check about JTAG pins
         * if the pin on of JATG pins (port c : pin 0 to pin 3) do no thing
         * on this pins, any pin else start the code and start the initialize.
         *
         */
        if( (Port_Configuration_Channel[Pin].port_num == PORTC_ID)
                && (Port_Configuration_Channel[Pin].pin_num <= PIN3_ID))
        {
            /* Do Nothing ...  this is the JTAG pins */
        }
        else
        {

            /*
             * The corresponding pin in the specific port was initialized and then
             * setup the mode for each pin
             *
             */
            if(Port_Configuration_Channel[Pin].mode == PIN_MODE_DIO)
            {
                /* Enable the Digital mode before define the pin as DIO pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET ),Port_Configuration_Channel[Pin].pin_num);
                /* Deactivate the alternative function */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET ),Port_Configuration_Channel[Pin].pin_num);
            }
            else
            {
                /* Enable the Digital mode before define the pin as DIO pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET ),Port_Configuration_Channel[Pin].pin_num);
                /* Anything except DIO, the alternate function should be activated */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET ),Port_Configuration_Channel[Pin].pin_num);
            }


            /*
             * set up the ADC mode individually because it need to set the analog mode
             * and clear the digital mode.
             *
             */
            if(Port_Configuration_Channel[Pin].mode == PIN_MODE_ADC)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET ),Port_Configuration_Channel[Pin].pin_num);
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET ),Port_Configuration_Channel[Pin].pin_num);
            }
            else
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET ),Port_Configuration_Channel[Pin].pin_num);
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET ),Port_Configuration_Channel[Pin].pin_num);
            }

            /*
             * Switch on the alternative function that defined in data structure
             * and then set the value of the corresponding bits in GPIOPCTRL
             * register.
             *
             * Note: this switch configure the register CTRL, configure the 4 pins so
             * the common macros can not used here.
             *
             */
            switch(Port_Configuration_Channel[Pin].mode)
            {

            case PIN_MODE_DIO:
                /* DIO mode do not need to alternative function */
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) &= ~(0x0000000F<<((Port_Configuration_Channel[Pin].pin_num)*4));
                break;

            case PIN_MODE_UART:

                /*
                 * Check if the pin of not pin 0 to pin pin 3 in port D because this pins
                 * used in SSI communication.
                 *
                 */
                if( (Port_Configuration_Channel[Pin].port_num == PORTD_ID)
                        &&(Port_Configuration_Channel[Pin].pin_num <= PIN3_ID))
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32)PIN_MODE_SSI)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }
                else
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32)PIN_MODE_UART)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }

                break;

            case PIN_MODE_SSI:

                if( (Port_Configuration_Channel[Pin].port_num == PORTC_ID)
                        && ((Port_Configuration_Channel[Pin].pin_num == PIN4_ID)
                                || (Port_Configuration_Channel[Pin].pin_num == PIN5_ID)))
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32)PIN_MODE_UART)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }
                else
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32)PIN_MODE_SSI)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }

                break;

            case PIN_MODE_TWI:
                /*
                 * Check if the pin not pin 0 and pin 3 in port F because they used for CAN
                 * for avoiding any conflicting.
                 *
                 */
                if( (Port_Configuration_Channel[Pin].port_num == PORTF_ID)
                        && ((Port_Configuration_Channel[Pin].pin_num == PIN0_ID)
                                || (Port_Configuration_Channel[Pin].pin_num == PIN3_ID)))
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_CAN)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }
                else
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_TWI)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }

                break;

            case PIN_MODE_M0PWM:
                /*
                 * Check if the pin not pin 2 and pin 6 in port D and not pin 2 in port F
                 * because they pins used for M0FUALT not for M0PWM, for avoiding any conflicting.
                 *
                 */
                if( ((Port_Configuration_Channel[Pin].port_num == PORTD_ID)
                        && ((Port_Configuration_Channel[Pin].pin_num == PIN2_ID)
                                || (Port_Configuration_Channel[Pin].pin_num == PIN6_ID)))
                        || ((Port_Configuration_Channel[Pin].port_num == PORTF_ID)
                                && (Port_Configuration_Channel[Pin].pin_num == PIN2_ID)))
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_M0FAULT)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }
                else
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_M0PWM)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }
                break;

            case PIN_MODE_M1PWM:
                /*
                 * Check if the pin not pin 4 port F because this pins used for
                 *  M0FUALT not for M0PWM, for avoiding any conflicting.
                 *
                 */
                if( (Port_Configuration_Channel[Pin].port_num == PORTF_ID)
                        && (Port_Configuration_Channel[Pin].pin_num == PIN4_ID))
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_M1FAULT)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }
                else
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_M1PWM)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }
                break;

            case PIN_MODE_PHASE:
                /*
                 * Check that pin not pin 4 in port C and not pin 3 in port D and not
                 * pin 4 in port F, because this pins used for IDX mode not Phase mode.
                 *
                 */
                if( ((Port_Configuration_Channel[Pin].port_num == PORTC_ID)
                        && (Port_Configuration_Channel[Pin].pin_num == PIN4_ID))
                        || ((Port_Configuration_Channel[Pin].port_num == PORTD_ID)
                                && (Port_Configuration_Channel[Pin].pin_num == PIN3_ID))
                                || (Port_Configuration_Channel[Pin].port_num == PORTF_ID)
                                && (Port_Configuration_Channel[Pin].pin_num == PIN4_ID))
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_IDX)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }
                else
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_PHASE)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }
                break;

            case PIN_MODE_TIMER:
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_TIMER)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                break;

            case PIN_MODE_CAN:

                /*
                 * The condition for checking that if :
                 * 1) the pin 4 and pin 5 in port C those pins used for UART.
                 * 2) the pin 6 and pin 7 in port C and pin 2 and pin 3 in port D and pin 4 in port F those pins used in
                 *    USB communication.
                 * 3) the pin 7 in port D and pin 0 in port F those pins used in NON MASKABLE INTERRUPT.
                 * 4) anything except the 3 points mentioned is used in CAN communication.
                 */
                if((Port_Configuration_Channel[Pin].port_num == PORTC_ID)
                        && ((Port_Configuration_Channel[Pin].pin_num == PIN4_ID)
                                || (Port_Configuration_Channel[Pin].pin_num == PIN5_ID)))
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_UART)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }
                else if( ((Port_Configuration_Channel[Pin].port_num == PORTC_ID)
                        &&((Port_Configuration_Channel[Pin].pin_num == PIN6_ID)
                                ||(Port_Configuration_Channel[Pin].pin_num == PIN7_ID)))
                        ||((Port_Configuration_Channel[Pin].port_num == PORTD_ID)
                                &&((Port_Configuration_Channel[Pin].pin_num == PIN2_ID)
                                        || (Port_Configuration_Channel[Pin].pin_num == PIN3_ID)))
                                        ||((Port_Configuration_Channel[Pin].port_num == PORTF_ID)
                                                && (Port_Configuration_Channel[Pin].pin_num == PIN4_ID)))
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_USB)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }
                else if( ((Port_Configuration_Channel[Pin].port_num == PORTD_ID)
                        && (Port_Configuration_Channel[Pin].pin_num == PIN7_ID))
                        || ((Port_Configuration_Channel[Pin].port_num == PORTF_ID)
                                && (Port_Configuration_Channel[Pin].pin_num == PIN0_ID)))
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_NMI)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }
                else
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_CAN)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }

                break;

            case PIN_MODE_ADC:

                if( (Port_Configuration_Channel[Pin].port_num == PORTF_ID)
                        && ((Port_Configuration_Channel[Pin].pin_num == PIN0_ID)
                                ||   (Port_Configuration_Channel[Pin].pin_num == PIN1_ID)))
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET ) |= ((( uint32 )PIN_MODE_ANALOG_COMP)<<((Port_Configuration_Channel[Pin].pin_num)*4));
                }
                else
                {
                    /*This is ADC mode and it was configured before*/
                }

                break;

            case PIN_MODE_NO_ACTION:

                /*No Action Required*/

                break;

            default:

                /*
                 * the default case report a DET error that indicates
                 * the mode entered is invalid mode.
                 *
                 */
                if (PORT_DEV_ERROR_DETECT == STD_ON){
                    Det_ReportError(
                            PORT_MODULE_ID,
                            PORT_INSTANCE_ID,
                            PORT_INIT_SID,
                            PORT_E_PARAM_INVALID_MODE);
                }
                else
                {
                    /* No Action Required */
                }
                break;

            }/*End of Switch Case*/

        }/*End of else of that pin not one of JTAG pins*/
    }
    else
    {
        /* No Action Required */
    }

}
#endif

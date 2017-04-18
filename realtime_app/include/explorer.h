/*******************************************************************************
*                                                                              *
*    explorer.h                                                                *
*                                                                              *
*******************************************************************************/


#ifndef EXPLORER_H
#define EXPLORER_H

/* Driver includes. */
#include "lpc17xx_gpio.h"

/* Application specific definitions. */
#define SetupHardware()     do{                                     \
                                GPIO_SetDir( 1, (1 << 18), 1 );     \
                                GPIO_SetDir( 1, (1 << 20), 1 );     \
                                GPIO_SetDir( 1, (1 << 21), 1 );     \
                                GPIO_SetDir( 1, (1 << 23), 1 );     \
                            }while(0)

#define TaskIN( tag )       do{                                                     \
                                switch( tag ){                                      \
                                    case 1: GPIO_SetValue( 1, (1 << 18) ); break;   \
                                    case 2: GPIO_SetValue( 1, (1 << 20) ); break;   \
                                    case 3: GPIO_SetValue( 1, (1 << 21) ); break;   \
                                    case 4: GPIO_SetValue( 1, (1 << 23) ); break;   \
                                }                                                   \
                            }while(0)

#define TaskOUT()           do{                                     \
                                GPIO_ClearValue( 1, (1 << 18) );    \
                                GPIO_ClearValue( 1, (1 << 20) );    \
                                GPIO_ClearValue( 1, (1 << 21) );    \
                                GPIO_ClearValue( 1, (1 << 23) );    \
                            }while(0)

#endif /* EXPLORER_H */
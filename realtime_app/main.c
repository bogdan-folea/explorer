/**************************************************************************************
*                                                                                     *
*   Proiect de Licenta                                                                *
*   Bogdan Folea, 2017                                                                *
*                                                                                     *
*   main.c                                                                            *
*                                                                                     *
**************************************************************************************/

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"
#include "semphr.h"

/* Driver includes. */
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pwm.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_i2c.h"

/* Priorities at which the tasks are created. */
#define tskMOTOR_PRIORITY               ( tskIDLE_PRIORITY + 8 )
#define tskSERIALRECV_PRIORITY          ( tskIDLE_PRIORITY + 7 )
#define tskSERIALSEND_PRIORITY          ( tskIDLE_PRIORITY + 6 )
#define tskGATEKEEPER_PRIORITY          ( tskIDLE_PRIORITY + 5 )
#define tskCOMPASS_PRIORITY             ( tskIDLE_PRIORITY + 4 )
#define tskLIGHT_PRIORITY               ( tskIDLE_PRIORITY + 3 )
#define tskPRESSURE_PRIORITY            ( tskIDLE_PRIORITY + 2 )
#define tskHUMIDITY_PRIORITY            ( tskIDLE_PRIORITY + 1 )

/* The task handlers. */
static void prvMotorTask( void );
static void prvSerialRecvTask( void );
static void prvSerialSendTask( void );
static void prvGatekeeperTask( void );
static void prvCompassTask( void );
static void prvLightTask( void );
static void prvPressureTask( void );
static void prvHumidityTask( void );

/* Other required resources. */
volatile uint8_t direction = 'x';
volatile uint8_t forward_block = 0;
volatile uint8_t backward_block = 0;

QueueHandle_t xSerialQueue;

TimerHandle_t xTimer;
TimerHandle_t xBuzzer;

EventGroupHandle_t xCommandEvent;
#define MoveForwardBit      ( 1 << 2 )
#define MoveBackBit         ( 1 << 3 )
#define MoveLeftBit         ( 1 << 4 )
#define MoveRightBit        ( 1 << 5 )

SemaphoreHandle_t xForwardSemaphore, xBackwardSemaphore;

typedef struct message_t{
    uint8_t size;
    uint8_t payload[20];
} message_t;

#define MOVING_SPEED        ( 100 )
#define ROTATE_SPEED        ( 100 )

#define SOUND_ENABLED

#define BMP280_ADDR    ( 0x76 >> 1 )

#define SBIT_CNTEN     0
#define SBIT_PWMEN     2

#define SBIT_PWMMR0R   1

#define SBIT_LEN0      0
#define SBIT_LEN1      1
#define SBIT_LEN2      2
#define SBIT_LEN3      3
#define SBIT_LEN4      4

#define SBIT_PWMENA1   9
#define SBIT_PWMENA2   10
#define SBIT_PWMENA3   11
#define SBIT_PWMENA4   12

/*-----------------------------------------------------------------------------------*/

/* Context switch tracing functions. */

void switchIN(uint32_t tag)
{
    switch( tag ){
    case 1: GPIO_SetValue( 1, ( 1 << 18 ) );
            break;
    case 2: GPIO_SetValue( 1, ( 1 << 20 ) );
            break;
    case 3: GPIO_SetValue( 1, ( 1 << 21 ) );
            break;
    case 4: GPIO_SetValue( 1, ( 1 << 23 ) );
            break;
    }    
}

void switchOUT()
{
    //GPIO_ClearValue( 1, ( 1 << 18 ) );
    GPIO_ClearValue( 1, ( 1 << 20 ) );
    GPIO_ClearValue( 1, ( 1 << 21 ) );
    GPIO_ClearValue( 1, ( 1 << 23 ) );
}

/*-----------------------------------------------------------------------------------*/

/* The error hook. */

void prvErrorHook( void )
{
    LPC_GPIO0->FIODIR &= ~(1 << 4);
    while(1);
}

/*-----------------------------------------------------------------------------------*/

/* Timer callback functions. */

void vTimerCallback( TimerHandle_t pxTimer )
{
    volatile TickType_t xTime, xSeconds, xMinutes, xHours;
    BaseType_t xResult;
        
    xTime = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
    xHours = xTime / 3600;
    xMinutes = ( xTime / 60 ) % 60;
    xSeconds = xTime % 60;

    message_t xMessage;
    xMessage.size = 8;
	xMessage.payload[0] = 'S';
	xMessage.payload[1] = 'U';
	xMessage.payload[2] = '0' + xHours % 10;
	xMessage.payload[3] = '0' + ( xMinutes / 10 ) % 10;
	xMessage.payload[4] = '0' + xMinutes % 10;
	xMessage.payload[5] = '0' + ( xSeconds / 10 ) % 10;
	xMessage.payload[6] = '0' + xSeconds % 10;
	xMessage.payload[7] = '\n';
	
	xResult = xQueueSend( xSerialQueue, /** Queue to send to. */
                          ( void * ) &xMessage, /** Message. */
                          (TickType_t) 0 ); /** Do not block. */
	
	if( xResult != pdPASS ){
	    /* Failed to post message to queue. */
	    prvErrorHook();
	}
}

void vBuzzerCallback( TimerHandle_t pxTimer )
{
    GPIO_ClearValue( 0, ( 1 << 5 ) );
}

/*-----------------------------------------------------------------------------------*/

/* PWM related functions. */

void SetDutyCycle(uint32_t m11, uint32_t m12, uint32_t m21, uint32_t m22)
{
    LPC_PWM1->MR1 = m11;
    LPC_PWM1->MR2 = m12;
    LPC_PWM1->MR3 = m21;
    LPC_PWM1->MR4 = m22;
}

void EnablePWM()
{
    /* Trigger the latch Enable Bits to load the new values. */
    LPC_PWM1->LER = ( 1 << SBIT_LEN0 ) |
                    ( 1 << SBIT_LEN1 ) |
                    ( 1 << SBIT_LEN2 ) |
                    ( 1 << SBIT_LEN3 ) |
                    ( 1 << SBIT_LEN4 );
    
    /* Enable the PWM output pins for PWM. */
    LPC_PWM1->PCR = ( 1 << SBIT_PWMENA1 ) |
                    ( 1 << SBIT_PWMENA2 ) |
                    ( 1 << SBIT_PWMENA3 ) |
                    ( 1 << SBIT_PWMENA4 );
}

/*-----------------------------------------------------------------------------------*/

/* External interrupt handler. */

void EINT3_IRQHandler( void )
{
    uint8_t sound = 0;

    portDISABLE_INTERRUPTS();
    LPC_GPIO1->FIOSET |= ( 1 << 18 );
    
    BaseType_t xResult, xHigherPriorityTaskWoken = pdFALSE;
    
    LPC_GPIOINT->IO0IntClr |= ( 1 << 8 );
    LPC_GPIOINT->IO0IntClr |= ( 1 << 9 );
    
    if( ( LPC_GPIO0->FIOPIN & ( 1 << 8 ) ) == 0 && direction == 'f' ){
        SetDutyCycle( 0, 0, 0, 0);
        EnablePWM();
        direction = 'x';
        sound = 1;
    }
    
    if( ( LPC_GPIO0->FIOPIN & ( 1 << 9 ) ) == 0 && direction == 'b' ){
        SetDutyCycle( 0, 0, 0, 0);
        EnablePWM();
        direction = 'x';
        sound = 1;
    }
    
    #ifdef SOUND_ENABLED
        if( sound == 1 ){
            GPIO_SetValue( 0, ( 1 << 5 ) );
            
            xResult = xTimerStartFromISR( xBuzzer, /* The timer being started. */
                                          &xHigherPriorityTaskWoken ); /* Check context. */
            
            if( xResult != pdPASS ){
                //prvErrorHook();
            }
        }
    #endif
    
    LPC_GPIO1->FIOCLR |= ( 1 << 18 );
    portENABLE_INTERRUPTS();
    
    /* Perform a context switch if needed. */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*-----------------------------------------------------------------------------------*/

void SetupHardware()
{
    /* Pin multiplexing configuration. */
	PINSEL_CFG_Type PinCfg;
	
	/* Enable built-in LEDs. */
	GPIO_SetDir( 1, ( 1 << 18 ), 1 );
    GPIO_SetDir( 1, ( 1 << 20 ), 1 );
    GPIO_SetDir( 1, ( 1 << 21 ), 1 );
    GPIO_SetDir( 1, ( 1 << 23 ), 1 );
    
    /* Enable buzzer and light pins. */
    GPIO_SetDir( 0, ( 1 << 5 ), 1 );
    //GPIO_SetDir( 0, (1 << 8), 1 ); !!
    //GPIO_SetDir( 0, (1 << 9), 1 ); !!
	
	/* Enable external interrupt. */
	LPC_GPIO0->FIODIR &= ~( 1 << 8 );
	LPC_GPIO0->FIODIR &= ~( 1 << 9 );
	LPC_GPIOINT->IO0IntEnR |= ( 1 << 8 );
	LPC_GPIOINT->IO0IntEnF |= ( 1 << 8 );
	LPC_GPIOINT->IO0IntEnR |= ( 1 << 9 );
	LPC_GPIOINT->IO0IntEnF |= ( 1 << 9 );
	NVIC_SetPriority(EINT3_IRQn, 6);
	NVIC_EnableIRQ(EINT3_IRQn);
	
    /* Enable Counters and the PWM module. */
	LPC_PINCON->PINSEL4 = 0x55; 
	
    LPC_PWM1->TCR = ( 1 << SBIT_CNTEN ) | ( 1 << SBIT_PWMEN );
	
    LPC_PWM1->PR  =  0x1000; /** Prescaler */
    LPC_PWM1->MCR = ( 1 << SBIT_PWMMR0R );  /** Reset on PWMMR0. */
	
	/* Set PWM duty cycle. */
	LPC_PWM1->MR0 = 100;   
    SetDutyCycle( 0, 0, 0, 0 );
	
	/* Enable PWM. */
    EnablePWM();
    
    /* ADC Configuration. */
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 23;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	
	/* Set ADC conversion rate to 200kHz. */
	ADC_Init( LPC_ADC, 200000 );
	ADC_IntConfig( LPC_ADC, ADC_ADINTEN0 ,DISABLE );
	ADC_ChannelCmd( LPC_ADC, ADC_CHANNEL_0, ENABLE );
	
	/* I2C Configuration. */
	PinCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);
	I2C_Init( LPC_I2C2, 100000 );
	I2C_Cmd( LPC_I2C2, I2C_MASTER_MODE, ENABLE );
    
    /* UART Configuration. */
	UART_CFG_Type UARTConfigStruct;
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);
	
	UART_ConfigStructInit(&UARTConfigStruct);

	UART_Init(LPC_UART3, &UARTConfigStruct);
	
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);

	UART_FIFOConfig(LPC_UART3, &UARTFIFOConfigStruct);
	
	/* Enable Interrupt for UART. */
	//UART_IntConfig((LPC_UART_TypeDef *)LPC_UART3, UART_INTCFG_RBR, ENABLE);
	//UART_IntConfig((LPC_UART_TypeDef *)LPC_UART3, UART_INTCFG_RLS, ENABLE);
    //NVIC_SetPriority(UART3_IRQn, ((0x01<<3)|0x01));
    //NVIC_EnableIRQ(UART3_IRQn);

	/** Enable UART. */
	UART_TxCmd(LPC_UART3, ENABLE);
}

/*-----------------------------------------------------------------------------------*/

#if (defined(GCC))
int main( void )
#endif
{
    /* Initialize the system. */
	SystemInit();
	
	/* Set up hardware. */
	SetupHardware();
	
	/* Create tasks and set priority for each. */
	TaskHandle_t xMotorHandle = NULL;
	TaskHandle_t xSerialRecvHandle = NULL;
	TaskHandle_t xSerialSendHandle = NULL;
	TaskHandle_t xGatekeeperHandle = NULL;
	TaskHandle_t xCompassHandle = NULL;
	TaskHandle_t xLightHandle = NULL;
	TaskHandle_t xPressureHandle = NULL;
	TaskHandle_t xHumidityHandle = NULL;
	
	xTaskCreate( (TaskFunction_t)prvMotorTask, /** Function that implements the task. */
	             "TaskMotor", /** Text name of the task. */
	             configMINIMAL_STACK_SIZE, /** Stack size in words, not bytes. */
	             NULL, /** No parameters passed into the task. */
	             tskMOTOR_PRIORITY, /** Priority at which the task is created. */
	             &xMotorHandle ); /** Used to pass out the created task's handle. */
	
	xTaskCreate( (TaskFunction_t)prvSerialRecvTask, /** Function that implements the task. */
	             "TaskSerialRecv", /** Text name of the task. */
	             configMINIMAL_STACK_SIZE, /** Stack size in words, not bytes. */
	             NULL, /** No parameters passed into the task. */
	             tskSERIALRECV_PRIORITY, /** Priority at which the task is created. */
	             &xSerialRecvHandle ); /** Used to pass out the created task's handle. */
	
	xTaskCreate( (TaskFunction_t)prvSerialSendTask, /** Function that implements the task. */
	             "TaskSerialSend", /** Text name of the task. */
	             configMINIMAL_STACK_SIZE, /** Stack size in words, not bytes. */
	             NULL, /** No parameters passed into the task. */
	             tskSERIALSEND_PRIORITY, /** Priority at which the task is created. */
	             &xSerialSendHandle ); /** Used to pass out the created task's handle. */
	
	xTaskCreate( (TaskFunction_t)prvGatekeeperTask, /** Function that implements the task. */
	             "TaskGatekeeper", /** Text name of the task. */
	             configMINIMAL_STACK_SIZE, /** Stack size in words, not bytes. */
	             NULL, /** No parameters passed into the task. */
	             tskGATEKEEPER_PRIORITY, /** Priority at which the task is created. */
	             &xGatekeeperHandle ); /** Used to pass out the created task's handle. */
	
	xTaskCreate( (TaskFunction_t)prvCompassTask, /** Function that implements the task. */
	             "TaskCompass", /** Text name of the task. */
	             configMINIMAL_STACK_SIZE, /** Stack size in words, not bytes. */
	             NULL, /** No parameters passed into the task. */
	             tskCOMPASS_PRIORITY, /** Priority at which the task is created. */
	             &xCompassHandle ); /** Used to pass out the created task's handle. */
	
	xTaskCreate( (TaskFunction_t)prvLightTask, /** Function that implements the task. */
	             "TaskLight", /** Text name of the task. */
	             configMINIMAL_STACK_SIZE, /** Stack size in words, not bytes. */
	             NULL, /** No parameters passed into the task. */
	             tskLIGHT_PRIORITY, /** Priority at which the task is created. */
	             &xLightHandle ); /** Used to pass out the created task's handle. */
	
	xTaskCreate( (TaskFunction_t)prvPressureTask, /** Function that implements the task. */
	             "TaskPressure", /** Text name of the task. */
	             configMINIMAL_STACK_SIZE, /** Stack size in words, not bytes. */
	             NULL, /** No parameters passed into the task. */
	             tskPRESSURE_PRIORITY, /** Priority at which the task is created. */
	             &xPressureHandle ); /** Used to pass out the created task's handle. */
	
	xTaskCreate( (TaskFunction_t)prvHumidityTask, /** Function that implements the task. */
	             "TaskHumidity", /** Text name of the task. */
	             configMINIMAL_STACK_SIZE, /** Stack size in words, not bytes. */
	             NULL, /** No parameters passed into the task. */
	             tskHUMIDITY_PRIORITY, /** Priority at which the task is created. */
	             &xHumidityHandle ); /** Used to pass out the created task's handle. */
	
	/* Assign a tag to each task. */
	vTaskSetApplicationTaskTag( xMotorHandle, ( void * ) 4 );
	vTaskSetApplicationTaskTag( xSerialRecvHandle, ( void * ) 2 );
	vTaskSetApplicationTaskTag( xSerialSendHandle, ( void * ) 2 );
	vTaskSetApplicationTaskTag( xGatekeeperHandle, ( void * ) 3 );
	vTaskSetApplicationTaskTag( xCompassHandle, ( void * ) 3 );
	vTaskSetApplicationTaskTag( xLightHandle, ( void * ) 3 );
	vTaskSetApplicationTaskTag( xPressureHandle, ( void * ) 3 );
	vTaskSetApplicationTaskTag( xHumidityHandle, ( void * ) 3 );
	
	/* Set up events and other resources. */
	xCommandEvent = xEventGroupCreate();
    
    if( xCommandEvent == NULL )
    {
        /* Insufficient heap memory available. */
        prvErrorHook();
    }
    
    xSerialQueue = xQueueCreate( 10, sizeof( message_t ) );
    if( xSerialQueue == 0 ){
        prvErrorHook();
    }
    
    xTimer = xTimerCreate( "Timer", /** Text name not used by the kernel. */
                           1000 * portTICK_PERIOD_MS, /** Set timer period to 1s. */
                           pdTRUE, /** Configure as a recurring timer. */
                           ( void * ) 0, /** ID of the timer. */
                           vTimerCallback ); /** Timer callback function. */
	
	xBuzzer = xTimerCreate( "BuzzerTimer", /** Text name not used by the kernel. */
                            500 * portTICK_PERIOD_MS, /** Set timer period to 500ms. */
                            pdFALSE, /** Configure as a non-recurring timer. */
                            ( void * ) 1, /** ID of the timer. */
                            vBuzzerCallback ); /** Timer callback function. */
	
	xForwardSemaphore = xSemaphoreCreateBinary();
	xBackwardSemaphore = xSemaphoreCreateBinary();
	
	BaseType_t xResult;
	xResult = xSemaphoreGive( xForwardSemaphore );
	if( xResult != pdPASS ){
	    prvErrorHook();
	}
	xResult = xSemaphoreGive( xBackwardSemaphore );
	if( xResult != pdPASS ){
	    prvErrorHook();
	}
	
	/* Wait for other devices to initialize. */
	uint32_t i = 0;
	for( i = 0; i < 30000000; i++ ){
	    asm("nop");
    }
	
	/* Clear STATUS LED. */
	LPC_GPIO0->FIODIR |= (1 << 4);
	LPC_GPIO0->FIOCLR |= (1 << 4);
	
	/* Start the tasks and set the timer running. */
	vTaskStartScheduler();
	
	/* If all is well, the scheduler will now be running, and the following line will
	never be reached.  If the following line does execute, then there was insufficient
	FreeRTOS heap memory available for the idle and/or timer tasks to be created. */
	prvErrorHook();
	
	return 0;
}

/*-----------------------------------------------------------------------------------*/

static void prvMotorTask( void )
{
	do{
	    EventBits_t uxBits;
	    
	    uxBits = xEventGroupWaitBits( xCommandEvent, /* Event group being tested. */
                                      MoveForwardBit | MoveBackBit | MoveLeftBit | MoveRightBit, /* Events to wait for. */
                                      pdTRUE, /* Clear events before returning. */
                                      pdFALSE, /* Wait for either event. */
                                      100 * portTICK_PERIOD_MS ); /* Set the timeout to 200ms. */
        
        if( ( uxBits & MoveForwardBit ) != 0 ){
            if( ( LPC_GPIO0->FIOPIN & ( 1 << 8 ) ) ){
                SetDutyCycle( MOVING_SPEED, 0, MOVING_SPEED , 0 );
                EnablePWM();
                direction = 'f';
            }
        } else if( ( uxBits & MoveBackBit ) != 0 ){
            if( ( LPC_GPIO0->FIOPIN & ( 1 << 9 ) ) ){
                SetDutyCycle( 0, MOVING_SPEED, 0, MOVING_SPEED );
                EnablePWM();
                direction = 'b';
            }
        } else if( ( uxBits & MoveLeftBit ) != 0 ){
            SetDutyCycle( ROTATE_SPEED, 0, 0, ROTATE_SPEED );
            EnablePWM();
            direction = 'l';
        } else if( ( uxBits & MoveRightBit ) != 0 ){
            SetDutyCycle( 0, ROTATE_SPEED, ROTATE_SPEED, 0 );
            EnablePWM();
            direction = 'r';
        } else { /* Timeout. */
            SetDutyCycle( 0, 0, 0, 0);
            EnablePWM();
            direction = 'x';
        }
	}while(1);
}

/*-----------------------------------------------------------------------------------*/

static void prvSerialRecvTask( void )
{
	uint32_t len = 0;
	uint8_t rxbuf[2];
	
	do{
		len = UART_Receive(LPC_UART3, rxbuf, 1, NONE_BLOCKING);
		if( len != 0 ){
	        switch( rxbuf[len - 1] ){
	        case 'f': xEventGroupSetBits( xCommandEvent, /* Event group being updated. */
                                          MoveForwardBit ); /* Event being set. */
	                  break;
	        case 'b': xEventGroupSetBits( xCommandEvent, /* Event group being updated. */
                                          MoveBackBit ); /* Event being set. */
	                  break;
	        case 'l': xEventGroupSetBits( xCommandEvent, /* Event group being updated. */
                                          MoveLeftBit ); /* Event being set. */
	                  break;
	        case 'r': xEventGroupSetBits( xCommandEvent, /* Event group being updated. */
                                          MoveRightBit ); /* Event being set. */
	                  break;
	        default: prvErrorHook();
	        }
		} else {
		    vTaskDelay( 100 * portTICK_PERIOD_MS ); /** Sleep 100ms. */
		}
	}while(1);
	    
}

/*-----------------------------------------------------------------------------------*/

static void prvSerialSendTask( void )
{
    BaseType_t xResult;
    message_t xMessage;
    
    /* Send initial message. */
	UART_Send(LPC_UART3, (uint8_t*)"START\0", 6, BLOCKING);
	
	xResult = xTimerStart( xTimer, /* The timer being started. */
                           (TickType_t) 0 ); /** Do not block. */
	
	if( xResult == pdFAIL ){
	    prvErrorHook();
	}
	
	do{
	    /* Get next message from queue. */
		xResult = xQueueReceive( xSerialQueue, /** Queue to receive from. */
                                 &xMessage, /** Message. */
                                 100 * portTICK_PERIOD_MS ); /** Block for 100ms. */
	    
	    if( xResult == pdTRUE ){
            /* Send message over UART. */
            UART_Send(LPC_UART3, xMessage.payload, xMessage.size, BLOCKING);
        }
	}while(1);
}

/*-----------------------------------------------------------------------------------*/

static void prvGatekeeperTask( void )
{
    I2C_M_SETUP_Type transferCfg;
	BaseType_t xResult;
    
	do{
	    uint8_t  recv, iocon_cfg[2] = { ( 0x0E << 3 ), 0x00 };
	    
	    /* Write to I2C. */
	    transferCfg.sl_addr7bit = BMP280_ADDR;
	    transferCfg.tx_data = (uint8_t *)iocon_cfg;
	    transferCfg.tx_length = sizeof(iocon_cfg);
	    transferCfg.rx_data = NULL;
	    transferCfg.rx_length = 0;
	    transferCfg.retransmissions_max = 2;
	    
	    I2C_MasterTransferData( LPC_I2C2, &transferCfg, I2C_TRANSFER_POLLING );
	    
	    /* Read from I2C. */
	    transferCfg.sl_addr7bit = BMP280_ADDR;
	    transferCfg.tx_data = (uint8_t *)iocon_cfg;
	    transferCfg.tx_length = 1;
	    transferCfg.rx_data = (uint8_t *)&recv;
	    transferCfg.rx_length = 1;
	    transferCfg.retransmissions_max = 2;
	    
	    I2C_MasterTransferData( LPC_I2C2, &transferCfg, I2C_TRANSFER_POLLING );
	    
	    message_t xMessage;
		xMessage.size = 4;
		xMessage.payload[0] = 'S';
		xMessage.payload[1] = 'P';
		xMessage.payload[2] = recv;
		xMessage.payload[3] = '\n';
		
		xResult = xQueueSend( xSerialQueue, /** Queue to send to. */
                              ( void * ) &xMessage, /** Message. */
                              (TickType_t) 0 ); /** Do not block. */
		
		if( xResult != pdPASS ){
		    /* Failed to post message to queue. */
		    prvErrorHook();
		}
	    
        vTaskDelay( 1000 * portTICK_PERIOD_MS ); /** Sleep 10ms. */
	}while(1);
}

/*-----------------------------------------------------------------------------------*/

static void prvLightTask( void )
{
    uint32_t adc_value;
    BaseType_t xResult;
    uint32_t i;
    
	do{
	    for( i = 0; i < 3; i++ ){
            /* Start conversion. */
		    ADC_StartCmd( LPC_ADC, ADC_START_NOW );
		
		    /* Wait for conversion to complete. */
		    while( !(ADC_ChannelGetStatus( LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE )));
		
		    /* Read value. */
		    if( i == 0 ){
		        adc_value = ADC_ChannelGetData( LPC_ADC, ADC_CHANNEL_0 );
		    } else {
		        adc_value += ADC_ChannelGetData( LPC_ADC, ADC_CHANNEL_0 );
		    }
		    
		    /* Delay. */
		    if( i < 2 ){		    
		        vTaskDelay( 100 * portTICK_PERIOD_MS ); /** Sleep 100ms. */
		    }
		}
		
        adc_value /= 5;
		
		if( adc_value < 15 ){
		    //GPIO_SetValue( 0, (1 << 8) ); !!
		    //GPIO_SetValue( 0, (1 << 9) ); !!
		} else {
		    //GPIO_ClearValue( 0, (1 << 8) ); !!
		    //GPIO_ClearValue( 0, (1 << 9) ); !!
		}
		
		message_t xMessage;
		xMessage.size = 7;
		xMessage.payload[0] = 'S';
		xMessage.payload[1] = 'L';
		xMessage.payload[2] = '0' + ( adc_value / 1000 ) % 10;
		xMessage.payload[3] = '0' + ( adc_value / 100 ) % 10;
		xMessage.payload[4] = '0' + ( adc_value / 10 ) % 10;
		xMessage.payload[5] = '0' + adc_value % 10;
		xMessage.payload[6] = '\n';
		
		xResult = xQueueSend( xSerialQueue, /** Queue to send to. */
                              ( void * ) &xMessage, /** Message. */
                              (TickType_t) 0 ); /** Do not block. */
		
		if( xResult != pdPASS ){
		    /* Failed to post message to queue. */
		    prvErrorHook();
		}
		
		vTaskDelay( 200 * portTICK_PERIOD_MS ); /** Sleep 200ms. */
	}while(1);
}

/*-----------------------------------------------------------------------------------*/

static void prvCompassTask( void )
{
	do{
        vTaskDelay( 1000 * portTICK_PERIOD_MS ); /** Sleep 10ms. */
	}while(1);
}

/*-----------------------------------------------------------------------------------*/

static void prvPressureTask( void )
{
	do{
        vTaskDelay( 1000 * portTICK_PERIOD_MS ); /** Sleep 10ms. */
	}while(1);
}

/*-----------------------------------------------------------------------------------*/

static void prvHumidityTask( void )
{
	do{
        vTaskDelay( 1000 * portTICK_PERIOD_MS ); /** Sleep 10ms. */
	}while(1);
}

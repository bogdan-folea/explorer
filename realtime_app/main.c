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

EventGroupHandle_t xCommandEvent;
#define MoveForwardBit      ( 1 << 2 )
#define MoveBackBit         ( 1 << 3 )
#define MoveLeftBit         ( 1 << 4 )
#define MoveRightBit        ( 1 << 5 )

EventGroupHandle_t xSerialEvent;
#define SerialEventBit      ( 1 << 2 )

typedef struct message_t{
    uint8_t payload[20];
} message_t;

#define MOVING_SPEED        ( 35 )
#define ROTATE_SPEED        ( 55 )

#define GPIO_INT (1 << 9)   // GPIO interrupt is on p0.9
#define OUTPIN	 (1 << 8)   // outpin on p0.8

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
    case 1: GPIO_SetValue( 1, (1 << 18) );
            break;
    case 2: GPIO_SetValue( 1, (1 << 20) );
            break;
    case 3: GPIO_SetValue( 1, (1 << 21) );
            break;
    case 4: GPIO_SetValue( 1, (1 << 23) );
            break;
    }    
}

void switchOUT()
{
    GPIO_ClearValue( 1, (1 << 18) );
    GPIO_ClearValue( 1, (1 << 20) );
    GPIO_ClearValue( 1, (1 << 21) );
    GPIO_ClearValue( 1, (1 << 23) );
}

/*-----------------------------------------------------------------------------------*/

/* External interrupt handler. */

void EINT3_IRQHandler(void){
	#if 0
	LPC_GPIOINT->IO0IntClr=(1 << 9);
	
	if( toggle == 0 ){
		LPC_GPIO1->FIOSET |= LED4;
		LPC_GPIO0->FIOSET |= (1 << 8);
		toggle = 1;
	}else{
		LPC_GPIO1->FIOCLR |= LED4;
		LPC_GPIO0->FIOCLR |= (1 << 8);		
		toggle = 0;
	}
	
	LPC_GPIO0->FIOSET |= (1 << 8);
	
	/*
	if (LPC_GPIO1->FIOPIN & (1 << 18))
    {
        LPC_GPIO1->FIOCLR |= (1 << 18);
    }
    else
    {
        LPC_GPIO1->FIOSET |= (1 << 18);
    }*/
    #endif
}

/*-----------------------------------------------------------------------------------*/

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

void SetupHardware()
{
    
	PINSEL_CFG_Type PinCfg;
	
	/* Enable built-in LEDs. */
	GPIO_SetDir( 1, (1 << 18), 1 );
    GPIO_SetDir( 1, (1 << 20), 1 );
    GPIO_SetDir( 1, (1 << 21), 1 );
    GPIO_SetDir( 1, (1 << 23), 1 );
	
	/* Enable external interrupt. */
	//LPC_GPIO0->FIODIR |= (1 << 8);
	//LPC_GPIO0->FIODIR &= ~GPIO_INT;
	//LPC_GPIO1->FIODIR |= LED1;
	//Enable interrupt
	//LPC_GPIOINT->IO0IntEnR |=(1<<9);
	//NVIC_EnableIRQ(EINT3_IRQn);
	
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
	
	/* Set ADC conversion rate to 200Hz. */
	ADC_Init( LPC_ADC, 200 );
	ADC_IntConfig( LPC_ADC, ADC_ADINTEN0 ,DISABLE );
	ADC_ChannelCmd( LPC_ADC, ADC_CHANNEL_0, ENABLE );
    
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
        LPC_GPIO0->FIODIR |= (1 << 4);
	    LPC_GPIO0->FIOSET |= (1 << 4);
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
	while(1);
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
                                      150 * portTICK_PERIOD_MS ); /* Set the timeout to 200ms. */
        
        if( ( uxBits & MoveForwardBit ) != 0 ){
            SetDutyCycle( MOVING_SPEED, 0, MOVING_SPEED , 0 );
            EnablePWM();
            direction = 'f';
        } else if( ( uxBits & MoveBackBit ) != 0 ){
            SetDutyCycle( 0, MOVING_SPEED, 0, MOVING_SPEED );
            EnablePWM();
            direction = 'b';
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
	        default: LPC_GPIO0->FIOSET |= (1 << 4);
	        }
		} else {
		    vTaskDelay( 100 * portTICK_PERIOD_MS ); /** Sleep 100ms. */
		}
	}while(1);
	    
}

/*-----------------------------------------------------------------------------------*/

static void prvSerialSendTask( void )
{	
	UART_Send(LPC_UART3, (uint8_t*)"START\0", 6, BLOCKING);
	
	do{
		// send message from queue		
		vTaskDelay( 1000 * portTICK_PERIOD_MS ); /** Sleep 10ms. */
	}while(1);
}

/*-----------------------------------------------------------------------------------*/

static void prvGatekeeperTask( void )
{
	do{	
        vTaskDelay( 1000 * portTICK_PERIOD_MS ); /** Sleep 10ms. */
	}while(1);
}

/*-----------------------------------------------------------------------------------*/

static void prvLightTask( void )
{
    UART_Send(LPC_UART3, (uint8_t*)"TEsTING\0", 8, BLOCKING);
    
    uint32_t adc_value;
    double lux_value;
    
	do{
        /* Start conversion. */
		ADC_StartCmd( LPC_ADC, ADC_START_NOW );
		
		/* Wait for conversion to complete. */
		while (!(ADC_ChannelGetStatus( LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE )));
		
		/* Read value. */
		adc_value = ADC_ChannelGetData( LPC_ADC, ADC_CHANNEL_0 );
		
		/* Convert value to lux. */
		lux_value = (double)200 * adc_value / 4096;
		
		lux_value = lux_value;
		if( lux_value < 2000 ){
	        //LPC_GPIO0->FIOSET |= (1 << 4);
	        LPC_GPIO0->FIODIR &= ~(1 << 4);
		}
		
		/* Convert to string. */
		uint8_t message[8];
		
		message[0] = 'S';
		message[1] = 'L';
		message[2] = '0' + adc_value % 10;
		message[3] = '0' + ( adc_value / 10 ) % 10;
		message[4] = '0' + ( adc_value / 100 ) % 10;
		message[5] = '0' + ( adc_value / 1000 ) % 10;
		message[6] = '\n';
		message[7] = '\0';
		
		//UART_Send(LPC_UART3, (uint8_t*)"SL", 2, BLOCKING);
		//UART_Send(LPC_UART3, value, 4, BLOCKING);
		UART_Send(LPC_UART3, message, 7, BLOCKING);
		
		vTaskDelay( 1000 * portTICK_PERIOD_MS ); /** Sleep 100ms. */
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
	    uint32_t i = 1000000;
	    while(i--);
        //vTaskDelay( 1000 * portTICK_PERIOD_MS ); /** Sleep 10ms. */
	}while(1);
}

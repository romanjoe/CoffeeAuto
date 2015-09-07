#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "semphr.h"
#include "timers.h"

#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_pwr.h"

#define HARD_FAULTS_DEBUG 0

#define MOTOR_PWM_PERIOD 3800;

typedef enum {EMPTY = 0, FULL = 1} jar_state;

xSemaphoreHandle empty_sem;
xSemaphoreHandle full_sem;

TaskHandle_t blink_handler;
TaskHandle_t start_filling_hanlder;
TaskHandle_t stop_filling_handler;

TimerHandle_t recue_tim_handler;

jar_state jar_status = EMPTY;

#if (HARD_FAULTS_DEBUG == 1)

void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress);
/* @naked@ attribute leads to compiler not save processor
 * context, while entering ISR. Use this, because we made this
 * routine explicitly in HardFault_Handler. Link to read
 * http://www.freertos.org/implementation/a00013.html */
__attribute__ ( (naked) ) void HardFault_Handler(void);

/*
 * pulFaultStackAddress - this is a pointer to a space in memory where
 * processor registers are located after HardFault_Handler execution.
 * This pointer is located in processor register R1, due to
 * @Procedure call Standard for ARM Architecture@, since
 * it is an argument to a function.
 *
 * */

void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
	volatile uint32_t r0 = 0xFFFFFFFF;
	volatile uint32_t r1 = 0xFFFFFFFF;
	volatile uint32_t r2 = 0xFFFFFFFF;
	volatile uint32_t r3 = 0xFFFFFFFF;
	volatile uint32_t r12 = 0xFFFFFFFF;
	volatile uint32_t lr = 0xFFFFFFFF;
	volatile uint32_t pc = 0xFFFFFFFF;
	volatile uint32_t psr = 0xFFFFFFFF;

	r0 = pulFaultStackAddress[0];
	r1 = pulFaultStackAddress[1];
	r2 = pulFaultStackAddress[2];
	r3 = pulFaultStackAddress[3];

	r12 = pulFaultStackAddress[4];
	lr = pulFaultStackAddress[5];
	pc = pulFaultStackAddress[6];
	psr = pulFaultStackAddress[7];

	taskDISABLE_INTERRUPTS();

	while(1);
}

/*
 * Hard fault handler for Cortex M0 core. This implementation is based on
 * description of such handler for Cortex M3-M4 here.
 * http://www.freertos.org/Debugging-Hard-Faults-On-Cortex-M-Microcontrollers.html
 * Required changes have been made due to differences in instruction sets.
 *
 * */

void HardFault_Handler(void)
{
	__asm volatile
	(
		"mov r0, lr													\n"
		"mov r1, #0xD												\n"
		"tst r0, r1 												\n"
		"beq proc_stack												\n"
		"mrs r0, psp												\n"
		"proc_stack: mrs r0, msp									\n"
		"ldr r1, [r0, #24]											\n"
		"ldr r2, handler2_address_const								\n"
		"bx r2														\n"
		".align 2  													\n"
		"handler2_address_const: .word prvGetRegistersFromStack		\n"
	);
}

#endif

/*
 * Application overflow hook function is described here
 * http://www.freertos.org/Stacks-and-stack-overflow-checking.html
 *
 * */

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

/*
 * Function to initialize TIM17 into PWM mode for motor
 * driving.
 *
 * */

void motor_pwm_setup(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

	TIM_TimeBaseInitTypeDef recue_tim;
	TIM_TimeBaseStructInit(&recue_tim);

	recue_tim.TIM_CounterMode = TIM_CounterMode_Down;
	recue_tim.TIM_Period = 4000;
	recue_tim.TIM_Prescaler = 48 - 1;
	TIM_TimeBaseInit(TIM17, &recue_tim);

	TIM_OCInitTypeDef pwm_motor_channel;
	TIM_OCStructInit(&pwm_motor_channel);

	pwm_motor_channel.TIM_OCMode = TIM_OCMode_PWM1;
	pwm_motor_channel.TIM_OutputState = TIM_OutputState_Enable;
	pwm_motor_channel.TIM_Pulse = MOTOR_PWM_PERIOD;
	TIM_OC1Init(TIM17, &pwm_motor_channel);

	TIM_OC1PreloadConfig(TIM17, ENABLE);
	/* without enabling there will be no signal on corresponding channels */
	TIM_CtrlPWMOutputs(TIM17, DISABLE);
	TIM_Cmd(TIM17, DISABLE);
}

/*
 * EXTI1 and EXTI0 are configured to work with 2 HSI sensors in trigger mode.
 * Both are mapped to the same handler in interrupt vector table.
 *
 * */

void interrupts_setup(void)
{
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1); //Connect EXTI LineX to GPIOAX pin

	//Configure EXTI Line 0

	EXTI_InitTypeDef hsi_high_int;

	hsi_high_int.EXTI_Line = EXTI_Line0;
	hsi_high_int.EXTI_Mode = EXTI_Mode_Interrupt;
	hsi_high_int.EXTI_Trigger = EXTI_Trigger_Rising;
	hsi_high_int.EXTI_LineCmd = ENABLE;
	EXTI_Init (&hsi_high_int);

	EXTI_InitTypeDef hsi_low_int;

	hsi_low_int.EXTI_Line = EXTI_Line1;
	hsi_low_int.EXTI_Mode = EXTI_Mode_Interrupt;
	hsi_low_int.EXTI_Trigger = EXTI_Trigger_Rising;
	hsi_low_int.EXTI_LineCmd = ENABLE;
	EXTI_Init (&hsi_low_int);

	NVIC_InitTypeDef NVIC_EXTI_Initstructure;

	NVIC_EXTI_Initstructure.NVIC_IRQChannel = EXTI0_1_IRQn;
	NVIC_EXTI_Initstructure.NVIC_IRQChannelPriority = 0x01;
	NVIC_EXTI_Initstructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_EXTI_Initstructure);
}

/*
 * GPIO_Pin_9 - is connected to green led, is used for ongoing process indication
 * GPIO_Pin_10 - is connected to red led, is used to indicate, that process was
 * 				 stopped after timeout and no signal from high point HSI sensor was received
 * GPIO_Pin_7 - is mapped to TIM17 channel 1, used to provide PWM signal to motor.
 * GPIO_Pin_1 and GPIO_Pin_0 - are used to connect HSI sensors. Lower point and higher point
 * */

void gpio_setup(void)
{
	RCC_AHBPeriphClockCmd(RCC_APB2ENR_SYSCFGEN, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_DeInit(GPIOA);

	GPIO_InitTypeDef led_init;

	led_init.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	led_init.GPIO_Mode = GPIO_Mode_OUT;
	led_init.GPIO_OType = GPIO_OType_PP;
	led_init.GPIO_PuPd = GPIO_PuPd_DOWN;
	led_init.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &led_init);

	GPIO_InitTypeDef hsi_high_sensor;

	hsi_high_sensor.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	hsi_high_sensor.GPIO_Mode = GPIO_Mode_IN;
	hsi_high_sensor.GPIO_OType = GPIO_OType_PP;
	hsi_high_sensor.GPIO_PuPd = GPIO_PuPd_DOWN;
	hsi_high_sensor.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_Init(GPIOA, &hsi_high_sensor);

	GPIO_InitTypeDef motor_control;

	motor_control.GPIO_Pin = GPIO_Pin_7;
	motor_control.GPIO_Mode = GPIO_Mode_AF;
	motor_control.GPIO_OType = GPIO_OType_PP;
	motor_control.GPIO_PuPd = GPIO_PuPd_UP;
	motor_control.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &motor_control);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5); // AF5 - alternate function for channel 1 of TIM17 (refer to datasheet, page 34)

}

/*
 * This is implementation of idle task hook. System goes to low power mode
 * if no active tasks are present and the jar is full.
 *
 * */

void vApplicationIdleHook( void )
{
	if (jar_status == FULL)
	{
		PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
	}
}

/*
 * Task to handle lower HSI sensor. Waiting for semaphore from EXTI1
 * interrupt, then starts recue timeout timer and starts pwm modulation
 * for motor driving.
 *
 * */

void start_filling(void * pvParameters)
{
	for(;;)
	{
		if (xSemaphoreTake(empty_sem, portMAX_DELAY) == pdPASS)
		{
			if (xTimerStart(recue_tim_handler, 0) == pdPASS)
			{
				if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == SET)
				{
					jar_status = EMPTY;
					TIM_Cmd(TIM17, ENABLE);
					TIM_CtrlPWMOutputs(TIM17, ENABLE);
					vTaskResume(blink_handler);
					GPIO_ResetBits(GPIOA, (GPIO_Pin_9 | GPIO_Pin_10));
				}
			}
		}
	}
	vTaskDelete(NULL);
}

/*
 * Task to handle higher HSI sensor. Waiting for semaphore from EXTI0
 * interrupt, then stops recue timeout timer and disables pwm modulation
 * to stop motor.
 *
 * */

void stop_filling(void * pvParameters )
{
	for(;;)
	{
		if (xSemaphoreTake(full_sem, portMAX_DELAY) == pdPASS)
		{
			if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == SET)
			{
				jar_status = FULL;
				TIM_Cmd(TIM17, DISABLE);
				TIM_CtrlPWMOutputs(TIM17, DISABLE);
				TIM17->CCR1 = MOTOR_PWM_PERIOD;
				vTaskSuspend(blink_handler);
				GPIO_ResetBits(GPIOA, (GPIO_Pin_9 | GPIO_Pin_10));
				xTimerStop(recue_tim_handler, 0);
			}
		}
	}
	vTaskDelete(NULL);
}

/*
 * Task indicate filling process. Suspended after creation, resumes from
 * start_filling() task and suspends again from stop_filling() task and
 * reque timer function.
 *
 * */

void led_blink(void * pvParameters)
{
	for(;;)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_9);
		vTaskDelay(50);
		GPIO_ResetBits(GPIOA, GPIO_Pin_9);
		vTaskDelay(50);
	}
	vTaskDelete(NULL);
}

/*
 * This function will be called if timer would reach programmed value. It is
 * used as a rescue routine, when no signal from higher HSI has not been reached
 * for a programmed period of time.
 *
 * */

void recue_tim_func(TimerHandle_t timer)
{
	configASSERT(timer);

	if(jar_status != FULL)
	{
		TIM_Cmd(TIM17, DISABLE);
		TIM_CtrlPWMOutputs(TIM17, DISABLE);

		GPIO_ResetBits(GPIOA, (GPIO_Pin_9));
		GPIO_SetBits(GPIOA, GPIO_Pin_10);

		xTimerStop(recue_tim_handler, 0);
		vTaskSuspend(blink_handler);
		jar_status = FULL;
	}
}

/*
 * Handler to service HSI sensors. Checks which sensor caused an interrupt,
 * then gives corresponding semaphore and calls scheduler. Since tasks, which handles
 * these interrupts have high priority, they obtain processor time immediately.
 *
 * */

void EXTI0_1_IRQHandler(void)
{
	static portBASE_TYPE fill_task = pdFALSE;
	static portBASE_TYPE full_task = pdFALSE;

	if(EXTI_GetITStatus(EXTI_Line0) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line0);
		if (xSemaphoreGiveFromISR(empty_sem, &fill_task) == pdTRUE)
		{
			portEND_SWITCHING_ISR(fill_task);
		}
	}

	else if (EXTI_GetITStatus(EXTI_Line1) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line1);

		if (xSemaphoreGiveFromISR(full_sem, &full_task) == pdTRUE)
		{
			portEND_SWITCHING_ISR(full_task);
		}
	}
}

int main(void)
{
	gpio_setup();
	interrupts_setup();
	motor_pwm_setup();

	empty_sem = xSemaphoreCreateBinary();
	configASSERT( empty_sem );
	full_sem = xSemaphoreCreateBinary();
	configASSERT( full_sem );

	xTaskCreate(led_blink, "led_bliker", configMINIMAL_STACK_SIZE, NULL, 1, &blink_handler);
	configASSERT( blink_handler );
	vTaskSuspend(blink_handler);
	xTaskCreate(start_filling, "start_filling", configMINIMAL_STACK_SIZE, NULL, 2, &start_filling_hanlder);
	configASSERT( start_filling_hanlder );
	xTaskCreate(stop_filling, "stop_filling", configMINIMAL_STACK_SIZE, NULL, 3, &stop_filling_handler);
	configASSERT( stop_filling_handler );

	recue_tim_handler = xTimerCreate("recue_timer", (1000/portTICK_PERIOD_MS), pdFALSE, (void *) 0, recue_tim_func );
	configASSERT(recue_tim_handler);

	vTaskStartScheduler();

	return 1;
}

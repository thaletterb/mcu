#include "FreeRTOSConfig.h"     // FreeRTOS header files
#include "FreeRTOS.h"
#include "task.h"

#include "stm32f10x.h"          // STM32VL headers
#include <stm32f10x_i2c.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>

int i = 0;
int off = 5;

void inc(void){
  i += off;
}

/** @brief: Gets an idle event from the scheduler
 *          #define configUSE_IDLE_HOOK             1
 **/
void vApplicationIdleHook(void)
{

}

/** @brief: Stack Overflow event handler
 *          #define configCHECK_FOR_STACK_OVERFLOW
 */
void vApplicationStackOverflowHook(void)
{

}

/** @brief: Malloc failed event handler
 *          #define configUSE_MALLOC_FAILED_HOOK    1
 */
void vApplicationMallocFailedHook(void)
{

}

/** @brief: Thread 1
 *          Toggles GPIOC Pin 9 every 300ms
 *
 */
static void Thread1(void *arg)
{
    int dir = 0;
    while(1)
    {
        vTaskDelay(300/portTICK_RATE_MS);
        GPIO_WriteBit(GPIOC, GPIO_Pin_9, dir ? Bit_SET : Bit_RESET);
        dir = 1-dir;
    }
}

/** @brief: Thread 2
 *          Toggles GPIOC Pin 8 every 500ms
 *
 */
static void Thread2(void *arg)
{
    int dir = 0;
    while(1)
    {
        vTaskDelay(500/portTICK_RATE_MS);
        GPIO_WriteBit(GPIOC, GPIO_Pin_8, dir ? Bit_SET : Bit_RESET);
        dir = 1-dir;
    }
}

int main(void)
{

    // Configure GPIOC Pin 9 as output
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable peripheral Clocks
    // Enable clocks for GPIO Port C
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // Configure Pins
    // Pin PC9 must be configured as an output
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_9|GPIO_Pin_8);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Create tasks
    //xTaskCreate(Function To Execute, Name, Stack Size, Parameter, Scheduling Priority, Storage for handle)
    xTaskCreate(Thread1, "Thread 1", 128, NULL, tskIDLE_PRIORITY+1, NULL);
    xTaskCreate(Thread2, "Thread 2", 128, NULL, tskIDLE_PRIORITY+1, NULL);

    // Start scheduler
    vTaskStartScheduler();

    // Scheduler never ends

    while (1) {
        inc();
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line){
    /* Infinite loop */
    /* Use GDB to find out why we're here */
    while(1);
}
#endif


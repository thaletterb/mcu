#include "FreeRTOSConfig.h"     // FreeRTOS header files
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "stm32f10x.h"          // STM32VL headers
#include <stm32f10x_i2c.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>

#include "my_i2c.h"
#include "my_ssd1306.h"

xSemaphoreHandle ssd1306_i2c_write_mutex;       // A mutex to control write access to the TWI bus

/** @brief: Thread 1
 *          Toggles GPIOC Pin 9 every 300ms
 */
static void Thread1(void *arg)
{
    int dir = 0;
    while(1)
    {
            vTaskDelay(300/portTICK_RATE_MS);
        if(xSemaphoreTake(ssd1306_i2c_write_mutex, 600) == pdTRUE){
            GPIO_WriteBit(GPIOC, GPIO_Pin_9, dir ? Bit_SET : Bit_RESET);
            dir = 1-dir;

            ssd1306_draw_string_to_buffer(0, 2, "Thread 1", global_display_buffer);

            ssd1306_i2c_draw_buffer(I2C1, SSD1306_I2C_SLAVE_ADDRESS8, global_display_buffer);
            xSemaphoreGive(ssd1306_i2c_write_mutex);
        }
    }
}

/** @brief: Thread 2
 *          Toggles GPIOC Pin 8 every 500ms
 */
static void Thread2(void *arg)
{
    int dir = 0;
    while(1)
    {
            vTaskDelay(500/portTICK_RATE_MS);
        if(xSemaphoreTake(ssd1306_i2c_write_mutex, 1500) == pdTRUE){
            GPIO_WriteBit(GPIOC, GPIO_Pin_8, dir ? Bit_SET : Bit_RESET);
            dir = 1-dir;
            ssd1306_draw_string_to_buffer(0, 4, "Thread 2", global_display_buffer);

            ssd1306_i2c_draw_buffer(I2C1, SSD1306_I2C_SLAVE_ADDRESS8, global_display_buffer);
            xSemaphoreGive(ssd1306_i2c_write_mutex);
        }
    }
}


/** @brief: Inits the required HW
*           GPIOC Pins 8 and 9 (LEDS)
*/
static void init_hw(void)
{
    /** Init HW */
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

    // Init I2C
    init_i2c1_peripheral(I2C1);                                               // To Do - Move All this into init_i2c1_peripheral()

    // Init SSD1306 OLED
    ssd1306_i2c_init(I2C1, SSD1306_I2C_SLAVE_ADDRESS8);
    ssd1306_i2c_draw_buffer(I2C1, SSD1306_I2C_SLAVE_ADDRESS8, global_display_buffer);
    ssd1306_i2c_write_mutex = xSemaphoreCreateMutex();

    /** End init hw */

} 

// Timer code
static __IO uint32_t TimingDelay;

void Delay(uint32_t nTime){
    TimingDelay = nTime;
    while(TimingDelay != 0);
}


int main(void)
{

    init_hw();

    ssd1306_clear_display_buffer(global_display_buffer);
    ssd1306_draw_string_to_buffer(0, 0, "FreeRTOS", global_display_buffer);

    ssd1306_i2c_draw_buffer(I2C1, SSD1306_I2C_SLAVE_ADDRESS8, global_display_buffer);

    // Create tasks
    //xTaskCreate(Function To Execute, Name, Stack Size, Parameter, Scheduling Priority, Storage for handle)
    xTaskCreate(Thread1, "Thread 1", 128, NULL, tskIDLE_PRIORITY+1, NULL);
    xTaskCreate(Thread2, "Thread 2", 128, NULL, tskIDLE_PRIORITY+1, NULL);

    // Start scheduler. Note - Scheduler never ends
    vTaskStartScheduler();
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line){
    /* Infinite loop */
    /* Use GDB to find out why we're here */
    while(1);
}
#endif

/** Functions with no current implementations **/
/** @brief: The user defined idle hook (calback function) for the idle task
 *          TODO: Implement putting the processor to low power state
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

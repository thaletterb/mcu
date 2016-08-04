#include "FreeRTOSConfig.h"     // FreeRTOS header files
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "stm32f10x.h"          // STM32VL headers
#include <stm32f10x_i2c.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_usart.h>

#include "my_i2c.h"
#include "my_ssd1306.h"
    
#define LEN 96 

uint8_t ch[LEN];                                // Global buffer to store char strings received from USART
uint8_t count = 0;

volatile uint8_t g_gps_line_ready = 0;
enum {START_WAIT, RECEIVING, MSG_RECEIVED} state = START_WAIT; 

xQueueHandle RxQueue, TxQueue;                  // Queues used for USART
xSemaphoreHandle ssd1306_i2c_write_mutex;       // A mutex to control write access to the TWI bus

/** @brief: function to clear all contents (set to 0) of the input buffer
*/
void clear_buffer(uint8_t *buffer, uint16_t buffer_length){
    uint16_t i=0;
    for(i=0; i<(buffer_length-1); i++){
        buffer[i] = 0;
    }
    buffer[i] = '\0';
}

/** @brief: function to compare to two strings
*           returns 0 if two strings are equal, else -1
*/
uint8_t string_compare(char *input, char *compare){
   while (*input == *compare) {
      if (*input == '\0' || *compare == '\0')
         break;
 
      input++;
      compare++;
   }
 
   if (*input == '\0' && *compare == '\0')
      return 0;
   else
      return -1;
}

/** Start of FreeRTOS Threads/Tasks **/

/** @brief: Thread 1
 *          Toggles GPIOC Pin 9 every 300ms
 */
static void Thread1(void *arg)
{
    #define LINE_LEN    24
    uint8_t char_buffer_truncated[LINE_LEN];
    char gpxxx_string[6];
    char time_string[7];
    uint8_t i=0;
    int dir = 0;
    static uint8_t count = 0; 
    
    while(1)
    {
        vTaskDelay(300/portTICK_RATE_MS);
        GPIO_WriteBit(GPIOC, GPIO_Pin_9, dir ? Bit_SET : Bit_RESET);
        dir = 1-dir;
        if(xSemaphoreTake(ssd1306_i2c_write_mutex, 2000) == pdTRUE){
            if(state = MSG_RECEIVED){
                
                // Read the first portion of the received message
                for(i=0; i<LINE_LEN; i++){
                    char_buffer_truncated[i] = ch[i];
                }
                char_buffer_truncated[i] = '\0'; 

                // Store the NMEA sentence type received
                for(i=0; i<5; i++){
                    gpxxx_string[i] = char_buffer_truncated[i];
                }
                gpxxx_string[i] = '\0';
                
                if(!string_compare(gpxxx_string, "GPGGA") || !string_compare(gpxxx_string, "GPRMC")){
                    // Store the UTC time string
                    // Exception is GPGSV and GPVTG 
                    // Print in the top right for now
                    for(i=0; i<6; i++){
                        time_string[i] = char_buffer_truncated[6+i];    // Time starts with the 7th char
                    }
                    time_string[i] = '\0';
                    ssd1306_draw_string_to_buffer(95, 0, time_string, global_display_buffer);

                    ssd1306_draw_string_to_buffer(0, 4, char_buffer_truncated, global_display_buffer);
                    for(i=0; i<LINE_LEN; i++){
                        char_buffer_truncated[i] = ch[LINE_LEN + i];
                    }
                    char_buffer_truncated[i] = '\0'; 
                    ssd1306_draw_string_to_buffer(0, 5, char_buffer_truncated, global_display_buffer);
                    for(i=0; i<LINE_LEN; i++){
                        char_buffer_truncated[i] = ch[2*LINE_LEN + i];
                    }
                    char_buffer_truncated[i] = '\0'; 
                    ssd1306_draw_string_to_buffer(0, 6, char_buffer_truncated, global_display_buffer);
                    for(i=0; i<LINE_LEN; i++){
                        char_buffer_truncated[i] = ch[3*LINE_LEN + i];
                    }
                    char_buffer_truncated[i] = '\0'; 
                    ssd1306_draw_string_to_buffer(0, 7, char_buffer_truncated, global_display_buffer);
                }
                    clear_buffer(ch, LEN);
                    state = START_WAIT;
                
            }
            ssd1306_draw_string_to_buffer(0, 2, "Thread 1:", global_display_buffer);
            ssd1306_draw_string_to_buffer(50, 2, gpxxx_string, global_display_buffer);
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

    char buffer[2] = "Z";
    while(1)
    {
        vTaskDelay(500/portTICK_RATE_MS);
        GPIO_WriteBit(GPIOC, GPIO_Pin_8, dir ? Bit_SET : Bit_RESET);
        dir = 1-dir;
        if(xSemaphoreTake(ssd1306_i2c_write_mutex, 1500) == pdTRUE){
            ssd1306_draw_string_to_buffer(0, 3, "Thread 2:", global_display_buffer);
            ssd1306_draw_string_to_buffer(50, 3, buffer, global_display_buffer);
            ssd1306_i2c_draw_buffer(I2C1, SSD1306_I2C_SLAVE_ADDRESS8, global_display_buffer);
            buffer[0] += 1;
            xSemaphoreGive(ssd1306_i2c_write_mutex);
        }
    }
}

//// TODO! Implementation of Queues
//static void vUSARTTask(void *pvParameters){
//    portTickType xLastWakeTime;
//    const portTickType xFrequency = 50;
//    xLastWakeTime=xTaskGetTickCount();
//    char ch;
//    char buffer[2] = "A";
//    //RxQueue = xQueueCreate( 128, sizeof( portCHAR ) );
//    if(RxQueue == NULL){
//        // Die
//        GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
//        GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
// 
//    }
//    TxQueue = xQueueCreate( 64, sizeof( portCHAR ) );
//    while(1)
//    {
//        if(Usart1GetChar(&ch)){
//            buffer[0] = ch;
//            ssd1306_draw_string_to_buffer(0, 6, buffer, global_display_buffer); 
//        }
//        vTaskDelayUntil(&xLastWakeTime,xFrequency);
//    }      
//}

/** End of FreeRTOS Threads/Tasks **/

void usart1_init(void){
// Init the USART1 peripheral

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);

    USART_InitTypeDef USART_InitStructure;
    USART_StructInit(&USART_InitStructure);

    // Enable the clocks to USART1, Alternate Function and GPIO Peripherals
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);

    // Configure the GPIO PIns. Pin 9 - TX, Pin 10 - RX
    // Initialize USART1_tx
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    // Initialize USART1_RX
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Initialize and Configure the USART1 Peripheral
    //USART_InitStructure.USART_BaudRate = 115200; 
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

    // Enabled RXNE Interrupt
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    // Enable USART1 global interrupt
    NVIC_EnableIRQ(USART1_IRQn);

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

    // USART1
    usart1_init();
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
    ssd1306_i2c_write_mutex = xSemaphoreCreateMutex();
    RxQueue = xQueueCreate( 128, sizeof( portCHAR ) );
    init_hw();

    ssd1306_clear_display_buffer(global_display_buffer);
    ssd1306_draw_string_to_buffer(0, 0, "FreeRTOS", global_display_buffer);
    ssd1306_i2c_draw_buffer(I2C1, SSD1306_I2C_SLAVE_ADDRESS8, global_display_buffer);

    // Create tasks
    //xTaskCreate(Function To Execute, Name, Stack Size, Parameter, Scheduling Priority, Storage for handle)
    xTaskCreate(Thread1, "Thread 1", 128, NULL, tskIDLE_PRIORITY+1, NULL);
    xTaskCreate(Thread2, "Thread 2", 128, NULL, tskIDLE_PRIORITY+2, NULL);
    //xTaskCreate(vUSARTTask, "vUSART Task", 128, NULL, tskIDLE_PRIORITY, NULL);    // TODO!!

    // Start scheduler. Note - Scheduler never ends
    vTaskStartScheduler();
}


/** FreeRTOS Functions with no current implementations **/

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

/** End FreeRTOS Functions with no current implementations **/

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line){
    /* Infinite loop */
    /* Use GDB to find out why we're here */
    while(1);
}
#endif

/** @brief: USART1 IRQ Handler
*/ 
void USART1_IRQHandler(void)
{
    //static uint8_t count = 0;
    static uint8_t newline = 0;
    uint8_t temp;
    long xHigherPriorityTaskWoken = pdFALSE;

    //if Receive interrupt
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        
        temp=(uint8_t)USART_ReceiveData(USART1);
        switch(state)
        {
           case START_WAIT:               // Waiting for start of message
              if(temp =='$')                         // Start character received...
              {
                 count = 0;                     // reset the data buffer index...
                 state = RECEIVING;        // and start receiving data
              }
              break;
      
           case RECEIVING:                   // Message Start received
              if(temp=='*')                          // If end of message...
             {
                 ch[count] = '\0';
                 state = MSG_RECEIVED;  // indicate we're done - don't process any more character until 
              }                                     // done processing and next start is received
                 else
                 {
                    ch[count] = temp;      // Save the character in our buffer
                    if (++count >= LEN)   // If we overran our data buffer...
                      //state = START_WAIT;   // wait for the next message
                      state = MSG_RECEIVED;   // wait for the next message
                }
                break;
        } 
    } 
  
        
   //if(xQueueIsQueueFullFromISR(RxQueue) == pdFALSE)          TODO!!! 
   //{
   //    ssd1306_draw_string_to_buffer(100, 0, ch, global_display_buffer);
   //    //xQueueSendToBackFromISR( RxQueue, &ch, &xHigherPriorityTaskWoken );
   //}

    // Blink the LED to make sure we get into IRQ handler
    static int dir = 0;
    GPIO_WriteBit(GPIOC, GPIO_Pin_9, dir ? Bit_SET : Bit_RESET);
    dir = 1 - dir;
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

}

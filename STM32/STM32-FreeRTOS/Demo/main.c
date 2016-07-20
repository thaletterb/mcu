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


xSemaphoreHandle ssd1306_i2c_write_mutex;       // A mutex to control write access to the TWI bus


/** @brief: Thread 1
 *          Toggles GPIOC Pin 9 every 300ms
 */
static void Thread1(void *arg)
{
    int dir = 0;
    static uint8_t count = 0; 
    
    char buffer[2] = "A";
    while(1)
    {
        vTaskDelay(300/portTICK_RATE_MS);
        GPIO_WriteBit(GPIOC, GPIO_Pin_9, dir ? Bit_SET : Bit_RESET);
        dir = 1-dir;
        if(xSemaphoreTake(ssd1306_i2c_write_mutex, 2000) == pdTRUE){
            //char buffer[2];
            //buffer[1] = '\0';
            //&buffer+1 = '\0';
            ssd1306_draw_string_to_buffer(0, 2, "Thread 1:", global_display_buffer);
            ssd1306_draw_string_to_buffer(50, 2, buffer, global_display_buffer);
            buffer[0] += 1;
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
            ssd1306_draw_string_to_buffer(0, 4, "Thread 2:", global_display_buffer);
            ssd1306_draw_string_to_buffer(50, 4, buffer, global_display_buffer);
            ssd1306_i2c_draw_buffer(I2C1, SSD1306_I2C_SLAVE_ADDRESS8, global_display_buffer);
            buffer[0] += 1;
            xSemaphoreGive(ssd1306_i2c_write_mutex);
        }
    }
}



//xQueueHandle RxQueue, TxQueue;   // Queue for USART RX
xQueueHandle RxQueue, TxQueue;   // Queue for USART RX

uint32_t Usart1GetChar(char *ch){
  if(xQueueReceive( RxQueue, ch, 0 ) == pdPASS)
    {
      return pdTRUE;
    }
  return pdFALSE;
}

static void vUSARTTask(void *pvParameters){
    portTickType xLastWakeTime;
    const portTickType xFrequency = 50;
    xLastWakeTime=xTaskGetTickCount();
    char ch;
    char buffer[2] = "A";
    // Create a queue capable of containing 128 characters.
    //RxQueue = xQueueCreate( configCOM0_RX_BUFFER_LENGTH, sizeof( portCHAR ) );
    RxQueue = xQueueCreate( 128, sizeof( portCHAR ) );
    if(RxQueue == NULL){
        // Die
        GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
        GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
 
    }
    TxQueue = xQueueCreate( 64, sizeof( portCHAR ) );
    while(1)
    {
        if(Usart1GetChar(&ch)){
            buffer[0] = ch;
            ssd1306_draw_string_to_buffer(0, 6, buffer, global_display_buffer); 
        }
        vTaskDelayUntil(&xLastWakeTime,xFrequency);
    }      
}

static void vLEDFlashTask(void *pvParameters){
  portTickType xLastWakeTime;
  const portTickType xFrequency = 1000;
  xLastWakeTime=xTaskGetTickCount();
  static int dir = 1;
    for( ;; )
    {
        GPIO_WriteBit(GPIOC, GPIO_Pin_8, dir ? Bit_SET : Bit_RESET);
        dir = 1-dir;
      vTaskDelayUntil(&xLastWakeTime,xFrequency);
    }
}
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

void USART_PutChar(uint8_t ch)
// Puts char ch into the USART data register for Tx
{
    while(!(USART1->SR & USART_SR_TXE));        // & TX Register Empty
    //USART1->DR = ch;                            // Put char 'ch' into data register
    USART_SendData(USART1, ch);
}

int USART_GetChar(void){
// Gets char 'ch' from the USART data register for rx

    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET);
    //return  USART1->DR & 0xff;
    USART_ClearFlag(USART1, USART_FLAG_RXNE);
    return  0xFF & USART_ReceiveData(USART1);

}

void USART_PutString(uint8_t * str)
{
    while(*str != 0)
    {
        USART_PutChar(*str);
        str++;
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

    //// Init TSL2561
    //#define TSL2561_I2C_SLAVE_ADDRESS7  0x39    // 0x39 is 7bit if ADDR is left floating
    //uint8_t tsl2561_8bit_address = (TSL2561_I2C_SLAVE_ADDRESS7 << 1);
    ////uint8_t tsl2561_8bit_address = 0x73;
    //uint8_t tsl2561_cmd_reg_val = 0;

    //tsl2561_cmd_reg_val |= (1<<7) | (0x00);            // Set the CMD byte to 1 and addr to 0x00 in the Command Register
    //uint8_t ret_val = i2c_send_command_new(I2C1, tsl2561_8bit_address, 0x03, tsl2561_cmd_reg_val);
    //if(ret_val)
    //{
    //    while(1){}  // If failed - Loop here forever
    //}
    //i2c_read_reg_new(I2C1, tsl2561_8bit_address, 0x00, tsl2561_cmd_reg_val);

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
    init_hw();

    ssd1306_clear_display_buffer(global_display_buffer);
    ssd1306_draw_string_to_buffer(0, 0, "FreeRTOS", global_display_buffer);
    ssd1306_i2c_draw_buffer(I2C1, SSD1306_I2C_SLAVE_ADDRESS8, global_display_buffer);

    //while(1){
    //    ssd1306_i2c_draw_buffer(I2C1, SSD1306_I2C_SLAVE_ADDRESS8, global_display_buffer);
    //}


    // Create tasks
    //xTaskCreate(Function To Execute, Name, Stack Size, Parameter, Scheduling Priority, Storage for handle)
    xTaskCreate(Thread1, "Thread 1", 128, NULL, tskIDLE_PRIORITY+1, NULL);
    xTaskCreate(Thread2, "Thread 2", 128, NULL, tskIDLE_PRIORITY+2, NULL);
    //xTaskCreate(vLEDFlashTask, "LEDFlashTask", 128, NULL, tskIDLE_PRIORITY+2, NULL);
    xTaskCreate(vUSARTTask, "vUSART Task", 256, NULL, tskIDLE_PRIORITY, NULL);

    //// Start scheduler. Note - Scheduler never ends
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


void USART1_IRQHandler(void)
{
    long xHigherPriorityTaskWoken = pdFALSE;
    uint8_t ch;
    //if Receive interrupt
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        ch=(uint8_t)USART_ReceiveData(USART1);
        //xQueueSendToBackFromISR( RxQueue, &ch, &xHigherPriorityTaskWoken );
    }
    if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        if( xQueueReceiveFromISR( TxQueue, &ch, &xHigherPriorityTaskWoken ) )
        {
             USART_SendData(USART1, ch);
        }
        else
        {
             //disable Transmit Data Register empty interrupt
             USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }
    }
    static int dir = 0;
    GPIO_WriteBit(GPIOC, GPIO_Pin_9, dir ? Bit_SET : Bit_RESET);
    dir = 1 - dir;
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

    //#define RX_BUFF_SIZE    5
    //static int dir = 0;
    //static int buf_count = 0;
    //char rx_data = 0;
    //char rx_buff[20];
    ////rx_buff[0] = 'A';
    ////rx_buff[1] = 'B';
    ////rx_buff[2] = 'C';
    ////rx_buff[3] = 'D';

    //char test_buff;
    //static uint8_t i = 0;

    ///* RXNE (not empty) handler */
    //if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    //{
    //    GPIO_WriteBit(GPIOC, GPIO_Pin_9, dir ? Bit_SET : Bit_RESET);
    //    dir = 1 - dir;
    //
    //    rx_data = (char)USART_ReceiveData(USART1);
    //    rx_buff[i++] = 'A';
    //    test_buff = 'A';
    //    ssd1306_draw_string_to_buffer(0, 1, "IRQ", global_display_buffer);

    //    //    /* Wait until Tx data register is empty, not really 
    //    //     * required for this example but put in here anyway.
    //    //     */
    //    //    /*
    //    //    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    //    //    {
    //    //    }*/
    //    //
    //}

    ///* ------------------------------------------------------------ */
    ///* Other USART1 interrupts handler can go here ...             */
}


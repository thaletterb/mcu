#include "stm32f10x.h"
#include <stm32f10x_i2c.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>

#include <stdio.h>
#include <stdlib.h>

#include "myFont.h"
//#include "math.h"


#define I2C_SPEED                   50000                     /*!< I2C Speed */
#define I2C1_SSD1306_SLAVE_ADDRESS8 0x78                      // 8 bit slave address (write)
#define I2C_TIMEOUT                 100000

typedef struct{
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
}system_time_t;

typedef enum{
    clock_state,
    pattern_state,
} my_states_t;

typedef struct{
    my_states_t current_state;
    system_time_t current_system_time;
}system_status_t; 

system_time_t my_system_time;
system_status_t my_system_status;

void init_system_time(system_time_t time_struct){
    time_struct.hours = 12;
    time_struct.minutes = 0;
    time_struct.seconds = 0;
}

// Globals
uint8_t current_button_status = 0;
uint8_t global_display_buffer[(128*64)/8] = {

    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,        // Page 0 (Top Most)
    0x80,0xC0,0x40,0x60,0x20,0x20,0x20,0x20,0x60,0xC0,0x80,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0xF8,0x7C,0x86,
    0xF6,0x1E,0x04,0x0C,0xF8,0xE0,0xC0,0x30,0x88,0xCC,0x24,0x26,0x06,0x06,0xFC,0xF0,
    0x00,0x00,0x80,0xC0,0x80,0xC0,0xC0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

    0x00,0x00,0x00,0x00,0x00,0x60,0xFC,0x0C,0x7A,0x0F,0x81,0x03,0x02,0x06,0x0C,0x1C,
    0x07,0x00,0xC0,0x43,0x01,0x01,0x03,0x04,0x00,0x21,0x3F,0x1E,0x18,0x08,0x0C,0x08,
    0x0C,0xC8,0x18,0x18,0x70,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x80,0xC0,0x60,0xE0,0xC0,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x3C,0xF6,0xC2,0xA3,0x61,0xC1,0xC1,0x81,0x03,0x03,0x06,0x1F,0x3A,0x21,
    0x81,0x00,0x00,0x00,0x07,0x7F,0xFF,0x1E,0x02,0x00,0x00,0xE0,0x70,0x3E,0x0F,0x06,
    0x83,0x81,0xC1,0x60,0x60,0x70,0x78,0x39,0x27,0x73,0x20,0x60,0xC0,0xC0,0x00,0x00,

    0x00,0x00,0x00,0x00,0x80,0xC0,0x31,0x1B,0x0E,0x0C,0x1C,0x00,0x18,0x10,0x30,0x30,
    0x00,0x00,0x83,0xFE,0x3C,0x18,0x18,0x70,0xE0,0x30,0x00,0x00,0x18,0x00,0x00,0x08,
    0x08,0x00,0x0C,0x4E,0xDB,0x30,0x30,0x60,0xC0,0x80,0x00,0x00,0x00,0xC0,0xA0,0x98,
    0xA4,0xB4,0xAC,0x08,0x30,0x30,0xC0,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0xD0,0x38,
    0x0E,0x0E,0x07,0x07,0x00,0x00,0x00,0x0F,0x07,0x02,0x33,0x99,0xEF,0x0B,0x00,0x00,
    0x00,0x80,0xC0,0x60,0x30,0x19,0x0B,0x0B,0x0E,0x06,0x0E,0x44,0x06,0x06,0x06,0x86,
    0x84,0x8C,0xEC,0xF8,0x0C,0xE6,0xB7,0x86,0x04,0x0C,0x79,0xF8,0x88,0x0C,0x02,0x86,
    0x05,0x00,0x00,0x02,0xC2,0xE1,0xE1,0xE1,0xA3,0xB2,0x30,0x10,0x18,0x1F,0x07,0x00,

    0x00,0x00,0x00,0x00,0x0F,0x39,0x70,0x60,0xC0,0xC0,0xC0,0xE0,0x00,0x00,0x80,0x10,
    0x18,0x0C,0x06,0x07,0x06,0x04,0x7E,0x67,0x07,0x06,0x04,0x0C,0x0C,0x0C,0x48,0x40,
    0xE0,0x72,0x60,0x62,0x23,0x31,0x30,0x18,0x0F,0x03,0x00,0x00,0x01,0x07,0x01,0x01,
    0x00,0x00,0x01,0x03,0x06,0x9C,0x60,0x01,0x06,0xDC,0x70,0x00,0x00,0xED,0xBF,0xE0,
    0xE0,0xF0,0x70,0x68,0x38,0x1C,0x16,0x0C,0x0D,0x06,0x02,0x03,0x00,0x00,0x00,0x00,
    0x0E,0x1F,0x18,0x30,0x31,0x33,0xF2,0x7A,0x19,0x1C,0x0C,0x8C,0x86,0x02,0x21,0x01,
    0x00,0x90,0x19,0x07,0x06,0x9C,0xF8,0x18,0x18,0x1C,0x1E,0x33,0xE1,0x81,0x01,0x03,
    0x07,0x06,0x0E,0x0C,0x18,0x70,0xE0,0x80,0x80,0x81,0x83,0x83,0xC6,0x7E,0x38,0x00,

    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x39,0x30,0x30,0x21,0x30,
    0x30,0x30,0x30,0x20,0x30,0x1E,0x1E,0x3E,0xF8,0xF0,0xF4,0xF4,0xF4,0xF6,0xB0,0x1C,
    0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0xE2,0x5F,0x9A,0xAA,0x5F,0xC1,0xF8,0xFB,0x4E,0x23,0x8F,
    0xC3,0x60,0x30,0x90,0xD8,0xC8,0xEC,0xA4,0xE4,0xA6,0xE4,0x8C,0x8C,0x98,0x30,0xE0,
    0x00,0x00,0x00,0x00,0x00,0x0F,0x1C,0x30,0x31,0x13,0x19,0x0D,0xAC,0xFE,0x07,0x01,
    0x01,0x00,0x00,0x00,0xE0,0xFE,0xED,0x80,0x00,0x00,0x00,0x00,0x00,0x07,0x7F,0xFE,
    0x18,0x18,0x30,0x30,0x20,0x30,0x19,0x1F,0x05,0x01,0x01,0x01,0x00,0x00,0x00,0x00,

    0x00,0x00,0x00,0x80,0xC0,0x60,0x60,0x20,0x30,0x20,0x20,0x30,0x20,0x30,0x20,0x20,
    0x20,0x60,0x60,0xC0,0x00,0x00,0x00,0x00,0x00,0x03,0xFF,0xFF,0x00,0xEB,0x7F,0x00,
    0x00,0x80,0xC0,0x40,0x60,0x20,0x90,0x18,0x08,0x08,0x88,0xCC,0x6C,0x34,0x1C,0x08,
    0x00,0x00,0x00,0x00,0xEC,0xBF,0x01,0xF1,0xFF,0xFE,0xFF,0xBB,0x7E,0x1E,0x0E,0x05,
    0x02,0x01,0x01,0x01,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x01,0x03,0x07,0x07,
    0x00,0x00,0x00,0x70,0xE0,0xC0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x03,0x07,0x04,
    0x04,0x06,0x02,0x03,0x00,0x0F,0xFF,0x9F,0x3F,0xFE,0x0C,0x06,0x02,0x02,0x03,0x01,
    0x00,0x00,0x00,0x00,0x00,0x80,0xC0,0xE0,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

    0x00,0x03,0x01,0x03,0x03,0x03,0x03,0x03,0x02,0x06,0x06,0x06,0x0C,0x18,0x18,0x70,
    0xEC,0x9E,0x9E,0x7C,0xF9,0xB3,0xC6,0x8C,0x38,0xE0,0xFF,0x81,0xF8,0x7F,0x98,0xC6,
    0xE3,0x78,0x3C,0x1C,0x8E,0xC3,0x61,0x18,0x0C,0x03,0x01,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x05,0x1F,0xFC,0xFF,0xCF,0x39,0x07,0x01,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x1F,0x78,0xC1,0xC1,0x01,0x0B,0x33,0x12,0x42,0x42,0x46,0xC6,
    0xC6,0x84,0x0C,0x18,0x30,0xE0,0xD9,0xFB,0xFA,0xEF,0x60,0x30,0x10,0x18,0x18,0x8C,
    0x0C,0xE4,0x06,0x06,0x03,0xE3,0x3F,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,        // Page 7 (bottom)
    0x00,0x00,0x03,0x03,0x0F,0x0D,0x1B,0x1B,0x33,0x32,0x37,0x2F,0x33,0x32,0x13,0x11,
    0x18,0x0C,0x06,0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x0F,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x07,0x06,0x0C,0x08,0x18,0x30,0x30,
    0x60,0x60,0x61,0xE6,0xEC,0x7E,0x7F,0x47,0x9B,0xCC,0x84,0x86,0xC2,0x43,0x41,0x61,
    0x20,0x30,0x1C,0x06,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

};



// Function Prototypes
void Delay(uint32_t nTime);


/* Private functions ---------------------------------------------------------*/

static void init_timer3(void){
// Initialize and enable interrupts on timer 3 every 1 second
// 1) Initiailze Timer 3
// 2) Enable Interrupt
// 3) Enable Timer
// 4) Enable interrupts:w


    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;                  // Timer init structure    

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);            // Enable the clocks to Tim3 periph

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/1000 - 1; // Prescaler 
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;                    // Auto Reload Register ARR (0..999)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     // Count up
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);                 // Enable timer

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);                      // Enable interrupt

    TIM_Cmd(TIM3, ENABLE);                                          // Enable timer

    NVIC_EnableIRQ(TIM3_IRQn);                                      // Enable Interrupts

}

void TIM3_IRQHandler(void){
// Timer 3 Interrupt Handler
    #define LED_BLUE_GPIO GPIOC
    #define LED_BLUE_PIN 8

    // Update the time values
    my_system_time.seconds++;
    if(my_system_time.seconds > 59){
        my_system_time.seconds = 0;
        my_system_time.minutes++;
    }
    if(my_system_time.minutes > 59){
        my_system_time.minutes = 0;
        my_system_time.hours++;
    }
    if(my_system_time.hours > 12){
        my_system_time.hours = 1;
    }
    
    // Update display
    switch(my_system_status.current_state){
        case clock_state:
            //GPIO_WriteBit(GPIOC, GPIO_Pin_9, 1);
            break;
        case pattern_state:
            //GPIO_WriteBit(GPIOC, GPIO_Pin_9, 0);
            break;
    }
                

    // Toggle the LEDs
    if (TIM_GetFlagStatus(TIM3, TIM_FLAG_Update) != RESET) {
        static int led_val = 0;
        TIM_ClearFlag(TIM3, TIM_IT_Update);
        GPIO_WriteBit(GPIOC, GPIO_Pin_8, 0);       // blue LED
        //GPIO_WriteBit(GPIOC, GPIO_Pin_8, led_val ? 1 : 0);       // blue LED
        led_val = 1-led_val;
    }

}

void init_error_led_pin(void){
    // Green LED is on PC9, Blue LED is on PC8
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable peripheral Clocks
    // Enable clocks for GPIO Port C
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // Configure Pins
    // Pin PC9 must be configured as an output
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void turn_on_error_led_pin(void){
    GPIO_WriteBit(GPIOC, GPIO_Pin_9, 1);
    //GPIO_WriteBit(GPIOC, GPIO_Pin_8, 1);
}
void turn_off_error_led_pin(void){
    GPIO_WriteBit(GPIOC, GPIO_Pin_9, 0);
}

void init_user_button(void){
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable peripheral clocks to GPIO Port A
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // Configure Button - Attached to PA0
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

    GPIO_Init(GPIOA, &GPIO_InitStructure);

}

uint8_t read_user_button_state(void){
    return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
}

int i2c_send_command(I2C_TypeDef *I2Cx, uint8_t slave_address, uint8_t slave_data){
// Sends I2C data over I2Cx:
//  1) Sends Start Condition. Checks for I2C EV5 
//  2) Sends 7 bit address & checks for EV6
//  3) Sends 8 bit command byte (0x00) & checks for EV8
//  4) Sends 8 bits (1 byte) of data & checks for EV8
//  5) Sends Stop Condition
    int TimeOut;

    #define COMMAND_BYTE 0x00

    /* Send I2C1 START condition */
    I2C_GenerateSTART(I2Cx, ENABLE);

    /* Test on I2C1 EV5 and clear it */
    TimeOut = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
    {
        TimeOut--;
        if (TimeOut == 0){
            turn_on_error_led_pin();       // Error LED
            return 1;
        }
    }

    /* Send SSD1306 7 bit slave Address for write. Check to make sure ACK received */
    I2C_Send7bitAddress(I2Cx, I2C1_SSD1306_SLAVE_ADDRESS8, I2C_Direction_Transmitter);

    //Test on I2C1 EV6 and clear it
    TimeOut = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        TimeOut--;
        if (TimeOut == 0){
            // Send I2C1 STOP Condition
            I2C_GenerateSTOP(I2Cx, ENABLE);
            turn_on_error_led_pin();        // Error LED

            return 2;
        }
    }
    I2C_SendData(I2Cx, COMMAND_BYTE);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){ // Wait for EV8
        TimeOut--;
        if (TimeOut == 0){
            // Send I2C1 STOP Condition
            I2C_GenerateSTOP(I2Cx, ENABLE);
            turn_on_error_led_pin();        // Error LED

            return 2;
        }
    }
    I2C_SendData(I2Cx, slave_data);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){ // Wait for EV8
        TimeOut--;
        if (TimeOut == 0){
            // Send I2C1 STOP Condition
            I2C_GenerateSTOP(I2Cx, ENABLE);
            turn_on_error_led_pin();        // Error LED

            return 2;
        }
    }
    I2C_GenerateSTOP(I2Cx, ENABLE);
}

void ssd1306_i2c_init(I2C_TypeDef *I2Cx, uint8_t ssd1306_slave_address){
// Sends the init commands to the display
    i2c_send_command(I2Cx, ssd1306_slave_address, 0xAE);

    i2c_send_command(I2Cx, ssd1306_slave_address, 0x00 | 0x0);      // low col = 0

    i2c_send_command(I2Cx, ssd1306_slave_address, 0x10 | 0x0);      // hi col = 0
    i2c_send_command(I2Cx, ssd1306_slave_address, 0x40 | 0x0);      // line #0

    i2c_send_command(I2Cx, ssd1306_slave_address, 0x81);            // Set Contrast 0x81
    i2c_send_command(I2Cx, ssd1306_slave_address, 0xCF);
    i2c_send_command(I2Cx, ssd1306_slave_address, 0xA1);            // Segremap - 0xA1
    i2c_send_command(I2Cx, ssd1306_slave_address, 0xC8);            // COMSCAN DEC 0xC8 C0
    i2c_send_command(I2Cx, ssd1306_slave_address, 0xA6);            // Normal Display 0xA6 (Invert A7)

    i2c_send_command(I2Cx, ssd1306_slave_address, 0xA4);            // DISPLAY ALL ON RESUME - 0xA4
    i2c_send_command(I2Cx, ssd1306_slave_address, 0xA8);            // Set Multiplex 0xA8
    i2c_send_command(I2Cx, ssd1306_slave_address, 0x3F);            // 1/64 Duty Cycle 

    i2c_send_command(I2Cx, ssd1306_slave_address, 0xD3);            // Set Display Offset 0xD3
    i2c_send_command(I2Cx, ssd1306_slave_address, 0x0);             // no offset

    i2c_send_command(I2Cx, ssd1306_slave_address, 0xD5);            // Set Display Clk Div 0xD5
    i2c_send_command(I2Cx, ssd1306_slave_address, 0x80);            // Recommneded resistor ratio 0x80

    i2c_send_command(I2Cx, ssd1306_slave_address, 0xD9);            // Set Precharge 0xd9
    i2c_send_command(I2Cx, ssd1306_slave_address, 0xF1);

    i2c_send_command(I2Cx, ssd1306_slave_address, 0xDA);            // Set COM Pins0xDA
    i2c_send_command(I2Cx, ssd1306_slave_address, 0x12);

    i2c_send_command(I2Cx, ssd1306_slave_address, 0xDB);            // Set VCOM Detect - 0xDB
    i2c_send_command(I2Cx, ssd1306_slave_address, 0x40);

    i2c_send_command(I2Cx, ssd1306_slave_address, 0x20);            // Set Memory Addressing Mode
    i2c_send_command(I2Cx, ssd1306_slave_address, 0x00);            // 0x00 - Horizontal

    i2c_send_command(I2Cx, ssd1306_slave_address, 0x40 | 0x0);      // Set start line at line 0 - 0x40 

    i2c_send_command(I2Cx, ssd1306_slave_address, 0x8D);            // Charge Pump -0x8D
    i2c_send_command(I2Cx, ssd1306_slave_address, 0x14);


    i2c_send_command(I2Cx, ssd1306_slave_address, 0xA4);            //--turn on all pixels - A5. Regular mode A4
    i2c_send_command(I2Cx, ssd1306_slave_address, 0xAF);            //--turn on oled panel - AF
}

int ssd1306_i2c_draw_buffer(I2C_TypeDef *I2Cx, uint8_t slave_address, uint8_t *buffer_pointer){
    #define SSD1306_COLUMNADDR  0x21
    #define SSD1306_PAGEADDR    0x22
    #define DATA_BYTE           0x40

    int TimeOut;

    i2c_send_command(I2Cx, slave_address, SSD1306_COLUMNADDR);
    i2c_send_command(I2Cx, slave_address, 0x00);            // Column Start address
    i2c_send_command(I2Cx, slave_address, 127);             // Column end address

    i2c_send_command(I2Cx, slave_address, SSD1306_PAGEADDR);
    i2c_send_command(I2Cx, slave_address, 0x00);            // Page Start address
    i2c_send_command(I2Cx, slave_address, 0x07);            // Page end address

    uint16_t i=0;
    uint8_t x, y;


    /* Send I2C1 START condition */
    I2C_GenerateSTART(I2Cx, ENABLE);

    /* Test on I2C1 EV5 and clear it */
    TimeOut = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
    {
        TimeOut--;
        if (TimeOut == 0){
            turn_on_error_led_pin();       // Error LED
            return 1;
        }
    }

    /* Send SSD1306 7 bit slave Address for write. Check to make sure ACK received */
    I2C_Send7bitAddress(I2Cx, slave_address, I2C_Direction_Transmitter);

    //Test on I2C1 EV6 and clear it
    TimeOut = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        TimeOut--;
        if (TimeOut == 0){
            // Send I2C1 STOP Condition
            I2C_GenerateSTOP(I2Cx, ENABLE);
            turn_on_error_led_pin();        // Error LED

            return 2;
        }
    }

    I2C_SendData(I2Cx, DATA_BYTE);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){ // Wait for EV8
        TimeOut--;
        if (TimeOut == 0){
            // Send I2C1 STOP Condition
            I2C_GenerateSTOP(I2Cx, ENABLE);
            turn_on_error_led_pin();        // Error LED

            return 2;
        }
    }

    for(y=0; y<8; y++){
        for(x=0; x<128; x++){
            //I2C_SendData(I2Cx, const_global_display_buffer[(128*y)+x]);
            I2C_SendData(I2Cx, buffer_pointer[(128*y)+x]);
            while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){ // Wait for EV8
                TimeOut--;
                if (TimeOut == 0){
                    // Send I2C1 STOP Condition
                    I2C_GenerateSTOP(I2Cx, ENABLE);
                    turn_on_error_led_pin();        // Error LED

                    return 2;
                }
            }
        }
    }
    I2C_GenerateSTOP(I2Cx, ENABLE);
}

void ssd1306_clear_display_buffer(uint8_t *buffer_pointer){
// Clear all bytes of the 128x64 display buffer given by buffer_pointer
    uint8_t x, page_index = 0;
    #define NUM_PAGES       8
    #define SSD1306_XRES    128

    for(page_index=0; page_index < NUM_PAGES; page_index++){
        for(x=0; x<SSD1306_XRES; x++){
            buffer_pointer[(128*page_index)+x] = 0x00;
        }
    }
}

void ssd1306_draw_pixel_to_buffer(uint8_t x, uint8_t y, uint8_t* buffer_pointer){
// Turns on the pixel at (x,y) in the local buffer
    #define SSD1306_HORIZONTAL_RES  128

    uint8_t which_bit, which_page, bit_mask, value = 0;
    uint16_t which_byte; 

    which_page = y / 8;             // There are 8 vertical pixels per page
    which_byte = x + (SSD1306_HORIZONTAL_RES * which_page);
    which_bit = y % 8;

    bit_mask = (1<<which_bit);
    
    buffer_pointer[which_byte] = (buffer_pointer[which_byte] & ~bit_mask) | ((1<<which_bit) & bit_mask);
}

uint32_t isqrt32(uint32_t n){
    uint32_t root, remainder, place;

    root = 0;
    remainder = n;
    place = 0x40000000;

    while(place > remainder)
        place = place >> 2;
    while(place){
        if(remainder >= root + place){
            remainder = remainder - root - place;
            root = root + (place << 1);
        }
        root = root >> 1;
        place = place >> 2;
    }
    return root;

}

void ssd1306_draw_circle_to_buffer(uint8_t x, uint8_t y, uint8_t radius, uint8_t* buffer_pointer){
// Draws a circle of radius centered at (x,y) 
    uint16_t x_square, y_square, radius_square;
    uint8_t i=0;
    uint8_t y_calc = 0;

    x_square = x * x;
    radius_square = radius * radius;

    // General equation of circle
    // (x-h)^2 + (y-k)^2 = r^2
    // (y-k)^2 = r^2 - (x-h)^2
    // y-k = sqrt(r^2 - (x-h)^2)
    // y = sqrt(r^2 - (x-h)^2) + k

    // (h, k) origin

    for(i=x-radius; i<=x+radius; i++){
        y_calc = isqrt32(radius_square - (i-x)*(i-x)) + y;
        ssd1306_draw_pixel_to_buffer(i, y_calc, global_display_buffer); 
        y_calc = -1 * isqrt32(radius_square - (i-x)*(i-x)) + y;
        ssd1306_draw_pixel_to_buffer(i, y_calc, global_display_buffer); 
    }
}

void ssd1306_draw_char_to_buffer(uint8_t x, uint8_t page_num, uint8_t which_char, uint8_t *buffer_pointer){
// Draws 'which_char' to the display buffer given by buffer_pointer at coord (x, page_num)

    uint8_t i;
    uint16_t which_byte = 0;

    //GPIO_WriteBit(GPIOC, GPIO_Pin_9, 0);       // Error LED

    //which_byte = x + ((page_num/8)*128);
    which_byte = x + ((page_num)*128);
    
    for(i=0; i<5; i++){
        buffer_pointer[which_byte+i] = Ascii_8x5_font[which_char-ASCII_8x5_OFFSET][i];
        //turn_on_error_led_pin();       // Error LED
    }

}
void ssd1306_draw_big_char_to_buffer(uint8_t x, uint8_t page_num, uint8_t which_char, uint8_t *buffer_pointer){
// Draws 'which_char' to the display buffer given by buffer_pointer at coord (x, page_num)
    uint8_t i, j;
    uint16_t which_byte = 0;

    which_byte = x + ((page_num)*128);
    
    for(j=0; j<5; j++){
    // 40 bits (5 bytes) tall
        which_byte = x + ((page_num+j)*128);
        for(i=0; i<24; i++){
        // 24 bits wide
            buffer_pointer[which_byte+i] = Tahoma24x40[which_char-ASCII_24x40_FONT_ASCII_OFFSET][(24*j)+i];
            //turn_on_error_led_pin();       // Error LED
        }
    }

}

void ssd1306_draw_string_to_buffer(uint8_t x, uint8_t page_num, uint8_t *string, uint8_t *buffer_pointer){
// Draws 'string' to the display buffer (buffer pointer) at coord (x, page_num)

    uint8_t i=0;
    while(string[i] != 0){
        ssd1306_draw_char_to_buffer(x+(5*i), page_num, string[i], buffer_pointer);
        i++;
    }
}

void ssd1306_draw_big_string_to_buffer(uint8_t x, uint8_t page_num, uint8_t *string, uint8_t *buffer_pointer){
// Draws 'string' to the display buffer (buffer pointer) at coord (x, page_num)

    uint8_t i=0;
    while(string[i] != 0){
        ssd1306_draw_big_char_to_buffer(x+(ASCII_24x40_FONT_WIDTH*i), page_num, string[i], buffer_pointer);
        i++;
    }
}


void init_i2c1_peripheral(I2C_TypeDef *I2Cx){
// Initializes the I2C1 Peripheral on PB6 & PB7
// 1) Enables the GPIOB Peripheral Clock
// 2) Enable the I2C1 Peripheral Clock
// 3) Configure the GPIO peripheral with GPIO_InitStructure and GPIO_Init()
// 4) Configure the I2C1 peripheral with I2C_InitStructure and I2C_Init() 

    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    /* GPIOB Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* I2C1 and I2C2 Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    /* Configure I2C1 pins: SCL and SDA ----------------------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; // Open Drain, I2C bus pulled high externally
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Enable I2C1 -------------------------------------------------------------*/
    I2C_DeInit(I2C1);

    I2C_Cmd(I2C1, ENABLE);

    /* I2C1 configuration ------------------------------------------------------*/
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x039;                                  // This is important for some reason!
    //I2C_InitStructure.I2C_OwnAddress1 = 0x000;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
    I2C_Init(I2C1, &I2C_InitStructure);
}



int main(void){
    int TimeOut;
    const uint8_t *global_buffer_ptr = global_display_buffer;

    init_system_time(my_system_time);
    init_error_led_pin();
    turn_off_error_led_pin();

    init_timer3();
    init_user_button();

    GPIO_InitTypeDef GPIO_InitStructure1;

    //Configure SysTick Timer
    //if (SysTick_Config(SystemCoreClock / 100000))
    if (SysTick_Config(SystemCoreClock / 1000))
        while (1);

    /*----- Configure I2C Phase -----*/
    init_i2c1_peripheral(I2C1);                                               // To Do - Move All this into init_i2c1_peripheral()
    
    /*----- Transmission Phase -----*/
    // Init sequence for 128x64 OLED module
    ssd1306_i2c_init(I2C1, I2C1_SSD1306_SLAVE_ADDRESS8);
   
    // Draw the buffer
    ssd1306_i2c_draw_buffer(I2C1, I2C1_SSD1306_SLAVE_ADDRESS8, global_display_buffer);
    Delay(1000);
    

    ssd1306_clear_display_buffer(global_display_buffer);
    ssd1306_draw_pixel_to_buffer(64, 50, global_display_buffer);
    ssd1306_i2c_draw_buffer(I2C1, I2C1_SSD1306_SLAVE_ADDRESS8, global_display_buffer);

    uint8_t square_root_result = 0;
    uint16_t array_index;
    
    while(1){
        ssd1306_clear_display_buffer(global_display_buffer);
        switch(my_system_status.current_state){
            case clock_state:
                ssd1306_draw_big_char_to_buffer(0, 0, my_system_time.minutes/10 + '0', global_display_buffer);
                ssd1306_draw_big_char_to_buffer(24, 0, my_system_time.minutes%10 + '0', global_display_buffer);
                ssd1306_draw_big_char_to_buffer(48, 0, ':', global_display_buffer);
                ssd1306_draw_big_char_to_buffer(72, 0, my_system_time.seconds/10 + '0', global_display_buffer);
                ssd1306_draw_big_char_to_buffer(96, 0, my_system_time.seconds%10 + '0', global_display_buffer);
                break;
            case pattern_state:
                ssd1306_draw_circle_to_buffer(64, 32, 15, global_display_buffer);
                ssd1306_draw_circle_to_buffer(64, 32, 13, global_display_buffer);
                ssd1306_draw_circle_to_buffer(64, 32, 11, global_display_buffer);
                ssd1306_draw_circle_to_buffer(64, 32, 9, global_display_buffer);
                ssd1306_draw_circle_to_buffer(64, 32, 7, global_display_buffer);
                ssd1306_draw_circle_to_buffer(64, 32, 5, global_display_buffer);
                ssd1306_draw_circle_to_buffer(64, 32, 3, global_display_buffer);
                break;
            } 

        ssd1306_i2c_draw_buffer(I2C1, I2C1_SSD1306_SLAVE_ADDRESS8, global_display_buffer);
        Delay(250);

    }
    /* Scratch are follow until the end of main
    *  aka never gets down to thse below
    *
    *  Useful for testing new/old functions
    */

    // Loop forever printng strings and patterns
    while(1){
        // Draw the big nums
        ssd1306_clear_display_buffer(global_display_buffer);
        ssd1306_draw_big_string_to_buffer(0, 0, "12:00", global_display_buffer);      // Loop through
        ssd1306_i2c_draw_buffer(I2C1, I2C1_SSD1306_SLAVE_ADDRESS8, global_display_buffer);
        Delay(5000);

        uint16_t array_index;
        ssd1306_draw_char_to_buffer(0, 0, 'A', global_display_buffer);
        ssd1306_draw_string_to_buffer(0, 1, "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz", global_display_buffer);
        ssd1306_i2c_draw_buffer(I2C1, I2C1_SSD1306_SLAVE_ADDRESS8, global_display_buffer);

        Delay(5000);

        for(array_index=1; array_index < ((128*64)/8); array_index++){
            global_display_buffer[array_index]=0xAA;
        }

        ssd1306_i2c_draw_buffer(I2C1, I2C1_SSD1306_SLAVE_ADDRESS8, global_display_buffer);

        Delay(5000);

        for(array_index=0; array_index < ((128*64)/8); array_index++){
            global_display_buffer[array_index]=0x11;
        }

        ssd1306_i2c_draw_buffer(I2C1, I2C1_SSD1306_SLAVE_ADDRESS8, global_display_buffer);

        Delay(5000);

        for(array_index=0; array_index < ((128*64)/8); array_index++){
            global_display_buffer[array_index]=0x00;
        }

        ssd1306_i2c_draw_buffer(I2C1, I2C1_SSD1306_SLAVE_ADDRESS8, global_display_buffer);
    }

}

// Timer code
static __IO uint32_t TimingDelay;

void Delay(uint32_t nTime){
    TimingDelay = nTime;
    while(TimingDelay != 0);
}

uint8_t button_status;                  // Holds the debounced status of the user button



my_states_t get_next_state(my_states_t current_state){
    my_states_t next_state;

    switch(current_state){
        case clock_state:
            next_state = pattern_state;
            break;
        case pattern_state:
            next_state = clock_state;
            break;
     }
    return next_state;
}

void SysTick_Handler(void){
// SysTick interrupt handler - Configured to run every 1ms

    const uint8_t debounce_time = 100;
    static uint16_t pressed_counter = 0;
    static uint16_t not_pressed_counter = 0;
    // Delay
    if (TimingDelay != 0x00)
        TimingDelay--;

    // Debounce user button on PA0
    current_button_status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);

    if(!current_button_status){
        GPIO_WriteBit(GPIOC, GPIO_Pin_8, 0);
        not_pressed_counter++;
        pressed_counter=0;
        if(not_pressed_counter >= debounce_time){
            not_pressed_counter = debounce_time +1;
            //my_system_status.current_state = my_system_status.current_state;
        }
    }
    else{
        //GPIO_WriteBit(GPIOC, GPIO_Pin_8, 1);
        not_pressed_counter=0;
        pressed_counter++;
        if(pressed_counter >= debounce_time){
            GPIO_WriteBit(GPIOC, GPIO_Pin_8, 1);
            pressed_counter = debounce_time+1;
            //button_status = 1;
            my_system_status.current_state = get_next_state(my_system_status.current_state);
        }
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line){
    /* Infinite loop */
    /* Use GDB to find out why we're here */
    while(1);
}
#endif

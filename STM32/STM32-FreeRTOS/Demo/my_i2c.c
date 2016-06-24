#include "my_i2c.h"

// Function to init I2C1 peripheral
void init_i2c1_peripheral(I2C_TypeDef *I2Cx)
{

// Initializes the I2C1 Peripheral on PB6 & PB7
// 1) Enables the GPIOB Peripheral Clock
// 2) Enable the I2C1 Peripheral Clock
// 3) Configure the GPIO peripheral with GPIO_InitStructure and GPIO_Init()
// 4) Configure the I2C1 peripheral with I2C_InitStructure and I2C_Init() 

    #define I2C_SPEED                   50000                     /*!< I2C Speed */

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


// Sends and I2C Command
int i2c_send_command(I2C_TypeDef *I2Cx, uint8_t slave_address, uint8_t slave_data){
// Sends I2C data over I2Cx:
//  1) Sends Start Condition. Checks for I2C EV5 
//  2) Sends 7 bit address & checks for EV6
//  3) Sends 8 bit command byte (0x00) & checks for EV8
//  4) Sends 8 bits (1 byte) of data & checks for EV8
//  5) Sends Stop Condition
    int TimeOut;

    #define COMMAND_BYTE                0x00
    #define I2C_TIMEOUT                 100000


    /* Send I2C1 START condition */
    I2C_GenerateSTART(I2Cx, ENABLE);

    /* Test on I2C1 EV5 and clear it */
    TimeOut = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
    {
        TimeOut--;
        if (TimeOut == 0){
            //turn_on_error_led_pin();       // Error LED
            return 1;
        }
    }

    /* Send SSD1306 7 bit slave Address for write. Check to make sure ACK received */
    //I2C_Send7bitAddress(I2Cx, I2C1_SSD1306_SLAVE_ADDRESS8, I2C_Direction_Transmitter);
    I2C_Send7bitAddress(I2Cx, slave_address, I2C_Direction_Transmitter);

    //Test on I2C1 EV6 and clear it
    TimeOut = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        TimeOut--;
        if (TimeOut == 0){
            // Send I2C1 STOP Condition
            I2C_GenerateSTOP(I2Cx, ENABLE);
            //turn_on_error_led_pin();        // Error LED

            return 2;
        }
    }
    I2C_SendData(I2Cx, COMMAND_BYTE);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){ // Wait for EV8
        TimeOut--;
        if (TimeOut == 0){
            // Send I2C1 STOP Condition
            I2C_GenerateSTOP(I2Cx, ENABLE);
            //turn_on_error_led_pin();        // Error LED

            return 2;
        }
    }
    I2C_SendData(I2Cx, slave_data);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){ // Wait for EV8
        TimeOut--;
        if (TimeOut == 0){
            // Send I2C1 STOP Condition
            I2C_GenerateSTOP(I2Cx, ENABLE);
            //turn_on_error_led_pin();        // Error LED

            return 2;
        }
    }
    I2C_GenerateSTOP(I2Cx, ENABLE);
}


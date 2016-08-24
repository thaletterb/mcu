#include "w25q_flash.h"

/** @brief: initialize the ss pin
 */
void w25q_ss_pin_init(void)
{
    W25Q_CONTROL_DDR |= (1<<W25Q_SS_PIN);   // Set as output
}

/** @brief: drives the ss pin
 */
void w25q_ss_pin_drive(uint8_t high)
{
    if(high)
    {
        W25Q_CONTROL_PORT |= (1<<W25Q_SS_PIN);
    }
    else
    {
        W25Q_CONTROL_PORT &= ~(1<<W25Q_SS_PIN);
    }
}


/** @brief: return the manufacturer ID
 */
char w25q_read_manufacturer_id(void);

/** @brief: return the device ID
 */
char w25q_read_device_id(void);


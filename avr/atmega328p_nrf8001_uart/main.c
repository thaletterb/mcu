/** A small driver for interfacing atmega328p with
 *  w25q spi flash
 */

#include <avr/io.h>
#include <avr/delay.h>

#include "spi.h"

#include "hal_aci_tl.h"
#include "aci.h"
#include "aci_evts.h"
#include "aci_cmds.h"
#include "lib_aci.h"

#define W25Q_SS_PIN 1

#define LOGIC_LOW   0
#define LOGIC_HIGH  1

int main(void)
{
    spi_init();

    while(1)
    {

    }
    return 1;
}

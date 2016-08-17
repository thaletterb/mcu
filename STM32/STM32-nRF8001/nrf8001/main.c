#include <stm32f10x_spi.h>
#include "spi.h"

#include "lib_aci.h"

// Timer code
static __IO uint32_t TimingDelay;

void Delay(uint32_t nTime){
    TimingDelay = nTime;
    while(TimingDelay != 0);
}

void SysTick_Handler(void){
// SysTick interrupt handler - Configured to run every 1ms

    // Delay
    if (TimingDelay != 0x00)
        TimingDelay--;
}

int main(void){
    //Configure SysTick Timer
    if (SysTick_Config(SystemCoreClock / 1000))
        while (1);

    while (1) {

    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line){
    /* Infinite loop */
    /* Use GDB to find out why we're here */
    while(1);
}
#endif


#include "bsp/include/bsp_hal.h"
#include "bsp/cpu/bsp_cpu.h"

void bsp_irq_disable(void)
{
    __disable_irq();
}

bool bsp_debugger_attached(void)
{
    return (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0u;
}

void bsp_breakpoint(void)
{
    __BKPT(0);
}

void bsp_system_reset(void)
{
    NVIC_SystemReset();
    for (;;)
    {
    }
}

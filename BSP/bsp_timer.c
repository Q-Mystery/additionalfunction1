#include "AllHeader.h"

volatile uint32_t systick_counter = 0;
static volatile uint32_t g_runtime_ms = 0U;

void Timer_20ms_Init(void)
{
    NVIC_ClearPendingIRQ(TIMER_20ms_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_20ms_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_20ms_INST);
}

void Timer_Display_Init(void)
{
    NVIC_ClearPendingIRQ(TIMER_DISPLAY_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_DISPLAY_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_DISPLAY_INST);
}

u8 time_cnt = 0;

void TIMER_20ms_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_20ms_INST) == DL_TIMER_IIDX_ZERO) {
        encoder_update();
        Motion_Handle();
        g_runtime_ms += MOTOR_CONTROL_PERIOD_MS;
    }
}

void TIMER_DISPLAY_INST_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(TIMER_DISPLAY_INST) == DL_TIMER_IIDX_ZERO) {
        OLED_SHOW_IR();
    }
}

uint32_t Timer_Get_Runtime_Ms(void)
{
    return g_runtime_ms;
}

uint8_t Timer_Get_Runtime_Seconds99(void)
{
    uint32_t seconds = g_runtime_ms / 1000U;

    return (seconds > 99U) ? 99U : (uint8_t)seconds;
}

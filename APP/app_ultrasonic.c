#include "app_ultrasonic.h"
#include "AllHeader.h"

static AppUltrasonic_Status_t g_ultrasonic_status;
static bool g_ultrasonic_external_alarm;

static void Ultrasonic_Trig_Low(void)
{
    DL_GPIO_clearPins(ULTRASONIC_PORT, ULTRASONIC_TRIG_PIN);
}

static void Ultrasonic_Trig_High(void)
{
    DL_GPIO_setPins(ULTRASONIC_PORT, ULTRASONIC_TRIG_PIN);
}

static bool Ultrasonic_Echo_IsHigh(void)
{
    return (DL_GPIO_readPins(ULTRASONIC_PORT, ULTRASONIC_ECHO_PIN) != 0U);
}

static void Ultrasonic_SetSignalPins(bool obstacle)
{
    uint32_t pins = ULTRASONIC_SIGNAL_PB19_PIN | ULTRASONIC_SIGNAL_PB24_PIN;

    if (obstacle || g_ultrasonic_external_alarm) {
        DL_GPIO_setPins(ULTRASONIC_SIGNAL_PORT, pins);
    } else {
        DL_GPIO_clearPins(ULTRASONIC_SIGNAL_PORT, pins);
    }
}

void AppUltrasonic_Init(void)
{
    memset(&g_ultrasonic_status, 0, sizeof(g_ultrasonic_status));
    g_ultrasonic_external_alarm = false;
    DL_GPIO_initDigitalOutput(ULTRASONIC_TRIG_IOMUX);
    DL_GPIO_initDigitalInputFeatures(ULTRASONIC_ECHO_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalOutput(ULTRASONIC_SIGNAL_PB19_IOMUX);
    DL_GPIO_initDigitalOutput(ULTRASONIC_SIGNAL_PB24_IOMUX);
    Ultrasonic_Trig_Low();
    DL_GPIO_enableOutput(ULTRASONIC_PORT, ULTRASONIC_TRIG_PIN);
    Ultrasonic_SetSignalPins(false);
    DL_GPIO_enableOutput(ULTRASONIC_SIGNAL_PORT,
        ULTRASONIC_SIGNAL_PB19_PIN | ULTRASONIC_SIGNAL_PB24_PIN);
}

static bool Ultrasonic_ReadPulseUs(uint16_t *pulse_us)
{
    uint16_t wait_us = 0U;
    uint16_t high_us = 0U;

    Ultrasonic_Trig_Low();
    delay_us(2U);
    Ultrasonic_Trig_High();
    delay_us(ULTRASONIC_TRIGGER_PULSE_US);
    Ultrasonic_Trig_Low();

    while (!Ultrasonic_Echo_IsHigh()) {
        if (wait_us++ >= ULTRASONIC_ECHO_TIMEOUT_US) {
            return false;
        }
        delay_us(1U);
    }

    while (Ultrasonic_Echo_IsHigh()) {
        if (high_us++ >= ULTRASONIC_ECHO_TIMEOUT_US) {
            return false;
        }
        delay_us(1U);
    }

    *pulse_us = high_us;
    return true;
}

void AppUltrasonic_Update(void)
{
    uint16_t pulse_us;
    uint16_t distance_cm;
    uint32_t now_ms = Timer_Get_Runtime_Ms();

    if ((g_ultrasonic_status.last_update_ms != 0U) &&
        ((uint32_t)(now_ms - g_ultrasonic_status.last_update_ms) <
            ULTRASONIC_UPDATE_INTERVAL_MS)) {
        return;
    }
    g_ultrasonic_status.last_update_ms = now_ms;

    if (!Ultrasonic_ReadPulseUs(&pulse_us)) {
        g_ultrasonic_status.valid = false;
        g_ultrasonic_status.obstacle = false;
        Ultrasonic_SetSignalPins(false);
        return;
    }

    distance_cm = (uint16_t)(pulse_us / 58U);
    if (distance_cm == 0U) {
        distance_cm = 1U;
    }

    g_ultrasonic_status.valid = true;
    g_ultrasonic_status.distance_cm = distance_cm;

    if (distance_cm <= ULTRASONIC_STOP_DISTANCE_CM) {
        g_ultrasonic_status.obstacle = true;
    } else if (distance_cm >= ULTRASONIC_CLEAR_DISTANCE_CM) {
        g_ultrasonic_status.obstacle = false;
    }
    Ultrasonic_SetSignalPins(g_ultrasonic_status.obstacle);
}

bool AppUltrasonic_IsObstacle(void)
{
    return g_ultrasonic_status.obstacle;
}

void AppUltrasonic_SetExternalAlarm(bool active)
{
    g_ultrasonic_external_alarm = active;
    Ultrasonic_SetSignalPins(g_ultrasonic_status.obstacle);
}

const AppUltrasonic_Status_t *AppUltrasonic_GetStatus(void)
{
    return &g_ultrasonic_status;
}

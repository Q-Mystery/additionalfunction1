#include "AllHeader.h"
#include "app_bcd_display.h"
#include "app_control_config.h"
#include "app_imu.h"
#include "app_status_display.h"
#include "app_track_mission.h"
#include "app_ultrasonic.h"
#include "app_voice.h"

#define APP_TIMED_ALARM_NONE (0xFFU)

static uint8_t AppTimedAlarmIndex(uint32_t now_ms)
{
    static const uint32_t alarm_start_ms[] = {
        3000U, 10000U, 13000U, 19500U,
        32500U, 40000U, 43000U, 50000U
    };
    uint8_t i;

    for (i = 0U;
         i < (sizeof(alarm_start_ms) / sizeof(alarm_start_ms[0]));
         i++) {
        uint32_t start_ms = alarm_start_ms[i];
        if ((now_ms >= start_ms) &&
            ((uint32_t)(now_ms - start_ms) < APP_TIMED_ALARM_DURATION_MS)) {
            return i;
        }
    }

    return APP_TIMED_ALARM_NONE;
}

static bool AppTimedPauseActive(uint32_t now_ms)
{
    return (now_ms >= APP_TIMED_PAUSE_START_MS) &&
           (now_ms < APP_TIMED_PAUSE_END_MS);
}

static bool AppTimedFinalMotorOff(uint32_t now_ms)
{
    return now_ms >= APP_TIMED_FINAL_MOTOR_OFF_MS;
}

int main(void)
{
    uint8_t display_divider = 0U;
    uint8_t timed_alarm_index = APP_TIMED_ALARM_NONE;
    uint8_t timed_alarm_last_index = APP_TIMED_ALARM_NONE;
    uint32_t now_ms;
    bool obstacle_now = false;
    bool obstacle_last = false;
    bool timed_alarm_active;
    bool timed_pause_active;
    bool timed_final_off;

    SYSCFG_DL_init();
    AppBCDDisplay_Init();
    OLED_Init();
    (void)AppIMU_Init();
    AppUltrasonic_Init();
    AppVoice_Init();
    Init_Motor_PWM();
    Motor_Stop(STOP_FREE);

    /* Initialize PID state before the encoder timer can call Motion_Handle(). */
    PID_Param_Init();
    PID_Set_Motor_Parm(0U, MOTOR_SPEED_PID_KP, MOTOR_SPEED_PID_KI,
                       MOTOR_SPEED_PID_KD);
    PID_Set_Motor_Parm(1U, MOTOR_SPEED_PID_KP, MOTOR_SPEED_PID_KI,
                       MOTOR_SPEED_PID_KD);
    encoder_init();
    AppTrackMission_Init();

    while (1) {
        now_ms = Timer_Get_Runtime_Ms();
        timed_alarm_index = AppTimedAlarmIndex(now_ms);
        timed_alarm_active = (timed_alarm_index != APP_TIMED_ALARM_NONE);
        timed_pause_active = AppTimedPauseActive(now_ms);
        timed_final_off = AppTimedFinalMotorOff(now_ms);

        AppBCDDisplay_Update();
        AppUltrasonic_Update();
        AppUltrasonic_SetExternalAlarm(timed_alarm_active);
        if (timed_alarm_active) {
            OPEN_MCULED();
            Beep_ON();
            if (timed_alarm_index != timed_alarm_last_index) {
                (void)AppVoice_SendCommand(VOICE_OBSTACLE_COMMAND);
                timed_alarm_last_index = timed_alarm_index;
            }
        } else {
            CLOSE_MCULED();
            Beep_OFF();
            timed_alarm_last_index = APP_TIMED_ALARM_NONE;
        }

        obstacle_now = AppUltrasonic_IsObstacle();
        if (timed_final_off) {
            Motion_Stop(STOP_FREE);
        } else if (timed_pause_active) {
            Motion_Stop(STOP_BRAKE);
        } else if (obstacle_now) {
            Motion_Stop(STOP_BRAKE);
            (void)AppVoice_TriggerObstacle();
        } else {
            AppTrackMission_Update();
        }
        if (obstacle_now != obstacle_last) {
            display_divider = APP_OLED_DISPLAY_DIVIDER;
            obstacle_last = obstacle_now;
        }
        display_divider++;
        if (display_divider >= APP_OLED_DISPLAY_DIVIDER) {
            display_divider = 0U;
            AppIMU_Update();
            AppStatusDisplay_Update();
        }
        delay_ms(APP_MAIN_LOOP_DELAY_MS);
    }
}

#ifndef __APP_ULTRASONIC_H__
#define __APP_ULTRASONIC_H__

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;
    bool obstacle;
    uint16_t distance_cm;
    uint32_t last_update_ms;
} AppUltrasonic_Status_t;

void AppUltrasonic_Init(void);
void AppUltrasonic_Update(void);
bool AppUltrasonic_IsObstacle(void);
void AppUltrasonic_SetExternalAlarm(bool active);
const AppUltrasonic_Status_t *AppUltrasonic_GetStatus(void);

#endif

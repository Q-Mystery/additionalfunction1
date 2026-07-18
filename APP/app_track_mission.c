#include "app_track_mission.h"
#include "AllHeader.h"
#include "app_imu.h"

static AppTrackMission_Status_t g_track_mission;
static uint32_t s_last_imu_ms;
static uint32_t s_last_mission_ms;
static uint32_t s_candidate_window_ms;
static int32_t s_candidate_positive_angle_x10;
static int32_t s_candidate_negative_angle_x10;
static int32_t s_candidate_carry_angle_x10;
static int8_t s_candidate_direction;
static uint32_t s_arc_duration_ms;
static uint32_t s_exit_stable_ms;
static uint32_t s_cooldown_ms;
static uint32_t s_first_lap_pause_ms;
static bool s_first_lap_pause_done;
static bool s_stop_commanded;

static int32_t Mission_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint32_t Mission_EncoderAvgAbsCounts(void)
{
    int encoder_counts[2] = {0, 0};
    uint32_t left_counts;
    uint32_t right_counts;
    uint32_t sum = 0U;
    uint8_t count = 0U;

    Encoder_Get_ALL(encoder_counts);
    left_counts = (uint32_t)Mission_Abs32(encoder_counts[0]);
    right_counts = (uint32_t)Mission_Abs32(encoder_counts[1]);

    if (left_counts != 0U) {
        sum += left_counts;
        count++;
    }
    if (right_counts != 0U) {
        sum += right_counts;
        count++;
    }

    return (count == 0U) ? 0U : (sum / count);
}

static bool Mission_EncoderStopReached(void)
{
    return Mission_EncoderAvgAbsCounts() >= TRACK_MISSION_ENCODER_STOP_COUNTS;
}

static int8_t Mission_Sign(int32_t value)
{
    if (value > 0) {
        return 1;
    }
    if (value < 0) {
        return -1;
    }
    return 0;
}

static bool Mission_CurveSensorSeen(void)
{
    return (X1 != 0U) || (X2 != 0U) || (X3 != 0U) ||
           (X6 != 0U) || (X7 != 0U) || (X8 != 0U);
}

static bool Mission_CenterStableSeen(void)
{
    return ((X4 != 0U) || (X5 != 0U)) &&
           (X1 == 0U) && (X2 == 0U) && (X3 == 0U) &&
           (X6 == 0U) && (X7 == 0U) && (X8 == 0U);
}

static void Mission_ResetCandidate(void)
{
    s_candidate_window_ms = 0U;
    s_candidate_positive_angle_x10 = 0;
    s_candidate_negative_angle_x10 = 0;
    s_candidate_carry_angle_x10 = 0;
    s_candidate_direction = 0;
}

static void Mission_ResetArc(void)
{
    g_track_mission.arc_angle_x10 = 0;
    g_track_mission.arc_direction = 0;
    s_arc_duration_ms = 0U;
    s_exit_stable_ms = 0U;
}

static void Mission_SetState(AppTrackMission_State_t state)
{
    g_track_mission.state = state;
    g_track_mission.state_elapsed_ms = 0U;

    if (state == APP_TRACK_WAIT_CURVE) {
        Mission_ResetCandidate();
        Mission_ResetArc();
    } else if (state == APP_TRACK_ARC_TRACKING) {
        g_track_mission.arc_direction = s_candidate_direction;
        g_track_mission.arc_angle_x10 = s_candidate_carry_angle_x10;
        s_arc_duration_ms = s_candidate_window_ms;
        s_exit_stable_ms = 0U;
        Mission_ResetCandidate();
    } else if (state == APP_TRACK_ARC_WAIT_EXIT) {
        s_exit_stable_ms = 0U;
    } else if (state == APP_TRACK_FIRST_LAP_PAUSE) {
        s_first_lap_pause_ms = 0U;
        Mission_ResetCandidate();
        Mission_ResetArc();
    } else if (state == APP_TRACK_COOLDOWN) {
        s_cooldown_ms = 0U;
        Mission_ResetCandidate();
        Mission_ResetArc();
    }
}

static bool Mission_UpdateImu(uint32_t *dt_ms)
{
    const AppIMU_Status_t *imu;
    int32_t raw_rate;

    AppIMU_Update();
    imu = AppIMU_GetStatus();
    g_track_mission.imu_available = imu->available;

    if (!imu->available || (imu->last_update_ms == 0U)) {
        s_last_imu_ms = imu->last_update_ms;
        return false;
    }

    if (s_last_imu_ms == 0U) {
        s_last_imu_ms = imu->last_update_ms;
        raw_rate = imu->yaw_rate_dps_x10;
        g_track_mission.yaw_rate_filtered_x10 = raw_rate;
        return false;
    }

    if (imu->last_update_ms == s_last_imu_ms) {
        return false;
    }

    *dt_ms = (uint32_t)(imu->last_update_ms - s_last_imu_ms);
    s_last_imu_ms = imu->last_update_ms;

    raw_rate = imu->yaw_rate_dps_x10;
    g_track_mission.yaw_rate_filtered_x10 =
        ((g_track_mission.yaw_rate_filtered_x10 *
          (TRACK_MISSION_GYRO_LPF_DEN - TRACK_MISSION_GYRO_LPF_NUM)) +
         (raw_rate * TRACK_MISSION_GYRO_LPF_NUM)) /
        TRACK_MISSION_GYRO_LPF_DEN;

    return true;
}

static int32_t Mission_AngleDeltaX10(int32_t rate_x10, uint32_t dt_ms)
{
    return (rate_x10 * (int32_t)dt_ms) / 1000;
}

static void Mission_UpdateWaitCurve(uint32_t dt_ms)
{
    int32_t rate = g_track_mission.yaw_rate_filtered_x10;
    int32_t abs_rate = Mission_Abs32(rate);
    bool curve_seen = Mission_CurveSensorSeen();
    int32_t delta_x10;
    int32_t main_angle_x10;
    int32_t counter_angle_x10;
    int32_t total_angle_x10;
    int32_t net_angle_x10;
    int8_t sign;

    if (!curve_seen && (abs_rate < TRACK_MISSION_ENTER_RATE_X10)) {
        Mission_ResetCandidate();
        return;
    }

    if (abs_rate < TRACK_MISSION_GYRO_DEADBAND_X10) {
        return;
    }

    sign = Mission_Sign(rate);
    if (sign == 0) {
        Mission_ResetCandidate();
        return;
    }

    if ((s_candidate_direction != 0) &&
        (sign != s_candidate_direction)) {
        Mission_ResetCandidate();
    }
    if (s_candidate_direction == 0) {
        s_candidate_direction = sign;
    }

    delta_x10 = Mission_AngleDeltaX10(abs_rate, dt_ms);
    if (sign > 0) {
        s_candidate_positive_angle_x10 += delta_x10;
    } else {
        s_candidate_negative_angle_x10 += delta_x10;
    }

    s_candidate_window_ms += dt_ms;
    if (s_candidate_window_ms < TRACK_MISSION_ENTER_WINDOW_MS) {
        return;
    }

    if (s_candidate_positive_angle_x10 >= s_candidate_negative_angle_x10) {
        s_candidate_direction = 1;
        main_angle_x10 = s_candidate_positive_angle_x10;
        counter_angle_x10 = s_candidate_negative_angle_x10;
    } else {
        s_candidate_direction = -1;
        main_angle_x10 = s_candidate_negative_angle_x10;
        counter_angle_x10 = s_candidate_positive_angle_x10;
    }

    total_angle_x10 = main_angle_x10 + counter_angle_x10;
    net_angle_x10 = main_angle_x10 - counter_angle_x10;

    if (((curve_seen != false) || (abs_rate >= TRACK_MISSION_ENTER_RATE_X10)) &&
        (total_angle_x10 > 0) &&
        ((main_angle_x10 * TRACK_MISSION_ENTER_SIGN_DEN) >=
         (total_angle_x10 * TRACK_MISSION_ENTER_SIGN_NUM)) &&
        (net_angle_x10 >= TRACK_MISSION_ENTER_NET_ANGLE_X10)) {
        s_candidate_carry_angle_x10 = main_angle_x10;
        Mission_SetState(APP_TRACK_ARC_TRACKING);
    } else {
        Mission_ResetCandidate();
    }
}

static void Mission_UpdateArcTracking(uint32_t dt_ms)
{
    int32_t rate = g_track_mission.yaw_rate_filtered_x10;
    int32_t abs_rate = Mission_Abs32(rate);
    int8_t sign = Mission_Sign(rate);

    s_arc_duration_ms += dt_ms;
    if ((sign == g_track_mission.arc_direction) &&
        (abs_rate >= TRACK_MISSION_GYRO_DEADBAND_X10)) {
        g_track_mission.arc_angle_x10 +=
            Mission_AngleDeltaX10(abs_rate, dt_ms);
    }

    if ((s_arc_duration_ms >= TRACK_MISSION_ARC_MIN_DURATION_MS) &&
        (g_track_mission.arc_angle_x10 >= TRACK_MISSION_ARC_DONE_ANGLE_X10)) {
        Mission_SetState(APP_TRACK_ARC_WAIT_EXIT);
    }
}

static void Mission_UpdateArcExit(uint32_t dt_ms)
{
    int32_t abs_rate = Mission_Abs32(g_track_mission.yaw_rate_filtered_x10);

    if ((abs_rate <= TRACK_MISSION_ARC_EXIT_RATE_X10) &&
        (Mission_CenterStableSeen() != false)) {
        s_exit_stable_ms += dt_ms;
    } else {
        s_exit_stable_ms = 0U;
    }

    if (s_exit_stable_ms < TRACK_MISSION_ARC_EXIT_CONFIRM_MS) {
        return;
    }

    if (g_track_mission.arc_count < TRACK_MISSION_TARGET_ARCS) {
        g_track_mission.arc_count++;
    }

    /*
     * The timed 50 s rule now owns the final motor-off action. Do not stop the
     * motor automatically on the fourth counted arc.
     */
    /*
    if (g_track_mission.arc_count >= TRACK_MISSION_TARGET_ARCS) {
        Mission_SetState(APP_TRACK_STOPPED);
    } else
    */
    if (!s_first_lap_pause_done &&
        (g_track_mission.arc_count >= TRACK_MISSION_FIRST_LAP_ARCS)) {
        Mission_SetState(APP_TRACK_FIRST_LAP_PAUSE);
    } else {
        Mission_SetState(APP_TRACK_COOLDOWN);
    }
}

static void Mission_UpdateFirstLapPause(uint32_t dt_ms)
{
    if (s_first_lap_pause_ms < TRACK_MISSION_FIRST_LAP_PAUSE_MS) {
        s_first_lap_pause_ms += dt_ms;
    }

    if (s_first_lap_pause_ms >= TRACK_MISSION_FIRST_LAP_PAUSE_MS) {
        s_first_lap_pause_done = true;
        PID_Clear_Motor(MAX_MOTOR);
        Mission_SetState(APP_TRACK_COOLDOWN);
    }
}

static void Mission_UpdateCooldown(uint32_t dt_ms)
{
    s_cooldown_ms += dt_ms;
    if (s_cooldown_ms >= TRACK_MISSION_ARC_COOLDOWN_MS) {
        Mission_SetState(APP_TRACK_WAIT_CURVE);
    }
}

static void Mission_StopOnce(void)
{
    if (!s_stop_commanded) {
        Motion_Stop(STOP_BRAKE);
        s_stop_commanded = true;
    }
}

void AppTrackMission_Init(void)
{
    memset(&g_track_mission, 0, sizeof(g_track_mission));
    s_last_imu_ms = 0U;
    s_last_mission_ms = Timer_Get_Runtime_Ms();
    s_first_lap_pause_ms = 0U;
    s_first_lap_pause_done = false;
    s_stop_commanded = false;
    Mission_ResetCandidate();
    Mission_ResetArc();
    Mission_SetState(APP_TRACK_WAIT_CURVE);
}

void AppTrackMission_Update(void)
{
    uint32_t now_ms;
    uint32_t dt_ms = 0U;
    uint32_t imu_dt_ms = 0U;

    if (Mission_EncoderStopReached()) {
        Mission_SetState(APP_TRACK_STOPPED);
    }

    if (g_track_mission.state == APP_TRACK_STOPPED) {
        Mission_StopOnce();
        return;
    }

    (void)Mission_UpdateImu(&imu_dt_ms);

    now_ms = Timer_Get_Runtime_Ms();
    if (s_last_mission_ms == 0U) {
        s_last_mission_ms = now_ms;
    }
    dt_ms = (uint32_t)(now_ms - s_last_mission_ms);
    if (dt_ms > 0U) {
        s_last_mission_ms = now_ms;
        g_track_mission.state_elapsed_ms += dt_ms;

        switch (g_track_mission.state) {
        case APP_TRACK_WAIT_CURVE:
            Mission_UpdateWaitCurve(dt_ms);
            break;
        case APP_TRACK_ARC_TRACKING:
            Mission_UpdateArcTracking(dt_ms);
            break;
        case APP_TRACK_ARC_WAIT_EXIT:
            Mission_UpdateArcExit(dt_ms);
            break;
        case APP_TRACK_FIRST_LAP_PAUSE:
            Mission_UpdateFirstLapPause(dt_ms);
            break;
        case APP_TRACK_COOLDOWN:
            Mission_UpdateCooldown(dt_ms);
            break;
        case APP_TRACK_STOPPED:
        default:
            Mission_SetState(APP_TRACK_STOPPED);
            break;
        }
    }

    if (g_track_mission.state == APP_TRACK_STOPPED) {
        Mission_StopOnce();
    } else if (g_track_mission.state == APP_TRACK_FIRST_LAP_PAUSE) {
        Motion_Stop(STOP_BRAKE);
    } else {
        LineWalking();
    }
}

bool AppTrackMission_IsStopped(void)
{
    return g_track_mission.state == APP_TRACK_STOPPED;
}

const AppTrackMission_Status_t *AppTrackMission_GetStatus(void)
{
    return &g_track_mission;
}

#ifndef __APP_CONTROL_CONFIG_H__
#define __APP_CONTROL_CONFIG_H__

/*
 * Central control configuration for the line-following car.
 * Tune the car from this file first; source files should not carry hard-coded
 * motor, encoder, grayscale, speed, or PID constants.
 */

/* Main loop and display timing. */
#define APP_MAIN_LOOP_DELAY_MS             (1U)
#define APP_OLED_DISPLAY_DIVIDER           (250U)
#define MOTOR_DIRECT_TEST_START_DELAY_MS   (3000U)

/* Timed race actions from power-on/runtime timer. */
#define APP_TIMED_PAUSE_START_MS           (19500U)
#define APP_TIMED_PAUSE_DURATION_MS        (10000U)
#define APP_TIMED_PAUSE_END_MS \
    (APP_TIMED_PAUSE_START_MS + APP_TIMED_PAUSE_DURATION_MS)
#define APP_TIMED_ALARM_DURATION_MS        (1000U)
#define APP_TIMED_FINAL_MOTOR_OFF_MS       (50000U)
#define APP_TIMED_DISPLAY_FREEZE_MS        APP_TIMED_FINAL_MOTOR_OFF_MS

/* Two-digit BCD 7-segment display digit select: PA0=tens, PA1=ones. */
#define BCD_DIGIT_SELECT_ACTIVE_HIGH       (1U)

/*
 * MPU6050 uses an independent software I2C bus.
 * OLED wiring stays on SCL = PA8, SDA = PA22. MPU6050 wiring is independent:
 * SCL = PB2, SDA = PB3, XDA = PA24, XCL = PA25, AD0 = PA26, INT = PA27.
 * AD0 is driven low so the MPU6050 address stays at 0x68.
 */
#define IMU_I2C_ADDRESS                    (0x68U)
#define IMU_INIT_RETRY_COUNT               (2U)
#define IMU_CALIBRATION_SAMPLES            (128U)
#define IMU_GYRO_Z_CURVE_THRESHOLD_RAW     (300)
#define IMU_GYRO_Z_DPS_X10_DIVISOR         (16)
#define IMU_UPDATE_MIN_INTERVAL_MS         (20U)

/*
 * Two-lap oval mission based on MPU6050 yaw rate only.
 * The bottom line follower still keeps the car on the black line; these
 * parameters only identify the four sustained half-circle turns in
 * A-B-C-D-A-B-C-D-A. Encoder distance is deliberately not used for this task.
 */
#define TRACK_MISSION_TARGET_ARCS          (4U)
#define TRACK_MISSION_FIRST_LAP_ARCS       (2U)
#define TRACK_MISSION_FIRST_LAP_PAUSE_MS   (10000U)
#define TRACK_MISSION_GYRO_LPF_NUM         (1)
#define TRACK_MISSION_GYRO_LPF_DEN         (4)
#define TRACK_MISSION_GYRO_DEADBAND_X10    (8)    /* 0.8 deg/s */
#define TRACK_MISSION_ENTER_RATE_X10       (15)   /* 1.5 deg/s */
#define TRACK_MISSION_ENTER_WINDOW_MS      (300U)
#define TRACK_MISSION_ENTER_NET_ANGLE_X10  (5)    /* 0.5 deg */
#define TRACK_MISSION_ENTER_SIGN_NUM       (7U)
#define TRACK_MISSION_ENTER_SIGN_DEN       (10U)
#define TRACK_MISSION_ARC_DONE_ANGLE_X10   (1350) /* 135 deg */
#define TRACK_MISSION_ARC_EXIT_RATE_X10    (20)   /* 2.0 deg/s */
#define TRACK_MISSION_ARC_EXIT_CONFIRM_MS  (2000U)
#define TRACK_MISSION_ARC_MIN_DURATION_MS  (1000U)
#define TRACK_MISSION_ARC_COOLDOWN_MS      (0U)
#define TRACK_MISSION_ENCODER_STOP_COUNTS  (38800U)

/*
 * Ultrasonic obstacle avoidance.
 * Wiring: TRIG = PA15, ECHO = PA17. Echo pulse width in microseconds is about
 * distance_cm * 58 for common HC-SR04-compatible modules.
 * Obstacle signal outputs: PB19 and PB24 are driven high while an obstacle is
 * latched, and low otherwise.
 */
#define ULTRASONIC_STOP_DISTANCE_CM        (5U)
#define ULTRASONIC_CLEAR_DISTANCE_CM       (8U)
#define ULTRASONIC_TRIGGER_PULSE_US        (10U)
#define ULTRASONIC_ECHO_TIMEOUT_US         (6000U)
#define ULTRASONIC_UPDATE_INTERVAL_MS      (60U)
#define ULTRASONIC_SIGNAL_PORT             (GPIOB)
#define ULTRASONIC_SIGNAL_PB19_PIN         (DL_GPIO_PIN_19)
#define ULTRASONIC_SIGNAL_PB19_IOMUX       (IOMUX_PINCM45)
#define ULTRASONIC_SIGNAL_PB24_PIN         (DL_GPIO_PIN_24)
#define ULTRASONIC_SIGNAL_PB24_IOMUX       (IOMUX_PINCM52)

/*
 * IIC voice module.
 * Wiring is independent of OLED and MPU6050: SCL = PA31 (header pin 37),
 * SDA = PA28 (header pin 38).
 *
 * The Excel protocol table gives 5-byte UART-style frames such as:
 *   alarm = AA 55 00 26 FB
 * The supplied IIC example does not send the whole frame. It calls
 * set_voice(data), which writes one byte to device address 0x2B/register 0x03.
 * For SCL/SDA wiring we therefore send the frame command byte, e.g. alarm 0x26.
 */
#define VOICE_IIC_ADDRESS                  (0x2BU)
#define VOICE_IIC_WRITE_REGISTER           (0x03U)
#define VOICE_IIC_READ_REGISTER            (0x64U)
#define VOICE_IIC_DELAY_US                 (5U)
#define VOICE_IIC_ACK_WAIT_COUNT           (10U)
#define VOICE_IIC_RETRY_COUNT              (2U)
#define VOICE_INIT_DELAY_MS                (50U)
#define VOICE_ALERT_REPEAT_INTERVAL_MS     (1500U)

/* Command bytes from /Users/wjx_macair/Desktop/命令词播报词协议列表V3_中文.xlsx. */
#define VOICE_FRAME_HEAD_0                 (0xAAU)
#define VOICE_FRAME_HEAD_1                 (0x55U)
#define VOICE_FRAME_TAIL                   (0xFBU)
#define VOICE_FRAME_GROUP_MAIN             (0x00U)
#define VOICE_FRAME_GROUP_BROADCAST        (0xFFU)
#define VOICE_CMD_ALARM                    (0x26U) /* AA 55 00 26 FB: 报警 */
#define VOICE_CMD_INIT_DONE                (0x58U) /* AA 55 FF 58 FB: 初始化完成 */
#define VOICE_INIT_COMMAND                 VOICE_CMD_INIT_DONE
#define VOICE_OBSTACLE_COMMAND             VOICE_CMD_ALARM

/*
 * Eight-channel grayscale sensor.
 * Keep EIGHT_IR_SENSOR_REVERSED at 0 if X1 is the left-most probe when the
 * car faces forward. Set it to 1 if the physical sensor order is mirrored.
 */
#define EIGHT_IR_SENSOR_REVERSED           (0U)
#define EIGHT_IR_CHANNEL_SETTLE_US         (50U)

/*
 * Motor driver PWM.
 * MOTOR_PWM_MAX_DUTY must match the SysConfig PWM period. MOTOR_PWM_DEAD_ZONE
 * is added only after a non-zero PID output so the motor can overcome static
 * friction without making a zero command move the wheel. The motor supply is
 * 7.4 V; the target average drive voltage below caps normal forward drive at
 * 7.4 V while still keeping zero commands stopped.
 */
#define MOTOR_PWM_MAX_DUTY                 (1000)
#define MOTOR_PWM_COMPARE_INVERTED         (1U)
#define MOTOR_PWM_DEAD_ZONE                (160)
#define MOTOR_SUPPLY_MV                    (7400U)
#define MOTOR_TARGET_MAX_AVERAGE_MV        (7400U)
#define MOTOR_TARGET_EFFECTIVE_PWM_DUTY_RAW \
    (((MOTOR_TARGET_MAX_AVERAGE_MV * MOTOR_PWM_MAX_DUTY) + \
      (MOTOR_SUPPLY_MV / 2U)) / MOTOR_SUPPLY_MV)
#define MOTOR_TARGET_EFFECTIVE_PWM_DUTY \
    ((MOTOR_TARGET_EFFECTIVE_PWM_DUTY_RAW > MOTOR_PWM_MAX_DUTY) ? \
        MOTOR_PWM_MAX_DUTY : MOTOR_TARGET_EFFECTIVE_PWM_DUTY_RAW)
#define MOTOR_LEFT_PWM_CHANNEL_INDEX       GPIO_motor_PWM_C1_IDX
#define MOTOR_RIGHT_PWM_CHANNEL_INDEX      GPIO_motor_PWM_C2_IDX

/*
 * Motor direction and encoder polarity.
 * Change one of these only after the lifted-wheel test confirms that wheel or
 * encoder direction is opposite to the chassis-forward direction.
 */
#define LEFT_MOTOR_DIR_REVERSED            (0U)
#define RIGHT_MOTOR_DIR_REVERSED           (1U)
#define ENCODER_E1_REVERSED                (0U)
#define ENCODER_E4_REVERSED                (0U)
#define MOTOR_ENCODER_IRQ_ENABLE           (1)

/*
 * Chassis and encoder geometry.
 * MD310 common configuration: 20:1 gearbox, 13-line encoder, both A/B edge
 * channels counted by software = 520 counts per wheel revolution.
 */
#define MD310_ENCODER_CIRCLE_PULSES        (520.0f)
#define MECANUM_CIRCLE_MM                  (150.796f)
#define MSPM0Car_APB                       (157.09f)
#define MOTOR_CONTROL_PERIOD_MS            (20U)

/*
 * Speed limit and overspeed protection.
 * MOTOR_MAX_FORWARD_SPEED_MM_S is the closed-loop speed command limit. The
 * active brake is disabled by default because an early brake pulse can make the
 * car appear unable to track; enable it only after encoder scale is verified.
 */
#define MOTOR_MAX_FORWARD_SPEED_MM_S       (160)
#define MOTOR_OVERSPEED_BRAKE_ENABLE       (0U)
#define MOTOR_MAX_PULSES_PER_20MS          (45)
#define MOTION_COMMAND_LIMIT_MM_S          (1000)
#define MOTION_PERCENT_SPEED_SCALE         (10U)
#define MOTION_SPIN_SPEED_MULTIPLIER       (5)
#define MOTION_YAW_RATE_SCALE              (1000.0f)
#define MOTOR_TARGET_RAMP_STEP_MM_S        (10)
#define MOTOR_PID_PWM_LIMIT \
    ((MOTOR_TARGET_EFFECTIVE_PWM_DUTY > MOTOR_PWM_DEAD_ZONE) ? \
        ((float)(MOTOR_TARGET_EFFECTIVE_PWM_DUTY - MOTOR_PWM_DEAD_ZONE)) : \
        0.0f)
#define MOTOR_SPEED_FEEDFORWARD_PWM_PER_MM_S (4.20f)
#define MOTOR_PID_CORRECTION_LIMIT         (120.0f)

/*
 * Wheel speed PID. The same gains are applied to left and right wheels.
 * Increase KP if speed response is too slow; increase KI only after the car can
 * already follow the line without large oscillation.
 */
#define MOTOR_SPEED_PID_KP                 (0.35f)
#define MOTOR_SPEED_PID_KI                 (0.008f)
#define MOTOR_SPEED_PID_KD                 (0.00f)

/* Optional yaw PID used by legacy IMU-assisted movement functions. */
#define YAW_PID_KP                         (0.40f)
#define YAW_PID_KI                         (0.00f)
#define YAW_PID_KD                         (0.10f)

/*
 * Black-line tracking for an oval/track-field style course.
 * Keep both wheels moving while correcting. Only enter the fast straight mode
 * after the central probe window is stable for a short time.
 */
#define LINE_TURN_KP                       (6.00f)
#define LINE_TURN_KD                       (10.00f)
#define LINE_BASE_SPEED_MM_S               (75)
#define LINE_FAST_SPEED_MM_S               (110)
#define LINE_CORNER_SPEED_MM_S             (80)
#define LINE_HARD_CORNER_SPEED_MM_S        (70)
#define LINE_CORRECTION_SPEED_MM_S         LINE_CORNER_SPEED_MM_S
#define LINE_SEARCH_SPEED_MM_S             (5)
#define LINE_MAX_WHEEL_SPEED_MM_S          MOTOR_MAX_FORWARD_SPEED_MM_S
#define LINE_CENTER_DEADBAND               (1)
#define LINE_MAX_TURN_DELTA_MM_S           (60)
#define LINE_FAST_STABLE_MS                (80U)
#define LINE_FAST_RAMP_STEP_MM_S           (10)
#define LINE_FAST_RAMP_STEP_MS             (120U)
#define LINE_TURN_LATCH_MS                 (80U)
#define LINE_LOST_BRAKE_MS                 (60U)
#define LINE_LOST_RECOVERY_MS              (560U)
#define LINE_FAST_AFTER_TURN_STABLE_MS     (180U)
#define LINE_CURVE_MAX_SPEED_MM_S          (LINE_FAST_SPEED_MM_S / 3)
#define LINE_CURVE_SLOW_SPEED_MM_S         (30)
#define LINE_CURVE_SLOW_HOLD_MS            (500U)
#define LINE_CURVE_SLOW_MAX_TURN_DELTA_MM_S (6)
#define LINE_CURVE_SLOW_SOFT_INNER_SPEED_MM_S (12)
#define LINE_CURVE_SLOW_SOFT_OUTER_SPEED_MM_S LINE_CURVE_MAX_SPEED_MM_S
#define LINE_CURVE_SLOW_MEDIUM_INNER_SPEED_MM_S (0)
#define LINE_CURVE_SLOW_MEDIUM_OUTER_SPEED_MM_S LINE_CURVE_MAX_SPEED_MM_S
#define LINE_CURVE_SLOW_HARD_INNER_SPEED_MM_S (0)
#define LINE_CURVE_SLOW_HARD_OUTER_SPEED_MM_S LINE_CURVE_MAX_SPEED_MM_S
#define LINE_SOFT_TURN_INNER_SPEED_MM_S    (40)
#define LINE_SOFT_TURN_OUTER_SPEED_MM_S    (100)
#define LINE_MEDIUM_TURN_INNER_SPEED_MM_S  (0)
#define LINE_MEDIUM_TURN_OUTER_SPEED_MM_S  (85)
#define LINE_HARD_TURN_INNER_SPEED_MM_S    (0)
#define LINE_HARD_TURN_OUTER_SPEED_MM_S    (70)
#define LINE_LOST_RECOVERY_INNER_SPEED_MM_S (0)
#define LINE_LOST_RECOVERY_OUTER_SPEED_MM_S (45)
#define LINE_LOST_RIGHT_SEARCH_INNER_SPEED_MM_S (0)
#define LINE_LOST_RIGHT_SEARCH_OUTER_SPEED_MM_S LINE_CURVE_MAX_SPEED_MM_S
#define LINE_LOST_REACQUIRE_STABLE_CYCLES  (3U)
#define LINE_PD_TRIM_DIVISOR               (4)
#define LINE_LOST_FORWARD_CYCLES           (8U)
#define LINE_TURN_INNER_SPEED_MM_S         (0)
#define LINE_TURN_OUTER_SPEED_MM_S         LINE_MEDIUM_TURN_OUTER_SPEED_MM_S

/*
 * Sensor scores for state-machine tracking. Left score and right score are
 * compared; negative error means the line is left, positive means right.
 */
#define LINE_SCORE_X1                      (4U)
#define LINE_SCORE_X2                      (3U)
#define LINE_SCORE_X3                      (2U)
#define LINE_SCORE_X4                      (1U)
#define LINE_SCORE_X5                      (1U)
#define LINE_SCORE_X6                      (2U)
#define LINE_SCORE_X7                      (3U)
#define LINE_SCORE_X8                      (4U)

#endif

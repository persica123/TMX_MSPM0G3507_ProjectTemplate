#ifndef RACE_LAPS_H
#define RACE_LAPS_H

/**
 * @file race_laps.h
 * @brief 任务三/任务四跑圈顶层编排。
 *
 * 本头文件在 main.c 声明竞速配置类型和辅助钩子后引入。底层转向、
 * 前进、找线等动作在 race_primitives.h，中层阶段状态机在
 * race_phase.h。
 */

#include <stdint.h>

#include "app_config.h"
#include "app_control.h"
#include "app_motion_utils.h"
#include "app_services.h"
#include "app_straight.h"
#include "board.h"
#include "bsp_encoder.h"
#include "bsp_ir_tracking.h"
#include "bsp_jy62.h"
#include "bsp_oled.h"
#include "bsp_tb6612.h"

/**
 * @brief 竞速阶段在点位或保护距离结束时保存的结果快照。
 */
typedef struct {
    uint8_t reason;
    uint32_t elapsed_ms;
    int32_t distance_count;
    int32_t yaw_cdeg;
    int32_t yaw_progress_cdeg;
    uint8_t ir_ok;
    uint8_t nav_ok;
    ir_tracking_sample_t sample;
} line_result_t;

/**
 * @brief 当前 AC/CB/BD/DA 阶段的静态参数。
 */
typedef struct {
    const char *phase_name;
    const char *point_name;
    const char *force_name;
    int32_t point_arm_count;
    int32_t force_count;
    int32_t phase_turn_dir;
    int32_t straight_target_cdeg;
    uint8_t arc_mode;
} race_phase_config_t;

/**
 * @brief 阶段辅助函数共享的完整竞速运行期状态。
 */
typedef struct {
    line_result_t result;
    ir_tracking_sample_t sample;
    jy62_navigation_t nav;
    straight_pid_t diff_pid;
    straight_drive_config_t drive_config;
    straight_drive_output_t drive;
    uint32_t elapsed_ms;
    uint32_t phase_start_ms;
    uint32_t report_elapsed_ms;
    uint32_t nav_frame_delta;
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t motor_b_total;
    int32_t motor_a_total;
    int32_t total_distance_count;
    int32_t phase_distance_count;
    int32_t phase_start_count;
    int32_t phase_start_calibration_count;
    int32_t lap_start_calibration_count;
    int32_t filtered_error;
    int32_t last_filtered_error;
    int32_t filtered_derivative;
    int32_t last_turn;
    int32_t yaw_start;
    int32_t yaw_cdeg;
    int32_t yaw_raw_cdeg;
    int32_t phase_yaw_cdeg;
    int32_t gyro_z_mdps;
    int32_t gyro_z_filtered_mdps;
    int32_t roll_cdeg;
    int32_t pitch_cdeg;
    int32_t yaw_progress_cdeg;
    int32_t raw_error;
    int32_t derivative;
    int32_t base_pwm;
    int32_t left_pwm;
    int32_t right_pwm;
    int32_t target_speed_diff;
    int32_t line_turn;
    int32_t nav_turn;
    int32_t control_turn;
    int32_t heading_error_cdeg;
    int32_t expected_yaw_cdeg;
    int32_t arc_actual_yaw_cdeg;
    uint8_t lap_count;
    uint8_t phase;
    uint8_t target_laps;
    uint8_t nav_ok;
    uint8_t nav_update_flags;
    uint8_t straight_point_count;
    uint16_t straight_line_seen_count;
    uint8_t ir_ok;
    uint8_t line_valid;
    uint8_t line_lost_seen;
    uint8_t line_lost_count;
    uint8_t line_control_seeded;
    uint8_t straight_point_candidate;
    uint8_t edge_point_seen;
    uint8_t point_ready;
    uint8_t stop_reason;
} race_context_t;

/**
 * @brief 将内部停止/点位原因码转换为日志文本。
 */
static const char *race_reason_name(uint8_t reason)
{
    if (reason == 0U) {
        return "none";
    }
    if (reason == 1U) {
        return "point";
    }
    if (reason == 2U) {
        return "force";
    }
    if (reason == 3U) {
        return "uart_stop";
    }
    if (reason == 5U) {
        return "nav_invalid";
    }
    if (reason == 6U) {
        return "yaw";
    }
    return "timeout";
}

/**
 * @brief 将阶段序号 0..3 转为 AC/CB/BD/DA 文本。
 */
static const char *race_phase_name(uint8_t phase)
{
    if (phase == 0U) {
        return "AC";
    }
    if (phase == 1U) {
        return "CB";
    }
    if (phase == 2U) {
        return "BD";
    }
    return "DA";
}

#include "race_primitives.h"
#include "race_phase.h"

/**
 * @brief 任务二 BC 弧线退出后专用的 CD 固定航向直线段。
 */
static uint8_t run_task2_cd_exit_angle_straight(const char *tag)
{
    const straight_line_segment_config_t config = {
        .tag = tag,
        .zero_heading = 0U,
        .start_alarm_ms = 0U,
        .stop_alarm_ms = 0U,
        .line_arm_count = TASK2_STRAIGHT_SEARCH_START_COUNT,
        .force_stop_count = RACE_STRAIGHT_FORCE_COUNT,
        .stop_min_ir_count = TASK1_STOP_MIN_IR_COUNT,
        .yaw_corr_enable = 1U,
        .task2_ab_yaw_tuning = 0U,
        .gray_guide_enable = TASK2_CD_GRAY_GUIDE_ENABLE,
        .gray_guide_max_active_count = TASK2_CD_GRAY_GUIDE_MAX_ACTIVE_COUNT,
        .gray_guide_deadband = TASK2_CD_GRAY_GUIDE_DEADBAND,
        .gray_guide_divisor = TASK2_CD_GRAY_GUIDE_DIVISOR,
        .gray_guide_corr_max = TASK2_CD_GRAY_GUIDE_CORR_MAX,
        .entry_brake_enable = 0U,
        .fixed_yaw_target_enable = 1U,
        .fixed_yaw_target_cdeg = TASK2_CD_STRAIGHT_TARGET_CDEG,
        .entry_b_pwm = TASK2_CD_ENTRY_B_PWM,
        .entry_a_pwm = TASK2_CD_ENTRY_A_PWM,
        .entry_ramp_ms = TASK2_CD_ENTRY_RAMP_MS,
        .pwm_percent = TASK2_STRAIGHT_PWM_PERCENT
    };

    return run_straight_to_line_segment(&config);
}

/**
 * @brief 为任务二单独启动一个竞速弧线阶段。
 *
 * 任务二只复用任务三/四的弧线巡线、航向辅助和出弧点判断，不执行竞速
 * 阶段点位后的快速转向动作。这样可以保持任务二路线为 AB 直行、BC 弧线、
 * CD 直行、DA 弧线。
 */
static void task2_prepare_race_arc_context(race_context_t *ctx,
    uint8_t race_phase)
{
    ctx->target_laps = 1U;
    ctx->lap_count = 0U;
    ctx->phase = race_phase;
    ctx->phase_start_ms = 0U;
    ctx->phase_start_count = 0;

    /*
     * 单段弧线从当前姿态和当前编码器距离重新开始计量，避免继承 AB/CD
     * 直线段的距离和滤波状态。
     */
    IRTracking_Init();
    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    race_reset_segment_control(ctx);
    race_read_navigation_state(ctx, 1U);
}

/**
 * @brief 运行一个复用竞速状态机的任务二弧线段，直到正常出弧或保护退出。
 */
static void task2_apply_arc_follow_control(race_context_t *ctx,
    uint8_t exit_decel_enable,
    uint8_t entry_slow_enable)
{
    uint8_t task3_da_mode = ((ctx->target_laps == 1U) &&
        (ctx->phase == 3U)) ? 1U : 0U;
    int32_t line_turn;
    int32_t line_derivative;
    int32_t control_turn;
    int32_t base_b_pwm;
    int32_t base_a_pwm;
    int32_t line_error;
    int32_t filtered_line_error;
    int32_t filter_delta;
    int32_t filter_step;
    int32_t error_delta;
    int32_t error_sign = (task3_da_mode != 0U) ?
        TASK3_DA_ERROR_SIGN : 1;
    int32_t pwm_percent = (task3_da_mode != 0U) ?
        TASK3_DA_PWM_PERCENT : TASK2_ARC_PWM_PERCENT;
    int32_t turn_boost_percent = (task3_da_mode != 0U) ?
        TASK3_DA_LINE_TURN_BOOST_PERCENT : TASK2_ARC_TURN_BOOST_PERCENT;
    int32_t line_kp_num = (task3_da_mode != 0U) ?
        TASK3_DA_LINE_KP_NUM : TASK2_ARC_LINE_KP_NUM;
    int32_t line_kp_den = (task3_da_mode != 0U) ?
        TASK3_DA_LINE_KP_DEN : TASK2_ARC_LINE_KP_DEN;
    int32_t line_kd_num = (task3_da_mode != 0U) ?
        TASK3_DA_LINE_KD_NUM : TASK2_ARC_LINE_KD_NUM;
    int32_t line_kd_den = (task3_da_mode != 0U) ?
        TASK3_DA_LINE_KD_DEN : TASK2_ARC_LINE_KD_DEN;
    int32_t line_turn_limit = (task3_da_mode != 0U) ?
        TASK3_DA_LINE_TURN_LIMIT : TASK2_ARC_LINE_TURN_LIMIT;
    int32_t control_turn_limit = (task3_da_mode != 0U) ?
        TASK3_DA_CONTROL_TURN_LIMIT : TASK2_ARC_CONTROL_TURN_LIMIT;
    int32_t error_deadband = (task3_da_mode != 0U) ?
        TASK3_DA_LINE_ERROR_DEADBAND : TASK2_ARC_LINE_ERROR_DEADBAND;
    int32_t error_filter_divisor = (task3_da_mode != 0U) ?
        TASK3_DA_LINE_FILTER_DIVISOR : TASK2_ARC_LINE_FILTER_DIVISOR;
    int32_t error_max_active_count = (task3_da_mode != 0U) ?
        TASK3_DA_ERROR_MAX_ACTIVE_COUNT : TASK2_ARC_ERROR_MAX_ACTIVE_COUNT;
    int32_t error_jump_limit = (task3_da_mode != 0U) ?
        TASK3_DA_ERROR_JUMP_LIMIT : TASK2_ARC_ERROR_JUMP_LIMIT;
    int32_t deriv_limit = (task3_da_mode != 0U) ?
        TASK3_DA_DERIV_LIMIT : RACE_LINE_DERIV_LIMIT;
    int32_t deriv_filter_divisor = (task3_da_mode != 0U) ?
        TASK3_DA_DERIV_FILTER_DIVISOR : RACE_LINE_DERIV_FILTER_DIVISOR;
    int32_t turn_slew_step = (task3_da_mode != 0U) ?
        TASK3_DA_TURN_SLEW_STEP : RACE_LINE_TURN_SLEW_STEP;
    int32_t lost_turn_decay_step = (task3_da_mode != 0U) ?
        TASK3_DA_LOST_TURN_DECAY_STEP : RACE_LINE_LOST_TURN_DECAY_STEP;
    int32_t entry_count = (task3_da_mode != 0U) ?
        TASK3_DA_ENTRY_COUNT : TASK2_DA_ENTRY_COUNT;
    int32_t entry_pwm_percent = (task3_da_mode != 0U) ?
        TASK3_DA_ENTRY_PWM_PERCENT : TASK2_DA_ENTRY_PWM_PERCENT;
    int32_t decel_start_count = (task3_da_mode != 0U) ?
        TASK3_DA_DECEL_START_COUNT : TASK2_BC_EXIT_DECEL_START_COUNT;
    int32_t decel_pwm_percent = (task3_da_mode != 0U) ?
        TASK3_DA_DECEL_PWM_PERCENT : TASK2_BC_EXIT_PWM_PERCENT;

    if ((ctx->line_valid != 0U) &&
        (ctx->sample.active_count <= error_max_active_count)) {
        line_error = (abs_i32(ctx->sample.error) <=
            error_deadband) ? 0 : (ctx->sample.error * error_sign);
        error_delta = line_error - ctx->filtered_error;
        line_error = ctx->filtered_error + clamp_i32(error_delta,
            -error_jump_limit,
            error_jump_limit);
        filter_delta = line_error - ctx->filtered_error;
        filter_step = race_filter_step(filter_delta,
            error_filter_divisor);
        ctx->filtered_error += filter_step;
        filtered_line_error = ctx->filtered_error;
        line_derivative = clamp_i32(filtered_line_error - ctx->last_filtered_error,
            -deriv_limit,
            deriv_limit);
        ctx->filtered_derivative += race_filter_step(line_derivative -
            ctx->filtered_derivative, deriv_filter_divisor);
        line_turn =
            ((filtered_line_error * line_kp_num) / line_kp_den) +
            ((ctx->filtered_derivative * line_kd_num) / line_kd_den);
        line_turn = clamp_i32(line_turn,
            -line_turn_limit,
            line_turn_limit);
        ctx->last_filtered_error = filtered_line_error;
        ctx->line_lost_count = 0U;
    } else if (ctx->last_turn != 0) {
        if (ctx->line_lost_count < 255U) {
            ctx->line_lost_count++;
        }
        line_turn = race_move_towards(ctx->last_turn, 0,
            lost_turn_decay_step);
    } else {
        line_turn = 0;
    }

    line_turn = race_move_towards(ctx->last_turn, line_turn,
        turn_slew_step);
    ctx->last_turn = line_turn;

    control_turn = clamp_i32(line_turn + ctx->nav_turn,
        -control_turn_limit,
        control_turn_limit);
    control_turn = (control_turn * turn_boost_percent) / 100;

    base_b_pwm = (ctx->drive.motor_b_pwm * pwm_percent) / 100;
    base_a_pwm = (ctx->drive.motor_a_pwm * pwm_percent) / 100;
    if ((entry_slow_enable != 0U) &&
        (ctx->phase_distance_count < entry_count)) {
        base_b_pwm = (base_b_pwm * entry_pwm_percent) / 100;
        base_a_pwm = (base_a_pwm * entry_pwm_percent) / 100;
    }
    if ((exit_decel_enable != 0U) &&
        (ctx->phase_distance_count >= decel_start_count)) {
        base_b_pwm = (base_b_pwm * decel_pwm_percent) / 100;
        base_a_pwm = (base_a_pwm * decel_pwm_percent) / 100;
    }

    ctx->line_turn = line_turn;
    ctx->control_turn = control_turn;
    ctx->left_pwm = clamp_i32(base_b_pwm + control_turn,
        RACE_LINE_MIN_PWM,
        RACE_LINE_MAX_PWM);
    ctx->right_pwm = clamp_i32(base_a_pwm - control_turn,
        RACE_LINE_MIN_PWM,
        RACE_LINE_MAX_PWM);
}

/**
 * @brief DA 首次拿到有效线位时，用真实灰度误差直接初始化循迹状态。
 *
 * D 点转向结束后阶段复位会把滤波误差和 last_turn 清零。如果 DA 已经以
 * 较大偏差接管，继续从零滤波和限速爬升会来不及纠偏。这里仅在 DA 的
 * 第一帧可用窄线出现时执行一次，后续仍完全使用 CB 共用的灰度 PD。
 */
static void task3_seed_da_follow_control(race_context_t *ctx)
{
    int32_t seed_error;
    int32_t seed_turn;

    if ((ctx->phase != 3U) || (ctx->line_control_seeded != 0U) ||
        (ctx->line_valid == 0U) ||
        (ctx->sample.active_count > TASK3_DA_ERROR_MAX_ACTIVE_COUNT)) {
        return;
    }

    seed_error = (abs_i32(ctx->sample.error) <=
        TASK3_DA_LINE_ERROR_DEADBAND) ? 0 :
        (ctx->sample.error * TASK3_DA_ERROR_SIGN);
    seed_error = clamp_i32(seed_error,
        -TASK3_DA_ENTRY_SEED_ERROR_LIMIT,
        TASK3_DA_ENTRY_SEED_ERROR_LIMIT);
    seed_turn = ((seed_error * TASK3_DA_LINE_KP_NUM) /
        TASK3_DA_LINE_KP_DEN);
    seed_turn = clamp_i32(seed_turn,
        -TASK3_DA_ENTRY_SEED_TURN_LIMIT,
        TASK3_DA_ENTRY_SEED_TURN_LIMIT);

    ctx->filtered_error = seed_error;
    ctx->last_filtered_error = seed_error;
    ctx->filtered_derivative = 0;
    ctx->last_turn = seed_turn;
    ctx->line_lost_count = 0U;
    ctx->line_control_seeded = 1U;
}

/*
 * 第四问专用圆弧循迹：代码结构复制任务三实际调用的
 * task2_apply_arc_follow_control()，但所有参数均来自 RACE_TASK4_*。
 */
static void task4_apply_arc_follow_control(race_context_t *ctx,
    uint8_t exit_decel_enable,
    uint8_t entry_slow_enable)
{
    uint8_t da_mode = (ctx->phase == 3U) ? 1U : 0U;
    int32_t line_turn;
    int32_t line_derivative;
    int32_t control_turn;
    int32_t base_b_pwm;
    int32_t base_a_pwm;
    int32_t line_error;
    int32_t filtered_line_error;
    int32_t filter_delta;
    int32_t filter_step;
    int32_t error_delta;
    int32_t error_sign = (da_mode != 0U) ?
        RACE_TASK4_DA_ERROR_SIGN : RACE_TASK4_CB_ERROR_SIGN;

    if ((ctx->line_valid != 0U) &&
        (ctx->sample.active_count <=
            RACE_TASK4_ARC_ERROR_MAX_ACTIVE_COUNT)) {
        line_error = (abs_i32(ctx->sample.error) <=
            RACE_TASK4_ARC_LINE_ERROR_DEADBAND) ? 0 :
            (ctx->sample.error * error_sign);
        error_delta = line_error - ctx->filtered_error;
        line_error = ctx->filtered_error + clamp_i32(error_delta,
            -RACE_TASK4_ARC_ERROR_JUMP_LIMIT,
            RACE_TASK4_ARC_ERROR_JUMP_LIMIT);
        filter_delta = line_error - ctx->filtered_error;
        filter_step = race_filter_step(filter_delta,
            RACE_TASK4_ARC_LINE_FILTER_DIVISOR);
        ctx->filtered_error += filter_step;
        filtered_line_error = ctx->filtered_error;
        line_derivative = clamp_i32(
            filtered_line_error - ctx->last_filtered_error,
            -RACE_TASK4_ARC_DERIV_LIMIT,
            RACE_TASK4_ARC_DERIV_LIMIT);
        ctx->filtered_derivative += race_filter_step(line_derivative -
            ctx->filtered_derivative,
            RACE_TASK4_ARC_DERIV_FILTER_DIVISOR);
        line_turn =
            ((filtered_line_error * RACE_TASK4_ARC_LINE_KP_NUM) /
                RACE_TASK4_ARC_LINE_KP_DEN) +
            ((ctx->filtered_derivative * RACE_TASK4_ARC_LINE_KD_NUM) /
                RACE_TASK4_ARC_LINE_KD_DEN);
        line_turn = clamp_i32(line_turn,
            -RACE_TASK4_ARC_LINE_TURN_LIMIT,
            RACE_TASK4_ARC_LINE_TURN_LIMIT);
        ctx->last_filtered_error = filtered_line_error;
        ctx->line_lost_count = 0U;
    } else if (ctx->last_turn != 0) {
        if (ctx->line_lost_count < 255U) {
            ctx->line_lost_count++;
        }
        line_turn = race_move_towards(ctx->last_turn, 0,
            RACE_TASK4_ARC_LOST_TURN_DECAY_STEP);
    } else {
        line_turn = 0;
    }

    line_turn = race_move_towards(ctx->last_turn, line_turn,
        RACE_TASK4_ARC_TURN_SLEW_STEP);
    ctx->last_turn = line_turn;

    control_turn = clamp_i32(line_turn + ctx->nav_turn,
        -RACE_TASK4_ARC_CONTROL_TURN_LIMIT,
        RACE_TASK4_ARC_CONTROL_TURN_LIMIT);
    control_turn = (control_turn *
        RACE_TASK4_ARC_LINE_TURN_BOOST_PERCENT) / 100;

    base_b_pwm = (ctx->drive.motor_b_pwm *
        RACE_TASK4_ARC_PWM_PERCENT) / 100;
    base_a_pwm = (ctx->drive.motor_a_pwm *
        RACE_TASK4_ARC_PWM_PERCENT) / 100;
    if ((entry_slow_enable != 0U) &&
        (ctx->phase_distance_count < RACE_TASK4_DA_ENTRY_COUNT)) {
        base_b_pwm = (base_b_pwm *
            RACE_TASK4_DA_ENTRY_PWM_PERCENT) / 100;
        base_a_pwm = (base_a_pwm *
            RACE_TASK4_DA_ENTRY_PWM_PERCENT) / 100;
    }
    if ((exit_decel_enable != 0U) &&
        (ctx->phase_distance_count >= RACE_TASK4_ARC_DECEL_START_COUNT)) {
        base_b_pwm = (base_b_pwm *
            RACE_TASK4_ARC_DECEL_PWM_PERCENT) / 100;
        base_a_pwm = (base_a_pwm *
            RACE_TASK4_ARC_DECEL_PWM_PERCENT) / 100;
    }

    ctx->line_turn = line_turn;
    ctx->control_turn = control_turn;
    ctx->left_pwm = clamp_i32(base_b_pwm + control_turn,
        RACE_TASK4_LINE_MIN_PWM,
        RACE_TASK4_LINE_MAX_PWM);
    ctx->right_pwm = clamp_i32(base_a_pwm - control_turn,
        RACE_TASK4_LINE_MIN_PWM,
        RACE_TASK4_LINE_MAX_PWM);
}

/* 第四问专用 DA 首帧误差初始化，数值复制任务三但互不引用。 */
static void task4_seed_da_follow_control(race_context_t *ctx)
{
    int32_t seed_error;
    int32_t seed_turn;

    if ((ctx->phase != 3U) || (ctx->line_control_seeded != 0U) ||
        (ctx->line_valid == 0U) ||
        (ctx->sample.active_count >
            RACE_TASK4_ARC_ERROR_MAX_ACTIVE_COUNT)) {
        return;
    }

    seed_error = (abs_i32(ctx->sample.error) <=
        RACE_TASK4_ARC_LINE_ERROR_DEADBAND) ? 0 :
        (ctx->sample.error * RACE_TASK4_DA_ERROR_SIGN);
    seed_error = clamp_i32(seed_error,
        -RACE_TASK4_DA_ENTRY_SEED_ERROR_LIMIT,
        RACE_TASK4_DA_ENTRY_SEED_ERROR_LIMIT);
    seed_turn = ((seed_error * RACE_TASK4_ARC_LINE_KP_NUM) /
        RACE_TASK4_ARC_LINE_KP_DEN);
    seed_turn = clamp_i32(seed_turn,
        -RACE_TASK4_DA_ENTRY_SEED_TURN_LIMIT,
        RACE_TASK4_DA_ENTRY_SEED_TURN_LIMIT);

    ctx->filtered_error = seed_error;
    ctx->last_filtered_error = seed_error;
    ctx->filtered_derivative = 0;
    ctx->last_turn = seed_turn;
    ctx->line_lost_count = 0U;
    ctx->line_control_seeded = 1U;
}

/*
 * BC 刚出弧时航向角可能已到目标，但车身仍带有较大的角速度。保持低速
 * 航向闭环数个周期后再交给 CD，避免直线段一开始就因惯性横摆而偏离。
 */
static void task2_stabilize_bc_exit(void)
{
    uint32_t elapsed_ms = 0U;
    uint8_t stable_count = 0U;
    jy62_navigation_t nav = {0};

    while (elapsed_ms < TASK2_BC_EXIT_STABILIZE_MAX_MS) {
        int32_t heading_error_cdeg;
        int32_t turn;
        int32_t left_pwm;
        int32_t right_pwm;

        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        (void)JY62_GetNavigation(&nav);

        if (nav.valid == 0U) {
            break;
        }

        heading_error_cdeg = normalize_cdeg(nav.yaw_relative_cdeg -
            TASK2_BC_EXIT_YAW_TARGET_CDEG);
        turn = race_heading_turn_from_error(heading_error_cdeg,
            nav.gyro_z_filtered_mdps,
            TASK2_CD_HEADING_CORR_DIVISOR,
            TASK2_CD_HEADING_GYRO_DAMP_DIVISOR,
            TASK2_CD_HEADING_CORR_MAX);
        left_pwm = clamp_i32(TASK2_BC_EXIT_STABILIZE_PWM + turn,
            RACE_LINE_MIN_PWM,
            RACE_LINE_MAX_PWM);
        right_pwm = clamp_i32(TASK2_BC_EXIT_STABILIZE_PWM - turn,
            RACE_LINE_MIN_PWM,
            RACE_LINE_MAX_PWM);
        TB6612_SetDifferential((int16_t)left_pwm, (int16_t)right_pwm);

        if ((abs_i32(heading_error_cdeg) <=
                TASK2_BC_EXIT_STABILIZE_YAW_TOL_CDEG) &&
            (abs_i32(nav.gyro_z_filtered_mdps) <=
                TASK2_BC_EXIT_STABILIZE_GYRO_TOL_MDPS)) {
            if (stable_count < 255U) {
                stable_count++;
            }
            if (stable_count >= TASK2_BC_EXIT_STABILIZE_CONFIRM_COUNT) {
                break;
            }
        } else {
            stable_count = 0U;
        }
    }
}

static uint8_t run_task2_race_arc_phase(const char *tag,
    uint8_t race_phase,
    uint8_t exit_yaw_gate_enable,
    uint8_t final_finish_gate_enable)
{
    race_context_t ctx = {0};
    race_phase_config_t phase_config;
    uint32_t oled_elapsed_ms = OLED_REFRESH_MIN_MS;
    uint8_t phase = (race_phase == 1U) ? 1U : 3U;

    /*
     * 只允许复用竞速 CB(左弧) 或 DA(右弧) 两种弧线阶段。
     * 当前任务二车场路线的 BC/DA 都按右弧线接入，因此默认映射到 DA。
     */
    task2_prepare_race_arc_context(&ctx, phase);
    race_configure_phase(&ctx, &phase_config);
    /* 覆盖日志名称，让串口输出仍然能直接看出这是任务二的 BC/DA 段。 */
    phase_config.phase_name = tag;
    phase_config.point_name = (phase == 1U) ? "TASK2_C_EXIT" : "TASK2_A_EXIT";
    phase_config.force_name = (phase == 1U) ? "TASK2_C_FORCE" : "TASK2_A_FORCE";
    phase_config.point_arm_count = TASK2_ARC_EXIT_ARM_COUNT;
    phase_config.force_count = TASK2_ARC_FORCE_STOP_COUNT;

    race_log_printf("%s start: reuse_race_arc src=%s turn=%ld yaw0=%ld nav=%u\r\n",
        tag,
        race_phase_name(phase),
        phase_config.phase_turn_dir,
        ctx.yaw_start,
        ctx.nav_ok);

    if (final_finish_gate_enable != 0U) {
        /* 先覆盖 CD 残留直线 PWM，避免 DA 的第一控制周期过快。 */
        TB6612_SetDifferential((int16_t)TASK2_DA_ENTRY_B_PWM,
            (int16_t)TASK2_DA_ENTRY_A_PWM);
    }

    while (1) {
#if !(APP_HAND_PUSH_CALIBRATION_MODE && APP_HAND_PUSH_DISABLE_TIMEOUT)
        if (ctx.elapsed_ms >= TASK3_ARC_MAX_RUN_MS) {
            break;
        }
#endif
        delay_ms_with_st011(CONTROL_PERIOD_MS);
        ctx.elapsed_ms += CONTROL_PERIOD_MS;
        ctx.report_elapsed_ms += CONTROL_PERIOD_MS;
        oled_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            ctx.stop_reason = 3U;
            break;
        }

        race_update_loop_state(&ctx, &phase_config);
        race_compute_loop_control(&ctx, &phase_config, 0U, 0U);
        /*
         * BC、DA 都采用同一套灰度 PD、同一限幅和同一出弧降速曲线。
         * 两段只允许在“何时结束”上不同，不能让终点判定反过来改变
         * 循迹时的轮速与转向余量。
         */
        task2_apply_arc_follow_control(&ctx, 1U, final_finish_gate_enable);
        int32_t total_distance_count = encoder_get_calibration_distance_count();
        if (oled_elapsed_ms >= OLED_REFRESH_MIN_MS) {
            OLED_ShowYawDistance(ctx.yaw_cdeg,
                total_distance_count / COUNTS_PER_CM,
                ctx.nav_ok);
            oled_elapsed_ms = 0U;
        }

        uint8_t point_ready;

        if (final_finish_gate_enable != 0U) {
            point_ready = ((total_distance_count >= TASK2_FINAL_FINISH_COUNT) &&
                (ctx.phase_distance_count >= TASK2_FINAL_DA_MIN_COUNT) &&
                (ctx.ir_ok != 0U) &&
                (ctx.sample.line_lost != 0U)) ? 1U : 0U;
        } else if (exit_yaw_gate_enable != 0U) {
            int32_t yaw_error_cdeg = abs_i32(normalize_cdeg(ctx.yaw_cdeg -
                TASK2_BC_EXIT_YAW_TARGET_CDEG));

            point_ready =
                ((ctx.phase_distance_count >= TASK2_ARC_EXIT_ARM_COUNT) &&
                 ((TASK2_BC_EXIT_YAW_GATE_ENABLE == 0) ||
                  (yaw_error_cdeg <= TASK2_BC_EXIT_YAW_TOLERANCE_CDEG))) ? 1U : 0U;

            if ((point_ready == 0U) &&
                (ctx.phase_distance_count >= phase_config.force_count) &&
                (yaw_error_cdeg <= TASK2_BC_EXIT_FORCE_YAW_TOLERANCE_CDEG)) {
                point_ready = 1U;
            }

            if ((point_ready == 0U) &&
                (ctx.phase_distance_count >= TASK2_BC_EXIT_HARD_COUNT)) {
                point_ready = 1U;
            }
        } else {
            point_ready = race_check_phase_point(&ctx, &phase_config);
        }

        if (point_ready != 0U) {
            /*
             * 任务三/四在点位后会执行快速转向或前推；任务二这里只需要
             * “出弧成功”这个事件，后续动作由 run_task2_abcd() 按路线编排。
             */
            race_capture_result(&ctx, 1U);
            race_log_point_state(&ctx,
                &phase_config,
                ctx.result.reason,
                1U);
            ctx.stop_reason = 1U;
            break;
        }

        if ((final_finish_gate_enable == 0U) &&
            (exit_yaw_gate_enable == 0U) &&
            (ctx.phase_distance_count >= phase_config.force_count)) {
            race_capture_result(&ctx, 2U);
            race_log_point_state(&ctx,
                &phase_config,
                ctx.result.reason,
                0U);
            ctx.stop_reason = 2U;
            break;
        }

        TB6612_SetDifferential((int16_t)ctx.left_pwm,
            (int16_t)ctx.right_pwm);
        race_log_periodic_data(&ctx, &phase_config);
    }

    if (ctx.stop_reason == 0U) {
        ctx.stop_reason = 4U;
    }
    if (ctx.stop_reason != 1U) {
        TB6612_Brake();
    }

    race_log_printf("%s stop: reason=%s src=%s t=%lu dist=%ld yaw=%ld yprog=%ld nav=%u ir=%u raw=0x%02X mask=0x%02X cnt=%u\r\n",
        tag,
        race_reason_name(ctx.stop_reason),
        race_phase_name(phase),
        ctx.elapsed_ms,
        ctx.phase_distance_count,
        ctx.yaw_cdeg,
        ctx.yaw_progress_cdeg,
        ctx.nav_ok,
        ctx.ir_ok,
        (ctx.ir_ok != 0U) ? ctx.sample.raw : 0xFFU,
        (ctx.ir_ok != 0U) ? ctx.sample.line_mask : 0U,
        (ctx.ir_ok != 0U) ? ctx.sample.active_count : 0U);

    return ctx.stop_reason;
}

/**
 * @brief 任务二 BC 弧线，按当前车场路线复用竞速 DA 右弧线控制。
 */
static uint8_t run_task2_bc_race_arc(const char *tag)
{
    uint8_t reason = run_task2_race_arc_phase(tag, 3U, 1U, 0U);

    if (reason == 1U) {
        task2_stabilize_bc_exit();
    }
    return reason;
}

/**
 * @brief 任务二 DA 弧线，直接复用竞速 DA 右弧线控制。
 */
static uint8_t run_task2_da_race_arc(const char *tag)
{
    return run_task2_race_arc_phase(tag, 3U, 0U, 1U);
}

/* ===========================================================================
 * 第四问独立副本：任务三完整控制逻辑，唯一区别为连续四圈
 * ========================================================================== */

static void race_task4_configure_phase(const race_context_t *ctx,
    race_phase_config_t *config)
{
    if (ctx->phase == 0U) {
        config->phase_name = "T4_AC";
        config->point_name = "T4_C_LINE";
        config->force_name = "T4_C_FORCE";
        config->point_arm_count = RACE_TASK4_AC_POINT_ARM_COUNT;
        config->force_count = RACE_TASK4_AC_FORCE_TURN_COUNT;
        config->phase_turn_dir = 0;
        config->straight_target_cdeg = RACE_TASK4_AC_HEADING_TARGET_CDEG;
        config->arc_mode = 0U;
    } else if (ctx->phase == 1U) {
        config->phase_name = "T4_CB";
        config->point_name = "T4_B_EXIT";
        config->force_name = "T4_B_FORCE";
        config->point_arm_count = RACE_TASK4_ARC_EXIT_IGNORE_COUNT;
        config->force_count = RACE_TASK4_ARC_FORCE_STOP_COUNT;
        config->phase_turn_dir = RACE_TASK4_ARC_TURN_LEFT;
        config->straight_target_cdeg = 0;
        config->arc_mode = 1U;
    } else if (ctx->phase == 2U) {
        config->phase_name = "T4_BD";
        config->point_name = "T4_D_LINE";
        config->force_name = "T4_D_FORCE";
        config->point_arm_count = RACE_TASK4_BD_POINT_ARM_COUNT;
        config->force_count = RACE_TASK4_D_BRAKE_DISTANCE_COUNT;
        config->phase_turn_dir = 0;
        config->straight_target_cdeg = RACE_TASK4_BD_HEADING_TARGET_CDEG;
        config->arc_mode = 0U;
    } else {
        config->phase_name = "T4_DA";
        config->point_name = ((uint8_t)(ctx->lap_count + 1U) <
            ctx->target_laps) ? "T4_A_EXIT" : "T4_A_FINISH";
        config->force_name = "T4_A_FORCE";
        config->point_arm_count = RACE_TASK4_ARC_EXIT_IGNORE_COUNT;
        config->force_count = RACE_TASK4_ARC_FORCE_STOP_COUNT;
        config->phase_turn_dir = RACE_TASK4_ARC_TURN_RIGHT;
        config->straight_target_cdeg = 0;
        config->arc_mode = 1U;
    }
}

static uint8_t race_task4_check_phase_point(race_context_t *ctx,
    const race_phase_config_t *config)
{
    int32_t lap_distance_count = encoder_get_calibration_distance_count() -
        ctx->lap_start_calibration_count;

    if (ctx->phase == 0U) {
        ctx->straight_point_candidate =
            ((ctx->phase_distance_count >= config->point_arm_count) &&
             (ctx->line_valid != 0U)) ? 1U : 0U;
        if (ctx->straight_point_candidate != 0U) {
            if (ctx->straight_point_count < 1U) {
                ctx->straight_point_count++;
            }
        } else {
            ctx->straight_point_count = 0U;
        }
        ctx->point_ready = (ctx->straight_point_count >= 1U) ? 1U : 0U;
    } else if (ctx->phase == 1U) {
        ctx->straight_point_candidate = ((lap_distance_count >=
                RACE_TASK4_B_EXIT_DISTANCE_COUNT) &&
            (ctx->line_lost_seen != 0U)) ? 1U : 0U;
        if (ctx->straight_point_candidate != 0U) {
            if (ctx->straight_point_count < 255U) {
                ctx->straight_point_count++;
            }
        } else {
            ctx->straight_point_count = 0U;
        }
        ctx->point_ready = (ctx->straight_point_count >=
            RACE_TASK4_B_LINE_LOST_CONFIRM_CYCLES) ? 1U : 0U;
    } else if (ctx->phase == 2U) {
        ctx->point_ready = (lap_distance_count >=
            RACE_TASK4_D_BRAKE_DISTANCE_COUNT) ? 1U : 0U;
    } else {
        ctx->point_ready = ((lap_distance_count >=
                RACE_TASK4_A_FINISH_DISTANCE_COUNT) &&
            (ctx->line_lost_seen != 0U)) ? 1U : 0U;
    }

    return ctx->point_ready;
}

static uint8_t race_task4_check_straight_force_turn(
    const race_context_t *ctx,
    const race_phase_config_t *config)
{
    if (config->arc_mode != 0U) {
        return 0U;
    }
    if (ctx->phase == 0U) {
        return (ctx->phase_distance_count >=
            RACE_TASK4_AC_FORCE_TURN_COUNT) ? 1U : 0U;
    }
    /* D 点严格使用逐圈 Dis=345 cm，不允许保护距离抢先触发。 */
    return 0U;
}

static void race_task4_log_point_state(const race_context_t *ctx,
    const race_phase_config_t *config,
    uint8_t reason,
    uint8_t normal_point)
{
    st011_start_pulse(RACE_TASK4_POINT_ALARM_MS);
    race_log_printf("TASK4 point: lap=%u phase=%s kind=%s reason=%s t=%lu dist=%ld yaw=%ld err=%ld\r\n",
        ctx->lap_count,
        config->phase_name,
        (normal_point != 0U) ? "point" : "force",
        race_reason_name(reason),
        ctx->elapsed_ms,
        ctx->phase_distance_count,
        ctx->yaw_cdeg,
        (ctx->ir_ok != 0U) ? ctx->sample.error : 0);
}

static uint8_t race_task4_turn_crossed_target(uint8_t error_valid,
    int32_t last_error_cdeg,
    int32_t current_error_cdeg)
{
    return ((error_valid != 0U) &&
        ((abs_i32(last_error_cdeg) <= RACE_TASK4_TURN_CROSS_ARM_CDEG) ||
         (abs_i32(current_error_cdeg) <= RACE_TASK4_TURN_CROSS_ARM_CDEG)) &&
        (((last_error_cdeg < 0) && (current_error_cdeg >= 0)) ||
         ((last_error_cdeg > 0) && (current_error_cdeg <= 0)))) ? 1U : 0U;
}

/* 第四问专用陀螺仪定角转向，复制任务三算法并隔离停转参数。 */
static uint8_t race_task4_gyro_turn_to_yaw(
    const gyro_turn_config_t *config)
{
    jy62_navigation_t nav = {0};
    uint32_t elapsed_ms = 0U;
    int32_t yaw_start_cdeg = 0;
    int32_t gyro_z_filtered_mdps = 0;
    int32_t yaw_error_cdeg;
    int32_t last_yaw_error_cdeg;
    int32_t slow_zone_cdeg = (config->slow_zone_cdeg > 0) ?
        config->slow_zone_cdeg : RACE_TASK4_TURN_SLOW_ZONE_CDEG;
    uint8_t slow_mode = 0U;
    uint8_t yaw_error_valid = 0U;
    uint8_t stop_reason = 0U;

    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    if (race_peek_yaw(&yaw_start_cdeg, &gyro_z_filtered_mdps) == 0U) {
        TB6612_Brake();
        return 0U;
    }
    (void)gyro_z_filtered_mdps;

    last_yaw_error_cdeg = normalize_cdeg(yaw_start_cdeg -
        config->yaw_stop_target_cdeg);
    yaw_error_valid = 1U;
    if (abs_i32(last_yaw_error_cdeg) <= slow_zone_cdeg) {
        slow_mode = 1U;
    }
    TB6612_SetDifferential(
        (slow_mode != 0U) ? config->slow_motor_b_pwm : config->motor_b_pwm,
        (slow_mode != 0U) ? config->slow_motor_a_pwm : config->motor_a_pwm);

    while (elapsed_ms < RACE_TASK4_GYRO_TURN_TIMEOUT_MS) {
        delay_ms_with_st011(RACE_TASK4_CONTROL_PERIOD_MS);
        elapsed_ms += RACE_TASK4_CONTROL_PERIOD_MS;
        if (task_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }
        if (JY62_PeekNavigation(&nav) == 0U) {
            stop_reason = 5U;
            break;
        }

        yaw_error_cdeg = normalize_cdeg(nav.yaw_relative_cdeg -
            config->yaw_stop_target_cdeg);
        if ((slow_mode == 0U) &&
            (abs_i32(yaw_error_cdeg) <= slow_zone_cdeg)) {
            slow_mode = 1U;
            TB6612_SetDifferential(config->slow_motor_b_pwm,
                config->slow_motor_a_pwm);
        }
        if ((abs_i32(yaw_error_cdeg) <=
                RACE_TASK4_TURN_YAW_STOP_TOL_CDEG) ||
            (race_task4_turn_crossed_target(yaw_error_valid,
                last_yaw_error_cdeg,
                yaw_error_cdeg) != 0U)) {
            stop_reason = 6U;
            break;
        }
        last_yaw_error_cdeg = yaw_error_cdeg;
        yaw_error_valid = 1U;
    }

    TB6612_Brake();
    encoder_reset_distance_counts();
    return (stop_reason == 6U) ? 1U : 0U;
}

/* 第四问 C 点专用灰度辅助左转，完整复制任务三的停转判据。 */
static uint8_t race_task4_sensor_turn_at_c(void)
{
    ir_tracking_sample_t sample = {0};
    jy62_navigation_t nav = {0};
    uint32_t elapsed_ms = 0U;
    int32_t yaw_start_cdeg = 0;
    int32_t gyro_z_filtered_mdps = 0;
    int32_t yaw_progress_cdeg = 0;
    uint8_t slow_mode = 0U;
    uint8_t ir_ok;
    uint8_t nav_ok;
    uint8_t line_seen;
    uint8_t line_ready;

    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    if (race_peek_yaw(&yaw_start_cdeg, &gyro_z_filtered_mdps) == 0U) {
        TB6612_Brake();
        return 0U;
    }
    (void)gyro_z_filtered_mdps;
    TB6612_SetDifferential(RACE_TASK4_C_TURN_B_PWM,
        RACE_TASK4_C_TURN_A_PWM);

    while (elapsed_ms < RACE_TASK4_FAST_TURN_TIMEOUT_MS) {
        delay_ms_with_st011(RACE_TASK4_CONTROL_PERIOD_MS);
        elapsed_ms += RACE_TASK4_CONTROL_PERIOD_MS;
        if (task_uart_stop_requested() != 0U) {
            break;
        }

        ir_ok = IRTracking_ReadSample(&sample);
        nav_ok = JY62_PeekNavigation(&nav);
        if (nav_ok != 0U) {
            yaw_progress_cdeg = abs_i32(normalize_cdeg(
                nav.yaw_relative_cdeg - yaw_start_cdeg));
        }
        line_seen = ((ir_ok != 0U) && (sample.line_lost == 0U) &&
            (sample.active_count >=
                RACE_TASK4_IR_TURN_STOP_MIN_COUNT)) ? 1U : 0U;
        if ((slow_mode == 0U) &&
            ((line_seen != 0U) ||
             ((RACE_TASK4_FAST_TURN_GYRO_SLOW_ENABLE != 0U) &&
              (yaw_progress_cdeg >=
                RACE_TASK4_FAST_TURN_GYRO_SLOW_CDEG)))) {
            slow_mode = 1U;
            TB6612_SetDifferential(RACE_TASK4_C_TURN_SLOW_B_PWM,
                RACE_TASK4_C_TURN_SLOW_A_PWM);
        }

        line_ready = ((line_seen != 0U) &&
            ((((sample.line_mask & RACE_TASK4_C_TURN_STOP_MASK) != 0U) &&
               ((sample.line_mask & RACE_TASK4_C_TURN_FORBID_MASK) == 0U)) ||
             (sample.line_mask == 0xFFU) ||
             (abs_i32(sample.error) <=
                RACE_TASK4_C_TURN_STOP_ERROR_MAX))) ? 1U : 0U;
        if ((nav_ok != 0U) && (yaw_progress_cdeg <
            RACE_TASK4_C_TURN_LINE_STOP_MIN_YAW_CDEG)) {
            line_ready = 0U;
        }
        if (line_ready != 0U) {
            TB6612_Brake();
            encoder_reset_distance_counts();
            return 1U;
        }
    }

    TB6612_Brake();
    encoder_reset_distance_counts();
    return 0U;
}

static uint8_t race_task4_align_start_copy(void)
{
    const gyro_turn_config_t turn_config = {
        .tag = "TASK4_START_TO_AC",
        .motor_b_pwm = RACE_TASK4_START_RIGHT_TURN_B_PWM,
        .motor_a_pwm = RACE_TASK4_START_RIGHT_TURN_A_PWM,
        .slow_motor_b_pwm = RACE_TASK4_START_RIGHT_TURN_SLOW_B_PWM,
        .slow_motor_a_pwm = RACE_TASK4_START_RIGHT_TURN_SLOW_A_PWM,
        .yaw_stop_target_cdeg = RACE_TASK4_AC_HEADING_TARGET_CDEG,
        .slow_zone_cdeg = RACE_TASK4_TURN_SLOW_ZONE_CDEG,
        .predictive_stop_enable = 0U,
        .predictive_stop_ms = 0,
        .predictive_stop_min_gz_mdps = 0,
        .control_period_ms = RACE_TASK4_CONTROL_PERIOD_MS
    };

#if RACE_TASK4_START_ALIGN_ENABLE
    return race_task4_gyro_turn_to_yaw(&turn_config);
#else
    return 1U;
#endif
}

/* C 点先复制任务三的 6 cm 低速前推。 */
static uint8_t race_task4_advance_after_c(void)
{
    uint32_t elapsed_ms = 0U;
    int32_t motor_b_total = 0;
    int32_t motor_a_total = 0;
    int32_t distance_count = 0;

    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    TB6612_SetDifferential((int16_t)RACE_TASK4_C_ADVANCE_PWM,
        (int16_t)RACE_TASK4_C_ADVANCE_PWM);

    while (elapsed_ms < RACE_TASK4_C_ADVANCE_TIMEOUT_MS) {
        delay_ms_with_st011(RACE_TASK4_CONTROL_PERIOD_MS);
        elapsed_ms += RACE_TASK4_CONTROL_PERIOD_MS;
        if (task_uart_stop_requested() != 0U) {
            TB6612_Brake();
            return 0U;
        }
        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        distance_count = motion_distance_count(motor_b_total, motor_a_total);
        if (distance_count >= RACE_TASK4_C_ADVANCE_COUNT) {
            return 1U;
        }
    }

    TB6612_Brake();
    return 0U;
}

static uint8_t race_task4_drive_forward_until_line(void)
{
    ir_tracking_sample_t sample = {0};
    uint32_t elapsed_ms = 0U;
    int32_t motor_b_total = 0;
    int32_t motor_a_total = 0;
    int32_t distance_count = 0;

    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    TB6612_SetDifferential((int16_t)RACE_TASK4_FORCE_FIND_LINE_PWM,
        (int16_t)RACE_TASK4_FORCE_FIND_LINE_PWM);

    while (elapsed_ms < RACE_TASK4_FORCE_FIND_LINE_TIMEOUT_MS) {
        delay_ms_with_st011(RACE_TASK4_CONTROL_PERIOD_MS);
        elapsed_ms += RACE_TASK4_CONTROL_PERIOD_MS;
        if (task_uart_stop_requested() != 0U) {
            TB6612_Brake();
            return 0U;
        }
        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        distance_count = motion_distance_count(motor_b_total, motor_a_total);
        if (IRTracking_ReadSample(&sample) &&
            (sample.line_lost == 0U)) {
            return 1U;
        }
        if (distance_count >= RACE_TASK4_FORCE_FIND_LINE_COUNT) {
            break;
        }
    }

    TB6612_Brake();
    return 0U;
}

static uint8_t race_task4_execute_point_action(const race_context_t *ctx)
{
    uint8_t turn_success = 1U;

    if (ctx->phase == 0U) {
        turn_success = race_task4_advance_after_c();
        if (turn_success != 0U) {
            turn_success = race_task4_sensor_turn_at_c();
        }
    } else if (ctx->phase == 1U) {
        const gyro_turn_config_t turn_config = {
            .tag = "TASK4_B_GYRO_TO_BD",
            .motor_b_pwm = RACE_TASK4_B_TURN_B_PWM,
            .motor_a_pwm = RACE_TASK4_B_TURN_A_PWM,
            .slow_motor_b_pwm = RACE_TASK4_B_TURN_SLOW_B_PWM,
            .slow_motor_a_pwm = RACE_TASK4_B_TURN_SLOW_A_PWM,
            .yaw_stop_target_cdeg = RACE_TASK4_BD_HEADING_TARGET_CDEG,
            .slow_zone_cdeg = RACE_TASK4_TURN_SLOW_ZONE_CDEG,
            .predictive_stop_enable = 0U,
            .predictive_stop_ms = 0,
            .predictive_stop_min_gz_mdps = 0,
            .control_period_ms = RACE_TASK4_CONTROL_PERIOD_MS
        };
        TB6612_BrakePwm(RACE_TASK4_B_BRAKE_B_PWM,
            RACE_TASK4_B_BRAKE_A_PWM);
        turn_success = race_task4_gyro_turn_to_yaw(&turn_config);
    } else if (ctx->phase == 2U) {
        int32_t target_cdeg = normalize_cdeg(ctx->yaw_cdeg -
            RACE_TASK4_D_GYRO_ENTRY_TURN_CDEG);
        const gyro_turn_config_t turn_config = {
            .tag = "TASK4_D_GYRO_ENTRY",
            .motor_b_pwm = RACE_TASK4_D_TURN_B_PWM,
            .motor_a_pwm = RACE_TASK4_D_TURN_A_PWM,
            .slow_motor_b_pwm = RACE_TASK4_D_TURN_SLOW_B_PWM,
            .slow_motor_a_pwm = RACE_TASK4_D_TURN_SLOW_A_PWM,
            .yaw_stop_target_cdeg = target_cdeg,
            .slow_zone_cdeg = RACE_TASK4_TURN_SLOW_ZONE_CDEG,
            .predictive_stop_enable = 0U,
            .predictive_stop_ms = 0,
            .predictive_stop_min_gz_mdps = 0,
            .control_period_ms = RACE_TASK4_CONTROL_PERIOD_MS
        };
        TB6612_Brake();
        OLED_ShowYawDistanceError(ctx->yaw_cdeg,
            (encoder_get_calibration_distance_count() -
                ctx->lap_start_calibration_count) / COUNTS_PER_CM,
            (ctx->ir_ok != 0U) ? ctx->sample.error : 0,
            "T4_D",
            ctx->line_valid);
        delay_ms_with_st011(RACE_TASK4_D_POINT_BRAKE_SETTLE_MS);
        turn_success = race_task4_gyro_turn_to_yaw(&turn_config);
        if (turn_success != 0U) {
            delay_ms_with_st011(RACE_TASK4_D_HANDOFF_BRAKE_MS);
        }
    } else if ((uint8_t)(ctx->lap_count + 1U) < ctx->target_laps) {
        const gyro_turn_config_t turn_config = {
            .tag = "TASK4_A_GYRO_TO_AC",
            .motor_b_pwm = RACE_TASK4_A_TURN_B_PWM,
            .motor_a_pwm = RACE_TASK4_A_TURN_A_PWM,
            .slow_motor_b_pwm = RACE_TASK4_A_TURN_SLOW_B_PWM,
            .slow_motor_a_pwm = RACE_TASK4_A_TURN_SLOW_A_PWM,
            .yaw_stop_target_cdeg = RACE_TASK4_AC_HEADING_TARGET_CDEG,
            .slow_zone_cdeg = RACE_TASK4_TURN_SLOW_ZONE_CDEG,
            .predictive_stop_enable = 0U,
            .predictive_stop_ms = 0,
            .predictive_stop_min_gz_mdps = 0,
            .control_period_ms = RACE_TASK4_CONTROL_PERIOD_MS
        };
        TB6612_Brake();
        delay_ms_with_st011(RACE_TASK4_POINT_SETTLE_MS);
        turn_success = race_task4_gyro_turn_to_yaw(&turn_config);
    }

    return turn_success;
}

static uint8_t race_task4_execute_straight_force_turn_action(
    const race_context_t *ctx)
{
    int32_t yaw_cdeg = 0;
    int32_t gyro_z_filtered_mdps = 0;
    int32_t target_cdeg;
    uint8_t turn_success;

    if ((ctx->phase != 0U) ||
        (race_peek_yaw(&yaw_cdeg, &gyro_z_filtered_mdps) == 0U)) {
        return 0U;
    }
    (void)gyro_z_filtered_mdps;
    target_cdeg = normalize_cdeg(yaw_cdeg +
        RACE_TASK4_FORCE_ENTRY_TURN_CDEG);
    {
        const gyro_turn_config_t turn_config = {
            .tag = "TASK4_C_FORCE_TURN",
            .motor_b_pwm = RACE_TASK4_C_TURN_B_PWM,
            .motor_a_pwm = RACE_TASK4_C_TURN_A_PWM,
            .slow_motor_b_pwm = RACE_TASK4_C_TURN_SLOW_B_PWM,
            .slow_motor_a_pwm = RACE_TASK4_C_TURN_SLOW_A_PWM,
            .yaw_stop_target_cdeg = target_cdeg,
            .slow_zone_cdeg = RACE_TASK4_TURN_SLOW_ZONE_CDEG,
            .predictive_stop_enable = 0U,
            .predictive_stop_ms = 0,
            .predictive_stop_min_gz_mdps = 0,
            .control_period_ms = RACE_TASK4_CONTROL_PERIOD_MS
        };
        turn_success = race_task4_gyro_turn_to_yaw(&turn_config);
    }
    if (turn_success != 0U) {
        turn_success = race_task4_drive_forward_until_line();
    }
    return turn_success;
}

static void race_task4_reset_segment_control(race_context_t *ctx)
{
    ctx->straight_point_count = 0U;
    ctx->straight_line_seen_count = 0U;
    ctx->filtered_error = 0;
    ctx->last_filtered_error = 0;
    ctx->filtered_derivative = 0;
    ctx->last_turn = 0;
    ctx->line_lost_count = 0U;
    ctx->line_control_seeded = 0U;
    ctx->report_elapsed_ms = 0U;
    race_diff_pid_reset(&ctx->diff_pid, 1U);
}

/*
 * 每圈 A 点结束事件专用距离复位。
 * 同时清除工作编码器、OLED/判据使用的累计 Dis 以及上下文内所有距离基准。
 */
static void race_task4_reset_lap_distance_state(race_context_t *ctx)
{
    encoder_reset_distance_counts();
    encoder_reset_calibration_distance_count();
    ctx->motor_b_delta = 0;
    ctx->motor_a_delta = 0;
    ctx->motor_b_total = 0;
    ctx->motor_a_total = 0;
    ctx->total_distance_count = 0;
    ctx->phase_distance_count = 0;
    ctx->phase_start_count = 0;
    ctx->phase_start_calibration_count = 0;
    ctx->lap_start_calibration_count = 0;
}

static void race_task4_advance_segment(race_context_t *ctx,
    uint8_t point_event)
{
    if (ctx->phase == 3U) {
        ctx->lap_count++;
        if (ctx->lap_count >= ctx->target_laps) {
            ctx->stop_reason = (point_event != 0U) ? 1U : 2U;
            return;
        }
        ctx->phase = 0U;
        /*
         * 距离已经在 A 点停止状态触发时清零；这里不再二次清零，
         * 只把 A->AC 转向产生的计数设为下一圈的扣除基准。
         */
        ctx->lap_start_calibration_count =
            encoder_get_calibration_distance_count();
    } else {
        ctx->phase++;
    }

    ctx->phase_start_ms = ctx->elapsed_ms;
    ctx->phase_start_calibration_count =
        encoder_get_calibration_distance_count();
    if (point_event != 0U) {
        ctx->phase_start_count = 0;
        race_task4_reset_segment_control(ctx);
        race_read_navigation_state(ctx, 1U);
    } else {
        ctx->phase_start_count = ctx->total_distance_count;
        ctx->yaw_start = ctx->yaw_cdeg;
        race_task4_reset_segment_control(ctx);
    }
}

static void race_task4_init_lap_context(race_context_t *ctx)
{
    ctx->target_laps = TASK4_LAP_COUNT;
    TB6612_Brake();
    delay_ms_with_st011(RACE_TASK4_POINT_SETTLE_MS);
    {
        jy62_navigation_t zero_nav = {0};
        (void)JY62_GetNavigation(&zero_nav);
        if (zero_nav.valid != 0U) {
            JY62_SetYawZeroToCurrent();
            g_jy62_zero_ready = 1U;
        }
    }
    delay_ms_with_st011(RACE_TASK4_POINT_SETTLE_MS);
    if (race_task4_align_start_copy() == 0U) {
        ctx->stop_reason = 2U;
        return;
    }
    delay_ms_with_st011(RACE_TASK4_POINT_SETTLE_MS);
    IRTracking_Init();
    encoder_reset_calibration_distance_count();
    ctx->phase_start_calibration_count = 0;
    ctx->lap_start_calibration_count = 0;
    encoder_enable_interrupts();
    race_diff_pid_reset(&ctx->diff_pid, 1U);
    race_read_navigation_state(ctx, 1U);
}

static void race_task4_finish_lap_context(race_context_t *ctx)
{
    TB6612_Brake();
    st011_finish_pending_pulse();
    encoder_reset_distance_counts();
    if (ctx->stop_reason == 0U) {
        ctx->stop_reason = (ctx->lap_count >= ctx->target_laps) ? 1U :
            ((ctx->elapsed_ms >= RACE_TASK4_TOTAL_MAX_RUN_MS) ? 4U : 2U);
    }
    race_log_printf("TASK4 stop: reason=%s laps=%u/%u phase=%s t=%lu\r\n",
        race_reason_name(ctx->stop_reason),
        ctx->lap_count,
        ctx->target_laps,
        race_phase_name(ctx->phase),
        ctx->elapsed_ms);
}

static void run_task4_laps(void)
{
    race_context_t ctx = {0};
    race_phase_config_t phase_config;
    uint32_t oled_elapsed_ms = OLED_REFRESH_MIN_MS;

    race_task4_init_lap_context(&ctx);
    if (ctx.stop_reason != 0U) {
        race_task4_finish_lap_context(&ctx);
        return;
    }

    while ((ctx.elapsed_ms < RACE_TASK4_TOTAL_MAX_RUN_MS) &&
        (ctx.lap_count < ctx.target_laps)) {
        uint8_t arc_mode;

        race_task4_configure_phase(&ctx, &phase_config);
        delay_ms_with_st011(RACE_TASK4_CONTROL_PERIOD_MS);
        ctx.elapsed_ms += RACE_TASK4_CONTROL_PERIOD_MS;
        ctx.report_elapsed_ms += RACE_TASK4_CONTROL_PERIOD_MS;
        oled_elapsed_ms += RACE_TASK4_CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            ctx.stop_reason = 3U;
            break;
        }

        race_update_loop_state(&ctx, &phase_config);
        if (oled_elapsed_ms >= OLED_REFRESH_MIN_MS) {
            int32_t lap_distance_count =
                encoder_get_calibration_distance_count() -
                ctx.lap_start_calibration_count;

            OLED_ShowYawDistanceError(ctx.yaw_cdeg,
                lap_distance_count / COUNTS_PER_CM,
                (ctx.ir_ok != 0U) ? ctx.sample.error : 0,
                phase_config.phase_name,
                ctx.line_valid);
            oled_elapsed_ms = 0U;
        }

        arc_mode = phase_config.arc_mode;
        race_compute_loop_control(&ctx,
            &phase_config,
            (arc_mode != 0U) ? 0U : 1U,
            arc_mode);
        if (arc_mode != 0U) {
            task4_seed_da_follow_control(&ctx);
            task4_apply_arc_follow_control(&ctx,
                1U,
                (ctx.phase == 3U) ? 1U : 0U);
        }

        if (race_task4_check_phase_point(&ctx, &phase_config) != 0U) {
            race_capture_result(&ctx, 1U);
            race_task4_log_point_state(&ctx, &phase_config, 1U, 1U);
            if (ctx.phase == 3U) {
                /* DA 到 A 的每圈最终停止状态，是唯一的整圈距离清零触发源。 */
                TB6612_Brake();
                race_task4_reset_lap_distance_state(&ctx);
            }
            if (race_task4_execute_point_action(&ctx) == 0U) {
                ctx.stop_reason = 2U;
                break;
            }
            race_task4_advance_segment(&ctx, 1U);
            if (ctx.stop_reason != 0U) {
                break;
            }
            continue;
        }

        if (race_task4_check_straight_force_turn(&ctx,
                &phase_config) != 0U) {
            race_capture_result(&ctx, 2U);
            race_task4_log_point_state(&ctx, &phase_config, 2U, 0U);
            if (race_task4_execute_straight_force_turn_action(&ctx) == 0U) {
                ctx.stop_reason = 2U;
                break;
            }
            race_task4_advance_segment(&ctx, 1U);
            if (ctx.stop_reason != 0U) {
                break;
            }
            continue;
        }

        /* 与任务三一致：CB/DA 不允许保护弧长绕过 B/A 的 Dis+丢线判据。 */
        if ((arc_mode == 0U) &&
            (ctx.phase_distance_count >= phase_config.force_count)) {
            ctx.stop_reason = 2U;
            break;
        }

        TB6612_SetDifferential((int16_t)ctx.left_pwm,
            (int16_t)ctx.right_pwm);
        race_log_periodic_data(&ctx, &phase_config);
    }

    race_task4_finish_lap_context(&ctx);
}

/**
 * @brief 使用共享 AC/CB/BD/DA 阶段循环执行任务三/任务四。
 */
static void run_race_laps(uint8_t target_laps)
{
    race_context_t ctx = {0};
    race_phase_config_t phase_config;
    uint32_t control_period_ms;
    uint32_t oled_elapsed_ms = OLED_REFRESH_MIN_MS;

    race_init_lap_context(&ctx, target_laps);
    if (ctx.stop_reason != 0U) {
        race_finish_lap_context(&ctx);
        return;
    }
    control_period_ms = (ctx.target_laps == TASK4_LAP_COUNT) ?
        RACE_TASK4_CONTROL_PERIOD_MS : CONTROL_PERIOD_MS;

    while ((ctx.elapsed_ms < RACE_TOTAL_MAX_RUN_MS) &&
        (ctx.lap_count < ctx.target_laps)) {
        uint8_t task3_use_task2_arc_follow;

        race_configure_phase(&ctx, &phase_config);

        delay_ms_with_st011(control_period_ms);
        ctx.elapsed_ms += control_period_ms;
        ctx.report_elapsed_ms += control_period_ms;
        oled_elapsed_ms += control_period_ms;

        if (task_uart_stop_requested() != 0U) {
            ctx.stop_reason = 3U;
            break;
        }

        race_update_loop_state(&ctx, &phase_config);
        /*
         * 此处读的是独立累计里程，阶段切换和快速转向中的编码器复位
         * 不会令 Dis 归零；该数值仅显示，不参与任何状态判断。
         */
        if (oled_elapsed_ms >= OLED_REFRESH_MIN_MS) {
            int32_t total_distance_count =
                encoder_get_calibration_distance_count();

            OLED_ShowYawDistanceError(ctx.yaw_cdeg,
                total_distance_count / COUNTS_PER_CM,
                (ctx.ir_ok != 0U) ? ctx.sample.error : 0,
                phase_config.phase_name,
                ctx.line_valid);
            oled_elapsed_ms = 0U;
        }
        task3_use_task2_arc_follow = ((ctx.target_laps == 1U) &&
            (phase_config.arc_mode != 0U)) ? 1U : 0U;
        race_compute_loop_control(&ctx, &phase_config,
            (task3_use_task2_arc_follow != 0U) ? 0U : 1U,
            task3_use_task2_arc_follow);
        if (task3_use_task2_arc_follow != 0U) {
            /* 第三问 CB/DA 共用同一速度比例、灰度 PD、限幅和出口减速。 */
            task3_seed_da_follow_control(&ctx);
            task2_apply_arc_follow_control(&ctx,
                1U,
                (ctx.phase == 3U) ? 1U : 0U);
        }

        if (race_check_phase_point(&ctx, &phase_config) != 0U) {
            race_capture_result(&ctx, 1U);
            race_log_point_state(&ctx,
                &phase_config,
                ctx.result.reason,
                1U);

            if (race_execute_point_action(&ctx) == 0U) {
                ctx.stop_reason = 2U;
                break;
            }

            race_advance_segment(&ctx, 1U);
            if (ctx.stop_reason != 0U) {
                break;
            }
            continue;
        }

        if (race_check_straight_force_turn(&ctx, &phase_config) != 0U) {
            race_capture_result(&ctx, 2U);
            race_log_point_state(&ctx,
                &phase_config,
                ctx.result.reason,
                0U);

            if (race_execute_straight_force_turn_action(&ctx) == 0U) {
                ctx.stop_reason = 2U;
                break;
            }

            race_advance_segment(&ctx, 1U);
            if (ctx.stop_reason != 0U) {
                break;
            }
            continue;
        }

        /*
         * 第三问 CB 的 B 点、DA 的 A 点只能由各自的“Dis 达标 + 灰度丢线”
         * 触发；禁止弧线保护距离绕过这两个判据直接切段或结束。
         */
        if ((ctx.phase_distance_count >= phase_config.force_count) &&
            !(((ctx.target_laps == 1U) ||
                (ctx.target_laps == TASK4_LAP_COUNT)) &&
                ((ctx.phase == 1U) || (ctx.phase == 3U)))) {
            uint8_t point_action_done = 0U;

            race_capture_result(&ctx, 2U);
            race_log_point_state(&ctx,
                &phase_config,
                ctx.result.reason,
                0U);

            /*
             * 第三问弧线达到保护距离时也必须先执行 B/A 点动作。
             * 原逻辑直接切段，会跳过 B 点转向，让小车先按 BD 状态直走，
             * 之后才靠航向误差缓慢拐向斜线。
             */
            if (((ctx.target_laps == 1U) ||
                (ctx.target_laps == TASK4_LAP_COUNT)) &&
                (phase_config.arc_mode != 0U)) {
                if (race_execute_point_action(&ctx) == 0U) {
                    ctx.stop_reason = 2U;
                    break;
                }
                point_action_done = 1U;
            }

            race_advance_segment(&ctx, point_action_done);
            if (ctx.stop_reason != 0U) {
                break;
            }
            continue;
        }

        TB6612_SetDifferential((int16_t)ctx.left_pwm, (int16_t)ctx.right_pwm);
        race_log_periodic_data(&ctx, &phase_config);
    }

    race_finish_lap_context(&ctx);
}

#endif

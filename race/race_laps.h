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
            !((ctx.target_laps == 1U) &&
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
            if ((ctx.target_laps == 1U) &&
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

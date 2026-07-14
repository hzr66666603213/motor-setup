#include "control/velocity_controller.h"
#include "foc/foc_math.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#if defined(__GNUC__)
#define VELOCITY_FAST_OPT __attribute__((optimize("O3")))
#else
#define VELOCITY_FAST_OPT
#endif

/*
 * velocity_controller.c
 *
 * 速度环比电流环慢，第一版采用 PI。
 * 输出是 iq 目标电流，而不是直接输出 PWM 或电压。
 */

void velocity_controller_init(VelocityController *controller, float kp, float ki, float current_limit_a, float velocity_limit_rad_s)
{
    controller->kp = kp;
    controller->ki = ki;
    controller->integrator_a = 0.0f;
    controller->integrator_limit_a = current_limit_a;
    controller->velocity_limit_rad_s = velocity_limit_rad_s;
    controller->current_limit_a = current_limit_a;
    controller->last_error_rad_s = 0.0f;
    controller->last_proportional_a = 0.0f;
    controller->last_unsaturated_a = 0.0f;
    controller->last_output_a = 0.0f;
    controller->output_slew_limit_a_per_s = 0.0f;
    controller->saturation_count = 0u;
    controller->slew_limit_count = 0u;
    controller->anti_windup_hold_count = 0u;
    controller->error_reversal_integrator_scale = 1.0f;
    controller->error_reversal_decay_count = 0u;
}

void velocity_controller_reset(VelocityController *controller)
{
    controller->integrator_a = 0.0f;
    controller->last_error_rad_s = 0.0f;
    controller->last_proportional_a = 0.0f;
    controller->last_unsaturated_a = 0.0f;
    controller->last_output_a = 0.0f;
    controller->saturation_count = 0u;
    controller->slew_limit_count = 0u;
    controller->anti_windup_hold_count = 0u;
    controller->error_reversal_decay_count = 0u;
}

void velocity_controller_preload_output(VelocityController *controller,
                                        float output_a)
{
    if (controller == 0) {
        return;
    }
    controller->last_output_a = foc_clamp(output_a,
                                          -controller->current_limit_a,
                                          controller->current_limit_a);
}

void velocity_controller_prepare_bumpless_handoff(
    VelocityController *controller,
    VelocityCountWindow *speed_window,
    float initial_output_a)
{
    if (controller == 0 || speed_window == 0) {
        return;
    }

    velocity_controller_reset(controller);
    velocity_count_window_reset(speed_window);
    velocity_controller_preload_output(controller, initial_output_a);
}

VELOCITY_FAST_OPT float velocity_controller_apply_hold_direction_guard(
    VelocityController *controller,
    float output_a,
    float target_velocity,
    bool braking_allowed)
{
    if (controller == 0 || braking_allowed) {
        return output_a;
    }

    if ((target_velocity > 0.0f && output_a < 0.0f) ||
        (target_velocity < 0.0f && output_a > 0.0f)) {
        controller->last_output_a = 0.0f;
        return 0.0f;
    }
    return output_a;
}

VELOCITY_FAST_OPT float velocity_controller_apply_coulomb_feedforward(
    VelocityController *controller,
    float output_a,
    float target_velocity,
    float feedforward_a,
    bool enabled)
{
    if (controller == 0 || !enabled || target_velocity == 0.0f ||
        feedforward_a <= 0.0f) {
        return output_a;
    }

    const float signed_feedforward_a =
        (target_velocity > 0.0f) ? feedforward_a : -feedforward_a;
    const float combined_a = output_a + signed_feedforward_a;
    const float limited_a = foc_clamp(combined_a,
                                      -controller->current_limit_a,
                                      controller->current_limit_a);
    if (limited_a != combined_a) {
        controller->saturation_count++;
    }
    controller->last_output_a = limited_a;
    return limited_a;
}

void velocity_low_speed_assist_reset(VelocityLowSpeedAssist *assist)
{
    if (assist != 0) {
        memset(assist, 0, sizeof(*assist));
    }
}

VELOCITY_FAST_OPT float velocity_low_speed_assist_update(
    VelocityLowSpeedAssist *assist,
    float base_output_a,
    float target_rpm,
    float measured_rpm,
    float assist_current_a,
    float enable_below_rpm,
    float disable_above_rpm,
    float current_limit_a,
    bool enabled)
{
    if (assist == 0 || current_limit_a <= 0.0f) {
        return 0.0f;
    }

    const float limited_base = foc_clamp(base_output_a,
                                         -current_limit_a,
                                         current_limit_a);
    const bool valid = enabled && target_rpm != 0.0f &&
                       assist_current_a > 0.0f &&
                       enable_below_rpm >= 0.0f &&
                       disable_above_rpm > enable_below_rpm;
    if (!valid) {
        if (assist->active) {
            assist->active = false;
            assist->deactivation_count++;
        }
        return limited_base;
    }

    const float direction = (target_rpm > 0.0f) ? 1.0f : -1.0f;
    const float directed_speed_rpm = measured_rpm * direction;
    const float directed_base_a = limited_base * direction;

    if (assist->active &&
        (directed_speed_rpm >= disable_above_rpm ||
         directed_base_a < 0.0f)) {
        assist->active = false;
        assist->deactivation_count++;
    } else if (!assist->active &&
               directed_speed_rpm <= enable_below_rpm &&
               directed_base_a >= 0.0f) {
        assist->active = true;
        assist->activation_count++;
    }

    if (!assist->active) {
        return limited_base;
    }

    assist->active_update_count++;
    return foc_clamp(limited_base + direction * assist_current_a,
                     -current_limit_a,
                     current_limit_a);
}

void velocity_controller_set_gains(VelocityController *controller, float kp, float ki)
{
    controller->kp = kp;
    controller->ki = ki;
}

void velocity_controller_set_error_reversal_decay(
    VelocityController *controller,
    float integrator_scale)
{
    if (controller == 0) {
        return;
    }
    controller->error_reversal_integrator_scale =
        foc_clamp(integrator_scale, 0.0f, 1.0f);
}

void velocity_controller_set_output_slew_limit(
    VelocityController *controller,
    float slew_limit_a_per_s)
{
    if (controller == 0) {
        return;
    }
    controller->output_slew_limit_a_per_s =
        (slew_limit_a_per_s > 0.0f) ? slew_limit_a_per_s : 0.0f;
}

VELOCITY_FAST_OPT float velocity_controller_update_gated(VelocityController *controller,
                                       float velocity_target_rad_s,
                                       float velocity_measured_rad_s,
                                       float dt_s,
                                       bool integrator_enable)
{
    /* 先限制速度目标，避免通信写入异常值导致速度环直接打满。 */
    const float target = foc_clamp(velocity_target_rad_s,
                                   -controller->velocity_limit_rad_s,
                                   controller->velocity_limit_rad_s);
    const float error = target - velocity_measured_rad_s;

    /* 积分项单位为 A，用于消除稳态速度误差。 */
    const float proportional_a = controller->kp * error;
    float integrator_base_a = controller->integrator_a;
    if (integrator_enable &&
        (error * controller->last_error_rad_s) < 0.0f &&
        controller->error_reversal_integrator_scale < 1.0f) {
        integrator_base_a *= controller->error_reversal_integrator_scale;
        controller->integrator_a = integrator_base_a;
        controller->error_reversal_decay_count++;
    }
    const float integrator_candidate_a = integrator_enable
        ? foc_clamp(integrator_base_a + (controller->ki * error * dt_s),
                    -controller->integrator_limit_a,
                    controller->integrator_limit_a)
        : integrator_base_a;
    const float candidate_output_a = proportional_a + integrator_candidate_a;
    const bool drives_positive_saturation =
        (candidate_output_a > controller->current_limit_a) && (error > 0.0f);
    const bool drives_negative_saturation =
        (candidate_output_a < -controller->current_limit_a) && (error < 0.0f);

    if (!integrator_enable) {
        controller->integrator_a = integrator_base_a;
    } else if (drives_positive_saturation || drives_negative_saturation) {
        controller->anti_windup_hold_count++;
    } else {
        controller->integrator_a = integrator_candidate_a;
    }

    float unsaturated_a = proportional_a + controller->integrator_a;
    float iq_target_a = foc_clamp(unsaturated_a,
                                  -controller->current_limit_a,
                                  controller->current_limit_a);
    if (iq_target_a != unsaturated_a) {
        controller->saturation_count++;
    }
    if (controller->output_slew_limit_a_per_s > 0.0f && dt_s > 0.0f) {
        const float max_delta_a = controller->output_slew_limit_a_per_s * dt_s;
        const float previous_output_a = controller->last_output_a;
        const float desired_abs_a =
            (iq_target_a < 0.0f) ? -iq_target_a : iq_target_a;
        const float previous_abs_a =
            (previous_output_a < 0.0f) ? -previous_output_a : previous_output_a;
        const bool reverses_direction =
            (iq_target_a * previous_output_a) < 0.0f;
        const bool increases_magnitude = desired_abs_a > previous_abs_a;
        float slew_limited_a = iq_target_a;
        if (reverses_direction) {
            /* Remove stale torque immediately. Opposite torque ramps from zero
             * on a later velocity update. */
            slew_limited_a = 0.0f;
        } else if (increases_magnitude) {
            const float lower_a = previous_output_a - max_delta_a;
            const float upper_a = previous_output_a + max_delta_a;
            slew_limited_a = foc_clamp(iq_target_a, lower_a, upper_a);
        }
        if (slew_limited_a != iq_target_a) {
            controller->slew_limit_count++;
            const bool drives_positive_slew_limit =
                !reverses_direction && (iq_target_a > previous_output_a) &&
                (error > 0.0f);
            const bool drives_negative_slew_limit =
                !reverses_direction && (iq_target_a < previous_output_a) &&
                (error < 0.0f);
            if (integrator_enable &&
                (drives_positive_slew_limit || drives_negative_slew_limit) &&
                controller->integrator_a != integrator_base_a) {
                controller->integrator_a = integrator_base_a;
                controller->anti_windup_hold_count++;
                unsaturated_a = proportional_a + controller->integrator_a;
                iq_target_a = foc_clamp(unsaturated_a,
                                        -controller->current_limit_a,
                                        controller->current_limit_a);
                if ((iq_target_a * previous_output_a) < 0.0f) {
                    slew_limited_a = 0.0f;
                } else {
                    const float lower_a = previous_output_a - max_delta_a;
                    const float upper_a = previous_output_a + max_delta_a;
                    slew_limited_a = foc_clamp(iq_target_a, lower_a, upper_a);
                }
            }
            iq_target_a = slew_limited_a;
        }
    }
    controller->last_error_rad_s = error;
    controller->last_proportional_a = proportional_a;
    controller->last_unsaturated_a = unsaturated_a;
    controller->last_output_a = iq_target_a;
    return iq_target_a;
}

float velocity_controller_update(VelocityController *controller,
                                 float velocity_target_rad_s,
                                 float velocity_measured_rad_s,
                                 float dt_s)
{
    return velocity_controller_update_gated(controller,
                                            velocity_target_rad_s,
                                            velocity_measured_rad_s,
                                            dt_s,
                                            true);
}

float velocity_bounded_profile_target_rpm(float elapsed_s,
                                          float total_s,
                                          float target_rpm,
                                          float rise_rate_rpm_per_s,
                                          float fall_rate_rpm_per_s)
{
    if (elapsed_s <= 0.0f || total_s <= 0.0f || target_rpm <= 0.0f ||
        rise_rate_rpm_per_s <= 0.0f || fall_rate_rpm_per_s <= 0.0f) {
        return 0.0f;
    }
    if (elapsed_s >= total_s) {
        return 0.0f;
    }

    const float rise_time_s = target_rpm / rise_rate_rpm_per_s;
    const float fall_time_s = target_rpm / fall_rate_rpm_per_s;
    if (elapsed_s < rise_time_s) {
        return elapsed_s * rise_rate_rpm_per_s;
    }

    const float fall_start_s = total_s - fall_time_s;
    if (fall_start_s <= rise_time_s) {
        const float triangle_peak = fall_start_s * rise_rate_rpm_per_s;
        if (elapsed_s < fall_start_s) {
            return elapsed_s * rise_rate_rpm_per_s;
        }
        const float falling = triangle_peak -
                              ((elapsed_s - fall_start_s) * fall_rate_rpm_per_s);
        return (falling > 0.0f) ? falling : 0.0f;
    }
    if (elapsed_s < fall_start_s) {
        return target_rpm;
    }

    const float falling = target_rpm -
                          ((elapsed_s - fall_start_s) * fall_rate_rpm_per_s);
    return (falling > 0.0f) ? falling : 0.0f;
}

float velocity_hold_then_fall_target_rpm(float elapsed_s,
                                         float total_s,
                                         float target_rpm,
                                         float fall_rate_rpm_per_s)
{
    if (elapsed_s < 0.0f || total_s <= 0.0f || target_rpm <= 0.0f ||
        fall_rate_rpm_per_s <= 0.0f || elapsed_s >= total_s) {
        return 0.0f;
    }

    const float fall_time_s = target_rpm / fall_rate_rpm_per_s;
    const float fall_start_s = total_s - fall_time_s;
    if (fall_start_s > 0.0f && elapsed_s < fall_start_s) {
        return target_rpm;
    }

    const float falling = target_rpm -
        ((elapsed_s - ((fall_start_s > 0.0f) ? fall_start_s : 0.0f)) *
         fall_rate_rpm_per_s);
    return (falling > 0.0f) ? falling : 0.0f;
}

void velocity_continuous_guard_init(VelocityContinuousRunGuard *guard,
                                    uint32_t watchdog_timeout_ms)
{
    if (guard == 0) {
        return;
    }
    memset(guard, 0, sizeof(*guard));
    guard->watchdog_timeout_ms = watchdog_timeout_ms;
}

bool velocity_continuous_guard_arm(VelocityContinuousRunGuard *guard,
                                   bool preflight_ok,
                                   uint32_t now_ms)
{
    if (guard == 0 || guard->fault_latched != 0u || guard->running ||
        guard->start_consumed || !preflight_ok) {
        if (guard != 0 && !preflight_ok) {
            guard->fault_latched |= VELOCITY_CONTINUOUS_FAULT_PREFLIGHT;
        }
        return false;
    }
    guard->armed = true;
    guard->last_heartbeat_ms = now_ms;
    return true;
}

bool velocity_continuous_guard_start(VelocityContinuousRunGuard *guard,
                                     uint32_t now_ms)
{
    if (guard == 0 || !guard->armed || guard->running ||
        guard->fault_latched != 0u || guard->start_consumed) {
        return false;
    }
    guard->armed = false;
    guard->running = true;
    guard->start_consumed = true;
    guard->last_heartbeat_ms = now_ms;
    return true;
}

void velocity_continuous_guard_heartbeat(VelocityContinuousRunGuard *guard,
                                         uint32_t now_ms)
{
    if (guard != 0 && guard->running && guard->fault_latched == 0u) {
        guard->last_heartbeat_ms = now_ms;
    }
}

void velocity_continuous_guard_latch_fault(VelocityContinuousRunGuard *guard,
                                           uint32_t fault)
{
    if (guard == 0) {
        return;
    }
    guard->fault_latched |= (fault != 0u) ? fault :
                            VELOCITY_CONTINUOUS_FAULT_RUNTIME;
    guard->armed = false;
    guard->running = false;
}

bool velocity_continuous_guard_poll(VelocityContinuousRunGuard *guard,
                                    bool runtime_safe,
                                    uint32_t now_ms)
{
    if (guard == 0 || !guard->running || guard->fault_latched != 0u) {
        return false;
    }
    if (!runtime_safe) {
        velocity_continuous_guard_latch_fault(
            guard, VELOCITY_CONTINUOUS_FAULT_RUNTIME);
        return false;
    }
    if (guard->watchdog_timeout_ms == 0u ||
        (uint32_t)(now_ms - guard->last_heartbeat_ms) >
            guard->watchdog_timeout_ms) {
        velocity_continuous_guard_latch_fault(
            guard, VELOCITY_CONTINUOUS_FAULT_WATCHDOG);
        return false;
    }
    return true;
}

void velocity_continuous_guard_stop(VelocityContinuousRunGuard *guard)
{
    if (guard == 0) {
        return;
    }
    guard->armed = false;
    guard->running = false;
}

bool velocity_continuous_guard_clear_fault(VelocityContinuousRunGuard *guard,
                                           bool outputs_safe)
{
    if (guard == 0 || !outputs_safe || guard->running) {
        return false;
    }
    guard->fault_latched = 0u;
    guard->armed = false;
    guard->start_consumed = false;
    guard->last_heartbeat_ms = 0u;
    return true;
}

void velocity_breakaway_probe_init(VelocityBreakawayProbe *probe,
                                   float iq_breakaway_a,
                                   uint32_t timeout_ticks,
                                   int32_t expected_direction,
                                   int32_t min_motion_counts,
                                   uint32_t min_direction_events,
                                   uint32_t max_step_counts)
{
    if (probe == 0) {
        return;
    }
    memset(probe, 0, sizeof(*probe));
    probe->iq_breakaway_a = iq_breakaway_a;
    probe->timeout_ticks = timeout_ticks;
    probe->expected_direction = (expected_direction < 0) ? -1 : 1;
    probe->min_motion_counts =
        (min_motion_counts > 0) ? min_motion_counts : 1;
    probe->min_direction_events =
        (min_direction_events > 0u) ? min_direction_events : 1u;
    probe->max_step_counts = (max_step_counts > 0u) ? max_step_counts : 1u;
    probe->result = VELOCITY_BREAKAWAY_IDLE;
}

bool velocity_breakaway_probe_start(VelocityBreakawayProbe *probe,
                                    int64_t encoder_count,
                                    uint32_t illegal_transition_count)
{
    if (probe == 0 || probe->start_consumed ||
        probe->result != VELOCITY_BREAKAWAY_IDLE ||
        probe->iq_breakaway_a <= 0.0f || probe->timeout_ticks == 0u) {
        return false;
    }
    probe->start_consumed = true;
    probe->result = VELOCITY_BREAKAWAY_ACTIVE;
    probe->encoder_start_count = encoder_count;
    probe->encoder_last_count = encoder_count;
    probe->illegal_transition_count_start = illegal_transition_count;
    probe->illegal_transition_count_end = illegal_transition_count;
    return true;
}

VelocityBreakawayResult velocity_breakaway_probe_update(
    VelocityBreakawayProbe *probe,
    int64_t encoder_count,
    uint32_t illegal_transition_count)
{
    if (probe == 0 || probe->result != VELOCITY_BREAKAWAY_ACTIVE) {
        return (probe != 0) ? probe->result : VELOCITY_BREAKAWAY_FAIL_ENCODER;
    }

    probe->elapsed_ticks++;
    const int64_t step = encoder_count - probe->encoder_last_count;
    probe->encoder_last_count = encoder_count;
    probe->encoder_final_delta_counts =
        encoder_count - probe->encoder_start_count;
    probe->illegal_transition_count_end = illegal_transition_count;

    const uint64_t step_abs = (step < 0) ? (uint64_t)(-step) : (uint64_t)step;
    if (step_abs > probe->max_step_seen_counts) {
        probe->max_step_seen_counts = (uint32_t)step_abs;
    }
    if (illegal_transition_count != probe->illegal_transition_count_start ||
        step_abs > probe->max_step_counts) {
        probe->result = VELOCITY_BREAKAWAY_FAIL_ENCODER;
        return probe->result;
    }

    const int64_t directed_step = step * probe->expected_direction;
    const int64_t directed_total =
        probe->encoder_final_delta_counts * probe->expected_direction;
    if (directed_step < 0 || directed_total < 0) {
        probe->result = VELOCITY_BREAKAWAY_FAIL_REVERSE;
        return probe->result;
    }
    if (directed_step > 0) {
        probe->direction_event_count++;
        probe->same_direction_event_streak++;
        if (probe->same_direction_event_streak >
            probe->same_direction_event_streak_max) {
            probe->same_direction_event_streak_max =
                probe->same_direction_event_streak;
        }
    }

    if (probe->motion_candidate) {
        probe->result = VELOCITY_BREAKAWAY_PASS;
        return probe->result;
    }
    if (directed_total >= probe->min_motion_counts &&
        probe->same_direction_event_streak >= probe->min_direction_events) {
        /* Confirm on the following coherent sample so an illegal transition
         * observed with the final motion edge cannot be accepted as motion. */
        probe->motion_candidate = true;
    }
    if (probe->elapsed_ticks >= probe->timeout_ticks) {
        probe->result = VELOCITY_BREAKAWAY_FAIL_NO_MOTION;
    }
    return probe->result;
}

float velocity_breakaway_probe_iq_ref(const VelocityBreakawayProbe *probe)
{
    if (probe == 0 || probe->result != VELOCITY_BREAKAWAY_ACTIVE) {
        return 0.0f;
    }
    return probe->iq_breakaway_a * (float)probe->expected_direction;
}

const char *velocity_breakaway_result_name(VelocityBreakawayResult result)
{
    switch (result) {
    case VELOCITY_BREAKAWAY_IDLE: return "IDLE";
    case VELOCITY_BREAKAWAY_ACTIVE: return "ACTIVE";
    case VELOCITY_BREAKAWAY_PASS: return "PASS";
    case VELOCITY_BREAKAWAY_FAIL_NO_MOTION: return "NO_MOTION";
    case VELOCITY_BREAKAWAY_FAIL_REVERSE: return "REVERSE_MOTION";
    case VELOCITY_BREAKAWAY_FAIL_ENCODER: return "ENCODER_INVALID";
    case VELOCITY_BREAKAWAY_FAIL_RUNTIME: return "RUNTIME_SAFETY_FAULT";
    default: return "UNKNOWN";
    }
}

void velocity_breakaway_handoff_init(VelocityBreakawayHandoff *handoff,
                                     float breakaway_iq_a,
                                     float continuous_iq_limit_a)
{
    if (handoff == 0) {
        return;
    }
    memset(handoff, 0, sizeof(*handoff));
    handoff->breakaway_iq_a =
        (breakaway_iq_a > 0.0f) ? breakaway_iq_a : 0.0f;
    handoff->continuous_iq_limit_a =
        (continuous_iq_limit_a > 0.0f) ? continuous_iq_limit_a : 0.0f;
    handoff->state = VELOCITY_BREAKAWAY_HANDOFF_IDLE;
}

bool velocity_breakaway_handoff_start(VelocityBreakawayHandoff *handoff)
{
    if (handoff == 0 || handoff->start_consumed ||
        handoff->state != VELOCITY_BREAKAWAY_HANDOFF_IDLE ||
        handoff->breakaway_iq_a <= 0.0f ||
        handoff->continuous_iq_limit_a <= 0.0f) {
        return false;
    }
    handoff->start_consumed = true;
    handoff->state = VELOCITY_BREAKAWAY_HANDOFF_BREAKAWAY;
    handoff->iq_command_a = handoff->breakaway_iq_a;
    return true;
}

float velocity_breakaway_handoff_update(
    VelocityBreakawayHandoff *handoff,
    VelocityBreakawayResult breakaway_result,
    float speed_pi_iq_request_a,
    uint32_t control_tick)
{
    if (handoff == 0) {
        return 0.0f;
    }
    if (handoff->state == VELOCITY_BREAKAWAY_HANDOFF_BREAKAWAY) {
        if (breakaway_result == VELOCITY_BREAKAWAY_PASS) {
            handoff->state = VELOCITY_BREAKAWAY_HANDOFF_SPEED_PI;
            handoff->iq_command_a = foc_clamp(
                handoff->breakaway_iq_a,
                -handoff->continuous_iq_limit_a,
                handoff->continuous_iq_limit_a);
            handoff->handoff_control_tick = control_tick;
            handoff->handoff_count++;
        } else if (breakaway_result != VELOCITY_BREAKAWAY_ACTIVE) {
            handoff->state = VELOCITY_BREAKAWAY_HANDOFF_FAILED;
            handoff->iq_command_a = 0.0f;
        }
    } else if (handoff->state == VELOCITY_BREAKAWAY_HANDOFF_SPEED_PI) {
        handoff->iq_command_a = foc_clamp(speed_pi_iq_request_a,
                                          -handoff->continuous_iq_limit_a,
                                          handoff->continuous_iq_limit_a);
    }
    return handoff->iq_command_a;
}

bool velocity_breakaway_handoff_speed_pi_active(
    const VelocityBreakawayHandoff *handoff)
{
    return handoff != 0 &&
           handoff->state == VELOCITY_BREAKAWAY_HANDOFF_SPEED_PI;
}

const char *velocity_breakaway_handoff_state_name(
    VelocityBreakawayHandoffState state)
{
    switch (state) {
    case VELOCITY_BREAKAWAY_HANDOFF_IDLE: return "IDLE";
    case VELOCITY_BREAKAWAY_HANDOFF_BREAKAWAY: return "BREAKAWAY";
    case VELOCITY_BREAKAWAY_HANDOFF_SPEED_PI: return "SPEED_PI";
    case VELOCITY_BREAKAWAY_HANDOFF_FAILED: return "FAILED";
    default: return "UNKNOWN";
    }
}

void velocity_count_window_reset(VelocityCountWindow *window)
{
    if (window != 0) {
        memset(window, 0, sizeof(*window));
        window->configured_samples = VELOCITY_COUNT_WINDOW_DEFAULT_SAMPLES;
    }
}

void velocity_edge_period_init(VelocityEdgePeriodEstimator *estimator,
                               int64_t encoder_count,
                               uint32_t stale_ticks)
{
    if (estimator == 0) {
        return;
    }
    memset(estimator, 0, sizeof(*estimator));
    estimator->last_count = encoder_count;
    estimator->stale_ticks = (stale_ticks > 0u) ? stale_ticks : 1u;
    estimator->initialized = true;
}

void velocity_edge_period_sample(VelocityEdgePeriodEstimator *estimator,
                                 int64_t encoder_count)
{
    if (estimator == 0 || !estimator->initialized) {
        return;
    }
    if (estimator->age_ticks != UINT32_MAX) {
        estimator->age_ticks++;
    }
    const int64_t delta = encoder_count - estimator->last_count;
    if (delta == 0) {
        return;
    }

    const uint64_t abs_delta = (delta < 0) ? (uint64_t)(-delta)
                                           : (uint64_t)delta;
    uint32_t period_ticks = estimator->age_ticks;
    if (abs_delta > 1u) {
        period_ticks /= (uint32_t)abs_delta;
    }
    if (period_ticks == 0u) {
        period_ticks = 1u;
    }
    estimator->period_ticks = period_ticks;
    estimator->direction = (delta < 0) ? -1 : 1;
    estimator->edge_count += (uint32_t)abs_delta;
    estimator->age_ticks = 0u;
    estimator->last_count = encoder_count;
}

float velocity_edge_period_rpm(const VelocityEdgePeriodEstimator *estimator,
                               float sample_tick_hz,
                               uint32_t encoder_cpr)
{
    if (estimator == 0 || !estimator->initialized ||
        estimator->period_ticks == 0u ||
        estimator->age_ticks > estimator->stale_ticks ||
        sample_tick_hz <= 0.0f || encoder_cpr == 0u) {
        return 0.0f;
    }
    return (float)estimator->direction * 60.0f * sample_tick_hz /
           ((float)encoder_cpr * (float)estimator->period_ticks);
}

bool velocity_count_window_set_samples(VelocityCountWindow *window,
                                       uint32_t sample_count)
{
    if (window == 0 || sample_count == 0u ||
        sample_count > VELOCITY_COUNT_WINDOW_SAMPLES) {
        return false;
    }
    velocity_count_window_reset(window);
    window->configured_samples = sample_count;
    return true;
}

float velocity_count_window_update_rpm(VelocityCountWindow *window,
                                       int32_t delta_counts,
                                       float sample_period_s,
                                       uint32_t encoder_cpr)
{
    if (window == 0 || sample_period_s <= 0.0f || encoder_cpr == 0u) {
        return 0.0f;
    }

    const uint32_t configured_samples =
        (window->configured_samples > 0u &&
         window->configured_samples <= VELOCITY_COUNT_WINDOW_SAMPLES)
            ? window->configured_samples
            : VELOCITY_COUNT_WINDOW_SAMPLES;

    if (window->sample_count == configured_samples) {
        window->sum_counts -= window->delta_counts[window->write_index];
    } else {
        window->sample_count++;
    }

    window->delta_counts[window->write_index] = delta_counts;
    window->sum_counts += delta_counts;
    window->write_index++;
    if (window->write_index == configured_samples) {
        window->write_index = 0u;
    }

    const float window_time_s = sample_period_s * (float)window->sample_count;
    return (float)window->sum_counts * 60.0f /
           ((float)encoder_cpr * window_time_s);
}

void velocity_count_window_analyze_overspeed(
    const VelocityCountWindow *window,
    float sample_period_s,
    uint32_t encoder_cpr,
    float hard_speed_limit_rpm,
    VelocityOverspeedAnalysis *analysis)
{
    if (analysis == 0) {
        return;
    }
    memset(analysis, 0, sizeof(*analysis));
    if (window == 0 || sample_period_s <= 0.0f || encoder_cpr == 0u ||
        hard_speed_limit_rpm <= 0.0f || window->sample_count == 0u ||
        window->sample_count > VELOCITY_COUNT_WINDOW_SAMPLES) {
        analysis->evidence = VELOCITY_OVERSPEED_EVIDENCE_INCONCLUSIVE;
        return;
    }

    const uint32_t configured_samples =
        (window->configured_samples > 0u &&
         window->configured_samples <= VELOCITY_COUNT_WINDOW_SAMPLES)
            ? window->configured_samples
            : VELOCITY_COUNT_WINDOW_SAMPLES;
    if (window->sample_count > configured_samples) {
        analysis->evidence = VELOCITY_OVERSPEED_EVIDENCE_INCONCLUSIVE;
        return;
    }

    analysis->sample_count = window->sample_count;
    analysis->sum_counts = window->sum_counts;
    const uint32_t start =
        (window->sample_count == configured_samples)
            ? window->write_index
            : 0u;
    for (uint32_t i = 0u; i < window->sample_count; ++i) {
        analysis->chronological_delta_counts[i] =
            window->delta_counts[(start + i) % configured_samples];
    }

    analysis->newest_delta_counts =
        analysis->chronological_delta_counts[window->sample_count - 1u];
    const int newest_sign = (analysis->newest_delta_counts > 0) ? 1 :
                            (analysis->newest_delta_counts < 0) ? -1 : 0;
    const int64_t newest_abs = llabs((long long)analysis->newest_delta_counts);
    analysis->window_abs_sum_counts = newest_abs;
    for (uint32_t i = 0u; i + 1u < window->sample_count; ++i) {
        const int32_t delta = analysis->chronological_delta_counts[i];
        const int32_t delta_abs = abs(delta);
        analysis->prior_abs_sum_counts += delta_abs;
        analysis->window_abs_sum_counts += delta_abs;
        if (delta_abs > analysis->prior_abs_max_counts) {
            analysis->prior_abs_max_counts = delta_abs;
        }
        if ((newest_sign > 0 && delta > 0) ||
            (newest_sign < 0 && delta < 0)) {
            analysis->prior_same_sign_count++;
            analysis->prior_same_sign_abs_sum_counts += delta_abs;
        }
    }

    analysis->instant_rpm =
        (float)analysis->newest_delta_counts * 60.0f /
        ((float)encoder_cpr * sample_period_s);
    analysis->windowed_rpm =
        (float)analysis->sum_counts * 60.0f /
        ((float)encoder_cpr * sample_period_s *
         (float)analysis->sample_count);
    if (analysis->window_abs_sum_counts > 0) {
        analysis->newest_window_abs_fraction =
            (float)newest_abs / (float)analysis->window_abs_sum_counts;
    }

    const float instant_abs = (analysis->instant_rpm < 0.0f)
                                  ? -analysis->instant_rpm
                                  : analysis->instant_rpm;
    const float windowed_abs = (analysis->windowed_rpm < 0.0f)
                                   ? -analysis->windowed_rpm
                                   : analysis->windowed_rpm;
    if (windowed_abs >= hard_speed_limit_rpm) {
        analysis->evidence = VELOCITY_OVERSPEED_EVIDENCE_WINDOWED;
    } else if (instant_abs < hard_speed_limit_rpm) {
        analysis->evidence = VELOCITY_OVERSPEED_EVIDENCE_NONE;
    } else if (analysis->prior_same_sign_count >= 2 &&
               analysis->prior_same_sign_abs_sum_counts * 4 >= newest_abs) {
        analysis->evidence =
            VELOCITY_OVERSPEED_EVIDENCE_SHORT_ACCELERATION;
    } else if (analysis->newest_window_abs_fraction >= 0.75f &&
               analysis->prior_same_sign_count <= 2) {
        analysis->evidence =
            VELOCITY_OVERSPEED_EVIDENCE_ISOLATED_DELTA_SPIKE;
    } else {
        analysis->evidence = VELOCITY_OVERSPEED_EVIDENCE_INCONCLUSIVE;
    }
}

const char *velocity_overspeed_evidence_name(
    VelocityOverspeedEvidence evidence)
{
    switch (evidence) {
    case VELOCITY_OVERSPEED_EVIDENCE_NONE:
        return "NO_OVERSPEED";
    case VELOCITY_OVERSPEED_EVIDENCE_WINDOWED:
        return "WINDOWED_OVERSPEED";
    case VELOCITY_OVERSPEED_EVIDENCE_SHORT_ACCELERATION:
        return "REAL_SHORT_ACCELERATION_EVIDENCE";
    case VELOCITY_OVERSPEED_EVIDENCE_ISOLATED_DELTA_SPIKE:
        return "ISOLATED_ENCODER_DELTA_SPIKE_CANDIDATE";
    case VELOCITY_OVERSPEED_EVIDENCE_INCONCLUSIVE:
    default:
        return "INCONCLUSIVE";
    }
}

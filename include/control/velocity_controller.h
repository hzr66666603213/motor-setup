#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * velocity_controller.h
 *
 * 速度环 PI 控制器。
 * 运行频率：建议 1 kHz。
 * 输入：目标机械速度 rad/s、实测机械速度 rad/s。
 * 输出：q 轴电流目标 A。
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;                  /* 比例增益，A/(rad/s) */
    float ki;                  /* 积分增益，A/rad */
    float integrator_a;        /* 速度环积分项，A */
    float integrator_limit_a;  /* 积分限幅，A */
    float velocity_limit_rad_s;/* 速度限幅，rad/s */
    float current_limit_a;     /* 输出电流限幅，A */
    float last_error_rad_s;
    float last_proportional_a;
    float last_unsaturated_a;
    float last_output_a;
    float output_slew_limit_a_per_s;
    uint32_t saturation_count;
    uint32_t slew_limit_count;
    uint32_t anti_windup_hold_count;
    float error_reversal_integrator_scale;
    uint32_t error_reversal_decay_count;
} VelocityController;

typedef enum {
    VELOCITY_CONTINUOUS_FAULT_NONE = 0u,
    VELOCITY_CONTINUOUS_FAULT_PREFLIGHT = (1u << 0),
    VELOCITY_CONTINUOUS_FAULT_WATCHDOG = (1u << 1),
    VELOCITY_CONTINUOUS_FAULT_RUNTIME = (1u << 2)
} VelocityContinuousFault;

typedef struct {
    uint32_t watchdog_timeout_ms;
    uint32_t last_heartbeat_ms;
    uint32_t fault_latched;
    bool armed;
    bool running;
    bool start_consumed;
} VelocityContinuousRunGuard;

typedef enum {
    VELOCITY_BREAKAWAY_IDLE = 0,
    VELOCITY_BREAKAWAY_ACTIVE,
    VELOCITY_BREAKAWAY_PASS,
    VELOCITY_BREAKAWAY_FAIL_NO_MOTION,
    VELOCITY_BREAKAWAY_FAIL_REVERSE,
    VELOCITY_BREAKAWAY_FAIL_ENCODER,
    VELOCITY_BREAKAWAY_FAIL_RUNTIME
} VelocityBreakawayResult;

typedef struct {
    float iq_breakaway_a;
    uint32_t timeout_ticks;
    int32_t expected_direction;
    int32_t min_motion_counts;
    uint32_t min_direction_events;
    uint32_t max_step_counts;
    VelocityBreakawayResult result;
    bool start_consumed;
    bool motion_candidate;
    uint32_t elapsed_ticks;
    uint32_t direction_event_count;
    uint32_t same_direction_event_streak;
    uint32_t same_direction_event_streak_max;
    uint32_t max_step_seen_counts;
    uint32_t illegal_transition_count_start;
    uint32_t illegal_transition_count_end;
    int64_t encoder_start_count;
    int64_t encoder_last_count;
    int64_t encoder_final_delta_counts;
} VelocityBreakawayProbe;

typedef enum {
    VELOCITY_BREAKAWAY_HANDOFF_IDLE = 0,
    VELOCITY_BREAKAWAY_HANDOFF_BREAKAWAY,
    VELOCITY_BREAKAWAY_HANDOFF_SPEED_PI,
    VELOCITY_BREAKAWAY_HANDOFF_FAILED
} VelocityBreakawayHandoffState;

typedef struct {
    VelocityBreakawayHandoffState state;
    float breakaway_iq_a;
    float continuous_iq_limit_a;
    float iq_command_a;
    uint32_t handoff_control_tick;
    uint32_t handoff_count;
    bool start_consumed;
} VelocityBreakawayHandoff;

#define VELOCITY_COUNT_WINDOW_SAMPLES 10u
#define VELOCITY_COUNT_WINDOW_DEFAULT_SAMPLES 5u

typedef struct {
    int32_t delta_counts[VELOCITY_COUNT_WINDOW_SAMPLES];
    int64_t sum_counts;
    uint32_t write_index;
    uint32_t sample_count;
    uint32_t configured_samples;
} VelocityCountWindow;

typedef struct {
    int64_t last_count;
    uint32_t age_ticks;
    uint32_t period_ticks;
    uint32_t stale_ticks;
    int32_t direction;
    uint32_t edge_count;
    bool initialized;
} VelocityEdgePeriodEstimator;

typedef struct {
    bool active;
    uint32_t activation_count;
    uint32_t deactivation_count;
    uint32_t active_update_count;
} VelocityLowSpeedAssist;

typedef enum {
    VELOCITY_OVERSPEED_EVIDENCE_NONE = 0,
    VELOCITY_OVERSPEED_EVIDENCE_WINDOWED,
    VELOCITY_OVERSPEED_EVIDENCE_SHORT_ACCELERATION,
    VELOCITY_OVERSPEED_EVIDENCE_ISOLATED_DELTA_SPIKE,
    VELOCITY_OVERSPEED_EVIDENCE_INCONCLUSIVE
} VelocityOverspeedEvidence;

typedef struct {
    int32_t chronological_delta_counts[VELOCITY_COUNT_WINDOW_SAMPLES];
    uint32_t sample_count;
    int64_t sum_counts;
    int32_t newest_delta_counts;
    int32_t prior_same_sign_count;
    int64_t prior_same_sign_abs_sum_counts;
    int32_t prior_abs_max_counts;
    int64_t prior_abs_sum_counts;
    int64_t window_abs_sum_counts;
    float instant_rpm;
    float windowed_rpm;
    float newest_window_abs_fraction;
    VelocityOverspeedEvidence evidence;
} VelocityOverspeedAnalysis;

/* 初始化速度 PI，限幅来自 MotorConfig。 */
void velocity_controller_init(VelocityController *controller, float kp, float ki, float current_limit_a, float velocity_limit_rad_s);

/* 清零积分器；退出闭环或切换模式时调用。 */
void velocity_controller_reset(VelocityController *controller);

/* Preload only the slew/output state for a bumpless bounded handoff. */
void velocity_controller_preload_output(VelocityController *controller,
                                        float output_a);

void velocity_controller_prepare_bumpless_handoff(
    VelocityController *controller,
    VelocityCountWindow *speed_window,
    float initial_output_a);

float velocity_controller_apply_hold_direction_guard(
    VelocityController *controller,
    float output_a,
    float target_velocity,
    bool braking_allowed);

float velocity_controller_apply_coulomb_feedforward(
    VelocityController *controller,
    float output_a,
    float target_velocity,
    float feedforward_a,
    bool enabled);

void velocity_low_speed_assist_reset(VelocityLowSpeedAssist *assist);

float velocity_low_speed_assist_update(
    VelocityLowSpeedAssist *assist,
    float base_output_a,
    float target_rpm,
    float measured_rpm,
    float assist_current_a,
    float enable_below_rpm,
    float disable_above_rpm,
    float current_limit_a,
    bool enabled);

/* 在线更新速度环增益。 */
void velocity_controller_set_gains(VelocityController *controller, float kp, float ki);

void velocity_controller_set_error_reversal_decay(
    VelocityController *controller,
    float integrator_scale);

void velocity_controller_set_output_slew_limit(
    VelocityController *controller,
    float slew_limit_a_per_s);

/* 执行一次速度 PI，返回 iq_target_A。 */
float velocity_controller_update(VelocityController *controller,
                                 float velocity_target_rad_s,
                                 float velocity_measured_rad_s,
                                 float dt_s);

float velocity_controller_update_gated(VelocityController *controller,
                                       float velocity_target_rad_s,
                                       float velocity_measured_rad_s,
                                       float dt_s,
                                       bool integrator_enable);

float velocity_bounded_profile_target_rpm(float elapsed_s,
                                          float total_s,
                                          float target_rpm,
                                          float rise_rate_rpm_per_s,
                                          float fall_rate_rpm_per_s);

float velocity_hold_then_fall_target_rpm(float elapsed_s,
                                         float total_s,
                                         float target_rpm,
                                         float fall_rate_rpm_per_s);

void velocity_continuous_guard_init(VelocityContinuousRunGuard *guard,
                                    uint32_t watchdog_timeout_ms);
bool velocity_continuous_guard_arm(VelocityContinuousRunGuard *guard,
                                   bool preflight_ok,
                                   uint32_t now_ms);
bool velocity_continuous_guard_start(VelocityContinuousRunGuard *guard,
                                     uint32_t now_ms);
void velocity_continuous_guard_heartbeat(VelocityContinuousRunGuard *guard,
                                         uint32_t now_ms);
bool velocity_continuous_guard_poll(VelocityContinuousRunGuard *guard,
                                    bool runtime_safe,
                                    uint32_t now_ms);
void velocity_continuous_guard_stop(VelocityContinuousRunGuard *guard);
void velocity_continuous_guard_latch_fault(VelocityContinuousRunGuard *guard,
                                           uint32_t fault);
bool velocity_continuous_guard_clear_fault(VelocityContinuousRunGuard *guard,
                                           bool outputs_safe);

void velocity_breakaway_probe_init(VelocityBreakawayProbe *probe,
                                   float iq_breakaway_a,
                                   uint32_t timeout_ticks,
                                   int32_t expected_direction,
                                   int32_t min_motion_counts,
                                   uint32_t min_direction_events,
                                   uint32_t max_step_counts);
bool velocity_breakaway_probe_start(VelocityBreakawayProbe *probe,
                                    int64_t encoder_count,
                                    uint32_t illegal_transition_count);
VelocityBreakawayResult velocity_breakaway_probe_update(
    VelocityBreakawayProbe *probe,
    int64_t encoder_count,
    uint32_t illegal_transition_count);
float velocity_breakaway_probe_iq_ref(const VelocityBreakawayProbe *probe);
const char *velocity_breakaway_result_name(VelocityBreakawayResult result);

void velocity_breakaway_handoff_init(VelocityBreakawayHandoff *handoff,
                                     float breakaway_iq_a,
                                     float continuous_iq_limit_a);
bool velocity_breakaway_handoff_start(VelocityBreakawayHandoff *handoff);
float velocity_breakaway_handoff_update(
    VelocityBreakawayHandoff *handoff,
    VelocityBreakawayResult breakaway_result,
    float speed_pi_iq_request_a,
    uint32_t control_tick);
bool velocity_breakaway_handoff_speed_pi_active(
    const VelocityBreakawayHandoff *handoff);
const char *velocity_breakaway_handoff_state_name(
    VelocityBreakawayHandoffState state);

void velocity_count_window_reset(VelocityCountWindow *window);

bool velocity_count_window_set_samples(VelocityCountWindow *window,
                                       uint32_t sample_count);

float velocity_count_window_update_rpm(VelocityCountWindow *window,
                                       int32_t delta_counts,
                                       float sample_period_s,
                                       uint32_t encoder_cpr);

void velocity_edge_period_init(VelocityEdgePeriodEstimator *estimator,
                               int64_t encoder_count,
                               uint32_t stale_ticks);

void velocity_edge_period_sample(VelocityEdgePeriodEstimator *estimator,
                                 int64_t encoder_count);

float velocity_edge_period_rpm(const VelocityEdgePeriodEstimator *estimator,
                               float sample_tick_hz,
                               uint32_t encoder_cpr);

static inline float velocity_first_order_filter_step(float previous,
                                                     float input,
                                                     float alpha)
{
    if (alpha <= 0.0f) {
        return previous;
    }
    if (alpha >= 1.0f) {
        return input;
    }
    return previous + alpha * (input - previous);
}

void velocity_count_window_analyze_overspeed(
    const VelocityCountWindow *window,
    float sample_period_s,
    uint32_t encoder_cpr,
    float hard_speed_limit_rpm,
    VelocityOverspeedAnalysis *analysis);

const char *velocity_overspeed_evidence_name(
    VelocityOverspeedEvidence evidence);

#ifdef __cplusplus
}
#endif

#endif /* VELOCITY_CONTROLLER_H */

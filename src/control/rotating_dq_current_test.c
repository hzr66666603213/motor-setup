#include "control/rotating_dq_current_test.h"

#include "foc/foc_math.h"

#include <math.h>
#include <string.h>

#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F415xx) || defined(STM32F417xx)
#include "stm32f4xx.h"
#define ROTATING_DQ_PROFILE_CYCLES 1
#else
#define ROTATING_DQ_PROFILE_CYCLES 0
#endif

/* Keep one production timing envelope; detailed section profiling is diagnostic-only. */
#define ROTATING_DQ_DETAILED_PROFILE_CYCLES 0

#define ROTATING_DQ_PI_F 3.14159265358979323846f
#define ROTATING_DQ_TWO_PI_F (2.0f * ROTATING_DQ_PI_F)
#define ROTATING_DQ_NEAR_ZERO_IQ_REF_A 0.010f
#define ROTATING_DQ_COMMON_MODE_ABS_COUNTS 10
#define ROTATING_DQ_COMMON_MODE_DIFF_COUNTS 6u

#if defined(__GNUC__)
#define ROTATING_DQ_FAST_OPT __attribute__((optimize("O3")))
#else
#define ROTATING_DQ_FAST_OPT
#endif

static inline bool rdq_finite(float x)
{
    return (x >= -1.0e30f) && (x <= 1.0e30f);
}

#if ROTATING_DQ_PROFILE_CYCLES
static inline uint32_t rdq_total_profile_now(void)
{
    return DWT->CYCCNT;
}

static inline void rdq_total_profile_update(uint32_t *max_cycles,
                                            uint32_t cycles)
{
    if (max_cycles != 0 && cycles > *max_cycles) {
        *max_cycles = cycles;
    }
}
#else
static inline uint32_t rdq_total_profile_now(void)
{
    return 0u;
}


static inline void rdq_total_profile_update(uint32_t *max_cycles,
                                            uint32_t cycles)
{
    (void)max_cycles;
    (void)cycles;
}
#endif

#if ROTATING_DQ_PROFILE_CYCLES && ROTATING_DQ_DETAILED_PROFILE_CYCLES
static inline uint32_t rdq_profile_now(void)
{
    return DWT->CYCCNT;
}

static inline void rdq_profile_update(uint32_t *max_cycles, uint32_t cycles)
{
    if (max_cycles != 0 && cycles > *max_cycles) {
        *max_cycles = cycles;
    }
}
#else
static inline uint32_t rdq_profile_now(void)
{
    return 0u;
}

static inline void rdq_profile_update(uint32_t *max_cycles, uint32_t cycles)
{
    (void)max_cycles;
    (void)cycles;
}
#endif

static uint32_t rdq_ticks_from_ms(uint32_t ms, float dt_s)
{
    const float ticks = ((float)ms * 0.001f) / dt_s;
    return (uint32_t)(ticks + 0.5f);
}

static inline int64_t rdq_abs_i64(int64_t x)
{
    return (x < 0) ? -x : x;
}

static float rdq_wrap_0_2pi(float x)
{
    while (x >= ROTATING_DQ_TWO_PI_F) {
        x -= ROTATING_DQ_TWO_PI_F;
    }
    while (x < 0.0f) {
        x += ROTATING_DQ_TWO_PI_F;
    }
    return x;
}

static bool rdq_state_is_power_test_active(RotatingDqCurrentTestState state)
{
    return state >= ROTATING_DQ_STATE_ENABLE_ZERO &&
           state <= ROTATING_DQ_STATE_HOLD_ZERO_2;
}

static bool rdq_state_is_zero_return(RotatingDqCurrentTestState state)
{
    return state == ROTATING_DQ_STATE_RAMP_ZERO_1 ||
           state == ROTATING_DQ_STATE_RAMP_ZERO_2 ||
           state == ROTATING_DQ_STATE_HOLD_ZERO_1 ||
           state == ROTATING_DQ_STATE_HOLD_ZERO_2;
}

static bool rdq_state_is_zero_output(RotatingDqCurrentTestState state)
{
    return state == ROTATING_DQ_STATE_ENABLE_ZERO ||
           rdq_state_is_zero_return(state);
}

static float rdq_angle_diff_abs(float now, float last)
{
    float d = now - last;
    while (d > ROTATING_DQ_PI_F) {
        d -= ROTATING_DQ_TWO_PI_F;
    }
    while (d < -ROTATING_DQ_PI_F) {
        d += ROTATING_DQ_TWO_PI_F;
    }
    return fabsf(d);
}

static inline float rdq_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static void rdq_fast_sincos(float theta, float *sin_out, float *cos_out)
    ROTATING_DQ_FAST_OPT;
static void rdq_fast_sincos(float theta, float *sin_out, float *cos_out)
{
    while (theta >= ROTATING_DQ_TWO_PI_F) {
        theta -= ROTATING_DQ_TWO_PI_F;
    }
    while (theta < 0.0f) {
        theta += ROTATING_DQ_TWO_PI_F;
    }

    int sin_sign = 1;
    int cos_sign = 1;
    float x = theta;
    if (x > ROTATING_DQ_PI_F) {
        x -= ROTATING_DQ_PI_F;
        sin_sign = -1;
        cos_sign = -1;
    }
    if (x > (0.5f * ROTATING_DQ_PI_F)) {
        x = ROTATING_DQ_PI_F - x;
        cos_sign = -cos_sign;
    }

    const float x2 = x * x;
    const float x4 = x2 * x2;
    const float x6 = x4 * x2;
    const float sin_x = x * (1.0f - (x2 * 0.1666666667f) +
                             (x4 * 0.0083333333f) -
                             (x6 * 0.0001984127f));
    const float cos_x = 1.0f - (x2 * 0.5f) +
                        (x4 * 0.0416666667f) -
                        (x6 * 0.0013888889f);

    *sin_out = (sin_sign > 0) ? sin_x : -sin_x;
    *cos_out = (cos_sign > 0) ? cos_x : -cos_x;
}

static void rdq_stats_reset(RotatingDqDirectionStats *stats)
{
    memset(stats, 0, sizeof(*stats));
}

static void rdq_stats_update_mean_fields(RotatingDqDirectionStats *stats)
{
    if (stats == 0 || stats->sample_count == 0u) {
        return;
    }
    const float inv_n = 1.0f / (float)stats->sample_count;
    stats->iq_ref_mean_a = stats->iq_ref_sum_a * inv_n;
    stats->iq_mean_a = stats->iq_sum_a * inv_n;
    stats->id_mean_a = stats->id_sum_a * inv_n;
    stats->speed_mean_rpm = stats->speed_sum_rpm * inv_n;
}

static void rdq_stats_finalize(RotatingDqDirectionStats *stats)
{
    if (stats == 0 || stats->sample_count == 0u) {
        return;
    }
    rdq_stats_update_mean_fields(stats);
    stats->voltage_vector_peak_v = sqrtf(stats->voltage_vector_peak_sq);
}

static bool rdq_tracking_stats_ok(const RotatingDqCurrentTest *test)
{
    if (test == 0) {
        return false;
    }

    const RotatingDqDirectionStats *pos = &test->positive_stats;
    const RotatingDqDirectionStats *neg = &test->negative_stats;
    const RotatingDqCurrentTestConfig *cfg = &test->config;

    if (cfg->enable_zero_block_integrator &&
        !cfg->enable_zero_diagnostic_only) {
        const RotatingDqBlockIntegratorHoldSnapshot *pos_hold =
            rotating_dq_block_integrator_positive_hold_snapshot();
        const RotatingDqBlockIntegratorHoldSnapshot *neg_hold =
            rotating_dq_block_integrator_negative_hold_snapshot();
        const float pos_den =
            (pos_hold->tracking_sample_count > 0u) ?
                (float)pos_hold->tracking_sample_count : 1.0f;
        const float neg_den =
            (neg_hold->tracking_sample_count > 0u) ?
                (float)neg_hold->tracking_sample_count : 1.0f;
        const float pos_ref_mean = pos_hold->iq_ref_sum_a / pos_den;
        const float neg_ref_mean = neg_hold->iq_ref_sum_a / neg_den;
        const float pos_iq_mean = pos_hold->iq_sum_a / pos_den;
        const float neg_iq_mean = neg_hold->iq_sum_a / neg_den;
        const float pos_id_mean = pos_hold->id_sum_a / pos_den;
        const float neg_id_mean = neg_hold->id_sum_a / neg_den;
        const bool pos_ok =
            pos_hold->valid &&
            pos_hold->tracking_sample_count >=
                ROTATING_DQ_BIPOLAR_HOLD_TRACKING_MIN_SAMPLES &&
            pos_ref_mean >= (0.95f * cfg->iq_target_a) &&
            pos_iq_mean >= cfg->tracking_iq_mean_min_a &&
            rdq_absf(pos_id_mean) <= cfg->tracking_id_mean_abs_limit_a &&
            pos_hold->saturation_count == 0u;
        if (cfg->single_direction_positive_only) {
            return pos_ok;
        }
        const bool neg_ok =
            neg_hold->valid &&
            neg_hold->tracking_sample_count >=
                ROTATING_DQ_BIPOLAR_HOLD_TRACKING_MIN_SAMPLES &&
            neg_ref_mean <= (-0.95f * cfg->iq_target_a) &&
            neg_iq_mean <= -cfg->tracking_iq_mean_min_a &&
            rdq_absf(neg_id_mean) <= cfg->tracking_id_mean_abs_limit_a &&
            neg_hold->saturation_count == 0u;
        return pos_ok && neg_ok;
    }

    const bool pos_ref_ok =
        pos->sample_count > 0u &&
        pos->iq_ref_mean_a >= cfg->tracking_iq_ref_mean_min_a;
    const bool neg_ref_ok =
        neg->sample_count > 0u &&
        neg->iq_ref_mean_a <= -cfg->tracking_iq_ref_mean_min_a;
    const bool pos_iq_ok =
        pos->iq_mean_a >= cfg->tracking_iq_mean_min_a;
    const bool neg_iq_ok =
        neg->iq_mean_a <= -cfg->tracking_iq_mean_min_a;
    if (cfg->single_direction_positive_only) {
        const bool pos_id_ok =
            rdq_absf(pos->id_mean_a) <= cfg->tracking_id_mean_abs_limit_a;
        return pos_ref_ok && pos_iq_ok && pos_id_ok &&
               pos->saturation_count == 0u;
    }
    const bool id_ok =
        rdq_absf(pos->id_mean_a) <= cfg->tracking_id_mean_abs_limit_a &&
        rdq_absf(neg->id_mean_a) <= cfg->tracking_id_mean_abs_limit_a;
    const bool saturation_ok =
        pos->saturation_count == 0u &&
        neg->saturation_count == 0u;

    return pos_ref_ok && neg_ref_ok && pos_iq_ok && neg_iq_ok &&
           id_ok && saturation_ok;
}

static uint32_t rdq_phase_trip_channel(float iu,
                                       float iv,
                                       float iw,
                                       float limit)
{
    uint32_t ch = 0u;
    if (rdq_absf(iu) > limit) { ch |= ROTATING_DQ_ZERO_TRIP_IU; }
    if (rdq_absf(iv) > limit) { ch |= ROTATING_DQ_ZERO_TRIP_IV; }
    if (rdq_absf(iw) > limit) { ch |= ROTATING_DQ_ZERO_TRIP_IW; }
    return ch;
}

static uint32_t rdq_dq_trip_channel(float id, float iq, float limit)
{
    uint32_t ch = 0u;
    if (rdq_absf(id) > limit) { ch |= ROTATING_DQ_ZERO_TRIP_ID; }
    if (rdq_absf(iq) > limit) { ch |= ROTATING_DQ_ZERO_TRIP_IQ; }
    return ch;
}

static void rdq_zero_diag_store(RotatingDqCurrentTest *test,
                                const RotatingDqZeroDiagSample *sample,
                                uint32_t source_line)
{
    if (test == 0 || sample == 0) {
        return;
    }
    if (test->zero_diag_count < ROTATING_DQ_ZERO_DIAG_CAPACITY) {
        test->zero_diag[test->zero_diag_count] = *sample;
        test->zero_diag[test->zero_diag_count].sample_index = test->zero_diag_count;
        test->zero_diag_count++;
    } else {
        test->zero_diag_dropped++;
    }
    if (sample->source_mask != 0u && !test->zero_first_trip.valid) {
        test->zero_first_trip.valid = true;
        test->zero_first_trip.source_line = source_line;
        test->zero_first_trip.control_tick = sample->control_tick;
        test->zero_first_trip.adc_seq = sample->adc_seq;
        test->zero_first_trip.source_mask = sample->source_mask;
        test->zero_first_trip.sample = *sample;
    }
}

static RotatingDqZeroDiagSample *rdq_zero_diag_alloc_slot(RotatingDqCurrentTest *test)
{
    if (test == 0) {
        return 0;
    }
    if (test->zero_diag_count >= ROTATING_DQ_ZERO_DIAG_CAPACITY) {
        test->zero_diag_dropped++;
        return 0;
    }
    RotatingDqZeroDiagSample *sample = &test->zero_diag[test->zero_diag_count];
    memset(sample, 0, sizeof(*sample));
    sample->sample_index = test->zero_diag_count;
    test->zero_diag_count++;
    return sample;
}

static bool __attribute__((unused))
rdq_zero_diag_should_capture_enable_zero(const RotatingDqCurrentTest *test)
{
    if (test == 0) {
        return false;
    }
#if ROTATING_DQ_ENABLE_ZERO_DIAG_LEVEL == 0u
    return false;
#else
    const uint32_t tick = test->control_tick_seq;
    if (tick <= ROTATING_DQ_ENABLE_ZERO_STARTUP_OBSERVE_TICKS) {
        return true;
    }
    if (test->config.enable_zero_diagnostic_ticks != 0u &&
        tick >= test->config.enable_zero_diagnostic_ticks) {
        return true;
    }
    return (tick % 4u) == 0u;
#endif
}

static void __attribute__((unused))
rdq_zero_diag_store_light_sample(RotatingDqCurrentTest *test,
                                             const RotatingDqCurrentTestInput *input,
                                             const CurrentControllerOutput *cc_out,
                                             RotatingDqZeroDiagStage stage,
                                             bool post_shutdown_sample,
                                             uint32_t source_mask,
                                             uint32_t source_line,
                                             float iu,
                                             float i_alpha,
                                             float i_beta,
                                             float id,
                                             float iq,
                                             float sin_theta,
                                             float cos_theta)
{
    if (test == 0 || input == 0) {
        return;
    }
    RotatingDqZeroDiagSample *sample = rdq_zero_diag_alloc_slot(test);
    if (sample == 0) {
        return;
    }

    sample->control_tick = test->control_tick_seq;
    sample->adc_seq = input->adc_seq;
    sample->stage = stage;
    sample->post_shutdown_sample = post_shutdown_sample;
    sample->raw_pc0 = input->raw_pc0;
    sample->raw_pc1 = input->raw_pc1;
    sample->offset_pc0 = input->offset_pc0;
    sample->offset_pc1 = input->offset_pc1;
    sample->delta_pc0 = (int32_t)input->raw_pc0 - (int32_t)input->offset_pc0;
    sample->delta_pc1 = (int32_t)input->raw_pc1 - (int32_t)input->offset_pc1;
    sample->iv_counts = sample->delta_pc0;
    sample->iw_counts = sample->delta_pc1;
    sample->iu_counts = -(sample->iv_counts + sample->iw_counts);
    sample->iv_a = input->iv_a;
    sample->iw_a = input->iw_a;
    sample->iu_a = iu;
    sample->i_alpha_a = i_alpha;
    sample->i_beta_a = i_beta;
    sample->id_a = id;
    sample->iq_a = iq;
    sample->phase_metric_a = rdq_absf(iu);
    if (rdq_absf(input->iv_a) > sample->phase_metric_a) {
        sample->phase_metric_a = rdq_absf(input->iv_a);
    }
    if (rdq_absf(input->iw_a) > sample->phase_metric_a) {
        sample->phase_metric_a = rdq_absf(input->iw_a);
    }
    sample->dq_metric_a = rdq_absf(id);
    if (rdq_absf(iq) > sample->dq_metric_a) {
        sample->dq_metric_a = rdq_absf(iq);
    }
    sample->phase_limit_a = test->config.phase_current_limit_a;
    sample->dq_limit_a = test->config.dq_current_limit_a;
    sample->phase_trip_channel =
        rdq_phase_trip_channel(iu, input->iv_a, input->iw_a,
                               sample->phase_limit_a);
    sample->dq_trip_channel =
        rdq_dq_trip_channel(id, iq, sample->dq_limit_a);
    sample->phase_trip = sample->phase_trip_channel != 0u;
    sample->dq_trip = sample->dq_trip_channel != 0u;
    sample->encoder_count = input->encoder_count;
    sample->electrical_offset_rad = input->electrical_offset_rad;
    sample->theta_electrical_used_rad = input->theta_e_rad;
    sample->sin_theta = sin_theta;
    sample->cos_theta = cos_theta;
    sample->id_ref_a = 0.0f;
    sample->iq_ref_a = test->iq_ref_a;
    sample->id_error_a = sample->id_ref_a - id;
    sample->iq_error_a = sample->iq_ref_a - iq;
    sample->vd_p_v = test->controller.kp * sample->id_error_a;
    sample->vq_p_v = test->controller.kp * sample->iq_error_a;
    sample->vd_i_v = test->controller.integrator_d_v;
    sample->vq_i_v = test->controller.integrator_q_v;
    if (cc_out != 0) {
        sample->vd_unsat_v = cc_out->vd_unsat_v;
        sample->vq_unsat_v = cc_out->vq_unsat_v;
        sample->vd_applied_v = cc_out->vd_v;
        sample->vq_applied_v = cc_out->vq_v;
        sample->voltage_saturated = cc_out->saturation_active;
        sample->v_alpha_v = cc_out->v_alpha_v;
        sample->v_beta_v = cc_out->v_beta_v;
    }
    sample->voltage_vector_sq =
        (sample->vd_applied_v * sample->vd_applied_v) +
        (sample->vq_applied_v * sample->vq_applied_v);
    sample->ccr1 = input->ccr1;
    sample->ccr2 = input->ccr2;
    sample->ccr3 = input->ccr3;
    sample->ccr4 = input->ccr4;
    sample->ccer = input->ccer;
    sample->bdtr = input->bdtr;
    sample->tim1_cnt = input->tim1_cnt;
    sample->adc_rank_order = input->adc_rank_order;
    sample->nfault_ok = input->nfault_ok;
    sample->callback_cycles = input->callback_cycles;
    sample->source_mask = source_mask;

    const float phase_sum_error = rdq_absf(iu + input->iv_a + input->iw_a);
    if (phase_sum_error > test->zero_phase_sum_error_max_a) {
        test->zero_phase_sum_error_max_a = phase_sum_error;
    }

    if (source_mask != 0u && !test->zero_first_trip.valid) {
        test->zero_first_trip.valid = true;
        test->zero_first_trip.source_line = source_line;
        test->zero_first_trip.control_tick = sample->control_tick;
        test->zero_first_trip.adc_seq = sample->adc_seq;
        test->zero_first_trip.source_mask = source_mask;
        test->zero_first_trip.sample = *sample;
    }
}

static void rdq_zero_diag_store_minimal_current_sample(
    RotatingDqCurrentTest *test,
    const RotatingDqCurrentTestInput *input,
    RotatingDqZeroDiagStage stage,
    uint32_t source_mask,
    uint32_t source_line,
    float iu,
    float i_alpha,
    float i_beta,
    float id,
    float iq,
    float sin_theta,
    float cos_theta,
    uint32_t phase_trip_channel,
    uint32_t dq_trip_channel)
{
    if (test == 0 || input == 0) {
        return;
    }

    if (source_mask == 0u || test->zero_first_trip.valid) {
        return;
    }

    RotatingDqZeroDiagSample *sample = &test->zero_first_trip.sample;
    test->zero_first_trip.valid = true;
    test->zero_first_trip.source_line = source_line;
    test->zero_first_trip.control_tick = test->control_tick_seq;
    test->zero_first_trip.adc_seq = input->adc_seq;
    test->zero_first_trip.source_mask = source_mask;

    sample->control_tick = test->control_tick_seq;
    sample->adc_seq = input->adc_seq;
    sample->stage = stage;
    sample->raw_pc0 = input->raw_pc0;
    sample->raw_pc1 = input->raw_pc1;
    sample->offset_pc0 = input->offset_pc0;
    sample->offset_pc1 = input->offset_pc1;
    sample->delta_pc0 = (int32_t)input->raw_pc0 - (int32_t)input->offset_pc0;
    sample->delta_pc1 = (int32_t)input->raw_pc1 - (int32_t)input->offset_pc1;
    sample->iv_counts = sample->delta_pc0;
    sample->iw_counts = sample->delta_pc1;
    sample->iu_counts = -(sample->iv_counts + sample->iw_counts);
    sample->iu_a = iu;
    sample->iv_a = input->iv_a;
    sample->iw_a = input->iw_a;
    sample->i_alpha_a = i_alpha;
    sample->i_beta_a = i_beta;
    sample->id_a = id;
    sample->iq_a = iq;
    float phase_metric = rdq_absf(iu);
    const float iv_abs = rdq_absf(input->iv_a);
    const float iw_abs = rdq_absf(input->iw_a);
    if (iv_abs > phase_metric) {
        phase_metric = iv_abs;
    }
    if (iw_abs > phase_metric) {
        phase_metric = iw_abs;
    }
    const float id_abs = rdq_absf(id);
    const float iq_abs = rdq_absf(iq);
    sample->phase_metric_a = phase_metric;
    sample->dq_metric_a = (id_abs > iq_abs) ? id_abs : iq_abs;
    sample->phase_limit_a = test->config.phase_current_limit_a;
    sample->dq_limit_a = test->config.dq_current_limit_a;
    sample->phase_trip_channel = phase_trip_channel;
    sample->dq_trip_channel = dq_trip_channel;
    sample->phase_trip = sample->phase_trip_channel != 0u;
    sample->dq_trip = sample->dq_trip_channel != 0u;
    sample->encoder_count = input->encoder_count;
    sample->electrical_offset_rad = input->electrical_offset_rad;
    sample->theta_electrical_used_rad = input->theta_e_rad;
    sample->sin_theta = sin_theta;
    sample->cos_theta = cos_theta;
    sample->id_ref_a = 0.0f;
    sample->iq_ref_a = test->iq_ref_a;
    sample->vd_applied_v = 0.0f;
    sample->vq_applied_v = 0.0f;
    sample->v_alpha_v = 0.0f;
    sample->v_beta_v = 0.0f;
    sample->ccr1 = input->ccr1;
    sample->ccr2 = input->ccr2;
    sample->ccr3 = input->ccr3;
    sample->ccr4 = input->ccr4;
    sample->ccer = input->ccer;
    sample->bdtr = input->bdtr;
    sample->tim1_cnt = input->tim1_cnt;
    sample->adc_rank_order = input->adc_rank_order;
    sample->nfault_ok = input->nfault_ok;
    sample->callback_cycles = input->callback_cycles;
    sample->source_mask = source_mask;
}

static void rdq_zero_diag_build_sample(RotatingDqCurrentTest *test,
                                       const RotatingDqCurrentTestInput *input,
                                       const CurrentControllerOutput *cc_out,
                                       RotatingDqZeroDiagStage stage,
                                       bool post_shutdown_sample,
                                       uint32_t fast_core_cycles,
                                       uint32_t source_mask,
                                       RotatingDqZeroDiagSample *sample)
{
    if (sample == 0 || input == 0 || test == 0) {
        return;
    }
    memset(sample, 0, sizeof(*sample));
    sample->control_tick = test->control_tick_seq;
    sample->adc_seq = input->adc_seq;
    sample->stage = stage;
    sample->post_shutdown_sample = post_shutdown_sample;
    sample->raw_pc0 = input->raw_pc0;
    sample->raw_pc1 = input->raw_pc1;
    sample->offset_pc0 = input->offset_pc0;
    sample->offset_pc1 = input->offset_pc1;
    sample->delta_pc0 = (int32_t)input->raw_pc0 - (int32_t)input->offset_pc0;
    sample->delta_pc1 = (int32_t)input->raw_pc1 - (int32_t)input->offset_pc1;
    sample->iv_counts = sample->delta_pc0;
    sample->iw_counts = sample->delta_pc1;
    sample->iu_counts = -(sample->iv_counts + sample->iw_counts);
    const float scale = (input->current_amp_per_count > 0.0f)
                            ? input->current_amp_per_count
                            : 1.0f;
    sample->iv_a = (float)sample->iv_counts * scale;
    sample->iw_a = (float)sample->iw_counts * scale;
    sample->iu_a = (float)sample->iu_counts * scale;
    const float iv_error = sample->iv_a - input->iv_a;
    const float iw_error = sample->iw_a - input->iw_a;
    const float reconstruction_tol = scale * 0.55f;
    if ((rdq_absf(iv_error) > reconstruction_tol) ||
        (rdq_absf(iw_error) > reconstruction_tol)) {
        test->zero_reconstruction_formula_mismatch_count++;
    }
    if (input->current_amp_per_count <= 0.0f) {
        test->zero_reconstruction_scale_mismatch_count++;
    }
    const float phase_sum_error =
        rdq_absf(sample->iu_a + sample->iv_a + sample->iw_a);
    if (phase_sum_error > test->zero_phase_sum_error_max_a) {
        test->zero_phase_sum_error_max_a = phase_sum_error;
    }
    sample->i_alpha_a = sample->iu_a;
    sample->i_beta_a = (sample->iv_a - sample->iw_a) * 0.57735026919f;
    rdq_fast_sincos(input->theta_e_rad, &sample->sin_theta, &sample->cos_theta);
    sample->id_a = (sample->cos_theta * sample->i_alpha_a) +
                   (sample->sin_theta * sample->i_beta_a);
    sample->iq_a = (-sample->sin_theta * sample->i_alpha_a) +
                   (sample->cos_theta * sample->i_beta_a);
    sample->phase_metric_a = rdq_absf(sample->iu_a);
    if (rdq_absf(sample->iv_a) > sample->phase_metric_a) {
        sample->phase_metric_a = rdq_absf(sample->iv_a);
    }
    if (rdq_absf(sample->iw_a) > sample->phase_metric_a) {
        sample->phase_metric_a = rdq_absf(sample->iw_a);
    }
    sample->dq_metric_a = rdq_absf(sample->id_a);
    if (rdq_absf(sample->iq_a) > sample->dq_metric_a) {
        sample->dq_metric_a = rdq_absf(sample->iq_a);
    }
    sample->phase_limit_a = test->config.phase_current_limit_a;
    sample->dq_limit_a = test->config.dq_current_limit_a;
    sample->phase_trip_channel =
        rdq_phase_trip_channel(sample->iu_a, sample->iv_a, sample->iw_a,
                               sample->phase_limit_a);
    sample->dq_trip_channel =
        rdq_dq_trip_channel(sample->id_a, sample->iq_a, sample->dq_limit_a);
    sample->phase_trip = sample->phase_trip_channel != 0u;
    sample->dq_trip = sample->dq_trip_channel != 0u;
    sample->encoder_count = input->encoder_count;
    sample->theta_mech_rad = (test->config.encoder_cpr > 0)
                                 ? (ROTATING_DQ_TWO_PI_F *
                                    (float)input->encoder_count /
                                    (float)test->config.encoder_cpr)
                                 : 0.0f;
    sample->electrical_offset_rad = input->electrical_offset_rad;
    sample->theta_electrical_raw_rad =
        ((float)test->config.encoder_direction *
         (float)test->config.pole_pairs * sample->theta_mech_rad) +
        input->electrical_offset_rad;
    sample->theta_electrical_used_rad = input->theta_e_rad;
    if (rdq_angle_diff_abs(rdq_wrap_0_2pi(sample->theta_electrical_raw_rad),
                           input->theta_e_rad) > 0.005f) {
        test->zero_park_transform_mismatch_count++;
    }
    sample->id_ref_a = 0.0f;
    sample->iq_ref_a = test->iq_ref_a;
    sample->id_error_a = sample->id_ref_a - sample->id_a;
    sample->iq_error_a = sample->iq_ref_a - sample->iq_a;
    sample->vd_p_v = test->controller.kp * sample->id_error_a;
    sample->vq_p_v = test->controller.kp * sample->iq_error_a;
    sample->vd_i_v = test->controller.integrator_d_v;
    sample->vq_i_v = test->controller.integrator_q_v;
    if (cc_out != 0) {
        sample->vd_unsat_v = cc_out->vd_unsat_v;
        sample->vq_unsat_v = cc_out->vq_unsat_v;
        sample->vd_applied_v = cc_out->vd_v;
        sample->vq_applied_v = cc_out->vq_v;
        sample->voltage_saturated = cc_out->saturation_active;
        sample->v_alpha_v = cc_out->v_alpha_v;
        sample->v_beta_v = cc_out->v_beta_v;
    }
    sample->voltage_vector_sq =
        (sample->vd_applied_v * sample->vd_applied_v) +
        (sample->vq_applied_v * sample->vq_applied_v);
    sample->ccr1 = input->ccr1;
    sample->ccr2 = input->ccr2;
    sample->ccr3 = input->ccr3;
    sample->ccr4 = input->ccr4;
    sample->ccer = input->ccer;
    sample->bdtr = input->bdtr;
    sample->tim1_cnt = input->tim1_cnt;
    sample->adc_rank_order = input->adc_rank_order;
    sample->nfault_ok = input->nfault_ok;
    sample->fast_core_cycles = fast_core_cycles;
    sample->callback_cycles = input->callback_cycles;
    sample->source_mask = source_mask;
}

void rotating_dq_current_test_capture_zero_diag_sample(
    RotatingDqCurrentTest *test,
    const RotatingDqCurrentTestInput *input,
    RotatingDqZeroDiagStage stage,
    bool post_shutdown_sample,
    uint32_t fast_core_cycles)
{
    RotatingDqZeroDiagSample sample;
    memset(&sample, 0, sizeof(sample));
    rdq_zero_diag_build_sample(test, input, 0, stage, post_shutdown_sample,
                               fast_core_cycles, 0u, &sample);
    rdq_zero_diag_store(test, &sample, 0u);
}

static inline void rdq_zero_stats_update_fast(RotatingDqDirectionStats *stats,
                                              float id_a,
                                              float iq_a,
                                              float phase_abs)
{
    stats->sample_count++;
    stats->id_sum_a += id_a;
    stats->iq_sum_a += iq_a;
    if (phase_abs > stats->phase_current_peak_a) {
        stats->phase_current_peak_a = phase_abs;
    }
}

static inline void rdq_direction_stats_update_fast(RotatingDqDirectionStats *stats,
                                                   float iq_ref_a,
                                                   float iq_a,
                                                   float id_a,
                                                   float mechanical_speed_rpm,
                                                   float phase_abs,
                                                   bool saturation_active,
                                                   int64_t encoder_delta)
{
    const float speed_abs = rdq_absf(mechanical_speed_rpm);

    stats->sample_count++;
    stats->iq_ref_sum_a += iq_ref_a;
    stats->iq_sum_a += iq_a;
    stats->id_sum_a += id_a;
    stats->speed_sum_rpm += mechanical_speed_rpm;
    stats->encoder_delta_counts += encoder_delta;
    if (stats->encoder_delta_counts > 0) {
        stats->mechanical_direction = 1;
    } else if (stats->encoder_delta_counts < 0) {
        stats->mechanical_direction = -1;
    }
    if (speed_abs > stats->speed_peak_rpm) {
        stats->speed_peak_rpm = speed_abs;
    }
    if (phase_abs > stats->phase_current_peak_a) {
        stats->phase_current_peak_a = phase_abs;
    }
    if (saturation_active) {
        stats->saturation_count++;
    }
}

static float rdq_block_inverse_count(uint32_t count)
{
    switch (count) {
    case 28u: return 0.0357142857f;
    case 29u: return 0.0344827586f;
    case 30u: return 0.0333333333f;
    case 31u: return 0.0322580645f;
    case 32u: return 0.03125f;
    default: return 0.0f;
    }
}

/* E-group diagnostics are single-instance and live in ordinary SRAM. */
static RotatingDqBlockIntegratorAdmission g_rdq_block_integrator;
static RotatingDqBlockIntegratorHoldSnapshot g_rdq_positive_hold_snapshot;
static RotatingDqBlockIntegratorHoldSnapshot g_rdq_negative_hold_snapshot;
static volatile bool g_rdq_hold_snapshot_pending;
static RotatingDqCurrentTestState g_rdq_hold_snapshot_pending_state;
static bool g_rdq_external_block_ref_valid;
static float g_rdq_external_block_ref_a;
static bool g_rdq_external_block_collect_latched;

static void rdq_external_block_reference_reset(void)
{
    g_rdq_external_block_ref_valid = false;
    g_rdq_external_block_ref_a = 0.0f;
    g_rdq_external_block_collect_latched = false;
}

void rotating_dq_block_integrator_reset(
    RotatingDqBlockIntegratorAdmission *admission)
{
    if (admission != 0) {
        memset(admission, 0, sizeof(*admission));
    }
}

const RotatingDqBlockIntegratorAdmission *
rotating_dq_block_integrator_diagnostic_state(void)
{
    return &g_rdq_block_integrator;
}

const RotatingDqBlockIntegratorHoldSnapshot *
rotating_dq_block_integrator_positive_hold_snapshot(void)
{
    return &g_rdq_positive_hold_snapshot;
}

const RotatingDqBlockIntegratorHoldSnapshot *
rotating_dq_block_integrator_negative_hold_snapshot(void)
{
    return &g_rdq_negative_hold_snapshot;
}

static void rdq_block_hold_snapshots_reset(void)
{
    memset(&g_rdq_positive_hold_snapshot, 0,
           sizeof(g_rdq_positive_hold_snapshot));
    memset(&g_rdq_negative_hold_snapshot, 0,
           sizeof(g_rdq_negative_hold_snapshot));
    g_rdq_hold_snapshot_pending = false;
    g_rdq_hold_snapshot_pending_state = ROTATING_DQ_STATE_PREFLIGHT;
}

static RotatingDqBlockIntegratorHoldSnapshot *
rdq_block_hold_snapshot_for_state(RotatingDqCurrentTestState state)
{
    if (state == ROTATING_DQ_STATE_HOLD_IQ_POSITIVE) {
        return &g_rdq_positive_hold_snapshot;
    }
    if (state == ROTATING_DQ_STATE_HOLD_IQ_NEGATIVE) {
        return &g_rdq_negative_hold_snapshot;
    }
    return 0;
}

static void rdq_block_hold_snapshot_update(RotatingDqCurrentTestState state,
                                           uint32_t control_tick,
                                           float iq_ref_a,
                                           float id_a,
                                           float iq_a,
                                           float integrator_q_v,
                                           bool saturation_active)
{
    RotatingDqBlockIntegratorHoldSnapshot *snapshot =
        rdq_block_hold_snapshot_for_state(state);
    if (snapshot == 0) {
        return;
    }
    if (snapshot->hold_sample_count == 0u) {
        snapshot->hold_state = state;
        snapshot->start_tick = control_tick;
        snapshot->integrator_q_start_v = integrator_q_v;
        snapshot->integrator_q_min_v = integrator_q_v;
        snapshot->integrator_q_max_v = integrator_q_v;
    }
    snapshot->end_tick = control_tick;
    snapshot->hold_sample_count++;
    snapshot->integrator_q_end_v = integrator_q_v;
    if (integrator_q_v < snapshot->integrator_q_min_v) {
        snapshot->integrator_q_min_v = integrator_q_v;
    }
    if (integrator_q_v > snapshot->integrator_q_max_v) {
        snapshot->integrator_q_max_v = integrator_q_v;
    }
    if (g_rdq_block_integrator.iq_axis.admitted &&
        snapshot->tracking_sample_count <
            ROTATING_DQ_BIPOLAR_HOLD_TRACKING_WINDOW_SAMPLES) {
        if (snapshot->tracking_sample_count == 0u) {
            snapshot->tracking_start_tick = control_tick;
        }
        snapshot->tracking_end_tick = control_tick;
        snapshot->tracking_sample_count++;
        snapshot->iq_ref_sum_a += iq_ref_a;
        snapshot->id_sum_a += id_a;
        snapshot->iq_sum_a += iq_a;
        if (saturation_active) {
            snapshot->saturation_count++;
        }
    }
}

static bool rdq_bounded_runtime_diagnostics_complete(
    const RotatingDqCurrentTest *test)
{
    if (test == 0 || test->config.direction_capture_counts < 1000000) {
        return false;
    }
    if (test->config.enable_external_iq_block_integrator) {
        const RotatingDqBlockIntegratorHoldSnapshot *snapshot =
            rotating_dq_block_integrator_positive_hold_snapshot();
        return snapshot != 0 &&
               snapshot->tracking_sample_count >=
                   ROTATING_DQ_BIPOLAR_HOLD_TRACKING_WINDOW_SAMPLES;
    }
    return test->positive_stats.sample_count >=
           ROTATING_DQ_DIRECTION_STATS_MAX_SAMPLES;
}

static bool rdq_block_hold_tracking_window_complete(
    RotatingDqCurrentTestState state)
{
    const RotatingDqBlockIntegratorHoldSnapshot *snapshot =
        rdq_block_hold_snapshot_for_state(state);
    return snapshot != 0 &&
           snapshot->tracking_sample_count >=
               ROTATING_DQ_BIPOLAR_HOLD_TRACKING_WINDOW_SAMPLES;
}

static void rdq_block_hold_snapshot_finalize(RotatingDqCurrentTestState state)
{
    RotatingDqBlockIntegratorHoldSnapshot *snapshot =
        rdq_block_hold_snapshot_for_state(state);
    if (snapshot == 0 || snapshot->hold_sample_count == 0u ||
        snapshot->valid || g_rdq_hold_snapshot_pending) {
        return;
    }
    g_rdq_hold_snapshot_pending_state = state;
    g_rdq_hold_snapshot_pending = true;
}

static void rdq_block_hold_snapshot_service_main(void)
{
    if (!g_rdq_hold_snapshot_pending) {
        return;
    }
    RotatingDqBlockIntegratorHoldSnapshot *snapshot =
        rdq_block_hold_snapshot_for_state(g_rdq_hold_snapshot_pending_state);
    if (snapshot != 0 && snapshot->hold_sample_count != 0u &&
        !snapshot->valid) {
        snapshot->admission = g_rdq_block_integrator;
        snapshot->valid = true;
    }
    g_rdq_hold_snapshot_pending = false;
}

static bool rdq_block_axis_finish(RotatingDqBlockIntegratorAxis *axis,
                                  float mean_error_a,
                                  uint32_t control_tick,
                                  float on_threshold_a,
                                  float off_threshold_a,
                                  int8_t required_sign)
{
    const float abs_mean = rdq_absf(mean_error_a);
    const int8_t sign = (mean_error_a > 0.0f) ? 1 :
                        (mean_error_a < 0.0f) ? -1 : 0;
    if (abs_mean > axis->mean_error_peak_a) {
        axis->mean_error_peak_a = abs_mean;
    }

    if (!axis->admitted) {
        if (abs_mean >= on_threshold_a && sign != 0 &&
            (required_sign == 0 || sign == required_sign)) {
            if (axis->candidate_sign == sign) {
                axis->on_block_count++;
            } else {
                axis->candidate_sign = sign;
                axis->on_block_count = 1u;
            }
            if (axis->on_block_count >= ROTATING_DQ_BLOCK_INTEGRATOR_ON_BLOCKS) {
                axis->admitted = true;
                axis->admitted_sign = sign;
                axis->candidate_sign = 0;
                axis->on_block_count = 0u;
                axis->off_block_count = 0u;
                axis->admit_count++;
                axis->last_admit_tick = control_tick;
                if (axis->first_admit_tick == 0u) {
                    axis->first_admit_tick = control_tick;
                }
            }
        } else {
            axis->candidate_sign = 0;
            axis->on_block_count = 0u;
        }
        return false;
    }

    if (sign != 0 && sign != axis->admitted_sign &&
        abs_mean >= off_threshold_a) {
        axis->admitted = false;
        axis->candidate_sign = sign;
        axis->on_block_count =
            (abs_mean >= on_threshold_a &&
             (required_sign == 0 || sign == required_sign)) ? 1u : 0u;
        axis->off_block_count = 0u;
        axis->sign_reversal_count++;
        return false;
    }
    if (abs_mean <= off_threshold_a) {
        axis->off_block_count++;
        if (axis->off_block_count >= ROTATING_DQ_BLOCK_INTEGRATOR_OFF_BLOCKS) {
            axis->admitted = false;
            axis->admitted_sign = 0;
            axis->off_block_count = 0u;
        }
        return false;
    }

    axis->off_block_count = 0u;
    axis->active_block_count++;
    return true;
}

static bool rdq_block_integrator_step_policy(
    RotatingDqBlockIntegratorAdmission *admission,
    uint32_t control_tick,
    float id_error_a,
    float iq_error_a,
    bool sample_valid,
    float ki_v_per_a_s,
    float dt_s,
    float on_threshold_a,
    float off_threshold_a,
    int8_t required_iq_sign,
    bool integrate_d_axis,
    float *id_integrator_delta_v,
    float *iq_integrator_delta_v)
{
    if (id_integrator_delta_v != 0) { *id_integrator_delta_v = 0.0f; }
    if (iq_integrator_delta_v != 0) { *iq_integrator_delta_v = 0.0f; }
    if (admission == 0 || id_integrator_delta_v == 0 ||
        iq_integrator_delta_v == 0 || !rdq_finite(id_error_a) ||
        !rdq_finite(iq_error_a) || !rdq_finite(ki_v_per_a_s) ||
        !rdq_finite(dt_s) || ki_v_per_a_s < 0.0f || dt_s <= 0.0f) {
        return false;
    }

    admission->block_tick_count++;
    if (sample_valid) {
        admission->id_error_sum_a += id_error_a;
        admission->iq_error_sum_a += iq_error_a;
        admission->valid_count++;
    }
    if (admission->block_tick_count < ROTATING_DQ_BLOCK_INTEGRATOR_TICKS) {
        return false;
    }

    admission->completed_block_count++;
    const uint32_t valid_count = admission->valid_count;
    const float id_sum = admission->id_error_sum_a;
    const float iq_sum = admission->iq_error_sum_a;
    admission->block_tick_count = 0u;
    admission->valid_count = 0u;
    admission->id_error_sum_a = 0.0f;
    admission->iq_error_sum_a = 0.0f;

    const float inverse_count = rdq_block_inverse_count(valid_count);
    if (valid_count < ROTATING_DQ_BLOCK_INTEGRATOR_MIN_VALID ||
        inverse_count <= 0.0f) {
        admission->invalid_block_count++;
        admission->id_axis.admitted = false;
        admission->iq_axis.admitted = false;
        admission->id_axis.candidate_sign = 0;
        admission->iq_axis.candidate_sign = 0;
        admission->id_axis.admitted_sign = 0;
        admission->iq_axis.admitted_sign = 0;
        admission->id_axis.on_block_count = 0u;
        admission->iq_axis.on_block_count = 0u;
        admission->id_axis.off_block_count = 0u;
        admission->iq_axis.off_block_count = 0u;
        return true;
    }

    admission->last_id_mean_error_a = id_sum * inverse_count;
    admission->last_iq_mean_error_a = iq_sum * inverse_count;
    const bool integrate_d = integrate_d_axis && rdq_block_axis_finish(
        &admission->id_axis,
        admission->last_id_mean_error_a,
        control_tick,
        on_threshold_a,
        off_threshold_a,
        0);
    const bool integrate_q = rdq_block_axis_finish(
        &admission->iq_axis,
        admission->last_iq_mean_error_a,
        control_tick,
        on_threshold_a,
        off_threshold_a,
        required_iq_sign);
    if (integrate_d) {
        *id_integrator_delta_v = ki_v_per_a_s * dt_s * id_sum;
    }
    if (integrate_q) {
        *iq_integrator_delta_v = ki_v_per_a_s * dt_s * iq_sum;
    }
    return true;
}

bool rotating_dq_block_integrator_step(
    RotatingDqBlockIntegratorAdmission *admission,
    uint32_t control_tick,
    float id_error_a,
    float iq_error_a,
    bool sample_valid,
    float ki_v_per_a_s,
    float dt_s,
    float *id_integrator_delta_v,
    float *iq_integrator_delta_v)
{
    return rdq_block_integrator_step_policy(
        admission,
        control_tick,
        id_error_a,
        iq_error_a,
        sample_valid,
        ki_v_per_a_s,
        dt_s,
        ROTATING_DQ_BLOCK_INTEGRATOR_ON_THRESHOLD_A,
        ROTATING_DQ_BLOCK_INTEGRATOR_OFF_THRESHOLD_A,
        0,
        true,
        id_integrator_delta_v,
        iq_integrator_delta_v);
}

RotatingDqCurrentTestConfig rotating_dq_current_test_default_config(void)
{
    RotatingDqCurrentTestConfig cfg;
    cfg.phase_resistance_ohm = 3.20f;
    cfg.phase_inductance_h = 0.00066f;
    cfg.bandwidth_hz = 100.0f;
    cfg.voltage_limit_v = 1.00f;
    cfg.kaw = 2.0f * ROTATING_DQ_PI_F * 100.0f;
    cfg.integrator_limit_v = 1.00f;
    cfg.dt_s = 0.00005f;
    cfg.iq_target_a = 0.02f;
    cfg.iq_ramp_rate_a_per_s = 0.05f;
    cfg.iq_ref_hard_limit_a = 0.03f;
    cfg.phase_current_limit_a = 0.20f;
    cfg.dq_current_limit_a = 0.15f;
    cfg.zero_phase_current_peak_limit_a = 0.20f;
    cfg.zero_dq_mean_limit_a = 0.04f;
    cfg.zero_clean_sample_min = 96u;
    cfg.speed_limit_rpm = 100.0f;
    cfg.one_rev_limit_counts = 4096;
    cfg.zero_encoder_limit_counts = 32;
    cfg.angle_jump_limit_rad = 0.25f;
    cfg.tracking_error_limit_a = 0.10f;
    cfg.tracking_iq_ref_mean_min_a = 0.005f;
    cfg.tracking_iq_mean_min_a = 0.010f;
    cfg.tracking_id_mean_abs_limit_a = 0.10f;
    cfg.tracking_error_limit_ticks = rdq_ticks_from_ms(50u, cfg.dt_s);
    cfg.saturation_limit_ticks = rdq_ticks_from_ms(20u, cfg.dt_s);
    cfg.zero_startup_guard_ticks = 16u;
    cfg.enable_zero_ticks = rdq_ticks_from_ms(300u, cfg.dt_s);
    cfg.iq_hold_ticks = rdq_ticks_from_ms(200u, cfg.dt_s);
    cfg.hold_zero_ticks = rdq_ticks_from_ms(200u, cfg.dt_s);
    cfg.direction_capture_counts = 32;
    cfg.log_decimation = 200u;
    cfg.enable_zero_diagnostic_only = false;
    cfg.enable_zero_current_pi = false;
    cfg.freeze_zero_reference_integrator = false;
    cfg.enable_zero_block_integrator = false;
    cfg.enable_external_iq_block_integrator = false;
    cfg.zero_window_observe_only = false;
    cfg.require_direction_match = true;
    cfg.single_direction_positive_only = false;
    cfg.enable_zero_diagnostic_ticks = 28u;
    cfg.vbus_min_v = 7.0f;
    cfg.vbus_max_v = 13.0f;
    cfg.control_time_limit_us = 20.0f;
    cfg.encoder_cpr = 4096;
    cfg.encoder_direction = 1;
    cfg.pole_pairs = 7;
    return cfg;
}

void rotating_dq_current_test_init(RotatingDqCurrentTest *test,
                                   const RotatingDqCurrentTestConfig *config)
{
    if (test == 0) {
        return;
    }

    memset(test, 0, sizeof(*test));
    test->config = (config != 0) ? *config : rotating_dq_current_test_default_config();
    if (test->config.enable_zero_block_integrator ||
        test->config.enable_external_iq_block_integrator) {
        rotating_dq_block_integrator_reset(&g_rdq_block_integrator);
        rdq_block_hold_snapshots_reset();
        rdq_external_block_reference_reset();
    }
    current_controller_init(&test->controller,
                            0.0f,
                            0.0f,
                            test->config.voltage_limit_v);
    current_controller_tune_from_rl(&test->controller,
                                    test->config.phase_resistance_ohm,
                                    test->config.phase_inductance_h,
                                    test->config.bandwidth_hz,
                                    test->config.voltage_limit_v);
    current_controller_set_antiwindup(&test->controller,
                                      test->config.kaw,
                                      test->config.integrator_limit_v);
    test->state = ROTATING_DQ_STATE_PREFLIGHT;
    test->result = ROTATING_DQ_RESULT_RUNNING;
}

static void rdq_fail(RotatingDqCurrentTest *test, uint32_t fault_code)
{
    rdq_block_hold_snapshot_finalize(test->state);
    test->fault_code |= fault_code;
    test->state = ROTATING_DQ_STATE_FAIL;
    test->result = ROTATING_DQ_RESULT_FAIL;
    test->iq_ref_a = 0.0f;
    current_controller_reset(&test->controller);
}

void rotating_dq_current_test_request_start(RotatingDqCurrentTest *test,
                                            bool electrical_offset_valid)
{
    if (test == 0 ||
        test->result != ROTATING_DQ_RESULT_RUNNING ||
        test->fault_code != ROTATING_DQ_FAULT_NONE) {
        return;
    }
    if (!electrical_offset_valid) {
        rdq_fail(test, ROTATING_DQ_FAULT_ELECTRICAL_OFFSET_INVALID);
        return;
    }

    test->state = ROTATING_DQ_STATE_ENABLE_ZERO;
    test->state_ticks = 0u;
    test->control_tick_seq = 0u;
    test->voltage_command_seq = 0u;
    test->missed_control_tick_count = 0u;
    test->duplicate_control_tick_count = 0u;
    test->last_adc_seq = 0u;
    test->have_last_adc_seq = false;
    test->have_last_encoder_count = false;
    test->have_last_theta = false;
    test->encoder_start_count = 0;
    test->encoder_last_count = 0;
    test->encoder_motion_max_counts = 0;
    test->theta_last_rad = 0.0f;
    test->mechanical_speed_rpm = 0.0f;
    test->speed_peak_rpm = 0.0f;
    test->iq_ref_a = 0.0f;
    test->tracking_error_ticks = 0u;
    test->saturation_ticks = 0u;
    test->log_count = 0u;
    test->log_dropped = 0u;
    test->zero_diag_count = 0u;
    test->zero_diag_dropped = 0u;
    memset(&test->zero_first_trip, 0, sizeof(test->zero_first_trip));
    test->zero_phase_fault_set_tick = 0u;
    test->zero_dq_fault_set_tick = 0u;
    test->zero_phase_startup_over_limit_count = 0u;
    test->zero_phase_startup_over_limit_last_tick = 0u;
    test->zero_phase_startup_over_limit_max_a = 0.0f;
    test->zero_dq_startup_over_limit_count = 0u;
    test->zero_dq_startup_over_limit_last_tick = 0u;
    test->zero_dq_startup_over_limit_max_a = 0.0f;
    test->zero_phase_over_limit_consecutive = 0u;
    test->zero_phase_over_limit_consecutive_max = 0u;
    test->zero_dq_over_limit_consecutive = 0u;
    test->zero_dq_over_limit_consecutive_max = 0u;
    test->zero_common_mode_shift_count = 0u;
    test->zero_common_mode_harmless_count = 0u;
    test->zero_common_mode_max_counts = 0u;
    test->zero_common_mode_diff_max_counts = 0u;
    test->zero_measured_phase_metric_max_a = 0.0f;
    test->zero_reconstructed_phase_metric_max_a = 0.0f;
    test->zero_clean_phase_metric_max_a = 0.0f;
    test->zero_direct_metric_peak_tick = 0u;
    test->zero_reconstructed_metric_peak_tick = 0u;
    test->zero_clean_metric_peak_tick = 0u;
    test->zero_clean_sample_count = 0u;
    test->zero_common_mode_excluded_count = 0u;
    test->zero_common_mode_phase_exclusion_count = 0u;
    test->zero_common_mode_dq_exclusion_count = 0u;
    test->zero_counterfactual_phase_metric_max_a = 0.0f;
    test->zero_raw_dq_metric_max_a = 0.0f;
    test->zero_counterfactual_dq_metric_max_a = 0.0f;
    test->zero_startup_direct_outlier_count = 0u;
    test->zero_startup_first_outlier_tick = 0u;
    test->zero_startup_last_outlier_tick = 0u;
    test->zero_startup_pc0_peak_delta_counts = 0;
    test->zero_startup_pc1_peak_delta_counts = 0;
    test->zero_startup_direct_metric_max_a = 0.0f;
    test->zero_startup_reconstructed_metric_max_a = 0.0f;
    test->zero_startup_dq_metric_max_a = 0.0f;
    test->zero_fault_bit_order = 0u;
    test->zero_reconstruction_formula_mismatch_count = 0u;
    test->zero_reconstruction_scale_mismatch_count = 0u;
    test->zero_stale_snapshot_count = 0u;
    test->zero_clarke_transform_mismatch_count = 0u;
    test->zero_park_transform_mismatch_count = 0u;
    test->zero_dq_norm_amplification_count = 0u;
    test->zero_phase_sum_error_max_a = 0.0f;
    test->zero_diag_completed = false;
    memset(&test->fast_profile, 0, sizeof(test->fast_profile));
    if (test->config.enable_zero_block_integrator ||
        test->config.enable_external_iq_block_integrator) {
        rotating_dq_block_integrator_reset(&g_rdq_block_integrator);
        rdq_block_hold_snapshots_reset();
        rdq_external_block_reference_reset();
    }
    current_controller_reset(&test->controller);
    rdq_stats_reset(&test->zero_stats);
    rdq_stats_reset(&test->positive_stats);
    rdq_stats_reset(&test->negative_stats);
}

void rotating_dq_current_test_force_fault(RotatingDqCurrentTest *test,
                                          uint32_t fault_code)
{
    if (test == 0 || fault_code == ROTATING_DQ_FAULT_NONE) {
        return;
    }
    if (test->result == ROTATING_DQ_RESULT_FAIL) {
        test->fault_code |= fault_code;
        return;
    }
    if (test->result == ROTATING_DQ_RESULT_PASS) {
        test->fault_code |= fault_code;
        test->result = ROTATING_DQ_RESULT_FAIL;
        test->state = ROTATING_DQ_STATE_FAIL;
        test->iq_ref_a = 0.0f;
        current_controller_reset(&test->controller);
        return;
    }
    rdq_fail(test, fault_code);
}

void rotating_dq_current_test_force_complete(RotatingDqCurrentTest *test)
{
    if (test == 0 || test->result != ROTATING_DQ_RESULT_RUNNING) {
        return;
    }
    test->iq_ref_a = 0.0f;
    current_controller_reset(&test->controller);
    test->state = ROTATING_DQ_STATE_COMPLETE;
    test->state_ticks = 0u;
    test->result = ROTATING_DQ_RESULT_PASS;
}

void rotating_dq_current_test_note_execution_time(RotatingDqCurrentTest *test,
                                                  uint32_t cycles,
                                                  float time_us)
{
    if (test == 0) {
        return;
    }
    if (!rdq_finite(time_us)) {
        rdq_fail(test, ROTATING_DQ_FAULT_NAN_INF);
        return;
    }
    if (time_us > test->worst_case_control_time_us) {
        test->worst_case_control_time_us = time_us;
        test->worst_case_control_cycles = cycles;
    }
    if (test->result == ROTATING_DQ_RESULT_RUNNING &&
        test->config.control_time_limit_us > 0.0f &&
        time_us > test->config.control_time_limit_us) {
        rdq_fail(test, ROTATING_DQ_FAULT_CONTROL_TIME);
    }
}

uint32_t rotating_dq_current_test_supervisor_timeout_ms(
    const RotatingDqCurrentTestConfig *config,
    uint32_t margin_ms)
{
    if (config == 0 || !rdq_finite(config->dt_s) || config->dt_s <= 0.0f) {
        return margin_ms;
    }

    uint64_t state_ticks;
    if (config->enable_zero_diagnostic_only) {
        state_ticks = config->enable_zero_diagnostic_ticks;
        if (state_ticks < config->enable_zero_ticks) {
            state_ticks = config->enable_zero_ticks;
        }
        const uint64_t minimum_zero_window_ticks =
            (uint64_t)config->zero_startup_guard_ticks + 128u;
        if (state_ticks < minimum_zero_window_ticks) {
            state_ticks = minimum_zero_window_ticks;
        }
    } else {
        const float ramp_step_a = config->iq_ramp_rate_a_per_s * config->dt_s;
        uint64_t ramp_ticks = 0u;
        if (rdq_finite(ramp_step_a) && ramp_step_a > 0.0f) {
            const float ramp_ticks_f = rdq_absf(config->iq_target_a) / ramp_step_a;
            ramp_ticks = (uint64_t)ramp_ticks_f;
            if ((float)ramp_ticks < ramp_ticks_f) {
                ramp_ticks++;
            }
        }

        state_ticks = (uint64_t)config->enable_zero_ticks +
                      (2u * ramp_ticks) +
                      config->iq_hold_ticks +
                      config->hold_zero_ticks;
        if (!config->single_direction_positive_only) {
            state_ticks += (2u * ramp_ticks) +
                           config->iq_hold_ticks +
                           config->hold_zero_ticks;
        }
    }

    const float runtime_ms_f = (float)state_ticks * config->dt_s * 1000.0f;
    uint64_t runtime_ms = (uint64_t)runtime_ms_f;
    if ((float)runtime_ms < runtime_ms_f) {
        runtime_ms++;
    }
    runtime_ms += margin_ms;
    return (runtime_ms > UINT32_MAX) ? UINT32_MAX : (uint32_t)runtime_ms;
}

bool rotating_dq_current_test_offset_admission_ok(
    const RotatingDqCurrentTestConfig *config,
    const RotatingDqOffsetAdmission *admission)
{
    const RotatingDqCurrentTestConfig def = rotating_dq_current_test_default_config();
    const RotatingDqCurrentTestConfig *cfg = (config != 0) ? config : &def;
    if (admission == 0 ||
        admission->sample_count < 128u ||
        !admission->adc_valid ||
        !admission->nfault_ok ||
        admission->fault_code != 0u) {
        return false;
    }
    return fabsf(admission->iv_mean_a) < 0.03f &&
           fabsf(admission->iw_mean_a) < 0.03f &&
           fabsf(admission->iu_mean_a) < 0.03f &&
           fabsf(admission->id_mean_a) < 0.03f &&
           fabsf(admission->iq_mean_a) < 0.03f &&
           cfg->phase_current_limit_a > 0.0f;
}

static void rdq_zero_controller_output(CurrentControllerOutput *output)
{
    if (output == 0) {
        return;
    }
    memset(output, 0, sizeof(*output));
}

static void rdq_current_controller_update_fast(CurrentController *controller,
                                               const CurrentControllerInput *input,
                                               float sin_theta,
                                               float cos_theta,
                                               float vq_feedforward_v,
                                               bool freeze_integrator,
                                               bool defer_integrator_update,
                                               CurrentControllerOutput *output)
    ROTATING_DQ_FAST_OPT;
static void rdq_current_controller_update_fast(CurrentController *controller,
                                               const CurrentControllerInput *input,
                                               float sin_theta,
                                               float cos_theta,
                                               float vq_feedforward_v,
                                               bool freeze_integrator,
                                               bool defer_integrator_update,
                                               CurrentControllerOutput *output)
{
    if (controller == 0 || input == 0 || output == 0) {
        if (controller != 0) {
            current_controller_reset(controller);
        }
        return;
    }

    const bool input_ok =
        rdq_finite(input->id_ref_a) &&
        rdq_finite(input->iq_ref_a) &&
        rdq_finite(input->id_measured_a) &&
        rdq_finite(input->iq_measured_a) &&
        rdq_finite(input->vbus_v) &&
        rdq_finite(input->dt_s) &&
        rdq_finite(sin_theta) &&
        rdq_finite(cos_theta) &&
        rdq_finite(controller->kp) &&
        rdq_finite(controller->ki) &&
        rdq_finite(controller->kaw) &&
        rdq_finite(controller->max_voltage_v) &&
        rdq_finite(controller->integrator_limit_v) &&
        input->dt_s > 0.0f &&
        input->vbus_v > 1.0f &&
        controller->max_voltage_v > 0.0f &&
        controller->integrator_limit_v > 0.0f;

    if (!input->enable || input->fault_active || !input_ok) {
        current_controller_reset(controller);
        return;
    }

    const float id_error = input->id_ref_a - input->id_measured_a;
    const float iq_error = input->iq_ref_a - input->iq_measured_a;
    const float vbus_limit = 0.57735026919f * input->vbus_v;
    const float limit_v = (controller->max_voltage_v < vbus_limit) ?
                          controller->max_voltage_v :
                          vbus_limit;

    if (freeze_integrator) {
        controller->integrator_d_v = 0.0f;
        controller->integrator_q_v = 0.0f;
    }

    const float vd_unsat =
        (controller->kp * id_error) + controller->integrator_d_v;
    const float vq_unsat =
        (controller->kp * iq_error) + controller->integrator_q_v +
        vq_feedforward_v;
    float vd_sat = vd_unsat;
    float vq_sat = vq_unsat;
    foc_limit_voltage(&vd_sat, &vq_sat, limit_v);

    if (!freeze_integrator && !defer_integrator_update) {
        controller->integrator_d_v +=
            ((controller->ki * id_error) +
             (controller->kaw * (vd_sat - vd_unsat))) * input->dt_s;
        controller->integrator_q_v +=
            ((controller->ki * iq_error) +
             (controller->kaw * (vq_sat - vq_unsat))) * input->dt_s;
        controller->integrator_d_v = foc_clamp(controller->integrator_d_v,
                                               -controller->integrator_limit_v,
                                               controller->integrator_limit_v);
        controller->integrator_q_v = foc_clamp(controller->integrator_q_v,
                                               -controller->integrator_limit_v,
                                               controller->integrator_limit_v);
    }

    output->vd_unsat_v = vd_unsat;
    output->vq_unsat_v = vq_unsat;
    output->vd_v = vd_sat;
    output->vq_v = vq_sat;
    output->v_alpha_v = (cos_theta * vd_sat) - (sin_theta * vq_sat);
    output->v_beta_v = (sin_theta * vd_sat) + (cos_theta * vq_sat);
    output->integrator_d_v = controller->integrator_d_v;
    output->integrator_q_v = controller->integrator_q_v;
    output->saturation_active =
        (fabsf(vd_sat - vd_unsat) > 1.0e-6f) ||
        (fabsf(vq_sat - vq_unsat) > 1.0e-6f);
    output->valid =
        rdq_finite(output->vd_unsat_v) &&
        rdq_finite(output->vq_unsat_v) &&
        rdq_finite(output->vd_v) &&
        rdq_finite(output->vq_v) &&
        rdq_finite(output->v_alpha_v) &&
        rdq_finite(output->v_beta_v) &&
        rdq_finite(output->integrator_d_v) &&
        rdq_finite(output->integrator_q_v);
    if (!output->valid) {
        current_controller_reset(controller);
        rdq_zero_controller_output(output);
    }
}

float rotating_dq_current_test_theta_from_count(
    const RotatingDqCurrentTestConfig *config,
    int64_t encoder_count,
    float electrical_offset_rad)
{
    const RotatingDqCurrentTestConfig def = rotating_dq_current_test_default_config();
    const RotatingDqCurrentTestConfig *cfg = (config != 0) ? config : &def;
    if (cfg->encoder_cpr <= 0 ||
        cfg->pole_pairs <= 0 ||
        cfg->encoder_direction == 0 ||
        !rdq_finite(electrical_offset_rad)) {
        return NAN;
    }
    const float theta_m =
        ROTATING_DQ_TWO_PI_F * (float)encoder_count / (float)cfg->encoder_cpr;
    return rdq_wrap_0_2pi(((float)cfg->encoder_direction *
                           (float)cfg->pole_pairs * theta_m) +
                          electrical_offset_rad);
}

int16_t rotating_dq_current_test_encoder_delta_u16(uint16_t now, uint16_t last)
{
    return (int16_t)(now - last);
}

static bool rdq_inputs_finite(const RotatingDqCurrentTestInput *in)
{
    return (in != 0) &&
           rdq_finite(in->theta_e_rad) &&
           rdq_finite(in->iv_a) &&
           rdq_finite(in->iw_a) &&
           rdq_finite(in->vbus_v) &&
           (!in->external_iq_ref_valid || rdq_finite(in->external_iq_ref_a));
}

static void rdq_advance(RotatingDqCurrentTest *test,
                        RotatingDqCurrentTestState next)
{
    test->state = next;
    test->state_ticks = 0u;
    if (next == ROTATING_DQ_STATE_COMPLETE) {
        rdq_stats_finalize(&test->zero_stats);
        rdq_stats_finalize(&test->positive_stats);
        rdq_stats_finalize(&test->negative_stats);
        if (test->config.enable_zero_diagnostic_only) {
            test->result = ROTATING_DQ_RESULT_PASS;
            test->iq_ref_a = 0.0f;
            current_controller_reset(&test->controller);
            return;
        }
        if (test->config.freeze_zero_reference_integrator) {
            test->result = ROTATING_DQ_RESULT_PASS;
            test->iq_ref_a = 0.0f;
            current_controller_reset(&test->controller);
            return;
        }
        const bool external_supervisor_owns_tracking =
            test->config.enable_external_iq_block_integrator &&
            test->config.single_direction_positive_only &&
            test->config.direction_capture_counts >= 1000000;
        if (external_supervisor_owns_tracking) {
            test->result = ROTATING_DQ_RESULT_PASS;
            test->iq_ref_a = 0.0f;
            current_controller_reset(&test->controller);
            return;
        }
        const bool directions_ok =
            test->positive_stats.sample_count > 0u &&
            test->negative_stats.sample_count > 0u &&
            test->positive_stats.mechanical_direction != 0 &&
            test->negative_stats.mechanical_direction != 0 &&
            test->positive_stats.mechanical_direction !=
                test->negative_stats.mechanical_direction;
        const bool tracking_ok = rdq_tracking_stats_ok(test);
        if (test->config.require_direction_match && !directions_ok) {
            rdq_fail(test, ROTATING_DQ_FAULT_DIRECTION);
        } else if (!test->config.require_direction_match && !tracking_ok) {
            rdq_fail(test, ROTATING_DQ_FAULT_TRACKING);
        } else {
            test->result = ROTATING_DQ_RESULT_PASS;
            test->iq_ref_a = 0.0f;
            current_controller_reset(&test->controller);
        }
    }
}

static void rdq_log(RotatingDqCurrentTest *test,
                    const RotatingDqCurrentLogRecord *rec)
{
    if ((test->config.log_decimation == 0u) ||
        ((test->control_tick_seq % test->config.log_decimation) != 0u)) {
        return;
    }
    if (test->log_count < ROTATING_DQ_CURRENT_TEST_LOG_CAPACITY) {
        test->log[test->log_count++] = *rec;
    } else {
        test->log_dropped++;
    }
}

static void rdq_fill_output_compact(const RotatingDqCurrentTest *test,
                                    const CurrentControllerOutput *cc_out,
                                    RotatingDqCurrentTestOutput *output)
{
    output->state = test->state;
    output->result = test->result;
    output->fault_code = test->fault_code;
    output->id_ref_a = 0.0f;
    output->iq_ref_a = test->iq_ref_a;
    output->vd_v = (test->result == ROTATING_DQ_RESULT_FAIL) ? 0.0f : cc_out->vd_v;
    output->vq_v = (test->result == ROTATING_DQ_RESULT_FAIL) ? 0.0f : cc_out->vq_v;
    output->v_alpha_v =
        (test->result == ROTATING_DQ_RESULT_FAIL) ? 0.0f : cc_out->v_alpha_v;
    output->v_beta_v =
        (test->result == ROTATING_DQ_RESULT_FAIL) ? 0.0f : cc_out->v_beta_v;
    output->common_mode_shape = false;
    output->common_mode_harmful = false;
    output->common_mode_caused_dq_crossing = false;
    output->power_stage_request =
        (test->state != ROTATING_DQ_STATE_PREFLIGHT) &&
        (test->state != ROTATING_DQ_STATE_COMPLETE) &&
        (test->state != ROTATING_DQ_STATE_FAIL);
    output->pwm_output_request = output->power_stage_request;
    output->safe_shutdown_request =
        (test->state == ROTATING_DQ_STATE_COMPLETE) ||
        (test->state == ROTATING_DQ_STATE_FAIL);
    output->done = output->safe_shutdown_request;
}

static void rdq_fill_output(const RotatingDqCurrentTest *test,
                            const CurrentControllerOutput *cc_out,
                            RotatingDqCurrentTestOutput *output)
{
    rdq_fill_output_compact(test, cc_out, output);
    output->vd_diagnostic_v = 0.0f;
    output->vq_diagnostic_v = 0.0f;
    output->vd_proportional_v = 0.0f;
    output->vq_proportional_v = 0.0f;
    output->vd_integrator_v = 0.0f;
    output->vq_integrator_v = 0.0f;
    output->vd_feedforward_v = 0.0f;
    output->vq_feedforward_v = 0.0f;
    output->vd_unclamped_v = 0.0f;
    output->vq_unclamped_v = 0.0f;
    output->iu_measured_a = 0.0f;
    output->iv_measured_a = 0.0f;
    output->iw_measured_a = 0.0f;
    output->id_measured_a = 0.0f;
    output->iq_measured_a = 0.0f;
    output->id_control_a = 0.0f;
    output->iq_control_a = 0.0f;
    output->id_error_a = 0.0f;
    output->iq_error_a = 0.0f;
    output->integrator_d_before_v = 0.0f;
    output->integrator_q_before_v = 0.0f;
    output->integrator_d_after_v = 0.0f;
    output->integrator_q_after_v = 0.0f;
    output->integrator_d_delta_v = 0.0f;
    output->integrator_q_delta_v = 0.0f;
    output->integrator_d_aw_clamp_delta_v = 0.0f;
    output->integrator_q_aw_clamp_delta_v = 0.0f;
}

void rotating_dq_current_test_fast_isr(RotatingDqCurrentTest *test,
                                       const RotatingDqCurrentTestInput *input,
                                       RotatingDqCurrentTestOutput *output)
    ROTATING_DQ_FAST_OPT;
void rotating_dq_current_test_fast_isr(RotatingDqCurrentTest *test,
                                       const RotatingDqCurrentTestInput *input,
                                       RotatingDqCurrentTestOutput *output)
{
    CurrentControllerOutput cc_out = {0};
    if (test == 0 || input == 0 || output == 0) {
        return;
    }

    if (test->result == ROTATING_DQ_RESULT_PASS ||
        test->result == ROTATING_DQ_RESULT_FAIL) {
        rdq_fill_output(test, &cc_out, output);
        return;
    }

    const uint32_t profile_total_start = rdq_total_profile_now();
    uint32_t profile_section_start = profile_total_start;
    uint32_t profile_log_stats_cycles = 0u;

    if (test->have_last_adc_seq) {
        if (input->adc_seq == test->last_adc_seq) {
            test->duplicate_control_tick_count++;
            rdq_fail(test, ROTATING_DQ_FAULT_ADC_DUPLICATE);
        } else if (input->adc_seq != (test->last_adc_seq + 1u)) {
            test->missed_control_tick_count++;
            rdq_fail(test, ROTATING_DQ_FAULT_ADC_SEQ_GAP);
        }
    }
    test->last_adc_seq = input->adc_seq;
    test->have_last_adc_seq = true;

    if (test->result != ROTATING_DQ_RESULT_FAIL) {
        test->control_tick_seq++;
    }

    if (!rdq_inputs_finite(input)) {
        rdq_fail(test, ROTATING_DQ_FAULT_NAN_INF);
    }
    if (!input->electrical_offset_valid || !input->theta_valid) {
        rdq_fail(test, ROTATING_DQ_FAULT_ELECTRICAL_OFFSET_INVALID);
    }
    if (!input->adc_valid) {
        rdq_fail(test, ROTATING_DQ_FAULT_ADC_SEQ_GAP);
    }
    if (!input->encoder_valid) {
        rdq_fail(test, ROTATING_DQ_FAULT_ENCODER);
    }
    if (!input->nfault_ok) {
        rdq_fail(test, ROTATING_DQ_FAULT_NFAULT);
    }
    if (!input->drv_ok || input->fault_active) {
        rdq_fail(test, ROTATING_DQ_FAULT_DRV);
    }
    if (!input->m1_safe) {
        rdq_fail(test, ROTATING_DQ_FAULT_M1);
    }
    if (!input->pwm_ccr_ok) {
        rdq_fail(test, ROTATING_DQ_FAULT_PWM_CCR);
    }
    if ((input->vbus_v < test->config.vbus_min_v) ||
        (input->vbus_v > test->config.vbus_max_v)) {
        rdq_fail(test, ROTATING_DQ_FAULT_VBUS);
    }
    uint32_t profile_now = rdq_profile_now();
    rdq_profile_update(&test->fast_profile.seq_input_max_cycles,
                       profile_now - profile_section_start);
    profile_section_start = profile_now;

    int64_t encoder_delta = 0;
    if (!test->have_last_encoder_count) {
        test->encoder_start_count = input->encoder_count;
        test->encoder_last_count = input->encoder_count;
        test->have_last_encoder_count = true;
    } else {
        encoder_delta = input->encoder_count - test->encoder_last_count;
        test->encoder_last_count = input->encoder_count;
    }
    const int64_t motion = rdq_abs_i64(input->encoder_count - test->encoder_start_count);
    if (motion > test->encoder_motion_max_counts) {
        test->encoder_motion_max_counts = motion;
    }
    if (motion > test->config.one_rev_limit_counts) {
        rdq_fail(test, ROTATING_DQ_FAULT_ONE_REV);
    }
    if (test->have_last_theta &&
        rdq_angle_diff_abs(input->theta_e_rad, test->theta_last_rad) >
            test->config.angle_jump_limit_rad) {
        rdq_fail(test, ROTATING_DQ_FAULT_ANGLE_JUMP);
    }
    test->theta_last_rad = input->theta_e_rad;
    test->have_last_theta = true;

    const float rpm_raw =
        (test->config.encoder_cpr > 0)
            ? ((float)encoder_delta * 60.0f /
               ((float)test->config.encoder_cpr * test->config.dt_s))
            : 0.0f;
    test->mechanical_speed_rpm =
        (0.90f * test->mechanical_speed_rpm) + (0.10f * rpm_raw);
    const float mechanical_speed_abs = rdq_absf(test->mechanical_speed_rpm);
    if (mechanical_speed_abs > test->speed_peak_rpm) {
        test->speed_peak_rpm = mechanical_speed_abs;
    }
    if (mechanical_speed_abs > test->config.speed_limit_rpm) {
        rdq_fail(test, ROTATING_DQ_FAULT_OVERSPEED);
    }
    profile_now = rdq_profile_now();
    rdq_profile_update(&test->fast_profile.encoder_speed_max_cycles,
                       profile_now - profile_section_start);
    profile_section_start = profile_now;

    float iu = 0.0f;
    float i_alpha = 0.0f;
    float i_beta = 0.0f;
    float id = 0.0f;
    float iq = 0.0f;
    float sin_theta = 0.0f;
    float cos_theta = 1.0f;
    rdq_fast_sincos(input->theta_e_rad, &sin_theta, &cos_theta);
    iu = -(input->iv_a + input->iw_a);
    i_alpha = iu;
    i_beta = (input->iv_a - input->iw_a) * 0.57735026919f;
    id = (cos_theta * i_alpha) + (sin_theta * i_beta);
    iq = (-sin_theta * i_alpha) + (cos_theta * i_beta);
    profile_now = rdq_profile_now();
    rdq_profile_update(&test->fast_profile.clarke_park_max_cycles,
                       profile_now - profile_section_start);
    profile_section_start = profile_now;

    const uint32_t phase_trip_channel =
        rdq_phase_trip_channel(iu, input->iv_a, input->iw_a,
                               test->config.phase_current_limit_a);
    const uint32_t dq_trip_channel =
        rdq_dq_trip_channel(id, iq, test->config.dq_current_limit_a);
    const float iv_abs_for_metric = rdq_absf(input->iv_a);
    const float iw_abs_for_metric = rdq_absf(input->iw_a);
    const float measured_phase_metric =
        (iv_abs_for_metric > iw_abs_for_metric) ?
            iv_abs_for_metric : iw_abs_for_metric;
    float reconstructed_phase_metric = rdq_absf(iu);
    if (measured_phase_metric > reconstructed_phase_metric) {
        reconstructed_phase_metric = measured_phase_metric;
    }
    const int32_t du_counts =
        (int32_t)input->raw_pc0 - (int32_t)input->offset_pc0;
    const int32_t dv_counts =
        (int32_t)input->raw_pc1 - (int32_t)input->offset_pc1;
    const int32_t abs_du_counts =
        (du_counts < 0) ? -du_counts : du_counts;
    const int32_t abs_dv_counts =
        (dv_counts < 0) ? -dv_counts : dv_counts;
    const int32_t cm_counts = du_counts + dv_counts;
    const int32_t diff_counts = du_counts - dv_counts;
    const uint32_t cm_abs_counts =
        (cm_counts < 0) ? (uint32_t)(-cm_counts) : (uint32_t)cm_counts;
    const uint32_t diff_abs_counts =
        (diff_counts < 0) ? (uint32_t)(-diff_counts) : (uint32_t)diff_counts;
    const bool same_sign_counts =
        (du_counts > 0 && dv_counts > 0) ||
        (du_counts < 0 && dv_counts < 0);
    const float measured_phase_limit_with_one_count =
        test->config.phase_current_limit_a +
        ((input->current_amp_per_count > 0.0f) ?
             input->current_amp_per_count : 0.0f);
    const bool near_zero_current_command =
        rdq_absf(test->iq_ref_a) <= ROTATING_DQ_NEAR_ZERO_IQ_REF_A;
    const bool common_mode_counts_like =
        rdq_state_is_power_test_active(test->state) &&
        same_sign_counts &&
        abs_du_counts <= ROTATING_DQ_COMMON_MODE_ABS_COUNTS &&
        abs_dv_counts <= ROTATING_DQ_COMMON_MODE_ABS_COUNTS &&
        diff_abs_counts <= ROTATING_DQ_COMMON_MODE_DIFF_COUNTS &&
        measured_phase_metric <= measured_phase_limit_with_one_count;
    const bool zero_diag_window =
        test->config.enable_zero_diagnostic_only &&
        test->state == ROTATING_DQ_STATE_ENABLE_ZERO;
    const uint32_t enable_age_ticks =
        (test->state == ROTATING_DQ_STATE_ENABLE_ZERO) ?
            (test->state_ticks + 1u) : 0u;
    const bool zero_diag_startup_guard = zero_diag_window &&
        enable_age_ticks <= test->config.zero_startup_guard_ticks;
    const bool zero_diag_steady_window = zero_diag_window &&
        !zero_diag_startup_guard;
    const float raw_dq_metric =
        (rdq_absf(id) > rdq_absf(iq)) ? rdq_absf(id) : rdq_absf(iq);
    float id_cf = id;
    float iq_cf = iq;
    float counterfactual_phase_metric = reconstructed_phase_metric;
    float counterfactual_dq_metric = raw_dq_metric;
    bool phase_crossing_caused_by_cm = false;
    bool dq_crossing_caused_by_cm = false;
    if (zero_diag_window && common_mode_counts_like) {
        /* PC0/PC1 enter this interface as physical V/W currents. */
        const float cm_a = 0.5f * (input->iv_a + input->iw_a);
        const float iv_cf = input->iv_a - cm_a;
        const float iw_cf = input->iw_a - cm_a;
        const float iu_cf = -(iv_cf + iw_cf);
        const float i_alpha_cf = iu_cf;
        const float i_beta_cf = (iv_cf - iw_cf) * 0.57735026919f;
        id_cf = (cos_theta * i_alpha_cf) + (sin_theta * i_beta_cf);
        iq_cf = (-sin_theta * i_alpha_cf) + (cos_theta * i_beta_cf);
        const float iv_cf_abs = rdq_absf(iv_cf);
        const float iw_cf_abs = rdq_absf(iw_cf);
        const float iu_cf_abs = rdq_absf(iu_cf);
        const float count_guard_a =
            (input->current_amp_per_count > 0.0f) ?
                input->current_amp_per_count : 0.0f;

        counterfactual_phase_metric = iv_cf_abs;
        if (iw_cf_abs > counterfactual_phase_metric) {
            counterfactual_phase_metric = iw_cf_abs;
        }
        if (iu_cf_abs > counterfactual_phase_metric) {
            counterfactual_phase_metric = iu_cf_abs;
        }
        counterfactual_dq_metric =
            (rdq_absf(id_cf) > rdq_absf(iq_cf)) ?
                rdq_absf(id_cf) : rdq_absf(iq_cf);
        phase_crossing_caused_by_cm =
            reconstructed_phase_metric > test->config.phase_current_limit_a &&
            counterfactual_phase_metric <= test->config.phase_current_limit_a &&
            (reconstructed_phase_metric - counterfactual_phase_metric) >=
                count_guard_a;
        dq_crossing_caused_by_cm =
            raw_dq_metric > test->config.dq_current_limit_a &&
            counterfactual_dq_metric <= test->config.dq_current_limit_a &&
            (raw_dq_metric - counterfactual_dq_metric) >= count_guard_a;
    }
    const bool common_mode_harmful = common_mode_counts_like &&
        (phase_crossing_caused_by_cm || dq_crossing_caused_by_cm);
    const bool zero_diag_use_counterfactual_measurement =
        zero_diag_window && common_mode_harmful;
    const bool near_zero_common_mode_like =
        common_mode_counts_like && near_zero_current_command;
    const bool zero_return_common_mode_like =
        common_mode_counts_like && rdq_state_is_zero_return(test->state);
    const bool low_current_tracking_common_mode_like =
        common_mode_counts_like &&
        !test->config.enable_zero_diagnostic_only &&
        !test->config.require_direction_match;
    const bool soft_current_common_mode_like =
        near_zero_common_mode_like ||
        zero_return_common_mode_like ||
        low_current_tracking_common_mode_like;
    const bool suppress_common_mode_for_runtime =
        soft_current_common_mode_like && !zero_diag_window;
    const float id_control = suppress_common_mode_for_runtime ? 0.0f :
                             zero_diag_use_counterfactual_measurement ? id_cf : id;
    const float iq_control = suppress_common_mode_for_runtime ? 0.0f :
                             zero_diag_use_counterfactual_measurement ? iq_cf : iq;
    if (test->state == ROTATING_DQ_STATE_ENABLE_ZERO) {
        if (zero_diag_startup_guard) {
            if (measured_phase_metric > test->zero_startup_direct_metric_max_a) {
                test->zero_startup_direct_metric_max_a = measured_phase_metric;
            }
            if (reconstructed_phase_metric >
                test->zero_startup_reconstructed_metric_max_a) {
                test->zero_startup_reconstructed_metric_max_a =
                    reconstructed_phase_metric;
            }
            if (raw_dq_metric > test->zero_startup_dq_metric_max_a) {
                test->zero_startup_dq_metric_max_a = raw_dq_metric;
            }
            if (measured_phase_metric >
                test->config.zero_phase_current_peak_limit_a) {
                test->zero_startup_direct_outlier_count++;
                if (test->zero_startup_first_outlier_tick == 0u) {
                    test->zero_startup_first_outlier_tick = enable_age_ticks;
                }
                test->zero_startup_last_outlier_tick = enable_age_ticks;
                const int32_t abs_startup_pc0 =
                    (test->zero_startup_pc0_peak_delta_counts < 0) ?
                        -test->zero_startup_pc0_peak_delta_counts :
                        test->zero_startup_pc0_peak_delta_counts;
                const int32_t abs_startup_pc1 =
                    (test->zero_startup_pc1_peak_delta_counts < 0) ?
                        -test->zero_startup_pc1_peak_delta_counts :
                        test->zero_startup_pc1_peak_delta_counts;
                if (abs_du_counts > abs_startup_pc0) {
                    test->zero_startup_pc0_peak_delta_counts = du_counts;
                }
                if (abs_dv_counts > abs_startup_pc1) {
                    test->zero_startup_pc1_peak_delta_counts = dv_counts;
                }
            }
        }
        if (!zero_diag_window ||
            (zero_diag_steady_window && !test->zero_diag_completed)) {
        if (measured_phase_metric > test->zero_measured_phase_metric_max_a) {
            test->zero_measured_phase_metric_max_a = measured_phase_metric;
            test->zero_direct_metric_peak_tick = test->control_tick_seq;
        }
        if (reconstructed_phase_metric >
            test->zero_reconstructed_phase_metric_max_a) {
            test->zero_reconstructed_phase_metric_max_a =
                reconstructed_phase_metric;
            test->zero_reconstructed_metric_peak_tick = test->control_tick_seq;
        }
        if (raw_dq_metric > test->zero_raw_dq_metric_max_a) {
            test->zero_raw_dq_metric_max_a = raw_dq_metric;
        }
        if (counterfactual_phase_metric >
            test->zero_counterfactual_phase_metric_max_a) {
            test->zero_counterfactual_phase_metric_max_a =
                counterfactual_phase_metric;
        }
        if (counterfactual_dq_metric >
            test->zero_counterfactual_dq_metric_max_a) {
            test->zero_counterfactual_dq_metric_max_a =
                counterfactual_dq_metric;
        }
        if (common_mode_counts_like) {
            test->zero_common_mode_shift_count++;
            if (cm_abs_counts > test->zero_common_mode_max_counts) {
                test->zero_common_mode_max_counts = cm_abs_counts;
            }
            if (diff_abs_counts > test->zero_common_mode_diff_max_counts) {
                test->zero_common_mode_diff_max_counts = diff_abs_counts;
            }
        }
        if (common_mode_harmful) {
            test->zero_common_mode_excluded_count++;
            if (phase_crossing_caused_by_cm) {
                test->zero_common_mode_phase_exclusion_count++;
            }
            if (dq_crossing_caused_by_cm) {
                test->zero_common_mode_dq_exclusion_count++;
            }
        } else {
            if (common_mode_counts_like) {
                test->zero_common_mode_harmless_count++;
            }
            test->zero_clean_sample_count++;
            if (reconstructed_phase_metric >
                test->zero_clean_phase_metric_max_a) {
                test->zero_clean_phase_metric_max_a =
                    reconstructed_phase_metric;
                test->zero_clean_metric_peak_tick = test->control_tick_seq;
            }
        }
        }
    }
    const uint32_t effective_phase_trip_channel =
        (suppress_common_mode_for_runtime || phase_crossing_caused_by_cm) ?
            0u : phase_trip_channel;
    const uint32_t effective_dq_trip_channel =
        (suppress_common_mode_for_runtime || dq_crossing_caused_by_cm) ?
            0u : dq_trip_channel;
    uint32_t current_source_mask = 0u;
    if (phase_trip_channel != 0u) {
        current_source_mask |= phase_trip_channel |
                               ROTATING_DQ_ZERO_TRIP_PHASE_METRIC;
    }
    if (dq_trip_channel != 0u) {
        current_source_mask |= dq_trip_channel |
                               ROTATING_DQ_ZERO_TRIP_DQ_METRIC;
    }
#if ROTATING_DQ_ENABLE_ZERO_DIAG_LEVEL >= 2u
    bool current_diag_stored = false;
#endif
    bool startup_soft_current_observed = false;
    const bool enable_zero_soft_consecutive =
        rdq_state_is_power_test_active(test->state);
    if (enable_zero_soft_consecutive) {
        if (effective_phase_trip_channel != 0u) {
            test->zero_phase_over_limit_consecutive++;
            if (test->zero_phase_over_limit_consecutive >
                test->zero_phase_over_limit_consecutive_max) {
                test->zero_phase_over_limit_consecutive_max =
                    test->zero_phase_over_limit_consecutive;
            }
        } else if (!phase_crossing_caused_by_cm) {
            test->zero_phase_over_limit_consecutive = 0u;
        }
        if (effective_dq_trip_channel != 0u) {
            test->zero_dq_over_limit_consecutive++;
            if (test->zero_dq_over_limit_consecutive >
                test->zero_dq_over_limit_consecutive_max) {
                test->zero_dq_over_limit_consecutive_max =
                    test->zero_dq_over_limit_consecutive;
            }
        } else if (!dq_crossing_caused_by_cm) {
            test->zero_dq_over_limit_consecutive = 0u;
        }
    } else {
        test->zero_phase_over_limit_consecutive = 0u;
        test->zero_dq_over_limit_consecutive = 0u;
    }
    const bool detailed_current_trip_diag_enabled =
        !(test->config.direction_capture_counts >= 1000000 &&
          test->config.enable_external_iq_block_integrator);
    if (current_source_mask != 0u) {
        const bool observe_startup_soft_current =
            test->config.enable_zero_diagnostic_only &&
            test->state == ROTATING_DQ_STATE_ENABLE_ZERO &&
            test->control_tick_seq <= ROTATING_DQ_ENABLE_ZERO_STARTUP_OBSERVE_TICKS;
        startup_soft_current_observed = observe_startup_soft_current;
        if (detailed_current_trip_diag_enabled) {
            rdq_zero_diag_store_minimal_current_sample(
                test,
                input,
                ROTATING_DQ_ZERO_STAGE_ENABLE_ZERO_MOE_ON,
                current_source_mask,
                (uint32_t)__LINE__,
                iu,
                i_alpha,
                i_beta,
                id,
                iq,
                sin_theta,
                cos_theta,
                phase_trip_channel,
                dq_trip_channel);
        }
#if ROTATING_DQ_ENABLE_ZERO_DIAG_LEVEL >= 2u
        current_diag_stored = detailed_current_trip_diag_enabled;
#endif
        if (effective_phase_trip_channel != 0u) {
            if (observe_startup_soft_current) {
                test->zero_phase_startup_over_limit_count++;
                test->zero_phase_startup_over_limit_last_tick =
                    test->control_tick_seq;
                if (reconstructed_phase_metric >
                    test->zero_phase_startup_over_limit_max_a) {
                    test->zero_phase_startup_over_limit_max_a =
                        reconstructed_phase_metric;
                }
            } else if (!enable_zero_soft_consecutive ||
                       test->zero_phase_over_limit_consecutive >=
                           ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS) {
                test->zero_phase_fault_set_tick = test->control_tick_seq;
                test->zero_fault_bit_order = 1u;
                rdq_fail(test, ROTATING_DQ_FAULT_PHASE_CURRENT_LIMIT);
            }
        }
        if (effective_dq_trip_channel != 0u) {
            if (observe_startup_soft_current) {
                const float id_abs = rdq_absf(id);
                const float iq_abs = rdq_absf(iq);
                const float dq_metric = (id_abs > iq_abs) ? id_abs : iq_abs;
                test->zero_dq_startup_over_limit_count++;
                test->zero_dq_startup_over_limit_last_tick = test->control_tick_seq;
                if (dq_metric > test->zero_dq_startup_over_limit_max_a) {
                    test->zero_dq_startup_over_limit_max_a = dq_metric;
                }
            } else if (!enable_zero_soft_consecutive ||
                       test->zero_dq_over_limit_consecutive >=
                           ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS) {
                test->zero_dq_fault_set_tick = test->control_tick_seq;
                if (test->zero_fault_bit_order == 0u) {
                    test->zero_fault_bit_order = 2u;
                }
                rdq_fail(test, ROTATING_DQ_FAULT_DQ_CURRENT_LIMIT);
            }
        }
        if (test->result == ROTATING_DQ_RESULT_FAIL) {
            profile_now = rdq_profile_now();
            rdq_profile_update(&test->fast_profile.current_protection_max_cycles,
                               profile_now - profile_section_start);
            rdq_profile_update(&test->fast_profile.total_max_cycles,
                               profile_now - profile_total_start);
            rdq_fill_output(test, &cc_out, output);
            return;
        }
    }
    profile_now = rdq_profile_now();
    rdq_profile_update(&test->fast_profile.current_protection_max_cycles,
                       profile_now - profile_section_start);
    profile_section_start = profile_now;

    if (test->state == ROTATING_DQ_STATE_ENABLE_ZERO) {
        test->iq_ref_a = 0.0f;
    } else if (test->state == ROTATING_DQ_STATE_RAMP_IQ_POSITIVE) {
        if (input->external_iq_ref_valid) {
            test->iq_ref_a = input->external_iq_ref_a;
            rdq_advance(test, ROTATING_DQ_STATE_HOLD_IQ_POSITIVE);
        } else {
            test->iq_ref_a = current_controller_ramp_toward(test->iq_ref_a,
                                                            test->config.iq_target_a,
                                                            test->config.iq_ramp_rate_a_per_s,
                                                            test->config.dt_s);
            if (rdq_abs_i64(test->positive_stats.encoder_delta_counts) >=
                    test->config.direction_capture_counts) {
                rdq_advance(test, ROTATING_DQ_STATE_RAMP_ZERO_1);
            } else if (test->iq_ref_a >= test->config.iq_target_a) {
                rdq_advance(test, ROTATING_DQ_STATE_HOLD_IQ_POSITIVE);
            }
        }
    } else if (test->state == ROTATING_DQ_STATE_HOLD_IQ_POSITIVE &&
               input->external_iq_ref_valid) {
        test->iq_ref_a = input->external_iq_ref_a;
    } else if (test->state == ROTATING_DQ_STATE_RAMP_ZERO_1 ||
               test->state == ROTATING_DQ_STATE_RAMP_ZERO_2) {
        test->iq_ref_a = current_controller_ramp_toward(test->iq_ref_a,
                                                        0.0f,
                                                        test->config.iq_ramp_rate_a_per_s,
                                                        test->config.dt_s);
        if (rdq_absf(test->iq_ref_a) <= 0.000001f) {
            rdq_advance(test,
                        (test->state == ROTATING_DQ_STATE_RAMP_ZERO_1)
                            ? ROTATING_DQ_STATE_HOLD_ZERO_1
                            : ROTATING_DQ_STATE_HOLD_ZERO_2);
        }
    } else if (test->state == ROTATING_DQ_STATE_RAMP_IQ_NEGATIVE) {
        test->iq_ref_a = current_controller_ramp_toward(test->iq_ref_a,
                                                        -test->config.iq_target_a,
                                                        test->config.iq_ramp_rate_a_per_s,
                                                        test->config.dt_s);
        if (rdq_abs_i64(test->negative_stats.encoder_delta_counts) >=
                test->config.direction_capture_counts) {
            rdq_advance(test, ROTATING_DQ_STATE_RAMP_ZERO_2);
        } else if (test->iq_ref_a <= -test->config.iq_target_a) {
            rdq_advance(test, ROTATING_DQ_STATE_HOLD_IQ_NEGATIVE);
        }
    }
    const bool external_block_integrator =
        test->config.enable_external_iq_block_integrator &&
        !test->config.enable_zero_diagnostic_only &&
        rdq_state_is_power_test_active(test->state);
    if (external_block_integrator) {
        test->iq_ref_a = input->external_iq_ref_valid ?
            input->external_iq_ref_a : 0.0f;
    }
    if (rdq_absf(test->iq_ref_a) > test->config.iq_ref_hard_limit_a) {
        rdq_fail(test, ROTATING_DQ_FAULT_IQ_REF_LIMIT);
    }

    const bool controller_enable =
        (test->result == ROTATING_DQ_RESULT_RUNNING) &&
        input->pwm_allowed &&
        (test->state != ROTATING_DQ_STATE_PREFLIGHT) &&
        (test->state != ROTATING_DQ_STATE_COMPLETE) &&
        (test->state != ROTATING_DQ_STATE_FAIL);
    const bool zero_output_state =
        rdq_state_is_zero_output(test->state) &&
        !(test->state == ROTATING_DQ_STATE_ENABLE_ZERO &&
          test->config.enable_zero_current_pi);
    const bool freeze_zero_reference_integrator =
        test->config.freeze_zero_reference_integrator &&
        ((test->config.enable_zero_diagnostic_only &&
          test->state == ROTATING_DQ_STATE_ENABLE_ZERO &&
          rdq_absf(test->iq_ref_a) <= ROTATING_DQ_NEAR_ZERO_IQ_REF_A) ||
         (!test->config.enable_zero_diagnostic_only &&
          rdq_state_is_power_test_active(test->state)));
    const bool block_zero_reference_integrator =
        test->config.enable_zero_block_integrator &&
        test->config.enable_zero_diagnostic_only &&
        test->state == ROTATING_DQ_STATE_ENABLE_ZERO &&
        rdq_absf(test->iq_ref_a) <= ROTATING_DQ_NEAR_ZERO_IQ_REF_A;
    const bool bipolar_block_integrator =
        test->config.enable_zero_block_integrator &&
        !test->config.enable_external_iq_block_integrator &&
        !test->config.enable_zero_diagnostic_only &&
        rdq_state_is_power_test_active(test->state);
    const bool bipolar_block_collect =
        bipolar_block_integrator &&
        (test->state == ROTATING_DQ_STATE_HOLD_IQ_POSITIVE ||
         test->state == ROTATING_DQ_STATE_HOLD_IQ_NEGATIVE) &&
        rdq_absf(test->iq_ref_a) >= (0.95f * test->config.iq_target_a);
    const bool external_reference_initialized =
        g_rdq_external_block_ref_valid;
    const float external_reference_previous_a =
        g_rdq_external_block_ref_a;
    const bool external_reference_reversed =
        external_reference_initialized &&
        (test->iq_ref_a * external_reference_previous_a) < 0.0f;
    const bool external_reference_large_step =
        external_reference_initialized &&
        rdq_absf(test->iq_ref_a - external_reference_previous_a) >
            ROTATING_DQ_EXTERNAL_BLOCK_REF_RESET_DELTA_A;
    const bool external_reference_reset_required =
        external_block_integrator &&
        (!external_reference_initialized ||
         external_reference_reversed ||
         external_reference_large_step);
    if (external_reference_reset_required) {
        if (!g_rdq_hold_snapshot_pending) {
            rotating_dq_block_integrator_reset(&g_rdq_block_integrator);
        }
        test->controller.integrator_d_v = 0.0f;
        test->controller.integrator_q_v = 0.0f;
    }
    if (external_block_integrator) {
        g_rdq_external_block_ref_valid = true;
        g_rdq_external_block_ref_a = test->iq_ref_a;
    }
    const float external_adc_ref_floor_a =
        ROTATING_DQ_EXTERNAL_BLOCK_MIN_ADC_COUNTS *
        input->current_amp_per_count;
    const float external_min_integrating_ref_a =
        (test->config.tracking_iq_ref_mean_min_a > external_adc_ref_floor_a)
            ? test->config.tracking_iq_ref_mean_min_a
            : external_adc_ref_floor_a;
    const float external_exit_ref_a =
        ROTATING_DQ_EXTERNAL_BLOCK_EXIT_ADC_COUNTS *
        input->current_amp_per_count;
    if (!external_block_integrator || !input->external_iq_ref_valid ||
        !input->external_integrator_enable || external_reference_reversed) {
        g_rdq_external_block_collect_latched = false;
    } else if (rdq_absf(test->iq_ref_a) >= external_min_integrating_ref_a) {
        g_rdq_external_block_collect_latched = true;
    } else if (rdq_absf(test->iq_ref_a) <= external_exit_ref_a) {
        g_rdq_external_block_collect_latched = false;
    }
    const bool external_block_collect =
        external_block_integrator && input->external_iq_ref_valid &&
        input->external_integrator_enable &&
        g_rdq_external_block_collect_latched;
    const bool block_integrator_defer =
        block_zero_reference_integrator || bipolar_block_integrator ||
        external_block_integrator;
    if (bipolar_block_integrator && !bipolar_block_collect) {
        test->controller.integrator_d_v = 0.0f;
        test->controller.integrator_q_v = 0.0f;
        if (!g_rdq_hold_snapshot_pending &&
            (g_rdq_block_integrator.block_tick_count != 0u ||
            g_rdq_block_integrator.completed_block_count != 0u ||
            g_rdq_block_integrator.id_axis.admitted ||
            g_rdq_block_integrator.iq_axis.admitted)) {
            rotating_dq_block_integrator_reset(&g_rdq_block_integrator);
        }
    } else if (bipolar_block_collect && test->state_ticks == 0u) {
        rotating_dq_block_integrator_reset(&g_rdq_block_integrator);
    }
    if (external_block_integrator && !external_block_collect) {
        test->controller.integrator_d_v = 0.0f;
        test->controller.integrator_q_v = 0.0f;
        if (!g_rdq_hold_snapshot_pending &&
            (g_rdq_block_integrator.block_tick_count != 0u ||
            g_rdq_block_integrator.completed_block_count != 0u ||
            g_rdq_block_integrator.id_axis.admitted ||
            g_rdq_block_integrator.iq_axis.admitted)) {
            rotating_dq_block_integrator_reset(&g_rdq_block_integrator);
        }
    }
    const float integrator_d_before = test->controller.integrator_d_v;
    const float integrator_q_before = test->controller.integrator_q_v;
    const float id_error = -id_control;
    const float iq_error = test->iq_ref_a - iq_control;
    float id_controller_error = id_error;
    float iq_controller_error = iq_error;
    const bool external_low_res_proportional_average =
        external_block_integrator && input->external_iq_ref_valid &&
        input->current_amp_per_count > 0.0f &&
        rdq_absf(test->iq_ref_a) <=
            (ROTATING_DQ_EXTERNAL_LOW_RES_P_AVERAGE_COUNTS *
             input->current_amp_per_count);
    if (external_low_res_proportional_average) {
        if (external_block_collect &&
            g_rdq_block_integrator.completed_block_count > 0u) {
            id_controller_error =
                g_rdq_block_integrator.last_id_mean_error_a;
            iq_controller_error =
                g_rdq_block_integrator.last_iq_mean_error_a;
        } else {
            /* Until the first coherent block is available, command from the
             * reference and R*Iq feedforward instead of a single ADC count. */
            id_controller_error = 0.0f;
            iq_controller_error = test->iq_ref_a;
        }
    }
    const float id_controller_measurement = -id_controller_error;
    const float iq_controller_measurement =
        test->iq_ref_a - iq_controller_error;
    const bool resistance_feedforward_enabled =
        external_block_integrator && input->external_iq_ref_valid &&
        test->config.phase_resistance_ohm > 0.0f;
    const float vd_feedforward_v = 0.0f;
    const float vq_feedforward_v = resistance_feedforward_enabled ?
        test->config.phase_resistance_ohm * test->iq_ref_a : 0.0f;
    if (zero_output_state) {
        rdq_zero_controller_output(&cc_out);
        cc_out.valid = true;
        current_controller_reset(&test->controller);
    } else {
        CurrentControllerInput cc_in;
        cc_in.id_ref_a = 0.0f;
        cc_in.iq_ref_a = test->iq_ref_a;
        cc_in.id_measured_a = id_controller_measurement;
        cc_in.iq_measured_a = iq_controller_measurement;
        cc_in.theta_rad = input->theta_e_rad;
        cc_in.vbus_v = input->vbus_v;
        cc_in.dt_s = test->config.dt_s;
        cc_in.enable = controller_enable;
        cc_in.fault_active = input->fault_active ||
                             (test->result == ROTATING_DQ_RESULT_FAIL);
        rdq_current_controller_update_fast(&test->controller,
                                           &cc_in,
                                           sin_theta,
                                           cos_theta,
                                           vq_feedforward_v,
                                           freeze_zero_reference_integrator,
                                           block_integrator_defer,
                                           &cc_out);
        if (cc_in.enable && !cc_out.valid) {
            rdq_fail(test, ROTATING_DQ_FAULT_NAN_INF);
        }
    }
    float id_block_delta_v = 0.0f;
    float iq_block_delta_v = 0.0f;
    if (block_zero_reference_integrator && controller_enable &&
        test->result != ROTATING_DQ_RESULT_FAIL &&
        zero_diag_steady_window) {
        (void)rotating_dq_block_integrator_step(
            &g_rdq_block_integrator,
            test->control_tick_seq,
            id_error,
            iq_error,
            !common_mode_harmful,
            test->controller.ki,
            test->config.dt_s,
            &id_block_delta_v,
            &iq_block_delta_v);
        test->controller.integrator_d_v = foc_clamp(
            test->controller.integrator_d_v + id_block_delta_v,
            -test->controller.integrator_limit_v,
            test->controller.integrator_limit_v);
        test->controller.integrator_q_v = foc_clamp(
            test->controller.integrator_q_v + iq_block_delta_v,
            -test->controller.integrator_limit_v,
            test->controller.integrator_limit_v);
        cc_out.integrator_d_v = test->controller.integrator_d_v;
        cc_out.integrator_q_v = test->controller.integrator_q_v;
    } else if ((bipolar_block_collect || external_block_collect) &&
               controller_enable &&
               test->result != ROTATING_DQ_RESULT_FAIL) {
        const int8_t required_iq_sign = (test->iq_ref_a > 0.0f) ? 1 : -1;
        (void)rdq_block_integrator_step_policy(
            &g_rdq_block_integrator,
            test->control_tick_seq,
            id_error,
            iq_error,
            !common_mode_harmful,
            test->controller.ki,
            test->config.dt_s,
            ROTATING_DQ_BIPOLAR_BLOCK_INTEGRATOR_ON_THRESHOLD_A,
            ROTATING_DQ_BIPOLAR_BLOCK_INTEGRATOR_OFF_THRESHOLD_A,
            required_iq_sign,
            false,
            &id_block_delta_v,
            &iq_block_delta_v);
        const float external_low_res_threshold_a =
            ROTATING_DQ_EXTERNAL_LOW_RES_THRESHOLD_COUNTS *
            input->current_amp_per_count;
        if (external_block_collect &&
            rdq_absf(test->iq_ref_a) < external_low_res_threshold_a) {
            iq_block_delta_v = foc_clamp(
                iq_block_delta_v,
                -ROTATING_DQ_EXTERNAL_LOW_RES_MAX_DELTA_V,
                ROTATING_DQ_EXTERNAL_LOW_RES_MAX_DELTA_V);
        }
        test->controller.integrator_d_v = 0.0f;
        test->controller.integrator_q_v = foc_clamp(
            test->controller.integrator_q_v + iq_block_delta_v,
            -test->controller.integrator_limit_v,
            test->controller.integrator_limit_v);
        cc_out.integrator_d_v = 0.0f;
        cc_out.integrator_q_v = test->controller.integrator_q_v;
    }
    const float integrator_d_after = test->controller.integrator_d_v;
    const float integrator_q_after = test->controller.integrator_q_v;
    const float integrator_d_delta = integrator_d_after - integrator_d_before;
    const float integrator_q_delta = integrator_q_after - integrator_q_before;
    float integrator_d_aw_clamp_delta = 0.0f;
    float integrator_q_aw_clamp_delta = 0.0f;
    if (!rdq_bounded_runtime_diagnostics_complete(test)) {
        const bool integrator_update_enabled =
            controller_enable && !freeze_zero_reference_integrator &&
            !block_integrator_defer;
        const float expected_d_ki_delta = block_integrator_defer ?
            id_block_delta_v : integrator_update_enabled ?
            test->controller.ki * id_error * test->config.dt_s : 0.0f;
        const float expected_q_ki_delta = block_integrator_defer ?
            iq_block_delta_v : integrator_update_enabled ?
            test->controller.ki * iq_error * test->config.dt_s : 0.0f;
        integrator_d_aw_clamp_delta =
            integrator_d_delta - expected_d_ki_delta;
        integrator_q_aw_clamp_delta =
            integrator_q_delta - expected_q_ki_delta;
    }
    if (controller_enable && test->result != ROTATING_DQ_RESULT_FAIL) {
        test->voltage_command_seq++;
    }
    profile_now = rdq_profile_now();
    rdq_profile_update(&test->fast_profile.controller_max_cycles,
                       profile_now - profile_section_start);
    profile_section_start = profile_now;

    const uint32_t production_log_stats_start = rdq_total_profile_now();

#if ROTATING_DQ_ENABLE_ZERO_DIAG_LEVEL >= 2u
    if (test->state == ROTATING_DQ_STATE_ENABLE_ZERO &&
        !test->config.enable_zero_diagnostic_only &&
        !current_diag_stored &&
        rdq_zero_diag_should_capture_enable_zero(test)) {
        rdq_zero_diag_store_light_sample(test,
                                         input,
                                         &cc_out,
                                         ROTATING_DQ_ZERO_STAGE_ENABLE_ZERO_MOE_ON,
                                         false,
                                         0u,
                                         0u,
                                         iu,
                                         i_alpha,
                                         i_beta,
                                         id,
                                         iq,
                                         sin_theta,
                                         cos_theta);
    }
#endif

    float phase_abs = rdq_absf(iu);
    const float phase_iv_abs = rdq_absf(input->iv_a);
    const float phase_iw_abs = rdq_absf(input->iw_a);
    if (phase_iv_abs > phase_abs) {
        phase_abs = phase_iv_abs;
    }
    if (phase_iw_abs > phase_abs) {
        phase_abs = phase_iw_abs;
    }
    if (test->config.log_decimation != 0u &&
        ((test->control_tick_seq % test->config.log_decimation) == 0u)) {
        RotatingDqCurrentLogRecord rec;
        rec.state = test->state;
        rec.control_tick_seq = test->control_tick_seq;
        rec.adc_seq = input->adc_seq;
        rec.voltage_command_seq = test->voltage_command_seq;
        rec.encoder_count = input->encoder_count;
        rec.theta_e_rad = input->theta_e_rad;
        rec.mechanical_speed_rpm = test->mechanical_speed_rpm;
        rec.id_ref_a = 0.0f;
        rec.iq_ref_a = test->iq_ref_a;
        rec.iu_a = iu;
        rec.iv_a = input->iv_a;
        rec.iw_a = input->iw_a;
        rec.id_a = id;
        rec.iq_a = iq;
        rec.vd_v = cc_out.vd_v;
        rec.vq_v = cc_out.vq_v;
        rec.v_alpha_v = cc_out.v_alpha_v;
        rec.v_beta_v = cc_out.v_beta_v;
        rec.integrator_d_v = cc_out.integrator_d_v;
        rec.integrator_q_v = cc_out.integrator_q_v;
        rec.saturation_active = cc_out.saturation_active;
        rec.vbus_v = input->vbus_v;
        rec.fault_code = test->fault_code;
        rdq_log(test, &rec);
    }

    if (test->state == ROTATING_DQ_STATE_ENABLE_ZERO &&
        test->config.enable_zero_diagnostic_only &&
        zero_diag_steady_window && !startup_soft_current_observed &&
        !common_mode_harmful) {
        rdq_zero_stats_update_fast(&test->zero_stats, id, iq,
                                   reconstructed_phase_metric);
    } else if (test->state == ROTATING_DQ_STATE_RAMP_IQ_POSITIVE ||
               test->state == ROTATING_DQ_STATE_HOLD_IQ_POSITIVE) {
        if (test->config.require_direction_match ||
            test->iq_ref_a >= test->config.tracking_iq_ref_mean_min_a) {
            const bool direction_capture_disabled =
                test->config.direction_capture_counts >= 1000000;
            const bool direction_stats_not_required =
                direction_capture_disabled &&
                test->config.enable_external_iq_block_integrator;
            const bool bounded_window_complete =
                direction_capture_disabled &&
                test->positive_stats.sample_count >=
                    ROTATING_DQ_DIRECTION_STATS_MAX_SAMPLES;
            if (!direction_stats_not_required && !bounded_window_complete) {
                rdq_direction_stats_update_fast(&test->positive_stats,
                                                test->iq_ref_a,
                                                iq,
                                                id,
                                                test->mechanical_speed_rpm,
                                                phase_abs,
                                                cc_out.saturation_active,
                                                encoder_delta);
            }
        }
    } else if (test->state == ROTATING_DQ_STATE_RAMP_IQ_NEGATIVE ||
               test->state == ROTATING_DQ_STATE_HOLD_IQ_NEGATIVE) {
        if (test->config.require_direction_match ||
            test->iq_ref_a <= -test->config.tracking_iq_ref_mean_min_a) {
            const bool direction_capture_disabled =
                test->config.direction_capture_counts >= 1000000;
            const bool direction_stats_not_required =
                direction_capture_disabled &&
                test->config.enable_external_iq_block_integrator;
            const bool bounded_window_complete =
                direction_capture_disabled &&
                test->negative_stats.sample_count >=
                    ROTATING_DQ_DIRECTION_STATS_MAX_SAMPLES;
            if (!direction_stats_not_required && !bounded_window_complete) {
                rdq_direction_stats_update_fast(&test->negative_stats,
                                                test->iq_ref_a,
                                                iq,
                                                id,
                                                test->mechanical_speed_rpm,
                                                phase_abs,
                                                cc_out.saturation_active,
                                                encoder_delta);
            }
        }
    }

    if ((bipolar_block_collect || external_block_collect) &&
        !rdq_block_hold_tracking_window_complete(test->state)) {
        rdq_block_hold_snapshot_update(test->state,
                                       test->control_tick_seq,
                                       test->iq_ref_a,
                                       id_control,
                                       iq_control,
                                       test->controller.integrator_q_v,
                                       cc_out.saturation_active);
    }

    if (cc_out.saturation_active) {
        test->saturation_ticks++;
        if (test->saturation_ticks > test->config.saturation_limit_ticks) {
            rdq_fail(test, ROTATING_DQ_FAULT_SATURATION);
        }
    } else {
        test->saturation_ticks = 0u;
    }

    if (zero_output_state) {
        test->tracking_error_ticks = 0u;
    } else if (rdq_absf(test->iq_ref_a - iq_control) > test->config.tracking_error_limit_a) {
        test->tracking_error_ticks++;
        if (test->tracking_error_ticks > test->config.tracking_error_limit_ticks) {
            rdq_fail(test, ROTATING_DQ_FAULT_TRACKING);
        }
    } else {
        test->tracking_error_ticks = 0u;
    }
    profile_now = rdq_profile_now();
    profile_log_stats_cycles = profile_now - profile_section_start;
    rdq_profile_update(&test->fast_profile.log_stats_max_cycles,
                       profile_log_stats_cycles);
    profile_section_start = profile_now;
    const uint32_t production_log_stats_cycles =
        rdq_total_profile_now() - production_log_stats_start;
    const uint32_t zero_steady_sample_count =
        test->zero_clean_sample_count +
        test->zero_common_mode_excluded_count;

    if (test->result != ROTATING_DQ_RESULT_FAIL) {
        test->state_ticks++;
        switch (test->state) {
        case ROTATING_DQ_STATE_ENABLE_ZERO:
            if (test->config.enable_zero_diagnostic_only &&
                !test->zero_diag_completed &&
                zero_steady_sample_count >= 128u) {
                rdq_stats_update_mean_fields(&test->zero_stats);
            }
            if (motion > test->config.zero_encoder_limit_counts) {
                rdq_fail(test, ROTATING_DQ_FAULT_ENCODER);
            } else if (test->config.enable_zero_diagnostic_only &&
                       !test->config.zero_window_observe_only &&
                       !test->zero_diag_completed &&
                       zero_steady_sample_count >= 128u &&
                       test->zero_measured_phase_metric_max_a >
                           test->config.zero_phase_current_peak_limit_a &&
                       (test->zero_phase_over_limit_consecutive_max >=
                            ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS ||
                        rdq_absf(test->zero_stats.id_mean_a) >
                            test->config.zero_dq_mean_limit_a ||
                        rdq_absf(test->zero_stats.iq_mean_a) >
                            test->config.zero_dq_mean_limit_a)) {
                rdq_fail(test, ROTATING_DQ_FAULT_CURRENT_OFFSET_INVALID);
            } else if (test->config.enable_zero_diagnostic_only &&
                       !test->config.zero_window_observe_only &&
                       !test->zero_diag_completed &&
                       zero_steady_sample_count >= 128u &&
                       test->zero_clean_sample_count <
                           test->config.zero_clean_sample_min) {
                rdq_fail(test, ROTATING_DQ_FAULT_CURRENT_SENSE_COMMON_MODE_EXCESS);
            } else if (test->config.enable_zero_diagnostic_only &&
                       !test->config.zero_window_observe_only &&
                       !test->zero_diag_completed &&
                       zero_steady_sample_count >= 128u &&
                       ((test->zero_clean_phase_metric_max_a >
                             test->config.zero_phase_current_peak_limit_a &&
                         test->zero_phase_over_limit_consecutive_max >=
                             ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS) ||
                        rdq_absf(test->zero_stats.id_mean_a) >
                            test->config.zero_dq_mean_limit_a ||
                        rdq_absf(test->zero_stats.iq_mean_a) >
                            test->config.zero_dq_mean_limit_a)) {
                rdq_fail(test, ROTATING_DQ_FAULT_ZERO_CURRENT_INVALID);
            } else if (test->config.enable_zero_diagnostic_only &&
                       !test->zero_diag_completed &&
                       zero_steady_sample_count >= 128u) {
                test->zero_diag_completed = true;
                if (test->state_ticks >=
                    test->config.enable_zero_diagnostic_ticks) {
                    rdq_advance(test, ROTATING_DQ_STATE_COMPLETE);
                }
            } else if (test->config.enable_zero_diagnostic_only &&
                       !test->zero_diag_completed &&
                       test->state_ticks >=
                           test->config.enable_zero_diagnostic_ticks) {
                /* Short diagnostic modes intentionally have no 128-point gate. */
                test->zero_diag_completed = true;
                rdq_advance(test, ROTATING_DQ_STATE_COMPLETE);
            } else if (test->config.enable_zero_diagnostic_only &&
                       test->zero_diag_completed &&
                       test->state_ticks >= test->config.enable_zero_diagnostic_ticks) {
                rdq_advance(test, ROTATING_DQ_STATE_COMPLETE);
            } else if (test->state_ticks >= test->config.enable_zero_ticks) {
                rdq_advance(test, ROTATING_DQ_STATE_RAMP_IQ_POSITIVE);
            }
            break;
        case ROTATING_DQ_STATE_HOLD_IQ_POSITIVE:
            if (rdq_abs_i64(test->positive_stats.encoder_delta_counts) >=
                    test->config.direction_capture_counts ||
                test->state_ticks >= test->config.iq_hold_ticks) {
                rdq_block_hold_snapshot_finalize(test->state);
                rdq_advance(test, ROTATING_DQ_STATE_RAMP_ZERO_1);
            }
            break;
        case ROTATING_DQ_STATE_HOLD_ZERO_1:
            if (test->state_ticks >= test->config.hold_zero_ticks) {
                rdq_advance(test,
                            test->config.single_direction_positive_only
                                ? ROTATING_DQ_STATE_COMPLETE
                                : ROTATING_DQ_STATE_RAMP_IQ_NEGATIVE);
            }
            break;
        case ROTATING_DQ_STATE_HOLD_IQ_NEGATIVE:
            if (rdq_abs_i64(test->negative_stats.encoder_delta_counts) >=
                    test->config.direction_capture_counts ||
                test->state_ticks >= test->config.iq_hold_ticks) {
                rdq_block_hold_snapshot_finalize(test->state);
                rdq_advance(test, ROTATING_DQ_STATE_RAMP_ZERO_2);
            }
            break;
        case ROTATING_DQ_STATE_HOLD_ZERO_2:
            if (test->state_ticks >= test->config.hold_zero_ticks) {
                rdq_advance(test, ROTATING_DQ_STATE_COMPLETE);
            }
            break;
        default:
            break;
        }
    }
    profile_now = rdq_profile_now();
    rdq_profile_update(&test->fast_profile.state_machine_max_cycles,
                       profile_now - profile_section_start);
    profile_section_start = profile_now;

    if (input->compact_output_requested) {
        rdq_fill_output_compact(test, &cc_out, output);
    } else {
        rdq_fill_output(test, &cc_out, output);
        output->vd_diagnostic_v = cc_out.vd_v;
        output->vq_diagnostic_v = cc_out.vq_v;
        output->vd_proportional_v = test->controller.kp * id_controller_error;
        output->vq_proportional_v = test->controller.kp * iq_controller_error;
        output->vd_integrator_v = integrator_d_before;
        output->vq_integrator_v = integrator_q_before;
        output->vd_feedforward_v = vd_feedforward_v;
        output->vq_feedforward_v = vq_feedforward_v;
        output->vd_unclamped_v = cc_out.vd_unsat_v;
        output->vq_unclamped_v = cc_out.vq_unsat_v;
        output->iu_measured_a = iu;
        output->iv_measured_a = input->iv_a;
        output->iw_measured_a = input->iw_a;
        output->id_control_a = id_controller_measurement;
        output->iq_control_a = iq_controller_measurement;
        output->id_error_a = id_controller_error;
        output->iq_error_a = iq_controller_error;
        output->integrator_d_before_v = integrator_d_before;
        output->integrator_q_before_v = integrator_q_before;
        output->integrator_d_after_v = integrator_d_after;
        output->integrator_q_after_v = integrator_q_after;
        output->integrator_d_delta_v = integrator_d_delta;
        output->integrator_q_delta_v = integrator_q_delta;
        output->integrator_d_aw_clamp_delta_v = integrator_d_aw_clamp_delta;
        output->integrator_q_aw_clamp_delta_v = integrator_q_aw_clamp_delta;
    }
    output->id_measured_a = id;
    output->iq_measured_a = iq;
    output->common_mode_shape = common_mode_counts_like;
    output->common_mode_harmful = common_mode_harmful;
    output->common_mode_caused_dq_crossing = dq_crossing_caused_by_cm;
    profile_now = rdq_profile_now();
    rdq_profile_update(&test->fast_profile.fill_output_max_cycles,
                       profile_now - profile_section_start);
    const uint32_t profile_total_cycles =
        rdq_total_profile_now() - profile_total_start;
    rdq_total_profile_update(&test->fast_profile.total_max_cycles,
                             profile_total_cycles);
    rdq_total_profile_update(&test->fast_profile.control_critical_max_cycles,
                             profile_total_cycles -
                                 production_log_stats_cycles);
}

void rotating_dq_current_test_service_main(const RotatingDqCurrentTest *test,
                                           RotatingDqCurrentTestOutput *output)
{
    CurrentControllerOutput zero;
    memset(&zero, 0, sizeof(zero));
    if (output != 0) {
        memset(output, 0, sizeof(*output));
    }
    if (test == 0 || output == 0) {
        return;
    }
    rdq_block_hold_snapshot_service_main();
    rdq_fill_output(test, &zero, output);
}

const char *rotating_dq_current_test_state_name(RotatingDqCurrentTestState state)
{
    switch (state) {
    case ROTATING_DQ_STATE_PREFLIGHT: return "PREFLIGHT";
    case ROTATING_DQ_STATE_OFFSET_CAL: return "OFFSET_CAL";
    case ROTATING_DQ_STATE_OFFSET_VERIFY: return "OFFSET_VERIFY";
    case ROTATING_DQ_STATE_ENABLE_ZERO: return "ENABLE_ZERO";
    case ROTATING_DQ_STATE_RAMP_IQ_POSITIVE: return "RAMP_IQ_POSITIVE";
    case ROTATING_DQ_STATE_HOLD_IQ_POSITIVE: return "HOLD_IQ_POSITIVE";
    case ROTATING_DQ_STATE_RAMP_ZERO_1: return "RAMP_ZERO_1";
    case ROTATING_DQ_STATE_HOLD_ZERO_1: return "HOLD_ZERO_1";
    case ROTATING_DQ_STATE_RAMP_IQ_NEGATIVE: return "RAMP_IQ_NEGATIVE";
    case ROTATING_DQ_STATE_HOLD_IQ_NEGATIVE: return "HOLD_IQ_NEGATIVE";
    case ROTATING_DQ_STATE_RAMP_ZERO_2: return "RAMP_ZERO_2";
    case ROTATING_DQ_STATE_HOLD_ZERO_2: return "HOLD_ZERO_2";
    case ROTATING_DQ_STATE_COMPLETE: return "COMPLETE";
    case ROTATING_DQ_STATE_FAIL: return "FAIL";
    default: return "UNKNOWN";
    }
}

const char *rotating_dq_current_test_result_name(RotatingDqCurrentTestResult result)
{
    switch (result) {
    case ROTATING_DQ_RESULT_NOT_RUN: return "NOT_RUN";
    case ROTATING_DQ_RESULT_RUNNING: return "RUNNING";
    case ROTATING_DQ_RESULT_PASS: return "PASS";
    case ROTATING_DQ_RESULT_FAIL: return "FAIL";
    default: return "UNKNOWN";
    }
}

int rotating_dq_velocity_iq_sign_candidate(const RotatingDqCurrentTest *test)
{
    if (test == 0 ||
        test->positive_stats.mechanical_direction == 0 ||
        test->negative_stats.mechanical_direction == 0 ||
        test->positive_stats.mechanical_direction ==
            test->negative_stats.mechanical_direction) {
        return 0;
    }

    return test->positive_stats.mechanical_direction > 0 ? 1 : -1;
}

const char *rotating_dq_zero_diag_stage_name(RotatingDqZeroDiagStage stage)
{
    switch (stage) {
    case ROTATING_DQ_ZERO_STAGE_BASELINE_MOE_OFF: return "BASELINE_MOE_OFF";
    case ROTATING_DQ_ZERO_STAGE_NEUTRAL_PRELOAD_MOE_OFF: return "NEUTRAL_PRELOAD_MOE_OFF";
    case ROTATING_DQ_ZERO_STAGE_ENABLE_ZERO_MOE_ON: return "ENABLE_ZERO_MOE_ON";
    case ROTATING_DQ_ZERO_STAGE_POST_SHUTDOWN: return "POST_SHUTDOWN";
    case ROTATING_DQ_ZERO_STAGE_NONE:
    default: return "NONE";
    }
}

const char *rotating_dq_zero_diag_classification(const RotatingDqCurrentTest *test)
{
    if (test == 0) {
        return "INCONCLUSIVE";
    }
    if (!test->zero_first_trip.valid) {
        return test->zero_diag_completed ?
                   "ENABLE_ZERO_DATA_VALID" :
                   "INCONCLUSIVE";
    }
    const RotatingDqZeroDiagSample *s = &test->zero_first_trip.sample;
    const bool ccr_neutral = (s->ccr1 == s->ccr2) && (s->ccr2 == s->ccr3);
    const bool voltage_near_zero =
        rdq_absf(s->vd_applied_v) < 0.005f &&
        rdq_absf(s->vq_applied_v) < 0.005f &&
        rdq_absf(s->v_alpha_v) < 0.005f &&
        rdq_absf(s->v_beta_v) < 0.005f;
    const bool phase_trip = (s->source_mask & ROTATING_DQ_ZERO_TRIP_PHASE_METRIC) != 0u;
    const bool dq_trip = (s->source_mask & ROTATING_DQ_ZERO_TRIP_DQ_METRIC) != 0u;
    if (test->zero_diag_completed &&
        test->fault_code == ROTATING_DQ_FAULT_NONE &&
        test->zero_first_trip.control_tick <=
            ROTATING_DQ_ENABLE_ZERO_STARTUP_OBSERVE_TICKS &&
        (phase_trip || dq_trip) &&
        (test->zero_phase_startup_over_limit_count != 0u ||
         test->zero_dq_startup_over_limit_count != 0u)) {
        return "STARTUP_CURRENT_TRANSIENT_OBSERVED";
    }
    if (!phase_trip && dq_trip) {
        if (test->zero_dq_startup_over_limit_count != 0u &&
            test->zero_first_trip.control_tick <=
                ROTATING_DQ_ENABLE_ZERO_STARTUP_OBSERVE_TICKS) {
            return "DQ_STARTUP_TRANSIENT_OBSERVED";
        }
        return "DQ_TRANSFORM_INVALID";
    }
    if (phase_trip && dq_trip &&
        s->delta_pc0 != 0 && s->delta_pc1 != 0 &&
        ((s->delta_pc0 > 0 && s->delta_pc1 > 0) ||
         (s->delta_pc0 < 0 && s->delta_pc1 < 0))) {
        const int32_t abs0 =
            (s->delta_pc0 < 0) ? -s->delta_pc0 : s->delta_pc0;
        const int32_t abs1 =
            (s->delta_pc1 < 0) ? -s->delta_pc1 : s->delta_pc1;
        const int32_t diff = (abs0 > abs1) ? (abs0 - abs1) : (abs1 - abs0);
        if (diff <= 2) {
            return "COMMON_MODE_ADC_SHIFT_RECONSTRUCTED_IU_LIMIT";
        }
        return "COMMON_MODE_ADC_SHIFT";
    }
    if (phase_trip && dq_trip) {
        const int32_t abs0 =
            (s->delta_pc0 < 0) ? -s->delta_pc0 : s->delta_pc0;
        const int32_t abs1 =
            (s->delta_pc1 < 0) ? -s->delta_pc1 : s->delta_pc1;
        if ((abs0 >= 5 && abs1 <= 1) || (abs1 >= 5 && abs0 <= 1)) {
            return "SINGLE_CHANNEL_ADC_SPIKE";
        }
        if ((s->delta_pc0 > 0 && s->delta_pc1 < 0) ||
            (s->delta_pc0 < 0 && s->delta_pc1 > 0)) {
            return "DIFFERENTIAL_CURRENT_OR_SAMPLE_NOISE";
        }
    }
    if (phase_trip && dq_trip && ccr_neutral && voltage_near_zero) {
        return "LIVE_ZERO_OFFSET_MISMATCH";
    }
    if ((s->source_mask & ROTATING_DQ_ZERO_TRIP_PI_INTEGRATOR_NONZERO) != 0u ||
        (s->source_mask & ROTATING_DQ_ZERO_TRIP_PI_VD_NONZERO) != 0u ||
        (s->source_mask & ROTATING_DQ_ZERO_TRIP_PI_VQ_NONZERO) != 0u) {
        return "PI_STARTUP_COMMAND_TRANSIENT";
    }
    if (phase_trip && !voltage_near_zero) {
        return "ZERO_CURRENT_SAMPLE_ERROR_FEEDS_PI";
    }
    if ((s->source_mask & ROTATING_DQ_ZERO_TRIP_ANGLE_INVALID) != 0u) {
        return "ANGLE_STARTUP_INVALID";
    }
    return "INCONCLUSIVE";
}

float rotating_dq_current_test_kp(const RotatingDqCurrentTest *test)
{
    return (test != 0) ? test->controller.kp : 0.0f;
}

float rotating_dq_current_test_ki(const RotatingDqCurrentTest *test)
{
    return (test != 0) ? test->controller.ki : 0.0f;
}

float rotating_dq_current_test_ki_times_ts(const RotatingDqCurrentTest *test)
{
    return (test != 0) ? (test->controller.ki * test->config.dt_s) : 0.0f;
}

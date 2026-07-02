#ifndef MOTOR_IDENTIFICATION_H
#define MOTOR_IDENTIFICATION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_IDENT_RISE_SAMPLES 30u
#define MOTOR_IDENT_FALL_SAMPLES 80u

typedef enum {
    MOTOR_IDENT_STATUS_NOT_RUN = 0,
    MOTOR_IDENT_STATUS_PASS,
    MOTOR_IDENT_STATUS_UNRELIABLE,
    MOTOR_IDENT_STATUS_FAIL
} MotorIdentStatus;

typedef struct {
    MotorIdentStatus status;
    bool test_completed_safely;
    bool phase_inductance_identification_reliable;
    bool current_loop_enable_permitted;
} MotorIdentClassification;

typedef struct {
    float rise_delta_i[MOTOR_IDENT_RISE_SAMPLES];
    float fall_delta_i[MOTOR_IDENT_FALL_SAMPLES];
    float beta_delta_i[MOTOR_IDENT_RISE_SAMPLES];
    uint32_t rise_count;
    uint32_t fall_count;
    float beta_ratio;
    bool hard_fault;
    bool pulse_too_short;
    bool dynamics_too_fast;
} MotorIdentPulseQuality;

typedef struct {
    float inductance_h;
    float tau_s;
    float r_squared;
    bool valid;
} MotorIdentFitResult;

typedef struct {
    float fitted_offset_a;
    float fitted_amplitude_a;
    float fitted_tau_s;
    float fitted_inductance_h;
    float r_squared;
    float residual_rms_a;
    float residual_max_a;
    float weighted_residual_rms_a;
    float normalized_residual_rms;
    float window_score;
    float residual_curvature;
    uint32_t residual_max_index;
    float residual_max_counts;
    uint32_t residual_same_sign_run;
    uint32_t start_index;
    uint32_t end_index;
    uint32_t point_count;
    bool valid;
} MotorIdentCurveFit;

typedef struct {
    float a;
    float b;
    float c;
    float resistance_ohm;
    float inductance_h;
    float tau_s;
    float r_squared;
    bool valid;
} MotorIdentArxFit;

typedef struct {
    uint32_t window_start;
    uint32_t window_end;
    uint32_t comparison_count;
    uint32_t violation_count;
    float violation_ratio;
    float max_violation_a;
    float max_violation_counts;
    float tolerance_max_a;
    float global_trend_slope;
    bool ok;
} MotorIdentMonotonicStats;

typedef struct {
    float noise_sigma_a;
    float fall_tail_sigma_a;
    float effective_noise_a;
    float effective_noise_counts;
    float rise_tail_median_a;
    float rise_peak_a;
    float rise_peak_minus_tail_a;
    bool monotonic_rise_ok;
    bool monotonic_fall_ok;
    MotorIdentMonotonicStats rise_monotonic;
    MotorIdentMonotonicStats fall_monotonic;
    MotorIdentCurveFit rise_fit;
    MotorIdentCurveFit fall_fit;
    MotorIdentArxFit arx_free;
    MotorIdentArxFit arx_fixed_r;
    MotorIdentArxFit arx_smoothed_fixed_r;
    float fused_inductance_h;
    uint32_t fused_method_count;
    bool reliable;
} MotorIdentRobustResult;

bool motor_ident_beta_quality_finalize(MotorIdentPulseQuality *quality,
                                       float beta_ratio_max);
bool motor_ident_pulse_too_short(const float *rise,
                                 uint32_t rise_count,
                                 float expected_final_current_a);
bool motor_ident_dynamics_too_fast(const float *rise, uint32_t rise_count);
MotorIdentClassification motor_ident_classify(bool test_completed_safely,
                                               bool level_a_reliable,
                                               bool level_b_reliable,
                                               float level_difference_percent,
                                               float max_difference_percent);
MotorIdentFitResult motor_ident_fit_rl_rise(const float *rise,
                                            uint32_t count,
                                            float sample_period_s,
                                            float resistance_ohm,
                                            float voltage_step_v);

bool motor_ident_robust_fit_level(const float *rise,
                                  uint32_t rise_count,
                                  const float *rise_std,
                                  const float *fall,
                                  uint32_t fall_count,
                                  const float *fall_std,
                                  float sample_period_s,
                                  float resistance_ohm,
                                  float voltage_step_v,
                                  float current_amp_per_count,
                                  MotorIdentRobustResult *result);

bool motor_ident_fit_repeated_arx_fixed_r(const float *rise_repeats,
                                          uint32_t repeat_count,
                                          uint32_t rise_count,
                                          const float *fall_repeats,
                                          uint32_t fall_count,
                                          float sample_period_s,
                                          float resistance_ohm,
                                          float voltage_step_v,
                                          MotorIdentArxFit *fit);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_IDENTIFICATION_H */

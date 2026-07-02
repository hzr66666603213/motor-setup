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

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_IDENTIFICATION_H */

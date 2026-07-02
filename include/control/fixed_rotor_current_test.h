#ifndef FIXED_ROTOR_CURRENT_TEST_H
#define FIXED_ROTOR_CURRENT_TEST_H

#include "control/current_controller.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FIXED_ROTOR_CURRENT_TEST_LOG_CAPACITY 512u

typedef enum {
    FIXED_ROTOR_STATE_PREFLIGHT = 0,
    FIXED_ROTOR_STATE_ENABLE_ZERO,
    FIXED_ROTOR_STATE_RAMP_ID_0P05,
    FIXED_ROTOR_STATE_HOLD_ID_0P05,
    FIXED_ROTOR_STATE_RAMP_ID_0P10,
    FIXED_ROTOR_STATE_HOLD_ID_0P10,
    FIXED_ROTOR_STATE_RAMP_ZERO,
    FIXED_ROTOR_STATE_HOLD_ZERO,
    FIXED_ROTOR_STATE_COMPLETE,
    FIXED_ROTOR_STATE_FAIL
} FixedRotorCurrentTestState;

typedef enum {
    FIXED_ROTOR_RESULT_NOT_RUN = 0,
    FIXED_ROTOR_RESULT_RUNNING,
    FIXED_ROTOR_RESULT_PASS,
    FIXED_ROTOR_RESULT_FAIL
} FixedRotorCurrentTestResult;

typedef enum {
    FIXED_ROTOR_FAULT_NONE = 0u,
    FIXED_ROTOR_FAULT_PHASE_CURRENT_LIMIT = 1u << 0,
    FIXED_ROTOR_FAULT_DQ_CURRENT_LIMIT = 1u << 1,
    FIXED_ROTOR_FAULT_IQ_DEVIATION = 1u << 2,
    FIXED_ROTOR_FAULT_ADC_SEQ_GAP = 1u << 3,
    FIXED_ROTOR_FAULT_ADC_DUPLICATE = 1u << 4,
    FIXED_ROTOR_FAULT_PIPELINE = 1u << 5,
    FIXED_ROTOR_FAULT_NFAULT = 1u << 6,
    FIXED_ROTOR_FAULT_DRV = 1u << 7,
    FIXED_ROTOR_FAULT_VBUS = 1u << 8,
    FIXED_ROTOR_FAULT_ENCODER = 1u << 9,
    FIXED_ROTOR_FAULT_ROTOR_MOVED = 1u << 10,
    FIXED_ROTOR_FAULT_M1 = 1u << 11,
    FIXED_ROTOR_FAULT_TIMEOUT = 1u << 12,
    FIXED_ROTOR_FAULT_NAN_INF = 1u << 13,
    FIXED_ROTOR_FAULT_PWM_CCR = 1u << 14,
    FIXED_ROTOR_FAULT_TRACKING = 1u << 15,
    FIXED_ROTOR_FAULT_SATURATION = 1u << 16,
    FIXED_ROTOR_FAULT_CONTROL_TIME = 1u << 17
} FixedRotorCurrentTestFault;

typedef struct {
    float phase_resistance_ohm;
    float phase_inductance_h;
    float bandwidth_hz;
    float voltage_limit_v;
    float kaw;
    float integrator_limit_v;
    float dt_s;
    float id_ref_max_a;
    float id_ramp_rate_a_per_s;
    float phase_current_limit_a;
    float dq_current_limit_a;
    float iq_deviation_limit_a;
    uint32_t iq_deviation_limit_ticks;
    float tracking_error_limit_a;
    uint32_t tracking_error_limit_ticks;
    uint32_t saturation_limit_ticks;
    int64_t encoder_motion_limit_counts;
    float vbus_min_v;
    float vbus_max_v;
    uint32_t log_decimation;
    uint32_t preflight_ticks;
    uint32_t enable_zero_ticks;
    uint32_t hold_0p05_ticks;
    uint32_t hold_0p10_ticks;
    uint32_t hold_zero_ticks;
    float control_time_limit_us;
} FixedRotorCurrentTestConfig;

typedef struct {
    uint64_t time_us;
    uint32_t adc_seq;
    int64_t encoder_count;
    float theta_test_rad;
    float iv_a;
    float iw_a;
    float vbus_v;
    bool adc_valid;
    bool encoder_valid;
    bool nfault_ok;
    bool drv_ok;
    bool m1_safe;
    bool pwm_ccr_ok;
    bool pwm_allowed;
    bool fault_active;
} FixedRotorCurrentTestInput;

typedef struct {
    FixedRotorCurrentTestState state;
    uint32_t control_tick_seq;
    uint32_t adc_seq;
    uint32_t voltage_command_seq;
    int64_t encoder_count;
    float theta_test_rad;
    float id_ref_a;
    float iq_ref_a;
    float iu_a;
    float iv_a;
    float iw_a;
    float id_a;
    float iq_a;
    float error_d_a;
    float error_q_a;
    float vd_unsat_v;
    float vq_unsat_v;
    float vd_v;
    float vq_v;
    float v_alpha_v;
    float v_beta_v;
    float integrator_d_v;
    float integrator_q_v;
    float vbus_v;
    bool saturation_active;
    uint32_t fault_code;
} FixedRotorCurrentLogRecord;

typedef struct {
    float id_ref_mean;
    float id_mean;
    float id_std;
    float id_error_mean;
    float id_error_peak;
    float iq_mean;
    float iq_std;
    float iq_peak;
    float phase_current_peak;
    float voltage_vector_peak;
    uint32_t saturation_count;
    uint32_t sample_count;
    uint32_t control_tick_gap_count;
    float overshoot_percent;
    float settling_time_ms;
    float steady_state_error_a;
    float cross_axis_iq_peak_a;
} FixedRotorCurrentStageStats;

typedef struct {
    FixedRotorCurrentTestState state;
    FixedRotorCurrentTestResult result;
    uint32_t fault_code;
    float id_ref_a;
    float iq_ref_a;
    float vd_v;
    float vq_v;
    float v_alpha_v;
    float v_beta_v;
    bool power_stage_request;
    bool pwm_output_request;
    bool safe_shutdown_request;
    bool done;
} FixedRotorCurrentTestOutput;

typedef struct {
    FixedRotorCurrentTestConfig config;
    CurrentController controller;
    FixedRotorCurrentTestState state;
    FixedRotorCurrentTestResult result;
    uint32_t fault_code;
    uint32_t control_tick_seq;
    uint32_t voltage_command_seq;
    uint32_t missed_control_tick_count;
    uint32_t duplicate_control_tick_count;
    uint32_t worst_case_control_cycles;
    float worst_case_control_time_us;
    uint32_t state_ticks;
    uint32_t last_adc_seq;
    bool have_last_adc_seq;
    bool theta_latched;
    float theta_test_rad;
    int64_t encoder_start_count;
    int64_t encoder_motion_max_counts;
    float id_ref_a;
    uint32_t iq_deviation_ticks;
    uint32_t tracking_error_ticks;
    uint32_t saturation_ticks;
    FixedRotorCurrentStageStats enable_zero_stats;
    FixedRotorCurrentStageStats hold_0p05_stats;
    FixedRotorCurrentStageStats hold_0p10_stats;
    FixedRotorCurrentStageStats hold_zero_stats;
    FixedRotorCurrentLogRecord log[FIXED_ROTOR_CURRENT_TEST_LOG_CAPACITY];
    uint32_t log_count;
    uint32_t log_dropped;
} FixedRotorCurrentTest;

FixedRotorCurrentTestConfig fixed_rotor_current_test_default_config(void);
void fixed_rotor_current_test_init(FixedRotorCurrentTest *test,
                                   const FixedRotorCurrentTestConfig *config);
void fixed_rotor_current_test_reset(FixedRotorCurrentTest *test);
void fixed_rotor_current_test_step(FixedRotorCurrentTest *test,
                                   const FixedRotorCurrentTestInput *input,
                                   FixedRotorCurrentTestOutput *output);
void fixed_rotor_current_test_note_execution_time(FixedRotorCurrentTest *test,
                                                  uint32_t cycles,
                                                  float time_us);
const char *fixed_rotor_current_test_state_name(FixedRotorCurrentTestState state);
const char *fixed_rotor_current_test_result_name(FixedRotorCurrentTestResult result);
float fixed_rotor_current_test_kp(const FixedRotorCurrentTest *test);
float fixed_rotor_current_test_ki(const FixedRotorCurrentTest *test);
float fixed_rotor_current_test_ki_times_ts(const FixedRotorCurrentTest *test);

#ifdef __cplusplus
}
#endif

#endif /* FIXED_ROTOR_CURRENT_TEST_H */

#ifndef ROTATING_DQ_CURRENT_TEST_H
#define ROTATING_DQ_CURRENT_TEST_H

#include "control/current_controller.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROTATING_DQ_CURRENT_TEST_LOG_CAPACITY 508u
#define ROTATING_DQ_ZERO_DIAG_CAPACITY 62u
#define ROTATING_DQ_ENABLE_ZERO_STARTUP_OBSERVE_TICKS 16u
#define ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS 4u
#define ROTATING_DQ_BLOCK_INTEGRATOR_TICKS 32u
#define ROTATING_DQ_BLOCK_INTEGRATOR_MIN_VALID 28u
#define ROTATING_DQ_BLOCK_INTEGRATOR_ON_THRESHOLD_A 0.050f
#define ROTATING_DQ_BLOCK_INTEGRATOR_OFF_THRESHOLD_A 0.030f
#define ROTATING_DQ_BLOCK_INTEGRATOR_ON_BLOCKS 3u
#define ROTATING_DQ_BLOCK_INTEGRATOR_OFF_BLOCKS 2u
#define ROTATING_DQ_BIPOLAR_BLOCK_INTEGRATOR_ON_THRESHOLD_A 0.008f
#define ROTATING_DQ_BIPOLAR_BLOCK_INTEGRATOR_OFF_THRESHOLD_A 0.004f
#define ROTATING_DQ_EXTERNAL_BLOCK_REF_RESET_DELTA_A 0.004f
#define ROTATING_DQ_EXTERNAL_BLOCK_MIN_ADC_COUNTS 0.75f
#define ROTATING_DQ_EXTERNAL_BLOCK_EXIT_ADC_COUNTS 0.20f
#define ROTATING_DQ_EXTERNAL_LOW_RES_THRESHOLD_COUNTS 1.5f
#define ROTATING_DQ_EXTERNAL_LOW_RES_MAX_DELTA_V 0.005f
#define ROTATING_DQ_EXTERNAL_LOW_RES_P_AVERAGE_COUNTS 1.25f
#define ROTATING_DQ_BIPOLAR_HOLD_TRACKING_MIN_SAMPLES 128u
#define ROTATING_DQ_BIPOLAR_HOLD_TRACKING_WINDOW_SAMPLES 128u
#define ROTATING_DQ_DIRECTION_STATS_MAX_SAMPLES 128u
#ifndef ROTATING_DQ_ENABLE_ZERO_DIAG_LEVEL
#define ROTATING_DQ_ENABLE_ZERO_DIAG_LEVEL 1u
#endif

typedef enum {
    ROTATING_DQ_ZERO_STAGE_NONE = 0,
    ROTATING_DQ_ZERO_STAGE_BASELINE_MOE_OFF,
    ROTATING_DQ_ZERO_STAGE_NEUTRAL_PRELOAD_MOE_OFF,
    ROTATING_DQ_ZERO_STAGE_ENABLE_ZERO_MOE_ON,
    ROTATING_DQ_ZERO_STAGE_POST_SHUTDOWN
} RotatingDqZeroDiagStage;

typedef enum {
    ROTATING_DQ_ZERO_TRIP_NONE = 0u,
    ROTATING_DQ_ZERO_TRIP_RAW_PC0 = 1u << 0,
    ROTATING_DQ_ZERO_TRIP_RAW_PC1 = 1u << 1,
    ROTATING_DQ_ZERO_TRIP_IU = 1u << 2,
    ROTATING_DQ_ZERO_TRIP_IV = 1u << 3,
    ROTATING_DQ_ZERO_TRIP_IW = 1u << 4,
    ROTATING_DQ_ZERO_TRIP_PHASE_METRIC = 1u << 5,
    ROTATING_DQ_ZERO_TRIP_ID = 1u << 6,
    ROTATING_DQ_ZERO_TRIP_IQ = 1u << 7,
    ROTATING_DQ_ZERO_TRIP_DQ_METRIC = 1u << 8,
    ROTATING_DQ_ZERO_TRIP_PI_VD_NONZERO = 1u << 9,
    ROTATING_DQ_ZERO_TRIP_PI_VQ_NONZERO = 1u << 10,
    ROTATING_DQ_ZERO_TRIP_PI_INTEGRATOR_NONZERO = 1u << 11,
    ROTATING_DQ_ZERO_TRIP_VOLTAGE_SATURATION = 1u << 12,
    ROTATING_DQ_ZERO_TRIP_PWM_NOT_NEUTRAL = 1u << 13,
    ROTATING_DQ_ZERO_TRIP_ADC_PIPELINE = 1u << 14,
    ROTATING_DQ_ZERO_TRIP_NFAULT = 1u << 15,
    ROTATING_DQ_ZERO_TRIP_ANGLE_INVALID = 1u << 16,
    ROTATING_DQ_ZERO_TRIP_NAN_INF = 1u << 17
} RotatingDqZeroTripMask;

typedef enum {
    ROTATING_DQ_STATE_PREFLIGHT = 0,
    ROTATING_DQ_STATE_OFFSET_CAL,
    ROTATING_DQ_STATE_OFFSET_VERIFY,
    ROTATING_DQ_STATE_ENABLE_ZERO,
    ROTATING_DQ_STATE_RAMP_IQ_POSITIVE,
    ROTATING_DQ_STATE_HOLD_IQ_POSITIVE,
    ROTATING_DQ_STATE_RAMP_ZERO_1,
    ROTATING_DQ_STATE_HOLD_ZERO_1,
    ROTATING_DQ_STATE_RAMP_IQ_NEGATIVE,
    ROTATING_DQ_STATE_HOLD_IQ_NEGATIVE,
    ROTATING_DQ_STATE_RAMP_ZERO_2,
    ROTATING_DQ_STATE_HOLD_ZERO_2,
    ROTATING_DQ_STATE_COMPLETE,
    ROTATING_DQ_STATE_FAIL
} RotatingDqCurrentTestState;

typedef enum {
    ROTATING_DQ_RESULT_NOT_RUN = 0,
    ROTATING_DQ_RESULT_RUNNING,
    ROTATING_DQ_RESULT_PASS,
    ROTATING_DQ_RESULT_FAIL
} RotatingDqCurrentTestResult;

typedef enum {
    ROTATING_DQ_FAULT_NONE = 0u,
    ROTATING_DQ_FAULT_CURRENT_OFFSET_INVALID = 1u << 0,
    ROTATING_DQ_FAULT_ELECTRICAL_OFFSET_INVALID = 1u << 1,
    ROTATING_DQ_FAULT_PHASE_CURRENT_LIMIT = 1u << 2,
    ROTATING_DQ_FAULT_DQ_CURRENT_LIMIT = 1u << 3,
    ROTATING_DQ_FAULT_IQ_REF_LIMIT = 1u << 4,
    ROTATING_DQ_FAULT_ADC_SEQ_GAP = 1u << 5,
    ROTATING_DQ_FAULT_ADC_DUPLICATE = 1u << 6,
    ROTATING_DQ_FAULT_NFAULT = 1u << 7,
    ROTATING_DQ_FAULT_DRV = 1u << 8,
    ROTATING_DQ_FAULT_VBUS = 1u << 9,
    ROTATING_DQ_FAULT_ENCODER = 1u << 10,
    ROTATING_DQ_FAULT_ANGLE_JUMP = 1u << 11,
    ROTATING_DQ_FAULT_OVERSPEED = 1u << 12,
    ROTATING_DQ_FAULT_ONE_REV = 1u << 13,
    ROTATING_DQ_FAULT_DIRECTION = 1u << 14,
    ROTATING_DQ_FAULT_M1 = 1u << 15,
    ROTATING_DQ_FAULT_NAN_INF = 1u << 16,
    ROTATING_DQ_FAULT_PWM_CCR = 1u << 17,
    ROTATING_DQ_FAULT_TRACKING = 1u << 18,
    ROTATING_DQ_FAULT_SATURATION = 1u << 19,
    ROTATING_DQ_FAULT_CONTROL_TIME = 1u << 20,
    ROTATING_DQ_FAULT_ZERO_CURRENT_INVALID = 1u << 21,
    ROTATING_DQ_FAULT_CURRENT_SENSE_COMMON_MODE_EXCESS = 1u << 22
} RotatingDqCurrentTestFault;

typedef struct {
    bool admitted;
    int8_t candidate_sign;
    int8_t admitted_sign;
    uint8_t on_block_count;
    uint8_t off_block_count;
    uint32_t admit_count;
    uint32_t active_block_count;
    uint32_t first_admit_tick;
    uint32_t last_admit_tick;
    uint32_t sign_reversal_count;
    float mean_error_peak_a;
} RotatingDqBlockIntegratorAxis;

typedef struct {
    uint32_t block_tick_count;
    uint32_t valid_count;
    uint32_t completed_block_count;
    uint32_t invalid_block_count;
    float id_error_sum_a;
    float iq_error_sum_a;
    float last_id_mean_error_a;
    float last_iq_mean_error_a;
    RotatingDqBlockIntegratorAxis id_axis;
    RotatingDqBlockIntegratorAxis iq_axis;
} RotatingDqBlockIntegratorAdmission;

typedef struct {
    bool valid;
    RotatingDqCurrentTestState hold_state;
    uint32_t start_tick;
    uint32_t end_tick;
    uint32_t hold_sample_count;
    uint32_t tracking_start_tick;
    uint32_t tracking_end_tick;
    uint32_t tracking_sample_count;
    float iq_ref_sum_a;
    float iq_sum_a;
    float id_sum_a;
    float integrator_q_start_v;
    float integrator_q_end_v;
    float integrator_q_min_v;
    float integrator_q_max_v;
    uint32_t saturation_count;
    RotatingDqBlockIntegratorAdmission admission;
} RotatingDqBlockIntegratorHoldSnapshot;

typedef struct {
    float phase_resistance_ohm;
    float phase_inductance_h;
    float bandwidth_hz;
    float voltage_limit_v;
    float kaw;
    float integrator_limit_v;
    float dt_s;
    float iq_target_a;
    float iq_ramp_rate_a_per_s;
    float iq_ref_hard_limit_a;
    float phase_current_limit_a;
    float dq_current_limit_a;
    float zero_phase_current_peak_limit_a;
    float zero_dq_mean_limit_a;
    uint32_t zero_clean_sample_min;
    float speed_limit_rpm;
    int64_t one_rev_limit_counts;
    int64_t zero_encoder_limit_counts;
    float angle_jump_limit_rad;
    float tracking_error_limit_a;
    float tracking_iq_ref_mean_min_a;
    float tracking_iq_mean_min_a;
    float tracking_id_mean_abs_limit_a;
    uint32_t tracking_error_limit_ticks;
    uint32_t saturation_limit_ticks;
    uint32_t enable_zero_ticks;
    uint32_t zero_startup_guard_ticks;
    uint32_t iq_hold_ticks;
    uint32_t hold_zero_ticks;
    int64_t direction_capture_counts;
    uint32_t log_decimation;
    bool enable_zero_diagnostic_only;
    bool enable_zero_current_pi;
    /* Diagnostic-only D group: keep Kp/Ki configured but lock d/q integrators at zero. */
    bool freeze_zero_reference_integrator;
    bool enable_zero_block_integrator;
    /* External low-speed iq reference uses E2 block admission; speed PI stays outside. */
    bool enable_external_iq_block_integrator;
    /* Used only by the MOE-off encoder control experiment; fast protections stay active. */
    bool zero_window_observe_only;
    bool require_direction_match;
    bool single_direction_positive_only;
    uint32_t enable_zero_diagnostic_ticks;
    float vbus_min_v;
    float vbus_max_v;
    float control_time_limit_us;
    int32_t encoder_cpr;
    int32_t encoder_direction;
    int32_t pole_pairs;
} RotatingDqCurrentTestConfig;

typedef struct {
    uint32_t sample_count;
    float iv_mean_a;
    float iw_mean_a;
    float iu_mean_a;
    float id_mean_a;
    float iq_mean_a;
    float iv_std_a;
    float iw_std_a;
    float iu_std_a;
    float id_std_a;
    float iq_std_a;
    float phase_current_peak_a;
    bool adc_valid;
    bool nfault_ok;
    uint32_t fault_code;
} RotatingDqOffsetAdmission;

typedef struct {
    uint64_t time_us;
    uint32_t adc_seq;
    int64_t encoder_count;
    float theta_e_rad;
    float iv_a;
    float iw_a;
    float vbus_v;
    bool adc_valid;
    bool encoder_valid;
    bool theta_valid;
    bool electrical_offset_valid;
    bool nfault_ok;
    bool drv_ok;
    bool m1_safe;
    bool pwm_ccr_ok;
    bool pwm_allowed;
    bool fault_active;
    uint16_t raw_pc0;
    uint16_t raw_pc1;
    uint16_t offset_pc0;
    uint16_t offset_pc1;
    float current_amp_per_count;
    uint32_t ccr1;
    uint32_t ccr2;
    uint32_t ccr3;
    uint32_t ccr4;
    uint32_t ccer;
    uint32_t bdtr;
    uint32_t tim1_cnt;
    uint32_t adc_rank_order;
    uint32_t callback_cycles;
    float electrical_offset_rad;
    bool external_iq_ref_valid;
    bool external_integrator_enable;
    float external_iq_ref_a;
    bool compact_output_requested;
} RotatingDqCurrentTestInput;

typedef struct {
    RotatingDqCurrentTestState state;
    uint32_t control_tick_seq;
    uint32_t adc_seq;
    uint32_t voltage_command_seq;
    int64_t encoder_count;
    float theta_e_rad;
    float mechanical_speed_rpm;
    float id_ref_a;
    float iq_ref_a;
    float iu_a;
    float iv_a;
    float iw_a;
    float id_a;
    float iq_a;
    float vd_v;
    float vq_v;
    float v_alpha_v;
    float v_beta_v;
    float integrator_d_v;
    float integrator_q_v;
    bool saturation_active;
    float vbus_v;
    uint32_t fault_code;
} RotatingDqCurrentLogRecord;

typedef struct {
    uint32_t sample_index;
    uint32_t control_tick;
    uint32_t adc_seq;
    RotatingDqZeroDiagStage stage;
    bool post_shutdown_sample;
    uint16_t raw_pc0;
    uint16_t raw_pc1;
    uint16_t offset_pc0;
    uint16_t offset_pc1;
    int32_t delta_pc0;
    int32_t delta_pc1;
    int32_t iu_counts;
    int32_t iv_counts;
    int32_t iw_counts;
    float iu_a;
    float iv_a;
    float iw_a;
    float i_alpha_a;
    float i_beta_a;
    float id_a;
    float iq_a;
    float phase_metric_a;
    float dq_metric_a;
    float phase_limit_a;
    float dq_limit_a;
    bool phase_trip;
    bool dq_trip;
    uint32_t phase_trip_channel;
    uint32_t dq_trip_channel;
    int64_t encoder_count;
    float theta_mech_rad;
    float theta_electrical_raw_rad;
    float electrical_offset_rad;
    float theta_electrical_used_rad;
    float sin_theta;
    float cos_theta;
    float id_ref_a;
    float iq_ref_a;
    float id_error_a;
    float iq_error_a;
    float vd_p_v;
    float vd_i_v;
    float vq_p_v;
    float vq_i_v;
    float vd_unsat_v;
    float vq_unsat_v;
    float vd_applied_v;
    float vq_applied_v;
    float voltage_vector_sq;
    bool voltage_saturated;
    float v_alpha_v;
    float v_beta_v;
    uint32_t ccr1;
    uint32_t ccr2;
    uint32_t ccr3;
    uint32_t ccr4;
    uint32_t ccer;
    uint32_t bdtr;
    uint32_t tim1_cnt;
    uint32_t adc_rank_order;
    bool nfault_ok;
    uint32_t fast_core_cycles;
    uint32_t callback_cycles;
    uint32_t source_mask;
} RotatingDqZeroDiagSample;

typedef struct {
    bool valid;
    uint32_t source_line;
    uint32_t control_tick;
    uint32_t adc_seq;
    uint32_t source_mask;
    RotatingDqZeroDiagSample sample;
} RotatingDqZeroFirstTripSnapshot;

typedef struct {
    float iq_ref_sum_a;
    float iq_sum_a;
    float id_sum_a;
    float speed_sum_rpm;
    float iq_ref_mean_a;
    float iq_mean_a;
    float id_mean_a;
    float speed_mean_rpm;
    float speed_peak_rpm;
    int64_t encoder_delta_counts;
    int8_t mechanical_direction;
    float phase_current_peak_a;
    float voltage_vector_peak_sq;
    float voltage_vector_peak_v;
    uint32_t saturation_count;
    uint32_t sample_count;
} RotatingDqDirectionStats;

typedef struct {
    uint32_t total_max_cycles;
    uint32_t control_critical_max_cycles;
    uint32_t seq_input_max_cycles;
    uint32_t encoder_speed_max_cycles;
    uint32_t clarke_park_max_cycles;
    uint32_t current_protection_max_cycles;
    uint32_t controller_max_cycles;
    uint32_t log_stats_max_cycles;
    uint32_t state_machine_max_cycles;
    uint32_t fill_output_max_cycles;
} RotatingDqFastLoopProfile;

typedef struct {
    RotatingDqCurrentTestState state;
    RotatingDqCurrentTestResult result;
    uint32_t fault_code;
    float id_ref_a;
    float iq_ref_a;
    float vd_v;
    float vq_v;
    float v_alpha_v;
    float v_beta_v;
    float vd_diagnostic_v;
    float vq_diagnostic_v;
    float vd_proportional_v;
    float vq_proportional_v;
    float vd_integrator_v;
    float vq_integrator_v;
    float vd_feedforward_v;
    float vq_feedforward_v;
    float vd_unclamped_v;
    float vq_unclamped_v;
    float iu_measured_a;
    float iv_measured_a;
    float iw_measured_a;
    float id_measured_a;
    float iq_measured_a;
    float id_control_a;
    float iq_control_a;
    float id_error_a;
    float iq_error_a;
    float integrator_d_before_v;
    float integrator_q_before_v;
    float integrator_d_after_v;
    float integrator_q_after_v;
    float integrator_d_delta_v;
    float integrator_q_delta_v;
    float integrator_d_aw_clamp_delta_v;
    float integrator_q_aw_clamp_delta_v;
    bool common_mode_shape;
    bool common_mode_harmful;
    bool common_mode_caused_dq_crossing;
    bool power_stage_request;
    bool pwm_output_request;
    bool safe_shutdown_request;
    bool done;
} RotatingDqCurrentTestOutput;

typedef struct {
    RotatingDqCurrentTestConfig config;
    CurrentController controller;
    RotatingDqCurrentTestState state;
    RotatingDqCurrentTestResult result;
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
    bool have_last_encoder_count;
    bool have_last_theta;
    int64_t encoder_start_count;
    int64_t encoder_last_count;
    int64_t encoder_motion_max_counts;
    float theta_last_rad;
    float mechanical_speed_rpm;
    float speed_peak_rpm;
    float iq_ref_a;
    uint32_t tracking_error_ticks;
    uint32_t saturation_ticks;
    RotatingDqDirectionStats zero_stats;
    RotatingDqDirectionStats positive_stats;
    RotatingDqDirectionStats negative_stats;
    RotatingDqCurrentLogRecord log[ROTATING_DQ_CURRENT_TEST_LOG_CAPACITY];
    uint32_t log_count;
    uint32_t log_dropped;
    RotatingDqZeroDiagSample zero_diag[ROTATING_DQ_ZERO_DIAG_CAPACITY];
    uint32_t zero_diag_count;
    uint32_t zero_diag_dropped;
    RotatingDqZeroFirstTripSnapshot zero_first_trip;
    uint32_t zero_phase_fault_set_tick;
    uint32_t zero_dq_fault_set_tick;
    uint32_t zero_phase_startup_over_limit_count;
    uint32_t zero_phase_startup_over_limit_last_tick;
    float zero_phase_startup_over_limit_max_a;
    uint32_t zero_dq_startup_over_limit_count;
    uint32_t zero_dq_startup_over_limit_last_tick;
    float zero_dq_startup_over_limit_max_a;
    uint32_t zero_phase_over_limit_consecutive;
    uint32_t zero_phase_over_limit_consecutive_max;
    uint32_t zero_dq_over_limit_consecutive;
    uint32_t zero_dq_over_limit_consecutive_max;
    uint32_t zero_common_mode_shift_count;
    uint32_t zero_common_mode_harmless_count;
    uint32_t zero_common_mode_max_counts;
    uint32_t zero_common_mode_diff_max_counts;
    float zero_measured_phase_metric_max_a;
    float zero_reconstructed_phase_metric_max_a;
    float zero_clean_phase_metric_max_a;
    uint32_t zero_direct_metric_peak_tick;
    uint32_t zero_reconstructed_metric_peak_tick;
    uint32_t zero_clean_metric_peak_tick;
    uint32_t zero_clean_sample_count;
    uint32_t zero_common_mode_excluded_count;
    uint32_t zero_common_mode_phase_exclusion_count;
    uint32_t zero_common_mode_dq_exclusion_count;
    float zero_counterfactual_phase_metric_max_a;
    float zero_raw_dq_metric_max_a;
    float zero_counterfactual_dq_metric_max_a;
    uint32_t zero_startup_direct_outlier_count;
    uint32_t zero_startup_first_outlier_tick;
    uint32_t zero_startup_last_outlier_tick;
    int32_t zero_startup_pc0_peak_delta_counts;
    int32_t zero_startup_pc1_peak_delta_counts;
    float zero_startup_direct_metric_max_a;
    float zero_startup_reconstructed_metric_max_a;
    float zero_startup_dq_metric_max_a;
    uint32_t zero_fault_bit_order;
    uint32_t zero_reconstruction_formula_mismatch_count;
    uint32_t zero_reconstruction_scale_mismatch_count;
    uint32_t zero_stale_snapshot_count;
    uint32_t zero_clarke_transform_mismatch_count;
    uint32_t zero_park_transform_mismatch_count;
    uint32_t zero_dq_norm_amplification_count;
    float zero_phase_sum_error_max_a;
    bool zero_diag_completed;
    RotatingDqFastLoopProfile fast_profile;
} RotatingDqCurrentTest;

RotatingDqCurrentTestConfig rotating_dq_current_test_default_config(void);
void rotating_dq_current_test_init(RotatingDqCurrentTest *test,
                                   const RotatingDqCurrentTestConfig *config);
void rotating_dq_current_test_request_start(RotatingDqCurrentTest *test,
                                            bool electrical_offset_valid);
void rotating_dq_current_test_fast_isr(RotatingDqCurrentTest *test,
                                       const RotatingDqCurrentTestInput *input,
                                       RotatingDqCurrentTestOutput *output);
void rotating_dq_current_test_service_main(const RotatingDqCurrentTest *test,
                                           RotatingDqCurrentTestOutput *output);
void rotating_dq_current_test_force_fault(RotatingDqCurrentTest *test,
                                          uint32_t fault_code);
void rotating_dq_current_test_force_complete(RotatingDqCurrentTest *test);
void rotating_dq_current_test_note_execution_time(RotatingDqCurrentTest *test,
                                                  uint32_t cycles,
                                                  float time_us);
uint32_t rotating_dq_current_test_supervisor_timeout_ms(
    const RotatingDqCurrentTestConfig *config,
    uint32_t margin_ms);
void rotating_dq_current_test_capture_zero_diag_sample(
    RotatingDqCurrentTest *test,
    const RotatingDqCurrentTestInput *input,
    RotatingDqZeroDiagStage stage,
    bool post_shutdown_sample,
    uint32_t fast_core_cycles);
const char *rotating_dq_zero_diag_stage_name(RotatingDqZeroDiagStage stage);
const char *rotating_dq_zero_diag_classification(const RotatingDqCurrentTest *test);
bool rotating_dq_current_test_offset_admission_ok(
    const RotatingDqCurrentTestConfig *config,
    const RotatingDqOffsetAdmission *admission);
void rotating_dq_block_integrator_reset(
    RotatingDqBlockIntegratorAdmission *admission);
const RotatingDqBlockIntegratorAdmission *
rotating_dq_block_integrator_diagnostic_state(void);
const RotatingDqBlockIntegratorHoldSnapshot *
rotating_dq_block_integrator_positive_hold_snapshot(void);
const RotatingDqBlockIntegratorHoldSnapshot *
rotating_dq_block_integrator_negative_hold_snapshot(void);
bool rotating_dq_block_integrator_step(
    RotatingDqBlockIntegratorAdmission *admission,
    uint32_t control_tick,
    float id_error_a,
    float iq_error_a,
    bool sample_valid,
    float ki_v_per_a_s,
    float dt_s,
    float *id_integrator_delta_v,
    float *iq_integrator_delta_v);
float rotating_dq_current_test_theta_from_count(
    const RotatingDqCurrentTestConfig *config,
    int64_t encoder_count,
    float electrical_offset_rad);
int16_t rotating_dq_current_test_encoder_delta_u16(uint16_t now, uint16_t last);
const char *rotating_dq_current_test_state_name(RotatingDqCurrentTestState state);
const char *rotating_dq_current_test_result_name(RotatingDqCurrentTestResult result);

int rotating_dq_velocity_iq_sign_candidate(const RotatingDqCurrentTest *test);
float rotating_dq_current_test_kp(const RotatingDqCurrentTest *test);
float rotating_dq_current_test_ki(const RotatingDqCurrentTest *test);
float rotating_dq_current_test_ki_times_ts(const RotatingDqCurrentTest *test);

#ifdef __cplusplus
}
#endif

#endif /* ROTATING_DQ_CURRENT_TEST_H */

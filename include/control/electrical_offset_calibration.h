#ifndef ELECTRICAL_OFFSET_CALIBRATION_H
#define ELECTRICAL_OFFSET_CALIBRATION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELECTRICAL_OFFSET_CAL_POINT_COUNT 3u

typedef enum {
    ELECTRICAL_OFFSET_CAL_FAIL_NONE = 0u,
    ELECTRICAL_OFFSET_CAL_FAIL_POINT_INVALID = 1u << 0,
    ELECTRICAL_OFFSET_CAL_FAIL_SPREAD = 1u << 1,
    ELECTRICAL_OFFSET_CAL_FAIL_DIRECTION = 1u << 2,
    ELECTRICAL_OFFSET_CAL_FAIL_POLE_PAIRS = 1u << 3,
    ELECTRICAL_OFFSET_CAL_FAIL_VOLTAGE_LIMIT = 1u << 4
} ElectricalOffsetCalibrationFail;

typedef struct {
    int32_t encoder_cpr;
    int32_t encoder_direction;
    int32_t pole_pairs;
    float max_offset_spread_deg;
    float expected_delta_tolerance_counts;
    float alignment_voltage_limit_v;
} ElectricalOffsetCalibrationConfig;

typedef struct {
    float theta_cmd_rad;
    float encoder_count_mean;
    bool valid;
} ElectricalOffsetCalibrationPoint;

typedef struct {
    ElectricalOffsetCalibrationPoint points[ELECTRICAL_OFFSET_CAL_POINT_COUNT];
    float point_offset_rad[ELECTRICAL_OFFSET_CAL_POINT_COUNT];
    float electrical_offset_rad;
    float electrical_offset_deg;
    float max_offset_spread_deg;
    float delta_count_0_to_pos120;
    float delta_count_0_to_neg120;
    float expected_delta_count;
    float pole_pairs_est_pos;
    float pole_pairs_est_neg;
    float pole_pairs_est_average;
    uint32_t fail_flags;
    bool encoder_direction_ok;
    bool pole_pairs_ok;
    bool electrical_offset_valid;
} ElectricalOffsetCalibrationResult;

typedef enum {
    ELECTRICAL_OFFSET_STATE_IDLE = 0,
    ELECTRICAL_OFFSET_STATE_PREFLIGHT_INIT,
    ELECTRICAL_OFFSET_STATE_PREFLIGHT_ADMISSION,
    ELECTRICAL_OFFSET_STATE_PREFLIGHT_HANDOFF,
    ELECTRICAL_OFFSET_STATE_ALIGNMENT_DRV_PRE_ENABLE,
    ELECTRICAL_OFFSET_STATE_ALIGNMENT_DRV_WAIT_NFAULT,
    ELECTRICAL_OFFSET_STATE_ALIGNMENT_DRV_SETTLE,
    ELECTRICAL_OFFSET_STATE_ALIGNMENT_DRV_CONFIGURE,
    ELECTRICAL_OFFSET_STATE_ALIGNMENT_DRV_VERIFY,
    ELECTRICAL_OFFSET_STATE_ALIGNMENT_CURRENT_OFFSET_REVALIDATE,
    ELECTRICAL_OFFSET_STATE_ALIGNMENT_ZERO_COMMAND_REQUEST,
    ELECTRICAL_OFFSET_STATE_ALIGNMENT_ZERO_COMMAND_WAIT_ACK,
    ELECTRICAL_OFFSET_STATE_ALIGNMENT_ZERO_COMMAND_SETTLE,
    ELECTRICAL_OFFSET_STATE_ALIGNMENT_PRE_GATE_CHECK,
    ELECTRICAL_OFFSET_STATE_ALIGN_POINT_0_RAMP,
    ELECTRICAL_OFFSET_STATE_ALIGN_POINT_0_HOLD,
    ELECTRICAL_OFFSET_STATE_ALIGN_POINT_0_SAMPLE,
    ELECTRICAL_OFFSET_STATE_ALIGN_POINT_0_RAMP_DOWN,
    ELECTRICAL_OFFSET_STATE_ALIGN_POINT_POS120_RAMP,
    ELECTRICAL_OFFSET_STATE_ALIGN_POINT_POS120_HOLD,
    ELECTRICAL_OFFSET_STATE_ALIGN_POINT_POS120_SAMPLE,
    ELECTRICAL_OFFSET_STATE_ALIGN_POINT_POS120_RAMP_DOWN,
    ELECTRICAL_OFFSET_STATE_ALIGN_POINT_NEG120_RAMP,
    ELECTRICAL_OFFSET_STATE_ALIGN_POINT_NEG120_HOLD,
    ELECTRICAL_OFFSET_STATE_ALIGN_POINT_NEG120_SAMPLE,
    ELECTRICAL_OFFSET_STATE_ALIGN_POINT_NEG120_RAMP_DOWN,
    ELECTRICAL_OFFSET_STATE_CALCULATE_OFFSET,
    ELECTRICAL_OFFSET_STATE_COMPLETE,
    ELECTRICAL_OFFSET_STATE_FAIL
} ElectricalOffsetBringupState;

typedef enum {
    ELECTRICAL_OFFSET_FAILURE_NONE = 0,
    ELECTRICAL_OFFSET_FAILURE_ADMISSION_REJECTED,
    ELECTRICAL_OFFSET_FAILURE_ADMISSION_DEADLINE_MISS,
    ELECTRICAL_OFFSET_FAILURE_ADMISSION_HANDOFF_FAILED,
    ELECTRICAL_OFFSET_FAILURE_HOOK_OVERLAP,
    ELECTRICAL_OFFSET_FAILURE_ADC_PIPELINE_INVALID,
    ELECTRICAL_OFFSET_FAILURE_DRV_RUNTIME_FAULT,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_OVERCURRENT,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_OVERSPEED,
    ELECTRICAL_OFFSET_FAILURE_ENCODER_DELTA_INVALID,
    ELECTRICAL_OFFSET_FAILURE_SPREAD_INVALID,
    ELECTRICAL_OFFSET_FAILURE_CALLBACK_OVERRUN,
    ELECTRICAL_OFFSET_FAILURE_STATE_TIMEOUT,
    ELECTRICAL_OFFSET_FAILURE_STATE_TRANSITION_INVALID,
    ELECTRICAL_OFFSET_FAILURE_STALE_FAULT_LATCHED,
    ELECTRICAL_OFFSET_FAILURE_PRE_ALIGNMENT_GATE_FAILED,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_DISPATCH_NOT_READY,
    ELECTRICAL_OFFSET_FAILURE_DRV_NOT_READY,
    ELECTRICAL_OFFSET_FAILURE_OFFSETS_INVALID,
    ELECTRICAL_OFFSET_FAILURE_DC_CAL_NOT_CLEAR,
    ELECTRICAL_OFFSET_FAILURE_ZERO_COMMAND_NOT_ZERO,
    ELECTRICAL_OFFSET_FAILURE_ZERO_COMMAND_NOT_APPLIED,
    ELECTRICAL_OFFSET_FAILURE_ZERO_COMMAND_HANDOFF_TIMEOUT,
    ELECTRICAL_OFFSET_FAILURE_CCR_NOT_SAFE,
    ELECTRICAL_OFFSET_FAILURE_PENDING_VOLTAGE_UPDATE,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_RAW_PC0_OVERCURRENT,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_RAW_PC1_OVERCURRENT,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_PHASE_SOFT_OVERCURRENT,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_PHASE_HARD_OVERCURRENT,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_DQ_OVERCURRENT,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_ADC_SATURATION,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_OFFSET_SHIFT,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_VOLTAGE_SCALING_INVALID,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_CURRENT_MODEL_INVALID,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_CORE_OVERRUN,
    ELECTRICAL_OFFSET_FAILURE_ADC_CALLBACK_OVERRUN,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_TIMING_SCOPE_INVALID,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_PROFILE_OVERHEAD,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_INTERRUPT_PREEMPTED,
    ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_ENTRY_INIT_OVERRUN
} ElectricalOffsetFailure;

typedef struct {
    uint32_t alignment_fast_hook_core_cycles;
    uint32_t adc_callback_total_cycles;
    uint32_t alignment_state_transition_once_cycles;
    uint32_t main_service_cycles;
    uint32_t cpu_hz;
    bool scope_valid;
    bool detailed_timing_enabled;
    bool interrupt_preempted;
} ElectricalOffsetTimingInput;

typedef struct {
    bool pass;
    bool alignment_core_overrun;
    bool adc_callback_overrun;
    bool timing_scope_invalid;
    bool profile_overhead;
    bool interrupt_preempted;
    bool entry_init_overrun;
    bool old_main_scope_would_overrun;
    ElectricalOffsetFailure failure;
} ElectricalOffsetTimingVerdict;

enum {
    ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_RAW_PC0_HARD = 1u << 0,
    ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_RAW_PC1_HARD = 1u << 1,
    ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_RECONSTRUCTED_IU = 1u << 2,
    ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_RECONSTRUCTED_IV = 1u << 3,
    ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_RECONSTRUCTED_IW = 1u << 4,
    ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_PHASE_SOFT = 1u << 5,
    ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_PHASE_HARD = 1u << 6,
    ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_DQ = 1u << 7,
    ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_ADC_SATURATION = 1u << 8,
    ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_CURRENT_SENSOR_INVALID = 1u << 9,
    ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_CURRENT_OFFSET_INVALID = 1u << 10,
    ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_NFAULT = 1u << 11
};

typedef struct {
    uint32_t soft_consecutive_count;
    uint32_t hard_consecutive_count;
    uint32_t soft_consecutive_max;
    uint32_t hard_consecutive_max;
} ElectricalOffsetAlignmentProtectionState;

typedef struct {
    uint16_t raw_pc0;
    uint16_t raw_pc1;
    int32_t delta_pc0_counts;
    int32_t delta_pc1_counts;
    float iu_a;
    float iv_a;
    float iw_a;
    float id_a;
    float iq_a;
    float phase_abs_a;
    float current_amp_per_count;
    float soft_limit_a;
    float phase_emergency_limit_a;
    int32_t raw_hard_limit_counts;
    uint32_t soft_consecutive_required;
    uint16_t raw_min_safe_count;
    uint16_t raw_max_safe_count;
    bool offset_valid;
    bool current_finite;
    bool nfault_ok;
} ElectricalOffsetAlignmentProtectionInput;

typedef struct {
    bool trip;
    bool immediate_trip;
    bool soft_trip;
    uint32_t source_mask;
    uint32_t soft_consecutive_count;
    uint32_t hard_consecutive_count;
    float first_trip_current_a;
    const char *first_trip_channel;
    ElectricalOffsetFailure failure;
} ElectricalOffsetAlignmentProtectionResult;

typedef struct {
    bool admission_preflight_pass;
    bool admission_handoff_pass;
    bool admission_active;
    bool admission_disabled_ack;
    uint32_t snapshots_after_admission_disable;
    uint32_t admission_hook_calls_after_disable;
    uint32_t overlap_count;
    bool offset_pc0_valid;
    bool offset_pc1_valid;
    bool dc_cal_bits_clear;
    bool gate_enabled;
    bool nfault_asserted;
    uint32_t producer_gap_count;
    uint32_t producer_duplicate_count;
    uint32_t true_unpaired_count;
    uint32_t torn_count;
    uint32_t generation_mismatch_count;
    bool alignment_dispatch_enabled;
    bool alignment_active;
    bool command_flag_zero;
    bool v_alpha_zero;
    bool v_beta_zero;
    bool voltage_magnitude_zero;
    bool modulation_command_zero;
    bool last_applied_command_zero;
    bool voltage_command_pending_clear;
    bool voltage_command_seq_stable;
    bool pwm_shadow_safe;
    bool pwm_active_safe;
    bool ccr1_safe;
    bool ccr2_safe;
    bool ccr3_safe;
    bool ccr_safe_for_moe_off;
    bool ccr_safe_for_moe_enable;
    bool ccr_alignment_start_ready;
    bool moe_off;
    bool drv_ready;
} ElectricalOffsetPreAlignmentGateInput;

typedef struct {
    bool admission_pass;
    bool handoff_pass;
    bool offsets_pass;
    bool dc_cal_clear_pass;
    bool adc_pass;
    bool drv_pass;
    bool dispatch_pass;
    bool software_zero_command_pass;
    bool applied_zero_command_pass;
    bool command_flag_zero_pass;
    bool v_alpha_zero_pass;
    bool v_beta_zero_pass;
    bool voltage_magnitude_zero_pass;
    bool modulation_command_zero_pass;
    bool last_applied_command_zero_pass;
    bool voltage_command_pending_clear_pass;
    bool voltage_command_seq_stable_pass;
    bool pwm_shadow_safe_pass;
    bool pwm_active_safe_pass;
    bool ccr1_safe_pass;
    bool ccr2_safe_pass;
    bool ccr3_safe_pass;
    bool ccr_safe_for_moe_off_pass;
    bool ccr_safe_for_moe_enable_pass;
    bool ccr_alignment_start_ready_pass;
    bool zero_command_pass;
    bool moe_still_off_pass;
    bool pass;
    ElectricalOffsetFailure failure;
} ElectricalOffsetPreAlignmentGateResult;

typedef enum {
    ELECTRICAL_OFFSET_PWM_ZERO_CLASS_UNKNOWN = 0,
    ELECTRICAL_OFFSET_PWM_ZERO_CLASS_PWM_ACTIVE_OFFSET_SHIFT,
    ELECTRICAL_OFFSET_PWM_ZERO_CLASS_ADC_SWITCHING_EDGE_CONTAMINATION,
    ELECTRICAL_OFFSET_PWM_ZERO_CLASS_PWM_ZERO_STATE_INVALID,
    ELECTRICAL_OFFSET_PWM_ZERO_CLASS_CURRENT_RECONSTRUCTION_INVALID,
    ELECTRICAL_OFFSET_PWM_ZERO_CLASS_POSSIBLE_REAL_UNINTENDED_CURRENT
} ElectricalOffsetPwmZeroClassification;

typedef enum {
    ELECTRICAL_OFFSET_TRIGGER_SWEEP_INCONCLUSIVE = 0,
    ELECTRICAL_OFFSET_TRIGGER_SWEEP_TOO_CLOSE_TO_SWITCH_EDGE,
    ELECTRICAL_OFFSET_TRIGGER_SWEEP_COMMON_MODE_SETTLING_LONG,
    ELECTRICAL_OFFSET_TRIGGER_SWEEP_ACTIVE_OFFSET_INDEPENDENT,
    ELECTRICAL_OFFSET_TRIGGER_SWEEP_NO_VALID_LOW_SIDE_WINDOW,
    ELECTRICAL_OFFSET_TRIGGER_SWEEP_EVENT_CONFIG_INVALID
} ElectricalOffsetTriggerSweepClassification;

typedef struct {
    bool commanded_v_alpha_zero;
    bool commanded_v_beta_zero;
    bool applied_v_alpha_zero;
    bool applied_v_beta_zero;
    bool duty_u_eq_v;
    bool duty_v_eq_w;
    bool ccr1_eq_ccr2;
    bool ccr2_eq_ccr3;
    bool preload_ack;
    bool tim_updates_after_preload;
    bool pending_voltage_update_clear;
    bool dc_cal_bits_clear;
    bool nfault_ok;
    bool gate_enabled;
    bool moe_off_before_enable;
} ElectricalOffsetPwmZeroStartInput;

typedef struct {
    bool command_zero;
    bool duties_equal;
    bool ccrs_equal;
    bool preload_complete;
    bool safe_to_enable_moe;
} ElectricalOffsetPwmZeroStartResult;

typedef struct {
    bool ccrs_equal;
    bool line_to_line_zero;
    bool stable_pc0_pc1_shift;
    bool speed_near_zero;
    bool nfault_ok;
    bool near_switch_edge;
    bool raw_within_limits;
    bool reconstruction_consistent;
    bool current_ramp_like;
    bool encoder_motion;
} ElectricalOffsetPwmZeroClassifyInput;

typedef struct {
    uint32_t pclk2_hz;
    uint32_t apb2_prescaler;
    uint32_t tim_prescaler;
    uint32_t arr;
    uint32_t phase_ccr;
    uint32_t deadtime_counts;
} ElectricalOffsetTriggerSweepTimingInput;

typedef struct {
    uint32_t tim_input_clock_hz;
    uint32_t tim_counter_clock_hz;
    float timer_tick_ns;
    float pwm_frequency_hz;
    uint32_t low_side_window_start_count;
    uint32_t low_side_window_end_count;
    uint32_t low_side_window_width_counts;
    float low_side_window_width_us;
    uint32_t deadtime_counts;
    float deadtime_us;
} ElectricalOffsetTriggerSweepTiming;

typedef struct {
    uint32_t candidate_index;
    float requested_guard_time_us;
    uint32_t requested_guard_counts;
    uint32_t programmed_trigger_count;
    uint32_t trigger_direction;
    uint32_t distance_to_nearest_switch_edge_counts;
    float distance_to_nearest_switch_edge_us;
    uint32_t distance_to_deadtime_end_counts;
    bool expected_low_side_state;
    bool candidate_valid;
    bool is_baseline;
    bool is_quiet_center;
} ElectricalOffsetTriggerSweepCandidate;

typedef struct {
    bool candidate_valid;
    bool raw_hard_trip;
    bool emergency_trip;
    bool nfault_trip;
    bool producer_error;
    bool adc_saturation;
    float delta_pc0_mean_counts;
    float delta_pc1_mean_counts;
    float pc0_std_counts;
    float pc1_std_counts;
    float iu_mean_a;
    float iv_mean_a;
    float iw_mean_a;
    float phase_abs_peak_a;
    uint32_t maximum_soft_consecutive_count;
    int32_t encoder_delta_counts;
    float reconstructed_zero_current_error_a;
    float distance_to_edge_us;
    float center_distance_counts;
} ElectricalOffsetTriggerSweepCandidateResult;

ElectricalOffsetCalibrationConfig electrical_offset_calibration_default_config(void);
float electrical_offset_wrap_0_2pi(float x);
float electrical_offset_angle_distance_rad(float a, float b);
float electrical_offset_count_delta(float now, float reference, int32_t cpr);
const char *electrical_offset_state_name(ElectricalOffsetBringupState state);
const char *electrical_offset_failure_name(ElectricalOffsetFailure failure);
const char *electrical_offset_pwm_zero_classification_name(
    ElectricalOffsetPwmZeroClassification classification);
const char *electrical_offset_trigger_sweep_classification_name(
    ElectricalOffsetTriggerSweepClassification classification);
uint32_t electrical_offset_dwt_elapsed(uint32_t start, uint32_t end);
ElectricalOffsetTimingVerdict electrical_offset_timing_evaluate(
    const ElectricalOffsetTimingInput *input);
int electrical_offset_format_signed_milli(char *buf,
                                          size_t len,
                                          int32_t milli,
                                          bool always_sign);
ElectricalOffsetBringupState electrical_offset_handoff_next_state(bool handoff_pass,
                                                                  bool same_iteration);
ElectricalOffsetPreAlignmentGateResult electrical_offset_pre_alignment_gate_evaluate(
    const ElectricalOffsetPreAlignmentGateInput *input);
bool electrical_offset_calibration_evaluate(
    const ElectricalOffsetCalibrationConfig *config,
    const ElectricalOffsetCalibrationPoint points[ELECTRICAL_OFFSET_CAL_POINT_COUNT],
    float alignment_voltage_v,
    ElectricalOffsetCalibrationResult *result);
void electrical_offset_alignment_protection_reset(
    ElectricalOffsetAlignmentProtectionState *state);
ElectricalOffsetAlignmentProtectionResult electrical_offset_alignment_protection_update(
    ElectricalOffsetAlignmentProtectionState *state,
    const ElectricalOffsetAlignmentProtectionInput *input);
ElectricalOffsetPwmZeroStartResult electrical_offset_pwm_zero_start_evaluate(
    const ElectricalOffsetPwmZeroStartInput *input);
ElectricalOffsetPwmZeroClassification electrical_offset_pwm_zero_classify(
    const ElectricalOffsetPwmZeroClassifyInput *input);
int32_t electrical_offset_reconstructed_iu_counts(int32_t iv_counts,
                                                  int32_t iw_counts);
float electrical_offset_common_mode_shift_counts(float delta_pc0_counts,
                                                 float delta_pc1_counts);
float electrical_offset_differential_shift_counts(float delta_pc0_counts,
                                                  float delta_pc1_counts);
uint32_t electrical_offset_min_phase_edge_distance_counts(uint32_t sample_count,
                                                         uint32_t ccr1,
                                                         uint32_t ccr2,
                                                         uint32_t ccr3);
ElectricalOffsetTriggerSweepTiming electrical_offset_trigger_sweep_timing(
    const ElectricalOffsetTriggerSweepTimingInput *input);
uint32_t electrical_offset_trigger_sweep_generate_candidates(
    const ElectricalOffsetTriggerSweepTiming *timing,
    const float *guard_times_us,
    uint32_t guard_count,
    uint32_t baseline_trigger_count,
    ElectricalOffsetTriggerSweepCandidate *candidates,
    uint32_t max_candidates);
uint32_t electrical_offset_trigger_sweep_generate_fixed_candidates(
    const ElectricalOffsetTriggerSweepTiming *timing,
    uint32_t trigger_count,
    uint32_t repeat_count,
    ElectricalOffsetTriggerSweepCandidate *candidates,
    uint32_t max_candidates);
bool electrical_offset_trigger_sweep_candidate_result_valid(
    const ElectricalOffsetTriggerSweepCandidate *candidate,
    const ElectricalOffsetTriggerSweepCandidateResult *result);
int32_t electrical_offset_trigger_sweep_recommend_candidate(
    const ElectricalOffsetTriggerSweepCandidate *candidates,
    const ElectricalOffsetTriggerSweepCandidateResult *results,
    uint32_t count);
ElectricalOffsetTriggerSweepClassification
electrical_offset_trigger_sweep_classify(bool baseline_bad,
                                         bool improves_with_guard,
                                         bool any_valid,
                                         bool all_valid_shifted,
                                         bool event_config_invalid);
void electrical_offset_alpha_beta_to_phase(float v_alpha,
                                           float v_beta,
                                           float *vu,
                                           float *vv,
                                           float *vw,
                                           float *max_line_to_line);
float electrical_offset_expected_phase_current_phase_resistance(
    float alpha_beta_voltage_v,
    float phase_resistance_ohm);
float electrical_offset_expected_phase_current_line_line_resistance(
    float alpha_beta_voltage_v,
    float line_line_resistance_ohm);

#ifdef __cplusplus
}
#endif

#endif /* ELECTRICAL_OFFSET_CALIBRATION_H */

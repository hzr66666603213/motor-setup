#ifndef CURRENT_SENSOR_ADMISSION_H
#define CURRENT_SENSOR_ADMISSION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CURRENT_SENSOR_ADMISSION_IDLE = 0,
    CURRENT_SENSOR_ADMISSION_WAIT_DC_CAL_ENABLE,
    CURRENT_SENSOR_ADMISSION_WAIT_DC_CAL_DISCARD,
    CURRENT_SENSOR_ADMISSION_COLLECT_DC_CAL,
    CURRENT_SENSOR_ADMISSION_WAIT_POST_CAL_ACK,
    CURRENT_SENSOR_ADMISSION_WAIT_POST_CAL_DISCARD,
    CURRENT_SENSOR_ADMISSION_COLLECT_POST_CAL,
    CURRENT_SENSOR_ADMISSION_COMPLETE,
    CURRENT_SENSOR_ADMISSION_FAIL
} CurrentSensorAdmissionState;

typedef enum {
    CURRENT_SENSOR_ADMISSION_FAIL_NONE = 0,
    CURRENT_SENSOR_ADMISSION_FAIL_ADC_SEQ_GAP = 1u << 0,
    CURRENT_SENSOR_ADMISSION_FAIL_ADC_DUPLICATE = 1u << 1,
    CURRENT_SENSOR_ADMISSION_FAIL_ADC_UNPAIRED = 1u << 2,
    CURRENT_SENSOR_ADMISSION_FAIL_TORN = 1u << 3,
    CURRENT_SENSOR_ADMISSION_FAIL_GENERATION_MISMATCH = 1u << 4,
    CURRENT_SENSOR_ADMISSION_FAIL_NFAULT = 1u << 5,
    CURRENT_SENSOR_ADMISSION_FAIL_RAW_SATURATION = 1u << 6,
    CURRENT_SENSOR_ADMISSION_FAIL_RAW_OUTLIER = 1u << 7,
    CURRENT_SENSOR_ADMISSION_FAIL_PHASE_OUTLIER = 1u << 8,
    CURRENT_SENSOR_ADMISSION_FAIL_MEAN = 1u << 9,
    CURRENT_SENSOR_ADMISSION_FAIL_STD = 1u << 10,
    CURRENT_SENSOR_ADMISSION_FAIL_CONSECUTIVE = 1u << 11,
    CURRENT_SENSOR_ADMISSION_FAIL_SAMPLE_COUNT = 1u << 12,
    CURRENT_SENSOR_ADMISSION_FAIL_DC_CAL_ACK_TIMEOUT = 1u << 13,
    CURRENT_SENSOR_ADMISSION_FAIL_POST_CAL_ACK_TIMEOUT = 1u << 14
} CurrentSensorAdmissionFailFlags;

typedef struct {
    uint16_t dc_cal_discard_samples;
    uint16_t dc_cal_collect_samples;
    uint16_t post_discard_samples;
    uint16_t post_collect_samples;
    float current_amp_per_count;
    float std_limit_counts;
    bool recenter_live_zero_offset;
    uint16_t live_zero_shift_limit_counts;
    uint16_t raw_adc_min;
    uint16_t raw_adc_max;
    uint16_t hard_delta_limit_counts;
    uint16_t hard_phase_limit_counts;
    uint16_t above_6_limit;
    uint16_t above_8_limit;
    uint16_t above_10_limit;
} CurrentSensorAdmissionConfig;

typedef struct {
    uint16_t raw_pc0;
    uint16_t raw_pc1;
    uint32_t adc_seq;
    bool snapshot_valid;
    bool nfault_raw_high;
    bool adc_true_unpaired;
    bool adc_torn;
    bool adc_generation_mismatch;
} CurrentSensorAdmissionFastInput;

typedef struct {
    bool request_enable_dc_cal;
    bool request_disable_dc_cal;
    bool complete;
    bool failed;
} CurrentSensorAdmissionMainAction;

typedef struct {
    uint32_t count;
    uint32_t sum0;
    uint32_t sum1;
    uint16_t min0;
    uint16_t max0;
    uint16_t min1;
    uint16_t max1;
} CurrentSensorAdmissionRawAccumulator;

typedef struct {
    uint32_t count;
    int64_t sum0;
    int64_t sum1;
    int64_t sum_u;
    int64_t sum0_sq;
    int64_t sum1_sq;
    int16_t min0;
    int16_t max0;
    int16_t min1;
    int16_t max1;
    uint16_t max_abs_delta_pc0;
    uint16_t max_abs_delta_pc1;
    uint16_t phase_metric_max_counts;
    uint16_t current_above_6;
    uint16_t current_above_8;
    uint16_t current_above_10;
    uint16_t longest_above_6;
    uint16_t longest_above_8;
    uint16_t longest_above_10;
} CurrentSensorAdmissionPostAccumulator;

typedef struct {
    uint16_t dc_cal_offset_pc0;
    uint16_t dc_cal_offset_pc1;
    uint16_t live_zero_offset_pc0;
    uint16_t live_zero_offset_pc1;
    int16_t live_zero_shift_pc0_counts;
    int16_t live_zero_shift_pc1_counts;
    uint32_t dc_cal_samples;
    uint32_t post_samples;
    uint32_t dc_cal_discard_count;
    uint32_t post_cal_discard_count;
    uint32_t wait_dc_cal_enable_snapshot_count;
    uint32_t wait_post_cal_ack_snapshot_count;
    uint32_t transition_wait_snapshot_count;
    uint32_t observed_snapshot_count;
    uint32_t scheduled_snapshot_count;
    uint32_t analysis_snapshot_count;
    uint32_t discard_snapshot_count;
    uint32_t total_snapshot_count;
    uint32_t producer_first_seq;
    uint32_t producer_last_seq;
    uint32_t producer_seq_span;
    uint32_t dc_cal_collect_last_seq;
    uint32_t post_cal_ack_observed_seq;
    uint32_t post_cal_discard_first_seq;
    uint16_t dc_cal_pc0_min;
    uint16_t dc_cal_pc0_max;
    uint16_t dc_cal_pc1_min;
    uint16_t dc_cal_pc1_max;
    float dc_cal_pc0_mean;
    float dc_cal_pc1_mean;
    float iv_mean_a;
    float iw_mean_a;
    float iu_mean_a;
    float pc0_std_counts;
    float pc1_std_counts;
    uint16_t max_abs_delta_pc0;
    uint16_t max_abs_delta_pc1;
    uint16_t phase_metric_max_counts;
    uint16_t longest_above_6_counts;
    uint16_t longest_above_8_counts;
    uint16_t longest_above_10_counts;
    uint32_t producer_gap_count;
    uint32_t producer_duplicate_count;
    uint32_t runtime_true_unpaired_count;
    uint32_t torn_count;
    uint32_t generation_mismatch_count;
    uint32_t nfault_runtime_asserted_count;
    bool mean_pass;
    bool std_pass;
    bool consecutive_pass;
    bool hard_outlier_pass;
    bool adc_pipeline_pass;
    bool drv_runtime_pass;
    bool admission_pass;
    uint32_t fail_flags;
    uint32_t worst_seq_check_cycles;
    uint32_t worst_raw_accumulate_cycles;
    uint32_t worst_phase_check_cycles;
    uint32_t worst_total_cycles;
    uint32_t worst_adc_callback_with_admission_cycles;
} CurrentSensorAdmissionResult;

typedef struct {
    bool target_5us_met;
    bool deadline_pass;
    bool functional_pass;
    bool preflight_pass;
} CurrentSensorAdmissionPreflightVerdict;

typedef struct {
    volatile CurrentSensorAdmissionState state;
    CurrentSensorAdmissionConfig cfg;
    CurrentSensorAdmissionRawAccumulator dc;
    CurrentSensorAdmissionPostAccumulator post;
    CurrentSensorAdmissionResult result;
    uint32_t last_seq;
    bool seq_initialized;
    bool offsets_valid;
    bool finalized;
    bool run_once_locked;
} CurrentSensorAdmission;

CurrentSensorAdmissionConfig current_sensor_admission_default_config(void);
void current_sensor_admission_init(CurrentSensorAdmission *adm,
                                   const CurrentSensorAdmissionConfig *cfg);
bool current_sensor_admission_request_start(CurrentSensorAdmission *adm);
CurrentSensorAdmissionMainAction
current_sensor_admission_service_main(CurrentSensorAdmission *adm);
bool current_sensor_admission_ack_dc_cal_enabled(CurrentSensorAdmission *adm);
bool current_sensor_admission_ack_dc_cal_disabled(CurrentSensorAdmission *adm);
void current_sensor_admission_fast_isr(CurrentSensorAdmission *adm,
                                       const CurrentSensorAdmissionFastInput *in);
void current_sensor_admission_finalize(CurrentSensorAdmission *adm);
CurrentSensorAdmissionResult
current_sensor_admission_get_result(const CurrentSensorAdmission *adm);
CurrentSensorAdmissionPreflightVerdict current_sensor_admission_evaluate_preflight(
    const CurrentSensorAdmissionResult *result,
    uint32_t worst_admission_cycles,
    uint32_t worst_adc_callback_cycles,
    uint32_t cpu_hz,
    bool sample_count_pass,
    bool adc_pipeline_pass,
    bool drv_runtime_pass,
    bool final_safe);
const char *current_sensor_admission_state_name(CurrentSensorAdmissionState state);

#ifdef __cplusplus
}
#endif

#endif

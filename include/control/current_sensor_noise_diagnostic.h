#ifndef CURRENT_SENSOR_NOISE_DIAGNOSTIC_H
#define CURRENT_SENSOR_NOISE_DIAGNOSTIC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CURRENT_SENSOR_NOISE_WORST_CAPACITY 16u
#define CURRENT_SENSOR_NOISE_DELTA_HIST_MIN (-32)
#define CURRENT_SENSOR_NOISE_DELTA_HIST_MAX 32
#define CURRENT_SENSOR_NOISE_DELTA_HIST_BINS 65u
#define CURRENT_SENSOR_NOISE_PHASE_HIST_BINS 401u

typedef struct {
    uint32_t sample_count;
    float mean;
    float standard_deviation;
    int16_t min;
    int16_t max;
    int16_t peak_to_peak;
    float median;
    float mad;
    float percentile_90;
    float percentile_95;
    float percentile_99;
    float percentile_99_5;
    float percentile_99_9;
    float abs_percentile_99_9;
    uint32_t abs_over_2_count;
    uint32_t abs_over_4_count;
    uint32_t abs_over_6_count;
    uint32_t abs_over_8_count;
    uint32_t abs_over_10_count;
    uint32_t abs_over_12_count;
} CurrentSensorNoiseCountStats;

typedef struct {
    uint32_t sample_count;
    float mean;
    float standard_deviation;
    float min;
    float max;
    float abs_peak;
} CurrentSensorNoiseCurrentStats;

typedef struct {
    uint32_t sample_count;
    float phase_abs_peak;
    float phase_percentile_99;
    float phase_percentile_99_5;
    float phase_percentile_99_9;
    uint32_t samples_above_0p08A;
    uint32_t samples_above_0p12A;
    uint32_t samples_above_0p16A;
    uint32_t samples_above_0p20A;
    uint32_t longest_consecutive_above_0p08A;
    uint32_t longest_consecutive_above_0p12A;
    uint32_t longest_consecutive_above_0p16A;
    uint32_t longest_consecutive_above_0p20A;
} CurrentSensorNoisePhaseStats;

typedef struct {
    uint32_t sample_index;
    uint32_t adc_seq;
    uint16_t raw_pc0;
    uint16_t raw_pc1;
    int16_t delta_pc0;
    int16_t delta_pc1;
    int16_t prev_delta_pc0;
    int16_t prev_delta_pc1;
    int16_t next_delta_pc0;
    int16_t next_delta_pc1;
    float iv;
    float iw;
    float iu;
    float id;
    float iq;
    float phase_abs;
    uint16_t tim1_cnt;
    uint8_t adc_rank_order;
    uint32_t callback_count;
} CurrentSensorNoiseWorstSample;

typedef struct {
    uint32_t count;
    int64_t sum;
    int64_t sum_square;
    int16_t min;
    int16_t max;
    uint32_t histogram[CURRENT_SENSOR_NOISE_DELTA_HIST_BINS];
    uint32_t underflow_count;
    uint32_t overflow_count;
} CurrentSensorNoiseOnlineCountAccumulator;

typedef struct {
    uint32_t count;
    int64_t sum;
    int64_t sum_square;
    int16_t min;
    int16_t max;
    uint16_t abs_peak;
    float scale;
} CurrentSensorNoiseOnlineCurrentAccumulator;

typedef struct {
    uint32_t count;
    uint16_t phase_abs_peak_counts;
    uint32_t phase_histogram[CURRENT_SENSOR_NOISE_PHASE_HIST_BINS];
    uint32_t phase_histogram_overflow_count;
    uint32_t samples_above_0p08A;
    uint32_t samples_above_0p12A;
    uint32_t samples_above_0p16A;
    uint32_t samples_above_0p20A;
    uint32_t longest_consecutive_above_0p08A;
    uint32_t longest_consecutive_above_0p12A;
    uint32_t longest_consecutive_above_0p16A;
    uint32_t longest_consecutive_above_0p20A;
    uint32_t current_consecutive_above_0p08A;
    uint32_t current_consecutive_above_0p12A;
    uint32_t current_consecutive_above_0p16A;
    uint32_t current_consecutive_above_0p20A;
} CurrentSensorNoiseOnlinePhaseAccumulator;

typedef struct {
    CurrentSensorNoiseOnlineCountAccumulator pc0;
    CurrentSensorNoiseOnlineCountAccumulator pc1;
    CurrentSensorNoiseOnlineCurrentAccumulator iv;
    CurrentSensorNoiseOnlineCurrentAccumulator iw;
    CurrentSensorNoiseOnlineCurrentAccumulator iu;
    CurrentSensorNoiseOnlineCurrentAccumulator id;
    CurrentSensorNoiseOnlineCurrentAccumulator iq;
    CurrentSensorNoiseOnlinePhaseAccumulator phase;
    CurrentSensorNoiseWorstSample worst[CURRENT_SENSOR_NOISE_WORST_CAPACITY];
    uint32_t worst_count;
    uint32_t worst_min_index;
    uint16_t worst_min_phase_metric_counts;
    uint16_t threshold_0p08_counts;
    uint16_t threshold_0p12_counts;
    uint16_t threshold_0p16_counts;
    uint16_t threshold_0p20_counts;
    bool pending_next_valid[CURRENT_SENSOR_NOISE_WORST_CAPACITY];
} CurrentSensorNoiseOnlineAccumulator;

typedef struct {
    CurrentSensorNoiseCountStats pc0;
    CurrentSensorNoiseCountStats pc1;
    CurrentSensorNoiseCurrentStats iv;
    CurrentSensorNoiseCurrentStats iw;
    CurrentSensorNoiseCurrentStats iu;
    CurrentSensorNoiseCurrentStats id;
    CurrentSensorNoiseCurrentStats iq;
    CurrentSensorNoisePhaseStats phase;
    CurrentSensorNoiseWorstSample worst[CURRENT_SENSOR_NOISE_WORST_CAPACITY];
    uint32_t worst_count;
} CurrentSensorNoiseAnalysis;

typedef struct {
    uint32_t total_adc_snapshot_count;
    uint32_t coherent_snapshot_count;
    uint32_t adc1_injected_complete_count;
    uint32_t adc2_injected_complete_count;
    uint32_t coherent_snapshot_publish_count;
    uint32_t diagnostic_expected_snapshot_seen;
    uint32_t diagnostic_analysis_sample_count;
    uint32_t diagnostic_discard_sample_count;
    uint32_t diagnostic_adc_seq_first;
    uint32_t diagnostic_adc_seq_last;
    uint32_t diagnostic_adc_seq_span;
    float adc_sync_rate;
    uint32_t missed_adc_seq;
    uint32_t duplicate_adc_seq;
    uint32_t torn_snapshot_count;
    uint32_t adc1_complete_without_adc2;
    uint32_t adc2_complete_without_adc1;
    uint32_t maximum_adc1_adc2_completion_gap_cycles;
    uint32_t worst_snapshot_publish_cycles;
    uint32_t worst_noise_diagnostic_isr_cycles;
    uint32_t worst_adc_callback_cycles;
    uint32_t max_same_generation_completion_gap_cycles;
    uint32_t max_boundary_completion_gap_cycles;
    uint32_t completion_gap_generation_mismatch_count;
    uint32_t boundary_adc1_pending_at_start;
    uint32_t boundary_adc2_pending_at_start;
    uint32_t boundary_adc1_pending_at_end;
    uint32_t boundary_adc2_pending_at_end;
    uint32_t post_freeze_adc1_completion_count;
    uint32_t post_freeze_adc2_completion_count;
    uint32_t consumer_samples_read;
    uint32_t consumer_skipped_snapshot_count;
    float consumer_coverage_ratio;
} CurrentSensorNoiseAdcIntegrity;

typedef struct {
    bool initialized;
    uint32_t first_seq;
    uint32_t last_seq;
    uint32_t observed_count;
    uint32_t gap_count;
    uint32_t duplicate_count;
} CurrentSensorNoiseSeqTracker;

typedef struct {
    bool raw_pin_high;
    bool asserted;
    bool ok;
} CurrentSensorNoiseNfaultSemantics;

typedef struct {
    bool diagnostic_data_valid;
    bool adc_hardware_pipeline_valid;
    bool adc_publish_sequence_valid;
    bool consumer_coverage_valid;
    bool drv_runtime_valid;
    bool noise_profile_valid;
    bool zero_current_admission_candidate;
} CurrentSensorNoiseDiagnosticVerdict;

typedef struct {
    bool mean_ok;
    bool base_noise_ok;
    bool high_tail_ok;
    bool consecutive_ok;
    bool hard_outlier_ok;
    bool candidate_ok;
    float high_tail_pc0_value_counts;
    float high_tail_pc1_value_counts;
    float high_tail_limit_counts;
    bool high_tail_pc0_pass;
    bool high_tail_pc1_pass;
} CurrentSensorNoiseAdmissionResult;

typedef struct {
    bool instantaneous_hard_fault;
    bool sustained_soft_fault;
    uint32_t consecutive_count;
    float filtered_or_window_peak;
} CurrentSensorNoiseTransientResult;

typedef struct {
    float cutoff_hz;
    float rms_reduction_ratio;
    float phase_delay_us_at_200hz;
    float snr_0p10a_after_filter;
} CurrentSensorNoiseLowpassEstimate;

float current_sensor_noise_percentile_i16(const int16_t *values,
                                          uint32_t count,
                                          float percentile);
float current_sensor_noise_median_i16(const int16_t *values, uint32_t count);
float current_sensor_noise_mad_i16(const int16_t *values, uint32_t count);

void current_sensor_noise_analyze(const int16_t *delta_pc0,
                                  const int16_t *delta_pc1,
                                  const uint16_t *raw_pc0,
                                  const uint16_t *raw_pc1,
                                  const uint32_t *adc_seq,
                                  const uint16_t *tim1_cnt,
                                  const uint32_t *callback_count,
                                  uint8_t adc_rank_order,
                                  uint32_t count,
                                  float current_amp_per_count,
                                  CurrentSensorNoiseAnalysis *out);

void current_sensor_noise_seq_tracker_init(CurrentSensorNoiseSeqTracker *tracker);
void current_sensor_noise_seq_tracker_observe(CurrentSensorNoiseSeqTracker *tracker,
                                              uint32_t seq);
void current_sensor_noise_seq_tracker_to_integrity(
    const CurrentSensorNoiseSeqTracker *tracker,
    uint32_t expected_snapshot_count,
    uint32_t analysis_sample_count,
    uint32_t discard_sample_count,
    CurrentSensorNoiseAdcIntegrity *out);

CurrentSensorNoiseAdmissionResult
current_sensor_noise_evaluate_zero_current_admission(
    const CurrentSensorNoiseAnalysis *analysis);

CurrentSensorNoiseTransientResult
current_sensor_noise_classify_phase_abs(const float *phase_abs,
                                         uint32_t count,
                                         float sustained_threshold_a,
                                         uint32_t sustained_ticks,
                                         float instant_threshold_a);

CurrentSensorNoiseLowpassEstimate
current_sensor_noise_estimate_one_pole_lowpass(float sample_rate_hz,
                                               float cutoff_hz,
                                               float noise_std_a,
                                               float signal_a);

void current_sensor_noise_online_reset(CurrentSensorNoiseOnlineAccumulator *acc);
void current_sensor_noise_online_push(CurrentSensorNoiseOnlineAccumulator *acc,
                                      uint32_t sample_index,
                                      uint32_t adc_seq,
                                      uint16_t raw_pc0,
                                      uint16_t raw_pc1,
                                      int16_t delta_pc0,
                                      int16_t delta_pc1,
                                      int16_t prev_delta_pc0,
                                      int16_t prev_delta_pc1,
                                      uint16_t tim1_cnt,
                                      uint8_t adc_rank_order,
                                      uint32_t callback_count,
                                      float current_amp_per_count);
void current_sensor_noise_online_finalize(
    const CurrentSensorNoiseOnlineAccumulator *acc,
    CurrentSensorNoiseAnalysis *out);

bool current_sensor_noise_should_skip_power_for_low_gain(float gain_v_v);
CurrentSensorNoiseNfaultSemantics
current_sensor_noise_nfault_from_raw(bool raw_pin_high);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_SENSOR_NOISE_DIAGNOSTIC_H */

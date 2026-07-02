/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app/axis0_types.h"
#include "board/board_odrive_v36.h"
#include "config/axis0_default_config.h"
#include "drivers/drv8301.h"
#include "foc/svpwm.h"
#include "hal/hal_adc.h"
#include "hal/hal_gpio.h"
#include "hal/hal_pwm.h"
#include "protection/fault.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint32_t samples;
  uint32_t offset_u;
  uint32_t offset_v;
  uint16_t u_min;
  uint16_t u_max;
  uint16_t v_min;
  uint16_t v_max;
  uint16_t u_noise_pp;
  uint16_t v_noise_pp;
} AdcOffsetStats;

typedef struct {
  uint32_t elapsed_ms;
  uint32_t state;
  uint32_t test_point_index;
  uint32_t seq;
  float theta_e;
  float theta_m;
  float electrical_offset_runtime;
  float vd;
  float vq;
  float iu;
  float iv;
  float iw;
  float id;
  float iq;
  float mechanical_rpm_filtered;
  float vbus_v;
  uint16_t tim3_cnt;
  int16_t encoder_delta;
  int64_t encoder_accum;
  uint16_t raw_u;
  uint16_t raw_v;
  uint16_t drv0_status1;
  uint16_t drv1_status1;
  uint16_t drv0_control2;
  uint16_t drv1_control2;
  uint32_t ccr1;
  uint32_t ccr2;
  uint32_t ccr3;
  uint32_t ccer;
  uint32_t bdtr;
  uint32_t moe;
  uint32_t en_gate_raw;
  uint32_t nfault;
  bool ok;
} OpenLoopMonitorSample;

typedef struct {
  bool active;
  uint32_t samples;
  float iu_min;
  float iu_max;
  float iu_sum;
  float iv_min;
  float iv_max;
  float iv_sum;
  float iw_min;
  float iw_max;
  float iw_sum;
  float id_min;
  float id_max;
  float id_sum;
  float iq_min;
  float iq_max;
  float iq_sum;
  float id_sumsq;
  float iq_sumsq;
  float current_magnitude_max;
} CurrentObserveStats;

typedef struct {
  float correction_rad;
  bool valid;
  CurrentObserveStats stats;
  float id_mean;
  float iq_mean;
  float id_std;
  float iq_std;
  float ratio;
  float iq_effective_counts;
  float current_magnitude_max;
  float mechanical_rpm;
} OffsetCandidateResult;

typedef struct {
  uint32_t ccr4;
  bool valid;
  bool edge_timing_ok;
  CurrentObserveStats stats;
  float id_mean;
  float iq_mean;
  float id_std;
  float iq_std;
  float iu_mean;
  float iv_mean;
  float iw_mean;
  float id_effective_counts;
  float iq_over_id_ratio;
  float current_magnitude_max;
  uint32_t encoder_motion_counts;
  uint16_t raw_u_min;
  uint16_t raw_u_max;
  uint16_t raw_v_min;
  uint16_t raw_v_max;
  float min_phase_edge_distance_us;
} SamplePositionResult;

typedef struct {
  float vd_command;
  float vq_command;
  float v_alpha;
  float v_beta;
  bool valid;
  bool faulted;
  uint32_t invalid_reason;
  CurrentObserveStats stats;
  float id_mean;
  float iq_mean;
  float id_std;
  float iq_std;
  float iu_mean;
  float iv_mean;
  float iw_mean;
  float id_effective_counts;
  float iq_effective_counts;
  float phase_current_abs_mean;
  float phase_current_rms;
  float phase_current_p95;
  float phase_current_p99;
  float phase_current_p999;
  float phase_current_max;
  uint32_t current_over_consecutive_max;
  uint32_t samples_above_0p45a;
  uint32_t longest_consecutive_samples_above_0p45a;
  uint32_t encoder_motion_counts;
  uint16_t raw_u_min;
  uint16_t raw_u_max;
  uint16_t raw_v_min;
  uint16_t raw_v_max;
  float vbus_mean;
  float iq_over_id_ratio;
} SweepPointResult;

typedef struct {
  float vd;
  float vq;
  float theta_e;
  float measured_vbus;
  float v_alpha;
  float v_beta;
  float normalized_alpha;
  float normalized_beta;
  uint32_t vbus_divide_count;
  float phase_a;
  float phase_b;
  float phase_c;
  float common_mode;
  float duty_a;
  float duty_b;
  float duty_c;
  uint32_t ccr1;
  uint32_t ccr2;
  uint32_t ccr3;
  int32_t ccr1_offset;
  int32_t ccr2_offset;
  int32_t ccr3_offset;
  uint32_t ccr_span;
  float applied_phase_a;
  float applied_phase_b;
  float applied_phase_c;
  float applied_v_alpha;
  float applied_v_beta;
  float applied_voltage_magnitude;
  float commanded_to_applied_ratio;
  bool double_scaled;
  bool command_too_small;
  bool applied_scale_fail;
} VoltagePathDiag;

typedef struct {
  uint32_t sample_cnt;
  uint32_t current_tim1_cnt;
  uint32_t current_tim1_dir;
  uint32_t pwm_mode;
  uint32_t ccer;
  uint32_t bdtr;
  uint32_t ccr1;
  uint32_t ccr2;
  uint32_t ccr3;
  uint32_t sample_high_side_mask;
  uint32_t sample_low_side_mask;
  uint32_t deadtime_mask;
  uint32_t low_side_active_count;
  bool current_sample_window_valid;
} SampleWindowDiag;

typedef struct {
  float v_alpha_command;
  float v_beta_command;
  bool valid;
  bool faulted;
  uint32_t samples;
  float i_alpha_mean;
  float i_beta_mean;
  float i_alpha_std;
  float i_beta_std;
  float iu_mean;
  float iv_mean;
  float iw_mean;
  float raw_u_mean;
  float raw_v_mean;
  float effective_counts;
  uint32_t ccr1;
  uint32_t ccr2;
  uint32_t ccr3;
  float applied_voltage_magnitude_est;
  uint32_t encoder_motion_counts;
} DirectAlphaResult;

typedef struct {
  uint32_t samples;
  uint32_t mean_u;
  uint32_t mean_v;
  uint16_t u_min;
  uint16_t u_max;
  uint16_t v_min;
  uint16_t v_max;
  uint16_t u_p2p;
  uint16_t v_p2p;
  float u_std_counts;
  float v_std_counts;
  float u_std_amp;
  float v_std_amp;
  float u_rms_noise_counts;
  float v_rms_noise_counts;
  uint16_t u_range99_counts;
  uint16_t v_range99_counts;
  bool glitch_detected;
} DcCalNoiseStats;

typedef struct {
  float iu;
  float iv;
  float iw;
  float i_alpha;
  float i_beta;
  float id;
  float iq;
  float magnitude_max;
} CurrentDqSample;

typedef struct {
  uint32_t state;
  uint32_t substate;
  uint16_t raw_u;
  uint16_t raw_v;
  float iu;
  float iv;
  float iw;
  float id;
  float iq;
  float vd;
  float vq;
  float theta_e;
  uint32_t ccr1;
  uint32_t ccr2;
  uint32_t ccr3;
  uint32_t ccr4;
  uint32_t tim1_cnt;
  uint32_t tim1_dir;
} CurrentTripRingSample;

typedef struct {
  bool latched;
  uint32_t fault_timestamp_us;
  uint32_t fault_state;
  uint32_t fault_substate;
  uint16_t raw_u;
  uint16_t raw_v;
  uint32_t offset_u;
  uint32_t offset_v;
  int32_t delta_u_counts;
  int32_t delta_v_counts;
  float iu;
  float iv;
  float iw;
  float id;
  float iq;
  float theta_e;
  float vd;
  float vq;
  uint32_t ccr1;
  uint32_t ccr2;
  uint32_t ccr3;
  uint32_t ccr4;
  uint32_t tim1_cnt;
  uint32_t tim1_dir;
  uint32_t moe;
  uint32_t en_gate;
  int64_t encoder_accum;
  float vbus_v;
  uint32_t adc_seq;
  bool trip_iu;
  bool trip_iv;
  bool trip_iw;
  bool trip_id;
  bool trip_iq;
  const char *protection_type;
  const char *first_trip_channel;
  float current_value;
  uint32_t consecutive_count;
} CurrentTripFaultSnapshot;

typedef struct {
  float align_current_abs_max;
  float zero_baseline_current_abs_max;
  float moe_first_1ms_current_abs_max;
  float switch_first_1ms_current_abs_max;
  uint32_t iu_over_count;
  uint32_t iv_over_count;
  uint32_t iw_over_count;
  uint32_t id_over_count;
  uint32_t iq_over_count;
  uint32_t current_over_consecutive_max;
  uint32_t iu_over_consecutive;
  uint32_t iv_over_consecutive;
  uint32_t iw_over_consecutive;
  uint32_t id_over_consecutive;
  uint32_t iq_over_consecutive;
  uint32_t ring_write_index;
  uint32_t ring_count;
  uint32_t moe_enable_timestamp_us;
  uint32_t switch_timestamp_us;
  bool completed_zero_baseline;
  const char *classification;
} CurrentTripDiagnostics;

typedef struct {
  bool ran;
  bool pass;
  bool adc_offset_pass;
  bool drv0_cfg;
  bool drv1_cfg;
  bool drv0_status_ok;
  bool drv1_status_ok;
  bool open_loop_pass;
  bool encoder_idle_pass;
  bool encoder_data_reliable;
  bool pole_pairs_available;
  bool motion_reliable;
  bool nfault_released;
  bool adc_seq_growing;
  bool raw_range_ok;
  bool m1_safe;
  bool current_scale_known;
  bool current_direction_ok;
  bool dq_alignment_ok;
  bool gain40_readback_ok;
  bool dc_cal_offsets_pass;
  bool dc_cal_clear_ok;
  bool adc_sample_timing_ok;
  bool current_resolution_ok;
  bool kcl_ok;
  bool adc_phase_edge_timing_ok;
  bool adc_sync_rate_ok;
  bool offset_fine_tune_reliable;
  bool final_observe_reliable;
  uint32_t adc_seq_before;
  uint32_t adc_seq_after;
  uint32_t nfault_release_ms;
  uint32_t fail_step;
  uint32_t fault_code;
  AdcOffsetStats offset;
  Drv8301Registers drv0_regs;
  Drv8301Registers drv1_regs;
  uint16_t run_raw_u_min;
  uint16_t run_raw_u_max;
  uint16_t run_raw_v_min;
  uint16_t run_raw_v_max;
  OpenLoopMonitorSample monitor[50];
  uint32_t monitor_count;
  int64_t enc_idle_min;
  int64_t enc_idle_max;
  uint32_t enc_idle_noise_pp;
  int64_t encoder_align_count;
  int64_t encoder_start_count;
  int64_t encoder_end_count;
  int64_t encoder_total_delta;
  int32_t encoder_direction;
  int32_t encoder_cpr;
  int32_t encoder_ppr;
  uint32_t encoder_counts_per_ab_cycle;
  uint32_t pole_pairs_runtime;
  float theta_m_align;
  float electrical_offset_runtime_rad;
  float electrical_offset_runtime_deg;
  int64_t encoder_delta_10ms;
  float mechanical_rpm_raw;
  float mechanical_rpm_filtered;
  float maximum_rpm;
  float minimum_rpm;
  uint32_t raw_current_max_deviation;
  float current_shunt_ohm;
  float current_amp_gain;
  float adc_vref_v;
  float current_amp_per_count;
  int32_t current_polarity_u;
  int32_t current_polarity_v;
  float iu_lpf;
  float iv_lpf;
  float iw_lpf;
  float id_lpf;
  float iq_lpf;
  float current_max_a;
  float id_iq_abs_ratio;
  float id_mean_effective_counts;
  float iq_mean_effective_counts;
  float kcl_residual_max;
  uint16_t expected_control2;
  uint16_t actual_control2_drv0;
  uint16_t actual_control2_drv1;
  uint8_t gain_field_drv0;
  uint8_t gain_field_drv1;
  float noise_pp_counts;
  float noise_pp_amp;
  float pwm_period_us;
  uint32_t adc_trigger_position_counts;
  float distance_to_nearest_pwm_edge_us;
  uint32_t min_distance_to_phase_edge_counts;
  float min_distance_to_phase_edge_us;
  uint32_t pwm_cycle_count;
  uint32_t adc_snapshot_count;
  float snapshots_per_pwm_cycle;
  DcCalNoiseStats dc_noise;
  OffsetCandidateResult candidates[7];
  SamplePositionResult sample_positions[3];
  SweepPointResult sweep_points[11];
  VoltagePathDiag voltage_diags[6];
  SampleWindowDiag sample_window_diag;
  DirectAlphaResult direct_alpha[5];
  uint32_t voltage_diag_count;
  uint32_t direct_alpha_count;
  bool voltage_command_double_scaled;
  bool pwm_voltage_command_too_small;
  bool applied_voltage_scale_fail;
  bool low_side_sample_window_valid;
  bool direct_alpha_reliable;
  const char *voltage_path_classification;
  int32_t best_candidate_index;
  int32_t recommended_ccr4;
  float achieved_vd;
  float first_reliable_current_voltage;
  float phase_resistance_est_ohm;
  float inverter_voltage_offset_est_v;
  float fit_r_squared;
  float voltage_required_for_0p2A_est;
  float voltage_required_for_0p3A_est;
  uint32_t sweep_point_count;
  uint32_t valid_fit_point_count;
  uint32_t first_reliable_point_index;
  bool current_monotonic_ok;
  bool phase_resistance_est_reliable;
  uint32_t encoder_motion_max_counts;
  bool d_axis_signal_ok;
  bool d_axis_polarity_ok;
  bool d_axis_motion_ok;
  bool sample_window_reliable;
  float electrical_offset_correction_candidate;
  float electrical_offset_runtime_corrected;
  CurrentObserveStats current_stats;
  bool final_gate;
  bool final_nfault;
  uint32_t final_ccer;
  uint32_t final_bdtr;
  uint32_t final_ccr1;
  uint32_t final_ccr2;
  uint32_t final_ccr3;
  CurrentTripFaultSnapshot current_trip_fault;
  CurrentTripDiagnostics current_trip_diag;
  CurrentTripRingSample current_trip_ring[32];
} DrvBringupTestResult;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_PRINT_TIMEOUT_MS 100u
#define BRINGUP_PRINT_PERIOD_MS 1000u
#define ADC_OFFSET_SAMPLE_COUNT 8192u
#define ADC_OFFSET_TIMEOUT_MS 2500u
#define DRV_NFAULT_RELEASE_TIMEOUT_MS 200u
#define ENCODER_IDLE_CHECK_MS 500u
#define ENCODER_IDLE_SAMPLE_PERIOD_MS 10u
#define ENCODER_IDLE_NOISE_MAX_COUNTS 64u
#define ENCODER_ALIGN_MS 500u
#define ENCODER_ALIGN_AVG_MS 100u
#define COMM_VQ_RAMP_MS 1000u
#define COMM_VQ_HOLD_MS 1500u
#define COMM_VQ_DOWN_MS 500u
#define COMM_TOTAL_MS (ENCODER_ALIGN_MS + COMM_VQ_RAMP_MS + COMM_VQ_HOLD_MS + COMM_VQ_DOWN_MS)
#define COMM_VD_MAX_ABS_V 0.25f
#define COMM_VQ_MAX_ABS_V 0.15f
#define COMM_VQ_TARGET_V 0.12f
#define COMM_ALIGN_V_ALPHA 0.20f
#define STATIC_D_AXIS_ALIGN_V_ALPHA 0.20f
#define D_AXIS_VD_TARGET_V 0.50f
#define D_AXIS_VD_MAX_V 0.50f
#define D_AXIS_ID_STOP_A 0.30f
#define D_AXIS_ID_HOLD_A 0.30f
#define D_AXIS_SIGNAL_MIN_A 0.30f
#define D_AXIS_CURRENT_LIMIT_A 0.60f
#define CURRENT_CONTINUOUS_LIMIT_A 0.60f
#define CURRENT_CONTINUOUS_RESET_A 0.55f
#define CURRENT_CONTINUOUS_TRIP_SAMPLES 3u
#define CURRENT_INSTANT_LIMIT_A 1.00f
#define D_AXIS_PHASE_STOP_A 0.45f
#define D_AXIS_MOTION_MAX_COUNTS 30u
#define D_AXIS_IQ_OVER_ID_RATIO_MAX 0.30f
#define D_AXIS_WINDOW_DIFF_MAX_RATIO 0.15f
#define D_AXIS_RAMP_MS 100u
#define D_AXIS_SETTLE_MS 150u
#define D_AXIS_SAMPLE_MS 300u
#define D_AXIS_HOLD_MS D_AXIS_SAMPLE_MS
#define D_AXIS_DOWN_MS 100u
#define D_AXIS_SAMPLE_RAMP_MS 200u
#define D_AXIS_SAMPLE_SETTLE_MS 100u
#define D_AXIS_SAMPLE_COLLECT_MS 300u
#define D_AXIS_SAMPLE_DOWN_MS 200u
#define D_AXIS_SAMPLE_POSITION_COUNT 3u
#define D_AXIS_SWEEP_POINT_COUNT 11u
#define D_AXIS_VALID_MIN_COUNTS 5.0f
#define D_AXIS_RELIABLE_MIN_COUNTS 10.0f
#define D_AXIS_SWEEP_MIN_FIT_POINTS 3u
#define D_AXIS_PHASE_CURRENT_STOP_A 0.45f
#define D_AXIS_PHASE_CURRENT_RMS_STOP_A 0.35f
#define D_AXIS_PHASE_CURRENT_HIST_SCALE 1000.0f
#define D_AXIS_PHASE_CURRENT_HIST_BINS 801u
#define D_AXIS_SAMPLE_IGNORE_MS 20u
#define PHASE_RESISTANCE_UNSET -1.0f
#define DC_CAL_STD_MAX_COUNTS 4.0f
#define CURRENT_SHUNT_OHM 0.0005f
#define CURRENT_AMP_GAIN_V_PER_V 40.0f
#define CURRENT_ADC_VREF_V 3.3f
#define CURRENT_ADC_FULL_SCALE_COUNTS 4096.0f
#define CURRENT_POLARITY_U 1
#define CURRENT_POLARITY_V 1
#define CURRENT_LIMIT_ABS_A D_AXIS_CURRENT_LIMIT_A
#define CURRENT_LPF_ALPHA 0.08f
#define CURRENT_MIN_EFFECTIVE_COUNTS 10.0f
#define CURRENT_DQ_RATIO_MAX 0.35f
#define CURRENT_RAW_MIN_SAFE_COUNT 100u
#define CURRENT_RAW_MAX_SAFE_COUNT 3995u
#define ADC_SAMPLE_EDGE_MARGIN_MIN_US 2.0f
#define TIM1_TIMER_CLOCK_HZ 168000000.0f
#define ADC_TRIGGER_SAFE_OFFSET_COUNTS 400u
#define PWM_CYCLES_PER_MS 20u
#define SNAPSHOT_RATE_MIN_PER_PWM 0.85f
#define SNAPSHOT_RATE_MAX_PER_PWM 1.15f
#define OFFSET_CANDIDATE_COUNT 7u
#define OFFSET_CANDIDATE_RAMP_MS 300u
#define OFFSET_CANDIDATE_SETTLE_MS 150u
#define OFFSET_CANDIDATE_SAMPLE_MS 300u
#define OFFSET_CANDIDATE_DOWN_MS 200u
#define FINAL_OBSERVE_RAMP_MS 1000u
#define FINAL_OBSERVE_HOLD_MS 1000u
#define FINAL_OBSERVE_DOWN_MS 500u
#define SQRT3_F 1.7320508075688772935f
#define COMM_ENCODER_CPR 4096
#define COMM_ENCODER_PPR 1024
#define COMM_ENCODER_DIRECTION 1
#define COMM_POLE_PAIRS 7u
#define COMM_RPM_UPDATE_MS 10u
#define COMM_MOTION_MIN_COUNTS 16
#define COMM_REVERSE_FAULT_COUNTS (-16)
#define COMM_RAW_CURRENT_MAX_DEV_COUNTS 250u
#define COMM_THETA_JUMP_MAX_RAD 0.25f
#define ENCODER_JUMP_MAX_COUNTS_PER_SAMPLE 512
#define OPEN_LOOP_MONITOR_PERIOD_MS 50u
#define OPEN_LOOP_CONTROL_TS_S (1.0f / 20000.0f)
#define OPEN_LOOP_VBUS_MIN_V 7.0f
#define OPEN_LOOP_VBUS_MAX_V 13.0f
#define OPEN_LOOP_MIN_DUTY 0.05f
#define OPEN_LOOP_MAX_DUTY 0.95f
#define TWO_PI_F 6.2831853071795864769f
#define POWER_CCER_MASK (TIM_CCER_CC1E | TIM_CCER_CC1NE | \
                         TIM_CCER_CC2E | TIM_CCER_CC2NE | \
                         TIM_CCER_CC3E | TIM_CCER_CC3NE)
#define M1_CCER_MASK POWER_CCER_MASK
#define TEST_STATE_IDLE 0u
#define TEST_STATE_ALIGN 1u
#define TEST_STATE_VD_RAMP 2u
#define TEST_STATE_VD_SETTLE 3u
#define TEST_STATE_VD_SAMPLE 4u
#define TEST_STATE_VD_DOWN 5u
#define TEST_STATE_SAMPLE_POSITION_TEST 6u
#define TEST_STATE_STOP 7u
#define TEST_STATE_VD_HOLD TEST_STATE_VD_SAMPLE
#define TEST_STATE_CURRENT_OBSERVE_RAMP TEST_STATE_VD_RAMP
#define TEST_STATE_CURRENT_OBSERVE_HOLD TEST_STATE_VD_SAMPLE
#define TEST_STATE_CURRENT_OBSERVE_DOWN TEST_STATE_VD_DOWN
#define DIAG_STATE_INIT 20u
#define DIAG_STATE_DC_CAL 21u
#define DIAG_STATE_ALIGN_RAMP 22u
#define DIAG_STATE_ALIGN_HOLD 23u
#define DIAG_STATE_ALIGN_DOWN 24u
#define DIAG_STATE_SWEEP_ZERO_RAMP 25u
#define DIAG_STATE_SWEEP_ZERO_SETTLE 26u
#define DIAG_STATE_SWEEP_ZERO_SAMPLE 27u
#define DIAG_STATE_SWEEP_POSITIVE 28u
#define DIAG_STATE_STOP 29u
#define CURRENT_TRIP_RING_COUNT 32u
#define CURRENT_TRIP_ALIGN_RAMP_MS 300u
#define CURRENT_TRIP_ALIGN_HOLD_MS 500u
#define CURRENT_TRIP_ALIGN_DOWN_MS 100u
#define CURRENT_TRIP_ZERO_RAMP_MS 100u
#define CURRENT_TRIP_ZERO_SETTLE_MS 150u
#define CURRENT_TRIP_ZERO_SAMPLE_MS 300u
#define CURRENT_TRIP_FIRST_1MS_US 1000u
#define CURRENT_TRIP_SWITCHING_TRANSIENT_US 200u
#define SWEEP_INVALID_ID_NEGATIVE (1u << 0)
#define SWEEP_INVALID_ID_COUNTS_TOO_SMALL (1u << 1)
#define SWEEP_INVALID_IQ_ID_RATIO_TOO_LARGE (1u << 2)
#define SWEEP_INVALID_ENCODER_MOVED (1u << 3)
#define SWEEP_INVALID_ADC_INVALID (1u << 4)
#define SWEEP_INVALID_SAFETY_FAULT (1u << 5)
#define DIRECT_ALPHA_POINT_COUNT 5u
#define VOLTAGE_DIAG_POINT_COUNT 6u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static Axis0Context g_axis0;
static Drv8301 g_drv0;
static Drv8301 g_drv1;
static DrvBringupTestResult g_drv_test;
static volatile uint32_t g_adc_callback_count = 0u;
static bool g_adc_init_ok = false;
static bool g_board_init_ok = false;
static uint16_t g_encoder_last_cnt = 0u;
static int64_t g_encoder_accum = 0;
static int16_t g_encoder_last_delta = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void uart2_write(const char *text);
static void uart2_printf_line(const char *text);
static void axis0_context_init_minimal(void);
static void print_bringup_status(void);
static void drv_bringup_test_run(void);
static void print_drv_bringup_test_status(void);
static CurrentDqSample current_observe_calculate(const HalAdcSnapshot *snap, float theta_e);
static void current_observe_lpf_update(const CurrentDqSample *sample);
static void current_observe_stats_update(CurrentObserveStats *stats, const CurrentDqSample *sample);
static bool power_stage_check_adc_sample_timing(void);
static bool power_stage_update_phase_edge_timing(void);
static bool direct_alpha_voltage_diagnostic_run(uint32_t *last_seq);
static void current_observe_stats_finalize(const CurrentObserveStats *stats,
                                           float *id_mean,
                                           float *iq_mean,
                                           float *id_std,
                                           float *iq_std);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void uart2_write(const char *text)
{
  if (text == NULL) {
    return;
  }

  const size_t len = strlen(text);
  if (len == 0u) {
    return;
  }

  (void)HAL_UART_Transmit(&huart2,
                          (uint8_t *)text,
                          (uint16_t)len,
                          UART_PRINT_TIMEOUT_MS);
}

static void uart2_printf_line(const char *text)
{
  uart2_write(text);
  uart2_write("\r\n");
}

static void axis0_context_init_minimal(void)
{
  memset(&g_axis0, 0, sizeof(g_axis0));

  g_axis0.config = axis0_default_config_make();
  g_axis0.state = AXIS0_STATE_BOOT;
  g_axis0.requested_state = AXIS0_STATE_IDLE;
  g_axis0.request_pending = false;
  g_axis0.cmd.control_mode = AXIS0_CONTROL_MODE_IDLE;
  g_axis0.cmd.id_target_a = 0.0f;
  g_axis0.cmd.iq_target_a = 0.0f;
}

static uint32_t tim1_half_ccr(void)
{
  return TIM1->ARR / 2u;
}

static bool drv_status_has_fault(uint16_t status1, uint16_t status2)
{
  return ((status1 & 0x07ffu) != 0u) || ((status2 & 0x0080u) != 0u);
}

static void power_stage_disable_six_outputs(void)
{
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  TIM1->CCER &= ~POWER_CCER_MASK;
}

static void power_stage_set_ccr_half(void)
{
  const uint32_t half = tim1_half_ccr();
  TIM1->CCR1 = half;
  TIM1->CCR2 = half;
  TIM1->CCR3 = half;
}

static void power_stage_force_safe_off_zero_ccr(void)
{
  power_stage_disable_six_outputs();
  TIM1->CCR1 = 0u;
  TIM1->CCR2 = 0u;
  TIM1->CCR3 = 0u;
  hal_gpio_set_gate_enable(false);
}

static void power_stage_startup_safe_half_ccr(void)
{
  hal_pwm_disable();
  hal_gpio_set_gate_enable(false);
  power_stage_disable_six_outputs();
  hal_pwm_start_adc_trigger_only();
  power_stage_disable_six_outputs();
  power_stage_set_ccr_half();
}

static bool power_stage_channels_off(void)
{
  return ((TIM1->BDTR & TIM_BDTR_MOE) == 0u) &&
         ((TIM1->CCER & POWER_CCER_MASK) == 0u);
}

static void drv_bringup_capture_final_state(void)
{
  g_drv_test.final_gate = HAL_GPIO_ReadPin(EN_GATE_GPIO_Port, EN_GATE_Pin) == GPIO_PIN_SET;
  g_drv_test.final_nfault = board_read_drv_nfault();
  g_drv_test.final_ccer = TIM1->CCER;
  g_drv_test.final_bdtr = TIM1->BDTR;
  g_drv_test.final_ccr1 = TIM1->CCR1;
  g_drv_test.final_ccr2 = TIM1->CCR2;
  g_drv_test.final_ccr3 = TIM1->CCR3;
}

static void drv_bringup_configure_nfault_pull(uint32_t pull)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = DRV_NFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = pull;
  HAL_GPIO_Init(DRV_NFAULT_GPIO_Port, &GPIO_InitStruct);
}

static uint32_t drv_bringup_get_adc_seq(void)
{
  HalAdcSnapshot snap = {0};
  return hal_adc_get_snapshot(&snap) ? snap.seq : 0u;
}

static void drv_bringup_mark_fault(Axis0FaultFlags fault)
{
  g_axis0.fault_flags |= (uint32_t)fault;
  g_axis0.state = AXIS0_STATE_FAULT;
}

static bool gate_raw_is_low(void)
{
  return HAL_GPIO_ReadPin(EN_GATE_GPIO_Port, EN_GATE_Pin) == GPIO_PIN_RESET;
}

static bool gate_raw_is_high(void)
{
  return HAL_GPIO_ReadPin(EN_GATE_GPIO_Port, EN_GATE_Pin) == GPIO_PIN_SET;
}

static bool nfault_ok(void)
{
  return !board_read_drv_nfault();
}

static void encoder_tracker_reset(void)
{
  g_encoder_last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  g_encoder_accum = 0;
  g_encoder_last_delta = 0;
}

static int64_t encoder_tracker_sample(void)
{
  const uint16_t cnt_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  const int16_t delta = (int16_t)(cnt_now - g_encoder_last_cnt);
  g_encoder_last_cnt = cnt_now;
  g_encoder_last_delta = delta;
  g_encoder_accum += (int64_t)delta;
  return g_encoder_accum;
}

static bool encoder_delta_ok(void)
{
  return (g_encoder_last_delta <= ENCODER_JUMP_MAX_COUNTS_PER_SAMPLE) &&
         (g_encoder_last_delta >= -ENCODER_JUMP_MAX_COUNTS_PER_SAMPLE);
}

static void power_stage_enable_six_outputs(void)
{
  TIM1->CCER |= POWER_CCER_MASK;
}

static void m1_force_safe_off(void)
{
  __HAL_RCC_TIM8_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  TIM8->BDTR &= ~TIM_BDTR_MOE;
  TIM8->CCER &= ~M1_CCER_MASK;
  TIM8->CCR1 = 0u;
  TIM8->CCR2 = 0u;
  TIM8->CCR3 = 0u;

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);
}

static bool m1_is_safe_off(void)
{
  const bool timer_safe = ((TIM8->BDTR & TIM_BDTR_MOE) == 0u) &&
                          ((TIM8->CCER & M1_CCER_MASK) == 0u);
  const bool pins_low = ((GPIOA->ODR & GPIO_PIN_7) == 0u) &&
                        ((GPIOB->ODR & (GPIO_PIN_0 | GPIO_PIN_1)) == 0u) &&
                        ((GPIOC->ODR & (GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8)) == 0u);
  return timer_safe && pins_low;
}

static float clampf(float x, float lo, float hi)
{
  if (x < lo) { return lo; }
  if (x > hi) { return hi; }
  return x;
}

static uint32_t duty_to_ccr_limited(float duty)
{
  const float limited = clampf(duty, OPEN_LOOP_MIN_DUTY, OPEN_LOOP_MAX_DUTY);
  return (uint32_t)(limited * (float)TIM1->ARR + 0.5f);
}

static bool ccrs_in_open_loop_range(void)
{
  const uint32_t ccr_min = duty_to_ccr_limited(OPEN_LOOP_MIN_DUTY);
  const uint32_t ccr_max = duty_to_ccr_limited(OPEN_LOOP_MAX_DUTY);
  return (TIM1->CCR1 >= ccr_min) && (TIM1->CCR1 <= ccr_max) &&
         (TIM1->CCR2 >= ccr_min) && (TIM1->CCR2 <= ccr_max) &&
         (TIM1->CCR3 >= ccr_min) && (TIM1->CCR3 <= ccr_max);
}

static const char *test_state_name(uint32_t state)
{
  switch (state) {
  case TEST_STATE_IDLE: return "IDLE";
  case TEST_STATE_ALIGN: return "ALIGN";
  case TEST_STATE_VD_RAMP: return "VD_RAMP";
  case TEST_STATE_VD_SETTLE: return "VD_SETTLE";
  case TEST_STATE_VD_SAMPLE: return "VD_SAMPLE";
  case TEST_STATE_VD_DOWN: return "VD_DOWN";
  case TEST_STATE_SAMPLE_POSITION_TEST: return "SAMPLE_POSITION_TEST";
  case TEST_STATE_STOP: return "STOP";
  case DIAG_STATE_INIT: return "INIT";
  case DIAG_STATE_DC_CAL: return "DC_CAL";
  case DIAG_STATE_ALIGN_RAMP: return "ALIGN_RAMP";
  case DIAG_STATE_ALIGN_HOLD: return "ALIGN_HOLD";
  case DIAG_STATE_ALIGN_DOWN: return "ALIGN_DOWN";
  case DIAG_STATE_SWEEP_ZERO_RAMP: return "SWEEP_ZERO_RAMP";
  case DIAG_STATE_SWEEP_ZERO_SETTLE: return "SWEEP_ZERO_SETTLE";
  case DIAG_STATE_SWEEP_ZERO_SAMPLE: return "SWEEP_ZERO_SAMPLE";
  case DIAG_STATE_SWEEP_POSITIVE: return "SWEEP_POSITIVE";
  case DIAG_STATE_STOP: return "STOP";
  default: return "UNKNOWN";
  }
}

static uint32_t diag_timestamp_us(void)
{
  return (HAL_GetTick() * 1000u) +
         (uint32_t)(((uint64_t)TIM1->CNT * 1000000ull) / TIM1_TIMER_CLOCK_HZ);
}

static uint32_t tim1_count_direction(void)
{
  return ((TIM1->CR1 & TIM_CR1_DIR) != 0u) ? 1u : 0u;
}

static void current_trip_diag_reset(void)
{
  memset(&g_drv_test.current_trip_fault, 0, sizeof(g_drv_test.current_trip_fault));
  memset(&g_drv_test.current_trip_diag, 0, sizeof(g_drv_test.current_trip_diag));
  memset(g_drv_test.current_trip_ring, 0, sizeof(g_drv_test.current_trip_ring));
  g_drv_test.current_trip_diag.classification = "CURRENT_TRIP_DIAGNOSTIC_NOT_RUN";
}

static void current_trip_ring_push(uint32_t state,
                                   uint32_t substate,
                                   const HalAdcSnapshot *snap,
                                   const CurrentDqSample *sample,
                                   float vd,
                                   float vq,
                                   float theta_e)
{
  CurrentTripDiagnostics *diag = &g_drv_test.current_trip_diag;
  CurrentTripRingSample *entry = &g_drv_test.current_trip_ring[diag->ring_write_index];
  entry->state = state;
  entry->substate = substate;
  entry->raw_u = snap->raw_u;
  entry->raw_v = snap->raw_v;
  entry->iu = sample->iu;
  entry->iv = sample->iv;
  entry->iw = sample->iw;
  entry->id = sample->id;
  entry->iq = sample->iq;
  entry->vd = vd;
  entry->vq = vq;
  entry->theta_e = theta_e;
  entry->ccr1 = TIM1->CCR1;
  entry->ccr2 = TIM1->CCR2;
  entry->ccr3 = TIM1->CCR3;
  entry->ccr4 = TIM1->CCR4;
  entry->tim1_cnt = TIM1->CNT;
  entry->tim1_dir = tim1_count_direction();

  diag->ring_write_index = (diag->ring_write_index + 1u) % CURRENT_TRIP_RING_COUNT;
  if (diag->ring_count < CURRENT_TRIP_RING_COUNT) {
    diag->ring_count++;
  }
}

static void current_trip_update_stage_max(uint32_t state, float current_abs_max)
{
  CurrentTripDiagnostics *diag = &g_drv_test.current_trip_diag;
  const uint32_t now_us = diag_timestamp_us();
  if ((state == DIAG_STATE_ALIGN_RAMP) || (state == DIAG_STATE_ALIGN_HOLD) ||
      (state == DIAG_STATE_ALIGN_DOWN)) {
    if (current_abs_max > diag->align_current_abs_max) {
      diag->align_current_abs_max = current_abs_max;
    }
  }
  if ((state == DIAG_STATE_SWEEP_ZERO_RAMP) ||
      (state == DIAG_STATE_SWEEP_ZERO_SETTLE) ||
      (state == DIAG_STATE_SWEEP_ZERO_SAMPLE)) {
    if (current_abs_max > diag->zero_baseline_current_abs_max) {
      diag->zero_baseline_current_abs_max = current_abs_max;
    }
  }
  if ((diag->moe_enable_timestamp_us != 0u) &&
      ((now_us - diag->moe_enable_timestamp_us) <= CURRENT_TRIP_FIRST_1MS_US) &&
      (current_abs_max > diag->moe_first_1ms_current_abs_max)) {
    diag->moe_first_1ms_current_abs_max = current_abs_max;
  }
  if ((diag->switch_timestamp_us != 0u) &&
      ((now_us - diag->switch_timestamp_us) <= CURRENT_TRIP_FIRST_1MS_US) &&
      (current_abs_max > diag->switch_first_1ms_current_abs_max)) {
    diag->switch_first_1ms_current_abs_max = current_abs_max;
  }
}

static uint32_t current_trip_current_consecutive_max(void)
{
  const CurrentTripDiagnostics *diag = &g_drv_test.current_trip_diag;
  uint32_t max_consecutive = diag->iu_over_consecutive;
  if (diag->iv_over_consecutive > max_consecutive) { max_consecutive = diag->iv_over_consecutive; }
  if (diag->iw_over_consecutive > max_consecutive) { max_consecutive = diag->iw_over_consecutive; }
  if (diag->id_over_consecutive > max_consecutive) { max_consecutive = diag->id_over_consecutive; }
  if (diag->iq_over_consecutive > max_consecutive) { max_consecutive = diag->iq_over_consecutive; }
  return max_consecutive;
}

static bool current_trip_diag_record_and_check(uint32_t state,
                                               uint32_t substate,
                                               const HalAdcSnapshot *snap,
                                               const CurrentDqSample *sample,
                                               float vd,
                                               float vq,
                                               float theta_e,
                                               float vbus_v)
{
  CurrentTripDiagnostics *diag = &g_drv_test.current_trip_diag;
  const float abs_iu = fabsf(sample->iu);
  const float abs_iv = fabsf(sample->iv);
  const float abs_iw = fabsf(sample->iw);
  const float abs_id = fabsf(sample->id);
  const float abs_iq = fabsf(sample->iq);
  const bool over_iu = abs_iu > CURRENT_CONTINUOUS_LIMIT_A;
  const bool over_iv = abs_iv > CURRENT_CONTINUOUS_LIMIT_A;
  const bool over_iw = abs_iw > CURRENT_CONTINUOUS_LIMIT_A;
  const bool over_id = abs_id > CURRENT_CONTINUOUS_LIMIT_A;
  const bool over_iq = abs_iq > CURRENT_CONTINUOUS_LIMIT_A;
  const bool instant_iu = abs_iu > CURRENT_INSTANT_LIMIT_A;
  const bool instant_iv = abs_iv > CURRENT_INSTANT_LIMIT_A;
  const bool instant_iw = abs_iw > CURRENT_INSTANT_LIMIT_A;
  const bool instant_id = abs_id > CURRENT_INSTANT_LIMIT_A;
  const bool instant_iq = abs_iq > CURRENT_INSTANT_LIMIT_A;

  if (over_iu) { diag->iu_over_count++; diag->iu_over_consecutive++; }
  else if (abs_iu < CURRENT_CONTINUOUS_RESET_A) { diag->iu_over_consecutive = 0u; }
  if (over_iv) { diag->iv_over_count++; diag->iv_over_consecutive++; }
  else if (abs_iv < CURRENT_CONTINUOUS_RESET_A) { diag->iv_over_consecutive = 0u; }
  if (over_iw) { diag->iw_over_count++; diag->iw_over_consecutive++; }
  else if (abs_iw < CURRENT_CONTINUOUS_RESET_A) { diag->iw_over_consecutive = 0u; }
  if (over_id) { diag->id_over_count++; diag->id_over_consecutive++; }
  else if (abs_id < CURRENT_CONTINUOUS_RESET_A) { diag->id_over_consecutive = 0u; }
  if (over_iq) { diag->iq_over_count++; diag->iq_over_consecutive++; }
  else if (abs_iq < CURRENT_CONTINUOUS_RESET_A) { diag->iq_over_consecutive = 0u; }

  uint32_t max_consecutive = current_trip_current_consecutive_max();
  if (max_consecutive > diag->current_over_consecutive_max) {
    diag->current_over_consecutive_max = max_consecutive;
  }

  bool trip_iu = instant_iu || (diag->iu_over_consecutive >= CURRENT_CONTINUOUS_TRIP_SAMPLES);
  bool trip_iv = instant_iv || (diag->iv_over_consecutive >= CURRENT_CONTINUOUS_TRIP_SAMPLES);
  bool trip_iw = instant_iw || (diag->iw_over_consecutive >= CURRENT_CONTINUOUS_TRIP_SAMPLES);
  bool trip_id = instant_id || (diag->id_over_consecutive >= CURRENT_CONTINUOUS_TRIP_SAMPLES);
  bool trip_iq = instant_iq || (diag->iq_over_consecutive >= CURRENT_CONTINUOUS_TRIP_SAMPLES);
  const bool any_instant = instant_iu || instant_iv || instant_iw || instant_id || instant_iq;
  const bool any_trip = trip_iu || trip_iv || trip_iw || trip_id || trip_iq;

  current_trip_update_stage_max(state, sample->magnitude_max);
  current_trip_ring_push(state, substate, snap, sample, vd, vq, theta_e);

  if (any_trip && !g_drv_test.current_trip_fault.latched) {
    CurrentTripFaultSnapshot *fault = &g_drv_test.current_trip_fault;
    fault->latched = true;
    fault->fault_timestamp_us = diag_timestamp_us();
    fault->fault_state = state;
    fault->fault_substate = substate;
    fault->raw_u = snap->raw_u;
    fault->raw_v = snap->raw_v;
    fault->offset_u = g_drv_test.offset.offset_u;
    fault->offset_v = g_drv_test.offset.offset_v;
    fault->delta_u_counts = (int32_t)snap->raw_u - (int32_t)g_drv_test.offset.offset_u;
    fault->delta_v_counts = (int32_t)snap->raw_v - (int32_t)g_drv_test.offset.offset_v;
    fault->iu = sample->iu;
    fault->iv = sample->iv;
    fault->iw = sample->iw;
    fault->id = sample->id;
    fault->iq = sample->iq;
    fault->theta_e = theta_e;
    fault->vd = vd;
    fault->vq = vq;
    fault->ccr1 = TIM1->CCR1;
    fault->ccr2 = TIM1->CCR2;
    fault->ccr3 = TIM1->CCR3;
    fault->ccr4 = TIM1->CCR4;
    fault->tim1_cnt = TIM1->CNT;
    fault->tim1_dir = tim1_count_direction();
    fault->moe = ((TIM1->BDTR & TIM_BDTR_MOE) != 0u) ? 1u : 0u;
    fault->en_gate = gate_raw_is_high() ? 1u : 0u;
    fault->encoder_accum = g_encoder_accum;
    fault->vbus_v = vbus_v;
    fault->adc_seq = snap->seq;
    fault->trip_iu = trip_iu;
    fault->trip_iv = trip_iv;
    fault->trip_iw = trip_iw;
    fault->trip_id = trip_id;
    fault->trip_iq = trip_iq;
    fault->protection_type = any_instant ? "INSTANT_OVERCURRENT" : "CONTINUOUS_OVERCURRENT";
    if (trip_iu) {
      fault->first_trip_channel = "IU";
      fault->current_value = sample->iu;
      fault->consecutive_count = diag->iu_over_consecutive;
    } else if (trip_iv) {
      fault->first_trip_channel = "IV";
      fault->current_value = sample->iv;
      fault->consecutive_count = diag->iv_over_consecutive;
    } else if (trip_iw) {
      fault->first_trip_channel = "IW";
      fault->current_value = sample->iw;
      fault->consecutive_count = diag->iw_over_consecutive;
    } else if (trip_id) {
      fault->first_trip_channel = "ID";
      fault->current_value = sample->id;
      fault->consecutive_count = diag->id_over_consecutive;
    } else {
      fault->first_trip_channel = "IQ";
      fault->current_value = sample->iq;
      fault->consecutive_count = diag->iq_over_consecutive;
    }
  }

  return !any_trip;
}

static void current_trip_diag_classify(void)
{
  CurrentTripDiagnostics *diag = &g_drv_test.current_trip_diag;
  const CurrentTripFaultSnapshot *fault = &g_drv_test.current_trip_fault;
  if (!fault->latched) {
    diag->classification = diag->completed_zero_baseline
                               ? "CURRENT_TRIP_DIAGNOSTIC_PASS"
                               : "CURRENT_TRIP_DIAGNOSTIC_INCOMPLETE";
    return;
  }

  if ((diag->moe_enable_timestamp_us != 0u) &&
      ((fault->fault_timestamp_us - diag->moe_enable_timestamp_us) <=
       CURRENT_TRIP_SWITCHING_TRANSIENT_US) &&
      (diag->current_over_consecutive_max <= 1u)) {
    diag->classification = "CURRENT_TRIP_SWITCHING_TRANSIENT";
  } else if (((fault->fault_state == DIAG_STATE_ALIGN_RAMP) ||
              (fault->fault_state == DIAG_STATE_ALIGN_HOLD) ||
              (fault->fault_state == DIAG_STATE_ALIGN_DOWN)) &&
             (diag->current_over_consecutive_max > 1u)) {
    diag->classification = "CURRENT_TRIP_REAL_ALIGNMENT_CURRENT";
  } else if (fault->trip_iw && !fault->trip_iu && !fault->trip_iv &&
             (fabsf(fault->iu) < 0.35f) && (fabsf(fault->iv) < 0.35f)) {
    diag->classification = "CURRENT_TRIP_RECONSTRUCTED_IW_NOISE";
  } else if (((fault->fault_state == DIAG_STATE_SWEEP_ZERO_RAMP) ||
              (fault->fault_state == DIAG_STATE_SWEEP_ZERO_SETTLE) ||
              (fault->fault_state == DIAG_STATE_SWEEP_ZERO_SAMPLE)) &&
             (diag->current_over_consecutive_max <= 1u)) {
    diag->classification = "CURRENT_TRIP_ADC_OUTLIER";
  } else {
    diag->classification = "CURRENT_TRIP_UNCLASSIFIED";
  }
}

static uint32_t float_to_scaled_u32(float value, float scale)
{
  if (value <= 0.0f) {
    return 0u;
  }
  return (uint32_t)((value * scale) + 0.5f);
}

static int32_t float_to_scaled_i32(float value, float scale)
{
  if (value >= 0.0f) {
    return (int32_t)((value * scale) + 0.5f);
  }
  return -(int32_t)(((-value) * scale) + 0.5f);
}

static float wrap_0_2pi_f(float x)
{
  while (x >= TWO_PI_F) {
    x -= TWO_PI_F;
  }
  while (x < 0.0f) {
    x += TWO_PI_F;
  }
  return x;
}

static uint32_t abs_i32_to_u32(int32_t x)
{
  return (x < 0) ? (uint32_t)(-x) : (uint32_t)x;
}

static void i64_to_dec(char *buf, size_t len, int64_t value)
{
  char tmp[24];
  uint32_t idx = 0u;
  uint32_t out = 0u;
  const bool neg = value < 0;
  uint64_t mag = neg ? ((uint64_t)(-(value + 1)) + 1u) : (uint64_t)value;

  if (len == 0u) {
    return;
  }

  do {
    tmp[idx++] = (char)('0' + (mag % 10u));
    mag /= 10u;
  } while ((mag != 0u) && (idx < sizeof(tmp)));

  if (neg && out < (len - 1u)) {
    buf[out++] = '-';
  }
  while ((idx > 0u) && (out < (len - 1u))) {
    buf[out++] = tmp[--idx];
  }
  buf[out] = '\0';
}

static int64_t positive_mod_i64(int64_t x, int64_t mod)
{
  int64_t r = x % mod;
  if (r < 0) {
    r += mod;
  }
  return r;
}

static float angle_delta_abs(float a, float b)
{
  float d = a - b;
  while (d > 3.14159265358979323846f) {
    d -= TWO_PI_F;
  }
  while (d < -3.14159265358979323846f) {
    d += TWO_PI_F;
  }
  return fabsf(d);
}

static void open_loop_print_sample(const OpenLoopMonitorSample *mon)
{
  char line[768];
  char encoder_accum_s[24];
  const uint32_t theta_e_milli = float_to_scaled_u32(mon->theta_e, 1000.0f);
  const int32_t vd_milli = float_to_scaled_i32(mon->vd, 1000.0f);
  const int32_t vq_milli = float_to_scaled_i32(mon->vq, 1000.0f);
  const int32_t iu_milli = float_to_scaled_i32(mon->iu, 1000.0f);
  const int32_t iv_milli = float_to_scaled_i32(mon->iv, 1000.0f);
  const int32_t iw_milli = float_to_scaled_i32(mon->iw, 1000.0f);
  const int32_t id_milli = float_to_scaled_i32(mon->id, 1000.0f);
  const int32_t iq_milli = float_to_scaled_i32(mon->iq, 1000.0f);
  const int32_t rpm_centi = float_to_scaled_i32(mon->mechanical_rpm_filtered, 100.0f);
  const uint32_t vbus_centi = float_to_scaled_u32(mon->vbus_v, 100.0f);
  const int32_t id_counts_centi =
      (g_drv_test.current_amp_per_count > 0.0f)
          ? float_to_scaled_i32(mon->id / g_drv_test.current_amp_per_count, 100.0f)
          : 0;
  const uint32_t edge_us_milli =
      float_to_scaled_u32(g_drv_test.distance_to_nearest_pwm_edge_us, 1000.0f);
  i64_to_dec(encoder_accum_s, sizeof(encoder_accum_s), mon->encoder_accum);
  snprintf(line,
           sizeof(line),
           "d_axis: elapsed_ms=%lu state=%s test_point_index=%lu vd=%s%lu.%03lu vq=%s%lu.%03lu encoder_accum=%s encoder_motion_counts=%lu theta_e=%lu.%03lu raw_u=%u raw_v=%u iu=%s%lu.%03lu iv=%s%lu.%03lu iw=%s%lu.%03lu id=%s%lu.%03lu iq=%s%lu.%03lu id_effective_counts=%s%lu.%02lu CCR1=%lu CCR2=%lu CCR3=%lu CCR4=%lu min_phase_edge_distance_us=%lu.%03lu mechanical_rpm=%s%lu.%02lu vbus_v=%lu.%02lu nFAULT=%lu drv0_status1=0x%04X drv1_status1=0x%04X control2_drv0=0x%04X control2_drv1=0x%04X MOE=%lu EN_GATE=%lu m1_safe=%u ok=%u",
           (unsigned long)mon->elapsed_ms,
           test_state_name(mon->state),
           (unsigned long)mon->test_point_index,
           (vd_milli < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(vd_milli) / 1000u),
           (unsigned long)(abs_i32_to_u32(vd_milli) % 1000u),
           (vq_milli < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(vq_milli) / 1000u),
           (unsigned long)(abs_i32_to_u32(vq_milli) % 1000u),
           encoder_accum_s,
           (unsigned long)g_drv_test.encoder_motion_max_counts,
           (unsigned long)(theta_e_milli / 1000u),
           (unsigned long)(theta_e_milli % 1000u),
           (unsigned int)mon->raw_u,
           (unsigned int)mon->raw_v,
           (iu_milli < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(iu_milli) / 1000u),
           (unsigned long)(abs_i32_to_u32(iu_milli) % 1000u),
           (iv_milli < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(iv_milli) / 1000u),
           (unsigned long)(abs_i32_to_u32(iv_milli) % 1000u),
           (iw_milli < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(iw_milli) / 1000u),
           (unsigned long)(abs_i32_to_u32(iw_milli) % 1000u),
           (id_milli < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(id_milli) / 1000u),
           (unsigned long)(abs_i32_to_u32(id_milli) % 1000u),
           (iq_milli < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(iq_milli) / 1000u),
           (unsigned long)(abs_i32_to_u32(iq_milli) % 1000u),
           (id_counts_centi < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(id_counts_centi) / 100u),
           (unsigned long)(abs_i32_to_u32(id_counts_centi) % 100u),
           (unsigned long)mon->ccr1,
           (unsigned long)mon->ccr2,
           (unsigned long)mon->ccr3,
           (unsigned long)TIM1->CCR4,
           (unsigned long)(edge_us_milli / 1000u),
           (unsigned long)(edge_us_milli % 1000u),
           (rpm_centi < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(rpm_centi) / 100u),
           (unsigned long)(abs_i32_to_u32(rpm_centi) % 100u),
           (unsigned long)(vbus_centi / 100u),
           (unsigned long)(vbus_centi % 100u),
           (unsigned long)mon->nfault,
           (unsigned int)mon->drv0_status1,
           (unsigned int)mon->drv1_status1,
           (unsigned int)mon->drv0_control2,
           (unsigned int)mon->drv1_control2,
           (unsigned long)mon->moe,
           (unsigned long)mon->en_gate_raw,
           (unsigned int)m1_is_safe_off(),
           (unsigned int)mon->ok);
  uart2_printf_line(line);
}

static void drv_bringup_fail(uint32_t step)
{
  g_drv_test.fail_step = step;
  g_drv_test.pass = false;
  g_drv_test.adc_seq_after = drv_bringup_get_adc_seq();
  g_drv_test.adc_seq_growing = g_drv_test.adc_seq_after > g_drv_test.adc_seq_before;
  if (g_axis0.fault_flags == 0u) {
    drv_bringup_mark_fault(AXIS0_FAULT_DRV8301_FAULT);
  }
  g_drv_test.fault_code = g_axis0.fault_flags;
  current_trip_diag_classify();
  (void)drv8301_set_dc_cal(&g_drv0, false, false);
  (void)drv8301_set_dc_cal(&g_drv1, false, false);
  power_stage_force_safe_off_zero_ccr();
  hal_pwm_start_adc_trigger_only();
  power_stage_force_safe_off_zero_ccr();
  m1_force_safe_off();
  drv_bringup_configure_nfault_pull(GPIO_PULLUP);
  HAL_Delay(2u);
  drv_bringup_capture_final_state();
}

static bool power_stage_wait_nfault_release(void)
{
  for (uint32_t ms = 0u; ms <= DRV_NFAULT_RELEASE_TIMEOUT_MS; ++ms) {
    if (nfault_ok()) {
      g_drv_test.nfault_released = true;
      g_drv_test.nfault_release_ms = ms;
      return true;
    }
    HAL_Delay(1u);
  }

  g_drv_test.nfault_release_ms = 0xffffffffu;
  return false;
}

static bool power_stage_collect_adc_offsets(void)
{
  HalAdcSnapshot snap = {0};
  uint32_t last_seq = 0u;
  uint32_t sum_u = 0u;
  uint32_t sum_v = 0u;
  uint64_t sumsq_u = 0u;
  uint64_t sumsq_v = 0u;
  uint16_t u_min = 0xffffu;
  uint16_t v_min = 0xffffu;
  uint16_t u_max = 0u;
  uint16_t v_max = 0u;
  uint16_t last_u = 0u;
  uint16_t last_v = 0u;
  uint32_t count = 0u;
  bool glitch = false;
  const uint32_t start_ms = HAL_GetTick();

  if (hal_adc_get_snapshot(&snap)) {
    last_seq = snap.seq;
  }

  while (count < ADC_OFFSET_SAMPLE_COUNT) {
    if ((HAL_GetTick() - start_ms) > ADC_OFFSET_TIMEOUT_MS) {
      break;
    }
    if (!hal_adc_get_snapshot(&snap) || !snap.valid || snap.seq == last_seq) {
      continue;
    }
    last_seq = snap.seq;

    const uint16_t u = snap.raw_u;
    const uint16_t v = snap.raw_v;
    sum_u += u;
    sum_v += v;
    sumsq_u += (uint64_t)u * (uint64_t)u;
    sumsq_v += (uint64_t)v * (uint64_t)v;
    if (count > 0u) {
      const uint16_t du = (u > last_u) ? (uint16_t)(u - last_u) : (uint16_t)(last_u - u);
      const uint16_t dv = (v > last_v) ? (uint16_t)(v - last_v) : (uint16_t)(last_v - v);
      if (du > 32u || dv > 32u) {
        glitch = true;
      }
    }
    last_u = u;
    last_v = v;
    if (u < u_min) { u_min = u; }
    if (u > u_max) { u_max = u; }
    if (v < v_min) { v_min = v; }
    if (v > v_max) { v_max = v; }
    count++;
  }

  g_drv_test.offset.samples = count;
  if (count == 0u) {
    return false;
  }

  g_drv_test.offset.offset_u = sum_u / count;
  g_drv_test.offset.offset_v = sum_v / count;
  g_drv_test.offset.u_min = u_min;
  g_drv_test.offset.u_max = u_max;
  g_drv_test.offset.v_min = v_min;
  g_drv_test.offset.v_max = v_max;
  g_drv_test.offset.u_noise_pp = (uint16_t)(u_max - u_min);
  g_drv_test.offset.v_noise_pp = (uint16_t)(v_max - v_min);
  const uint16_t max_noise_pp = (g_drv_test.offset.u_noise_pp > g_drv_test.offset.v_noise_pp)
                                    ? g_drv_test.offset.u_noise_pp
                                    : g_drv_test.offset.v_noise_pp;
  g_drv_test.noise_pp_counts = (float)max_noise_pp;
  g_drv_test.noise_pp_amp = g_drv_test.noise_pp_counts *
                            g_drv_test.current_amp_per_count;

  const float mean_u = (float)sum_u / (float)count;
  const float mean_v = (float)sum_v / (float)count;
  float var_u = ((float)sumsq_u / (float)count) - (mean_u * mean_u);
  float var_v = ((float)sumsq_v / (float)count) - (mean_v * mean_v);
  if (var_u < 0.0f) { var_u = 0.0f; }
  if (var_v < 0.0f) { var_v = 0.0f; }
  g_drv_test.dc_noise.samples = count;
  g_drv_test.dc_noise.mean_u = g_drv_test.offset.offset_u;
  g_drv_test.dc_noise.mean_v = g_drv_test.offset.offset_v;
  g_drv_test.dc_noise.u_min = u_min;
  g_drv_test.dc_noise.u_max = u_max;
  g_drv_test.dc_noise.v_min = v_min;
  g_drv_test.dc_noise.v_max = v_max;
  g_drv_test.dc_noise.u_p2p = g_drv_test.offset.u_noise_pp;
  g_drv_test.dc_noise.v_p2p = g_drv_test.offset.v_noise_pp;
  g_drv_test.dc_noise.u_std_counts = sqrtf(var_u);
  g_drv_test.dc_noise.v_std_counts = sqrtf(var_v);
  g_drv_test.dc_noise.u_std_amp = g_drv_test.dc_noise.u_std_counts *
                                  g_drv_test.current_amp_per_count;
  g_drv_test.dc_noise.v_std_amp = g_drv_test.dc_noise.v_std_counts *
                                  g_drv_test.current_amp_per_count;
  g_drv_test.dc_noise.u_rms_noise_counts = g_drv_test.dc_noise.u_std_counts;
  g_drv_test.dc_noise.v_rms_noise_counts = g_drv_test.dc_noise.v_std_counts;
  g_drv_test.dc_noise.u_range99_counts =
      (uint16_t)float_to_scaled_u32(5.152f * g_drv_test.dc_noise.u_std_counts, 1.0f);
  g_drv_test.dc_noise.v_range99_counts =
      (uint16_t)float_to_scaled_u32(5.152f * g_drv_test.dc_noise.v_std_counts, 1.0f);
  g_drv_test.dc_noise.glitch_detected = glitch;
  return (count == ADC_OFFSET_SAMPLE_COUNT) &&
         (u_min > 0u) && (u_max < 4095u) &&
         (v_min > 0u) && (v_max < 4095u) &&
         (g_drv_test.dc_noise.u_std_counts <= DC_CAL_STD_MAX_COUNTS) &&
         (g_drv_test.dc_noise.v_std_counts <= DC_CAL_STD_MAX_COUNTS) &&
         !glitch;
}

static bool power_stage_configure_drivers(void)
{
  drv8301_prepare_axis(&g_drv0, 0u);
  drv8301_prepare_axis(&g_drv1, 1u);

  g_drv_test.drv0_cfg = drv8301_configure_and_verify(&g_drv0, &g_drv_test.drv0_regs);
  g_drv_test.drv1_cfg = drv8301_configure_and_verify(&g_drv1, &g_drv_test.drv1_regs);
  g_drv_test.expected_control2 =
      drv8301_make_control2(DRV8301_SHUNT_GAIN_40V_PER_V, false, false);

  if (g_drv_test.drv0_cfg) {
    g_drv_test.drv0_cfg =
        drv8301_set_shunt_amp_gain(&g_drv0, DRV8301_SHUNT_GAIN_40V_PER_V) &&
        drv8301_read_registers(&g_drv0, &g_drv_test.drv0_regs);
  }
  if (g_drv_test.drv1_cfg) {
    g_drv_test.drv1_cfg =
        drv8301_set_shunt_amp_gain(&g_drv1, DRV8301_SHUNT_GAIN_40V_PER_V) &&
        drv8301_read_registers(&g_drv1, &g_drv_test.drv1_regs);
  }

  g_drv_test.actual_control2_drv0 = g_drv_test.drv0_regs.control2;
  g_drv_test.actual_control2_drv1 = g_drv_test.drv1_regs.control2;
  g_drv_test.gain_field_drv0 =
      drv8301_control2_gain_field(g_drv_test.actual_control2_drv0);
  g_drv_test.gain_field_drv1 =
      drv8301_control2_gain_field(g_drv_test.actual_control2_drv1);
  g_drv_test.gain40_readback_ok =
      (g_drv_test.actual_control2_drv0 == g_drv_test.expected_control2) &&
      (g_drv_test.actual_control2_drv1 == g_drv_test.expected_control2) &&
      (g_drv_test.gain_field_drv0 == DRV8301_SHUNT_GAIN_40V_PER_V) &&
      (g_drv_test.gain_field_drv1 == DRV8301_SHUNT_GAIN_40V_PER_V);

  g_drv_test.drv0_status_ok = g_drv_test.drv0_cfg &&
                              !drv_status_has_fault(g_drv_test.drv0_regs.status1,
                                                    g_drv_test.drv0_regs.status2);
  g_drv_test.drv1_status_ok = g_drv_test.drv1_cfg &&
                              !drv_status_has_fault(g_drv_test.drv1_regs.status1,
                                                    g_drv_test.drv1_regs.status2);
  return g_drv_test.drv0_status_ok &&
         g_drv_test.drv1_status_ok &&
         g_drv_test.gain40_readback_ok;
}

static bool power_stage_collect_dc_cal_offsets(void)
{
  power_stage_disable_six_outputs();
  __HAL_TIM_MOE_DISABLE(&htim1);

  if (!drv8301_set_dc_cal(&g_drv0, true, true) ||
      !drv8301_set_dc_cal(&g_drv1, true, true)) {
    return false;
  }

  HAL_Delay(1u);
  g_drv_test.dc_cal_offsets_pass = power_stage_collect_adc_offsets();

  const bool clear_ok =
      drv8301_set_dc_cal(&g_drv0, false, false) &&
      drv8301_set_dc_cal(&g_drv1, false, false) &&
      drv8301_read_registers(&g_drv0, &g_drv_test.drv0_regs) &&
      drv8301_read_registers(&g_drv1, &g_drv_test.drv1_regs);

  g_drv_test.actual_control2_drv0 = g_drv_test.drv0_regs.control2;
  g_drv_test.actual_control2_drv1 = g_drv_test.drv1_regs.control2;
  g_drv_test.gain_field_drv0 =
      drv8301_control2_gain_field(g_drv_test.actual_control2_drv0);
  g_drv_test.gain_field_drv1 =
      drv8301_control2_gain_field(g_drv_test.actual_control2_drv1);
  g_drv_test.dc_cal_clear_ok =
      clear_ok &&
      ((g_drv_test.actual_control2_drv0 &
        (DRV8301_CONTROL2_DC_CAL_CH1 | DRV8301_CONTROL2_DC_CAL_CH2)) == 0u) &&
      ((g_drv_test.actual_control2_drv1 &
        (DRV8301_CONTROL2_DC_CAL_CH1 | DRV8301_CONTROL2_DC_CAL_CH2)) == 0u) &&
      (g_drv_test.gain_field_drv0 == DRV8301_SHUNT_GAIN_40V_PER_V) &&
      (g_drv_test.gain_field_drv1 == DRV8301_SHUNT_GAIN_40V_PER_V);

  return g_drv_test.dc_cal_offsets_pass && g_drv_test.dc_cal_clear_ok;
}

static bool power_stage_check_adc_sample_timing(void)
{
  const uint32_t arr = TIM1->ARR;
  if (arr > ADC_TRIGGER_SAFE_OFFSET_COUNTS) {
    TIM1->CCR4 = arr - ADC_TRIGGER_SAFE_OFFSET_COUNTS;
  }

  g_drv_test.pwm_period_us =
      (2.0f * (float)(arr + 1u) * 1000000.0f) / TIM1_TIMER_CLOCK_HZ;
  return power_stage_update_phase_edge_timing();
}

static uint32_t abs_diff_u32(uint32_t a, uint32_t b)
{
  return (a > b) ? (a - b) : (b - a);
}

static float max3f_local(float a, float b, float c)
{
  float m = (a > b) ? a : b;
  return (m > c) ? m : c;
}

static float min3f_local(float a, float b, float c)
{
  float m = (a < b) ? a : b;
  return (m < c) ? m : c;
}

static uint32_t ccr_span_u32(uint32_t a, uint32_t b, uint32_t c)
{
  const uint32_t max_ab = (a > b) ? a : b;
  const uint32_t max_abc = (max_ab > c) ? max_ab : c;
  const uint32_t min_ab = (a < b) ? a : b;
  const uint32_t min_abc = (min_ab < c) ? min_ab : c;
  return max_abc - min_abc;
}

static void voltage_path_diag_compute(VoltagePathDiag *diag,
                                      float vd,
                                      float vq,
                                      float theta_e,
                                      float vbus_v)
{
  if (diag == NULL) {
    return;
  }

  memset(diag, 0, sizeof(*diag));
  diag->vd = vd;
  diag->vq = vq;
  diag->theta_e = theta_e;
  diag->measured_vbus = vbus_v;
  const float c = cosf(theta_e);
  const float s = sinf(theta_e);
  diag->v_alpha = vd * c - vq * s;
  diag->v_beta = vd * s + vq * c;
  diag->normalized_alpha = (vbus_v > 1.0f) ? (diag->v_alpha / vbus_v) : 0.0f;
  diag->normalized_beta = (vbus_v > 1.0f) ? (diag->v_beta / vbus_v) : 0.0f;
  diag->vbus_divide_count = 1u;
  diag->phase_a = diag->v_alpha;
  diag->phase_b = -0.5f * diag->v_alpha + 0.86602540378f * diag->v_beta;
  diag->phase_c = -0.5f * diag->v_alpha - 0.86602540378f * diag->v_beta;
  diag->common_mode =
      -0.5f * (max3f_local(diag->phase_a, diag->phase_b, diag->phase_c) +
               min3f_local(diag->phase_a, diag->phase_b, diag->phase_c));

  const SvpwmDuty duty = svpwm_generate(diag->v_alpha, diag->v_beta, vbus_v);
  diag->duty_a = duty.duty_a;
  diag->duty_b = duty.duty_b;
  diag->duty_c = duty.duty_c;
  diag->ccr1 = duty_to_ccr_limited(duty.duty_a);
  diag->ccr2 = duty_to_ccr_limited(duty.duty_b);
  diag->ccr3 = duty_to_ccr_limited(duty.duty_c);
  const int32_t half = (int32_t)tim1_half_ccr();
  diag->ccr1_offset = (int32_t)diag->ccr1 - half;
  diag->ccr2_offset = (int32_t)diag->ccr2 - half;
  diag->ccr3_offset = (int32_t)diag->ccr3 - half;
  diag->ccr_span = ccr_span_u32(diag->ccr1, diag->ccr2, diag->ccr3);

  diag->applied_phase_a = ((float)diag->ccr1 / (float)TIM1->ARR - 0.5f) * vbus_v;
  diag->applied_phase_b = ((float)diag->ccr2 / (float)TIM1->ARR - 0.5f) * vbus_v;
  diag->applied_phase_c = ((float)diag->ccr3 / (float)TIM1->ARR - 0.5f) * vbus_v;
  diag->applied_v_alpha =
      (2.0f / 3.0f) *
      (diag->applied_phase_a - 0.5f * diag->applied_phase_b -
       0.5f * diag->applied_phase_c);
  diag->applied_v_beta =
      (0.57735026919f) * (diag->applied_phase_b - diag->applied_phase_c);
  diag->applied_voltage_magnitude =
      sqrtf(diag->applied_v_alpha * diag->applied_v_alpha +
            diag->applied_v_beta * diag->applied_v_beta);
  const float commanded_mag =
      sqrtf(diag->v_alpha * diag->v_alpha + diag->v_beta * diag->v_beta);
  diag->commanded_to_applied_ratio =
      (commanded_mag > 0.001f) ? (diag->applied_voltage_magnitude / commanded_mag) : 1.0f;
  diag->double_scaled =
      (commanded_mag > 0.10f) && (diag->commanded_to_applied_ratio < 0.25f);
  diag->command_too_small = (vd >= 0.499f) && (diag->ccr_span < 50u);
  diag->applied_scale_fail = (vd >= 0.499f) && (diag->applied_voltage_magnitude < 0.40f);
}

static void sample_window_diag_update(void)
{
  SampleWindowDiag *diag = &g_drv_test.sample_window_diag;
  memset(diag, 0, sizeof(*diag));
  diag->sample_cnt = TIM1->CCR4;
  diag->current_tim1_cnt = TIM1->CNT;
  diag->current_tim1_dir = tim1_count_direction();
  diag->pwm_mode = 1u;
  diag->ccer = TIM1->CCER;
  diag->bdtr = TIM1->BDTR;
  diag->ccr1 = TIM1->CCR1;
  diag->ccr2 = TIM1->CCR2;
  diag->ccr3 = TIM1->CCR3;

  const uint32_t ccrs[3] = {TIM1->CCR1, TIM1->CCR2, TIM1->CCR3};
  for (uint32_t i = 0u; i < 3u; ++i) {
    const bool main_enabled = (TIM1->CCER & (TIM_CCER_CC1E << (i * 4u))) != 0u;
    const bool comp_enabled = (TIM1->CCER & (TIM_CCER_CC1NE << (i * 4u))) != 0u;
    const bool moe = (TIM1->BDTR & TIM_BDTR_MOE) != 0u;
    const bool high_active_pwm1 = diag->sample_cnt < ccrs[i];
    const bool in_deadtime = abs_diff_u32(diag->sample_cnt, ccrs[i]) <= 50u;
    if (moe && main_enabled && high_active_pwm1 && !in_deadtime) {
      diag->sample_high_side_mask |= (1u << i);
    }
    if (moe && comp_enabled && !high_active_pwm1 && !in_deadtime) {
      diag->sample_low_side_mask |= (1u << i);
      diag->low_side_active_count++;
    }
    if (in_deadtime) {
      diag->deadtime_mask |= (1u << i);
    }
  }
  diag->current_sample_window_valid = diag->low_side_active_count >= 2u;
  g_drv_test.low_side_sample_window_valid = diag->current_sample_window_valid;
}

static bool power_stage_update_phase_edge_timing(void)
{
  const uint32_t ccr4 = TIM1->CCR4;
  uint32_t min_counts = abs_diff_u32(ccr4, TIM1->CCR1);
  const uint32_t d2 = abs_diff_u32(ccr4, TIM1->CCR2);
  const uint32_t d3 = abs_diff_u32(ccr4, TIM1->CCR3);
  if (d2 < min_counts) { min_counts = d2; }
  if (d3 < min_counts) { min_counts = d3; }

  const float min_us = ((float)min_counts * 1000000.0f) / TIM1_TIMER_CLOCK_HZ;
  g_drv_test.adc_trigger_position_counts = ccr4;
  if (g_drv_test.min_distance_to_phase_edge_counts == 0u ||
      min_counts < g_drv_test.min_distance_to_phase_edge_counts) {
    g_drv_test.min_distance_to_phase_edge_counts = min_counts;
    g_drv_test.min_distance_to_phase_edge_us = min_us;
  }
  g_drv_test.distance_to_nearest_pwm_edge_us = min_us;
  g_drv_test.adc_phase_edge_timing_ok = min_us >= ADC_SAMPLE_EDGE_MARGIN_MIN_US;
  g_drv_test.adc_sample_timing_ok = g_drv_test.adc_phase_edge_timing_ok;
  return g_drv_test.adc_phase_edge_timing_ok;
}

static bool open_loop_wait_next_adc_sample(uint32_t *last_seq, HalAdcSnapshot *snap)
{
  const uint32_t start_ms = HAL_GetTick();
  while ((HAL_GetTick() - start_ms) < 5u) {
    if (hal_adc_get_snapshot(snap) && snap->valid && snap->seq != *last_seq) {
      *last_seq = snap->seq;
      return true;
    }
  }
  return false;
}

static bool open_loop_capture_and_print(uint32_t elapsed_ms,
                                        uint32_t state,
                                        float theta_m,
                                        float theta_e,
                                        float electrical_offset_runtime,
                                        float vd,
                                        float vq,
                                        float mechanical_rpm_filtered,
                                        const HalAdcSnapshot *snap,
                                        bool theta_ok,
                                        bool print_now)
{
  bool drv0_ok = true;
  bool drv1_ok = true;
  OpenLoopMonitorSample live = {0};
  OpenLoopMonitorSample *store = 0;

  if (print_now) {
    drv0_ok = drv8301_read_status(&g_drv0);
    drv1_ok = drv8301_read_status(&g_drv1);
  }

  live.elapsed_ms = elapsed_ms;
  live.state = state;
  live.test_point_index = g_drv_test.sweep_point_count;
  live.seq = snap->seq;
  live.theta_m = theta_m;
  live.theta_e = theta_e;
  live.electrical_offset_runtime = electrical_offset_runtime;
  live.vd = vd;
  live.vq = vq;
  const CurrentDqSample current = current_observe_calculate(snap, theta_e);
  current_observe_lpf_update(&current);
  live.iu = g_drv_test.iu_lpf;
  live.iv = g_drv_test.iv_lpf;
  live.iw = g_drv_test.iw_lpf;
  live.id = g_drv_test.id_lpf;
  live.iq = g_drv_test.iq_lpf;
  live.mechanical_rpm_filtered = mechanical_rpm_filtered;
  live.vbus_v = board_read_vbus_v();
  live.tim3_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  live.encoder_delta = g_encoder_last_delta;
  live.encoder_accum = g_encoder_accum;
  live.raw_u = snap->raw_u;
  live.raw_v = snap->raw_v;
  live.drv0_status1 = g_drv0.status.status1_raw;
  live.drv1_status1 = g_drv1.status.status1_raw;
  live.drv0_control2 = g_drv_test.actual_control2_drv0;
  live.drv1_control2 = g_drv_test.actual_control2_drv1;
  live.ccr1 = TIM1->CCR1;
  live.ccr2 = TIM1->CCR2;
  live.ccr3 = TIM1->CCR3;
  live.ccer = TIM1->CCER;
  live.bdtr = TIM1->BDTR;
  live.moe = ((TIM1->BDTR & TIM_BDTR_MOE) != 0u) ? 1u : 0u;
  live.en_gate_raw = gate_raw_is_high() ? 1u : 0u;
  live.nfault = board_read_drv_nfault() ? 1u : 0u;

  if (snap->raw_u < g_drv_test.run_raw_u_min) { g_drv_test.run_raw_u_min = snap->raw_u; }
  if (snap->raw_u > g_drv_test.run_raw_u_max) { g_drv_test.run_raw_u_max = snap->raw_u; }
  if (snap->raw_v < g_drv_test.run_raw_v_min) { g_drv_test.run_raw_v_min = snap->raw_v; }
  if (snap->raw_v > g_drv_test.run_raw_v_max) { g_drv_test.run_raw_v_max = snap->raw_v; }

  const bool raw_ok = (snap->raw_u > CURRENT_RAW_MIN_SAFE_COUNT) &&
                      (snap->raw_u < CURRENT_RAW_MAX_SAFE_COUNT) &&
                      (snap->raw_v > CURRENT_RAW_MIN_SAFE_COUNT) &&
                      (snap->raw_v < CURRENT_RAW_MAX_SAFE_COUNT);
  const uint32_t u_dev = (snap->raw_u > g_drv_test.offset.offset_u)
                             ? (uint32_t)(snap->raw_u - g_drv_test.offset.offset_u)
                             : (uint32_t)(g_drv_test.offset.offset_u - snap->raw_u);
  const uint32_t v_dev = (snap->raw_v > g_drv_test.offset.offset_v)
                             ? (uint32_t)(snap->raw_v - g_drv_test.offset.offset_v)
                             : (uint32_t)(g_drv_test.offset.offset_v - snap->raw_v);
  if (u_dev > g_drv_test.raw_current_max_deviation) {
    g_drv_test.raw_current_max_deviation = u_dev;
  }
  if (v_dev > g_drv_test.raw_current_max_deviation) {
    g_drv_test.raw_current_max_deviation = v_dev;
  }
  if (current.magnitude_max > g_drv_test.current_max_a) {
    g_drv_test.current_max_a = current.magnitude_max;
  }
  const float kcl_residual = fabsf(current.iu + current.iv + current.iw);
  if (kcl_residual > g_drv_test.kcl_residual_max) {
    g_drv_test.kcl_residual_max = kcl_residual;
  }
  if (state == TEST_STATE_VD_SAMPLE) {
    current_observe_stats_update(&g_drv_test.current_stats, &current);
  }
  const bool current_dev_ok = (u_dev <= COMM_RAW_CURRENT_MAX_DEV_COUNTS) &&
                              (v_dev <= COMM_RAW_CURRENT_MAX_DEV_COUNTS);
  const bool current_a_ok =
      (state == TEST_STATE_ALIGN && elapsed_ms < 20u) ||
      (current.magnitude_max <= CURRENT_INSTANT_LIMIT_A);
  const bool phase_edge_ok = power_stage_update_phase_edge_timing();
  const bool vbus_ok = (live.vbus_v >= OPEN_LOOP_VBUS_MIN_V) &&
                       (live.vbus_v <= OPEN_LOOP_VBUS_MAX_V);
  const bool drv_ok = drv0_ok && drv1_ok &&
                      !drv_status_has_fault(g_drv0.status.status1_raw, g_drv0.status.status2_raw) &&
                      !drv_status_has_fault(g_drv1.status.status1_raw, g_drv1.status.status2_raw);
  const bool pwm_ok = ccrs_in_open_loop_range() &&
                      ((TIM1->CCER & POWER_CCER_MASK) == POWER_CCER_MASK) &&
                      ((TIM1->BDTR & TIM_BDTR_MOE) != 0u) &&
                      gate_raw_is_high() && nfault_ok();
  live.ok = raw_ok && current_dev_ok && current_a_ok && vbus_ok && drv_ok && pwm_ok &&
            phase_edge_ok && theta_ok && m1_is_safe_off() &&
            encoder_delta_ok();

  if (print_now) {
    if (g_drv_test.monitor_count < (sizeof(g_drv_test.monitor) / sizeof(g_drv_test.monitor[0]))) {
      store = &g_drv_test.monitor[g_drv_test.monitor_count++];
      *store = live;
    }
    open_loop_print_sample(&live);
  }

  return live.ok;
}

static bool encoder_idle_check(void)
{
  encoder_tracker_reset();
  g_drv_test.enc_idle_min = 0;
  g_drv_test.enc_idle_max = 0;

  const uint32_t start_ms = HAL_GetTick();
  uint32_t last_sample_ms = start_ms;
  while ((HAL_GetTick() - start_ms) <= ENCODER_IDLE_CHECK_MS) {
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - last_sample_ms) < ENCODER_IDLE_SAMPLE_PERIOD_MS) {
      continue;
    }
    last_sample_ms = now_ms;
    const int64_t enc = encoder_tracker_sample();
    if (enc < g_drv_test.enc_idle_min) { g_drv_test.enc_idle_min = enc; }
    if (enc > g_drv_test.enc_idle_max) { g_drv_test.enc_idle_max = enc; }
    if (!encoder_delta_ok()) {
      return false;
    }
  }

  g_drv_test.enc_idle_noise_pp =
      abs_i32_to_u32(g_drv_test.enc_idle_max - g_drv_test.enc_idle_min);
  return g_drv_test.enc_idle_noise_pp <= ENCODER_IDLE_NOISE_MAX_COUNTS;
}

static void encoder_apply_alpha_beta_svpwm(float v_alpha, float v_beta, float vbus_v)
{
  const SvpwmDuty duty = svpwm_generate(v_alpha, v_beta, vbus_v);
  TIM1->CCR1 = duty_to_ccr_limited(duty.duty_a);
  TIM1->CCR2 = duty_to_ccr_limited(duty.duty_b);
  TIM1->CCR3 = duty_to_ccr_limited(duty.duty_c);
}

static void encoder_apply_dq_svpwm(float theta_e, float vd, float vq, float vbus_v)
{
  const float limited_vd = clampf(vd, -COMM_VD_MAX_ABS_V, COMM_VD_MAX_ABS_V);
  const float limited_vq = clampf(vq, -COMM_VQ_MAX_ABS_V, COMM_VQ_MAX_ABS_V);
  const float c = cosf(theta_e);
  const float s = sinf(theta_e);
  const float v_alpha = limited_vd * c - limited_vq * s;
  const float v_beta = limited_vd * s + limited_vq * c;
  encoder_apply_alpha_beta_svpwm(v_alpha, v_beta, vbus_v);
}

static float encoder_theta_m_from_accum(int64_t encoder_accum)
{
  const int64_t mod_count = positive_mod_i64(encoder_accum, COMM_ENCODER_CPR);
  return TWO_PI_F * (float)mod_count / (float)COMM_ENCODER_CPR;
}

static float encoder_theta_e_from_theta_m(float theta_m, float electrical_offset_runtime)
{
  return wrap_0_2pi_f((float)COMM_ENCODER_DIRECTION *
                      (float)COMM_POLE_PAIRS *
                      theta_m +
                      electrical_offset_runtime);
}

static void current_observe_configure_scale(void)
{
  g_drv_test.current_shunt_ohm = CURRENT_SHUNT_OHM;
  g_drv_test.current_amp_gain = CURRENT_AMP_GAIN_V_PER_V;
  g_drv_test.adc_vref_v = CURRENT_ADC_VREF_V;
  g_drv_test.current_polarity_u = CURRENT_POLARITY_U;
  g_drv_test.current_polarity_v = CURRENT_POLARITY_V;
  g_drv_test.current_scale_known =
      (g_drv_test.current_shunt_ohm > 0.0f) &&
      (g_drv_test.current_amp_gain > 0.0f) &&
      (g_drv_test.adc_vref_v > 0.0f);
  if (g_drv_test.current_scale_known) {
    g_drv_test.current_amp_per_count =
        g_drv_test.adc_vref_v /
        (CURRENT_ADC_FULL_SCALE_COUNTS *
         g_drv_test.current_amp_gain *
         g_drv_test.current_shunt_ohm);
  } else {
    g_drv_test.current_amp_per_count = 0.0f;
  }
}

static CurrentDqSample current_observe_calculate(const HalAdcSnapshot *snap, float theta_e)
{
  CurrentDqSample sample = {0};
  const float du = (float)((int32_t)snap->raw_u - (int32_t)g_drv_test.offset.offset_u);
  const float dv = (float)((int32_t)snap->raw_v - (int32_t)g_drv_test.offset.offset_v);
  sample.iu = (float)g_drv_test.current_polarity_u * du * g_drv_test.current_amp_per_count;
  sample.iv = (float)g_drv_test.current_polarity_v * dv * g_drv_test.current_amp_per_count;
  sample.iw = -(sample.iu + sample.iv);
  sample.i_alpha = sample.iu;
  sample.i_beta = (sample.iu + 2.0f * sample.iv) / SQRT3_F;
  const float c = cosf(theta_e);
  const float s = sinf(theta_e);
  sample.id = sample.i_alpha * c + sample.i_beta * s;
  sample.iq = -sample.i_alpha * s + sample.i_beta * c;
  sample.magnitude_max = fabsf(sample.iu);
  if (fabsf(sample.iv) > sample.magnitude_max) { sample.magnitude_max = fabsf(sample.iv); }
  if (fabsf(sample.iw) > sample.magnitude_max) { sample.magnitude_max = fabsf(sample.iw); }
  if (fabsf(sample.id) > sample.magnitude_max) { sample.magnitude_max = fabsf(sample.id); }
  if (fabsf(sample.iq) > sample.magnitude_max) { sample.magnitude_max = fabsf(sample.iq); }
  return sample;
}

static void current_observe_lpf_update(const CurrentDqSample *sample)
{
  g_drv_test.iu_lpf += CURRENT_LPF_ALPHA * (sample->iu - g_drv_test.iu_lpf);
  g_drv_test.iv_lpf += CURRENT_LPF_ALPHA * (sample->iv - g_drv_test.iv_lpf);
  g_drv_test.iw_lpf += CURRENT_LPF_ALPHA * (sample->iw - g_drv_test.iw_lpf);
  g_drv_test.id_lpf += CURRENT_LPF_ALPHA * (sample->id - g_drv_test.id_lpf);
  g_drv_test.iq_lpf += CURRENT_LPF_ALPHA * (sample->iq - g_drv_test.iq_lpf);
}

static void current_observe_stats_reset(CurrentObserveStats *stats)
{
  memset(stats, 0, sizeof(*stats));
  stats->iu_min = stats->iv_min = stats->iw_min = stats->id_min = stats->iq_min = 1000000.0f;
  stats->iu_max = stats->iv_max = stats->iw_max = stats->id_max = stats->iq_max = -1000000.0f;
}

static void current_observe_stats_update(CurrentObserveStats *stats, const CurrentDqSample *sample)
{
  stats->active = true;
  stats->samples++;
  if (sample->iu < stats->iu_min) { stats->iu_min = sample->iu; }
  if (sample->iu > stats->iu_max) { stats->iu_max = sample->iu; }
  if (sample->iv < stats->iv_min) { stats->iv_min = sample->iv; }
  if (sample->iv > stats->iv_max) { stats->iv_max = sample->iv; }
  if (sample->iw < stats->iw_min) { stats->iw_min = sample->iw; }
  if (sample->iw > stats->iw_max) { stats->iw_max = sample->iw; }
  if (sample->id < stats->id_min) { stats->id_min = sample->id; }
  if (sample->id > stats->id_max) { stats->id_max = sample->id; }
  if (sample->iq < stats->iq_min) { stats->iq_min = sample->iq; }
  if (sample->iq > stats->iq_max) { stats->iq_max = sample->iq; }
  stats->iu_sum += sample->iu;
  stats->iv_sum += sample->iv;
  stats->iw_sum += sample->iw;
  stats->id_sum += sample->id;
  stats->iq_sum += sample->iq;
  stats->id_sumsq += sample->id * sample->id;
  stats->iq_sumsq += sample->iq * sample->iq;
  if (sample->magnitude_max > stats->current_magnitude_max) {
    stats->current_magnitude_max = sample->magnitude_max;
  }
}

static void current_observe_stats_finalize(const CurrentObserveStats *stats,
                                           float *id_mean,
                                           float *iq_mean,
                                           float *id_std,
                                           float *iq_std)
{
  if (stats == 0 || stats->samples == 0u) {
    if (id_mean != 0) { *id_mean = 0.0f; }
    if (iq_mean != 0) { *iq_mean = 0.0f; }
    if (id_std != 0) { *id_std = 0.0f; }
    if (iq_std != 0) { *iq_std = 0.0f; }
    return;
  }

  const float inv = 1.0f / (float)stats->samples;
  const float id_m = stats->id_sum * inv;
  const float iq_m = stats->iq_sum * inv;
  float id_var = stats->id_sumsq * inv - id_m * id_m;
  float iq_var = stats->iq_sumsq * inv - iq_m * iq_m;
  if (id_var < 0.0f) { id_var = 0.0f; }
  if (iq_var < 0.0f) { iq_var = 0.0f; }
  if (id_mean != 0) { *id_mean = id_m; }
  if (iq_mean != 0) { *iq_mean = iq_m; }
  if (id_std != 0) { *id_std = sqrtf(id_var); }
  if (iq_std != 0) { *iq_std = sqrtf(iq_var); }
}

static void sample_position_finalize(SamplePositionResult *pos)
{
  if (pos == 0 || pos->stats.samples == 0u) {
    return;
  }

  const float inv = 1.0f / (float)pos->stats.samples;
  pos->iu_mean = pos->stats.iu_sum * inv;
  pos->iv_mean = pos->stats.iv_sum * inv;
  pos->iw_mean = pos->stats.iw_sum * inv;
  current_observe_stats_finalize(&pos->stats,
                                 &pos->id_mean,
                                 &pos->iq_mean,
                                 &pos->id_std,
                                 &pos->iq_std);
  pos->id_effective_counts =
      (g_drv_test.current_amp_per_count > 0.0f)
          ? (pos->id_mean / g_drv_test.current_amp_per_count)
          : 0.0f;
  pos->iq_over_id_ratio =
      (fabsf(pos->id_mean) > 0.001f)
          ? (fabsf(pos->iq_mean) / fabsf(pos->id_mean))
          : 999.0f;
  pos->current_magnitude_max = pos->stats.current_magnitude_max;
  pos->valid = pos->edge_timing_ok &&
               pos->id_mean > 0.0f &&
               fabsf(pos->id_mean) >= D_AXIS_ID_HOLD_A &&
               fabsf(pos->id_effective_counts) >= CURRENT_MIN_EFFECTIVE_COUNTS &&
               pos->iq_over_id_ratio < D_AXIS_IQ_OVER_ID_RATIO_MAX &&
               pos->current_magnitude_max <= D_AXIS_CURRENT_LIMIT_A &&
               pos->encoder_motion_counts <= D_AXIS_MOTION_MAX_COUNTS &&
               pos->raw_u_min > CURRENT_RAW_MIN_SAFE_COUNT &&
               pos->raw_u_max < CURRENT_RAW_MAX_SAFE_COUNT &&
               pos->raw_v_min > CURRENT_RAW_MIN_SAFE_COUNT &&
               pos->raw_v_max < CURRENT_RAW_MAX_SAFE_COUNT;
}

static float histogram_percentile_current(const uint16_t *bins,
                                          uint32_t bin_count,
                                          uint32_t sample_count,
                                          uint32_t permille)
{
  if (bins == 0 || bin_count == 0u || sample_count == 0u) {
    return 0.0f;
  }

  uint32_t rank = (sample_count * permille + 999u) / 1000u;
  if (rank == 0u) {
    rank = 1u;
  }

  uint32_t accum = 0u;
  for (uint32_t i = 0u; i < bin_count; ++i) {
    accum += bins[i];
    if (accum >= rank) {
      return (float)i / D_AXIS_PHASE_CURRENT_HIST_SCALE;
    }
  }
  return (float)(bin_count - 1u) / D_AXIS_PHASE_CURRENT_HIST_SCALE;
}

static void sweep_point_finalize(SweepPointResult *pt)
{
  if (pt == 0 || pt->stats.samples == 0u) {
    return;
  }

  const float inv = 1.0f / (float)pt->stats.samples;
  pt->iu_mean = pt->stats.iu_sum * inv;
  pt->iv_mean = pt->stats.iv_sum * inv;
  pt->iw_mean = pt->stats.iw_sum * inv;
  current_observe_stats_finalize(&pt->stats,
                                 &pt->id_mean,
                                 &pt->iq_mean,
                                 &pt->id_std,
                                 &pt->iq_std);
  pt->id_effective_counts =
      (g_drv_test.current_amp_per_count > 0.0f)
          ? (pt->id_mean / g_drv_test.current_amp_per_count)
          : 0.0f;
  pt->iq_effective_counts =
      (g_drv_test.current_amp_per_count > 0.0f)
          ? (pt->iq_mean / g_drv_test.current_amp_per_count)
          : 0.0f;
  pt->iq_over_id_ratio =
      (fabsf(pt->id_mean) > 0.001f)
          ? (fabsf(pt->iq_mean) / fabsf(pt->id_mean))
          : 999.0f;
  pt->invalid_reason = 0u;
  if (pt->id_mean < 0.0f) {
    pt->invalid_reason |= SWEEP_INVALID_ID_NEGATIVE;
  }
  if (fabsf(pt->id_effective_counts) < D_AXIS_VALID_MIN_COUNTS) {
    pt->invalid_reason |= SWEEP_INVALID_ID_COUNTS_TOO_SMALL;
  }
  if (pt->iq_over_id_ratio >= D_AXIS_IQ_OVER_ID_RATIO_MAX) {
    pt->invalid_reason |= SWEEP_INVALID_IQ_ID_RATIO_TOO_LARGE;
  }
  if (pt->encoder_motion_counts > D_AXIS_MOTION_MAX_COUNTS) {
    pt->invalid_reason |= SWEEP_INVALID_ENCODER_MOVED;
  }
  if (pt->raw_u_min <= CURRENT_RAW_MIN_SAFE_COUNT ||
      pt->raw_u_max >= CURRENT_RAW_MAX_SAFE_COUNT ||
      pt->raw_v_min <= CURRENT_RAW_MIN_SAFE_COUNT ||
      pt->raw_v_max >= CURRENT_RAW_MAX_SAFE_COUNT) {
    pt->invalid_reason |= SWEEP_INVALID_ADC_INVALID;
  }
  if (pt->faulted) {
    pt->invalid_reason |= SWEEP_INVALID_SAFETY_FAULT;
  }
  pt->valid = (pt->vd_command > 0.001f) && (pt->invalid_reason == 0u);
}

static void sweep_fit_phase_resistance(void)
{
  float sx = 0.0f;
  float sy = 0.0f;
  float sxx = 0.0f;
  float sxy = 0.0f;
  float y_mean = 0.0f;
  uint32_t n = 0u;
  bool monotonic = true;
  float prev_id = -1000000.0f;

  g_drv_test.valid_fit_point_count = 0u;
  g_drv_test.first_reliable_current_voltage = PHASE_RESISTANCE_UNSET;
  g_drv_test.first_reliable_point_index = 0xffffffffu;
  g_drv_test.phase_resistance_est_ohm = PHASE_RESISTANCE_UNSET;
  g_drv_test.inverter_voltage_offset_est_v = 0.0f;
  g_drv_test.fit_r_squared = 0.0f;
  g_drv_test.voltage_required_for_0p2A_est = PHASE_RESISTANCE_UNSET;
  g_drv_test.voltage_required_for_0p3A_est = PHASE_RESISTANCE_UNSET;

  for (uint32_t i = 0u; i < g_drv_test.sweep_point_count; ++i) {
    SweepPointResult *pt = &g_drv_test.sweep_points[i];
    if (!pt->valid) {
      continue;
    }
    if (pt->id_mean + 0.02f < prev_id) {
      monotonic = false;
    }
    prev_id = pt->id_mean;
    if (fabsf(pt->id_effective_counts) >= D_AXIS_RELIABLE_MIN_COUNTS &&
        g_drv_test.first_reliable_current_voltage < 0.0f) {
      g_drv_test.first_reliable_current_voltage = pt->vd_command;
      g_drv_test.first_reliable_point_index = i;
    }
    if (pt->vd_command <= 0.001f) {
      continue;
    }
    sx += pt->id_mean;
    sy += pt->vd_command;
    sxx += pt->id_mean * pt->id_mean;
    sxy += pt->id_mean * pt->vd_command;
    y_mean += pt->vd_command;
    n++;
  }

  g_drv_test.valid_fit_point_count = n;
  g_drv_test.current_monotonic_ok = monotonic && (n > 0u);
  if (n < D_AXIS_SWEEP_MIN_FIT_POINTS) {
    g_drv_test.phase_resistance_est_reliable = false;
    return;
  }

  const float nf = (float)n;
  const float denom = nf * sxx - sx * sx;
  if (fabsf(denom) < 0.000001f) {
    g_drv_test.phase_resistance_est_reliable = false;
    return;
  }

  const float slope = (nf * sxy - sx * sy) / denom;
  const float intercept = (sy - slope * sx) / nf;
  g_drv_test.phase_resistance_est_ohm = slope;
  g_drv_test.inverter_voltage_offset_est_v = intercept;
  g_drv_test.voltage_required_for_0p2A_est = slope * 0.2f + intercept;
  g_drv_test.voltage_required_for_0p3A_est = slope * 0.3f + intercept;

  y_mean /= nf;
  float ss_tot = 0.0f;
  float ss_res = 0.0f;
  for (uint32_t i = 0u; i < g_drv_test.sweep_point_count; ++i) {
    const SweepPointResult *pt = &g_drv_test.sweep_points[i];
    if (!pt->valid || pt->vd_command <= 0.001f) {
      continue;
    }
    const float y_fit = slope * pt->id_mean + intercept;
    const float dy = pt->vd_command - y_mean;
    const float er = pt->vd_command - y_fit;
    ss_tot += dy * dy;
    ss_res += er * er;
  }
  g_drv_test.fit_r_squared =
      (ss_tot > 0.000001f) ? (1.0f - (ss_res / ss_tot)) : 0.0f;
  g_drv_test.phase_resistance_est_reliable =
      g_drv_test.current_monotonic_ok &&
      (g_drv_test.first_reliable_current_voltage >= 0.0f) &&
      (g_drv_test.phase_resistance_est_ohm > 0.0f);
}

static bool __attribute__((unused)) current_sense_dq_observe_run(void)
{
  HalAdcSnapshot snap = {0};
  uint32_t last_seq = 0u;
  uint32_t last_print_ms = 0u;
  int64_t align_sum = 0;
  uint32_t align_count = 0u;
  uint32_t last_rpm_ms = 0u;
  int64_t rpm_last_count = 0;
  float rpm_filtered = 0.0f;
  float theta_e_prev = 0.0f;
  bool theta_prev_valid = false;
  static const float candidate_deg[OFFSET_CANDIDATE_COUNT] =
      {-30.0f, -20.0f, -10.0f, 0.0f, 10.0f, 20.0f, 30.0f};

  if (hal_adc_get_snapshot(&snap)) {
    last_seq = snap.seq;
  }

  g_drv_test.run_raw_u_min = 0xffffu;
  g_drv_test.run_raw_v_min = 0xffffu;
  g_drv_test.run_raw_u_max = 0u;
  g_drv_test.run_raw_v_max = 0u;
  g_drv_test.encoder_direction = COMM_ENCODER_DIRECTION;
  g_drv_test.encoder_cpr = COMM_ENCODER_CPR;
  g_drv_test.encoder_ppr = COMM_ENCODER_PPR;
  g_drv_test.pole_pairs_runtime = COMM_POLE_PAIRS;
  g_drv_test.encoder_counts_per_ab_cycle = 4u;
  g_drv_test.maximum_rpm = 0.0f;
  g_drv_test.minimum_rpm = 0.0f;
  g_drv_test.mechanical_rpm_filtered = 0.0f;
  g_drv_test.iu_lpf = 0.0f;
  g_drv_test.iv_lpf = 0.0f;
  g_drv_test.iw_lpf = 0.0f;
  g_drv_test.id_lpf = 0.0f;
  g_drv_test.iq_lpf = 0.0f;
  g_drv_test.current_max_a = 0.0f;
  g_drv_test.kcl_residual_max = 0.0f;
  g_drv_test.min_distance_to_phase_edge_counts = 0u;
  g_drv_test.min_distance_to_phase_edge_us = 0.0f;
  g_drv_test.best_candidate_index = -1;
  current_observe_stats_reset(&g_drv_test.current_stats);

  if (!g_drv_test.current_scale_known) {
    return false;
  }

  encoder_tracker_reset();
  rpm_last_count = g_encoder_accum;
  const float initial_vbus_v = board_read_vbus_v();
  if (initial_vbus_v < OPEN_LOOP_VBUS_MIN_V || initial_vbus_v > OPEN_LOOP_VBUS_MAX_V) {
    return false;
  }
  encoder_apply_alpha_beta_svpwm(COMM_ALIGN_V_ALPHA, 0.0f, initial_vbus_v);
  power_stage_enable_six_outputs();
  __HAL_TIM_MOE_ENABLE(&htim1);
  HAL_Delay(5u);
  if (hal_adc_get_snapshot(&snap)) {
    last_seq = snap.seq;
  }

  const uint32_t test_start_ms = HAL_GetTick();
  last_rpm_ms = test_start_ms;
  while ((HAL_GetTick() - test_start_ms) <= ENCODER_ALIGN_MS) {
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t elapsed_ms = now_ms - test_start_ms;

    if (!open_loop_wait_next_adc_sample(&last_seq, &snap)) {
      return false;
    }
    (void)encoder_tracker_sample();
    if (!encoder_delta_ok()) {
      return false;
    }

    if ((now_ms - last_rpm_ms) >= COMM_RPM_UPDATE_MS) {
      const uint32_t dt_ms = now_ms - last_rpm_ms;
      g_drv_test.encoder_delta_10ms = g_encoder_accum - rpm_last_count;
      rpm_last_count = g_encoder_accum;
      last_rpm_ms = now_ms;
      g_drv_test.mechanical_rpm_raw =
          ((float)g_drv_test.encoder_delta_10ms * 60000.0f) /
          ((float)COMM_ENCODER_CPR * (float)dt_ms);
      rpm_filtered += 0.2f * (g_drv_test.mechanical_rpm_raw - rpm_filtered);
      g_drv_test.mechanical_rpm_filtered = rpm_filtered;
      if (rpm_filtered > g_drv_test.maximum_rpm) { g_drv_test.maximum_rpm = rpm_filtered; }
      if (rpm_filtered < g_drv_test.minimum_rpm) { g_drv_test.minimum_rpm = rpm_filtered; }
    }

    const float vbus_v = board_read_vbus_v();
    if (vbus_v < OPEN_LOOP_VBUS_MIN_V || vbus_v > OPEN_LOOP_VBUS_MAX_V) {
      return false;
    }

    if (elapsed_ms >= (ENCODER_ALIGN_MS - ENCODER_ALIGN_AVG_MS)) {
      align_sum += g_encoder_accum;
      align_count++;
    }
    encoder_apply_alpha_beta_svpwm(COMM_ALIGN_V_ALPHA, 0.0f, vbus_v);

    const bool print_now = (elapsed_ms == 0u) ||
                           ((elapsed_ms - last_print_ms) >= OPEN_LOOP_MONITOR_PERIOD_MS);
    if (print_now) {
      last_print_ms = elapsed_ms;
    }
    if (!open_loop_capture_and_print(elapsed_ms,
                                     TEST_STATE_ALIGN,
                                     encoder_theta_m_from_accum(g_encoder_accum),
                                     0.0f,
                                     0.0f,
                                     0.0f,
                                     0.0f,
                                     rpm_filtered,
                                     &snap,
                                     true,
                                     print_now)) {
      return false;
    }
  }

  if (align_count == 0u) {
    return false;
  }

  g_drv_test.encoder_align_count = align_sum / (int64_t)align_count;
  g_drv_test.theta_m_align = encoder_theta_m_from_accum(g_drv_test.encoder_align_count);
  g_drv_test.electrical_offset_runtime_rad =
      wrap_0_2pi_f(-((float)COMM_ENCODER_DIRECTION) *
                   (float)COMM_POLE_PAIRS *
                   g_drv_test.theta_m_align);
  g_drv_test.electrical_offset_runtime_deg =
      g_drv_test.electrical_offset_runtime_rad * 180.0f /
      3.14159265358979323846f;
  g_drv_test.encoder_start_count = g_encoder_accum;

  float best_ratio = 999.0f;
  for (uint32_t ci = 0u; ci < OFFSET_CANDIDATE_COUNT; ++ci) {
    OffsetCandidateResult *cand = &g_drv_test.candidates[ci];
    cand->correction_rad = candidate_deg[ci] * 3.14159265358979323846f / 180.0f;
    current_observe_stats_reset(&cand->stats);
    const uint32_t cand_start_ms = HAL_GetTick();
    bool cand_ok = true;
    while ((HAL_GetTick() - cand_start_ms) <
           (OFFSET_CANDIDATE_RAMP_MS + OFFSET_CANDIDATE_SETTLE_MS +
            OFFSET_CANDIDATE_SAMPLE_MS + OFFSET_CANDIDATE_DOWN_MS)) {
      const uint32_t now_ms = HAL_GetTick();
      const uint32_t elapsed_ms = now_ms - cand_start_ms;
      if (!open_loop_wait_next_adc_sample(&last_seq, &snap)) {
        cand_ok = false;
        break;
      }
      (void)encoder_tracker_sample();
      if (!encoder_delta_ok()) {
        cand_ok = false;
        break;
      }

      const float vbus_v = board_read_vbus_v();
      float vq = 0.0f;
      bool collect = false;
      if (elapsed_ms < OFFSET_CANDIDATE_RAMP_MS) {
        vq = COMM_VQ_TARGET_V * ((float)elapsed_ms / (float)OFFSET_CANDIDATE_RAMP_MS);
      } else if (elapsed_ms < (OFFSET_CANDIDATE_RAMP_MS + OFFSET_CANDIDATE_SETTLE_MS)) {
        vq = COMM_VQ_TARGET_V;
      } else if (elapsed_ms < (OFFSET_CANDIDATE_RAMP_MS + OFFSET_CANDIDATE_SETTLE_MS +
                               OFFSET_CANDIDATE_SAMPLE_MS)) {
        vq = COMM_VQ_TARGET_V;
        collect = true;
      } else {
        const uint32_t down_ms = elapsed_ms - OFFSET_CANDIDATE_RAMP_MS -
                                 OFFSET_CANDIDATE_SETTLE_MS -
                                 OFFSET_CANDIDATE_SAMPLE_MS;
        vq = COMM_VQ_TARGET_V *
             (1.0f - ((float)down_ms / (float)OFFSET_CANDIDATE_DOWN_MS));
      }

      const float theta_m = encoder_theta_m_from_accum(g_encoder_accum);
      const float theta_e = encoder_theta_e_from_theta_m(
          theta_m,
          wrap_0_2pi_f(g_drv_test.electrical_offset_runtime_rad +
                       cand->correction_rad));
      encoder_apply_dq_svpwm(theta_e, 0.0f, vq, vbus_v);
      const CurrentDqSample sample = current_observe_calculate(&snap, theta_e);
      if (sample.magnitude_max > CURRENT_LIMIT_ABS_A ||
          !power_stage_update_phase_edge_timing() ||
          vbus_v < OPEN_LOOP_VBUS_MIN_V ||
          vbus_v > OPEN_LOOP_VBUS_MAX_V ||
          !nfault_ok() ||
          !m1_is_safe_off()) {
        cand_ok = false;
        break;
      }
      if (collect) {
        current_observe_stats_update(&cand->stats, &sample);
      }
    }

    encoder_apply_dq_svpwm(encoder_theta_e_from_theta_m(encoder_theta_m_from_accum(g_encoder_accum),
                                                        g_drv_test.electrical_offset_runtime_rad),
                           0.0f,
                           0.0f,
                           board_read_vbus_v());
    HAL_Delay(20u);

    current_observe_stats_finalize(&cand->stats,
                                   &cand->id_mean,
                                   &cand->iq_mean,
                                   &cand->id_std,
                                   &cand->iq_std);
    cand->iq_effective_counts =
        (g_drv_test.current_amp_per_count > 0.0f)
            ? (cand->iq_mean / g_drv_test.current_amp_per_count)
            : 0.0f;
    cand->ratio = (fabsf(cand->iq_mean) > 0.001f)
                      ? (fabsf(cand->id_mean) / fabsf(cand->iq_mean))
                      : 999.0f;
    cand->current_magnitude_max = cand->stats.current_magnitude_max;
    cand->mechanical_rpm = g_drv_test.mechanical_rpm_filtered;
    cand->valid = cand_ok &&
                  cand->iq_mean > 0.0f &&
                  fabsf(cand->iq_effective_counts) >= CURRENT_MIN_EFFECTIVE_COUNTS &&
                  cand->current_magnitude_max < CURRENT_LIMIT_ABS_A &&
                  cand->stats.samples > 0u;
    if (cand->valid && cand->ratio < best_ratio) {
      best_ratio = cand->ratio;
      g_drv_test.best_candidate_index = (int32_t)ci;
      g_drv_test.electrical_offset_correction_candidate = cand->correction_rad;
    }
  }

  g_drv_test.offset_fine_tune_reliable = g_drv_test.best_candidate_index >= 0;
  if (g_drv_test.offset_fine_tune_reliable) {
    g_drv_test.electrical_offset_runtime_corrected =
        wrap_0_2pi_f(g_drv_test.electrical_offset_runtime_rad +
                     g_drv_test.electrical_offset_correction_candidate);
  } else {
    g_drv_test.electrical_offset_runtime_corrected =
        g_drv_test.electrical_offset_runtime_rad;
  }

  current_observe_stats_reset(&g_drv_test.current_stats);
  theta_prev_valid = false;
  const uint32_t final_start_ms = HAL_GetTick();
  last_print_ms = 0u;
  while ((HAL_GetTick() - final_start_ms) <
         (FINAL_OBSERVE_RAMP_MS + FINAL_OBSERVE_HOLD_MS + FINAL_OBSERVE_DOWN_MS)) {
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t elapsed_ms = now_ms - final_start_ms;
    if (!open_loop_wait_next_adc_sample(&last_seq, &snap)) {
      return false;
    }
    (void)encoder_tracker_sample();
    if (!encoder_delta_ok()) {
      return false;
    }

    const float vbus_v = board_read_vbus_v();
    float vq = 0.0f;
    uint32_t state = TEST_STATE_CURRENT_OBSERVE_RAMP;
    if (elapsed_ms < FINAL_OBSERVE_RAMP_MS) {
      vq = COMM_VQ_TARGET_V * ((float)elapsed_ms / (float)FINAL_OBSERVE_RAMP_MS);
    } else if (elapsed_ms < (FINAL_OBSERVE_RAMP_MS + FINAL_OBSERVE_HOLD_MS)) {
      vq = COMM_VQ_TARGET_V;
      state = TEST_STATE_CURRENT_OBSERVE_HOLD;
    } else {
      state = TEST_STATE_CURRENT_OBSERVE_DOWN;
      const uint32_t down_ms = elapsed_ms - FINAL_OBSERVE_RAMP_MS - FINAL_OBSERVE_HOLD_MS;
      vq = COMM_VQ_TARGET_V * (1.0f - ((float)down_ms / (float)FINAL_OBSERVE_DOWN_MS));
    }

    if ((now_ms - last_rpm_ms) >= COMM_RPM_UPDATE_MS) {
      const uint32_t dt_ms = now_ms - last_rpm_ms;
      g_drv_test.encoder_delta_10ms = g_encoder_accum - rpm_last_count;
      rpm_last_count = g_encoder_accum;
      last_rpm_ms = now_ms;
      g_drv_test.mechanical_rpm_raw =
          ((float)g_drv_test.encoder_delta_10ms * 60000.0f) /
          ((float)COMM_ENCODER_CPR * (float)dt_ms);
      rpm_filtered += 0.2f * (g_drv_test.mechanical_rpm_raw - rpm_filtered);
      g_drv_test.mechanical_rpm_filtered = rpm_filtered;
      if (rpm_filtered > g_drv_test.maximum_rpm) { g_drv_test.maximum_rpm = rpm_filtered; }
      if (rpm_filtered < g_drv_test.minimum_rpm) { g_drv_test.minimum_rpm = rpm_filtered; }
    }

    const float theta_m = encoder_theta_m_from_accum(g_encoder_accum);
    const float theta_e = encoder_theta_e_from_theta_m(
        theta_m,
        g_drv_test.electrical_offset_runtime_corrected);
    bool theta_ok = true;
    if (theta_prev_valid) {
      theta_ok = angle_delta_abs(theta_e, theta_e_prev) <= COMM_THETA_JUMP_MAX_RAD;
    }
    theta_e_prev = theta_e;
    theta_prev_valid = true;
    encoder_apply_dq_svpwm(theta_e, 0.0f, vq, vbus_v);

    const bool print_now = (elapsed_ms == 0u) ||
                           ((elapsed_ms - last_print_ms) >= OPEN_LOOP_MONITOR_PERIOD_MS);
    if (print_now) {
      last_print_ms = elapsed_ms;
    }
    if (!open_loop_capture_and_print(elapsed_ms,
                                     state,
                                     theta_m,
                                     theta_e,
                                     g_drv_test.electrical_offset_runtime_corrected,
                                     0.0f,
                                     vq,
                                     rpm_filtered,
                                     &snap,
                                     theta_ok,
                                     print_now)) {
      return false;
    }
    if (print_now) {
      theta_prev_valid = false;
    }
  }

  encoder_apply_dq_svpwm(encoder_theta_e_from_theta_m(encoder_theta_m_from_accum(g_encoder_accum),
                                                      g_drv_test.electrical_offset_runtime_corrected),
                         0.0f,
                         0.0f,
                         board_read_vbus_v());
  (void)open_loop_capture_and_print(HAL_GetTick() - final_start_ms,
                                    TEST_STATE_STOP,
                                    encoder_theta_m_from_accum(g_encoder_accum),
                                    encoder_theta_e_from_theta_m(encoder_theta_m_from_accum(g_encoder_accum),
                                                                 g_drv_test.electrical_offset_runtime_corrected),
                                    g_drv_test.electrical_offset_runtime_corrected,
                                    0.0f,
                                    0.0f,
                                    rpm_filtered,
                                    &snap,
                                    true,
                                    true);

  g_drv_test.encoder_end_count = g_encoder_accum;
  g_drv_test.encoder_total_delta =
      g_drv_test.encoder_end_count - g_drv_test.encoder_start_count;
  g_drv_test.motion_reliable = g_drv_test.encoder_total_delta >= COMM_MOTION_MIN_COUNTS;
  g_drv_test.encoder_data_reliable = true;

  return true;
}

static bool __attribute__((unused)) stationary_d_axis_current_sense_run_old(void)
{
  HalAdcSnapshot snap = {0};
  uint32_t last_seq = 0u;
  uint32_t last_print_ms = 0u;
  uint32_t last_rpm_ms = 0u;
  int64_t rpm_last_count = 0;
  float rpm_filtered = 0.0f;
  int64_t align_sum = 0;
  uint32_t align_count = 0u;
  float theta_e_prev = 0.0f;
  bool theta_prev_valid = false;

  static const uint32_t ccr4_offsets[D_AXIS_SAMPLE_POSITION_COUNT] = {250u, 400u, 550u};

  if (hal_adc_get_snapshot(&snap)) {
    last_seq = snap.seq;
  }

  g_drv_test.run_raw_u_min = 0xffffu;
  g_drv_test.run_raw_v_min = 0xffffu;
  g_drv_test.run_raw_u_max = 0u;
  g_drv_test.run_raw_v_max = 0u;
  g_drv_test.encoder_direction = COMM_ENCODER_DIRECTION;
  g_drv_test.encoder_cpr = COMM_ENCODER_CPR;
  g_drv_test.encoder_ppr = COMM_ENCODER_PPR;
  g_drv_test.pole_pairs_runtime = COMM_POLE_PAIRS;
  g_drv_test.encoder_counts_per_ab_cycle = 4u;
  g_drv_test.maximum_rpm = 0.0f;
  g_drv_test.minimum_rpm = 0.0f;
  g_drv_test.mechanical_rpm_filtered = 0.0f;
  g_drv_test.iu_lpf = 0.0f;
  g_drv_test.iv_lpf = 0.0f;
  g_drv_test.iw_lpf = 0.0f;
  g_drv_test.id_lpf = 0.0f;
  g_drv_test.iq_lpf = 0.0f;
  g_drv_test.current_max_a = 0.0f;
  g_drv_test.kcl_residual_max = 0.0f;
  g_drv_test.min_distance_to_phase_edge_counts = 0u;
  g_drv_test.min_distance_to_phase_edge_us = 0.0f;
  g_drv_test.recommended_ccr4 = -1;
  g_drv_test.achieved_vd = 0.0f;
  g_drv_test.encoder_motion_max_counts = 0u;
  current_observe_stats_reset(&g_drv_test.current_stats);

  if (!g_drv_test.current_scale_known) {
    return false;
  }

  encoder_tracker_reset();
  rpm_last_count = g_encoder_accum;
  power_stage_set_ccr_half();
  power_stage_enable_six_outputs();
  __HAL_TIM_MOE_ENABLE(&htim1);

  const uint32_t test_start_ms = HAL_GetTick();
  last_rpm_ms = test_start_ms;
  while ((HAL_GetTick() - test_start_ms) <= ENCODER_ALIGN_MS) {
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t elapsed_ms = now_ms - test_start_ms;

    if (!open_loop_wait_next_adc_sample(&last_seq, &snap)) {
      return false;
    }
    (void)encoder_tracker_sample();
    if (!encoder_delta_ok()) {
      return false;
    }

    if ((now_ms - last_rpm_ms) >= COMM_RPM_UPDATE_MS) {
      const uint32_t dt_ms = now_ms - last_rpm_ms;
      g_drv_test.encoder_delta_10ms = g_encoder_accum - rpm_last_count;
      rpm_last_count = g_encoder_accum;
      last_rpm_ms = now_ms;
      g_drv_test.mechanical_rpm_raw =
          ((float)g_drv_test.encoder_delta_10ms * 60000.0f) /
          ((float)COMM_ENCODER_CPR * (float)dt_ms);
      rpm_filtered += 0.2f * (g_drv_test.mechanical_rpm_raw - rpm_filtered);
      g_drv_test.mechanical_rpm_filtered = rpm_filtered;
      if (rpm_filtered > g_drv_test.maximum_rpm) { g_drv_test.maximum_rpm = rpm_filtered; }
      if (rpm_filtered < g_drv_test.minimum_rpm) { g_drv_test.minimum_rpm = rpm_filtered; }
    }

    const float vbus_v = board_read_vbus_v();
    if (vbus_v < OPEN_LOOP_VBUS_MIN_V || vbus_v > OPEN_LOOP_VBUS_MAX_V) {
      return false;
    }

    if (elapsed_ms >= (ENCODER_ALIGN_MS - ENCODER_ALIGN_AVG_MS)) {
      align_sum += g_encoder_accum;
      align_count++;
    }
    encoder_apply_alpha_beta_svpwm(COMM_ALIGN_V_ALPHA, 0.0f, vbus_v);

    const bool print_now = (elapsed_ms == 0u) ||
                           ((elapsed_ms - last_print_ms) >= OPEN_LOOP_MONITOR_PERIOD_MS);
    if (print_now) {
      last_print_ms = elapsed_ms;
    }
    if (!open_loop_capture_and_print(elapsed_ms,
                                     TEST_STATE_ALIGN,
                                     encoder_theta_m_from_accum(g_encoder_accum),
                                     0.0f,
                                     0.0f,
                                     0.0f,
                                     0.0f,
                                     rpm_filtered,
                                     &snap,
                                     true,
                                     print_now)) {
      return false;
    }
  }

  if (align_count == 0u) {
    return false;
  }

  g_drv_test.encoder_align_count = align_sum / (int64_t)align_count;
  g_drv_test.theta_m_align = encoder_theta_m_from_accum(g_drv_test.encoder_align_count);
  g_drv_test.electrical_offset_runtime_rad =
      wrap_0_2pi_f(-((float)COMM_ENCODER_DIRECTION) *
                   (float)COMM_POLE_PAIRS *
                   g_drv_test.theta_m_align);
  g_drv_test.electrical_offset_runtime_deg =
      g_drv_test.electrical_offset_runtime_rad * 180.0f /
      3.14159265358979323846f;
  g_drv_test.encoder_start_count = g_encoder_accum;

  int64_t encoder_inject_start = g_encoder_accum;
  bool hold_started = false;
  uint32_t hold_start_ms = 0u;
  uint32_t id_hold_consecutive = 0u;
  uint32_t id_stop_consecutive = 0u;
  float vd_cmd = 0.0f;
  float achieved_id_abs = 0.0f;
  const uint32_t inject_start_ms = HAL_GetTick();
  last_print_ms = 0u;
  theta_prev_valid = false;
  current_observe_stats_reset(&g_drv_test.current_stats);

  while ((HAL_GetTick() - inject_start_ms) <
         (D_AXIS_RAMP_MS + D_AXIS_HOLD_MS + D_AXIS_DOWN_MS)) {
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t elapsed_ms = now_ms - inject_start_ms;
    uint32_t state = TEST_STATE_VD_RAMP;

    if (!open_loop_wait_next_adc_sample(&last_seq, &snap)) {
      return false;
    }
    (void)encoder_tracker_sample();
    if (!encoder_delta_ok()) {
      return false;
    }

    const uint32_t motion_counts =
        abs_i32_to_u32((int32_t)(g_encoder_accum - encoder_inject_start));
    if (motion_counts > g_drv_test.encoder_motion_max_counts) {
      g_drv_test.encoder_motion_max_counts = motion_counts;
    }
    if (motion_counts > D_AXIS_MOTION_MAX_COUNTS) {
      return false;
    }

    if ((now_ms - last_rpm_ms) >= COMM_RPM_UPDATE_MS) {
      const uint32_t dt_ms = now_ms - last_rpm_ms;
      g_drv_test.encoder_delta_10ms = g_encoder_accum - rpm_last_count;
      rpm_last_count = g_encoder_accum;
      last_rpm_ms = now_ms;
      g_drv_test.mechanical_rpm_raw =
          ((float)g_drv_test.encoder_delta_10ms * 60000.0f) /
          ((float)COMM_ENCODER_CPR * (float)dt_ms);
      rpm_filtered += 0.2f * (g_drv_test.mechanical_rpm_raw - rpm_filtered);
      g_drv_test.mechanical_rpm_filtered = rpm_filtered;
    }

    const float theta_m = encoder_theta_m_from_accum(g_encoder_accum);
    const float theta_e = encoder_theta_e_from_theta_m(theta_m,
                                                       g_drv_test.electrical_offset_runtime_rad);
    bool theta_ok = true;
    if (theta_prev_valid) {
      theta_ok = angle_delta_abs(theta_e, theta_e_prev) <= COMM_THETA_JUMP_MAX_RAD;
    }
    theta_e_prev = theta_e;
    theta_prev_valid = true;

    const CurrentDqSample sample = current_observe_calculate(&snap, theta_e);
    const float id_abs = fabsf(sample.id);
    id_hold_consecutive = (id_abs >= D_AXIS_ID_HOLD_A) ? (id_hold_consecutive + 1u) : 0u;
    id_stop_consecutive = (id_abs >= D_AXIS_ID_STOP_A) ? (id_stop_consecutive + 1u) : 0u;
    if (id_abs > achieved_id_abs) {
      achieved_id_abs = id_abs;
    }
    if (sample.magnitude_max > D_AXIS_CURRENT_LIMIT_A) {
      return false;
    }

    if (!hold_started) {
      const float ramp_vd = D_AXIS_VD_TARGET_V * ((float)elapsed_ms / (float)D_AXIS_RAMP_MS);
      if (id_stop_consecutive >= 20u) {
        vd_cmd = g_drv_test.achieved_vd;
        hold_started = true;
        hold_start_ms = now_ms;
      } else {
        vd_cmd = clampf(ramp_vd, 0.0f, D_AXIS_VD_TARGET_V);
        if (id_hold_consecutive >= 20u || vd_cmd >= D_AXIS_VD_TARGET_V) {
          hold_started = true;
          hold_start_ms = now_ms;
        }
      }
    }

    if (hold_started) {
      const uint32_t hold_elapsed = now_ms - hold_start_ms;
      if (hold_elapsed < D_AXIS_HOLD_MS) {
        state = TEST_STATE_VD_HOLD;
      } else {
        state = TEST_STATE_VD_DOWN;
        const uint32_t down_elapsed = hold_elapsed - D_AXIS_HOLD_MS;
        if (down_elapsed >= D_AXIS_DOWN_MS) {
          break;
        }
        vd_cmd = g_drv_test.achieved_vd *
                 (1.0f - ((float)down_elapsed / (float)D_AXIS_DOWN_MS));
      }
    }

    vd_cmd = clampf(vd_cmd, 0.0f, D_AXIS_VD_MAX_V);
    if (state != TEST_STATE_VD_DOWN && vd_cmd > g_drv_test.achieved_vd) {
      g_drv_test.achieved_vd = vd_cmd;
    }
    encoder_apply_dq_svpwm(theta_e, vd_cmd, 0.0f, board_read_vbus_v());

    const bool print_now = (elapsed_ms == 0u) ||
                           ((elapsed_ms - last_print_ms) >= OPEN_LOOP_MONITOR_PERIOD_MS);
    if (print_now) {
      last_print_ms = elapsed_ms;
    }
    if (!open_loop_capture_and_print(elapsed_ms,
                                     state,
                                     theta_m,
                                     theta_e,
                                     g_drv_test.electrical_offset_runtime_rad,
                                     vd_cmd,
                                     0.0f,
                                     rpm_filtered,
                                     &snap,
                                     theta_ok,
                                     print_now)) {
      return false;
    }
  }

  encoder_apply_dq_svpwm(encoder_theta_e_from_theta_m(encoder_theta_m_from_accum(g_encoder_accum),
                                                      g_drv_test.electrical_offset_runtime_rad),
                         0.0f,
                         0.0f,
                         board_read_vbus_v());
  HAL_Delay(20u);
  g_drv_test.d_axis_signal_ok = achieved_id_abs >= D_AXIS_SIGNAL_MIN_A;
  g_drv_test.d_axis_motion_ok = g_drv_test.encoder_motion_max_counts <= D_AXIS_MOTION_MAX_COUNTS;

  float initial_id_mean = 0.0f;
  float initial_iq_mean = 0.0f;
  float initial_id_std = 0.0f;
  float initial_iq_std = 0.0f;
  current_observe_stats_finalize(&g_drv_test.current_stats,
                                 &initial_id_mean,
                                 &initial_iq_mean,
                                 &initial_id_std,
                                 &initial_iq_std);
  g_drv_test.id_mean_effective_counts =
      (g_drv_test.current_amp_per_count > 0.0f)
          ? (initial_id_mean / g_drv_test.current_amp_per_count)
          : 0.0f;
  g_drv_test.iq_mean_effective_counts =
      (g_drv_test.current_amp_per_count > 0.0f)
          ? (initial_iq_mean / g_drv_test.current_amp_per_count)
          : 0.0f;
  g_drv_test.d_axis_polarity_ok = initial_id_mean > 0.0f;
  g_drv_test.id_iq_abs_ratio =
      (fabsf(initial_id_mean) > 0.001f)
          ? (fabsf(initial_iq_mean) / fabsf(initial_id_mean))
          : 999.0f;

  if (g_drv_test.d_axis_signal_ok) {
    uint32_t valid_count = 0u;
    float valid_id_min = 1000000.0f;
    float valid_id_max = -1000000.0f;
    float best_std = 1000000.0f;

    for (uint32_t pi = 0u; pi < D_AXIS_SAMPLE_POSITION_COUNT; ++pi) {
      SamplePositionResult *pos = &g_drv_test.sample_positions[pi];
      memset(pos, 0, sizeof(*pos));
      current_observe_stats_reset(&pos->stats);
      pos->raw_u_min = 0xffffu;
      pos->raw_v_min = 0xffffu;
      pos->raw_u_max = 0u;
      pos->raw_v_max = 0u;

      encoder_apply_dq_svpwm(encoder_theta_e_from_theta_m(encoder_theta_m_from_accum(g_encoder_accum),
                                                          g_drv_test.electrical_offset_runtime_rad),
                             0.0f,
                             0.0f,
                             board_read_vbus_v());
      __HAL_TIM_MOE_DISABLE(&htim1);
      power_stage_set_ccr_half();
      const uint32_t arr = TIM1->ARR;
      pos->ccr4 = (arr > ccr4_offsets[pi]) ? (arr - ccr4_offsets[pi]) : (arr / 2u);
      TIM1->CCR4 = pos->ccr4;
      pos->edge_timing_ok = power_stage_update_phase_edge_timing();
      pos->min_phase_edge_distance_us = g_drv_test.distance_to_nearest_pwm_edge_us;
      if (!pos->edge_timing_ok) {
        continue;
      }
      __HAL_TIM_MOE_ENABLE(&htim1);

      const int64_t pos_start_count = g_encoder_accum;
      const uint32_t pos_start_ms = HAL_GetTick();
      last_print_ms = 0u;
      while ((HAL_GetTick() - pos_start_ms) <
             (D_AXIS_SAMPLE_RAMP_MS + D_AXIS_SAMPLE_SETTLE_MS +
              D_AXIS_SAMPLE_COLLECT_MS + D_AXIS_SAMPLE_DOWN_MS)) {
        const uint32_t now_ms = HAL_GetTick();
        const uint32_t elapsed_ms = now_ms - pos_start_ms;
        if (!open_loop_wait_next_adc_sample(&last_seq, &snap)) {
          return false;
        }
        (void)encoder_tracker_sample();
        if (!encoder_delta_ok()) {
          return false;
        }

        const uint32_t motion_counts =
            abs_i32_to_u32((int32_t)(g_encoder_accum - pos_start_count));
        if (motion_counts > pos->encoder_motion_counts) {
          pos->encoder_motion_counts = motion_counts;
        }
        if (motion_counts > D_AXIS_MOTION_MAX_COUNTS) {
          return false;
        }

        float vd = 0.0f;
        bool collect = false;
        if (elapsed_ms < D_AXIS_SAMPLE_RAMP_MS) {
          vd = g_drv_test.achieved_vd * ((float)elapsed_ms / (float)D_AXIS_SAMPLE_RAMP_MS);
        } else if (elapsed_ms < (D_AXIS_SAMPLE_RAMP_MS + D_AXIS_SAMPLE_SETTLE_MS)) {
          vd = g_drv_test.achieved_vd;
        } else if (elapsed_ms < (D_AXIS_SAMPLE_RAMP_MS + D_AXIS_SAMPLE_SETTLE_MS +
                                 D_AXIS_SAMPLE_COLLECT_MS)) {
          vd = g_drv_test.achieved_vd;
          collect = true;
        } else {
          const uint32_t down_ms = elapsed_ms - D_AXIS_SAMPLE_RAMP_MS -
                                   D_AXIS_SAMPLE_SETTLE_MS -
                                   D_AXIS_SAMPLE_COLLECT_MS;
          vd = g_drv_test.achieved_vd *
               (1.0f - ((float)down_ms / (float)D_AXIS_SAMPLE_DOWN_MS));
        }
        vd = clampf(vd, 0.0f, D_AXIS_VD_TARGET_V);

        const float theta_m = encoder_theta_m_from_accum(g_encoder_accum);
        const float theta_e = encoder_theta_e_from_theta_m(theta_m,
                                                           g_drv_test.electrical_offset_runtime_rad);
        encoder_apply_dq_svpwm(theta_e, vd, 0.0f, board_read_vbus_v());
        const CurrentDqSample sample = current_observe_calculate(&snap, theta_e);
        if (sample.magnitude_max > D_AXIS_CURRENT_LIMIT_A ||
            !power_stage_update_phase_edge_timing() ||
            !nfault_ok() ||
            !m1_is_safe_off()) {
          return false;
        }
        if (snap.raw_u < pos->raw_u_min) { pos->raw_u_min = snap.raw_u; }
        if (snap.raw_u > pos->raw_u_max) { pos->raw_u_max = snap.raw_u; }
        if (snap.raw_v < pos->raw_v_min) { pos->raw_v_min = snap.raw_v; }
        if (snap.raw_v > pos->raw_v_max) { pos->raw_v_max = snap.raw_v; }
        if (collect) {
          current_observe_stats_update(&pos->stats, &sample);
        }

        const bool print_now = (elapsed_ms == 0u) ||
                               ((elapsed_ms - last_print_ms) >= OPEN_LOOP_MONITOR_PERIOD_MS);
        if (print_now) {
          last_print_ms = elapsed_ms;
        }
        if (!open_loop_capture_and_print(elapsed_ms,
                                         TEST_STATE_SAMPLE_POSITION_TEST,
                                         theta_m,
                                         theta_e,
                                         g_drv_test.electrical_offset_runtime_rad,
                                         vd,
                                         0.0f,
                                         rpm_filtered,
                                         &snap,
                                         true,
                                         print_now)) {
          return false;
        }
      }

      sample_position_finalize(pos);
      if (pos->valid) {
        valid_count++;
        if (pos->id_mean < valid_id_min) { valid_id_min = pos->id_mean; }
        if (pos->id_mean > valid_id_max) { valid_id_max = pos->id_mean; }
        if (pos->id_std < best_std) {
          best_std = pos->id_std;
          g_drv_test.recommended_ccr4 = (int32_t)pos->ccr4;
        }
      }
    }

    if (valid_count > 0u) {
      const float avg_span_ref = 0.5f * (fabsf(valid_id_min) + fabsf(valid_id_max));
      const float diff_ratio = (avg_span_ref > 0.001f)
                                   ? ((valid_id_max - valid_id_min) / avg_span_ref)
                                   : 999.0f;
      g_drv_test.sample_window_reliable = diff_ratio <= D_AXIS_WINDOW_DIFF_MAX_RATIO;
    }
  }

  encoder_apply_dq_svpwm(encoder_theta_e_from_theta_m(encoder_theta_m_from_accum(g_encoder_accum),
                                                      g_drv_test.electrical_offset_runtime_rad),
                         0.0f,
                         0.0f,
                         board_read_vbus_v());
  (void)open_loop_capture_and_print(HAL_GetTick() - test_start_ms,
                                    TEST_STATE_STOP,
                                    encoder_theta_m_from_accum(g_encoder_accum),
                                    encoder_theta_e_from_theta_m(encoder_theta_m_from_accum(g_encoder_accum),
                                                                 g_drv_test.electrical_offset_runtime_rad),
                                    g_drv_test.electrical_offset_runtime_rad,
                                    0.0f,
                                    0.0f,
                                    rpm_filtered,
                                    &snap,
                                    true,
                                    true);

  g_drv_test.encoder_end_count = g_encoder_accum;
  g_drv_test.encoder_total_delta =
      g_drv_test.encoder_end_count - g_drv_test.encoder_start_count;
  g_drv_test.encoder_data_reliable = true;
  return true;
}

static bool static_d_axis_voltage_sweep_run(void)
{
  HalAdcSnapshot snap = {0};
  uint32_t last_seq = 0u;
  uint32_t last_print_ms = 0u;
  uint32_t last_rpm_ms = 0u;
  uint32_t last_drv_status_ms = 0u;
  int64_t rpm_last_count = 0;
  float rpm_filtered = 0.0f;
  int64_t align_sum = 0;
  uint32_t align_count = 0u;
  float theta_e_prev = 0.0f;
  bool theta_prev_valid = false;
  float previous_vd = 0.0f;

  static const float vd_points[D_AXIS_SWEEP_POINT_COUNT] = {
      0.00f, 0.05f, 0.10f, 0.15f, 0.20f, 0.25f,
      0.30f, 0.35f, 0.40f, 0.45f, 0.50f};

  if (hal_adc_get_snapshot(&snap)) {
    last_seq = snap.seq;
  }

  g_drv_test.run_raw_u_min = 0xffffu;
  g_drv_test.run_raw_v_min = 0xffffu;
  g_drv_test.run_raw_u_max = 0u;
  g_drv_test.run_raw_v_max = 0u;
  g_drv_test.encoder_direction = COMM_ENCODER_DIRECTION;
  g_drv_test.encoder_cpr = COMM_ENCODER_CPR;
  g_drv_test.encoder_ppr = COMM_ENCODER_PPR;
  g_drv_test.pole_pairs_runtime = COMM_POLE_PAIRS;
  g_drv_test.encoder_counts_per_ab_cycle = 4u;
  g_drv_test.maximum_rpm = 0.0f;
  g_drv_test.minimum_rpm = 0.0f;
  g_drv_test.mechanical_rpm_filtered = 0.0f;
  g_drv_test.iu_lpf = 0.0f;
  g_drv_test.iv_lpf = 0.0f;
  g_drv_test.iw_lpf = 0.0f;
  g_drv_test.id_lpf = 0.0f;
  g_drv_test.iq_lpf = 0.0f;
  g_drv_test.current_max_a = 0.0f;
  g_drv_test.kcl_residual_max = 0.0f;
  g_drv_test.min_distance_to_phase_edge_counts = 0u;
  g_drv_test.min_distance_to_phase_edge_us = 0.0f;
  g_drv_test.recommended_ccr4 = (int32_t)((TIM1->ARR > ADC_TRIGGER_SAFE_OFFSET_COUNTS)
                                            ? (TIM1->ARR - ADC_TRIGGER_SAFE_OFFSET_COUNTS)
                                            : (TIM1->ARR / 2u));
  g_drv_test.achieved_vd = 0.0f;
  g_drv_test.first_reliable_current_voltage = PHASE_RESISTANCE_UNSET;
  g_drv_test.phase_resistance_est_ohm = PHASE_RESISTANCE_UNSET;
  g_drv_test.voltage_required_for_0p2A_est = PHASE_RESISTANCE_UNSET;
  g_drv_test.voltage_required_for_0p3A_est = PHASE_RESISTANCE_UNSET;
  g_drv_test.first_reliable_point_index = 0xffffffffu;
  g_drv_test.encoder_motion_max_counts = 0u;
  g_drv_test.sweep_point_count = 0u;
  g_drv_test.voltage_diag_count = 0u;
  g_drv_test.direct_alpha_count = 0u;
  g_drv_test.voltage_command_double_scaled = false;
  g_drv_test.pwm_voltage_command_too_small = false;
  g_drv_test.applied_voltage_scale_fail = false;
  g_drv_test.low_side_sample_window_valid = false;
  g_drv_test.direct_alpha_reliable = false;
  g_drv_test.voltage_path_classification = "STATIC_VOLTAGE_PATH_DIAGNOSTIC_NOT_RUN";
  current_observe_stats_reset(&g_drv_test.current_stats);
  current_trip_diag_reset();

  if (!g_drv_test.current_scale_known) {
    return false;
  }

  TIM1->CCR4 = (uint32_t)g_drv_test.recommended_ccr4;
  if (!power_stage_update_phase_edge_timing()) {
    return false;
  }

  encoder_tracker_reset();
  rpm_last_count = g_encoder_accum;
  power_stage_set_ccr_half();
  power_stage_enable_six_outputs();
  __HAL_TIM_MOE_ENABLE(&htim1);
  g_drv_test.current_trip_diag.moe_enable_timestamp_us = diag_timestamp_us();
  g_drv_test.current_trip_diag.switch_timestamp_us = g_drv_test.current_trip_diag.moe_enable_timestamp_us;

  const uint32_t test_start_ms = HAL_GetTick();
  last_rpm_ms = test_start_ms;
  last_drv_status_ms = test_start_ms;
  while ((HAL_GetTick() - test_start_ms) <= ENCODER_ALIGN_MS) {
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t elapsed_ms = now_ms - test_start_ms;

    if (!open_loop_wait_next_adc_sample(&last_seq, &snap)) {
      return false;
    }
    (void)encoder_tracker_sample();
    if (!encoder_delta_ok()) {
      return false;
    }

    if ((now_ms - last_rpm_ms) >= COMM_RPM_UPDATE_MS) {
      const uint32_t dt_ms = now_ms - last_rpm_ms;
      g_drv_test.encoder_delta_10ms = g_encoder_accum - rpm_last_count;
      rpm_last_count = g_encoder_accum;
      last_rpm_ms = now_ms;
      g_drv_test.mechanical_rpm_raw =
          ((float)g_drv_test.encoder_delta_10ms * 60000.0f) /
          ((float)COMM_ENCODER_CPR * (float)dt_ms);
      rpm_filtered += 0.2f * (g_drv_test.mechanical_rpm_raw - rpm_filtered);
      g_drv_test.mechanical_rpm_filtered = rpm_filtered;
      if (rpm_filtered > g_drv_test.maximum_rpm) { g_drv_test.maximum_rpm = rpm_filtered; }
      if (rpm_filtered < g_drv_test.minimum_rpm) { g_drv_test.minimum_rpm = rpm_filtered; }
    }

    const float vbus_v = board_read_vbus_v();
    if (vbus_v < OPEN_LOOP_VBUS_MIN_V || vbus_v > OPEN_LOOP_VBUS_MAX_V) {
      return false;
    }
    if ((now_ms - last_drv_status_ms) >= 100u) {
      last_drv_status_ms = now_ms;
      if (!drv8301_read_status(&g_drv0) || !drv8301_read_status(&g_drv1) ||
          drv_status_has_fault(g_drv0.status.status1_raw, g_drv0.status.status2_raw) ||
          drv_status_has_fault(g_drv1.status.status1_raw, g_drv1.status.status2_raw)) {
        return false;
      }
    }

    if (elapsed_ms >= (ENCODER_ALIGN_MS - ENCODER_ALIGN_AVG_MS)) {
      align_sum += g_encoder_accum;
      align_count++;
    }
    encoder_apply_alpha_beta_svpwm(STATIC_D_AXIS_ALIGN_V_ALPHA, 0.0f, vbus_v);
    const CurrentDqSample align_sample = current_observe_calculate(&snap, 0.0f);
    if (!current_trip_diag_record_and_check(TEST_STATE_ALIGN,
                                            elapsed_ms,
                                            &snap,
                                            &align_sample,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            vbus_v)) {
      power_stage_force_safe_off_zero_ccr();
      hal_pwm_start_adc_trigger_only();
      power_stage_force_safe_off_zero_ccr();
      return false;
    }

    const bool print_now = (elapsed_ms == 0u) ||
                           ((elapsed_ms - last_print_ms) >= OPEN_LOOP_MONITOR_PERIOD_MS);
    if (print_now) {
      last_print_ms = elapsed_ms;
    }
    if (!open_loop_capture_and_print(elapsed_ms,
                                     TEST_STATE_ALIGN,
                                     encoder_theta_m_from_accum(g_encoder_accum),
                                     0.0f,
                                     0.0f,
                                     0.0f,
                                     0.0f,
                                     rpm_filtered,
                                     &snap,
                                     true,
                                     print_now)) {
      return false;
    }
  }

  if (align_count == 0u) {
    return false;
  }

  g_drv_test.encoder_align_count = align_sum / (int64_t)align_count;
  g_drv_test.theta_m_align = encoder_theta_m_from_accum(g_drv_test.encoder_align_count);
  g_drv_test.electrical_offset_runtime_rad =
      wrap_0_2pi_f(-((float)COMM_ENCODER_DIRECTION) *
                   (float)COMM_POLE_PAIRS *
                   g_drv_test.theta_m_align);
  g_drv_test.electrical_offset_runtime_deg =
      g_drv_test.electrical_offset_runtime_rad * 180.0f /
      3.14159265358979323846f;
  g_drv_test.encoder_start_count = g_encoder_accum;

  for (uint32_t pi = 0u; pi < D_AXIS_SWEEP_POINT_COUNT; ++pi) {
    SweepPointResult *pt = &g_drv_test.sweep_points[pi];
    memset(pt, 0, sizeof(*pt));
    current_observe_stats_reset(&pt->stats);
    pt->vd_command = vd_points[pi];
    pt->vq_command = 0.0f;
    pt->raw_u_min = 0xffffu;
    pt->raw_v_min = 0xffffu;
    pt->raw_u_max = 0u;
    pt->raw_v_max = 0u;
    float vbus_sum = 0.0f;
    uint32_t vbus_samples = 0u;
    float phase_abs_sum = 0.0f;
    float phase_abs_sumsq = 0.0f;
    uint32_t phase_above_run = 0u;
    uint16_t phase_hist[D_AXIS_PHASE_CURRENT_HIST_BINS];
    memset(phase_hist, 0, sizeof(phase_hist));
    uint32_t point_current_over_consecutive_max = 0u;

    const int64_t point_start_count = g_encoder_accum;
    const uint32_t point_start_ms = HAL_GetTick();
    const uint32_t total_point_ms = D_AXIS_RAMP_MS + D_AXIS_SETTLE_MS + D_AXIS_SAMPLE_MS;
    last_print_ms = 0u;
    theta_prev_valid = false;
    bool voltage_diag_done = false;

    while ((HAL_GetTick() - point_start_ms) < total_point_ms) {
      const uint32_t now_ms = HAL_GetTick();
      const uint32_t elapsed_ms = now_ms - point_start_ms;
      uint32_t state = TEST_STATE_VD_RAMP;
      bool collect = false;
      float vd_cmd = pt->vd_command;

      if (!open_loop_wait_next_adc_sample(&last_seq, &snap)) {
        pt->faulted = true;
        return false;
      }
      (void)encoder_tracker_sample();
      if (!encoder_delta_ok()) {
        pt->faulted = true;
        return false;
      }

      if ((now_ms - last_rpm_ms) >= COMM_RPM_UPDATE_MS) {
        const uint32_t dt_ms = now_ms - last_rpm_ms;
        g_drv_test.encoder_delta_10ms = g_encoder_accum - rpm_last_count;
        rpm_last_count = g_encoder_accum;
        last_rpm_ms = now_ms;
        g_drv_test.mechanical_rpm_raw =
            ((float)g_drv_test.encoder_delta_10ms * 60000.0f) /
            ((float)COMM_ENCODER_CPR * (float)dt_ms);
        rpm_filtered += 0.2f * (g_drv_test.mechanical_rpm_raw - rpm_filtered);
        g_drv_test.mechanical_rpm_filtered = rpm_filtered;
        if (rpm_filtered > g_drv_test.maximum_rpm) { g_drv_test.maximum_rpm = rpm_filtered; }
        if (rpm_filtered < g_drv_test.minimum_rpm) { g_drv_test.minimum_rpm = rpm_filtered; }
      }

      const uint32_t motion_counts =
          abs_i32_to_u32((int32_t)(g_encoder_accum - point_start_count));
      if (motion_counts > pt->encoder_motion_counts) {
        pt->encoder_motion_counts = motion_counts;
      }
      if (motion_counts > g_drv_test.encoder_motion_max_counts) {
        g_drv_test.encoder_motion_max_counts = motion_counts;
      }
      if (motion_counts > D_AXIS_MOTION_MAX_COUNTS) {
        pt->faulted = true;
        return false;
      }

      if (elapsed_ms < D_AXIS_RAMP_MS) {
        const float t = (float)elapsed_ms / (float)D_AXIS_RAMP_MS;
        vd_cmd = previous_vd + (pt->vd_command - previous_vd) * t;
        state = TEST_STATE_VD_RAMP;
      } else if (elapsed_ms < (D_AXIS_RAMP_MS + D_AXIS_SETTLE_MS)) {
        vd_cmd = pt->vd_command;
        state = TEST_STATE_VD_SETTLE;
      } else {
        vd_cmd = pt->vd_command;
        state = TEST_STATE_VD_SAMPLE;
        collect = elapsed_ms >= (D_AXIS_RAMP_MS + D_AXIS_SETTLE_MS +
                                 D_AXIS_SAMPLE_IGNORE_MS);
      }
      vd_cmd = clampf(vd_cmd, 0.0f, D_AXIS_VD_MAX_V);

      const float theta_m = encoder_theta_m_from_accum(g_encoder_accum);
      const float theta_e = encoder_theta_e_from_theta_m(theta_m,
                                                         g_drv_test.electrical_offset_runtime_rad);
      pt->v_alpha = vd_cmd * cosf(theta_e);
      pt->v_beta = vd_cmd * sinf(theta_e);
      bool theta_ok = true;
      if (theta_prev_valid) {
        theta_ok = angle_delta_abs(theta_e, theta_e_prev) <= COMM_THETA_JUMP_MAX_RAD;
      }
      theta_e_prev = theta_e;
      theta_prev_valid = true;

      const float vbus_v = board_read_vbus_v();
      if (vbus_v < OPEN_LOOP_VBUS_MIN_V || vbus_v > OPEN_LOOP_VBUS_MAX_V) {
        pt->faulted = true;
        return false;
      }
      encoder_apply_dq_svpwm(theta_e, vd_cmd, 0.0f, vbus_v);
      sample_window_diag_update();
      if (!voltage_diag_done &&
          (state == TEST_STATE_VD_SAMPLE) &&
          ((pi == 0u) || (pi == 2u) || (pi == 4u) ||
           (pi == 6u) || (pi == 8u) || (pi == 10u)) &&
          (g_drv_test.voltage_diag_count < VOLTAGE_DIAG_POINT_COUNT)) {
        voltage_path_diag_compute(&g_drv_test.voltage_diags[g_drv_test.voltage_diag_count],
                                  pt->vd_command,
                                  0.0f,
                                  theta_e,
                                  vbus_v);
        if (g_drv_test.voltage_diags[g_drv_test.voltage_diag_count].double_scaled) {
          g_drv_test.voltage_command_double_scaled = true;
        }
        if (g_drv_test.voltage_diags[g_drv_test.voltage_diag_count].command_too_small) {
          g_drv_test.pwm_voltage_command_too_small = true;
        }
        if (g_drv_test.voltage_diags[g_drv_test.voltage_diag_count].applied_scale_fail) {
          g_drv_test.applied_voltage_scale_fail = true;
        }
        g_drv_test.voltage_diag_count++;
        voltage_diag_done = true;
      }
      const CurrentDqSample sample = current_observe_calculate(&snap, theta_e);
      if (!current_trip_diag_record_and_check(state,
                                              elapsed_ms,
                                              &snap,
                                              &sample,
                                              vd_cmd,
                                              0.0f,
                                              theta_e,
                                              vbus_v)) {
        pt->faulted = true;
        power_stage_force_safe_off_zero_ccr();
        hal_pwm_start_adc_trigger_only();
        power_stage_force_safe_off_zero_ccr();
        return false;
      }
      if (!power_stage_update_phase_edge_timing() ||
          !nfault_ok() ||
          !m1_is_safe_off()) {
        pt->faulted = true;
        return false;
      }
      const uint32_t point_consecutive_now = current_trip_current_consecutive_max();
      if (point_consecutive_now > point_current_over_consecutive_max) {
        point_current_over_consecutive_max = point_consecutive_now;
      }
      if ((now_ms - last_drv_status_ms) >= 100u) {
        last_drv_status_ms = now_ms;
        if (!drv8301_read_status(&g_drv0) || !drv8301_read_status(&g_drv1) ||
            drv_status_has_fault(g_drv0.status.status1_raw, g_drv0.status.status2_raw) ||
            drv_status_has_fault(g_drv1.status.status1_raw, g_drv1.status.status2_raw)) {
          pt->faulted = true;
          return false;
        }
      }

      if (snap.raw_u < pt->raw_u_min) { pt->raw_u_min = snap.raw_u; }
      if (snap.raw_u > pt->raw_u_max) { pt->raw_u_max = snap.raw_u; }
      if (snap.raw_v < pt->raw_v_min) { pt->raw_v_min = snap.raw_v; }
      if (snap.raw_v > pt->raw_v_max) { pt->raw_v_max = snap.raw_v; }
      if (collect) {
        current_observe_stats_update(&pt->stats, &sample);
        vbus_sum += vbus_v;
        vbus_samples++;

        float phase_abs = fabsf(sample.iu);
        const float phase_v = fabsf(sample.iv);
        const float phase_w = fabsf(sample.iw);
        if (phase_v > phase_abs) { phase_abs = phase_v; }
        if (phase_w > phase_abs) { phase_abs = phase_w; }
        phase_abs_sum += phase_abs;
        phase_abs_sumsq += phase_abs * phase_abs;
        if (phase_abs > pt->phase_current_max) {
          pt->phase_current_max = phase_abs;
        }
        if (phase_abs >= D_AXIS_PHASE_CURRENT_STOP_A) {
          pt->samples_above_0p45a++;
          phase_above_run++;
          if (phase_above_run > pt->longest_consecutive_samples_above_0p45a) {
            pt->longest_consecutive_samples_above_0p45a = phase_above_run;
          }
        } else {
          phase_above_run = 0u;
        }
        uint32_t bin = float_to_scaled_u32(phase_abs, D_AXIS_PHASE_CURRENT_HIST_SCALE);
        if (bin >= D_AXIS_PHASE_CURRENT_HIST_BINS) {
          bin = D_AXIS_PHASE_CURRENT_HIST_BINS - 1u;
        }
        if (phase_hist[bin] < 0xffffu) {
          phase_hist[bin]++;
        }
      }

      const bool print_now = (elapsed_ms == 0u) ||
                             ((elapsed_ms - last_print_ms) >= OPEN_LOOP_MONITOR_PERIOD_MS);
      if (print_now) {
        last_print_ms = elapsed_ms;
      }
      if (!open_loop_capture_and_print(elapsed_ms,
                                       state,
                                       theta_m,
                                       theta_e,
                                       g_drv_test.electrical_offset_runtime_rad,
                                       vd_cmd,
                                       0.0f,
                                       rpm_filtered,
                                       &snap,
                                       theta_ok,
                                       print_now)) {
        pt->faulted = true;
        return false;
      }
    }

    pt->vbus_mean = (vbus_samples > 0u) ? (vbus_sum / (float)vbus_samples) : 0.0f;
    if (pt->stats.samples > 0u) {
      const float inv = 1.0f / (float)pt->stats.samples;
      pt->phase_current_abs_mean = phase_abs_sum * inv;
      pt->phase_current_rms = sqrtf(phase_abs_sumsq * inv);
      pt->phase_current_p95 =
          histogram_percentile_current(phase_hist,
                                       D_AXIS_PHASE_CURRENT_HIST_BINS,
                                       pt->stats.samples,
                                       950u);
      pt->phase_current_p99 =
          histogram_percentile_current(phase_hist,
                                       D_AXIS_PHASE_CURRENT_HIST_BINS,
                                       pt->stats.samples,
                                       990u);
      pt->phase_current_p999 =
          histogram_percentile_current(phase_hist,
                                       D_AXIS_PHASE_CURRENT_HIST_BINS,
                                       pt->stats.samples,
                                       999u);
    }
    sweep_point_finalize(pt);
    pt->current_over_consecutive_max = point_current_over_consecutive_max;
    g_drv_test.sweep_point_count = pi + 1u;
    previous_vd = pt->vd_command;
    if (pt->vd_command > g_drv_test.achieved_vd) {
      g_drv_test.achieved_vd = pt->vd_command;
    }

    if ((pt->vd_command > 0.001f) &&
        (fabsf(pt->id_mean) >= D_AXIS_ID_STOP_A ||
         pt->phase_current_rms >= D_AXIS_PHASE_CURRENT_RMS_STOP_A ||
         pt->phase_current_p99 >= D_AXIS_PHASE_CURRENT_STOP_A)) {
      break;
    }
  }

  encoder_apply_dq_svpwm(encoder_theta_e_from_theta_m(encoder_theta_m_from_accum(g_encoder_accum),
                                                      g_drv_test.electrical_offset_runtime_rad),
                         0.0f,
                         0.0f,
                         board_read_vbus_v());
  HAL_Delay(D_AXIS_DOWN_MS);

  sweep_fit_phase_resistance();
  g_drv_test.current_trip_diag.completed_zero_baseline = true;
  current_trip_diag_classify();

  g_drv_test.d_axis_signal_ok =
      g_drv_test.first_reliable_current_voltage >= 0.0f;
  g_drv_test.d_axis_motion_ok =
      g_drv_test.encoder_motion_max_counts <= D_AXIS_MOTION_MAX_COUNTS;
  g_drv_test.d_axis_polarity_ok = true;
  g_drv_test.current_direction_ok = true;
  for (uint32_t i = 0u; i < g_drv_test.sweep_point_count; ++i) {
    const SweepPointResult *pt = &g_drv_test.sweep_points[i];
    if (pt->valid && pt->id_mean <= 0.0f) {
      g_drv_test.current_direction_ok = false;
      g_drv_test.d_axis_polarity_ok = false;
    }
  }

  g_drv_test.current_resolution_ok =
      g_drv_test.first_reliable_current_voltage >= 0.0f;
  g_drv_test.dq_alignment_ok = false;
  for (uint32_t i = 0u; i < g_drv_test.sweep_point_count; ++i) {
    const SweepPointResult *pt = &g_drv_test.sweep_points[i];
    if (pt->valid && fabsf(pt->id_effective_counts) >= D_AXIS_RELIABLE_MIN_COUNTS) {
      g_drv_test.dq_alignment_ok = true;
    }
  }
  g_drv_test.sample_window_reliable = g_drv_test.phase_resistance_est_reliable;
  g_drv_test.final_observe_reliable =
      g_drv_test.current_monotonic_ok &&
      g_drv_test.current_resolution_ok &&
      g_drv_test.dq_alignment_ok &&
      g_drv_test.d_axis_motion_ok &&
      g_drv_test.phase_resistance_est_reliable;

  if (!direct_alpha_voltage_diagnostic_run(&last_seq)) {
    return false;
  }

  (void)open_loop_capture_and_print(HAL_GetTick() - test_start_ms,
                                    TEST_STATE_STOP,
                                    encoder_theta_m_from_accum(g_encoder_accum),
                                    encoder_theta_e_from_theta_m(encoder_theta_m_from_accum(g_encoder_accum),
                                                                 g_drv_test.electrical_offset_runtime_rad),
                                    g_drv_test.electrical_offset_runtime_rad,
                                    0.0f,
                                    0.0f,
                                    rpm_filtered,
                                    &snap,
                                    true,
                                    true);

  g_drv_test.encoder_end_count = g_encoder_accum;
  g_drv_test.encoder_total_delta =
      g_drv_test.encoder_end_count - g_drv_test.encoder_start_count;
  g_drv_test.encoder_data_reliable = true;
  return true;
}

static bool direct_alpha_voltage_diagnostic_run(uint32_t *last_seq)
{
  static const float alpha_points[DIRECT_ALPHA_POINT_COUNT] = {
      0.10f, 0.20f, 0.30f, 0.40f, 0.50f};

  if (g_drv_test.voltage_command_double_scaled ||
      g_drv_test.pwm_voltage_command_too_small ||
      g_drv_test.applied_voltage_scale_fail ||
      !g_drv_test.low_side_sample_window_valid) {
    if (g_drv_test.voltage_command_double_scaled ||
        g_drv_test.pwm_voltage_command_too_small ||
        g_drv_test.applied_voltage_scale_fail) {
      g_drv_test.voltage_path_classification = "SVPWM_VOLTAGE_SCALING_FAIL";
    } else {
      g_drv_test.voltage_path_classification = "LOW_SIDE_SAMPLE_WINDOW_FAIL";
    }
    return true;
  }

  HalAdcSnapshot snap = {0};
  if (hal_adc_get_snapshot(&snap) && last_seq != 0) {
    *last_seq = snap.seq;
  }

  power_stage_enable_six_outputs();
  __HAL_TIM_MOE_ENABLE(&htim1);
  g_drv_test.current_trip_diag.moe_enable_timestamp_us = diag_timestamp_us();
  g_drv_test.current_trip_diag.switch_timestamp_us = g_drv_test.current_trip_diag.moe_enable_timestamp_us;

  const uint32_t align_start_ms = HAL_GetTick();
  while ((HAL_GetTick() - align_start_ms) <= ENCODER_ALIGN_MS) {
    if (!open_loop_wait_next_adc_sample(last_seq, &snap)) {
      power_stage_force_safe_off_zero_ccr();
      hal_pwm_start_adc_trigger_only();
      power_stage_force_safe_off_zero_ccr();
      return false;
    }
    (void)encoder_tracker_sample();
    if (!encoder_delta_ok()) {
      power_stage_force_safe_off_zero_ccr();
      return false;
    }
    const float vbus_v = board_read_vbus_v();
    if (vbus_v < OPEN_LOOP_VBUS_MIN_V || vbus_v > OPEN_LOOP_VBUS_MAX_V) {
      power_stage_force_safe_off_zero_ccr();
      return false;
    }
    encoder_apply_alpha_beta_svpwm(STATIC_D_AXIS_ALIGN_V_ALPHA, 0.0f, vbus_v);
    sample_window_diag_update();
    const CurrentDqSample sample = current_observe_calculate(&snap, 0.0f);
    if (!current_trip_diag_record_and_check(TEST_STATE_ALIGN,
                                            HAL_GetTick() - align_start_ms,
                                            &snap,
                                            &sample,
                                            STATIC_D_AXIS_ALIGN_V_ALPHA,
                                            0.0f,
                                            0.0f,
                                            vbus_v) ||
        !nfault_ok() ||
        !m1_is_safe_off()) {
      power_stage_force_safe_off_zero_ccr();
      hal_pwm_start_adc_trigger_only();
      power_stage_force_safe_off_zero_ccr();
      return false;
    }
  }

  float previous_alpha = 0.0f;
  for (uint32_t pi = 0u; pi < DIRECT_ALPHA_POINT_COUNT; ++pi) {
    DirectAlphaResult *res = &g_drv_test.direct_alpha[pi];
    memset(res, 0, sizeof(*res));
    res->v_alpha_command = alpha_points[pi];
    res->v_beta_command = 0.0f;

    CurrentObserveStats stats;
    current_observe_stats_reset(&stats);
    float raw_u_sum = 0.0f;
    float raw_v_sum = 0.0f;
    uint32_t raw_samples = 0u;
    float i_alpha_sumsq = 0.0f;
    float i_beta_sumsq = 0.0f;
    int64_t point_start_count = g_encoder_accum;
    const uint32_t point_start_ms = HAL_GetTick();
    const uint32_t total_ms = D_AXIS_RAMP_MS + D_AXIS_SETTLE_MS + D_AXIS_SAMPLE_MS;

    while ((HAL_GetTick() - point_start_ms) < total_ms) {
      const uint32_t elapsed_ms = HAL_GetTick() - point_start_ms;
      bool collect = false;
      float alpha_cmd = res->v_alpha_command;
      uint32_t state = TEST_STATE_VD_RAMP;

      if (!open_loop_wait_next_adc_sample(last_seq, &snap)) {
        res->faulted = true;
        power_stage_force_safe_off_zero_ccr();
        return false;
      }
      (void)encoder_tracker_sample();
      if (!encoder_delta_ok()) {
        res->faulted = true;
        power_stage_force_safe_off_zero_ccr();
        return false;
      }

      const uint32_t motion_counts =
          abs_i32_to_u32((int32_t)(g_encoder_accum - point_start_count));
      if (motion_counts > res->encoder_motion_counts) {
        res->encoder_motion_counts = motion_counts;
      }
      if (motion_counts > D_AXIS_MOTION_MAX_COUNTS) {
        res->faulted = true;
        power_stage_force_safe_off_zero_ccr();
        return false;
      }

      if (elapsed_ms < D_AXIS_RAMP_MS) {
        const float t = (float)elapsed_ms / (float)D_AXIS_RAMP_MS;
        alpha_cmd = previous_alpha + (res->v_alpha_command - previous_alpha) * t;
        state = TEST_STATE_VD_RAMP;
      } else if (elapsed_ms < (D_AXIS_RAMP_MS + D_AXIS_SETTLE_MS)) {
        alpha_cmd = res->v_alpha_command;
        state = TEST_STATE_VD_SETTLE;
      } else {
        alpha_cmd = res->v_alpha_command;
        state = TEST_STATE_VD_SAMPLE;
        collect = elapsed_ms >= (D_AXIS_RAMP_MS + D_AXIS_SETTLE_MS +
                                 D_AXIS_SAMPLE_IGNORE_MS);
      }
      alpha_cmd = clampf(alpha_cmd, 0.0f, D_AXIS_VD_MAX_V);

      const float vbus_v = board_read_vbus_v();
      if (vbus_v < OPEN_LOOP_VBUS_MIN_V || vbus_v > OPEN_LOOP_VBUS_MAX_V) {
        res->faulted = true;
        power_stage_force_safe_off_zero_ccr();
        return false;
      }

      encoder_apply_alpha_beta_svpwm(alpha_cmd, 0.0f, vbus_v);
      sample_window_diag_update();
      const CurrentDqSample sample = current_observe_calculate(&snap, 0.0f);
      if (!current_trip_diag_record_and_check(state,
                                              elapsed_ms,
                                              &snap,
                                              &sample,
                                              alpha_cmd,
                                              0.0f,
                                              0.0f,
                                              vbus_v) ||
          !power_stage_update_phase_edge_timing() ||
          !nfault_ok() ||
          !m1_is_safe_off()) {
        res->faulted = true;
        power_stage_force_safe_off_zero_ccr();
        hal_pwm_start_adc_trigger_only();
        power_stage_force_safe_off_zero_ccr();
        return false;
      }

      if (collect) {
        current_observe_stats_update(&stats, &sample);
        i_alpha_sumsq += sample.i_alpha * sample.i_alpha;
        i_beta_sumsq += sample.i_beta * sample.i_beta;
        raw_u_sum += (float)snap.raw_u;
        raw_v_sum += (float)snap.raw_v;
        raw_samples++;
      }
    }

    if (stats.samples > 0u) {
      const float inv = 1.0f / (float)stats.samples;
      res->samples = stats.samples;
      res->i_alpha_mean = stats.iu_sum * inv;
      res->i_beta_mean = stats.iv_sum * inv * 0.0f;
      res->iu_mean = stats.iu_sum * inv;
      res->iv_mean = stats.iv_sum * inv;
      res->iw_mean = stats.iw_sum * inv;
      res->i_beta_mean = (res->iu_mean + 2.0f * res->iv_mean) * 0.57735026919f;
      float alpha_var = i_alpha_sumsq * inv - res->i_alpha_mean * res->i_alpha_mean;
      float beta_var = i_beta_sumsq * inv - res->i_beta_mean * res->i_beta_mean;
      if (alpha_var < 0.0f) { alpha_var = 0.0f; }
      if (beta_var < 0.0f) { beta_var = 0.0f; }
      res->i_alpha_std = sqrtf(alpha_var);
      res->i_beta_std = sqrtf(beta_var);
      res->raw_u_mean = (raw_samples > 0u) ? (raw_u_sum / (float)raw_samples) : 0.0f;
      res->raw_v_mean = (raw_samples > 0u) ? (raw_v_sum / (float)raw_samples) : 0.0f;
      res->effective_counts =
          (g_drv_test.current_amp_per_count > 0.0f)
              ? (res->i_alpha_mean / g_drv_test.current_amp_per_count)
              : 0.0f;
      res->ccr1 = TIM1->CCR1;
      res->ccr2 = TIM1->CCR2;
      res->ccr3 = TIM1->CCR3;
      VoltagePathDiag vdiag;
      voltage_path_diag_compute(&vdiag, res->v_alpha_command, 0.0f, 0.0f, board_read_vbus_v());
      res->applied_voltage_magnitude_est = vdiag.applied_voltage_magnitude;
      res->valid = !res->faulted &&
                   (fabsf(res->effective_counts) >= CURRENT_MIN_EFFECTIVE_COUNTS) &&
                   (res->encoder_motion_counts <= D_AXIS_MOTION_MAX_COUNTS);
      if (res->valid) {
        g_drv_test.direct_alpha_reliable = true;
      }
    }

    g_drv_test.direct_alpha_count = pi + 1u;
    previous_alpha = res->v_alpha_command;
  }

  encoder_apply_alpha_beta_svpwm(0.0f, 0.0f, board_read_vbus_v());
  power_stage_set_ccr_half();

  if (g_drv_test.applied_voltage_scale_fail ||
      g_drv_test.voltage_command_double_scaled ||
      g_drv_test.pwm_voltage_command_too_small) {
    g_drv_test.voltage_path_classification = "SVPWM_VOLTAGE_SCALING_FAIL";
  } else if (!g_drv_test.low_side_sample_window_valid) {
    g_drv_test.voltage_path_classification = "LOW_SIDE_SAMPLE_WINDOW_FAIL";
  } else if (g_drv_test.direct_alpha_reliable && !g_drv_test.dq_alignment_ok) {
    g_drv_test.voltage_path_classification = "PARK_OR_ELECTRICAL_OFFSET_PATH_FAIL";
  } else if (!g_drv_test.direct_alpha_reliable) {
    g_drv_test.voltage_path_classification = "CURRENT_MEASUREMENT_PATH_UNRESPONSIVE";
  } else {
    g_drv_test.voltage_path_classification = "STATIC_VOLTAGE_PATH_DIAGNOSTIC_PASS";
  }

  return true;
}

static bool __attribute__((unused)) static_d_axis_current_trip_diagnostic_run(void)
{
  HalAdcSnapshot snap = {0};
  uint32_t last_seq = 0u;
  uint32_t last_print_ms = 0u;
  uint32_t last_drv_status_ms = 0u;
  int64_t align_sum = 0;
  uint32_t align_count = 0u;
  float theta_e_prev = 0.0f;
  bool theta_prev_valid = false;
  bool ok = true;

  current_trip_diag_reset();
  g_drv_test.run_raw_u_min = 0xffffu;
  g_drv_test.run_raw_v_min = 0xffffu;
  g_drv_test.run_raw_u_max = 0u;
  g_drv_test.run_raw_v_max = 0u;
  g_drv_test.encoder_direction = COMM_ENCODER_DIRECTION;
  g_drv_test.encoder_cpr = COMM_ENCODER_CPR;
  g_drv_test.encoder_ppr = COMM_ENCODER_PPR;
  g_drv_test.pole_pairs_runtime = COMM_POLE_PAIRS;
  g_drv_test.encoder_counts_per_ab_cycle = 4u;
  g_drv_test.current_max_a = 0.0f;
  g_drv_test.raw_current_max_deviation = 0u;
  g_drv_test.kcl_residual_max = 0.0f;
  g_drv_test.min_distance_to_phase_edge_counts = 0u;
  g_drv_test.min_distance_to_phase_edge_us = 0.0f;
  g_drv_test.recommended_ccr4 = (int32_t)((TIM1->ARR > ADC_TRIGGER_SAFE_OFFSET_COUNTS)
                                            ? (TIM1->ARR - ADC_TRIGGER_SAFE_OFFSET_COUNTS)
                                            : (TIM1->ARR / 2u));
  g_drv_test.achieved_vd = 0.0f;
  g_drv_test.first_reliable_current_voltage = PHASE_RESISTANCE_UNSET;
  g_drv_test.phase_resistance_est_ohm = PHASE_RESISTANCE_UNSET;
  g_drv_test.inverter_voltage_offset_est_v = PHASE_RESISTANCE_UNSET;
  g_drv_test.voltage_required_for_0p2A_est = PHASE_RESISTANCE_UNSET;
  g_drv_test.voltage_required_for_0p3A_est = PHASE_RESISTANCE_UNSET;
  g_drv_test.first_reliable_point_index = 0xffffffffu;
  g_drv_test.encoder_motion_max_counts = 0u;
  g_drv_test.sweep_point_count = 0u;
  g_drv_test.valid_fit_point_count = 0u;
  g_drv_test.phase_resistance_est_reliable = false;
  g_drv_test.current_monotonic_ok = false;
  current_observe_stats_reset(&g_drv_test.current_stats);

  if (!g_drv_test.current_scale_known) {
    current_trip_diag_classify();
    return false;
  }

  TIM1->CCR4 = (uint32_t)g_drv_test.recommended_ccr4;
  if (!power_stage_update_phase_edge_timing()) {
    current_trip_diag_classify();
    return false;
  }

  if (hal_adc_get_snapshot(&snap)) {
    last_seq = snap.seq;
  }

  encoder_tracker_reset();
  g_drv_test.encoder_start_count = g_encoder_accum;
  power_stage_set_ccr_half();
  power_stage_enable_six_outputs();
  __HAL_TIM_MOE_ENABLE(&htim1);
  g_drv_test.current_trip_diag.moe_enable_timestamp_us = diag_timestamp_us();
  g_drv_test.current_trip_diag.switch_timestamp_us = g_drv_test.current_trip_diag.moe_enable_timestamp_us;

  const uint32_t test_start_ms = HAL_GetTick();
  last_drv_status_ms = test_start_ms;
  const uint32_t total_ms =
      CURRENT_TRIP_ALIGN_RAMP_MS + CURRENT_TRIP_ALIGN_HOLD_MS +
      CURRENT_TRIP_ALIGN_DOWN_MS + CURRENT_TRIP_ZERO_RAMP_MS +
      CURRENT_TRIP_ZERO_SETTLE_MS + CURRENT_TRIP_ZERO_SAMPLE_MS;

  while ((HAL_GetTick() - test_start_ms) <= total_ms) {
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t elapsed_ms = now_ms - test_start_ms;
    uint32_t state = DIAG_STATE_INIT;
    uint32_t substate = 0u;
    float v_alpha = 0.0f;
    float vd_cmd = 0.0f;
    float vq_cmd = 0.0f;
    float theta_e = 0.0f;

    if (!open_loop_wait_next_adc_sample(&last_seq, &snap)) {
      ok = false;
      break;
    }
    (void)encoder_tracker_sample();
    if (!encoder_delta_ok()) {
      ok = false;
      break;
    }

    if (elapsed_ms < CURRENT_TRIP_ALIGN_RAMP_MS) {
      state = DIAG_STATE_ALIGN_RAMP;
      substate = elapsed_ms;
      v_alpha = COMM_ALIGN_V_ALPHA * ((float)elapsed_ms /
                                      (float)CURRENT_TRIP_ALIGN_RAMP_MS);
    } else if (elapsed_ms < (CURRENT_TRIP_ALIGN_RAMP_MS + CURRENT_TRIP_ALIGN_HOLD_MS)) {
      state = DIAG_STATE_ALIGN_HOLD;
      substate = elapsed_ms - CURRENT_TRIP_ALIGN_RAMP_MS;
      v_alpha = COMM_ALIGN_V_ALPHA;
      if (substate >= (CURRENT_TRIP_ALIGN_HOLD_MS - ENCODER_ALIGN_AVG_MS)) {
        align_sum += g_encoder_accum;
        align_count++;
      }
    } else if (elapsed_ms < (CURRENT_TRIP_ALIGN_RAMP_MS +
                             CURRENT_TRIP_ALIGN_HOLD_MS +
                             CURRENT_TRIP_ALIGN_DOWN_MS)) {
      state = DIAG_STATE_ALIGN_DOWN;
      substate = elapsed_ms - CURRENT_TRIP_ALIGN_RAMP_MS - CURRENT_TRIP_ALIGN_HOLD_MS;
      const float down_t = (float)substate / (float)CURRENT_TRIP_ALIGN_DOWN_MS;
      v_alpha = COMM_ALIGN_V_ALPHA * (1.0f - clampf(down_t, 0.0f, 1.0f));
    } else if (elapsed_ms < (CURRENT_TRIP_ALIGN_RAMP_MS +
                             CURRENT_TRIP_ALIGN_HOLD_MS +
                             CURRENT_TRIP_ALIGN_DOWN_MS +
                             CURRENT_TRIP_ZERO_RAMP_MS)) {
      state = DIAG_STATE_SWEEP_ZERO_RAMP;
      substate = elapsed_ms - CURRENT_TRIP_ALIGN_RAMP_MS -
                 CURRENT_TRIP_ALIGN_HOLD_MS - CURRENT_TRIP_ALIGN_DOWN_MS;
      vd_cmd = 0.0f;
    } else if (elapsed_ms < (CURRENT_TRIP_ALIGN_RAMP_MS +
                             CURRENT_TRIP_ALIGN_HOLD_MS +
                             CURRENT_TRIP_ALIGN_DOWN_MS +
                             CURRENT_TRIP_ZERO_RAMP_MS +
                             CURRENT_TRIP_ZERO_SETTLE_MS)) {
      state = DIAG_STATE_SWEEP_ZERO_SETTLE;
      substate = elapsed_ms - CURRENT_TRIP_ALIGN_RAMP_MS -
                 CURRENT_TRIP_ALIGN_HOLD_MS - CURRENT_TRIP_ALIGN_DOWN_MS -
                 CURRENT_TRIP_ZERO_RAMP_MS;
      vd_cmd = 0.0f;
    } else {
      state = DIAG_STATE_SWEEP_ZERO_SAMPLE;
      substate = elapsed_ms - CURRENT_TRIP_ALIGN_RAMP_MS -
                 CURRENT_TRIP_ALIGN_HOLD_MS - CURRENT_TRIP_ALIGN_DOWN_MS -
                 CURRENT_TRIP_ZERO_RAMP_MS - CURRENT_TRIP_ZERO_SETTLE_MS;
      vd_cmd = 0.0f;
    }

    if (align_count > 0u) {
      g_drv_test.encoder_align_count = align_sum / (int64_t)align_count;
      g_drv_test.theta_m_align = encoder_theta_m_from_accum(g_drv_test.encoder_align_count);
      g_drv_test.electrical_offset_runtime_rad =
          wrap_0_2pi_f(-((float)COMM_ENCODER_DIRECTION) *
                       (float)COMM_POLE_PAIRS *
                       g_drv_test.theta_m_align);
      g_drv_test.electrical_offset_runtime_deg =
          g_drv_test.electrical_offset_runtime_rad * 180.0f /
          3.14159265358979323846f;
    }

    if ((state == DIAG_STATE_SWEEP_ZERO_RAMP) ||
        (state == DIAG_STATE_SWEEP_ZERO_SETTLE) ||
        (state == DIAG_STATE_SWEEP_ZERO_SAMPLE)) {
      const float theta_m = encoder_theta_m_from_accum(g_encoder_accum);
      theta_e = encoder_theta_e_from_theta_m(theta_m,
                                             g_drv_test.electrical_offset_runtime_rad);
      if (theta_prev_valid &&
          angle_delta_abs(theta_e, theta_e_prev) > COMM_THETA_JUMP_MAX_RAD) {
        ok = false;
        break;
      }
      theta_prev_valid = true;
      theta_e_prev = theta_e;
    }

    const float vbus_v = board_read_vbus_v();
    if (vbus_v < OPEN_LOOP_VBUS_MIN_V || vbus_v > OPEN_LOOP_VBUS_MAX_V) {
      ok = false;
      break;
    }
    if ((state == DIAG_STATE_ALIGN_RAMP) ||
        (state == DIAG_STATE_ALIGN_HOLD) ||
        (state == DIAG_STATE_ALIGN_DOWN)) {
      encoder_apply_alpha_beta_svpwm(v_alpha, 0.0f, vbus_v);
    } else {
      encoder_apply_dq_svpwm(theta_e, vd_cmd, vq_cmd, vbus_v);
    }

    const CurrentDqSample sample = current_observe_calculate(&snap, theta_e);
    if (state == DIAG_STATE_SWEEP_ZERO_SAMPLE) {
      current_observe_stats_update(&g_drv_test.current_stats, &sample);
    }
    current_observe_lpf_update(&sample);
    if (sample.magnitude_max > g_drv_test.current_max_a) {
      g_drv_test.current_max_a = sample.magnitude_max;
    }
    const uint32_t u_dev = (snap.raw_u > g_drv_test.offset.offset_u)
                               ? (uint32_t)(snap.raw_u - g_drv_test.offset.offset_u)
                               : (uint32_t)(g_drv_test.offset.offset_u - snap.raw_u);
    const uint32_t v_dev = (snap.raw_v > g_drv_test.offset.offset_v)
                               ? (uint32_t)(snap.raw_v - g_drv_test.offset.offset_v)
                               : (uint32_t)(g_drv_test.offset.offset_v - snap.raw_v);
    if (u_dev > g_drv_test.raw_current_max_deviation) {
      g_drv_test.raw_current_max_deviation = u_dev;
    }
    if (v_dev > g_drv_test.raw_current_max_deviation) {
      g_drv_test.raw_current_max_deviation = v_dev;
    }
    if (snap.raw_u < g_drv_test.run_raw_u_min) { g_drv_test.run_raw_u_min = snap.raw_u; }
    if (snap.raw_u > g_drv_test.run_raw_u_max) { g_drv_test.run_raw_u_max = snap.raw_u; }
    if (snap.raw_v < g_drv_test.run_raw_v_min) { g_drv_test.run_raw_v_min = snap.raw_v; }
    if (snap.raw_v > g_drv_test.run_raw_v_max) { g_drv_test.run_raw_v_max = snap.raw_v; }

    const int64_t motion = g_encoder_accum - g_drv_test.encoder_start_count;
    const uint32_t motion_abs = abs_i32_to_u32((int32_t)motion);
    if (motion_abs > g_drv_test.encoder_motion_max_counts) {
      g_drv_test.encoder_motion_max_counts = motion_abs;
    }

    const bool raw_ok = (snap.raw_u > CURRENT_RAW_MIN_SAFE_COUNT) &&
                        (snap.raw_u < CURRENT_RAW_MAX_SAFE_COUNT) &&
                        (snap.raw_v > CURRENT_RAW_MIN_SAFE_COUNT) &&
                        (snap.raw_v < CURRENT_RAW_MAX_SAFE_COUNT);
    if (!current_trip_diag_record_and_check(state,
                                            substate,
                                            &snap,
                                            &sample,
                                            vd_cmd,
                                            vq_cmd,
                                            theta_e,
                                            vbus_v) ||
        !raw_ok ||
        !power_stage_update_phase_edge_timing() ||
        !nfault_ok() ||
        !m1_is_safe_off() ||
        !ccrs_in_open_loop_range()) {
      ok = false;
      break;
    }

    if ((now_ms - last_drv_status_ms) >= 100u) {
      last_drv_status_ms = now_ms;
      if (!drv8301_read_status(&g_drv0) || !drv8301_read_status(&g_drv1) ||
          drv_status_has_fault(g_drv0.status.status1_raw, g_drv0.status.status2_raw) ||
          drv_status_has_fault(g_drv1.status.status1_raw, g_drv1.status.status2_raw)) {
        ok = false;
        break;
      }
    }

    const bool print_now = (elapsed_ms == 0u) ||
                           ((elapsed_ms - last_print_ms) >= OPEN_LOOP_MONITOR_PERIOD_MS);
    if (print_now) {
      char line[512];
      const int32_t va_milli = float_to_scaled_i32(v_alpha, 1000.0f);
      const int32_t vd_milli = float_to_scaled_i32(vd_cmd, 1000.0f);
      const int32_t vq_milli = float_to_scaled_i32(vq_cmd, 1000.0f);
      const int32_t iu_milli = float_to_scaled_i32(sample.iu, 1000.0f);
      const int32_t iv_milli = float_to_scaled_i32(sample.iv, 1000.0f);
      const int32_t iw_milli = float_to_scaled_i32(sample.iw, 1000.0f);
      const int32_t id_milli = float_to_scaled_i32(sample.id, 1000.0f);
      const int32_t iq_milli = float_to_scaled_i32(sample.iq, 1000.0f);
      const uint32_t current_max_milli = float_to_scaled_u32(sample.magnitude_max, 1000.0f);
      char encoder_accum_s[24];
      i64_to_dec(encoder_accum_s, sizeof(encoder_accum_s), g_encoder_accum);
      last_print_ms = elapsed_ms;
      snprintf(line,
               sizeof(line),
               "current_trip_diag: elapsed_ms=%lu state=%s v_alpha=%s%lu.%03lu vd=%s%lu.%03lu vq=%s%lu.%03lu iu=%s%lu.%03lu iv=%s%lu.%03lu iw=%s%lu.%03lu id=%s%lu.%03lu iq=%s%lu.%03lu current_abs_max=%lu.%03lu over_consecutive_max=%lu encoder_accum=%s MOE=%lu EN_GATE=%lu",
               (unsigned long)elapsed_ms,
               test_state_name(state),
               (va_milli < 0) ? "-" : "",
               (unsigned long)(abs_i32_to_u32(va_milli) / 1000u),
               (unsigned long)(abs_i32_to_u32(va_milli) % 1000u),
               (vd_milli < 0) ? "-" : "",
               (unsigned long)(abs_i32_to_u32(vd_milli) / 1000u),
               (unsigned long)(abs_i32_to_u32(vd_milli) % 1000u),
               (vq_milli < 0) ? "-" : "",
               (unsigned long)(abs_i32_to_u32(vq_milli) / 1000u),
               (unsigned long)(abs_i32_to_u32(vq_milli) % 1000u),
               (iu_milli < 0) ? "-" : "",
               (unsigned long)(abs_i32_to_u32(iu_milli) / 1000u),
               (unsigned long)(abs_i32_to_u32(iu_milli) % 1000u),
               (iv_milli < 0) ? "-" : "",
               (unsigned long)(abs_i32_to_u32(iv_milli) / 1000u),
               (unsigned long)(abs_i32_to_u32(iv_milli) % 1000u),
               (iw_milli < 0) ? "-" : "",
               (unsigned long)(abs_i32_to_u32(iw_milli) / 1000u),
               (unsigned long)(abs_i32_to_u32(iw_milli) % 1000u),
               (id_milli < 0) ? "-" : "",
               (unsigned long)(abs_i32_to_u32(id_milli) / 1000u),
               (unsigned long)(abs_i32_to_u32(id_milli) % 1000u),
               (iq_milli < 0) ? "-" : "",
               (unsigned long)(abs_i32_to_u32(iq_milli) / 1000u),
               (unsigned long)(abs_i32_to_u32(iq_milli) % 1000u),
               (unsigned long)(current_max_milli / 1000u),
               (unsigned long)(current_max_milli % 1000u),
               (unsigned long)g_drv_test.current_trip_diag.current_over_consecutive_max,
               encoder_accum_s,
               (unsigned long)(((TIM1->BDTR & TIM_BDTR_MOE) != 0u) ? 1u : 0u),
               (unsigned long)(gate_raw_is_high() ? 1u : 0u));
      uart2_printf_line(line);
    }
  }

  if (align_count > 0u) {
    g_drv_test.encoder_align_count = align_sum / (int64_t)align_count;
    g_drv_test.theta_m_align = encoder_theta_m_from_accum(g_drv_test.encoder_align_count);
    g_drv_test.electrical_offset_runtime_rad =
        wrap_0_2pi_f(-((float)COMM_ENCODER_DIRECTION) *
                     (float)COMM_POLE_PAIRS *
                     g_drv_test.theta_m_align);
    g_drv_test.electrical_offset_runtime_deg =
        g_drv_test.electrical_offset_runtime_rad * 180.0f /
        3.14159265358979323846f;
  }

  g_drv_test.current_trip_diag.completed_zero_baseline =
      ok && !g_drv_test.current_trip_fault.latched;
  current_trip_diag_classify();

  encoder_apply_dq_svpwm(encoder_theta_e_from_theta_m(encoder_theta_m_from_accum(g_encoder_accum),
                                                      g_drv_test.electrical_offset_runtime_rad),
                         0.0f,
                         0.0f,
                         board_read_vbus_v());
  power_stage_set_ccr_half();
  power_stage_disable_six_outputs();
  power_stage_force_safe_off_zero_ccr();
  hal_pwm_start_adc_trigger_only();
  power_stage_force_safe_off_zero_ccr();
  m1_force_safe_off();

  g_drv_test.encoder_end_count = g_encoder_accum;
  g_drv_test.encoder_total_delta =
      g_drv_test.encoder_end_count - g_drv_test.encoder_start_count;
  g_drv_test.encoder_data_reliable = true;
  return ok && !g_drv_test.current_trip_fault.latched;
}

static void drv_bringup_test_run(void)
{
  memset(&g_drv_test, 0, sizeof(g_drv_test));
  g_drv_test.ran = true;

  drv_bringup_configure_nfault_pull(GPIO_PULLUP);
  current_observe_configure_scale();
  m1_force_safe_off();
  power_stage_startup_safe_half_ccr();
  HAL_Delay(2u);
  g_drv_test.adc_seq_before = drv_bringup_get_adc_seq();

  if (!g_drv_test.current_scale_known) {
    drv_bringup_mark_fault(AXIS0_FAULT_CURRENT_SENSOR_INVALID);
    drv_bringup_fail(1u);
    return;
  }

  if (!power_stage_channels_off() || !gate_raw_is_low() ||
      TIM1->CCR1 != tim1_half_ccr() ||
      TIM1->CCR2 != tim1_half_ccr() ||
      TIM1->CCR3 != tim1_half_ccr() ||
      !m1_is_safe_off()) {
    drv_bringup_mark_fault(AXIS0_FAULT_PWM_NOT_ENABLED);
    drv_bringup_fail(1u);
    return;
  }
  g_drv_test.m1_safe = true;

  g_drv_test.encoder_ppr = COMM_ENCODER_PPR;
  g_drv_test.encoder_cpr = COMM_ENCODER_CPR;
  g_drv_test.encoder_direction = COMM_ENCODER_DIRECTION;
  g_drv_test.pole_pairs_runtime = COMM_POLE_PAIRS;
  g_drv_test.encoder_counts_per_ab_cycle = 4u;
  g_drv_test.encoder_idle_pass = encoder_idle_check();
  if (!g_drv_test.encoder_idle_pass) {
    drv_bringup_mark_fault(AXIS0_FAULT_ENCODER_CALIBRATION_FAILED);
    drv_bringup_fail(2u);
    return;
  }

  hal_gpio_set_gate_enable(true);
  if (!power_stage_wait_nfault_release()) {
    drv_bringup_fail(3u);
    return;
  }
  HAL_Delay(10u);

  if (!power_stage_configure_drivers()) {
    drv_bringup_fail(4u);
    return;
  }

  if (!power_stage_check_adc_sample_timing()) {
    drv_bringup_mark_fault(AXIS0_FAULT_CURRENT_SENSOR_INVALID);
    drv_bringup_fail(5u);
    return;
  }

  power_stage_disable_six_outputs();
  power_stage_set_ccr_half();
  g_drv_test.adc_offset_pass = power_stage_collect_dc_cal_offsets();
  if (!g_drv_test.adc_offset_pass) {
    drv_bringup_fail(6u);
    return;
  }

  if (!gate_raw_is_high() || !nfault_ok()) {
    drv_bringup_fail(7u);
    return;
  }

  const uint32_t observe_start_ms = HAL_GetTick();
  const uint32_t observe_start_seq = drv_bringup_get_adc_seq();
  g_drv_test.open_loop_pass = static_d_axis_voltage_sweep_run();
  const uint32_t observe_end_ms = HAL_GetTick();
  const uint32_t observe_end_seq = drv_bringup_get_adc_seq();
  g_drv_test.adc_seq_after = drv_bringup_get_adc_seq();
  g_drv_test.adc_seq_growing = g_drv_test.adc_seq_after > g_drv_test.adc_seq_before;
  g_drv_test.adc_snapshot_count = observe_end_seq - observe_start_seq;
  g_drv_test.pwm_cycle_count = (observe_end_ms > observe_start_ms)
                                   ? ((observe_end_ms - observe_start_ms) *
                                      PWM_CYCLES_PER_MS)
                                   : 0u;
  g_drv_test.snapshots_per_pwm_cycle =
      (g_drv_test.pwm_cycle_count > 0u)
          ? ((float)g_drv_test.adc_snapshot_count /
             (float)g_drv_test.pwm_cycle_count)
          : 0.0f;
  g_drv_test.adc_sync_rate_ok =
      (g_drv_test.snapshots_per_pwm_cycle >= SNAPSHOT_RATE_MIN_PER_PWM) &&
      (g_drv_test.snapshots_per_pwm_cycle <= SNAPSHOT_RATE_MAX_PER_PWM);
  g_drv_test.raw_range_ok = (g_drv_test.run_raw_u_min > CURRENT_RAW_MIN_SAFE_COUNT) &&
                            (g_drv_test.run_raw_u_max < CURRENT_RAW_MAX_SAFE_COUNT) &&
                            (g_drv_test.run_raw_v_min > CURRENT_RAW_MIN_SAFE_COUNT) &&
                            (g_drv_test.run_raw_v_max < CURRENT_RAW_MAX_SAFE_COUNT);
  if (g_drv_test.current_stats.samples > 0u) {
    float id_std = 0.0f;
    float iq_std = 0.0f;
    float id_mean = 0.0f;
    float iq_mean = 0.0f;
    current_observe_stats_finalize(&g_drv_test.current_stats,
                                   &id_mean,
                                   &iq_mean,
                                   &id_std,
                                   &iq_std);
    g_drv_test.id_mean_effective_counts =
        (g_drv_test.current_amp_per_count > 0.0f) ? (id_mean / g_drv_test.current_amp_per_count) : 0.0f;
    g_drv_test.iq_mean_effective_counts =
        (g_drv_test.current_amp_per_count > 0.0f) ? (iq_mean / g_drv_test.current_amp_per_count) : 0.0f;
    g_drv_test.current_direction_ok = g_drv_test.d_axis_polarity_ok;
    g_drv_test.id_iq_abs_ratio = (fabsf(id_mean) > 0.001f)
                                     ? (fabsf(iq_mean) / fabsf(id_mean))
                                     : 999.0f;
    g_drv_test.current_resolution_ok =
        g_drv_test.first_reliable_current_voltage >= 0.0f;
    g_drv_test.dq_alignment_ok = false;
    for (uint32_t i = 0u; i < g_drv_test.sweep_point_count; ++i) {
      const SweepPointResult *pt = &g_drv_test.sweep_points[i];
      if (pt->valid && fabsf(pt->id_effective_counts) >= D_AXIS_RELIABLE_MIN_COUNTS) {
        g_drv_test.dq_alignment_ok = true;
      }
    }
    g_drv_test.kcl_ok = g_drv_test.kcl_residual_max < 0.001f;
    g_drv_test.final_observe_reliable =
        g_drv_test.current_direction_ok &&
        g_drv_test.current_resolution_ok &&
        g_drv_test.dq_alignment_ok &&
        g_drv_test.kcl_ok &&
        g_drv_test.d_axis_motion_ok &&
        g_drv_test.phase_resistance_est_reliable &&
        g_drv_test.current_monotonic_ok;
  } else {
    g_drv_test.current_direction_ok = false;
    g_drv_test.dq_alignment_ok = false;
    g_drv_test.current_resolution_ok = false;
    g_drv_test.kcl_ok = false;
    g_drv_test.final_observe_reliable = false;
    g_drv_test.id_iq_abs_ratio = 999.0f;
  }
  if (!g_drv_test.open_loop_pass || !g_drv_test.adc_seq_growing ||
      !g_drv_test.raw_range_ok || !g_drv_test.gain40_readback_ok ||
      !g_drv_test.adc_phase_edge_timing_ok || !g_drv_test.adc_sync_rate_ok ||
      !m1_is_safe_off()) {
    if (g_drv_test.current_trip_fault.latched) {
      drv_bringup_mark_fault(AXIS0_FAULT_PHASE_OVERCURRENT);
    } else {
      drv_bringup_mark_fault(AXIS0_FAULT_ENCODER_CALIBRATION_FAILED);
    }
    drv_bringup_fail(8u);
    return;
  }

  power_stage_force_safe_off_zero_ccr();
  hal_pwm_start_adc_trigger_only();
  power_stage_force_safe_off_zero_ccr();
  m1_force_safe_off();
  g_drv_test.pass = true;
  g_drv_test.fault_code = g_axis0.fault_flags;
  HAL_Delay(2u);
  drv_bringup_capture_final_state();
}

static void print_bringup_status(void)
{
  char line[512];

  HalAdcSnapshot snap = {0};
  HalAdcDiagnostics adc_diag = {0};
  HalPwmDiagnostics pwm_diag = {0};
  const bool snap_ok = hal_adc_get_snapshot(&snap);
  const BoardOdriveV36Status board_status = board_get_status();
  const unsigned int enc_cnt = (unsigned int)__HAL_TIM_GET_COUNTER(&htim3);
  hal_adc_get_diagnostics(&adc_diag);
  hal_pwm_get_diagnostics(&pwm_diag);

  snprintf(line,
           sizeof(line),
           "bringup: adc_init=%u board_init=%u irq=%lu cb=%lu cb1=%lu cb2=%lu snapcnt=%lu snap_ok=%u valid=%u seq=%lu raw_u=%u raw_v=%u raw_vbus=%u enc_cnt=%u nfault=%u gate=%u pwm_disabled=%u tim_base=%lu tim_oc4=%lu adc1_start=%lu adc2_start=%lu fault=0x%08lX",
           (unsigned int)g_adc_init_ok,
           (unsigned int)g_board_init_ok,
           (unsigned long)adc_diag.irq_count,
           (unsigned long)g_adc_callback_count,
           (unsigned long)adc_diag.adc1_callback_count,
           (unsigned long)adc_diag.adc2_callback_count,
           (unsigned long)adc_diag.snapshot_count,
           (unsigned int)snap_ok,
           (unsigned int)snap.valid,
           (unsigned long)snap.seq,
           (unsigned int)snap.raw_u,
           (unsigned int)snap.raw_v,
           (unsigned int)snap.raw_vbus,
           enc_cnt,
           (unsigned int)board_status.drv_nfault_active,
           (unsigned int)board_status.drv_gate_enabled,
           (unsigned int)board_status.pwm_disabled,
           (unsigned long)pwm_diag.base_start_status,
           (unsigned long)pwm_diag.oc4_start_status,
           (unsigned long)adc_diag.injected_start_adc1_status,
           (unsigned long)adc_diag.injected_start_adc2_status,
           (unsigned long)g_axis0.fault_flags);

  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "tim1: cr1=0x%08lX cnt=%lu arr=%lu ccr4=%lu ccer=0x%08lX sr=0x%08lX bdtr=0x%08lX",
           (unsigned long)TIM1->CR1,
           (unsigned long)TIM1->CNT,
           (unsigned long)TIM1->ARR,
           (unsigned long)TIM1->CCR4,
           (unsigned long)TIM1->CCER,
           (unsigned long)TIM1->SR,
           (unsigned long)TIM1->BDTR);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "adc1: cr1=0x%08lX cr2=0x%08lX sr=0x%08lX jsqr=0x%08lX",
           (unsigned long)ADC1->CR1,
           (unsigned long)ADC1->CR2,
           (unsigned long)ADC1->SR,
           (unsigned long)ADC1->JSQR);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "adc2: cr1=0x%08lX cr2=0x%08lX sr=0x%08lX jsqr=0x%08lX",
           (unsigned long)ADC2->CR1,
           (unsigned long)ADC2->CR2,
           (unsigned long)ADC2->SR,
           (unsigned long)ADC2->JSQR);
  uart2_printf_line(line);
}

static void print_drv_bringup_test_status(void)
{
  char line[1536];

  snprintf(line,
           sizeof(line),
           "static_d_axis_voltage_sweep_test: ran=%u pass=%u gain40_readback_ok=%u dc_cal_offsets_pass=%u dc_cal_clear_ok=%u adc_phase_edge_timing_ok=%u adc_sync_rate_ok=%u final_observe_reliable=%u current_monotonic_ok=%u phase_resistance_est_reliable=%u d_axis_signal_ok=%u d_axis_polarity_ok=%u d_axis_motion_ok=%u current_resolution_ok=%u dq_alignment_ok=%u kcl_ok=%u fail_step=%lu fault_code=0x%08lX nfault_released=%u nfault_release_ms=%lu encoder_idle_pass=%u adc_offset_pass=%u drv0_cfg=%u drv1_cfg=%u drv0_status_ok=%u drv1_status_ok=%u observe_pass=%u adc_seq_before=%lu adc_seq_after=%lu adc_seq_growing=%u raw_range_ok=%u m1_safe=%u",
           (unsigned int)g_drv_test.ran,
           (unsigned int)g_drv_test.pass,
           (unsigned int)g_drv_test.gain40_readback_ok,
           (unsigned int)g_drv_test.dc_cal_offsets_pass,
           (unsigned int)g_drv_test.dc_cal_clear_ok,
           (unsigned int)g_drv_test.adc_phase_edge_timing_ok,
           (unsigned int)g_drv_test.adc_sync_rate_ok,
           (unsigned int)g_drv_test.final_observe_reliable,
           (unsigned int)g_drv_test.current_monotonic_ok,
           (unsigned int)g_drv_test.phase_resistance_est_reliable,
           (unsigned int)g_drv_test.d_axis_signal_ok,
           (unsigned int)g_drv_test.d_axis_polarity_ok,
           (unsigned int)g_drv_test.d_axis_motion_ok,
           (unsigned int)g_drv_test.current_resolution_ok,
           (unsigned int)g_drv_test.dq_alignment_ok,
           (unsigned int)g_drv_test.kcl_ok,
           (unsigned long)g_drv_test.fail_step,
           (unsigned long)g_drv_test.fault_code,
           (unsigned int)g_drv_test.nfault_released,
           (unsigned long)g_drv_test.nfault_release_ms,
           (unsigned int)g_drv_test.encoder_idle_pass,
           (unsigned int)g_drv_test.adc_offset_pass,
           (unsigned int)g_drv_test.drv0_cfg,
           (unsigned int)g_drv_test.drv1_cfg,
           (unsigned int)g_drv_test.drv0_status_ok,
           (unsigned int)g_drv_test.drv1_status_ok,
           (unsigned int)g_drv_test.open_loop_pass,
           (unsigned long)g_drv_test.adc_seq_before,
           (unsigned long)g_drv_test.adc_seq_after,
           (unsigned int)g_drv_test.adc_seq_growing,
           (unsigned int)g_drv_test.raw_range_ok,
           (unsigned int)m1_is_safe_off());
  uart2_printf_line(line);

  if (!g_drv_test.current_scale_known) {
    uart2_printf_line("CURRENT_SCALE_UNKNOWN");
  }

  const uint32_t shunt_uohm = float_to_scaled_u32(g_drv_test.current_shunt_ohm, 1000000.0f);
  const uint32_t gain_centi = float_to_scaled_u32(g_drv_test.current_amp_gain, 100.0f);
  const uint32_t vref_milli = float_to_scaled_u32(g_drv_test.adc_vref_v, 1000.0f);
  const uint32_t amp_per_count_micro = float_to_scaled_u32(g_drv_test.current_amp_per_count, 1000000.0f);
  snprintf(line,
           sizeof(line),
           "current_scale: current_shunt_ohm=%lu.%06lu current_amp_gain=%lu.%02lu adc_vref=%lu.%03lu adc_resolution_counts=4096 current_amp_per_count=%lu.%06lu adc_map=ADC2_IN10/M0_SO1->raw_u,ADC2_IN11/M0_SO2->raw_v current_polarity_u=%ld current_polarity_v=%ld",
           (unsigned long)(shunt_uohm / 1000000u),
           (unsigned long)(shunt_uohm % 1000000u),
           (unsigned long)(gain_centi / 100u),
           (unsigned long)(gain_centi % 100u),
           (unsigned long)(vref_milli / 1000u),
           (unsigned long)(vref_milli % 1000u),
           (unsigned long)(amp_per_count_micro / 1000000u),
           (unsigned long)(amp_per_count_micro % 1000000u),
           (long)g_drv_test.current_polarity_u,
           (long)g_drv_test.current_polarity_v);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "drv_gain40: expected_control2=0x%04X actual_control2_drv0=0x%04X actual_control2_drv1=0x%04X gain_field_drv0=%u gain_field_drv1=%u gain40_readback_ok=%u",
           (unsigned int)g_drv_test.expected_control2,
           (unsigned int)g_drv_test.actual_control2_drv0,
           (unsigned int)g_drv_test.actual_control2_drv1,
           (unsigned int)g_drv_test.gain_field_drv0,
           (unsigned int)g_drv_test.gain_field_drv1,
           (unsigned int)g_drv_test.gain40_readback_ok);
  uart2_printf_line(line);

  const uint32_t noise_counts_milli = float_to_scaled_u32(g_drv_test.noise_pp_counts, 1000.0f);
  const uint32_t noise_amp_milli = float_to_scaled_u32(g_drv_test.noise_pp_amp, 1000.0f);
  snprintf(line,
           sizeof(line),
           "dc_cal_offset: dc_cal_offsets_pass=%u dc_cal_clear_ok=%u noise_pp_counts=%lu.%03lu noise_pp_amp=%lu.%03lu",
           (unsigned int)g_drv_test.dc_cal_offsets_pass,
           (unsigned int)g_drv_test.dc_cal_clear_ok,
           (unsigned long)(noise_counts_milli / 1000u),
           (unsigned long)(noise_counts_milli % 1000u),
           (unsigned long)(noise_amp_milli / 1000u),
           (unsigned long)(noise_amp_milli % 1000u));
  uart2_printf_line(line);

  const uint32_t u_std_milli = float_to_scaled_u32(g_drv_test.dc_noise.u_std_counts, 1000.0f);
  const uint32_t v_std_milli = float_to_scaled_u32(g_drv_test.dc_noise.v_std_counts, 1000.0f);
  const uint32_t u_std_amp_milli = float_to_scaled_u32(g_drv_test.dc_noise.u_std_amp, 1000.0f);
  const uint32_t v_std_amp_milli = float_to_scaled_u32(g_drv_test.dc_noise.v_std_amp, 1000.0f);
  snprintf(line,
           sizeof(line),
           "dc_cal_noise: samples=%lu u_mean=%lu v_mean=%lu u_min=%u u_max=%u v_min=%u v_max=%u u_p2p=%u v_p2p=%u u_std_counts=%lu.%03lu v_std_counts=%lu.%03lu u_std_amp=%lu.%03lu v_std_amp=%lu.%03lu u_range99=%u v_range99=%u glitch=%u",
           (unsigned long)g_drv_test.dc_noise.samples,
           (unsigned long)g_drv_test.dc_noise.mean_u,
           (unsigned long)g_drv_test.dc_noise.mean_v,
           (unsigned int)g_drv_test.dc_noise.u_min,
           (unsigned int)g_drv_test.dc_noise.u_max,
           (unsigned int)g_drv_test.dc_noise.v_min,
           (unsigned int)g_drv_test.dc_noise.v_max,
           (unsigned int)g_drv_test.dc_noise.u_p2p,
           (unsigned int)g_drv_test.dc_noise.v_p2p,
           (unsigned long)(u_std_milli / 1000u),
           (unsigned long)(u_std_milli % 1000u),
           (unsigned long)(v_std_milli / 1000u),
           (unsigned long)(v_std_milli % 1000u),
           (unsigned long)(u_std_amp_milli / 1000u),
           (unsigned long)(u_std_amp_milli % 1000u),
           (unsigned long)(v_std_amp_milli / 1000u),
           (unsigned long)(v_std_amp_milli % 1000u),
           (unsigned int)g_drv_test.dc_noise.u_range99_counts,
           (unsigned int)g_drv_test.dc_noise.v_range99_counts,
           (unsigned int)g_drv_test.dc_noise.glitch_detected);
  uart2_printf_line(line);

  const uint32_t pwm_period_milli = float_to_scaled_u32(g_drv_test.pwm_period_us, 1000.0f);
  const uint32_t edge_margin_milli =
      float_to_scaled_u32(g_drv_test.distance_to_nearest_pwm_edge_us, 1000.0f);
  snprintf(line,
           sizeof(line),
           "adc_phase_edge_timing: pwm_period_us=%lu.%03lu adc_trigger_position_counts=%lu min_distance_to_phase_edge_counts=%lu min_distance_to_phase_edge_us=%lu.%03lu ccr1=%lu ccr2=%lu ccr3=%lu ccr4=%lu tim_dir=%lu adc_phase_edge_timing_ok=%u",
           (unsigned long)(pwm_period_milli / 1000u),
           (unsigned long)(pwm_period_milli % 1000u),
           (unsigned long)g_drv_test.adc_trigger_position_counts,
           (unsigned long)g_drv_test.min_distance_to_phase_edge_counts,
           (unsigned long)(edge_margin_milli / 1000u),
           (unsigned long)(edge_margin_milli % 1000u),
           (unsigned long)TIM1->CCR1,
           (unsigned long)TIM1->CCR2,
           (unsigned long)TIM1->CCR3,
           (unsigned long)g_drv_test.adc_trigger_position_counts,
           (unsigned long)((TIM1->CR1 & TIM_CR1_DIR) ? 1u : 0u),
           (unsigned int)g_drv_test.adc_phase_edge_timing_ok);
  uart2_printf_line(line);
  if (!g_drv_test.adc_phase_edge_timing_ok) {
    uart2_printf_line("ADC_PHASE_EDGE_TIMING_UNSAFE");
  }

  const uint32_t snapshots_per_pwm_milli =
      float_to_scaled_u32(g_drv_test.snapshots_per_pwm_cycle, 1000.0f);
  snprintf(line,
           sizeof(line),
           "adc_sync_rate: pwm_cycle_count=%lu adc_snapshot_count=%lu snapshots_per_pwm_cycle=%lu.%03lu adc_sync_rate_ok=%u",
           (unsigned long)g_drv_test.pwm_cycle_count,
           (unsigned long)g_drv_test.adc_snapshot_count,
           (unsigned long)(snapshots_per_pwm_milli / 1000u),
           (unsigned long)(snapshots_per_pwm_milli % 1000u),
           (unsigned int)g_drv_test.adc_sync_rate_ok);
  uart2_printf_line(line);

  const SampleWindowDiag *sw = &g_drv_test.sample_window_diag;
  snprintf(line,
           sizeof(line),
           "sample_window_diag: TIM1_CNT=%lu sample_cnt=%lu count_dir=%lu pwm_mode=%lu ccer=0x%08lX bdtr=0x%08lX ccr1=%lu ccr2=%lu ccr3=%lu ccr4=%lu high_mask=0x%lX low_mask=0x%lX deadtime_mask=0x%lX low_side_active_count=%lu current_sample_window_valid=%u",
           (unsigned long)sw->current_tim1_cnt,
           (unsigned long)sw->sample_cnt,
           (unsigned long)sw->current_tim1_dir,
           (unsigned long)sw->pwm_mode,
           (unsigned long)sw->ccer,
           (unsigned long)sw->bdtr,
           (unsigned long)sw->ccr1,
           (unsigned long)sw->ccr2,
           (unsigned long)sw->ccr3,
           (unsigned long)sw->sample_cnt,
           (unsigned long)sw->sample_high_side_mask,
           (unsigned long)sw->sample_low_side_mask,
           (unsigned long)sw->deadtime_mask,
           (unsigned long)sw->low_side_active_count,
           (unsigned int)sw->current_sample_window_valid);
  uart2_printf_line(line);
  if (!sw->current_sample_window_valid) {
    uart2_printf_line("LOW_SIDE_CURRENT_SAMPLE_STATE_INVALID");
  }

  snprintf(line,
           sizeof(line),
           "encoder_info: encoder_ppr=%ld encoder_cpr=%ld encoder_direction=%ld pole_pairs=%lu tim3_mode=TIM_ENCODERMODE_TI12 counts_per_ab_cycle=%lu",
           (long)g_drv_test.encoder_ppr,
           (long)g_drv_test.encoder_cpr,
           (long)g_drv_test.encoder_direction,
           (unsigned long)g_drv_test.pole_pairs_runtime,
           (unsigned long)g_drv_test.encoder_counts_per_ab_cycle);
  uart2_printf_line(line);
  if (g_drv_test.encoder_cpr <= 0) {
    uart2_printf_line("ENCODER_CPR_UNKNOWN");
  }

  char enc_idle_min_s[24];
  char enc_idle_max_s[24];
  i64_to_dec(enc_idle_min_s, sizeof(enc_idle_min_s), g_drv_test.enc_idle_min);
  i64_to_dec(enc_idle_max_s, sizeof(enc_idle_max_s), g_drv_test.enc_idle_max);
  snprintf(line,
           sizeof(line),
           "encoder_idle: enc_idle_min=%s enc_idle_max=%s enc_idle_noise_pp=%lu",
           enc_idle_min_s,
           enc_idle_max_s,
           (unsigned long)g_drv_test.enc_idle_noise_pp);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "offset: samples=%lu offset_u=%lu offset_v=%lu u_min=%u u_max=%u v_min=%u v_max=%u u_noise_pp=%u v_noise_pp=%u",
           (unsigned long)g_drv_test.offset.samples,
           (unsigned long)g_drv_test.offset.offset_u,
           (unsigned long)g_drv_test.offset.offset_v,
           (unsigned int)g_drv_test.offset.u_min,
           (unsigned int)g_drv_test.offset.u_max,
           (unsigned int)g_drv_test.offset.v_min,
           (unsigned int)g_drv_test.offset.v_max,
           (unsigned int)g_drv_test.offset.u_noise_pp,
           (unsigned int)g_drv_test.offset.v_noise_pp);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "drv0_regs: status1=0x%04X status2=0x%04X control1=0x%04X control2=0x%04X",
           (unsigned int)g_drv_test.drv0_regs.status1,
           (unsigned int)g_drv_test.drv0_regs.status2,
           (unsigned int)g_drv_test.drv0_regs.control1,
           (unsigned int)g_drv_test.drv0_regs.control2);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "drv1_regs: status1=0x%04X status2=0x%04X control1=0x%04X control2=0x%04X",
           (unsigned int)g_drv_test.drv1_regs.status1,
           (unsigned int)g_drv_test.drv1_regs.status2,
           (unsigned int)g_drv_test.drv1_regs.control1,
           (unsigned int)g_drv_test.drv1_regs.control2);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "current_obs_raw_range: samples=%lu raw_u_min=%u raw_u_max=%u raw_v_min=%u raw_v_max=%u adc_max_deviation_counts=%lu current_max_a=%s%lu.%03lu",
           (unsigned long)g_drv_test.monitor_count,
           (unsigned int)g_drv_test.run_raw_u_min,
           (unsigned int)g_drv_test.run_raw_u_max,
           (unsigned int)g_drv_test.run_raw_v_min,
           (unsigned int)g_drv_test.run_raw_v_max,
           (unsigned long)g_drv_test.raw_current_max_deviation,
           (float_to_scaled_i32(g_drv_test.current_max_a, 1000.0f) < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(float_to_scaled_i32(g_drv_test.current_max_a, 1000.0f)) / 1000u),
           (unsigned long)(abs_i32_to_u32(float_to_scaled_i32(g_drv_test.current_max_a, 1000.0f)) % 1000u));
  uart2_printf_line(line);

  const CurrentObserveStats *st = &g_drv_test.current_stats;
  const float inv_samples = (st->samples > 0u) ? (1.0f / (float)st->samples) : 0.0f;
  const int32_t iu_min = float_to_scaled_i32(st->iu_min, 1000.0f);
  const int32_t iu_max = float_to_scaled_i32(st->iu_max, 1000.0f);
  const int32_t iu_mean = float_to_scaled_i32(st->iu_sum * inv_samples, 1000.0f);
  const int32_t iv_min = float_to_scaled_i32(st->iv_min, 1000.0f);
  const int32_t iv_max = float_to_scaled_i32(st->iv_max, 1000.0f);
  const int32_t iv_mean = float_to_scaled_i32(st->iv_sum * inv_samples, 1000.0f);
  const int32_t iw_min = float_to_scaled_i32(st->iw_min, 1000.0f);
  const int32_t iw_max = float_to_scaled_i32(st->iw_max, 1000.0f);
  const int32_t iw_mean = float_to_scaled_i32(st->iw_sum * inv_samples, 1000.0f);
  snprintf(line,
           sizeof(line),
           "current_phase_stats: samples=%lu iu_min=%s%lu.%03lu iu_max=%s%lu.%03lu iu_mean=%s%lu.%03lu iv_min=%s%lu.%03lu iv_max=%s%lu.%03lu iv_mean=%s%lu.%03lu iw_min=%s%lu.%03lu iw_max=%s%lu.%03lu iw_mean=%s%lu.%03lu",
           (unsigned long)st->samples,
           (iu_min < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(iu_min) / 1000u), (unsigned long)(abs_i32_to_u32(iu_min) % 1000u),
           (iu_max < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(iu_max) / 1000u), (unsigned long)(abs_i32_to_u32(iu_max) % 1000u),
           (iu_mean < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(iu_mean) / 1000u), (unsigned long)(abs_i32_to_u32(iu_mean) % 1000u),
           (iv_min < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(iv_min) / 1000u), (unsigned long)(abs_i32_to_u32(iv_min) % 1000u),
           (iv_max < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(iv_max) / 1000u), (unsigned long)(abs_i32_to_u32(iv_max) % 1000u),
           (iv_mean < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(iv_mean) / 1000u), (unsigned long)(abs_i32_to_u32(iv_mean) % 1000u),
           (iw_min < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(iw_min) / 1000u), (unsigned long)(abs_i32_to_u32(iw_min) % 1000u),
           (iw_max < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(iw_max) / 1000u), (unsigned long)(abs_i32_to_u32(iw_max) % 1000u),
           (iw_mean < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(iw_mean) / 1000u), (unsigned long)(abs_i32_to_u32(iw_mean) % 1000u));
  uart2_printf_line(line);

  const int32_t id_min = float_to_scaled_i32(st->id_min, 1000.0f);
  const int32_t id_max = float_to_scaled_i32(st->id_max, 1000.0f);
  const int32_t id_mean = float_to_scaled_i32(st->id_sum * inv_samples, 1000.0f);
  const int32_t iq_min = float_to_scaled_i32(st->iq_min, 1000.0f);
  const int32_t iq_max = float_to_scaled_i32(st->iq_max, 1000.0f);
  const int32_t iq_mean = float_to_scaled_i32(st->iq_sum * inv_samples, 1000.0f);
  const uint32_t id_iq_ratio_centi = float_to_scaled_u32(g_drv_test.id_iq_abs_ratio, 100.0f);
  const uint32_t current_mag_milli = float_to_scaled_u32(st->current_magnitude_max, 1000.0f);
  const int32_t id_counts_centi =
      float_to_scaled_i32(g_drv_test.id_mean_effective_counts, 100.0f);
  const int32_t iq_counts_centi =
      float_to_scaled_i32(g_drv_test.iq_mean_effective_counts, 100.0f);
  const uint32_t kcl_micro = float_to_scaled_u32(g_drv_test.kcl_residual_max, 1000000.0f);
  snprintf(line,
           sizeof(line),
           "current_dq_stats: id_min=%s%lu.%03lu id_max=%s%lu.%03lu id_mean=%s%lu.%03lu iq_min=%s%lu.%03lu iq_max=%s%lu.%03lu iq_mean=%s%lu.%03lu id_mean_effective_counts=%s%lu.%02lu iq_mean_effective_counts=%s%lu.%02lu id_direction=%s abs_iq_mean_over_abs_id_mean=%lu.%02lu current_magnitude_max=%lu.%03lu kcl_residual_max=%lu.%06lu",
           (id_min < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(id_min) / 1000u), (unsigned long)(abs_i32_to_u32(id_min) % 1000u),
           (id_max < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(id_max) / 1000u), (unsigned long)(abs_i32_to_u32(id_max) % 1000u),
           (id_mean < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(id_mean) / 1000u), (unsigned long)(abs_i32_to_u32(id_mean) % 1000u),
           (iq_min < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(iq_min) / 1000u), (unsigned long)(abs_i32_to_u32(iq_min) % 1000u),
           (iq_max < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(iq_max) / 1000u), (unsigned long)(abs_i32_to_u32(iq_max) % 1000u),
           (iq_mean < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(iq_mean) / 1000u), (unsigned long)(abs_i32_to_u32(iq_mean) % 1000u),
           (id_counts_centi < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(id_counts_centi) / 100u), (unsigned long)(abs_i32_to_u32(id_counts_centi) % 100u),
           (iq_counts_centi < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(iq_counts_centi) / 100u), (unsigned long)(abs_i32_to_u32(iq_counts_centi) % 100u),
           g_drv_test.current_direction_ok ? "positive" : "negative_or_zero",
           (unsigned long)(id_iq_ratio_centi / 100u),
           (unsigned long)(id_iq_ratio_centi % 100u),
           (unsigned long)(current_mag_milli / 1000u),
           (unsigned long)(current_mag_milli % 1000u),
           (unsigned long)(kcl_micro / 1000000u),
           (unsigned long)(kcl_micro % 1000000u));
  uart2_printf_line(line);

  if (!g_drv_test.current_direction_ok) {
    uart2_printf_line("D_AXIS_CURRENT_POLARITY_FAIL");
  }
  if (!g_drv_test.current_resolution_ok) {
    uart2_printf_line("D_AXIS_CURRENT_SIGNAL_TOO_SMALL");
  }
  if (!g_drv_test.dq_alignment_ok) {
    uart2_printf_line("DQ_ALIGNMENT_UNRELIABLE");
  }
  if (g_drv_test.encoder_motion_max_counts > D_AXIS_MOTION_MAX_COUNTS) {
    uart2_printf_line("D_AXIS_INJECTION_MOTION_FAIL");
  }

  const uint32_t achieved_vd_milli = float_to_scaled_u32(g_drv_test.achieved_vd, 1000.0f);
  const int32_t first_reliable_milli =
      float_to_scaled_i32(g_drv_test.first_reliable_current_voltage, 1000.0f);
  const int32_t r_milliohm =
      float_to_scaled_i32(g_drv_test.phase_resistance_est_ohm, 1000.0f);
  const int32_t inv_offset_milli =
      float_to_scaled_i32(g_drv_test.inverter_voltage_offset_est_v, 1000.0f);
  const int32_t v02_milli =
      float_to_scaled_i32(g_drv_test.voltage_required_for_0p2A_est, 1000.0f);
  const int32_t v03_milli =
      float_to_scaled_i32(g_drv_test.voltage_required_for_0p3A_est, 1000.0f);
  const uint32_t r2_milli = float_to_scaled_u32(g_drv_test.fit_r_squared, 1000.0f);
  const uint32_t baseline_rms_milli =
      (g_drv_test.sweep_point_count > 0u)
          ? float_to_scaled_u32(g_drv_test.sweep_points[0].phase_current_rms, 1000.0f)
          : 0u;
  const uint32_t baseline_p99_milli =
      (g_drv_test.sweep_point_count > 0u)
          ? float_to_scaled_u32(g_drv_test.sweep_points[0].phase_current_p99, 1000.0f)
          : 0u;
  snprintf(line,
           sizeof(line),
           "static_d_axis_voltage_sweep_summary: sweep_points=%lu valid_fit_point_count=%lu achieved_vd=%lu.%03lu baseline_phase_current_rms=%lu.%03lu baseline_phase_current_p99=%lu.%03lu first_reliable_current_voltage=%s%lu.%03lu encoder_motion_max_counts=%lu current_monotonic_ok=%u phase_resistance_est_reliable=%u phase_resistance_est_ohm=%s%lu.%03lu inverter_voltage_offset_est_v=%s%lu.%03lu fit_r_squared=%lu.%03lu voltage_required_for_0p2A_est=%s%lu.%03lu voltage_required_for_0p3A_est=%s%lu.%03lu recommended_CCR4=%ld",
           (unsigned long)g_drv_test.sweep_point_count,
           (unsigned long)g_drv_test.valid_fit_point_count,
           (unsigned long)(achieved_vd_milli / 1000u),
           (unsigned long)(achieved_vd_milli % 1000u),
           (unsigned long)(baseline_rms_milli / 1000u),
           (unsigned long)(baseline_rms_milli % 1000u),
           (unsigned long)(baseline_p99_milli / 1000u),
           (unsigned long)(baseline_p99_milli % 1000u),
           (first_reliable_milli < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(first_reliable_milli) / 1000u),
           (unsigned long)(abs_i32_to_u32(first_reliable_milli) % 1000u),
           (unsigned long)g_drv_test.encoder_motion_max_counts,
           (unsigned int)g_drv_test.current_monotonic_ok,
           (unsigned int)g_drv_test.phase_resistance_est_reliable,
           (r_milliohm < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(r_milliohm) / 1000u),
           (unsigned long)(abs_i32_to_u32(r_milliohm) % 1000u),
           (inv_offset_milli < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(inv_offset_milli) / 1000u),
           (unsigned long)(abs_i32_to_u32(inv_offset_milli) % 1000u),
           (unsigned long)(r2_milli / 1000u),
           (unsigned long)(r2_milli % 1000u),
           (v02_milli < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(v02_milli) / 1000u),
           (unsigned long)(abs_i32_to_u32(v02_milli) % 1000u),
           (v03_milli < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(v03_milli) / 1000u),
           (unsigned long)(abs_i32_to_u32(v03_milli) % 1000u),
           (long)g_drv_test.recommended_ccr4);
  uart2_printf_line(line);

  uart2_printf_line("invalid_reason_map: ID_NEGATIVE=0x00000001 ID_COUNTS_TOO_SMALL=0x00000002 IQ_ID_RATIO_TOO_LARGE=0x00000004 ENCODER_MOVED=0x00000008 ADC_INVALID=0x00000010 SAFETY_FAULT=0x00000020");

  for (uint32_t i = 0u; i < g_drv_test.sweep_point_count; ++i) {
    const SweepPointResult *pt = &g_drv_test.sweep_points[i];
    const uint32_t vd_milli = float_to_scaled_u32(pt->vd_command, 1000.0f);
    const int32_t vq_milli = float_to_scaled_i32(pt->vq_command, 1000.0f);
    const int32_t va_milli = float_to_scaled_i32(pt->v_alpha, 1000.0f);
    const int32_t vb_milli = float_to_scaled_i32(pt->v_beta, 1000.0f);
    const int32_t id_mean_milli = float_to_scaled_i32(pt->id_mean, 1000.0f);
    const int32_t iq_mean_milli = float_to_scaled_i32(pt->iq_mean, 1000.0f);
    const uint32_t id_std_milli = float_to_scaled_u32(pt->id_std, 1000.0f);
    const uint32_t iq_std_milli = float_to_scaled_u32(pt->iq_std, 1000.0f);
    const int32_t iu_mean_milli = float_to_scaled_i32(pt->iu_mean, 1000.0f);
    const int32_t iv_mean_milli = float_to_scaled_i32(pt->iv_mean, 1000.0f);
    const int32_t iw_mean_milli = float_to_scaled_i32(pt->iw_mean, 1000.0f);
    const int32_t id_counts_centi = float_to_scaled_i32(pt->id_effective_counts, 100.0f);
    const int32_t iq_counts_centi = float_to_scaled_i32(pt->iq_effective_counts, 100.0f);
    const uint32_t ratio_centi = float_to_scaled_u32(pt->iq_over_id_ratio, 100.0f);
    const uint32_t phase_abs_mean_milli =
        float_to_scaled_u32(pt->phase_current_abs_mean, 1000.0f);
    const uint32_t phase_rms_milli =
        float_to_scaled_u32(pt->phase_current_rms, 1000.0f);
    const uint32_t phase_p95_milli =
        float_to_scaled_u32(pt->phase_current_p95, 1000.0f);
    const uint32_t phase_p99_milli =
        float_to_scaled_u32(pt->phase_current_p99, 1000.0f);
    const uint32_t phase_p999_milli =
        float_to_scaled_u32(pt->phase_current_p999, 1000.0f);
    const uint32_t phase_current_milli = float_to_scaled_u32(pt->phase_current_max, 1000.0f);
    const uint32_t vbus_centi = float_to_scaled_u32(pt->vbus_mean, 100.0f);
    snprintf(line,
             sizeof(line),
             "sweep_point%02lu: point_index=%lu vd_command=%lu.%03lu vq_command=%s%lu.%03lu v_alpha=%s%lu.%03lu v_beta=%s%lu.%03lu valid=%u invalid_reason=0x%08lX faulted=%u samples=%lu id_mean=%s%lu.%03lu id_std=%lu.%03lu iq_mean=%s%lu.%03lu iq_std=%lu.%03lu iu_mean=%s%lu.%03lu iv_mean=%s%lu.%03lu iw_mean=%s%lu.%03lu id_effective_counts=%s%lu.%02lu iq_effective_counts=%s%lu.%02lu abs_iq_over_abs_id=%lu.%02lu phase_current_abs_mean=%lu.%03lu phase_current_rms=%lu.%03lu phase_current_p95=%lu.%03lu phase_current_p99=%lu.%03lu phase_current_p999=%lu.%03lu phase_current_max=%lu.%03lu current_over_consecutive_max=%lu samples_above_0p45A=%lu longest_consecutive_samples_above_0p45A=%lu encoder_motion_counts=%lu raw_u_min=%u raw_u_max=%u raw_v_min=%u raw_v_max=%u vbus_avg=%lu.%02lu",
             (unsigned long)i,
             (unsigned long)i,
             (unsigned long)(vd_milli / 1000u),
             (unsigned long)(vd_milli % 1000u),
             (vq_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(vq_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(vq_milli) % 1000u),
             (va_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(va_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(va_milli) % 1000u),
             (vb_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(vb_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(vb_milli) % 1000u),
             (unsigned int)pt->valid,
             (unsigned long)pt->invalid_reason,
             (unsigned int)pt->faulted,
             (unsigned long)pt->stats.samples,
             (id_mean_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(id_mean_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(id_mean_milli) % 1000u),
             (unsigned long)(id_std_milli / 1000u),
             (unsigned long)(id_std_milli % 1000u),
             (iq_mean_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(iq_mean_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(iq_mean_milli) % 1000u),
             (unsigned long)(iq_std_milli / 1000u),
             (unsigned long)(iq_std_milli % 1000u),
             (iu_mean_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(iu_mean_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(iu_mean_milli) % 1000u),
             (iv_mean_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(iv_mean_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(iv_mean_milli) % 1000u),
             (iw_mean_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(iw_mean_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(iw_mean_milli) % 1000u),
             (id_counts_centi < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(id_counts_centi) / 100u),
             (unsigned long)(abs_i32_to_u32(id_counts_centi) % 100u),
             (iq_counts_centi < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(iq_counts_centi) / 100u),
             (unsigned long)(abs_i32_to_u32(iq_counts_centi) % 100u),
             (unsigned long)(ratio_centi / 100u),
             (unsigned long)(ratio_centi % 100u),
             (unsigned long)(phase_abs_mean_milli / 1000u),
             (unsigned long)(phase_abs_mean_milli % 1000u),
             (unsigned long)(phase_rms_milli / 1000u),
             (unsigned long)(phase_rms_milli % 1000u),
             (unsigned long)(phase_p95_milli / 1000u),
             (unsigned long)(phase_p95_milli % 1000u),
             (unsigned long)(phase_p99_milli / 1000u),
             (unsigned long)(phase_p99_milli % 1000u),
             (unsigned long)(phase_p999_milli / 1000u),
             (unsigned long)(phase_p999_milli % 1000u),
             (unsigned long)(phase_current_milli / 1000u),
             (unsigned long)(phase_current_milli % 1000u),
             (unsigned long)pt->current_over_consecutive_max,
             (unsigned long)pt->samples_above_0p45a,
             (unsigned long)pt->longest_consecutive_samples_above_0p45a,
             (unsigned long)pt->encoder_motion_counts,
             (unsigned int)pt->raw_u_min,
             (unsigned int)pt->raw_u_max,
             (unsigned int)pt->raw_v_min,
             (unsigned int)pt->raw_v_max,
             (unsigned long)(vbus_centi / 100u),
             (unsigned long)(vbus_centi % 100u));
    uart2_printf_line(line);
  }
  if (!g_drv_test.phase_resistance_est_reliable) {
    uart2_printf_line("PHASE_RESISTANCE_EST_UNRELIABLE");
  }

  for (uint32_t i = 0u; i < g_drv_test.voltage_diag_count; ++i) {
    const VoltagePathDiag *vdg = &g_drv_test.voltage_diags[i];
    const int32_t vd_m = float_to_scaled_i32(vdg->vd, 1000.0f);
    const int32_t vq_m = float_to_scaled_i32(vdg->vq, 1000.0f);
    const uint32_t vbus_c = float_to_scaled_u32(vdg->measured_vbus, 100.0f);
    const uint32_t theta_m = float_to_scaled_u32(vdg->theta_e, 1000.0f);
    const int32_t va_m = float_to_scaled_i32(vdg->v_alpha, 1000.0f);
    const int32_t vb_m = float_to_scaled_i32(vdg->v_beta, 1000.0f);
    const int32_t na_m = float_to_scaled_i32(vdg->normalized_alpha, 1000000.0f);
    const int32_t nb_m = float_to_scaled_i32(vdg->normalized_beta, 1000000.0f);
    const int32_t pa_m = float_to_scaled_i32(vdg->phase_a, 1000.0f);
    const int32_t pb_m = float_to_scaled_i32(vdg->phase_b, 1000.0f);
    const int32_t pc_m = float_to_scaled_i32(vdg->phase_c, 1000.0f);
    const int32_t cm_m = float_to_scaled_i32(vdg->common_mode, 1000.0f);
    const uint32_t da_m = float_to_scaled_u32(vdg->duty_a, 1000000.0f);
    const uint32_t db_m = float_to_scaled_u32(vdg->duty_b, 1000000.0f);
    const uint32_t dc_m = float_to_scaled_u32(vdg->duty_c, 1000000.0f);
    const int32_t ava_m = float_to_scaled_i32(vdg->applied_v_alpha, 1000.0f);
    const int32_t avb_m = float_to_scaled_i32(vdg->applied_v_beta, 1000.0f);
    const uint32_t avmag_m = float_to_scaled_u32(vdg->applied_voltage_magnitude, 1000.0f);
    const uint32_t ratio_m = float_to_scaled_u32(vdg->commanded_to_applied_ratio, 1000.0f);
    snprintf(line,
             sizeof(line),
             "voltage_path_diag%02lu: measured_vbus=%lu.%02lu vd=%s%lu.%03lu vq=%s%lu.%03lu theta_e=%lu.%03lu v_alpha=%s%lu.%03lu v_beta=%s%lu.%03lu input_svpwm_v_alpha=%s%lu.%03lu input_svpwm_v_beta=%s%lu.%03lu divided_by_vbus=%u vbus_divide_count=%lu normalized_alpha=%s%ld.%06lu normalized_beta=%s%ld.%06lu phase_a=%s%lu.%03lu phase_b=%s%lu.%03lu phase_c=%s%lu.%03lu common_mode=%s%lu.%03lu duty_a=0.%06lu duty_b=0.%06lu duty_c=0.%06lu ccr1=%lu ccr2=%lu ccr3=%lu ccr1_offset=%ld ccr2_offset=%ld ccr3_offset=%ld ccr_span=%lu applied_v_alpha_est=%s%lu.%03lu applied_v_beta_est=%s%lu.%03lu applied_voltage_magnitude_est=%lu.%03lu commanded_to_applied_ratio=%lu.%03lu",
             (unsigned long)i,
             (unsigned long)(vbus_c / 100u), (unsigned long)(vbus_c % 100u),
             (vd_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(vd_m) / 1000u), (unsigned long)(abs_i32_to_u32(vd_m) % 1000u),
             (vq_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(vq_m) / 1000u), (unsigned long)(abs_i32_to_u32(vq_m) % 1000u),
             (unsigned long)(theta_m / 1000u), (unsigned long)(theta_m % 1000u),
             (va_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(va_m) / 1000u), (unsigned long)(abs_i32_to_u32(va_m) % 1000u),
             (vb_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(vb_m) / 1000u), (unsigned long)(abs_i32_to_u32(vb_m) % 1000u),
             (va_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(va_m) / 1000u), (unsigned long)(abs_i32_to_u32(va_m) % 1000u),
             (vb_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(vb_m) / 1000u), (unsigned long)(abs_i32_to_u32(vb_m) % 1000u),
             (unsigned int)(vdg->vbus_divide_count > 0u),
             (unsigned long)vdg->vbus_divide_count,
             (na_m < 0) ? "-" : "", (long)(abs_i32_to_u32(na_m) / 1000000u), (unsigned long)(abs_i32_to_u32(na_m) % 1000000u),
             (nb_m < 0) ? "-" : "", (long)(abs_i32_to_u32(nb_m) / 1000000u), (unsigned long)(abs_i32_to_u32(nb_m) % 1000000u),
             (pa_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(pa_m) / 1000u), (unsigned long)(abs_i32_to_u32(pa_m) % 1000u),
             (pb_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(pb_m) / 1000u), (unsigned long)(abs_i32_to_u32(pb_m) % 1000u),
             (pc_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(pc_m) / 1000u), (unsigned long)(abs_i32_to_u32(pc_m) % 1000u),
             (cm_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(cm_m) / 1000u), (unsigned long)(abs_i32_to_u32(cm_m) % 1000u),
             (unsigned long)da_m,
             (unsigned long)db_m,
             (unsigned long)dc_m,
             (unsigned long)vdg->ccr1,
             (unsigned long)vdg->ccr2,
             (unsigned long)vdg->ccr3,
             (long)vdg->ccr1_offset,
             (long)vdg->ccr2_offset,
             (long)vdg->ccr3_offset,
             (unsigned long)vdg->ccr_span,
             (ava_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(ava_m) / 1000u), (unsigned long)(abs_i32_to_u32(ava_m) % 1000u),
             (avb_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(avb_m) / 1000u), (unsigned long)(abs_i32_to_u32(avb_m) % 1000u),
             (unsigned long)(avmag_m / 1000u), (unsigned long)(avmag_m % 1000u),
             (unsigned long)(ratio_m / 1000u), (unsigned long)(ratio_m % 1000u));
    uart2_printf_line(line);
  }
  if (g_drv_test.voltage_command_double_scaled) {
    uart2_printf_line("VOLTAGE_COMMAND_DOUBLE_SCALED");
  }
  if (g_drv_test.pwm_voltage_command_too_small) {
    uart2_printf_line("PWM_VOLTAGE_COMMAND_TOO_SMALL");
  }
  if (g_drv_test.applied_voltage_scale_fail) {
    uart2_printf_line("APPLIED_VOLTAGE_SCALE_FAIL");
  }

  for (uint32_t i = 0u; i < g_drv_test.direct_alpha_count; ++i) {
    const DirectAlphaResult *da = &g_drv_test.direct_alpha[i];
    const uint32_t va_m = float_to_scaled_u32(da->v_alpha_command, 1000.0f);
    const int32_t ia_m = float_to_scaled_i32(da->i_alpha_mean, 1000.0f);
    const int32_t ib_m = float_to_scaled_i32(da->i_beta_mean, 1000.0f);
    const uint32_t ias_m = float_to_scaled_u32(da->i_alpha_std, 1000.0f);
    const uint32_t ibs_m = float_to_scaled_u32(da->i_beta_std, 1000.0f);
    const int32_t iu_m = float_to_scaled_i32(da->iu_mean, 1000.0f);
    const int32_t iv_m = float_to_scaled_i32(da->iv_mean, 1000.0f);
    const int32_t iw_m = float_to_scaled_i32(da->iw_mean, 1000.0f);
    const uint32_t ru_c = float_to_scaled_u32(da->raw_u_mean, 100.0f);
    const uint32_t rv_c = float_to_scaled_u32(da->raw_v_mean, 100.0f);
    const int32_t counts_c = float_to_scaled_i32(da->effective_counts, 100.0f);
    const uint32_t app_m = float_to_scaled_u32(da->applied_voltage_magnitude_est, 1000.0f);
    snprintf(line,
             sizeof(line),
             "direct_alpha%02lu: v_alpha=%lu.%03lu v_beta=0.000 valid=%u faulted=%u samples=%lu i_alpha_mean=%s%lu.%03lu i_alpha_std=%lu.%03lu i_beta_mean=%s%lu.%03lu i_beta_std=%lu.%03lu iu_mean=%s%lu.%03lu iv_mean=%s%lu.%03lu iw_mean=%s%lu.%03lu raw_u_mean=%lu.%02lu raw_v_mean=%lu.%02lu effective_counts=%s%lu.%02lu ccr1=%lu ccr2=%lu ccr3=%lu applied_voltage_magnitude_est=%lu.%03lu encoder_motion_counts=%lu",
             (unsigned long)i,
             (unsigned long)(va_m / 1000u), (unsigned long)(va_m % 1000u),
             (unsigned int)da->valid,
             (unsigned int)da->faulted,
             (unsigned long)da->samples,
             (ia_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(ia_m) / 1000u), (unsigned long)(abs_i32_to_u32(ia_m) % 1000u),
             (unsigned long)(ias_m / 1000u), (unsigned long)(ias_m % 1000u),
             (ib_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(ib_m) / 1000u), (unsigned long)(abs_i32_to_u32(ib_m) % 1000u),
             (unsigned long)(ibs_m / 1000u), (unsigned long)(ibs_m % 1000u),
             (iu_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(iu_m) / 1000u), (unsigned long)(abs_i32_to_u32(iu_m) % 1000u),
             (iv_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(iv_m) / 1000u), (unsigned long)(abs_i32_to_u32(iv_m) % 1000u),
             (iw_m < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(iw_m) / 1000u), (unsigned long)(abs_i32_to_u32(iw_m) % 1000u),
             (unsigned long)(ru_c / 100u), (unsigned long)(ru_c % 100u),
             (unsigned long)(rv_c / 100u), (unsigned long)(rv_c % 100u),
             (counts_c < 0) ? "-" : "", (unsigned long)(abs_i32_to_u32(counts_c) / 100u), (unsigned long)(abs_i32_to_u32(counts_c) % 100u),
             (unsigned long)da->ccr1,
             (unsigned long)da->ccr2,
             (unsigned long)da->ccr3,
             (unsigned long)(app_m / 1000u), (unsigned long)(app_m % 1000u),
             (unsigned long)da->encoder_motion_counts);
    uart2_printf_line(line);
  }
  snprintf(line,
           sizeof(line),
           "static_voltage_path_diagnostic: classification=%s direct_alpha_reliable=%u voltage_command_double_scaled=%u pwm_voltage_command_too_small=%u applied_voltage_scale_fail=%u low_side_sample_window_valid=%u",
           (g_drv_test.voltage_path_classification != NULL)
               ? g_drv_test.voltage_path_classification
               : "UNKNOWN",
           (unsigned int)g_drv_test.direct_alpha_reliable,
           (unsigned int)g_drv_test.voltage_command_double_scaled,
           (unsigned int)g_drv_test.pwm_voltage_command_too_small,
           (unsigned int)g_drv_test.applied_voltage_scale_fail,
           (unsigned int)g_drv_test.low_side_sample_window_valid);
  uart2_printf_line(line);
  if (g_drv_test.voltage_path_classification != NULL) {
    uart2_printf_line(g_drv_test.voltage_path_classification);
  }

  for (uint32_t i = 0u; i < g_drv_test.monitor_count; ++i) {
    const OpenLoopMonitorSample *mon = &g_drv_test.monitor[i];
    const uint32_t theta_m_milli = float_to_scaled_u32(mon->theta_m, 1000.0f);
    const uint32_t theta_e_milli = float_to_scaled_u32(mon->theta_e, 1000.0f);
    const uint32_t offset_milli = float_to_scaled_u32(mon->electrical_offset_runtime, 1000.0f);
    const int32_t vd_milli = float_to_scaled_i32(mon->vd, 1000.0f);
    const int32_t vq_milli = float_to_scaled_i32(mon->vq, 1000.0f);
    const int32_t rpm_centi = float_to_scaled_i32(mon->mechanical_rpm_filtered, 100.0f);
    const uint32_t vbus_centi = float_to_scaled_u32(mon->vbus_v, 100.0f);
    char encoder_accum_s[24];
    i64_to_dec(encoder_accum_s, sizeof(encoder_accum_s), mon->encoder_accum);
    snprintf(line,
             sizeof(line),
             "encoder_comm_mon%02lu: elapsed_ms=%lu state=%s encoder_accum=%s theta_m=%lu.%03lu theta_e_encoder=%lu.%03lu electrical_offset_runtime=%lu.%03lu vd=%s%lu.%03lu vq=%s%lu.%03lu mechanical_rpm_filtered=%s%lu.%02lu ccr1=%lu ccr2=%lu ccr3=%lu raw_u=%u raw_v=%u vbus_v=%lu.%02lu drv0_status1=0x%04X drv1_status1=0x%04X ccer=0x%08lX bdtr=0x%08lX moe=%lu en_gate=%lu nfault=%lu m1_safe=%u ok=%u",
             (unsigned long)(i + 1u),
             (unsigned long)mon->elapsed_ms,
             test_state_name(mon->state),
             encoder_accum_s,
             (unsigned long)(theta_m_milli / 1000u),
             (unsigned long)(theta_m_milli % 1000u),
             (unsigned long)(theta_e_milli / 1000u),
             (unsigned long)(theta_e_milli % 1000u),
             (unsigned long)(offset_milli / 1000u),
             (unsigned long)(offset_milli % 1000u),
             (vd_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(vd_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(vd_milli) % 1000u),
             (vq_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(vq_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(vq_milli) % 1000u),
             (rpm_centi < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(rpm_centi) / 100u),
             (unsigned long)(abs_i32_to_u32(rpm_centi) % 100u),
             (unsigned long)mon->ccr1,
             (unsigned long)mon->ccr2,
             (unsigned long)mon->ccr3,
             (unsigned int)mon->raw_u,
             (unsigned int)mon->raw_v,
             (unsigned long)(vbus_centi / 100u),
             (unsigned long)(vbus_centi % 100u),
             (unsigned int)mon->drv0_status1,
             (unsigned int)mon->drv1_status1,
             (unsigned long)mon->ccer,
             (unsigned long)mon->bdtr,
             (unsigned long)mon->moe,
             (unsigned long)mon->en_gate_raw,
             (unsigned long)mon->nfault,
             (unsigned int)m1_is_safe_off(),
             (unsigned int)mon->ok);
    uart2_printf_line(line);
  }

  const uint32_t align_theta_m_milli = float_to_scaled_u32(g_drv_test.theta_m_align, 1000.0f);
  const uint32_t offset_rad_milli = float_to_scaled_u32(g_drv_test.electrical_offset_runtime_rad, 1000.0f);
  const uint32_t offset_deg_centi = float_to_scaled_u32(g_drv_test.electrical_offset_runtime_deg, 100.0f);
  char encoder_align_count_s[24];
  char encoder_start_count_s[24];
  char encoder_end_count_s[24];
  char encoder_total_delta_s[24];
  char encoder_delta_10ms_s[24];
  i64_to_dec(encoder_align_count_s, sizeof(encoder_align_count_s), g_drv_test.encoder_align_count);
  i64_to_dec(encoder_start_count_s, sizeof(encoder_start_count_s), g_drv_test.encoder_start_count);
  i64_to_dec(encoder_end_count_s, sizeof(encoder_end_count_s), g_drv_test.encoder_end_count);
  i64_to_dec(encoder_total_delta_s, sizeof(encoder_total_delta_s), g_drv_test.encoder_total_delta);
  i64_to_dec(encoder_delta_10ms_s, sizeof(encoder_delta_10ms_s), g_drv_test.encoder_delta_10ms);
  snprintf(line,
           sizeof(line),
           "commutation_summary: encoder_align_count=%s theta_m_align=%lu.%03lu electrical_offset_runtime_rad=%lu.%03lu electrical_offset_runtime_deg=%lu.%02lu encoder_start_count=%s encoder_end_count=%s encoder_total_delta=%s encoder_delta_10ms=%s mechanical_rpm_raw=%s%lu.%02lu mechanical_rpm_filtered=%s%lu.%02lu maximum_rpm=%s%lu.%02lu minimum_rpm=%s%lu.%02lu raw_current_max_deviation=%lu fault_code=0x%08lX",
           encoder_align_count_s,
           (unsigned long)(align_theta_m_milli / 1000u),
           (unsigned long)(align_theta_m_milli % 1000u),
           (unsigned long)(offset_rad_milli / 1000u),
           (unsigned long)(offset_rad_milli % 1000u),
           (unsigned long)(offset_deg_centi / 100u),
           (unsigned long)(offset_deg_centi % 100u),
           encoder_start_count_s,
           encoder_end_count_s,
           encoder_total_delta_s,
           encoder_delta_10ms_s,
           (float_to_scaled_i32(g_drv_test.mechanical_rpm_raw, 100.0f) < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(float_to_scaled_i32(g_drv_test.mechanical_rpm_raw, 100.0f)) / 100u),
           (unsigned long)(abs_i32_to_u32(float_to_scaled_i32(g_drv_test.mechanical_rpm_raw, 100.0f)) % 100u),
           (float_to_scaled_i32(g_drv_test.mechanical_rpm_filtered, 100.0f) < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(float_to_scaled_i32(g_drv_test.mechanical_rpm_filtered, 100.0f)) / 100u),
           (unsigned long)(abs_i32_to_u32(float_to_scaled_i32(g_drv_test.mechanical_rpm_filtered, 100.0f)) % 100u),
           (float_to_scaled_i32(g_drv_test.maximum_rpm, 100.0f) < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(float_to_scaled_i32(g_drv_test.maximum_rpm, 100.0f)) / 100u),
           (unsigned long)(abs_i32_to_u32(float_to_scaled_i32(g_drv_test.maximum_rpm, 100.0f)) % 100u),
           (float_to_scaled_i32(g_drv_test.minimum_rpm, 100.0f) < 0) ? "-" : "",
           (unsigned long)(abs_i32_to_u32(float_to_scaled_i32(g_drv_test.minimum_rpm, 100.0f)) / 100u),
           (unsigned long)(abs_i32_to_u32(float_to_scaled_i32(g_drv_test.minimum_rpm, 100.0f)) % 100u),
           (unsigned long)g_drv_test.raw_current_max_deviation,
           (unsigned long)g_drv_test.fault_code);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "fault_decode: fault_code=0x%08lX fault_enum_name=%s fault_source=%s",
           (unsigned long)g_drv_test.fault_code,
           get_fault_string((Axis0FaultFlags)g_drv_test.fault_code),
           g_drv_test.current_trip_fault.latched
               ? "current_trip_diag_record_and_check"
               : "drv_bringup_test_run");
  uart2_printf_line(line);
  uart2_printf_line("current_protection_config: continuous_limit=0.600A reset_below=0.550A continuous_samples=3 instant_limit=1.000A raw_currents_unfiltered=1");

  const CurrentTripDiagnostics *diag = &g_drv_test.current_trip_diag;
  const CurrentTripFaultSnapshot *fault = &g_drv_test.current_trip_fault;
  snprintf(line,
           sizeof(line),
           "current_trip_diag_summary: classification=%s latched=%u completed_zero_baseline=%u iu_over_count=%lu iv_over_count=%lu iw_over_count=%lu id_over_count=%lu iq_over_count=%lu iu_over_consecutive=%lu iv_over_consecutive=%lu iw_over_consecutive=%lu id_over_consecutive=%lu iq_over_consecutive=%lu current_over_consecutive_max=%lu align_current_abs_max=%lu.%03lu zero_baseline_current_abs_max=%lu.%03lu moe_first_1ms_current_abs_max=%lu.%03lu switch_first_1ms_current_abs_max=%lu.%03lu",
           (diag->classification != NULL) ? diag->classification : "UNKNOWN",
           (unsigned int)fault->latched,
           (unsigned int)diag->completed_zero_baseline,
           (unsigned long)diag->iu_over_count,
           (unsigned long)diag->iv_over_count,
           (unsigned long)diag->iw_over_count,
           (unsigned long)diag->id_over_count,
           (unsigned long)diag->iq_over_count,
           (unsigned long)diag->iu_over_consecutive,
           (unsigned long)diag->iv_over_consecutive,
           (unsigned long)diag->iw_over_consecutive,
           (unsigned long)diag->id_over_consecutive,
           (unsigned long)diag->iq_over_consecutive,
           (unsigned long)diag->current_over_consecutive_max,
           (unsigned long)(float_to_scaled_u32(diag->align_current_abs_max, 1000.0f) / 1000u),
           (unsigned long)(float_to_scaled_u32(diag->align_current_abs_max, 1000.0f) % 1000u),
           (unsigned long)(float_to_scaled_u32(diag->zero_baseline_current_abs_max, 1000.0f) / 1000u),
           (unsigned long)(float_to_scaled_u32(diag->zero_baseline_current_abs_max, 1000.0f) % 1000u),
           (unsigned long)(float_to_scaled_u32(diag->moe_first_1ms_current_abs_max, 1000.0f) / 1000u),
           (unsigned long)(float_to_scaled_u32(diag->moe_first_1ms_current_abs_max, 1000.0f) % 1000u),
           (unsigned long)(float_to_scaled_u32(diag->switch_first_1ms_current_abs_max, 1000.0f) / 1000u),
           (unsigned long)(float_to_scaled_u32(diag->switch_first_1ms_current_abs_max, 1000.0f) % 1000u));
  uart2_printf_line(line);

  if (fault->latched) {
    char encoder_fault_s[24];
    i64_to_dec(encoder_fault_s, sizeof(encoder_fault_s), fault->encoder_accum);
    const int32_t iu_milli = float_to_scaled_i32(fault->iu, 1000.0f);
    const int32_t iv_milli = float_to_scaled_i32(fault->iv, 1000.0f);
    const int32_t iw_milli = float_to_scaled_i32(fault->iw, 1000.0f);
    const int32_t id_milli = float_to_scaled_i32(fault->id, 1000.0f);
    const int32_t iq_milli = float_to_scaled_i32(fault->iq, 1000.0f);
    const uint32_t theta_milli = float_to_scaled_u32(fault->theta_e, 1000.0f);
    const int32_t vd_milli = float_to_scaled_i32(fault->vd, 1000.0f);
    const int32_t vq_milli = float_to_scaled_i32(fault->vq, 1000.0f);
    const uint32_t vbus_centi = float_to_scaled_u32(fault->vbus_v, 100.0f);
    const int32_t trip_current_milli = float_to_scaled_i32(fault->current_value, 1000.0f);
    snprintf(line,
             sizeof(line),
             "current_trip_fault_snapshot: protection_type=%s first_trip_channel=%s current_value=%s%lu.%03lu consecutive_count=%lu fault_timestamp_us=%lu fault_state=%s fault_substate=%lu raw_u=%u raw_v=%u offset_u=%lu offset_v=%lu delta_u_counts=%ld delta_v_counts=%ld iu=%s%lu.%03lu iv=%s%lu.%03lu iw=%s%lu.%03lu id=%s%lu.%03lu iq=%s%lu.%03lu theta_e=%lu.%03lu vd=%s%lu.%03lu vq=%s%lu.%03lu CCR1=%lu CCR2=%lu CCR3=%lu CCR4=%lu TIM1_CNT=%lu TIM1_dir=%lu MOE=%lu EN_GATE=%lu encoder_accum=%s vbus=%lu.%02lu adc_seq=%lu trip_iu=%u trip_iv=%u trip_iw=%u trip_id=%u trip_iq=%u fault_code=0x%08lX",
             (fault->protection_type != NULL) ? fault->protection_type : "NONE",
             (fault->first_trip_channel != NULL) ? fault->first_trip_channel : "NONE",
             (trip_current_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(trip_current_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(trip_current_milli) % 1000u),
             (unsigned long)fault->consecutive_count,
             (unsigned long)fault->fault_timestamp_us,
             test_state_name(fault->fault_state),
             (unsigned long)fault->fault_substate,
             (unsigned int)fault->raw_u,
             (unsigned int)fault->raw_v,
             (unsigned long)fault->offset_u,
             (unsigned long)fault->offset_v,
             (long)fault->delta_u_counts,
             (long)fault->delta_v_counts,
             (iu_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(iu_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(iu_milli) % 1000u),
             (iv_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(iv_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(iv_milli) % 1000u),
             (iw_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(iw_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(iw_milli) % 1000u),
             (id_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(id_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(id_milli) % 1000u),
             (iq_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(iq_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(iq_milli) % 1000u),
             (unsigned long)(theta_milli / 1000u),
             (unsigned long)(theta_milli % 1000u),
             (vd_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(vd_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(vd_milli) % 1000u),
             (vq_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(vq_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(vq_milli) % 1000u),
             (unsigned long)fault->ccr1,
             (unsigned long)fault->ccr2,
             (unsigned long)fault->ccr3,
             (unsigned long)fault->ccr4,
             (unsigned long)fault->tim1_cnt,
             (unsigned long)fault->tim1_dir,
             (unsigned long)fault->moe,
             (unsigned long)fault->en_gate,
             encoder_fault_s,
             (unsigned long)(vbus_centi / 100u),
             (unsigned long)(vbus_centi % 100u),
             (unsigned long)fault->adc_seq,
             (unsigned int)fault->trip_iu,
             (unsigned int)fault->trip_iv,
             (unsigned int)fault->trip_iw,
             (unsigned int)fault->trip_id,
             (unsigned int)fault->trip_iq,
             (unsigned long)g_drv_test.fault_code);
    uart2_printf_line(line);
  }

  const uint32_t ring_count = diag->ring_count;
  const uint32_t ring_start =
      (ring_count < CURRENT_TRIP_RING_COUNT) ? 0u : diag->ring_write_index;
  for (uint32_t i = 0u; i < ring_count; ++i) {
    const uint32_t idx = (ring_start + i) % CURRENT_TRIP_RING_COUNT;
    const CurrentTripRingSample *rs = &g_drv_test.current_trip_ring[idx];
    const int32_t iu_milli = float_to_scaled_i32(rs->iu, 1000.0f);
    const int32_t iv_milli = float_to_scaled_i32(rs->iv, 1000.0f);
    const int32_t iw_milli = float_to_scaled_i32(rs->iw, 1000.0f);
    const int32_t id_milli = float_to_scaled_i32(rs->id, 1000.0f);
    const int32_t iq_milli = float_to_scaled_i32(rs->iq, 1000.0f);
    const int32_t vd_milli = float_to_scaled_i32(rs->vd, 1000.0f);
    const int32_t vq_milli = float_to_scaled_i32(rs->vq, 1000.0f);
    const uint32_t theta_milli = float_to_scaled_u32(rs->theta_e, 1000.0f);
    snprintf(line,
             sizeof(line),
             "current_trip_pre%02lu: state=%s raw_u=%u raw_v=%u iu=%s%lu.%03lu iv=%s%lu.%03lu iw=%s%lu.%03lu id=%s%lu.%03lu iq=%s%lu.%03lu vd=%s%lu.%03lu vq=%s%lu.%03lu theta_e=%lu.%03lu CCR1=%lu CCR2=%lu CCR3=%lu CCR4=%lu TIM1_CNT=%lu TIM1_dir=%lu",
             (unsigned long)i,
             test_state_name(rs->state),
             (unsigned int)rs->raw_u,
             (unsigned int)rs->raw_v,
             (iu_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(iu_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(iu_milli) % 1000u),
             (iv_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(iv_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(iv_milli) % 1000u),
             (iw_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(iw_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(iw_milli) % 1000u),
             (id_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(id_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(id_milli) % 1000u),
             (iq_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(iq_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(iq_milli) % 1000u),
             (vd_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(vd_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(vd_milli) % 1000u),
             (vq_milli < 0) ? "-" : "",
             (unsigned long)(abs_i32_to_u32(vq_milli) / 1000u),
             (unsigned long)(abs_i32_to_u32(vq_milli) % 1000u),
             (unsigned long)(theta_milli / 1000u),
             (unsigned long)(theta_milli % 1000u),
             (unsigned long)rs->ccr1,
             (unsigned long)rs->ccr2,
             (unsigned long)rs->ccr3,
             (unsigned long)rs->ccr4,
             (unsigned long)rs->tim1_cnt,
             (unsigned long)rs->tim1_dir);
    uart2_printf_line(line);
  }

  snprintf(line,
           sizeof(line),
           "power_final: gate=%u nfault=%u ccer=0x%08lX bdtr=0x%08lX moe=%lu ccr1=%lu ccr2=%lu ccr3=%lu",
           (unsigned int)g_drv_test.final_gate,
           (unsigned int)g_drv_test.final_nfault,
           (unsigned long)g_drv_test.final_ccer,
           (unsigned long)g_drv_test.final_bdtr,
           (unsigned long)((g_drv_test.final_bdtr & TIM_BDTR_MOE) ? 1u : 0u),
           (unsigned long)g_drv_test.final_ccr1,
           (unsigned long)g_drv_test.final_ccr2,
           (unsigned long)g_drv_test.final_ccr3);
  uart2_printf_line(line);

  if (g_drv_test.pass) {
    if (g_drv_test.final_observe_reliable &&
        g_drv_test.adc_phase_edge_timing_ok &&
        g_drv_test.adc_sync_rate_ok &&
        !g_drv_test.current_trip_fault.latched) {
      uart2_printf_line("STATIC_D_AXIS_VOLTAGE_SWEEP_PASS");
    } else {
      uart2_printf_line("STATIC_D_AXIS_VOLTAGE_SWEEP_UNRELIABLE");
    }
  } else {
    snprintf(line,
             sizeof(line),
             "STATIC_D_AXIS_VOLTAGE_SWEEP_FAIL fail_step=%lu fault_code=0x%08lX fault_enum_name=%s protection_type=%s first_trip_channel=%s",
             (unsigned long)g_drv_test.fail_step,
             (unsigned long)g_drv_test.fault_code,
             get_fault_string((Axis0FaultFlags)g_drv_test.fault_code),
             (g_drv_test.current_trip_fault.protection_type != NULL)
                 ? g_drv_test.current_trip_fault.protection_type
                 : "NONE",
             (g_drv_test.current_trip_fault.first_trip_channel != NULL)
                 ? g_drv_test.current_trip_fault.first_trip_channel
                 : "NONE");
    uart2_printf_line(line);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  axis0_context_init_minimal();
  drv_bringup_configure_nfault_pull(GPIO_PULLUP);

  /*
   * 先启动 ADC injected interrupt，再启动 TIM1 trigger-only。
   * hal_adc_init() 只是让 ADC 等待外部触发；
   * board_init_power_safe() 会保持 EN_GATE=0/MOE=0，并启动 TIM1 CC4 触发 ADC。
   */
  g_adc_init_ok = hal_adc_init();
  g_board_init_ok = board_init_power_safe(&g_axis0);

  /*
   * 编码器计数器先启动，方便后面手转检查 TIM3 AB。
   * 这一步不使能功率级，不会驱动电机。
   */
  (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  uart2_printf_line("");
  uart2_printf_line("odrive_v36_cube bringup start");
  uart2_printf_line("STATIC D AXIS VOLTAGE SWEEP TEST: gain40, 0.60A continuous x3 protection, 1.00A instant protection.");
  drv_bringup_test_run();
  print_drv_bringup_test_status();
  print_bringup_status();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_print_ms = HAL_GetTick();

  while (1)
  {
    const uint32_t now_ms = HAL_GetTick();

    if ((now_ms - last_print_ms) >= BRINGUP_PRINT_PERIOD_MS) {
      last_print_ms = now_ms;
      print_bringup_status();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  hal_adc_stm32f405_on_injected_complete((void *)hadc);
  g_adc_callback_count++;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

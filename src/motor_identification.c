#include "motor_identification.h"

#include <math.h>
#include <string.h>

static bool finite_positive(float value)
{
    return isfinite(value) && value > 0.0f;
}

bool motor_ident_beta_quality_finalize(MotorIdentPulseQuality *quality,
                                       float beta_ratio_max)
{
    if (quality == 0) {
        return false;
    }

    float alpha_peak = 0.0f;
    float beta_peak = 0.0f;
    for (uint32_t i = 0u; i < quality->rise_count && i < MOTOR_IDENT_RISE_SAMPLES; ++i) {
        if (fabsf(quality->rise_delta_i[i]) > alpha_peak) {
            alpha_peak = fabsf(quality->rise_delta_i[i]);
        }
        if (fabsf(quality->beta_delta_i[i]) > beta_peak) {
            beta_peak = fabsf(quality->beta_delta_i[i]);
        }
    }

    quality->beta_ratio = (alpha_peak > 0.000001f) ? (beta_peak / alpha_peak) : 999.0f;
    return !quality->hard_fault && quality->beta_ratio <= beta_ratio_max;
}

bool motor_ident_pulse_too_short(const float *rise,
                                 uint32_t rise_count,
                                 float expected_final_current_a)
{
    if (rise == 0 || rise_count == 0u || expected_final_current_a <= 0.0f) {
        return true;
    }

    const float last_valid = rise[rise_count - 1u];
    return last_valid < expected_final_current_a * 0.80f;
}

bool motor_ident_dynamics_too_fast(const float *rise, uint32_t rise_count)
{
    if (rise == 0 || rise_count < 2u) {
        return false;
    }

    float peak = 0.0f;
    for (uint32_t i = 0u; i < rise_count; ++i) {
        if (rise[i] > peak) {
            peak = rise[i];
        }
    }

    return peak > 0.001f && rise[0] >= peak * 0.90f;
}

MotorIdentClassification motor_ident_classify(bool test_completed_safely,
                                               bool level_a_reliable,
                                               bool level_b_reliable,
                                               float level_difference_percent,
                                               float max_difference_percent)
{
    MotorIdentClassification cls;
    memset(&cls, 0, sizeof(cls));
    cls.test_completed_safely = test_completed_safely;

    if (!test_completed_safely) {
        cls.status = MOTOR_IDENT_STATUS_FAIL;
        return cls;
    }

    cls.phase_inductance_identification_reliable =
        level_a_reliable &&
        level_b_reliable &&
        isfinite(level_difference_percent) &&
        level_difference_percent < max_difference_percent;
    cls.current_loop_enable_permitted =
        cls.phase_inductance_identification_reliable;
    cls.status = cls.phase_inductance_identification_reliable
                     ? MOTOR_IDENT_STATUS_PASS
                     : MOTOR_IDENT_STATUS_UNRELIABLE;
    return cls;
}

MotorIdentFitResult motor_ident_fit_rl_rise(const float *rise,
                                            uint32_t count,
                                            float sample_period_s,
                                            float resistance_ohm,
                                            float voltage_step_v)
{
    MotorIdentFitResult result;
    memset(&result, 0, sizeof(result));
    if (rise == 0 || count < 4u ||
        !finite_positive(sample_period_s) ||
        !finite_positive(resistance_ohm) ||
        !finite_positive(voltage_step_v)) {
        return result;
    }

    const float amplitude = voltage_step_v / resistance_ohm;
    if (!finite_positive(amplitude)) {
        return result;
    }

    float tau_sum = 0.0f;
    uint32_t tau_count = 0u;
    for (uint32_t i = 0u; i < count; ++i) {
        const float y = rise[i];
        if (y <= 0.0f || y >= amplitude * 0.98f) {
            continue;
        }
        const float denom = logf(1.0f - y / amplitude);
        if (denom < -0.000001f && isfinite(denom)) {
            tau_sum += -(((float)i + 1.0f) * sample_period_s) / denom;
            tau_count++;
        }
    }
    if (tau_count < 3u) {
        return result;
    }

    const float tau = tau_sum / (float)tau_count;
    if (!finite_positive(tau)) {
        return result;
    }

    float y_mean = 0.0f;
    for (uint32_t i = 0u; i < count; ++i) {
        y_mean += rise[i];
    }
    y_mean /= (float)count;

    float ss_tot = 0.0f;
    float ss_res = 0.0f;
    for (uint32_t i = 0u; i < count; ++i) {
        const float t = ((float)i + 1.0f) * sample_period_s;
        const float y_fit = amplitude * (1.0f - expf(-t / tau));
        const float err = rise[i] - y_fit;
        ss_tot += (rise[i] - y_mean) * (rise[i] - y_mean);
        ss_res += err * err;
    }

    result.tau_s = tau;
    result.inductance_h = tau * resistance_ohm;
    result.r_squared = (ss_tot > 0.000000001f) ? (1.0f - ss_res / ss_tot) : 0.0f;
    result.valid = isfinite(result.inductance_h) &&
                   isfinite(result.r_squared) &&
                   result.inductance_h > 0.0f &&
                   result.r_squared <= 1.0001f;
    return result;
}

#include "motor_identification.h"

#include <math.h>
#include <string.h>

static bool finite_positive(float value)
{
    return isfinite(value) && value > 0.0f;
}

static void sort_float_array(float *values, uint32_t count)
{
    for (uint32_t i = 1u; i < count; ++i) {
        const float key = values[i];
        uint32_t j = i;
        while (j > 0u && values[j - 1u] > key) {
            values[j] = values[j - 1u];
            --j;
        }
        values[j] = key;
    }
}

static float median_of_values(const float *values, uint32_t count)
{
    if (values == 0 || count == 0u || count > MOTOR_IDENT_FALL_SAMPLES) {
        return 0.0f;
    }
    float tmp[MOTOR_IDENT_FALL_SAMPLES];
    for (uint32_t i = 0u; i < count; ++i) {
        tmp[i] = values[i];
    }
    sort_float_array(tmp, count);
    if ((count & 1u) != 0u) {
        return tmp[count / 2u];
    }
    return 0.5f * (tmp[count / 2u - 1u] + tmp[count / 2u]);
}

static float robust_sigma_mad(const float *values, uint32_t count)
{
    if (values == 0 || count < 2u || count > MOTOR_IDENT_FALL_SAMPLES) {
        return 0.0f;
    }
    const float med = median_of_values(values, count);
    float deviations[MOTOR_IDENT_FALL_SAMPLES];
    for (uint32_t i = 0u; i < count; ++i) {
        deviations[i] = fabsf(values[i] - med);
    }
    return 1.4826f * median_of_values(deviations, count);
}

static float stddev_values(const float *values, uint32_t count)
{
    if (values == 0 || count < 2u) {
        return 0.0f;
    }
    float mean = 0.0f;
    for (uint32_t i = 0u; i < count; ++i) {
        mean += values[i];
    }
    mean /= (float)count;
    float var = 0.0f;
    for (uint32_t i = 0u; i < count; ++i) {
        const float d = values[i] - mean;
        var += d * d;
    }
    return sqrtf(var / (float)(count - 1u));
}

static float r_squared_for_fit(const float *y,
                               uint32_t start,
                               uint32_t end,
                               const float *fit,
                               float *rms,
                               float *max_residual)
{
    float mean = 0.0f;
    uint32_t n = 0u;
    for (uint32_t i = start; i <= end; ++i) {
        mean += y[i];
        ++n;
    }
    if (n == 0u) {
        return 0.0f;
    }
    mean /= (float)n;
    float ss_tot = 0.0f;
    float ss_res = 0.0f;
    float max_res = 0.0f;
    for (uint32_t i = start; i <= end; ++i) {
        const float err = y[i] - fit[i - start];
        ss_res += err * err;
        const float centered = y[i] - mean;
        ss_tot += centered * centered;
        if (fabsf(err) > max_res) {
            max_res = fabsf(err);
        }
    }
    if (rms != 0) {
        *rms = sqrtf(ss_res / (float)n);
    }
    if (max_residual != 0) {
        *max_residual = max_res;
    }
    return (ss_tot > 1.0e-12f) ? (1.0f - ss_res / ss_tot) : 0.0f;
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

static void fit_rise_nonlinear(const float *rise,
                               uint32_t count,
                               float sample_period_s,
                               float resistance_ohm,
                               float noise_a,
                               MotorIdentCurveFit *fit)
{
    memset(fit, 0, sizeof(*fit));
    if (rise == 0 || count < 6u || count > MOTOR_IDENT_RISE_SAMPLES ||
        !finite_positive(sample_period_s) || !finite_positive(resistance_ohm)) {
        return;
    }

    float tail[10u];
    uint32_t tail_n = 0u;
    const uint32_t tail_start = (count > 10u) ? (count - 10u) : 0u;
    for (uint32_t i = tail_start; i < count; ++i) {
        tail[tail_n++] = rise[i];
    }
    const float tail_median = median_of_values(tail, tail_n);
    const float offset_limit = fmaxf(fmaxf(3.0f * noise_a, fabsf(tail_median) * 0.05f), 0.0005f);

    float best_sse = 1.0e30f;
    float best_fit[MOTOR_IDENT_RISE_SAMPLES] = {0};
    float best_tau = 0.0f;
    float best_offset = 0.0f;
    float best_amp = 0.0f;

    for (uint32_t delay_step = 0u; delay_step <= 4u; ++delay_step) {
        const float delay = sample_period_s * (float)delay_step * 0.25f;
        for (uint32_t tau_step = 0u; tau_step < 120u; ++tau_step) {
            const float tau = 20.0e-6f * expf(logf(500.0f) * (float)tau_step / 119.0f);
            float s00 = 0.0f, s01 = 0.0f, s11 = 0.0f, sy0 = 0.0f, sy1 = 0.0f;
            for (uint32_t i = 0u; i < count; ++i) {
                const float t = ((float)i + 1.0f) * sample_period_s - delay;
                const float z = (t <= 0.0f) ? 0.0f : (1.0f - expf(-t / tau));
                s00 += 1.0f;
                s01 += z;
                s11 += z * z;
                sy0 += rise[i];
                sy1 += z * rise[i];
            }
            const float det = s00 * s11 - s01 * s01;
            if (fabsf(det) < 1.0e-12f) {
                continue;
            }
            const float offset = (sy0 * s11 - sy1 * s01) / det;
            const float amp = (s00 * sy1 - s01 * sy0) / det;
            if (!(amp > 0.0f) || fabsf(offset) > offset_limit) {
                continue;
            }
            float sse = 0.0f;
            const float clip = fmaxf(4.0f * noise_a, 0.04f);
            float local_fit[MOTOR_IDENT_RISE_SAMPLES];
            for (uint32_t i = 0u; i < count; ++i) {
                const float t = ((float)i + 1.0f) * sample_period_s - delay;
                const float z = (t <= 0.0f) ? 0.0f : (1.0f - expf(-t / tau));
                local_fit[i] = offset + amp * z;
                const float err = rise[i] - local_fit[i];
                const float clipped = fminf(fabsf(err), clip);
                sse += clipped * clipped;
            }
            if (sse < best_sse) {
                best_sse = sse;
                best_tau = tau;
                best_offset = offset;
                best_amp = amp;
                for (uint32_t i = 0u; i < count; ++i) {
                    best_fit[i] = local_fit[i];
                }
            }
        }
    }

    if (!finite_positive(best_tau) || !finite_positive(best_amp)) {
        return;
    }
    fit->start_index = 0u;
    fit->end_index = count - 1u;
    fit->point_count = count;
    fit->fitted_offset_a = best_offset;
    fit->fitted_amplitude_a = best_amp;
    fit->fitted_tau_s = best_tau;
    fit->fitted_inductance_h = resistance_ohm * best_tau;
    fit->r_squared = r_squared_for_fit(rise, 0u, count - 1u, best_fit,
                                       &fit->residual_rms_a,
                                       &fit->residual_max_a);
    fit->valid = isfinite(fit->r_squared) &&
                 fit->fitted_inductance_h > 0.0f &&
                 fit->r_squared > 0.50f &&
                 fit->residual_rms_a <= fmaxf(8.0f * noise_a, 0.12f);
}

static void fit_fall_adaptive(const float *fall,
                              uint32_t count,
                              const float *fall_std,
                              float sample_period_s,
                              float resistance_ohm,
                              float noise_a,
                              float current_amp_per_count,
                              MotorIdentCurveFit *fit)
{
    memset(fit, 0, sizeof(*fit));
    if (fall == 0 || count < 8u || count > MOTOR_IDENT_FALL_SAMPLES ||
        !finite_positive(sample_period_s) || !finite_positive(resistance_ohm)) {
        return;
    }

    float tail[10u];
    uint32_t tail_n = 0u;
    const uint32_t tail_start = (count > 10u) ? (count - 10u) : 0u;
    for (uint32_t i = tail_start; i < count; ++i) {
        tail[tail_n++] = fall[i];
    }
    const float offset_est = median_of_values(tail, tail_n);
    const float initial = fall[0] - offset_est;
    if (!(initial > 0.0f)) {
        return;
    }

    float best_score = 1.0e30f;
    float best_fit[MOTOR_IDENT_FALL_SAMPLES] = {0};
    float best_tau = 0.0f;
    float best_offset = 0.0f;
    float best_amp = 0.0f;
    uint32_t best_start = 0u;
    uint32_t best_end = 0u;
    float best_weighted_rms = 0.0f;
    float best_r2 = 0.0f;
    float best_residual_rms = 0.0f;
    float best_residual_max = 0.0f;

    for (uint32_t start = 0u; start <= 1u && start + 4u < count; ++start) {
        const float start_initial = fall[start] - offset_est;
        if (!(start_initial > 0.0f)) {
            continue;
        }

        uint32_t max_end = count - 1u;
        const float floor_a = 3.0f * noise_a;
        const float ratio_floor_a = 0.15f * fabsf(start_initial);
        uint32_t tail_candidate_run = 0u;
        for (uint32_t i = start + 1u; i < count; ++i) {
            const float y = fabsf(fall[i] - offset_est);
            if ((y < floor_a) || (y < ratio_floor_a)) {
                tail_candidate_run++;
            } else {
                tail_candidate_run = 0u;
            }
            if (tail_candidate_run >= 3u) {
                const uint32_t tail_start_index = i - tail_candidate_run + 1u;
                max_end = (tail_start_index > start + 4u) ? (tail_start_index - 1u) : (start + 4u);
                break;
            }
        }
        if (max_end >= count) {
            max_end = count - 1u;
        }
        if (max_end < start + 4u) {
            max_end = start + 4u;
        }

        for (uint32_t end = start + 4u; end <= max_end; ++end) {
            const uint32_t point_count = end - start + 1u;
            for (int32_t off_step = -8; off_step <= 8; ++off_step) {
                const float offset = offset_est + (float)off_step * noise_a * 0.25f;
                for (uint32_t tau_step = 0u; tau_step < 160u; ++tau_step) {
                    const float tau = 20.0e-6f * expf(logf(500.0f) * (float)tau_step / 159.0f);
                    float sz2 = 0.0f;
                    float szy = 0.0f;
                    float weight_sum = 0.0f;
                    for (uint32_t i = start; i <= end; ++i) {
                        const float t = (float)(i - start) * sample_period_s;
                        const float z = expf(-t / tau);
                        const float std_a = (fall_std != 0) ? fall_std[i] : 0.0f;
                        const float weight = 1.0f / (std_a * std_a + noise_a * noise_a);
                        sz2 += weight * z * z;
                        szy += weight * z * (fall[i] - offset);
                        weight_sum += weight;
                    }
                    if (sz2 <= 1.0e-12f || weight_sum <= 1.0e-12f) {
                        continue;
                    }
                    const float amp = szy / sz2;
                    if (!(amp > 0.0f)) {
                        continue;
                    }

                    float weighted_sse = 0.0f;
                    float local_fit[MOTOR_IDENT_FALL_SAMPLES];
                    for (uint32_t i = start; i <= end; ++i) {
                        const float t = (float)(i - start) * sample_period_s;
                        local_fit[i - start] = offset + amp * expf(-t / tau);
                        const float err = fall[i] - local_fit[i - start];
                        const float std_a = (fall_std != 0) ? fall_std[i] : 0.0f;
                        const float weight = 1.0f / (std_a * std_a + noise_a * noise_a);
                        weighted_sse += weight * err * err;
                    }
                    float residual_rms = 0.0f;
                    float residual_max = 0.0f;
                    const float r2 =
                        r_squared_for_fit(fall, start, end, local_fit,
                                          &residual_rms, &residual_max);
                    const float weighted_rms = sqrtf(weighted_sse / weight_sum);
                    const float normalized_rms =
                        (noise_a > 0.0f) ? (weighted_rms / noise_a) : 999.0f;
                    const float r2_penalty =
                        (r2 < 0.95f) ? ((0.95f - r2) * 4.0f) : 0.0f;
                    const float point_penalty =
                        (point_count < 10u) ? (0.03f * (float)(10u - point_count)) : 0.0f;
                    const float score = normalized_rms + r2_penalty + point_penalty;
                    if (score < best_score) {
                        best_score = score;
                        best_tau = tau;
                        best_offset = offset;
                        best_amp = amp;
                        best_start = start;
                        best_end = end;
                        best_weighted_rms = weighted_rms;
                        best_r2 = r2;
                        best_residual_rms = residual_rms;
                        best_residual_max = residual_max;
                        for (uint32_t i = start; i <= end; ++i) {
                            best_fit[i - start] = local_fit[i - start];
                        }
                    }
                }
            }
        }
    }

    if (!finite_positive(best_tau) || !finite_positive(best_amp)) {
        return;
    }
    fit->start_index = best_start;
    fit->end_index = best_end;
    fit->point_count = best_end - best_start + 1u;
    fit->fitted_offset_a = best_offset;
    fit->fitted_amplitude_a = best_amp;
    fit->fitted_tau_s = best_tau;
    fit->fitted_inductance_h = resistance_ohm * best_tau;
    fit->r_squared = best_r2;
    fit->residual_rms_a = best_residual_rms;
    fit->residual_max_a = best_residual_max;
    fit->weighted_residual_rms_a = best_weighted_rms;
    fit->normalized_residual_rms =
        (noise_a > 0.0f) ? (best_weighted_rms / noise_a) : 0.0f;
    fit->window_score = best_score;
    fit->residual_max_counts =
        (current_amp_per_count > 0.0f)
            ? (fit->residual_max_a / current_amp_per_count)
            : 0.0f;

    uint32_t same_sign_run = 0u;
    uint32_t max_same_sign_run = 0u;
    int32_t last_sign = 0;
    float curvature = 0.0f;
    uint32_t curvature_n = 0u;
    for (uint32_t i = best_start; i <= best_end; ++i) {
        const float err = fall[i] - best_fit[i - best_start];
        const float err_abs = fabsf(err);
        if (err_abs >= fit->residual_max_a) {
            fit->residual_max_index = i;
        }
        const int32_t sign = (err > 0.0f) ? 1 : ((err < 0.0f) ? -1 : 0);
        if (sign != 0 && sign == last_sign) {
            same_sign_run++;
        } else {
            same_sign_run = (sign != 0) ? 1u : 0u;
        }
        if (same_sign_run > max_same_sign_run) {
            max_same_sign_run = same_sign_run;
        }
        last_sign = sign;
        if (i >= best_start + 2u) {
            const float e0 = fall[i - 2u] - best_fit[i - 2u - best_start];
            const float e1 = fall[i - 1u] - best_fit[i - 1u - best_start];
            const float e2 = err;
            const float d2 = e2 - 2.0f * e1 + e0;
            curvature += d2 * d2;
            curvature_n++;
        }
    }
    fit->residual_same_sign_run = max_same_sign_run;
    fit->residual_curvature =
        (curvature_n > 0u) ? sqrtf(curvature / (float)curvature_n) : 0.0f;
    fit->valid = fit->point_count >= 5u &&
                 fit->fitted_inductance_h > 0.0f &&
                 isfinite(fit->r_squared) &&
                 fit->r_squared > 0.80f &&
                 fit->residual_rms_a <= fmaxf(6.0f * noise_a, 0.08f);
}

static void fit_arx(const float *rise,
                    uint32_t rise_count,
                    const float *fall,
                    uint32_t fall_count,
                    float sample_period_s,
                    float resistance_ohm,
                    float voltage_step_v,
                    bool fixed_r,
                    bool smooth,
                    MotorIdentArxFit *fit)
{
    memset(fit, 0, sizeof(*fit));
    if (rise == 0 || fall == 0 || rise_count < 3u || fall_count < 3u ||
        !finite_positive(sample_period_s) || !finite_positive(voltage_step_v)) {
        return;
    }

    float r[MOTOR_IDENT_RISE_SAMPLES];
    float f[MOTOR_IDENT_FALL_SAMPLES];
    for (uint32_t i = 0u; i < rise_count; ++i) {
        const float prev = (i > 0u) ? rise[i - 1u] : rise[i];
        const float next = (i + 1u < rise_count) ? rise[i + 1u] : rise[i];
        r[i] = smooth ? ((prev + rise[i] + next) / 3.0f) : rise[i];
    }
    for (uint32_t i = 0u; i < fall_count; ++i) {
        const float prev = (i > 0u) ? fall[i - 1u] : fall[i];
        const float next = (i + 1u < fall_count) ? fall[i + 1u] : fall[i];
        f[i] = smooth ? ((prev + fall[i] + next) / 3.0f) : fall[i];
    }

    if (fixed_r) {
        if (!finite_positive(resistance_ohm)) {
            return;
        }
        float sxx = 0.0f, sx = 0.0f, syx = 0.0f, sy = 0.0f;
        uint32_t n = 0u;
        for (uint32_t i = 0u; i + 1u < rise_count; ++i) {
            const float x = r[i] - voltage_step_v / resistance_ohm;
            const float y = r[i + 1u] - voltage_step_v / resistance_ohm;
            sxx += x * x; sx += x; syx += y * x; sy += y; n++;
        }
        for (uint32_t i = 0u; i + 1u < fall_count; ++i) {
            const float x = f[i];
            const float y = f[i + 1u];
            sxx += x * x; sx += x; syx += y * x; sy += y; n++;
        }
        const float nf = (float)n;
        const float det = nf * sxx - sx * sx;
        if (n < 4u || fabsf(det) < 1.0e-12f) {
            return;
        }
        fit->a = (nf * syx - sx * sy) / det;
        fit->c = (sy - fit->a * sx) / nf;
        fit->b = (1.0f - fit->a) / resistance_ohm;
    } else {
        float m00 = 0.0f, m01 = 0.0f, m02 = 0.0f;
        float m11 = 0.0f, m12 = 0.0f, m22 = 0.0f;
        float y0 = 0.0f, y1 = 0.0f, y2 = 0.0f;
        uint32_t n = 0u;
        for (uint32_t pass = 0u; pass < 2u; ++pass) {
            const float *arr = (pass == 0u) ? r : f;
            const uint32_t cnt = (pass == 0u) ? rise_count : fall_count;
            const float v = (pass == 0u) ? voltage_step_v : 0.0f;
            for (uint32_t i = 0u; i + 1u < cnt; ++i) {
                const float x0 = arr[i];
                const float x1 = v;
                const float x2 = 1.0f;
                const float yy = arr[i + 1u];
                m00 += x0 * x0; m01 += x0 * x1; m02 += x0 * x2;
                m11 += x1 * x1; m12 += x1 * x2; m22 += x2 * x2;
                y0 += x0 * yy; y1 += x1 * yy; y2 += x2 * yy;
                n++;
            }
        }
        const float det =
            m00 * (m11 * m22 - m12 * m12) -
            m01 * (m01 * m22 - m12 * m02) +
            m02 * (m01 * m12 - m11 * m02);
        if (n < 5u || fabsf(det) < 1.0e-12f) {
            return;
        }
        fit->a = (y0 * (m11 * m22 - m12 * m12) -
                  m01 * (y1 * m22 - m12 * y2) +
                  m02 * (y1 * m12 - m11 * y2)) / det;
        fit->b = (m00 * (y1 * m22 - m12 * y2) -
                  y0 * (m01 * m22 - m12 * m02) +
                  m02 * (m01 * y2 - y1 * m02)) / det;
        fit->c = (m00 * (m11 * y2 - y1 * m12) -
                  m01 * (m01 * y2 - y1 * m02) +
                  y0 * (m01 * m12 - m11 * m02)) / det;
    }

    if (!(fit->a > 0.0f && fit->a < 1.0f && fit->b > 0.0f)) {
        return;
    }
    fit->tau_s = -sample_period_s / logf(fit->a);
    fit->resistance_ohm = (1.0f - fit->a) / fit->b;
    fit->inductance_h = fit->resistance_ohm * fit->tau_s;
    if (fixed_r) {
        fit->resistance_ohm = resistance_ohm;
        fit->inductance_h = resistance_ohm * fit->tau_s;
    }
    fit->valid = finite_positive(fit->tau_s) &&
                 finite_positive(fit->resistance_ohm) &&
                 finite_positive(fit->inductance_h) &&
                 isfinite(fit->a) && isfinite(fit->b) && isfinite(fit->c);
    fit->r_squared = fit->valid ? 1.0f : 0.0f;
}

bool motor_ident_fit_repeated_arx_fixed_r(const float *rise_repeats,
                                          uint32_t repeat_count,
                                          uint32_t rise_count,
                                          const float *fall_repeats,
                                          uint32_t fall_count,
                                          float sample_period_s,
                                          float resistance_ohm,
                                          float voltage_step_v,
                                          MotorIdentArxFit *fit)
{
    if (fit == 0) {
        return false;
    }
    memset(fit, 0, sizeof(*fit));
    if (rise_repeats == 0 || fall_repeats == 0 || repeat_count == 0u ||
        rise_count < 3u || fall_count < 3u ||
        !finite_positive(sample_period_s) ||
        !finite_positive(resistance_ohm) ||
        !finite_positive(voltage_step_v)) {
        return false;
    }

    float sxx = 0.0f;
    float sx = 0.0f;
    float syx = 0.0f;
    float sy = 0.0f;
    uint32_t n = 0u;
    for (uint32_t rep = 0u; rep < repeat_count; ++rep) {
        const float *rise = &rise_repeats[rep * rise_count];
        const float *fall = &fall_repeats[rep * fall_count];
        for (uint32_t i = 0u; i + 1u < rise_count; ++i) {
            const float x = rise[i] - voltage_step_v / resistance_ohm;
            const float y = rise[i + 1u] - voltage_step_v / resistance_ohm;
            sxx += x * x;
            sx += x;
            syx += y * x;
            sy += y;
            n++;
        }
        for (uint32_t i = 0u; i + 1u < fall_count; ++i) {
            const float x = fall[i];
            const float y = fall[i + 1u];
            sxx += x * x;
            sx += x;
            syx += y * x;
            sy += y;
            n++;
        }
    }

    const float nf = (float)n;
    const float det = nf * sxx - sx * sx;
    if (n < 4u || fabsf(det) < 1.0e-12f) {
        return false;
    }
    fit->a = (nf * syx - sx * sy) / det;
    fit->c = (sy - fit->a * sx) / nf;
    fit->b = (1.0f - fit->a) / resistance_ohm;
    if (!(fit->a > 0.0f && fit->a < 1.0f && fit->b > 0.0f)) {
        return false;
    }

    fit->tau_s = -sample_period_s / logf(fit->a);
    fit->resistance_ohm = resistance_ohm;
    fit->inductance_h = resistance_ohm * fit->tau_s;
    if (!finite_positive(fit->tau_s) || !finite_positive(fit->inductance_h)) {
        return false;
    }

    float y_mean = 0.0f;
    uint32_t yn = 0u;
    for (uint32_t rep = 0u; rep < repeat_count; ++rep) {
        const float *rise = &rise_repeats[rep * rise_count];
        const float *fall = &fall_repeats[rep * fall_count];
        for (uint32_t i = 0u; i + 1u < rise_count; ++i) {
            y_mean += rise[i + 1u] - voltage_step_v / resistance_ohm;
            yn++;
        }
        for (uint32_t i = 0u; i + 1u < fall_count; ++i) {
            y_mean += fall[i + 1u];
            yn++;
        }
    }
    if (yn == 0u) {
        return false;
    }
    y_mean /= (float)yn;

    float ss_tot = 0.0f;
    float ss_res = 0.0f;
    for (uint32_t rep = 0u; rep < repeat_count; ++rep) {
        const float *rise = &rise_repeats[rep * rise_count];
        const float *fall = &fall_repeats[rep * fall_count];
        for (uint32_t i = 0u; i + 1u < rise_count; ++i) {
            const float x = rise[i] - voltage_step_v / resistance_ohm;
            const float y = rise[i + 1u] - voltage_step_v / resistance_ohm;
            const float y_fit = fit->a * x + fit->c;
            ss_tot += (y - y_mean) * (y - y_mean);
            ss_res += (y - y_fit) * (y - y_fit);
        }
        for (uint32_t i = 0u; i + 1u < fall_count; ++i) {
            const float x = fall[i];
            const float y = fall[i + 1u];
            const float y_fit = fit->a * x + fit->c;
            ss_tot += (y - y_mean) * (y - y_mean);
            ss_res += (y - y_fit) * (y - y_fit);
        }
    }
    fit->r_squared = (ss_tot > 1.0e-12f) ? (1.0f - ss_res / ss_tot) : 0.0f;
    fit->valid = true;
    return true;
}

static float linear_slope_per_sample(const float *values,
                                     uint32_t start,
                                     uint32_t end)
{
    if (values == 0 || end <= start) {
        return 0.0f;
    }

    const float n = (float)(end - start + 1u);
    float sx = 0.0f;
    float sy = 0.0f;
    float sxx = 0.0f;
    float sxy = 0.0f;
    for (uint32_t i = start; i <= end; ++i) {
        const float x = (float)(i - start);
        const float y = values[i];
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
    }
    const float det = n * sxx - sx * sx;
    return (fabsf(det) > 1.0e-12f) ? ((n * sxy - sx * sy) / det) : 0.0f;
}

static void evaluate_monotonic_window(const float *values,
                                      const float *std,
                                      uint32_t count,
                                      uint32_t start,
                                      uint32_t end,
                                      bool rise,
                                      float current_amp_per_count,
                                      MotorIdentMonotonicStats *stats)
{
    if (values == 0 || stats == 0 || count < 2u || start >= count) {
        return;
    }
    memset(stats, 0, sizeof(*stats));
    if (end >= count) {
        end = count - 1u;
    }
    if (end <= start) {
        end = (start + 1u < count) ? (start + 1u) : start;
    }

    stats->window_start = start;
    stats->window_end = end;
    stats->global_trend_slope = linear_slope_per_sample(values, start, end);

    bool large_violation = false;
    for (uint32_t k = start + 1u; k <= end; ++k) {
        const float s0 = (std != 0) ? std[k - 1u] : 0.0f;
        const float s1 = (std != 0) ? std[k] : 0.0f;
        const float sigma_delta = sqrtf(s0 * s0 + s1 * s1);
        const float tolerance =
            fmaxf(3.0f * sigma_delta, 0.5f * current_amp_per_count);
        const float local_max_allowed =
            fmaxf(4.0f * sigma_delta, 1.0f * current_amp_per_count);
        const float reversal =
            rise ? (values[k - 1u] - values[k])
                 : (values[k] - values[k - 1u]);

        if (tolerance > stats->tolerance_max_a) {
            stats->tolerance_max_a = tolerance;
        }
        if (reversal > tolerance) {
            stats->violation_count++;
            if (reversal > stats->max_violation_a) {
                stats->max_violation_a = reversal;
            }
            if (reversal > local_max_allowed) {
                large_violation = true;
            }
        }
        stats->comparison_count++;
    }

    stats->violation_ratio =
        (stats->comparison_count > 0u)
            ? ((float)stats->violation_count / (float)stats->comparison_count)
            : 1.0f;
    stats->max_violation_counts =
        (current_amp_per_count > 0.0f)
            ? (stats->max_violation_a / current_amp_per_count)
            : 0.0f;
    const bool global_trend_ok =
        rise ? (stats->global_trend_slope > 0.0f)
             : (stats->global_trend_slope < 0.0f);
    stats->ok = (stats->comparison_count > 0u) &&
                (stats->violation_ratio <= 0.15f) &&
                !large_violation &&
                global_trend_ok;
}

static uint32_t rise_monotonic_end_before_90_percent(const float *rise,
                                                     uint32_t count,
                                                     const MotorIdentCurveFit *fit)
{
    if (rise == 0 || fit == 0 || count < 2u || !fit->valid) {
        return (count > 0u) ? (count - 1u) : 0u;
    }
    const float steady = fit->fitted_offset_a + fit->fitted_amplitude_a;
    const float threshold = steady * 0.90f;
    for (uint32_t i = 0u; i < count; ++i) {
        if (rise[i] >= threshold) {
            if (i > 1u) {
                return i - 1u;
            }
            return (count > 1u) ? 1u : 0u;
        }
    }
    return count - 1u;
}

static float median_positive_methods(float *values, uint32_t count)
{
    uint32_t n = 0u;
    for (uint32_t i = 0u; i < count; ++i) {
        if (finite_positive(values[i])) {
            values[n++] = values[i];
        }
    }
    return (n > 0u) ? median_of_values(values, n) : 0.0f;
}

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
                                  MotorIdentRobustResult *result)
{
    if (result == 0) {
        return false;
    }
    memset(result, 0, sizeof(*result));
    if (rise == 0 || fall == 0 ||
        rise_count < 6u || rise_count > MOTOR_IDENT_RISE_SAMPLES ||
        fall_count < 8u || fall_count > MOTOR_IDENT_FALL_SAMPLES ||
        !finite_positive(sample_period_s) ||
        !finite_positive(resistance_ohm) ||
        !finite_positive(voltage_step_v)) {
        return false;
    }

    float rise_tail[10u];
    uint32_t rise_tail_n = 0u;
    const uint32_t rise_tail_start = (rise_count > 10u) ? (rise_count - 10u) : 0u;
    for (uint32_t i = rise_tail_start; i < rise_count; ++i) {
        rise_tail[rise_tail_n++] = rise[i];
    }
    result->rise_tail_median_a = median_of_values(rise_tail, rise_tail_n);
    result->rise_peak_a = rise[0];
    for (uint32_t i = 1u; i < rise_count; ++i) {
        if (rise[i] > result->rise_peak_a) {
            result->rise_peak_a = rise[i];
        }
    }
    result->rise_peak_minus_tail_a =
        result->rise_peak_a - result->rise_tail_median_a;

    float fall_tail[10u];
    uint32_t tail_n = 0u;
    const uint32_t tail_start = (fall_count > 10u) ? (fall_count - 10u) : 0u;
    for (uint32_t i = tail_start; i < fall_count; ++i) {
        fall_tail[tail_n++] = fall[i];
    }
    result->fall_tail_sigma_a = robust_sigma_mad(fall_tail, tail_n);
    const float std_tail = stddev_values(fall_tail, tail_n);
    result->noise_sigma_a = fmaxf(result->fall_tail_sigma_a, std_tail * 0.5f);
    if (current_amp_per_count > 0.0f) {
        result->noise_sigma_a =
            fmaxf(result->noise_sigma_a, 0.5f * current_amp_per_count);
    }
    result->effective_noise_a = result->noise_sigma_a;
    result->effective_noise_counts =
        (current_amp_per_count > 0.0f)
            ? (result->effective_noise_a / current_amp_per_count)
            : 0.0f;

    fit_rise_nonlinear(rise, rise_count, sample_period_s, resistance_ohm,
                       result->effective_noise_a, &result->rise_fit);
    fit_fall_adaptive(fall, fall_count, fall_std, sample_period_s, resistance_ohm,
                      result->effective_noise_a, current_amp_per_count,
                      &result->fall_fit);

    const uint32_t rise_mono_start = 0u;
    const uint32_t rise_mono_end =
        rise_monotonic_end_before_90_percent(rise, rise_count, &result->rise_fit);
    evaluate_monotonic_window(rise,
                              rise_std,
                              rise_count,
                              rise_mono_start,
                              rise_mono_end,
                              true,
                              current_amp_per_count,
                              &result->rise_monotonic);
    const uint32_t fall_mono_start = result->fall_fit.valid ? result->fall_fit.start_index : 0u;
    const uint32_t fall_mono_end =
        result->fall_fit.valid ? result->fall_fit.end_index :
        ((fall_count > 0u) ? (fall_count - 1u) : 0u);
    evaluate_monotonic_window(fall,
                              fall_std,
                              fall_count,
                              fall_mono_start,
                              fall_mono_end,
                              false,
                              current_amp_per_count,
                              &result->fall_monotonic);
    result->monotonic_rise_ok = result->rise_monotonic.ok;
    result->monotonic_fall_ok = result->fall_monotonic.ok;

    fit_arx(rise, rise_count, fall, fall_count, sample_period_s, resistance_ohm,
            voltage_step_v, false, false, &result->arx_free);
    fit_arx(rise, rise_count, fall, fall_count, sample_period_s, resistance_ohm,
            voltage_step_v, true, false, &result->arx_fixed_r);
    fit_arx(rise, rise_count, fall, fall_count, sample_period_s, resistance_ohm,
            voltage_step_v, true, true, &result->arx_smoothed_fixed_r);

    float methods[5u] = {0};
    uint32_t n = 0u;
    if (result->rise_fit.valid && result->rise_fit.r_squared >= 0.90f) {
        methods[n++] = result->rise_fit.fitted_inductance_h;
    }
    if (result->fall_fit.valid && result->fall_fit.r_squared >= 0.90f) {
        methods[n++] = result->fall_fit.fitted_inductance_h;
    }
    if (result->arx_fixed_r.valid) {
        methods[n++] = result->arx_fixed_r.inductance_h;
    }
    if (result->arx_smoothed_fixed_r.valid) {
        methods[n++] = result->arx_smoothed_fixed_r.inductance_h;
    }
    if (result->arx_free.valid &&
        result->arx_free.resistance_ohm > resistance_ohm * 0.25f &&
        result->arx_free.resistance_ohm < resistance_ohm * 4.0f) {
        methods[n++] = result->arx_free.inductance_h;
    }

    result->fused_method_count = n;
    result->fused_inductance_h = median_positive_methods(methods, n);
    result->reliable = result->fused_method_count >= 2u &&
                       result->fused_inductance_h > 0.0f &&
                       result->monotonic_rise_ok &&
                       result->monotonic_fall_ok &&
                       result->rise_fit.r_squared >= 0.90f;
    return true;
}

#include "control/current_sensor_noise_diagnostic.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

static int16_t s_sort_scratch[4096];

static void sort_i16(int16_t *values, uint32_t count)
{
    for (uint32_t i = 1u; i < count; ++i) {
        const int16_t key = values[i];
        uint32_t j = i;
        while (j > 0u && values[j - 1u] > key) {
            values[j] = values[j - 1u];
            --j;
        }
        values[j] = key;
    }
}

static float percentile_sorted_i16(const int16_t *sorted,
                                   uint32_t count,
                                   float percentile)
{
    if (sorted == NULL || count == 0u) {
        return 0.0f;
    }
    if (percentile <= 0.0f) {
        return (float)sorted[0];
    }
    if (percentile >= 100.0f) {
        return (float)sorted[count - 1u];
    }

    const float pos = (percentile * 0.01f) * (float)(count - 1u);
    const uint32_t lo = (uint32_t)floorf(pos);
    uint32_t hi = lo + 1u;
    if (hi >= count) {
        hi = count - 1u;
    }
    const float frac = pos - (float)lo;
    return (float)sorted[lo] + ((float)sorted[hi] - (float)sorted[lo]) * frac;
}

float current_sensor_noise_percentile_i16(const int16_t *values,
                                          uint32_t count,
                                          float percentile)
{
    if (values == NULL || count == 0u || count > 4096u) {
        return 0.0f;
    }
    memcpy(s_sort_scratch, values, count * sizeof(s_sort_scratch[0]));
    sort_i16(s_sort_scratch, count);
    return percentile_sorted_i16(s_sort_scratch, count, percentile);
}

float current_sensor_noise_median_i16(const int16_t *values, uint32_t count)
{
    return current_sensor_noise_percentile_i16(values, count, 50.0f);
}

float current_sensor_noise_mad_i16(const int16_t *values, uint32_t count)
{
    if (values == NULL || count == 0u || count > 4096u) {
        return 0.0f;
    }

    const float median = current_sensor_noise_median_i16(values, count);
    for (uint32_t i = 0u; i < count; ++i) {
        s_sort_scratch[i] = (int16_t)lrintf(fabsf((float)values[i] - median));
    }
    sort_i16(s_sort_scratch, count);
    return percentile_sorted_i16(s_sort_scratch, count, 50.0f);
}

static void count_stats_compute(const int16_t *values,
                                uint32_t count,
                                CurrentSensorNoiseCountStats *out)
{
    memset(out, 0, sizeof(*out));
    if (values == NULL || count == 0u) {
        return;
    }

    int16_t min_v = values[0];
    int16_t max_v = values[0];
    double sum = 0.0;
    double sumsq = 0.0;
    for (uint32_t i = 0u; i < count; ++i) {
        const int16_t v = values[i];
        const int16_t av = (v < 0) ? (int16_t)-v : v;
        if (v < min_v) { min_v = v; }
        if (v > max_v) { max_v = v; }
        sum += (double)v;
        sumsq += (double)v * (double)v;
        if (av > 2) { out->abs_over_2_count++; }
        if (av > 4) { out->abs_over_4_count++; }
        if (av > 6) { out->abs_over_6_count++; }
        if (av > 8) { out->abs_over_8_count++; }
        if (av > 10) { out->abs_over_10_count++; }
        if (av > 12) { out->abs_over_12_count++; }
    }

    const double inv = 1.0 / (double)count;
    const double mean = sum * inv;
    double var = sumsq * inv - mean * mean;
    if (var < 0.0) { var = 0.0; }

    out->sample_count = count;
    out->mean = (float)mean;
    out->standard_deviation = sqrtf((float)var);
    out->min = min_v;
    out->max = max_v;
    out->peak_to_peak = (int16_t)(max_v - min_v);
    out->median = current_sensor_noise_median_i16(values, count);
    out->mad = current_sensor_noise_mad_i16(values, count);
    out->percentile_90 = current_sensor_noise_percentile_i16(values, count, 90.0f);
    out->percentile_95 = current_sensor_noise_percentile_i16(values, count, 95.0f);
    out->percentile_99 = current_sensor_noise_percentile_i16(values, count, 99.0f);
    out->percentile_99_5 = current_sensor_noise_percentile_i16(values, count, 99.5f);
    out->percentile_99_9 = current_sensor_noise_percentile_i16(values, count, 99.9f);
    for (uint32_t i = 0u; i < count; ++i) {
        const int16_t v = values[i];
        s_sort_scratch[i] = (v < 0) ? (int16_t)-v : v;
    }
    sort_i16(s_sort_scratch, count);
    out->abs_percentile_99_9 =
        percentile_sorted_i16(s_sort_scratch, count, 99.9f);
}

static void current_stats_update(CurrentSensorNoiseCurrentStats *stats, float v)
{
    if (stats->sample_count == 0u) {
        stats->min = v;
        stats->max = v;
    }
    const float n = (float)stats->sample_count;
    const float old_mean = stats->mean;
    stats->mean = (stats->mean * n + v) / (n + 1.0f);
    stats->standard_deviation += (v - old_mean) * (v - stats->mean);
    if (v < stats->min) { stats->min = v; }
    if (v > stats->max) { stats->max = v; }
    if (fabsf(v) > stats->abs_peak) { stats->abs_peak = fabsf(v); }
    stats->sample_count++;
}

static void current_stats_finalize(CurrentSensorNoiseCurrentStats *stats)
{
    if (stats->sample_count > 1u) {
        stats->standard_deviation =
            sqrtf(stats->standard_deviation / (float)(stats->sample_count - 1u));
    } else {
        stats->standard_deviation = 0.0f;
    }
}

static void worst_insert(CurrentSensorNoiseAnalysis *out,
                         const CurrentSensorNoiseWorstSample *sample)
{
    uint32_t pos = out->worst_count;
    if (pos < CURRENT_SENSOR_NOISE_WORST_CAPACITY) {
        out->worst_count++;
    } else if (sample->phase_abs <=
               out->worst[CURRENT_SENSOR_NOISE_WORST_CAPACITY - 1u].phase_abs) {
        return;
    } else {
        pos = CURRENT_SENSOR_NOISE_WORST_CAPACITY - 1u;
    }

    while (pos > 0u && sample->phase_abs > out->worst[pos - 1u].phase_abs) {
        out->worst[pos] = out->worst[pos - 1u];
        --pos;
    }
    out->worst[pos] = *sample;
}

static void online_count_push(CurrentSensorNoiseOnlineCountAccumulator *acc,
                              int16_t value)
{
    if (acc->count == 0u) {
        acc->min = value;
        acc->max = value;
    }
    if (value < acc->min) { acc->min = value; }
    if (value > acc->max) { acc->max = value; }
    acc->count++;
    acc->sum += (int64_t)value;
    acc->sum_square += (int64_t)value * (int64_t)value;

    if (value < CURRENT_SENSOR_NOISE_DELTA_HIST_MIN) {
        acc->underflow_count++;
    } else if (value > CURRENT_SENSOR_NOISE_DELTA_HIST_MAX) {
        acc->overflow_count++;
    } else {
        const uint32_t bin =
            (uint32_t)((int32_t)value - CURRENT_SENSOR_NOISE_DELTA_HIST_MIN);
        acc->histogram[bin]++;
    }
}

static uint16_t abs_i16_to_u16(int16_t value)
{
    return (value < 0) ? (uint16_t)(-((int32_t)value)) : (uint16_t)value;
}

static void online_current_push_counts(CurrentSensorNoiseOnlineCurrentAccumulator *acc,
                                       int16_t value,
                                       float scale)
{
    if (acc->count == 0u) {
        acc->min = value;
        acc->max = value;
        acc->scale = scale;
    }
    if (value < acc->min) { acc->min = value; }
    if (value > acc->max) { acc->max = value; }
    const uint16_t av = abs_i16_to_u16(value);
    if (av > acc->abs_peak) { acc->abs_peak = av; }
    acc->count++;
    acc->sum += (int64_t)value;
    acc->sum_square += (int64_t)value * (int64_t)value;
    acc->scale = scale;
}

static float hist_percentile_counts(
    const CurrentSensorNoiseOnlineCountAccumulator *acc,
    float percentile)
{
    if (acc == NULL || acc->count == 0u) {
        return 0.0f;
    }
    uint32_t target = (uint32_t)ceilf((percentile * 0.01f) * (float)acc->count);
    if (target == 0u) { target = 1u; }
    uint32_t seen = acc->underflow_count;
    if (seen >= target) {
        return (float)CURRENT_SENSOR_NOISE_DELTA_HIST_MIN;
    }
    for (uint32_t i = 0u; i < CURRENT_SENSOR_NOISE_DELTA_HIST_BINS; ++i) {
        seen += acc->histogram[i];
        if (seen >= target) {
            return (float)((int32_t)i + CURRENT_SENSOR_NOISE_DELTA_HIST_MIN);
        }
    }
    return (float)CURRENT_SENSOR_NOISE_DELTA_HIST_MAX;
}

static float hist_abs_percentile_counts(
    const CurrentSensorNoiseOnlineCountAccumulator *acc,
    float percentile)
{
    if (acc == NULL || acc->count == 0u) {
        return 0.0f;
    }
    uint32_t target = (uint32_t)ceilf((percentile * 0.01f) * (float)acc->count);
    if (target == 0u) { target = 1u; }
    uint32_t seen = 0u;
    for (uint32_t abs_v = 0u;
         abs_v <= (uint32_t)CURRENT_SENSOR_NOISE_DELTA_HIST_MAX;
         ++abs_v) {
        uint32_t bin_count = 0u;
        const int32_t pos = (int32_t)abs_v;
        const int32_t neg = -(int32_t)abs_v;
        if (pos >= CURRENT_SENSOR_NOISE_DELTA_HIST_MIN &&
            pos <= CURRENT_SENSOR_NOISE_DELTA_HIST_MAX) {
            bin_count += acc->histogram[(uint32_t)(pos - CURRENT_SENSOR_NOISE_DELTA_HIST_MIN)];
        }
        if (abs_v != 0u &&
            neg >= CURRENT_SENSOR_NOISE_DELTA_HIST_MIN &&
            neg <= CURRENT_SENSOR_NOISE_DELTA_HIST_MAX) {
            bin_count += acc->histogram[(uint32_t)(neg - CURRENT_SENSOR_NOISE_DELTA_HIST_MIN)];
        }
        seen += bin_count;
        if (seen >= target) {
            return (float)abs_v;
        }
    }
    return (float)CURRENT_SENSOR_NOISE_DELTA_HIST_MAX;
}

static float hist_mad_counts(
    const CurrentSensorNoiseOnlineCountAccumulator *acc,
    float median)
{
    if (acc == NULL || acc->count == 0u) {
        return 0.0f;
    }
    uint32_t deviation_hist[CURRENT_SENSOR_NOISE_DELTA_HIST_BINS] = {0};
    uint32_t total = 0u;
    for (uint32_t i = 0u; i < CURRENT_SENSOR_NOISE_DELTA_HIST_BINS; ++i) {
        const int32_t value = (int32_t)i + CURRENT_SENSOR_NOISE_DELTA_HIST_MIN;
        uint32_t d = (uint32_t)lrintf(fabsf((float)value - median));
        if (d >= CURRENT_SENSOR_NOISE_DELTA_HIST_BINS) {
            d = CURRENT_SENSOR_NOISE_DELTA_HIST_BINS - 1u;
        }
        deviation_hist[d] += acc->histogram[i];
        total += acc->histogram[i];
    }
    if (total == 0u) {
        return 0.0f;
    }
    const uint32_t target = (total + 1u) / 2u;
    uint32_t seen = 0u;
    for (uint32_t i = 0u; i < CURRENT_SENSOR_NOISE_DELTA_HIST_BINS; ++i) {
        seen += deviation_hist[i];
        if (seen >= target) {
            return (float)i;
        }
    }
    return 0.0f;
}

static void online_count_finalize(const CurrentSensorNoiseOnlineCountAccumulator *acc,
                                  CurrentSensorNoiseCountStats *out)
{
    memset(out, 0, sizeof(*out));
    if (acc == NULL || acc->count == 0u) {
        return;
    }
    const float mean = (float)((double)acc->sum / (double)acc->count);
    double var = ((double)acc->sum_square / (double)acc->count) -
                 ((double)mean * (double)mean);
    if (var < 0.0) { var = 0.0; }
    out->sample_count = acc->count;
    out->mean = mean;
    out->standard_deviation = sqrtf((float)var);
    out->min = acc->min;
    out->max = acc->max;
    out->peak_to_peak = (int16_t)(acc->max - acc->min);
    out->median = hist_percentile_counts(acc, 50.0f);
    out->mad = hist_mad_counts(acc, out->median);
    out->percentile_90 = hist_percentile_counts(acc, 90.0f);
    out->percentile_95 = hist_percentile_counts(acc, 95.0f);
    out->percentile_99 = hist_percentile_counts(acc, 99.0f);
    out->percentile_99_5 = hist_percentile_counts(acc, 99.5f);
    out->percentile_99_9 = hist_percentile_counts(acc, 99.9f);
    out->abs_percentile_99_9 = hist_abs_percentile_counts(acc, 99.9f);

    for (uint32_t i = 0u; i < CURRENT_SENSOR_NOISE_DELTA_HIST_BINS; ++i) {
        const int16_t v = (int16_t)((int32_t)i + CURRENT_SENSOR_NOISE_DELTA_HIST_MIN);
        const uint32_t c = acc->histogram[i];
        const int16_t av = (v < 0) ? (int16_t)-v : v;
        if (av > 2) { out->abs_over_2_count += c; }
        if (av > 4) { out->abs_over_4_count += c; }
        if (av > 6) { out->abs_over_6_count += c; }
        if (av > 8) { out->abs_over_8_count += c; }
        if (av > 10) { out->abs_over_10_count += c; }
        if (av > 12) { out->abs_over_12_count += c; }
    }
    out->abs_over_12_count += acc->underflow_count + acc->overflow_count;
}

static void online_current_finalize(
    const CurrentSensorNoiseOnlineCurrentAccumulator *acc,
    CurrentSensorNoiseCurrentStats *out)
{
    memset(out, 0, sizeof(*out));
    if (acc == NULL || acc->count == 0u) {
        return;
    }
    const float scale = acc->scale;
    const double inv = 1.0 / (double)acc->count;
    const double mean_counts = (double)acc->sum * inv;
    double var_counts = ((double)acc->sum_square * inv) - (mean_counts * mean_counts);
    if (var_counts < 0.0) {
        var_counts = 0.0;
    }
    out->sample_count = acc->count;
    out->mean = (float)mean_counts * scale;
    out->standard_deviation = sqrtf((float)var_counts) * scale;
    out->min = (float)acc->min * scale;
    out->max = (float)acc->max * scale;
    out->abs_peak = (float)acc->abs_peak * scale;
}

static float phase_hist_percentile_counts(
    const CurrentSensorNoiseOnlinePhaseAccumulator *acc,
    float percentile)
{
    if (acc == NULL || acc->count == 0u) {
        return 0.0f;
    }
    uint32_t target = (uint32_t)ceilf((percentile * 0.01f) * (float)acc->count);
    if (target == 0u) { target = 1u; }
    uint32_t seen = 0u;
    for (uint32_t i = 0u; i < CURRENT_SENSOR_NOISE_PHASE_HIST_BINS; ++i) {
        seen += acc->phase_histogram[i];
        if (seen >= target) {
            return (float)i;
        }
    }
    return (float)(CURRENT_SENSOR_NOISE_PHASE_HIST_BINS - 1u);
}

static void online_worst_refresh_min(CurrentSensorNoiseOnlineAccumulator *acc)
{
    if (acc == NULL || acc->worst_count == 0u) {
        return;
    }
    uint32_t min_index = 0u;
    uint16_t min_metric = (uint16_t)acc->worst[0].phase_abs;
    for (uint32_t i = 1u; i < acc->worst_count; ++i) {
        const uint16_t metric = (uint16_t)acc->worst[i].phase_abs;
        if (metric < min_metric) {
            min_metric = metric;
            min_index = i;
        }
    }
    acc->worst_min_index = min_index;
    acc->worst_min_phase_metric_counts = min_metric;
}

static void online_worst_insert(CurrentSensorNoiseOnlineAccumulator *acc,
                                const CurrentSensorNoiseWorstSample *sample)
{
    if (acc == NULL || sample == NULL) {
        return;
    }
    const uint16_t metric = (uint16_t)sample->phase_abs;
    if (acc->worst_count < CURRENT_SENSOR_NOISE_WORST_CAPACITY) {
        const uint32_t pos = acc->worst_count++;
        acc->worst[pos] = *sample;
        acc->pending_next_valid[pos] = true;
        online_worst_refresh_min(acc);
        return;
    }
    if (metric <= acc->worst_min_phase_metric_counts) {
        return;
    }
    const uint32_t pos = acc->worst_min_index;
    acc->worst[pos] = *sample;
    acc->pending_next_valid[pos] = true;
    online_worst_refresh_min(acc);
}

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
                                  CurrentSensorNoiseAnalysis *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (delta_pc0 == NULL || delta_pc1 == NULL || count == 0u || count > 4096u) {
        return;
    }

    count_stats_compute(delta_pc0, count, &out->pc0);
    count_stats_compute(delta_pc1, count, &out->pc1);

    uint32_t above08 = 0u;
    uint32_t above12 = 0u;
    uint32_t above16 = 0u;
    uint32_t above20 = 0u;
    uint32_t run08 = 0u;
    uint32_t run12 = 0u;
    uint32_t run16 = 0u;
    uint32_t run20 = 0u;
    uint32_t best08 = 0u;
    uint32_t best12 = 0u;
    uint32_t best16 = 0u;
    uint32_t best20 = 0u;

    for (uint32_t i = 0u; i < count; ++i) {
        const float iv = (float)delta_pc0[i] * current_amp_per_count;
        const float iw = (float)delta_pc1[i] * current_amp_per_count;
        const float iu = -(iv + iw);
        const float id = iu;
        const float iq = (iv - iw) * 0.57735026919f;
        float phase_abs = fabsf(iu);
        if (fabsf(iv) > phase_abs) { phase_abs = fabsf(iv); }
        if (fabsf(iw) > phase_abs) { phase_abs = fabsf(iw); }

        current_stats_update(&out->iv, iv);
        current_stats_update(&out->iw, iw);
        current_stats_update(&out->iu, iu);
        current_stats_update(&out->id, id);
        current_stats_update(&out->iq, iq);

        s_sort_scratch[i] = (int16_t)lrintf(phase_abs * 1000.0f);
        if (phase_abs > out->phase.phase_abs_peak) {
            out->phase.phase_abs_peak = phase_abs;
        }
        if (phase_abs > 0.08f) {
            above08++;
            run08++;
            if (run08 > best08) { best08 = run08; }
        } else {
            run08 = 0u;
        }
        if (phase_abs > 0.12f) {
            above12++;
            run12++;
            if (run12 > best12) { best12 = run12; }
        } else {
            run12 = 0u;
        }
        if (phase_abs > 0.16f) {
            above16++;
            run16++;
            if (run16 > best16) { best16 = run16; }
        } else {
            run16 = 0u;
        }
        if (phase_abs > 0.20f) {
            above20++;
            run20++;
            if (run20 > best20) { best20 = run20; }
        } else {
            run20 = 0u;
        }

        CurrentSensorNoiseWorstSample worst = {0};
        worst.sample_index = i;
        worst.adc_seq = (adc_seq != NULL) ? adc_seq[i] : 0u;
        worst.raw_pc0 = (raw_pc0 != NULL) ? raw_pc0[i] : 0u;
        worst.raw_pc1 = (raw_pc1 != NULL) ? raw_pc1[i] : 0u;
        worst.delta_pc0 = delta_pc0[i];
        worst.delta_pc1 = delta_pc1[i];
        worst.prev_delta_pc0 = (i > 0u) ? delta_pc0[i - 1u] : delta_pc0[i];
        worst.prev_delta_pc1 = (i > 0u) ? delta_pc1[i - 1u] : delta_pc1[i];
        worst.next_delta_pc0 = (i + 1u < count) ? delta_pc0[i + 1u] : delta_pc0[i];
        worst.next_delta_pc1 = (i + 1u < count) ? delta_pc1[i + 1u] : delta_pc1[i];
        worst.iv = iv;
        worst.iw = iw;
        worst.iu = iu;
        worst.id = id;
        worst.iq = iq;
        worst.phase_abs = phase_abs;
        worst.tim1_cnt = (tim1_cnt != NULL) ? tim1_cnt[i] : 0u;
        worst.adc_rank_order = adc_rank_order;
        worst.callback_count = (callback_count != NULL) ? callback_count[i] : 0u;
        worst_insert(out, &worst);
    }

    current_stats_finalize(&out->iv);
    current_stats_finalize(&out->iw);
    current_stats_finalize(&out->iu);
    current_stats_finalize(&out->id);
    current_stats_finalize(&out->iq);

    out->phase.sample_count = count;
    sort_i16(s_sort_scratch, count);
    out->phase.phase_percentile_99 =
        percentile_sorted_i16(s_sort_scratch, count, 99.0f) * 0.001f;
    out->phase.phase_percentile_99_5 =
        percentile_sorted_i16(s_sort_scratch, count, 99.5f) * 0.001f;
    out->phase.phase_percentile_99_9 =
        percentile_sorted_i16(s_sort_scratch, count, 99.9f) * 0.001f;
    out->phase.samples_above_0p08A = above08;
    out->phase.samples_above_0p12A = above12;
    out->phase.samples_above_0p16A = above16;
    out->phase.samples_above_0p20A = above20;
    out->phase.longest_consecutive_above_0p08A = best08;
    out->phase.longest_consecutive_above_0p12A = best12;
    out->phase.longest_consecutive_above_0p16A = best16;
    out->phase.longest_consecutive_above_0p20A = best20;
}

void current_sensor_noise_seq_tracker_init(CurrentSensorNoiseSeqTracker *tracker)
{
    if (tracker != NULL) {
        memset(tracker, 0, sizeof(*tracker));
    }
}

void current_sensor_noise_seq_tracker_observe(CurrentSensorNoiseSeqTracker *tracker,
                                              uint32_t seq)
{
    if (tracker == NULL) {
        return;
    }
    if (!tracker->initialized) {
        tracker->initialized = true;
        tracker->first_seq = seq;
        tracker->last_seq = seq;
        tracker->observed_count = 1u;
        return;
    }

    if (seq == tracker->last_seq) {
        tracker->duplicate_count++;
        return;
    }

    const uint32_t delta = seq - tracker->last_seq;
    if (delta == 0u) {
        tracker->duplicate_count++;
    } else if (delta < 0x80000000u) {
        if (delta > 1u) {
            tracker->gap_count += delta - 1u;
        }
        tracker->last_seq = seq;
        tracker->observed_count++;
    } else {
        tracker->duplicate_count++;
    }
}

void current_sensor_noise_seq_tracker_to_integrity(
    const CurrentSensorNoiseSeqTracker *tracker,
    uint32_t expected_snapshot_count,
    uint32_t analysis_sample_count,
    uint32_t discard_sample_count,
    CurrentSensorNoiseAdcIntegrity *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->diagnostic_expected_snapshot_seen = expected_snapshot_count;
    out->diagnostic_analysis_sample_count = analysis_sample_count;
    out->diagnostic_discard_sample_count = discard_sample_count;
    if (tracker == NULL || !tracker->initialized) {
        return;
    }
    out->total_adc_snapshot_count = tracker->observed_count;
    out->coherent_snapshot_count = tracker->observed_count;
    out->diagnostic_adc_seq_first = tracker->first_seq;
    out->diagnostic_adc_seq_last = tracker->last_seq;
    out->diagnostic_adc_seq_span = tracker->last_seq - tracker->first_seq + 1u;
    out->missed_adc_seq = tracker->gap_count;
    out->duplicate_adc_seq = tracker->duplicate_count;
    out->adc_sync_rate = (expected_snapshot_count > 0u)
                             ? ((float)tracker->observed_count /
                                (float)expected_snapshot_count)
                             : 0.0f;
}

CurrentSensorNoiseAdmissionResult
current_sensor_noise_evaluate_zero_current_admission(
    const CurrentSensorNoiseAnalysis *analysis)
{
    CurrentSensorNoiseAdmissionResult out = {0};
    if (analysis == NULL) {
        return out;
    }

    const float pc0_abs_p999 = analysis->pc0.abs_percentile_99_9;
    const float pc1_abs_p999 = analysis->pc1.abs_percentile_99_9;
    const float high_tail_limit_counts = 12.0f;

    out.mean_ok = fabsf(analysis->iv.mean) <= 0.03f &&
                  fabsf(analysis->iw.mean) <= 0.03f &&
                  fabsf(analysis->iu.mean) <= 0.04f;
    out.base_noise_ok = analysis->pc0.standard_deviation <= 3.0f &&
                        analysis->pc1.standard_deviation <= 3.0f &&
                        analysis->pc0.mad <= 2.0f &&
                        analysis->pc1.mad <= 2.0f;
    out.high_tail_pc0_value_counts = pc0_abs_p999;
    out.high_tail_pc1_value_counts = pc1_abs_p999;
    out.high_tail_limit_counts = high_tail_limit_counts;
    out.high_tail_pc0_pass = pc0_abs_p999 <= high_tail_limit_counts;
    out.high_tail_pc1_pass = pc1_abs_p999 <= high_tail_limit_counts;
    out.high_tail_ok = out.high_tail_pc0_pass && out.high_tail_pc1_pass;
    out.consecutive_ok =
        analysis->phase.longest_consecutive_above_0p20A <= 1u &&
        analysis->phase.longest_consecutive_above_0p16A <= 4u &&
        analysis->phase.longest_consecutive_above_0p12A <= 8u;
    out.hard_outlier_ok =
        abs((int)analysis->pc0.min) <= 20 &&
        abs((int)analysis->pc0.max) <= 20 &&
        abs((int)analysis->pc1.min) <= 20 &&
        abs((int)analysis->pc1.max) <= 20;
    out.candidate_ok = out.mean_ok && out.base_noise_ok && out.high_tail_ok &&
                       out.consecutive_ok && out.hard_outlier_ok;
    return out;
}

CurrentSensorNoiseTransientResult
current_sensor_noise_classify_phase_abs(const float *phase_abs,
                                         uint32_t count,
                                         float sustained_threshold_a,
                                         uint32_t sustained_ticks,
                                         float instant_threshold_a)
{
    CurrentSensorNoiseTransientResult out = {0};
    uint32_t run = 0u;
    for (uint32_t i = 0u; i < count; ++i) {
        const float v = fabsf(phase_abs[i]);
        if (v > out.filtered_or_window_peak) {
            out.filtered_or_window_peak = v;
        }
        if (v > instant_threshold_a) {
            out.instantaneous_hard_fault = true;
        }
        if (v > sustained_threshold_a) {
            run++;
            if (run > out.consecutive_count) {
                out.consecutive_count = run;
            }
        } else {
            run = 0u;
        }
    }
    out.sustained_soft_fault =
        sustained_ticks > 0u && out.consecutive_count >= sustained_ticks;
    return out;
}

CurrentSensorNoiseLowpassEstimate
current_sensor_noise_estimate_one_pole_lowpass(float sample_rate_hz,
                                               float cutoff_hz,
                                               float noise_std_a,
                                               float signal_a)
{
    CurrentSensorNoiseLowpassEstimate out = {0};
    out.cutoff_hz = cutoff_hz;
    if (sample_rate_hz <= 0.0f || cutoff_hz <= 0.0f) {
        return out;
    }

    const float pi = 3.14159265358979323846f;
    const float alpha = 1.0f - expf(-2.0f * pi * cutoff_hz / sample_rate_hz);
    out.rms_reduction_ratio = sqrtf(alpha / (2.0f - alpha));

    const float omega = 2.0f * pi * 200.0f / sample_rate_hz;
    const float pole = 1.0f - alpha;
    const float denom_re = 1.0f - pole * cosf(omega);
    const float denom_im = pole * sinf(omega);
    const float phase = -atan2f(denom_im, denom_re);
    out.phase_delay_us_at_200hz = (-phase / (2.0f * pi * 200.0f)) * 1000000.0f;
    const float filtered_noise = noise_std_a * out.rms_reduction_ratio;
    out.snr_0p10a_after_filter =
        (filtered_noise > 0.0f) ? (signal_a / filtered_noise) : 0.0f;
    return out;
}

void current_sensor_noise_online_reset(CurrentSensorNoiseOnlineAccumulator *acc)
{
    if (acc != NULL) {
        memset(acc, 0, sizeof(*acc));
    }
}

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
                                      float current_amp_per_count)
{
    if (acc == NULL) {
        return;
    }

    for (uint32_t i = 0u; i < acc->worst_count; ++i) {
        if (acc->pending_next_valid[i] &&
            acc->worst[i].sample_index + 1u == sample_index) {
            acc->worst[i].next_delta_pc0 = delta_pc0;
            acc->worst[i].next_delta_pc1 = delta_pc1;
            acc->pending_next_valid[i] = false;
        }
    }

    online_count_push(&acc->pc0, delta_pc0);
    online_count_push(&acc->pc1, delta_pc1);

    const int16_t iv_counts = delta_pc0;
    const int16_t iw_counts = delta_pc1;
    const int16_t iu_counts =
        (int16_t)(-((int32_t)delta_pc0 + (int32_t)delta_pc1));
    const int16_t id_counts = iu_counts;
    const int16_t iq_counts = (int16_t)((int32_t)delta_pc0 - (int32_t)delta_pc1);
    uint16_t phase_metric_counts = abs_i16_to_u16(iu_counts);
    const uint16_t abs_iv_counts = abs_i16_to_u16(iv_counts);
    const uint16_t abs_iw_counts = abs_i16_to_u16(iw_counts);
    if (abs_iv_counts > phase_metric_counts) { phase_metric_counts = abs_iv_counts; }
    if (abs_iw_counts > phase_metric_counts) { phase_metric_counts = abs_iw_counts; }

    online_current_push_counts(&acc->iv, iv_counts, current_amp_per_count);
    online_current_push_counts(&acc->iw, iw_counts, current_amp_per_count);
    online_current_push_counts(&acc->iu, iu_counts, current_amp_per_count);
    online_current_push_counts(&acc->id, id_counts, current_amp_per_count);
    online_current_push_counts(&acc->iq, iq_counts,
                               current_amp_per_count * 0.57735026919f);

    acc->phase.count++;
    if (phase_metric_counts > acc->phase.phase_abs_peak_counts) {
        acc->phase.phase_abs_peak_counts = phase_metric_counts;
    }
    uint32_t phase_bin = phase_metric_counts;
    if (phase_bin >= CURRENT_SENSOR_NOISE_PHASE_HIST_BINS) {
        acc->phase.phase_histogram_overflow_count++;
        phase_bin = CURRENT_SENSOR_NOISE_PHASE_HIST_BINS - 1u;
    }
    acc->phase.phase_histogram[phase_bin]++;

    if (acc->threshold_0p20_counts == 0u) {
        acc->threshold_0p08_counts = (uint16_t)(0.08f / current_amp_per_count);
        acc->threshold_0p12_counts = (uint16_t)(0.12f / current_amp_per_count);
        acc->threshold_0p16_counts = (uint16_t)(0.16f / current_amp_per_count);
        acc->threshold_0p20_counts = (uint16_t)(0.20f / current_amp_per_count);
    }

    if (phase_metric_counts > acc->threshold_0p08_counts) {
        acc->phase.samples_above_0p08A++;
        acc->phase.current_consecutive_above_0p08A++;
        if (acc->phase.current_consecutive_above_0p08A >
            acc->phase.longest_consecutive_above_0p08A) {
            acc->phase.longest_consecutive_above_0p08A =
                acc->phase.current_consecutive_above_0p08A;
        }
    } else {
        acc->phase.current_consecutive_above_0p08A = 0u;
    }
    if (phase_metric_counts > acc->threshold_0p12_counts) {
        acc->phase.samples_above_0p12A++;
        acc->phase.current_consecutive_above_0p12A++;
        if (acc->phase.current_consecutive_above_0p12A >
            acc->phase.longest_consecutive_above_0p12A) {
            acc->phase.longest_consecutive_above_0p12A =
                acc->phase.current_consecutive_above_0p12A;
        }
    } else {
        acc->phase.current_consecutive_above_0p12A = 0u;
    }
    if (phase_metric_counts > acc->threshold_0p16_counts) {
        acc->phase.samples_above_0p16A++;
        acc->phase.current_consecutive_above_0p16A++;
        if (acc->phase.current_consecutive_above_0p16A >
            acc->phase.longest_consecutive_above_0p16A) {
            acc->phase.longest_consecutive_above_0p16A =
                acc->phase.current_consecutive_above_0p16A;
        }
    } else {
        acc->phase.current_consecutive_above_0p16A = 0u;
    }
    if (phase_metric_counts > acc->threshold_0p20_counts) {
        acc->phase.samples_above_0p20A++;
        acc->phase.current_consecutive_above_0p20A++;
        if (acc->phase.current_consecutive_above_0p20A >
            acc->phase.longest_consecutive_above_0p20A) {
            acc->phase.longest_consecutive_above_0p20A =
                acc->phase.current_consecutive_above_0p20A;
        }
    } else {
        acc->phase.current_consecutive_above_0p20A = 0u;
    }

    CurrentSensorNoiseWorstSample worst = {0};
    worst.sample_index = sample_index;
    worst.adc_seq = adc_seq;
    worst.raw_pc0 = raw_pc0;
    worst.raw_pc1 = raw_pc1;
    worst.delta_pc0 = delta_pc0;
    worst.delta_pc1 = delta_pc1;
    worst.prev_delta_pc0 = prev_delta_pc0;
    worst.prev_delta_pc1 = prev_delta_pc1;
    worst.next_delta_pc0 = delta_pc0;
    worst.next_delta_pc1 = delta_pc1;
    worst.iv = 0.0f;
    worst.iw = 0.0f;
    worst.iu = 0.0f;
    worst.id = 0.0f;
    worst.iq = 0.0f;
    worst.phase_abs = (float)phase_metric_counts;
    worst.tim1_cnt = tim1_cnt;
    worst.adc_rank_order = adc_rank_order;
    worst.callback_count = callback_count;
    online_worst_insert(acc, &worst);
}

void current_sensor_noise_online_finalize(
    const CurrentSensorNoiseOnlineAccumulator *acc,
    CurrentSensorNoiseAnalysis *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (acc == NULL) {
        return;
    }
    online_count_finalize(&acc->pc0, &out->pc0);
    online_count_finalize(&acc->pc1, &out->pc1);
    online_current_finalize(&acc->iv, &out->iv);
    online_current_finalize(&acc->iw, &out->iw);
    online_current_finalize(&acc->iu, &out->iu);
    online_current_finalize(&acc->id, &out->id);
    online_current_finalize(&acc->iq, &out->iq);

    const float phase_scale = acc->iv.scale;
    out->phase.sample_count = acc->phase.count;
    out->phase.phase_abs_peak =
        (float)acc->phase.phase_abs_peak_counts * phase_scale;
    out->phase.phase_percentile_99 =
        phase_hist_percentile_counts(&acc->phase, 99.0f) * phase_scale;
    out->phase.phase_percentile_99_5 =
        phase_hist_percentile_counts(&acc->phase, 99.5f) * phase_scale;
    out->phase.phase_percentile_99_9 =
        phase_hist_percentile_counts(&acc->phase, 99.9f) * phase_scale;
    out->phase.samples_above_0p08A = acc->phase.samples_above_0p08A;
    out->phase.samples_above_0p12A = acc->phase.samples_above_0p12A;
    out->phase.samples_above_0p16A = acc->phase.samples_above_0p16A;
    out->phase.samples_above_0p20A = acc->phase.samples_above_0p20A;
    out->phase.longest_consecutive_above_0p08A =
        acc->phase.longest_consecutive_above_0p08A;
    out->phase.longest_consecutive_above_0p12A =
        acc->phase.longest_consecutive_above_0p12A;
    out->phase.longest_consecutive_above_0p16A =
        acc->phase.longest_consecutive_above_0p16A;
    out->phase.longest_consecutive_above_0p20A =
        acc->phase.longest_consecutive_above_0p20A;

    out->worst_count = acc->worst_count;
    for (uint32_t i = 0u; i < acc->worst_count; ++i) {
        out->worst[i] = acc->worst[i];
        CurrentSensorNoiseWorstSample *w = &out->worst[i];
        w->iv = (float)w->delta_pc0 * phase_scale;
        w->iw = (float)w->delta_pc1 * phase_scale;
        w->iu = -((float)w->delta_pc0 + (float)w->delta_pc1) * phase_scale;
        w->id = w->iu;
        w->iq = ((float)w->delta_pc0 - (float)w->delta_pc1) *
                phase_scale * 0.57735026919f;
        w->phase_abs *= phase_scale;
    }
    for (uint32_t i = 1u; i < out->worst_count; ++i) {
        CurrentSensorNoiseWorstSample key = out->worst[i];
        uint32_t j = i;
        while (j > 0u && key.phase_abs > out->worst[j - 1u].phase_abs) {
            out->worst[j] = out->worst[j - 1u];
            --j;
        }
        out->worst[j] = key;
    }
}

bool current_sensor_noise_should_skip_power_for_low_gain(float gain_v_v)
{
    return gain_v_v <= 20.0f;
}

CurrentSensorNoiseNfaultSemantics
current_sensor_noise_nfault_from_raw(bool raw_pin_high)
{
    CurrentSensorNoiseNfaultSemantics out;
    out.raw_pin_high = raw_pin_high;
    out.asserted = !raw_pin_high;
    out.ok = raw_pin_high;
    return out;
}

#ifndef CURRENT_SAMPLE_PIPELINE_H
#define CURRENT_SAMPLE_PIPELINE_H

#include <stdbool.h>
#include <stdint.h>

#include "hal/hal_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float command_v_alpha;
    float command_v_beta;
    float applied_v_alpha;
    float applied_v_beta;
    uint32_t command_seq;
} CurrentSampleAppliedVoltage;

typedef struct {
    CurrentSampleAppliedVoltage active;
    CurrentSampleAppliedVoltage pending;
    bool pending_valid;
} CurrentSamplePipeline;

void current_sample_pipeline_init(CurrentSamplePipeline *pipe,
                                  float v_alpha,
                                  float v_beta,
                                  float applied_alpha,
                                  float applied_beta);
CurrentSampleAppliedVoltage current_sample_pipeline_bind_sample(
    const CurrentSamplePipeline *pipe);
void current_sample_pipeline_write_next(CurrentSamplePipeline *pipe,
                                        float v_alpha,
                                        float v_beta,
                                        float applied_alpha,
                                        float applied_beta);
void current_sample_pipeline_advance_pwm_cycle(CurrentSamplePipeline *pipe);
void current_sample_pipeline_demux_adc2(HalAdcM0RankOrder order,
                                        uint16_t rank1,
                                        uint16_t rank2,
                                        uint16_t rank3,
                                        uint16_t rank4,
                                        HalAdcSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_SAMPLE_PIPELINE_H */

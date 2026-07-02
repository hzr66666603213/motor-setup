#include "current_sample_pipeline.h"

#include <string.h>

void current_sample_pipeline_init(CurrentSamplePipeline *pipe,
                                  float v_alpha,
                                  float v_beta,
                                  float applied_alpha,
                                  float applied_beta)
{
    if (pipe == 0) {
        return;
    }

    memset(pipe, 0, sizeof(*pipe));
    pipe->active.command_v_alpha = v_alpha;
    pipe->active.command_v_beta = v_beta;
    pipe->active.applied_v_alpha = applied_alpha;
    pipe->active.applied_v_beta = applied_beta;
    pipe->active.command_seq = 1u;
}

CurrentSampleAppliedVoltage current_sample_pipeline_bind_sample(
    const CurrentSamplePipeline *pipe)
{
    CurrentSampleAppliedVoltage bound;
    memset(&bound, 0, sizeof(bound));
    if (pipe != 0) {
        bound = pipe->active;
    }
    return bound;
}

void current_sample_pipeline_write_next(CurrentSamplePipeline *pipe,
                                        float v_alpha,
                                        float v_beta,
                                        float applied_alpha,
                                        float applied_beta)
{
    if (pipe == 0) {
        return;
    }

    pipe->pending.command_v_alpha = v_alpha;
    pipe->pending.command_v_beta = v_beta;
    pipe->pending.applied_v_alpha = applied_alpha;
    pipe->pending.applied_v_beta = applied_beta;
    pipe->pending.command_seq = pipe->active.command_seq + 1u;
    pipe->pending_valid = true;
}

void current_sample_pipeline_advance_pwm_cycle(CurrentSamplePipeline *pipe)
{
    if (pipe == 0 || !pipe->pending_valid) {
        return;
    }

    pipe->active = pipe->pending;
    pipe->pending_valid = false;
}

void current_sample_pipeline_demux_adc2(HalAdcM0RankOrder order,
                                        uint16_t rank1,
                                        uint16_t rank2,
                                        uint16_t rank3,
                                        uint16_t rank4,
                                        HalAdcSnapshot *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    if (order == HAL_ADC_M0_ORDER_PC1_PC0) {
        snapshot->raw_pc0_m0_so1 = rank2;
        snapshot->raw_pc1_m0_so2 = rank1;
    } else {
        snapshot->raw_pc0_m0_so1 = rank1;
        snapshot->raw_pc1_m0_so2 = rank2;
    }
    snapshot->raw_pc2_m1_so2 = rank3;
    snapshot->raw_pc3_m1_so1 = rank4;
    snapshot->raw_u = snapshot->raw_pc0_m0_so1;
    snapshot->raw_v = snapshot->raw_pc1_m0_so2;
    snapshot->raw_w = 0u;
}

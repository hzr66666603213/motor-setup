#include "app/calibration.h"

#include "board/board_odrive_v36.h"
#include "foc/foc_math.h"
#include "protection/fault.h"

/*
 * calibration.c
 *
 * 非阻塞校准 skeleton。
 * 真实工程中，电阻/电感/编码器校准需要专用开环电压矢量或小电流注入函数。
 * 当前保留状态、超时、限流和结果写入位置，便于学习和二次开发。
 *
 * 为什么必须非阻塞：
 * - 校准期间仍要周期检查 nFAULT、母线电压和过流。
 * - 如果用 delay 卡住主循环，故障响应会变慢。
 * - 状态机每次 update 只推进一点点，适合 1kHz 调度。
 *
 * 对 2804 第一次调试的建议：
 * - 12V 或更低电压；
 * - 电流限制 0.5A~1A；
 * - 电机空载，转子可自由转动；
 * - 发现抖动、尖叫、过流立即断电。
 */

#define CALIB_OFFSET_SAMPLES          2000u
#define CALIB_STEP_TIMEOUT_S          3.0f
#define CALIB_ENCODER_MIN_DELTA_COUNT 16
#define CALIB_RESISTANCE_START_V      0.05f
#define CALIB_INDUCTANCE_START_V      0.05f
#define CALIB_INJECTION_MAX_V         0.10f

static void calib_fail(Axis0Context *axis, Axis0CalibrationContext *calib, Axis0CalibrationError error)
{
    /*
     * 校准失败必须立刻关闭功率级。
     * 不在这里清 fault_flags，因为调用者会根据当前步骤设置更具体的故障码。
     */
    calib->step = CALIB_FAILED;
    calib->error = error;
    board_disable_axis0_power_stage(axis);
}

void axis0_calibration_start(Axis0CalibrationContext *calib, Axis0CalibrationStep first_step)
{
    /*
     * 每次进入校准状态都重置上下文。
     * 如果用户中途退出再重新请求，旧的累计值不能继续沿用。
     */
    calib->step = first_step;
    calib->error = CALIB_ERROR_NONE;
    calib->step_elapsed_s = 0.0f;
    calib->sample_count = 0u;
    calib->accum_a = 0.0f;
    calib->accum_b = 0.0f;
    calib->accum_c = 0.0f;
    calib->max_offset_span_count = 20.0f;
    calib->resistance_test_voltage_v = CALIB_RESISTANCE_START_V;
    calib->inductance_pulse_voltage_v = CALIB_INDUCTANCE_START_V;
    calib->start_encoder_count = 0;
}

static void calib_next(Axis0CalibrationContext *calib, Axis0CalibrationStep next)
{
    /*
     * 切换校准步骤时清掉当前步骤累计值和计时。
     * 这样每一步的超时和采样统计彼此独立。
     */
    calib->step = next;
    calib->step_elapsed_s = 0.0f;
    calib->sample_count = 0u;
    calib->accum_a = 0.0f;
    calib->accum_b = 0.0f;
    calib->accum_c = 0.0f;
}

void axis0_calibration_update(Axis0Context *axis,
                              Axis0CalibrationContext *calib,
                              CurrentSensorConfig *current_sensor,
                              EncoderMt6701AbzState *encoder,
                              float dt_s)
{
    calib->step_elapsed_s += dt_s;
    if (calib->step_elapsed_s > CALIB_STEP_TIMEOUT_S) {
        /* 任一步骤超时都视为校准失败，避免电机长时间受激。 */
        calib_fail(axis, calib, CALIB_ERROR_TIMEOUT);
        return;
    }

    if (calib->step == CALIB_CURRENT_OFFSET) {
        uint16_t raw_a = 0u, raw_b = 0u, raw_c = 0u;
        /*
         * A. 电流采样零偏校准
         * 目的：得到“0A 时 ADC 的 count 值”。
         * 条件：PWM 关闭，DRV8301 关闭或处于安全状态，确保相电流真实接近 0A。
         */
        board_disable_axis0_power_stage(axis);
        if (board_axis0_read_phase_current_raw(&raw_a, &raw_b, &raw_c)) {
            calib->accum_a += (float)raw_a;
            calib->accum_b += (float)raw_b;
            calib->accum_c += (float)raw_c;
            calib->sample_count++;
        }
        if (calib->sample_count >= CALIB_OFFSET_SAMPLES) {
            /*
             * 这里先做简单平均。
             * 后续可加入 max/min span 检查：如果零偏波动过大，说明 ADC 噪声、
             * 运放饱和或电源异常，应返回 CALIB_ERROR_OFFSET_NOISE。
             */
            current_sensor->offset_a_count = calib->accum_a / (float)calib->sample_count;
            current_sensor->offset_b_count = calib->accum_b / (float)calib->sample_count;
            current_sensor->offset_c_count = calib->accum_c / (float)calib->sample_count;
            axis->current_offset_valid = true;
            calib_next(calib, CALIB_RESISTANCE);
        }
    } else if (calib->step == CALIB_RESISTANCE) {
        /*
         * B. 相电阻测量
         * 低电压/低电流注入，等待稳态电流后估算 R=V/I。
         * 起步电压必须非常低：0.05V，最多逐步到 0.1V。
         * 当前 skeleton 不直接输出电压，避免未接硬件时误操作。
         *
         * 真实实现建议：
         * - 施加 0.05V 起步的小 d 轴电压或相间电压；
         * - 等待电流到稳态；
         * - 用已知电压 / 稳态电流估算相电阻；
         * - 全程检查 calibration_current_a。
         */
        if (axis->rt.ia_a > axis->config.motor.calibration_current_a ||
            axis->rt.ia_a < -axis->config.motor.calibration_current_a) {
            calib_fail(axis, calib, CALIB_ERROR_OVERCURRENT);
            return;
        }
        if (calib->step_elapsed_s > 0.5f) {
            /*
             * 当前保留为 0，表示还没有真实测量结果。
             * 状态机仍保留步骤位置，后续接入注入函数时不需要改外部接口。
             */
            axis->config.motor.phase_resistance_ohm = 0.0f;
            calib_next(calib, CALIB_INDUCTANCE);
        }
    } else if (calib->step == CALIB_INDUCTANCE) {
        /*
         * C. 相电感测量
         * 施加短脉冲并测量 di/dt，估算 L=V/(di/dt)。
         * 起步脉冲电压也按 0.05V，最多 0.1V，不从 0.5V/1V 开始。
         * 当前只保留步骤位置，真实实现必须加过流保护。
         *
         * 注意：电感测量脉冲很短，但仍可能在低电阻小电机上造成电流快速上升。
         * 第一次调试必须把脉冲电压和脉冲时间设得非常保守。
         */
        if (calib->step_elapsed_s > 0.2f) {
            axis->config.motor.phase_inductance_h = 0.0f;
            axis->motor_calibrated = true;
            calib_next(calib, CALIB_ENCODER_DIRECTION);
        }
    } else if (calib->step == CALIB_ENCODER_DIRECTION) {
        if (calib->sample_count == 0u) {
            /* 记录开始时的 ABZ 计数，之后比较开环正转后的变化方向。 */
            calib->start_encoder_count = encoder->raw_count;
            calib->sample_count = 1u;
        }
        /*
         * D. 编码器方向判断
         * 开环电角度缓慢正向旋转后检查 ABZ count 变化。
         * 当前 skeleton 不主动旋转，只评估计数变化位置。
         *
         * 真实实现需要在 1kHz update 中逐步推进开环电角度，而不是 delay。
         * 如果 delta 太小，说明电机没动、编码器没接好或转子被卡住。
         */
        if (calib->step_elapsed_s > 1.0f) {
            int32_t delta = encoder->raw_count - calib->start_encoder_count;
            if (delta > CALIB_ENCODER_MIN_DELTA_COUNT) {
                axis->config.encoder.encoder_direction = 1;
                encoder->direction = 1;
            } else if (delta < -CALIB_ENCODER_MIN_DELTA_COUNT) {
                axis->config.encoder.encoder_direction = -1;
                encoder->direction = -1;
            } else {
                calib_fail(axis, calib, CALIB_ERROR_ENCODER_NO_MOVEMENT);
                return;
            }
            calib_next(calib, CALIB_ENCODER_OFFSET);
        }
    } else if (calib->step == CALIB_ENCODER_OFFSET) {
        /*
         * E. 电角度零位校准
         * 用小 d 轴电流/固定电压矢量锁定转子到已知电角度。
         * 读机械角后计算 encoder_offset_rad。
         *
         * 这个 offset 是闭环成败的关键：
         * - offset 错误会导致 d/q 轴错位；
         * - 轻则抖动和效率低；
         * - 重则电流环发散、过流。
         */
        if (calib->step_elapsed_s > 0.5f) {
            /*
             * 假设锁定目标电角度为 0 rad，则需要 offset 抵消当前机械角*极对数*方向。
             * 如果实际锁定目标不是 0，应把目标电角度加到该公式中。
             */
            axis->config.encoder.encoder_offset_rad =
                foc_wrap_0_2pi(-encoder->mechanical_angle_rad *
                               (float)axis->config.motor.pole_pairs *
                               (float)axis->config.encoder.encoder_direction);
            encoder->offset_rad = axis->config.encoder.encoder_offset_rad;
            axis->encoder_calibrated = true;
            calib_next(calib, CALIB_DONE);
        }
    } else if (calib->step == CALIB_DONE) {
        /* 校准完成后仍保持功率级关闭，由用户显式请求 closed_loop。 */
        board_disable_axis0_power_stage(axis);
    } else {
        calib_fail(axis, calib, CALIB_ERROR_INVALID_RESULT);
    }
}

bool axis0_calibration_finished(const Axis0CalibrationContext *calib)
{
    return calib->step == CALIB_DONE || calib->step == CALIB_FAILED;
}

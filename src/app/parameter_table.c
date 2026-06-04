#include "app/parameter_table.h"

#include <stdlib.h>
#include <string.h>
#include "foc/foc_math.h"

/*
 * parameter_table.c
 *
 * 简单字符串参数表。
 * 所有 set 都做范围检查，默认禁止大电流和大电压。
 *
 * 为什么不用直接暴露结构体：
 * - 需要集中做范围检查；
 * - 需要区分只读状态量和可写配置；
 * - 后续保存 Flash 时可以统一管理参数名和版本；
 * - 通信层不应该知道 Axis0Context 内部字段布局。
 */

static bool in_range(float value, float min_value, float max_value)
{
    /* 范围检查使用闭区间，所有危险 set 都必须经过这里或等价检查。 */
    return value >= min_value && value <= max_value;
}

bool axis0_param_get(Axis0ParameterContext *ctx, const char *name, float *value)
{
    /*
     * 只读观测量：
     * vbus/ia/ib/ic/id/iq/angle/velocity/fault/state。
     * 这些值主要由 ISR 或状态机更新，后台读取时可能不是严格同步快照。
     * 调试显示可以接受；闭环控制不要通过 console 读回再参与实时计算。
     */
    if (strcmp(name, "vbus") == 0) { *value = ctx->axis->rt.vbus_v; return true; }
    if (strcmp(name, "ia") == 0) { *value = ctx->axis->rt.ia_a; return true; }
    if (strcmp(name, "ib") == 0) { *value = ctx->axis->rt.ib_a; return true; }
    if (strcmp(name, "ic") == 0) { *value = ctx->axis->rt.ic_a; return true; }
    if (strcmp(name, "id") == 0) { *value = ctx->axis->rt.id_a; return true; }
    if (strcmp(name, "iq") == 0) { *value = ctx->axis->rt.iq_a; return true; }
    if (strcmp(name, "angle") == 0) { *value = ctx->axis->rt.mechanical_angle_rad; return true; }
    if (strcmp(name, "velocity") == 0) { *value = ctx->axis->rt.velocity_rad_s; return true; }
    if (strcmp(name, "fault") == 0) { *value = (float)ctx->axis->fault_flags; return true; }
    if (strcmp(name, "state") == 0) { *value = (float)ctx->axis->state; return true; }
    return false;
}

bool axis0_param_set(Axis0ParameterContext *ctx, const char *name, const char *value_text)
{
    /*
     * strtof 用于后台命令解析，不进入 ISR。
     * 如果后续做 CAN 二进制协议，可以绕过字符串解析但仍复用范围检查逻辑。
     */
    float value = strtof(value_text, 0);
    if (strcmp(name, "current_limit") == 0) {
        /*
         * 第一阶段最多允许 3A，默认 1A。
         * 真实 2804 和 ODrive 能力可能更高，但学习调试必须保守。
         */
        if (!in_range(value, 0.0f, 3.0f)) { return false; }
        ctx->axis->config.motor.current_limit_a = value;
        return true;
    }
    if (strcmp(name, "voltage_limit") == 0) {
        /*
         * 第一阶段最多允许 6V 电压矢量，默认 3V。
         * 在 12V 测试电源下，这能显著降低误接线/相序错误时的风险。
         */
        if (!in_range(value, 0.0f, 6.0f)) { return false; }
        ctx->axis->config.motor.voltage_limit_v = value;
        return true;
    }
    if (strcmp(name, "pole_pairs") == 0) {
        /*
         * pole_pairs 影响电角度计算。
         * 2804 默认 7 只是常见值，必须通过实测确认。
         */
        if (!in_range(value, 1.0f, 32.0f)) { return false; }
        ctx->axis->config.motor.pole_pairs = (uint8_t)value;
        return true;
    }
    if (strcmp(name, "encoder_cpr") == 0) {
        /*
         * MT6701 ABZ 默认 4096 CPR。
         * 修改 CPR 会改变角度比例，闭环中禁止修改。
         */
        if (!in_range(value, 64.0f, 262144.0f)) { return false; }
        ctx->axis->config.encoder.encoder_cpr = (int32_t)value;
        encoder_mt6701_abz_set_cpr(ctx->encoder, (int32_t)value);
        return true;
    }
    if (strcmp(name, "control_mode") == 0) {
        /* 控制模式支持 torque/velocity/position/idle 文本。 */
        Axis0ControlMode mode;
        if (!axis0_parse_control_mode(value_text, &mode)) { return false; }
        ctx->axis->cmd.control_mode = mode;
        return true;
    }
    if (strcmp(name, "input_torque") == 0) {
        /*
         * torque 输入也做严格限制。
         * 如果 torque_constant 不准确，实际 iq 会偏差，因此第一版范围很小。
         */
        if (!in_range(value, -0.2f, 0.2f)) { return false; }
        ctx->axis->cmd.input_torque_nm = value;
        return true;
    }
    if (strcmp(name, "input_velocity") == 0) {
        /* 速度输入受 motor.velocity_limit_rad_s 限制。 */
        if (!in_range(value, -ctx->axis->config.motor.velocity_limit_rad_s,
                            ctx->axis->config.motor.velocity_limit_rad_s)) { return false; }
        ctx->axis->cmd.input_velocity_rad_s = value;
        return true;
    }
    if (strcmp(name, "input_position") == 0) {
        /* 第一版位置输入归一化到 [-pi, pi)，避免巨大位置阶跃。 */
        ctx->axis->cmd.input_position_rad = foc_wrap_minuspi_pi(value);
        return true;
    }
    return false;
}

const char *axis0_state_to_string(Axis0StateId state)
{
    /* 枚举转文本，方便 console 输出和日志阅读。 */
    switch (state) {
    case AXIS0_STATE_BOOT: return "boot";
    case AXIS0_STATE_IDLE: return "idle";
    case AXIS0_STATE_CURRENT_OFFSET_CALIBRATION: return "current_offset_calibration";
    case AXIS0_STATE_MOTOR_CALIBRATION: return "motor_calibration";
    case AXIS0_STATE_ENCODER_CALIBRATION: return "encoder_calibration";
    case AXIS0_STATE_READY: return "ready";
    case AXIS0_STATE_CLOSED_LOOP_CONTROL: return "closed_loop";
    case AXIS0_STATE_FAULT: return "fault";
    default: return "unknown";
    }
}

bool axis0_parse_state(const char *text, Axis0StateId *state)
{
    /*
     * 只允许用户请求安全定义过的状态。
     * BOOT/FAULT 不通过 request_state 进入。
     */
    if (strcmp(text, "idle") == 0) { *state = AXIS0_STATE_IDLE; return true; }
    if (strcmp(text, "current_offset_calibration") == 0) { *state = AXIS0_STATE_CURRENT_OFFSET_CALIBRATION; return true; }
    if (strcmp(text, "motor_calibration") == 0) { *state = AXIS0_STATE_MOTOR_CALIBRATION; return true; }
    if (strcmp(text, "encoder_calibration") == 0) { *state = AXIS0_STATE_ENCODER_CALIBRATION; return true; }
    if (strcmp(text, "closed_loop") == 0) { *state = AXIS0_STATE_CLOSED_LOOP_CONTROL; return true; }
    return false;
}

bool axis0_parse_control_mode(const char *text, Axis0ControlMode *mode)
{
    /* 控制模式解析，未知字符串返回 false。 */
    if (strcmp(text, "torque") == 0) { *mode = AXIS0_CONTROL_MODE_TORQUE; return true; }
    if (strcmp(text, "velocity") == 0) { *mode = AXIS0_CONTROL_MODE_VELOCITY; return true; }
    if (strcmp(text, "position") == 0) { *mode = AXIS0_CONTROL_MODE_POSITION; return true; }
    if (strcmp(text, "idle") == 0) { *mode = AXIS0_CONTROL_MODE_IDLE; return true; }
    return false;
}

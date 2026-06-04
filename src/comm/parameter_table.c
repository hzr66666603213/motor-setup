#include "comm/parameter_table.h"

#include <string.h>
#include "foc/foc_math.h"

/*
 * parameter_table.c
 *
 * 参数表实现。
 * 通信层只关心参数名/ID，具体字段映射集中在这里，避免协议直接访问内部结构。
 */

static const ParamDescriptor s_params[] = {
    /* 电机参数：可写，通常在校准或用户配置后保存到 Flash。 */
    { PARAM_MOTOR_POLE_PAIRS, "motor.pole_pairs", PARAM_TYPE_UINT32, true, 1.0f, 64.0f },
    { PARAM_MOTOR_PHASE_RESISTANCE, "motor.phase_resistance", PARAM_TYPE_FLOAT, true, 0.001f, 100.0f },
    { PARAM_MOTOR_PHASE_INDUCTANCE, "motor.phase_inductance", PARAM_TYPE_FLOAT, true, 0.000001f, 1.0f },
    { PARAM_MOTOR_TORQUE_CONSTANT, "motor.torque_constant", PARAM_TYPE_FLOAT, true, 0.0001f, 10.0f },
    { PARAM_MOTOR_CURRENT_LIMIT, "motor.current_limit", PARAM_TYPE_FLOAT, true, 0.0f, 200.0f },
    { PARAM_MOTOR_VELOCITY_LIMIT, "motor.velocity_limit", PARAM_TYPE_FLOAT, true, 0.0f, 10000.0f },
    /* 控制模式和输入指令：可写，由 UART/CAN 或上层控制器更新。 */
    { PARAM_AXIS_CONTROL_MODE, "axis.control_mode", PARAM_TYPE_INT32, true, 0.0f, 3.0f },
    { PARAM_INPUT_TORQUE, "input.torque", PARAM_TYPE_FLOAT, true, -1000.0f, 1000.0f },
    { PARAM_INPUT_VELOCITY, "input.velocity", PARAM_TYPE_FLOAT, true, -10000.0f, 10000.0f },
    { PARAM_INPUT_POSITION, "input.position", PARAM_TYPE_FLOAT, true, -100000.0f, 100000.0f },
    /* 控制器增益：可写，调试阶段通过通信在线整定。 */
    { PARAM_CURRENT_KP, "current.kp", PARAM_TYPE_FLOAT, true, 0.0f, 10000.0f },
    { PARAM_CURRENT_KI, "current.ki", PARAM_TYPE_FLOAT, true, 0.0f, 1000000.0f },
    { PARAM_VELOCITY_KP, "velocity.kp", PARAM_TYPE_FLOAT, true, 0.0f, 10000.0f },
    { PARAM_VELOCITY_KI, "velocity.ki", PARAM_TYPE_FLOAT, true, 0.0f, 100000.0f },
    { PARAM_POSITION_KP, "position.kp", PARAM_TYPE_FLOAT, true, 0.0f, 10000.0f },
    /* 编码器校准结果和状态请求。 */
    { PARAM_ENCODER_OFFSET, "encoder.offset", PARAM_TYPE_FLOAT, true, -FOC_TWO_PI_F, FOC_TWO_PI_F },
    { PARAM_ENCODER_DIRECTION, "encoder.direction", PARAM_TYPE_INT32, true, -1.0f, 1.0f },
    { PARAM_AXIS_REQUESTED_STATE, "axis.requested_state", PARAM_TYPE_INT32, true, 0.0f, 5.0f },
    /* 只读状态量：用于调试、监控和上位机显示。 */
    { PARAM_AXIS_ERROR, "axis.error", PARAM_TYPE_UINT32, false, 0.0f, 4294967295.0f },
    { PARAM_BUS_VOLTAGE, "bus_voltage", PARAM_TYPE_FLOAT, false, 0.0f, 1000.0f },
    { PARAM_IQ_MEASURED, "iq_measured", PARAM_TYPE_FLOAT, false, -1000.0f, 1000.0f },
    { PARAM_ID_MEASURED, "id_measured", PARAM_TYPE_FLOAT, false, -1000.0f, 1000.0f },
    { PARAM_VELOCITY_MEASURED, "velocity_measured", PARAM_TYPE_FLOAT, false, -100000.0f, 100000.0f },
    { PARAM_POSITION_MEASURED, "position_measured", PARAM_TYPE_FLOAT, false, -100000.0f, 100000.0f },
};

const ParamDescriptor *parameter_find_by_name(const char *name)
{
    /* 线性查找足够用于小参数表；后续可换成哈希或排序表。 */
    for (size_t i = 0u; i < parameter_count(); ++i) {
        if (strcmp(s_params[i].name, name) == 0) {
            return &s_params[i];
        }
    }
    return 0;
}

const ParamDescriptor *parameter_find_by_id(ParamId id)
{
    for (size_t i = 0u; i < parameter_count(); ++i) {
        if (s_params[i].id == id) {
            return &s_params[i];
        }
    }
    return 0;
}

bool parameter_read(const ParameterContext *ctx, ParamId id, float *value)
{
    /* 统一读接口：所有类型先转换成 float 返回，便于简化文本协议。 */
    switch (id) {
    case PARAM_MOTOR_POLE_PAIRS: *value = (float)ctx->axis->motor.pole_pairs; return true;
    case PARAM_MOTOR_PHASE_RESISTANCE: *value = ctx->axis->motor.phase_resistance_ohm; return true;
    case PARAM_MOTOR_PHASE_INDUCTANCE: *value = ctx->axis->motor.phase_inductance_h; return true;
    case PARAM_MOTOR_TORQUE_CONSTANT: *value = ctx->axis->motor.torque_constant_nm_per_a; return true;
    case PARAM_MOTOR_CURRENT_LIMIT: *value = ctx->axis->motor.current_limit_a; return true;
    case PARAM_MOTOR_VELOCITY_LIMIT: *value = ctx->axis->motor.velocity_limit_rad_s; return true;
    case PARAM_AXIS_CONTROL_MODE: *value = (float)ctx->axis->axis.control_mode; return true;
    case PARAM_INPUT_TORQUE: *value = ctx->axis->input.torque_nm; return true;
    case PARAM_INPUT_VELOCITY: *value = ctx->axis->input.velocity_rad_s; return true;
    case PARAM_INPUT_POSITION: *value = ctx->axis->input.position_rad; return true;
    case PARAM_CURRENT_KP: *value = ctx->current->kp; return true;
    case PARAM_CURRENT_KI: *value = ctx->current->ki; return true;
    case PARAM_VELOCITY_KP: *value = ctx->velocity->kp; return true;
    case PARAM_VELOCITY_KI: *value = ctx->velocity->ki; return true;
    case PARAM_POSITION_KP: *value = ctx->position->kp; return true;
    case PARAM_ENCODER_OFFSET: *value = ctx->axis->encoder.offset_rad; return true;
    case PARAM_ENCODER_DIRECTION: *value = (float)ctx->axis->encoder.direction; return true;
    case PARAM_AXIS_REQUESTED_STATE: *value = (float)ctx->axis->axis.requested_state; return true;
    case PARAM_AXIS_ERROR: *value = (float)ctx->axis->axis.error; return true;
    case PARAM_BUS_VOLTAGE: *value = ctx->axis->motor_state.bus_voltage_v; return true;
    case PARAM_IQ_MEASURED: *value = ctx->axis->foc_state.iq_a; return true;
    case PARAM_ID_MEASURED: *value = ctx->axis->foc_state.id_a; return true;
    case PARAM_VELOCITY_MEASURED: *value = ctx->axis->motor_state.mechanical_velocity_rad_s; return true;
    case PARAM_POSITION_MEASURED: *value = ctx->axis->motor_state.mechanical_angle_rad; return true;
    default: return false;
    }
}

bool parameter_write(ParameterContext *ctx, ParamId id, float value)
{
    /* 写入前先查描述符，统一检查是否可写和范围是否合法。 */
    const ParamDescriptor *desc = parameter_find_by_id(id);
    if (!desc || !desc->writable || value < desc->min_value || value > desc->max_value) {
        return false;
    }

    switch (id) {
    /* 写入 motor.current_limit 时，同时同步速度环输出限幅。 */
    case PARAM_MOTOR_POLE_PAIRS: ctx->axis->motor.pole_pairs = (uint8_t)value; return true;
    case PARAM_MOTOR_PHASE_RESISTANCE: ctx->axis->motor.phase_resistance_ohm = value; return true;
    case PARAM_MOTOR_PHASE_INDUCTANCE: ctx->axis->motor.phase_inductance_h = value; return true;
    case PARAM_MOTOR_TORQUE_CONSTANT: ctx->axis->motor.torque_constant_nm_per_a = value; return true;
    case PARAM_MOTOR_CURRENT_LIMIT: ctx->axis->motor.current_limit_a = value; ctx->velocity->current_limit_a = value; return true;
    case PARAM_MOTOR_VELOCITY_LIMIT: ctx->axis->motor.velocity_limit_rad_s = value; ctx->velocity->velocity_limit_rad_s = value; ctx->position->velocity_limit_rad_s = value; return true;
    case PARAM_AXIS_CONTROL_MODE: ctx->axis->axis.control_mode = (ControlMode)(int)value; return true;
    case PARAM_INPUT_TORQUE: ctx->axis->input.torque_nm = value; return true;
    case PARAM_INPUT_VELOCITY: ctx->axis->input.velocity_rad_s = value; return true;
    case PARAM_INPUT_POSITION: ctx->axis->input.position_rad = value; return true;
    case PARAM_CURRENT_KP: ctx->current->kp = value; return true;
    case PARAM_CURRENT_KI: ctx->current->ki = value; return true;
    case PARAM_VELOCITY_KP: ctx->velocity->kp = value; return true;
    case PARAM_VELOCITY_KI: ctx->velocity->ki = value; return true;
    case PARAM_POSITION_KP: ctx->position->kp = value; return true;
    case PARAM_ENCODER_OFFSET: ctx->axis->encoder.offset_rad = value; return true;
    case PARAM_ENCODER_DIRECTION: ctx->axis->encoder.direction = (EncoderDirection)(int)value; return true;
    case PARAM_AXIS_REQUESTED_STATE: ctx->axis->axis.requested_state = (AxisStateId)(int)value; return true;
    default: return false;
    }
}

size_t parameter_count(void)
{
    return sizeof(s_params) / sizeof(s_params[0]);
}

const ParamDescriptor *parameter_at(size_t index)
{
    return (index < parameter_count()) ? &s_params[index] : 0;
}

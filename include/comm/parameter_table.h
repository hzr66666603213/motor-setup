#ifndef PARAMETER_TABLE_H
#define PARAMETER_TABLE_H

/*
 * parameter_table.h
 *
 * 简化参数表。
 * 目标是让 UART/CAN 通过统一参数 ID 读写 motor、axis、controller 和状态量。
 * 第一版不做 Flash 保存，只做运行期参数访问和范围检查。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "control/current_controller.h"
#include "control/position_controller.h"
#include "control/velocity_controller.h"
#include "core/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PARAM_TYPE_FLOAT = 0, /* float 参数 */
    PARAM_TYPE_INT32,     /* int32 参数 */
    PARAM_TYPE_UINT32,    /* uint32 参数 */
    PARAM_TYPE_BOOL       /* bool 参数 */
} ParamType;

typedef enum {
    PARAM_MOTOR_POLE_PAIRS = 1,
    PARAM_MOTOR_PHASE_RESISTANCE,
    PARAM_MOTOR_PHASE_INDUCTANCE,
    PARAM_MOTOR_TORQUE_CONSTANT,
    PARAM_MOTOR_CURRENT_LIMIT,
    PARAM_MOTOR_VELOCITY_LIMIT,
    PARAM_AXIS_CONTROL_MODE,
    PARAM_INPUT_TORQUE,
    PARAM_INPUT_VELOCITY,
    PARAM_INPUT_POSITION,
    PARAM_CURRENT_KP,
    PARAM_CURRENT_KI,
    PARAM_VELOCITY_KP,
    PARAM_VELOCITY_KI,
    PARAM_POSITION_KP,
    PARAM_ENCODER_OFFSET,
    PARAM_ENCODER_DIRECTION,
    PARAM_AXIS_REQUESTED_STATE,
    PARAM_AXIS_ERROR,
    PARAM_BUS_VOLTAGE,
    PARAM_IQ_MEASURED,
    PARAM_ID_MEASURED,
    PARAM_VELOCITY_MEASURED,
    PARAM_POSITION_MEASURED
} ParamId;

typedef struct {
    ParamId id;           /* 参数 ID，用于二进制协议或内部查找 */
    const char *name;     /* 文本参数名，例如 motor.current_limit */
    ParamType type;       /* 参数类型 */
    bool writable;        /* 是否允许通信写入 */
    float min_value;      /* 写入下限，统一用 float 做范围检查 */
    float max_value;      /* 写入上限，统一用 float 做范围检查 */
} ParamDescriptor;

typedef struct {
    Axis *axis;                     /* 单轴对象 */
    CurrentController *current;     /* 电流控制器对象 */
    VelocityController *velocity;   /* 速度控制器对象 */
    PositionController *position;   /* 位置控制器对象 */
} ParameterContext;

/* 按文本名查找参数描述符。 */
const ParamDescriptor *parameter_find_by_name(const char *name);
/* 按 ID 查找参数描述符。 */
const ParamDescriptor *parameter_find_by_id(ParamId id);
/* 读取参数值，统一返回 float 表示。 */
bool parameter_read(const ParameterContext *ctx, ParamId id, float *value);
/* 写入参数值，自动检查 writable 和范围。 */
bool parameter_write(ParameterContext *ctx, ParamId id, float value);
/* 参数表数量。 */
size_t parameter_count(void);
/* 按索引枚举参数表。 */
const ParamDescriptor *parameter_at(size_t index);

#ifdef __cplusplus
}
#endif

#endif /* PARAMETER_TABLE_H */
